#include "frenet_lattice_planner/end_condition_sampler.hpp"
#include <obstacle_manager/st_graph.hpp>
#include "obstacle_manager/obstacle.hpp"
#include "planning_config.hpp"
#include <utility>

namespace planning {
using namespace common;
using State = std::array<double, 3>;
using EndCondition = std::pair<std::array<double, 3>, double>;
EndConditionSampler::EndConditionSampler(const std::array<double, 3> &init_s,
                                         const std::array<double, 3> &init_d,
                                         const ReferenceLine &ref_line,
                                         const std::vector<std::shared_ptr<Obstacle>> &ptr_obstacles,
                                         std::shared_ptr<STGraph> ptr_st_graph)
    : init_s_(init_s),
      init_d_(init_d),
      ref_line_(ref_line),
      ptr_st_graph_(std::move(ptr_st_graph)) {
  for (const auto &obstacle : ptr_obstacles) {
    obstacles_.emplace(obstacle->Id(), obstacle);
  }
}

std::vector<EndCondition> EndConditionSampler::SampleLonEndConditionForStopping(
    const double ref_stop_point) const {
  constexpr size_t time_samples_num = 9;
  std::array<double, time_samples_num> time_samples{};
  for (size_t i = 1; i < time_samples_num; ++i) {
    auto ratio = static_cast<double>(i) / static_cast<double>(time_samples_num - 1);
    time_samples[i] = PlanningConfig::Instance().max_lookahead_time() * ratio;
  }
  time_samples[0] = PlanningConfig::Instance().min_lookahead_time();
  std::vector<EndCondition> end_s_conditions;
  for (const auto &time : time_samples) {
    State end_s = {std::max(init_s_[0], ref_stop_point), 0.0, 0.0};
    end_s_conditions.emplace_back(end_s, time);
  }
  return end_s_conditions;
}

std::vector<EndCondition> EndConditionSampler::SampleLatEndCondition() {
  std::vector<EndCondition> end_d_conditions;
  std::array<double, 3> end_d_candidates = {0, -0.5, 0.5};
  std::array<double, 3> end_s_candidates = {10, 20, 40};
  for (const auto &s : end_s_candidates) {
    for (const auto &d : end_d_candidates) {
      State end_d_state = {d, 0.0, 0.0};
      end_d_conditions.emplace_back(end_d_state, s);
    }
  }
  return end_d_conditions;
}

std::vector<EndCondition> EndConditionSampler::SampleLonEndConditionForCruising(const double ref_target_vel) const {
  constexpr int kVelSampleNum = 9;  // put  into PlanningConfig
  constexpr int kTimeSampleNum = 9; // put into PlanningConfig
  constexpr double vel_interval_step = 0.3; // put int PlanningConfig
  std::array<double, kTimeSampleNum> time_samples{};

  for (size_t i = 1; i < kTimeSampleNum; ++i) {
    auto ratio = static_cast<double>(i) / static_cast<double>(kTimeSampleNum - 1);
    time_samples[i] = PlanningConfig::Instance().max_lookahead_time() * ratio;
  }
  time_samples[0] = PlanningConfig::Instance().min_lookahead_time();
  std::vector<EndCondition> end_s_conditions;
  for (const auto &time : time_samples) {
    double v_upper = std::min(VUpper(time), ref_target_vel);
    double v_lower = VLower(time);
    State lower_end_s = {0.0, v_lower, 0.0};
    end_s_conditions.emplace_back(lower_end_s, time);
    State upper_end_s = {0.0, v_upper, 0.0};
    end_s_conditions.emplace_back(upper_end_s, time);

    double vel_gap = v_upper - v_lower;
    size_t vel_intervals_num =
        std::min(static_cast<size_t>(kVelSampleNum - 2),
                 static_cast<size_t>(vel_gap / vel_interval_step));
    if (vel_intervals_num > 0) {
      double vel_ratio = vel_gap / static_cast<double>(vel_intervals_num);
      for (size_t i = 1; i < vel_intervals_num; ++i) {
        State end_s = {0.0, v_lower + vel_ratio * static_cast<double>(i), 0.0};

        end_s_conditions.emplace_back(end_s, time);
      }
    }

  }
#if DEBUG
  for (const auto &end_s : end_s_conditions) {
    std::cout << " end_conditions for cruising : s: " << end_s.first[0] << ", s_dot: " << end_s.first[1] << ", s_ddot: "
              << end_s.first[2] << ", time: " << end_s.second << std::endl;
  }
#endif
  return end_s_conditions;
}

double EndConditionSampler::VUpper(double t) const {
  double comfortable_acc = PlanningConfig::Instance().max_lon_acc() * 0.9;
  return init_s_[1] + comfortable_acc * t;
}

double EndConditionSampler::VLower(double t) const {
  double comfortable_decel = -PlanningConfig::Instance().min_lon_acc() * 0.9;
  double t_at_zero_speed = init_s_[1] / (comfortable_decel);
  return t < t_at_zero_speed ?
         init_s_[1] - comfortable_decel * t :
         0.0;
}

double EndConditionSampler::SUpper(double t) const {
  double comfortable_acc = PlanningConfig::Instance().max_lon_acc() * 0.9;
  return init_s_[0] + init_s_[1] * t +
      0.5 * comfortable_acc * t * t;
}

double EndConditionSampler::SLower(double t) const {
  double comfortable_decel = -PlanningConfig::Instance().min_lon_acc() * 0.9;
  const double t_at_zero_speed = init_s_[1] / (comfortable_decel);
  const double
      s_at_zero_speed = init_s_[0] + init_s_[1] * init_s_[1] / (2.0 * comfortable_decel);
  return t < t_at_zero_speed ? init_s_[0] + init_s_[1] * t - 0.5 * comfortable_decel * t * t
                             : s_at_zero_speed;
}

std::vector<EndCondition> EndConditionSampler::SampleLonEndConditionWithSTGraph() const {
  std::vector<EndCondition> end_s_conditions;
  std::vector<std::pair<STPoint, double>> sample_points_follow;
  std::vector<std::pair<STPoint, double>> sample_points_overtake;
  std::vector<std::pair<STPoint, double>> sample_points;
  for (const auto &st_boundary : ptr_st_graph_->GetObstaclesSTBoundary()) {
    const int obstacle_id = st_boundary.id();
    sample_points_follow = FollowingSamplePoints(obstacle_id);
    sample_points_overtake = OvertakeSamplePoints(obstacle_id);
    sample_points.insert(sample_points.end(), sample_points_follow.begin(),
                         sample_points_follow.end());
    sample_points.insert(sample_points.end(), sample_points_overtake.begin(),
                         sample_points_overtake.end());
  }

  for (const auto &sample_point : sample_points) {
    if (sample_point.first.t() < PlanningConfig::Instance().min_lookahead_time()) {
      continue;
    }
    double s = sample_point.first.s();
    double v = sample_point.second;
    double t = sample_point.first.t();
    if (s > SUpper(t) || s < SLower(t)) {
      continue;
    }
    State end_state = {s, v, 0.0};
    end_s_conditions.emplace_back(end_state, t);
  }
#if DEBUG
  for (const auto &sample_point : sample_points_overtake) {
    std::cout << "overtake sample_point: s: " << sample_point.first.s() << ", t: " << sample_point.first.t() << ", v: "
              << sample_point.second << std::endl;
    auto ref_point = ref_line_.GetReferencePoint(sample_point.first.s());
    std::cout << " overtake point in cartersian coordinate : x: " << ref_point.x() <<", y: " << ref_point.y() << std::endl;
  }
  std::cout << "----------------------------------" << std::endl;
  for (const auto &sample_point : sample_points_follow) {
    std::cout << "following sample_point: s: " << sample_point.first.s() << ", t: " << sample_point.first.t() << ", v: "
              << sample_point.second << std::endl;
    auto ref_point = ref_line_.GetReferencePoint(sample_point.first.s());
    std::cout << " following point in cartersian coordinate : x: " << ref_point.x() <<", y: " << ref_point.y() << std::endl;
  }


//  std::cout << "---------------------------------" << std::endl;
//  for (const auto &end_condition : end_s_conditions) {
//    std::cout << "overtake-following end conditions: " << " s: " << end_condition.first[0]
//              << " s_dot: " << end_condition.first[1] << " s_ddot: " << end_condition.first[2] <<
//              " t: " << end_condition.second << std::endl;
//  }
#endif

  return end_s_conditions;
}

std::vector<std::pair<STPoint, double>> EndConditionSampler::OvertakeSamplePoints(int obstacle_id) const {
  std::vector<std::pair<STPoint, double>> sample_points{};
  std::vector<STPoint> overtake_st_points = ptr_st_graph_->GetObstacleSurroundingPoints(
      obstacle_id, 1e-3, 0.2);
  for (const auto &st_point : overtake_st_points) {
    double v = GetObstacleSpeedAlongReferenceLine(obstacle_id, st_point.s(), st_point.t(), ref_line_);
    std::pair<STPoint, double> sample_point;
    sample_point.first = st_point;
    sample_point.first.set_s(st_point.s() + PlanningConfig::Instance().lon_safety_buffer()
                                 + PlanningConfig::Instance().vehicle_params().half_length
                                 - PlanningConfig::Instance().vehicle_params().back_axle_to_center_length);
    sample_point.second = v;
    sample_points.push_back(sample_point);
  }

  return sample_points;
}

std::vector<std::pair<STPoint, double>> EndConditionSampler::FollowingSamplePoints(int obstacle_id) const {
  constexpr size_t num_sample_follow_per_timestamp = 3;
  std::vector<std::pair<STPoint, double>> sample_points{};
  std::vector<STPoint> follow_st_points = ptr_st_graph_->GetObstacleSurroundingPoints(
      obstacle_id, -1e-3, 0.2);
//  std::cout << "============= vehicle params ========" << std::endl;
//  std::cout << " length: " << PlanningConfig::Instance().vehicle_params().length
//            << " width: " << PlanningConfig::Instance().vehicle_params().width
//            << " back_to_center: " << PlanningConfig::Instance().vehicle_params().back_axle_to_center_length
//            << " front_to_center: " << PlanningConfig::Instance().vehicle_params().front_axle_to_center_length << std::endl;
  for (const auto &st_point : follow_st_points) {
    double v = GetObstacleSpeedAlongReferenceLine(obstacle_id, st_point.s(), st_point.t(), ref_line_);
    double s_upper = st_point.s() - PlanningConfig::Instance().lon_safety_buffer()
        - PlanningConfig::Instance().vehicle_params().half_length
        - PlanningConfig::Instance().vehicle_params().back_axle_to_center_length;
    double s_lower = s_upper - PlanningConfig::Instance().lon_safety_buffer();
    double s_gap =
        PlanningConfig::Instance().lon_safety_buffer() / static_cast<double>(num_sample_follow_per_timestamp - 1);
    for (size_t i = 0; i < num_sample_follow_per_timestamp; ++i) {
      double s = s_lower + s_gap * static_cast<double>(i);
      std::pair<STPoint, double> sample_point;
      sample_point.first = st_point;
      sample_point.first.set_s(s);
      sample_point.second = v;
      sample_points.push_back(sample_point);
    }
  }

  return sample_points;
}

double EndConditionSampler::GetObstacleSpeedAlongReferenceLine(int obstacle_id, double s, double t,
                                                               const ReferenceLine &ref_line) const {

  if (obstacles_.empty()) {
    return 0.0;
  }
  if (obstacles_.find(obstacle_id) == obstacles_.end()) {
    return 0.0;
  }
  auto matched_obstacle = obstacles_.at(obstacle_id);
  const auto &trajectory = matched_obstacle->trajectory();
  size_t num_traj_point = trajectory.trajectory_points.size();
  if (num_traj_point < 2) {
    return 0.0;
  }
  if (t < trajectory.trajectory_points[0].relative_time ||
      t > trajectory.trajectory_points.back().relative_time) {
    return 0.0;
  }
  auto matched_iter =
      std::lower_bound(trajectory.trajectory_points.begin(), trajectory.trajectory_points.end(),
                       t, [](const planning_msgs::TrajectoryPoint &p, const double t) {
            return p.relative_time < t;
          });
  double v = matched_iter->vel;
  double theta = matched_iter->path_point.theta;
  double v_x = v * std::cos(theta);
  double v_y = v * std::sin(theta);
  ReferencePoint matched_ref_point = ref_line.GetReferencePoint(s);
  double ref_theta = matched_ref_point.theta();
  return std::cos(ref_theta) * v_x + std::sin(ref_theta) * v_y;
}
}
