#pragma once

#include <array>
#include <vector>
#include <memory>

#include "AbstractModel.hpp"

using  Vector2VP = std::array<GRANSAC::VPFloat, 2>;
using GransacVec = std::vector<std::shared_ptr<GRANSAC::AbstractParameter>>;
class Point2D : public GRANSAC::AbstractParameter {
public:
	Point2D(GRANSAC::VPFloat x, GRANSAC::VPFloat y)
	{
		m_Point2D[0] = x;
		m_Point2D[1] = y;
	};

	Vector2VP m_Point2D;
};

class Line2DModel : public GRANSAC::AbstractModel<2> {
public:
	Line2DModel(const GransacVec &InputParams)
	{
		Initialize(InputParams);
	};

    /*Get different Line type*/
	void Initialize(const GransacVec &InputParams) override
	{
		if (InputParams.size() != 2) {
            throw std::runtime_error("Line2DModel - Number of input parameters does not match minimum number required for this model.");
		}
			
		// Check for AbstractParamter types
		auto Point1 = std::dynamic_pointer_cast<Point2D>(InputParams[0]);
		auto Point2 = std::dynamic_pointer_cast<Point2D>(InputParams[1]);
		if (Point1 == nullptr || Point2 == nullptr)
			throw std::runtime_error("Line2DModel - InputParams type mismatch. It is not a Point2D.");

		std::copy(InputParams.begin(), InputParams.end(), m_MinModelParams.begin());

		// Compute the line parameters
		m_m = (Point2->m_Point2D[1] - Point1->m_Point2D[1]) / (Point2->m_Point2D[0] - Point1->m_Point2D[0]); // Slope
		m_d = Point1->m_Point2D[1] - m_m * Point1->m_Point2D[0]; // Intercept
		// m_d = Point2->m_Point2D[1] - m_m * Point2->m_Point2D[0]; // Intercept - alternative should be the same as above

		// mx - y + d = 0
		m_a = m_m;
		m_b = -1.0;
		m_c = m_d;

		m_DistDenominator = sqrt(m_a * m_a + m_b * m_b); // Cache square root for efficiency
	};

	virtual std::pair<GRANSAC::VPFloat, GransacVec> Evaluate(const GransacVec& EvaluateParams, GRANSAC::VPFloat Threshold)
	{
		std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> Inliers;
		std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> outliers;
		int nTotalParams = EvaluateParams.size();
		int nInliers = 0;

		for (auto& Param : EvaluateParams) {
			if (ComputeDistanceMeasure(Param) < Threshold) {
				Inliers.push_back(Param);
				nInliers++;
			} else {
                outliers.push_back(Param);
			}
		}

		GRANSAC::VPFloat InlierFraction = GRANSAC::VPFloat(nInliers) / GRANSAC::VPFloat(nTotalParams); // This is the inlier fraction

        std::cout << "iterator: inliners size:" << Inliers.size() << std::endl;
		return std::make_pair(InlierFraction, Inliers);
	};
protected:
	// Parametric form
	GRANSAC::VPFloat m_a, m_b, m_c; // ax + by + c = 0
	GRANSAC::VPFloat m_DistDenominator; // = sqrt(a^2 + b^2). Stored for efficiency reasons

	// Another parametrization y = mx + d
	GRANSAC::VPFloat m_m; // Slope
	GRANSAC::VPFloat m_d; // Intercept

	virtual GRANSAC::VPFloat ComputeDistanceMeasure(GransacVec Param) override
	{
		auto ExtPoint2D = std::dynamic_pointer_cast<Point2D>(Param);
		if (ExtPoint2D == nullptr) {
			return -1;
		}

		// Return distance between passed "point" and this line
		// http://mathworld.wolfram.com/Point-LineDistance2-Dimensional.html
		GRANSAC::VPFloat Numer = fabs(m_a * ExtPoint2D->m_Point2D[0] + m_b * ExtPoint2D->m_Point2D[1] + m_c);
		GRANSAC::VPFloat Dist = Numer / m_DistDenominator;

		return Dist;
	};
};

