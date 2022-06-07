#ifndef LAYOUT_H
#define LAYOUT_H

#include <vector>
#include <string>

#include "picojson.h"
#include "globals.h"
#include <fstream>

using namespace std;

class Layout {

public:
    //constructor for layout
    Layout(std::string worldname);
    ~Layout(){}
    int  getWidth () {return int(data_.get("size").get(0).get<double>());};
    int  getHeight ();
    int  getStartX();
    int  getStartY();
    int getBeliefRows();
    int getBeliefCols();
    
    vector<vector<int>>  getLineData();
    vector<vector<int>>  getOtherData();
    std::string getHostDir();
    vector<int>  getFinish ();
    vector<vector<int>> getBlockData ();
    vector<vector<int>> getIntersectionData();
    vector<vector<int>> getAgentGraph();
    vector<vector<int>> getHostGraph();

private:
    void loadData(std::string filename);
    void assertValid();
    picojson::value data_;
};

Layout::Layout(std::string worldname) 
{
     loadData(worldname);
     assertValid();
}

void Layout::loadData(std::string worldname)
{
    //read file and load file
    std::string filename = worldname + ".json";
    std::string layoutpath = "../layout/"+filename;
    std::ifstream infile(layoutpath);
    if (infile) {
        infile >> data_;
    } else {
        std::cerr<<"Can't open the file!"<<std::endl;
    }
}

vector<vector<int>> Layout::getLineData(){
    vector<vector<int>> result;
    if  (data_.contains("roadline")){
        picojson::array val = data_.get("roadline").get<picojson::array>();
        result.resize(val.size());
        for (unsigned int i = 0; i < result.size(); ++i)
            for (picojson::value ele: val[i].get<picojson::array>())
               result[i].push_back(ele.get<double>());
    }
    return result;
}

vector<vector<int>> Layout:: getOtherData()
{
    vector<vector<int>> result;
    if (data_.contains("others")) {
        picojson::array re =  data_.get("others").get<picojson::array>();
        result.resize(re.size());
        for (int i = 0; i < re.size(); i++){
            size_t le = re[i].get<picojson::array>().size();
            result[i].resize(le);
            for (int j = 0; j < result[i].size(); j++)
                result[i][j] = re[i].get<picojson::array>()[j].get<double>();
        }
    }
    return result;
}

int Layout::getHeight() {

    return int(data_.get("size").get(1).get<double>());
}

int Layout:: getStartX() {
   return int(data_.get("host").get(0).get<double>());
}

int Layout:: getStartY() {
     return int(data_.get("host").get(1).get<double>());
}

vector<int> Layout::getFinish() {
    vector<int> result;
    picojson::array finish =  data_.get("finish").get<picojson::array>();
    for (picojson::value val : finish)
        result.push_back(val.get<double>());
    return result;
}

vector<vector<int>>  Layout::getBlockData() {
    picojson::array blocks =   data_.get("blocks").get<picojson::array>();
    vector<vector<int>>  a;
    a.resize(blocks.size());
    for (int i = 0; i < a.size(); i++) {
        picojson::array arrayele = blocks[i].get<picojson::array>();
        for (picojson::value ele : arrayele) {
            a[i].push_back(int(ele.get<double>()));
        }
    }
    return a;
}

vector<vector<int>> Layout::getAgentGraph()
{
    picojson::array arr =  data_.get("agentGraph").get("nodes").get<picojson::array>();
    vector<vector<int>> result(arr.size());
    for (int i = 0; i < result.size(); i++){
        for (picojson::value ele: arr[i].get<picojson::array>())
            result[i].push_back(ele.get<double>());
    }
    return result;
}

vector<vector<int>> Layout::getHostGraph() 
{
    picojson::array arr =  data_.get("hostGraph").get("nodes").get<picojson::array>();
    vector<vector<int>> result(arr.size());
    for (int i = 0; i < result.size(); i++){
        for (picojson::value ele: arr[i].get<picojson::array>())
            result[i].push_back(ele.get<double>());
    }
    return result;
}


vector<vector<int>> Layout::getIntersectionData() {
    
    vector<vector<int>> output;
    if (!data_.contains("intersections"))
        return output;
    picojson::value val = data_.get("intersections");
    picojson::array arr = val.get("nodes").get<picojson::array>();
    vector<vector<int>> result(arr.size());
    for (int i = 0; i < result.size(); i++){
        for (picojson::value ele: arr[i].get<picojson::array>())
            result[i].push_back(ele.get<double>());
    }
    return result;

}


int Layout::getBeliefRows() {
    return int(getHeight()/Globals::constant.BELIEF_TILE_SIZE);
}

int Layout::getBeliefCols() {
    return int(getWidth()/Globals::constant.BELIEF_TILE_SIZE);
}

std::string Layout::getHostDir() {
    return data_.get("hostDir").to_str();
}

void Layout::assertValid() {
    int width = getWidth();
    int height = getHeight();
    assert(width %  Globals::constant.BELIEF_TILE_SIZE == 0);
    assert(height % Globals::constant.BELIEF_TILE_SIZE == 0);
}
#endif
