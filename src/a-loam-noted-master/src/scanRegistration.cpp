// This is an advanced implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014. 

// Modifier: Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk


// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#include <cmath>
#include <vector>
#include <string>
#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "aloam_velodyne/enhanced.h"


using std::atan2;
using std::cos;
using std::sin;
typedef unsigned char uint8;
const double scanPeriod = 0.1;

const int systemDelay = 0; 
int systemInitCount = 0;
bool systemInited = false;
int N_SCANS = 0;
const uint8 N_SCANS_SEG_G = 4;//地面分离点的行数
const int H_SCANS = 1800;//VLP 16 0.2°
const int groundAngleThresold = 10;//地面点的角度值
float cloudCurvature[400000];
int cloudSortInd[400000];
int cloudNeighborPicked[400000];
int cloudLabel[400000];
PointType fullCloud[16][H_SCANS];
uint8 fullCloud_type[16][H_SCANS];//点云类型 -1 无效 0 未分离的后景点 1 地面点 2 平面点 3 边缘点 4 角点
PointType nanPoint ;
DCVC_Relative dcvc_relative;
#define INVALID_POINT 0
#define UNKNOWD_POINT 1
#define GROUND_POINT 2
#define PLANE_POINT 3
#define EDGE_POINT 4
#define CORNER_POINT 5


#define REMOVE_GROUND_POINT 1           //是否移除地面点

bool comp (int i,int j) { return (cloudCurvature[i]<cloudCurvature[j]); }

ros::Publisher pubLaserCloud;
ros::Publisher pubGroundPoint;//
ros::Publisher pubCornerPointsSharp;
ros::Publisher pubCornerPointsLessSharp;
ros::Publisher pubSurfPointsFlat;
ros::Publisher pubSurfPointsLessFlat;
ros::Publisher pubRemovePoints;
std::vector<ros::Publisher> pubEachScan;

bool PUB_EACH_LINE = false;

double MINIMUM_RANGE = 0.1; 

void My_Create::pointPrint(PointType P,int row,int col){

    printf("[%d][%d]  x: %f y: %f z: %f i: %f\n",row,col,P.x,P.y,P.z,P.intensity);
}

void My_Create::pointPrint2file(PointType P,int row,int col,std::ofstream file){
    file <<"["<<row<<"]["<<col<<"<<]  x: "<<P.x<<" y: "<<P.y<<" z: "<<P.z<<" i: "<<P.intensity<<"\n";
}

void My_Create::print_pointtypes(){
    int point_type[6];
    memset(point_type,0,sizeof (point_type));
    for(int i = 0;i<N_SCANS;i++)
        for(int j = 0;j<H_SCANS;j++){
            if(fullCloud_type[i][j] == INVALID_POINT)point_type[INVALID_POINT]++;
            if(fullCloud_type[i][j] == GROUND_POINT)point_type[GROUND_POINT]++;
            if(fullCloud_type[i][j] == UNKNOWD_POINT)point_type[UNKNOWD_POINT]++;
        }
    printf("点的类型 %d %d %d \n",point_type[INVALID_POINT],point_type[GROUND_POINT],point_type[UNKNOWD_POINT]);
}

void My_Create::removeGroundPoint(std::vector<pcl::PointCloud<PointType>> &laserCloud_input){//分离地面店， 并且存入点云lasercloud
    for(int line = 0;line < N_SCANS_SEG_G;line++){//取四分之一线束进行地面分割就也可以了   在网上也没有地面点
        for(int wire = 0;wire<H_SCANS;wire++){
            if(fullCloud[line][wire].intensity==-1)continue;//本身无效 直接下一次循环
            if(fullCloud[line+1][wire].intensity==-1){fullCloud_type[line][wire] = UNKNOWD_POINT; laserCloud_input[line].push_back(fullCloud[line][wire]);continue;}//参考点无效 标记为非地面点后再进入下一次循环

                double diffX,diffY,diffZ;
                diffX = fullCloud[line][wire].x- fullCloud[line+1][wire].x;
                diffY = fullCloud[line][wire].y- fullCloud[line+1][wire].y;
                diffZ = fullCloud[line][wire].z- fullCloud[line+1][wire].z;
                double angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;

                if (abs(angle ) <= groundAngleThresold){
                    fullCloud_type[line][wire] = GROUND_POINT;//设置为地面点

                }
                else{

                    fullCloud_type[line][wire] = UNKNOWD_POINT;//否则为未分类的后景点
                    laserCloud_input[line].push_back(fullCloud[line][wire]);
                }

        }

    }
}

/*
 * 初始化一些基本参数*/
void My_Create:: init_parameters(){
    //无效点的定义
    nanPoint.x = std::numeric_limits<float>::quiet_NaN();
    nanPoint.y = std::numeric_limits<float>::quiet_NaN();
    nanPoint.z = std::numeric_limits<float>::quiet_NaN();
    nanPoint.intensity = -1;
    //参数文件的导入
    std::string config_file_path = "/home/ren/catkin_ws/catkin_aloam/src/a-loam-noted-master/config/config.yaml";
    printf("dddd     %s",config_file_path.c_str());
    printf("dddd    ");
    YAML::Node config_node = YAML::LoadFile(config_file_path);
    dcvc_relative.startR = config_node["DCVC"]["startR"].as<double>();
    dcvc_relative.deltaR = config_node["DCVC"]["deltaR"].as<double>();
    dcvc_relative.deltaP = config_node["DCVC"]["deltaP"].as<double>();
    dcvc_relative.deltaA = config_node["DCVC"]["deltaA"].as<double>();
    dcvc_relative.minSeg = config_node["DCVC"]["minSeg"].as<int>();
}

void My_Create::reset_parameters(){
    //全点云初始化
    for (int i = 0; i<N_SCANS ;i++)//将
        for (int j = 0; j<H_SCANS ;j++)//将
        {
            fullCloud[i][j] = nanPoint;
            fullCloud_type[i][j] = INVALID_POINT;//全部初始化为无效点
        }
}
int DCVC_Relative::getPolarIndex(double &radius) {
    for (auto r = 0; r < polarNum; ++r){
        if (radius < polarBounds[r])
            return r;
    }

    return polarNum - 1;
}
bool DCVC_Relative::createHashTable() {
    size_t totalSize = polarCor.size();
    if (totalSize <= 0){
        return false;
    }

    Eigen::Vector3d cur = Eigen::Vector3d::Zero();
    int polarIndex, pitchIndex, azimuthIndex, voxelIndex;
    voxelMap.reserve(totalSize);

    for (size_t item = 0; item < totalSize; ++item){
        cur = polarCor[item];
        polarIndex = getPolarIndex(cur.x());
        pitchIndex = static_cast<int>(std::round((cur.y() - minPitch) / deltaP));
        azimuthIndex = static_cast<int>(std::round(cur.z() / deltaA));

        voxelIndex = (azimuthIndex * (polarNum + 1) + polarIndex) + pitchIndex * (polarNum + 1) * (width + 1);

        auto iter = voxelMap.find(voxelIndex);
        if (iter != voxelMap.end()){
            //iter->second.index.emplace_back(item);
            iter->second.emplace_back(item);
        }else{
            std::vector<int> index{};
            index.emplace_back(item);
            voxelMap.insert(std::make_pair(voxelIndex, index));
        }
    }

    return true;
}

void DCVC_Relative::searchKNN(int &polar_index, int &pitch_index, int &azimuth_index,
                             std::vector<int> &out_neighIndex) const {

    for (auto z = pitch_index - 1; z <= pitch_index + 1; ++z){
        if (z < 0 || z > height)
            continue;
        for (int y = polar_index - 1; y <= polar_index + 1; ++y){
            if (y < 0 || y > polarNum)
                continue;

            for (int x = azimuth_index - 1; x <= azimuth_index + 1; ++x){
                int ax = x;
                if (ax < 0)
                    ax = width - 1;
                if (ax > 300)
                    ax = 300;

                out_neighIndex.emplace_back((ax*(polarNum+1)+y) + z*(polarNum+1)*(width+1));
            }
        }
    }
}
void DCVC_Relative::convertToPolar(pcl::PointCloud<pcl::PointXYZ> &cloud_in_) {
    if (cloud_in_.empty()){
        ROS_ERROR("object cloud don't have point, please check !");
        return;
    }
    auto azimuthCal = [&](double x, double y)->double{//用xy计算偏航角方法
        auto angle = static_cast<double>(std::atan2(y, x));
        return angle > 0.0 ? angle*180/M_PI : (angle+2*M_PI)*180/M_PI;
    };

    size_t totalSize = cloud_in_.points.size();
    polarCor.resize(totalSize);

    //首先把点云再笛卡尔坐标系，转换为极坐标系

    Eigen::Vector3d cur = Eigen::Vector3d::Zero();
    std::vector<double> temp_see;
    for (size_t i = 0; i < cloud_in_.points.size(); i++){
        Eigen::Vector3d rpa = Eigen::Vector3d::Zero();
        cur.x() = cloud_in_.points[i].x;//先读取值，将pcl格式的变量转为eigen变量
        cur.y() = cloud_in_.points[i].y;
        cur.z() = cloud_in_.points[i].z;
        rpa.x() = cur.norm(); // 用rpa的x存储极坐标的极径
        rpa.y() = std::asin(cur.z() / rpa.x()) * 180.0 / M_PI;    //俯仰角
        rpa.z() = azimuthCal(cur.x(), cur.y());                   // 偏航角
        temp_see.emplace_back(cur.x());
        temp_see.emplace_back(cur.y());
        temp_see.emplace_back(cur.z());
        temp_see.emplace_back(rpa.x());
        temp_see.emplace_back(rpa.y());
        temp_see.emplace_back(rpa.z());
        temp_see.clear();
//        if (rpa.x() >= sensorMaxRange || rpa.x() <= sensorMinRange)   //后续补充一下  传感器范围
//            continue;

        minPitch = rpa.y() < minPitch ? rpa.y() : minPitch;//存储该点云中的最大最小值
        maxPitch = rpa.y() > maxPitch ? rpa.y() : maxPitch;//存储该点云中的最大最小值
        minPolar = rpa.x() < minPolar ? rpa.x() : minPolar;//存储该点云中的最大最小值
        maxPolar = rpa.x() > maxPolar ? rpa.x() : maxPolar;//存储该点云中的最大最小值
        minAzi = rpa.z() < minAzi ? rpa.z() : minAzi;//存储该点云中的最大最小值
        maxAzi = rpa.z() > maxAzi ? rpa.z() : maxAzi;//存储该点云中的最大最小值
        polarCor[i] = rpa;//存储该点云转化后的值
    }

    polarCor.shrink_to_fit();

    polarNum = 0;
    polarBounds.clear();
    width = static_cast<int>(std::round(360.0 / deltaA) + 1);
    height = static_cast<int>((maxPitch - minPitch) / deltaP);
    double range = minPolar;
    int step = 1;//步长设置
    while (range <= maxPolar){//该循环的作用是，实现论文中的每层极径分割
        range += (startR - step * deltaR);
        polarBounds.emplace_back(range);
        polarNum++, step++;
    }
}

bool DCVC_Relative ::DCVC(std::vector<int> &label_info) {

    int labelCount = 0;
    size_t totalSize = polarCor.size();
    if (totalSize <= 0){
        ROS_ERROR("there are not enough point clouds to complete the DCVC algorithm");
        return false;
    }

    label_info.resize(totalSize, -1);
    Eigen::Vector3d cur = Eigen::Vector3d::Zero(); //重置cur为0
    int polar_index, pitch_index, azimuth_index, voxel_index, currInfo, neighInfo;

    for (size_t i = 0; i < totalSize; ++i){
        if (label_info[i] != -1) //对于已经分类的体素，不再判断。
            continue;
        cur = polarCor[i];

        polar_index = getPolarIndex(cur.x());//通过迭代半径数组， 判断半径的长度 ，存在哪个polar中。
        pitch_index = static_cast<int>(std::round((cur.y() - minPitch) / deltaP));//角度 / 各个格子的角度大小
        azimuth_index = static_cast<int>(std::round(cur.z() / deltaA)); //角度 / 各个格子的角度大小
        voxel_index = (azimuth_index*(polarNum+1) + polar_index) + pitch_index*(polarNum+1)*(width+1);

        auto iter_find = voxelMap.find(voxel_index); // 找到体素对应map中的迭代数
        std::vector<int> neighbors;  // 体素的邻居
        if (iter_find != voxelMap.end()){

            std::vector<int> KNN{}; //最近邻的k个体素
            searchKNN(polar_index, pitch_index, azimuth_index, KNN);//找最近邻的26个体素（3*3*3方格）

            for (auto& k : KNN){
                iter_find = voxelMap.find(k);//找到栅格地图中的相应体素

                if (iter_find != voxelMap.end()){
                    neighbors.reserve(iter_find->second.size());//指定vector的大小为非空体素的大小
                    for (auto& id : iter_find->second){ //second是值
                        neighbors.emplace_back(id);//再vector（neighbors）后插入一个体素
                    }
                }
            }
        }

        neighbors.swap(neighbors);//swap再此处的作用是：swap自身，内容不变；改变了vector大小，删除冗余部分

        if (!neighbors.empty()){//如果当前体素有邻近体素   那么就进行统一的标记
            for (auto& id : neighbors){
                currInfo = label_info[i];       // current label index
                neighInfo = label_info[id];     // voxel label index
                if (currInfo != -1 && neighInfo != -1 && currInfo != neighInfo){//若当前的体素与其邻居体素的标签类别不一样，而且都有值
                    for (auto& seg : label_info){
                        if (seg == currInfo)
                            seg = neighInfo;
                    }
                }else if (neighInfo != -1){//如果邻居体素有标签，自己没标签，自己的标签就用邻居体素的
                    label_info[i] = neighInfo;
                }else if (currInfo != -1){//反之邻居用自身标签
                    label_info[id] = currInfo;
                }else{
                    continue; // 都是空的话（就是都没标记），就不会标记，如果全部continue  就会进入下面的 lable_info == -1 的环节
                }
            }
        }

        // If there is no category information yet, then create a new label information
        if (label_info[i] == -1){//基本上是用于第一个体素的
            labelCount++;
            label_info[i] = labelCount;
            for (auto& id : neighbors){
                label_info[id] = labelCount;//将邻居的标签也全部用于这里
            }
        }
    }
    //至此所有的标签都打上了
    // free memory
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>().swap(polarCor);

    return true;
}



template <typename PointT/*这是一个模板*/>
void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                            pcl::PointCloud<PointT> &cloud_out, float thres)
{
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;
    // 把点云距离小于给定阈值的去除掉
    for (size_t i = 0; i < cloud_in.points.size(); ++i)
    {
        if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
            continue;
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }
    if (j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }

    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}


// 订阅lidar消息
long frame_cur = 0;
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{

    setlocale(LC_ALL, ""); // 设置rosinfo中文不乱吗
    // 如果系统没有初始化的话，就等几帧
    if (!systemInited)
    { 
        systemInitCount++;
        if (systemInitCount >= systemDelay)
        {
            systemInited = true;
        }
        else
            return;
    }
    My_Create myCreate;//实例化类方法
    printf("當前幀[%d]",frame_cur++);

    TicToc t_whole;
    TicToc t_prepare;
    std::vector<int> scanStartInd(N_SCANS, 0);
    std::vector<int> scanEndInd(N_SCANS, 0);
    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;//输入点云经过nan值和小于阈值处理后，基本上为原来的一班
    // 把点云从ros格式转到pcl的格式
    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
    myCreate.reset_parameters(); //参数重置 / 刷新
    printf("当前帧的点云数目为：【%lu】",laserCloudIn.points.size());
    printf("full_cloud_size :%zu \n",sizeof (fullCloud) / sizeof(fullCloud[0][0]));
    printf("full_cloud_value :%f %f %f %f \n", fullCloud[0][0].x,fullCloud[0][0].x,fullCloud[0][0].x,fullCloud[0][0].intensity);
    std::vector<int> indices;

    // 去除掉点云中的nan点
    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
    printf("当前帧去除nan后的点云数目为：【%lu】",laserCloudIn.points.size());
    // 去除距离小于阈值的点
    removeClosedPointCloud(laserCloudIn, laserCloudIn, MINIMUM_RANGE);
    printf("当前帧去除小于距离后的点云数目为：【%lu】",laserCloudIn.points.size());
    // 计算起始点和结束点的角度，由于激光雷达是顺时针旋转，这里取反就相当于转成了逆时针
    int cloudSize = laserCloudIn.points.size();
    float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);
    // atan2范围是[-Pi,PI]，这里加上2PI是为了保证起始到结束相差2PI符合实际
    float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y, laserCloudIn.points[cloudSize - 1].x) +
                   2 * M_PI;

    // 总有一些例外，比如这里大于3PI，和小于PI，就需要做一些调整到合理范围
    if (endOri - startOri > 3 * M_PI)
    {
        endOri -= 2 * M_PI;
    }
    else if (endOri - startOri < M_PI)
    {
        endOri += 2 * M_PI;
    }
    //printf("end Ori %f\n", endOri);

    bool halfPassed = false;
    int count = cloudSize;
    PointType point;
    std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);
    // 遍历每一个点
    for (int i = 0; i < cloudSize; i++)
    {
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;
        // 计算他的俯仰角
        float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
        int scanID = 0;//激光雷达行数
        // 计算是第几根scan
        if (N_SCANS == 16)
        {
            scanID = int((angle + 15) / 2 + 0.5);
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (N_SCANS == 32)
        {
            scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (N_SCANS == 64)
        {
            if (angle >= -8.83)
                scanID = int((2 - angle) * 3.0 + 0.5);
            else
                scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);

            // use [0 50]  > 50 remove outlies
            if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else
        {
            printf("wrong scan number\n");
            ROS_BREAK();
        }
        //printf("angle %f scanID %d \n", angle, scanID);
        // 计算水平角
        float ori = -atan2(point.y, point.x);
        if (!halfPassed)
        {
            // 确保-PI / 2 < ori - startOri < 3 / 2 * PI
            if (ori < startOri - M_PI / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > startOri + M_PI * 3 / 2)
            {
                ori -= 2 * M_PI;
            }
            // 如果超过180度，就说明过了一半了
            if (ori - startOri > M_PI)
            {
                halfPassed = true;
            }
        }
        else
        {
            // 确保-PI * 3 / 2 < ori - endOri < PI / 2
            ori += 2 * M_PI;    // 先补偿2PI
            if (ori < endOri - M_PI * 3 / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > endOri + M_PI / 2)
            {
                ori -= 2 * M_PI;
            }
        }
        // 角度的计算是为了计算相对的起始时刻的时间
        float relTime = (ori - startOri) / (endOri - startOri);
        // 整数部分是scan的索引，小数部分是相对起始时刻的时间
        point.intensity = scanID + scanPeriod * relTime;
        // 根据scan的idx送入各自数组
        if(scanID >= N_SCANS_SEG_G)//不是分离地面点的部分  可以直接存入lasercloud中。
        laserCloudScans[scanID].push_back(point);
        //否则先存入全点云
        fullCloud[scanID][int(relTime*1800.0)]=point;//压入全点云中

    }

    // cloudSize是有效的点云的数目
    cloudSize = count;
    printf("points size %d \n", cloudSize);

    pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
    // 全部集合到一个点云里面去，但是使用两个数组标记其实和结果，这里分别+5和-6是为了计算曲率方便
    for (int i = 0; i < N_SCANS; i++)
    { 
        scanStartInd[i] = laserCloud->size() + 5;
        *laserCloud += laserCloudScans[i];
        scanEndInd[i] = laserCloud->size() - 6;
    }

    printf("prepare time %f cloudsize是%d laserCloud大小%lu\n", t_prepare.toc(),cloudSize,laserCloud->points.size());

    if(myCreate.use_update) {//更新方法

        //1.将点中的地面点分离出来
#if(REMOVE_GROUND_POINT)
        myCreate.removeGroundPoint(laserCloudScans);// 地面定义激光的下面四条线还是空的，先进行地面点的删除，再加入到点云中。
        myCreate.print_pointtypes();
#endif


        for (int i = 0; i < N_SCANS; i++) {
            int count_ = 0;
            for (int j = 0; j < H_SCANS; j++) {
                if (fullCloud[i][j].intensity != -1)count_++;
            }
            printf("%d行数量%lu  ____   full :%d\n", i, laserCloudScans[i].size(), count_);
        }
        if (frame_cur == 10) {
            std::ofstream fullcloud_txt;
            fullcloud_txt.open("fullcloud_txt.txt");
            if (fullcloud_txt.is_open()) {
                char *temp = getcwd(NULL, 0);
                printf("文件打开了 %s\n", temp);

            }

            for (int i = 0; i < N_SCANS; i++) {
                for (int j = 0; j < H_SCANS; j++) {
                    fullcloud_txt << "[" << i << "][" << j << "]  x: " << fullCloud[i][j].x << " y: " << fullCloud[i][j].y
                                  << " z: " << fullCloud[i][j].z << " i: " << fullCloud[i][j].intensity << "\n";
                }
            }
            fullcloud_txt.close();
        }

        laserCloudIn.clear();//清空
        laserCloudIn.resize(laserCloud->points.size());
        for(size_t i = 0;i<laserCloud->points.size();i++){//重置lasercloudin
            double tt ;
            laserCloudIn[i].x = laserCloud->points[i].x;
            tt = laserCloud->points[i].x;
            tt =   laserCloudIn[i].x;
            laserCloudIn[i].y = laserCloud->points[i].y;
            laserCloudIn[i].z = laserCloud->points[i].z;
        }


        ROS_INFO("重置后的lasercloudin大小为%lu %zu",laserCloudIn.size(),laserCloudScans.size());


        dcvc_relative.convertToPolar(laserCloudIn); //1、将坐标系转为球坐标系下
        //dcvc_relative.createHashTable();//2、建立哈希表
        //dcvc_relative.DCVC(dcvc_relative.labelInfo);//3、动态体素滤波
    }

    // 开始计算曲率
    for (int i = 5; i < cloudSize - 5; i++)
    {
        float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;
        float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;
        float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;
        // 存储曲率，索引
        cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
        cloudSortInd[i] = i;
        cloudNeighborPicked[i] = 0;
        cloudLabel[i] = 0;
    }
    ROS_INFO("曲率计算完毕");

    TicToc t_pts;

    pcl::PointCloud<PointType> cornerPointsSharp;
    pcl::PointCloud<PointType> cornerPointsLessSharp;
    pcl::PointCloud<PointType> surfPointsFlat;
    pcl::PointCloud<PointType> surfPointsLessFlat;

    float t_q_sort = 0;
    // 遍历每个scan
    for (int i = 0; i < N_SCANS; i++)
    {
        // 没有有效的点了，就continue
        if( scanEndInd[i] - scanStartInd[i] < 6)
            continue;
        // 用来存储不太平整的点
        pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
        // 将每个scan等分成6等分
        for (int j = 0; j < 6; j++)
        {
            // 每个等分的起始和结束点
            int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6; 
            int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;

            TicToc t_tmp;
            // 对点云按照曲率进行排序，小的在前，大的在后
            std::sort (cloudSortInd + sp, cloudSortInd + ep + 1, comp);
            t_q_sort += t_tmp.toc();

            int largestPickedNum = 0;
            // 挑选曲率比较大的部分
            for (int k = ep; k >= sp; k--)
            {
                // 排序后顺序就乱了，这个时候索引的作用就体现出来了
                int ind = cloudSortInd[k]; 

                // 看看这个点是否是有效点，同时曲率是否大于阈值
                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] > 0.1)
                {

                    largestPickedNum++;
                    // 每段选2个曲率大的点
                    if (largestPickedNum <= 2)
                    {                        
                        // label为2是曲率大的标记
                        cloudLabel[ind] = 2;
                        // cornerPointsSharp存放大曲率的点
                        cornerPointsSharp.push_back(laserCloud->points[ind]);
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                    }
                    // 以及20个曲率稍微大一些的点
                    else if (largestPickedNum <= 20)
                    {                        
                        // label置1表示曲率稍微大
                        cloudLabel[ind] = 1; 
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                    }
                    // 超过20个就算了
                    else
                    {
                        break;
                    }
                    // 这个点被选中后 pick标志位置1
                    cloudNeighborPicked[ind] = 1; 
                    // 为了保证特征点不过度集中，将选中的点周围5个点都置1,避免后续会选到
                    for (int l = 1; l <= 5; l++)
                    {
                        // 查看相邻点距离是否差异过大，如果差异过大说明点云在此不连续，是特征边缘，就会是新的特征，因此就不置位了
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    // 下面同理
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }
            // 下面开始挑选面点
            int smallestPickedNum = 0;
            for (int k = sp; k <= ep; k++)
            {
                int ind = cloudSortInd[k];
                // 确保这个点没有被pick且曲率小于阈值
                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] < 0.1)
                {
                    // -1认为是平坦的点
                    cloudLabel[ind] = -1; 
                    surfPointsFlat.push_back(laserCloud->points[ind]);

                    smallestPickedNum++;
                    // 这里不区分平坦和比较平坦，因为剩下的点label默认是0,就是比较平坦
                    if (smallestPickedNum >= 4)
                    { 
                        break;
                    }
                    // 下面同理
                    cloudNeighborPicked[ind] = 1;
                    for (int l = 1; l <= 5; l++)
                    { 
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            for (int k = sp; k <= ep; k++)
            {
                // 这里可以看到，剩下来的点都是一般平坦，这个也符合实际
                if (cloudLabel[k] <= 0)
                {
                    surfPointsLessFlatScan->push_back(laserCloud->points[k]);
                }
            }
        }

        pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
        pcl::VoxelGrid<PointType> downSizeFilter;
        // 一般平坦的点比较多，所以这里做一个体素滤波
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);
        downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
        downSizeFilter.filter(surfPointsLessFlatScanDS);

        surfPointsLessFlat += surfPointsLessFlatScanDS;
    }
    printf("sort q time %f \n", t_q_sort);
    printf("seperate points time %f \n", t_pts.toc());

    // 分别将当前点云、四种特征的点云发布出去
    sensor_msgs::PointCloud2 laserCloudOutMsg;
    pcl::toROSMsg(*laserCloud, laserCloudOutMsg);
    laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;
    laserCloudOutMsg.header.frame_id = "camera_init";
    pubLaserCloud.publish(laserCloudOutMsg);

    sensor_msgs::PointCloud2 cornerPointsSharpMsg;
    pcl::toROSMsg(cornerPointsSharp, cornerPointsSharpMsg);
    cornerPointsSharpMsg.header.stamp = laserCloudMsg->header.stamp;
    cornerPointsSharpMsg.header.frame_id = "camera_init";
    pubCornerPointsSharp.publish(cornerPointsSharpMsg);

    sensor_msgs::PointCloud2 cornerPointsLessSharpMsg;
    pcl::toROSMsg(cornerPointsLessSharp, cornerPointsLessSharpMsg);
    cornerPointsLessSharpMsg.header.stamp = laserCloudMsg->header.stamp;
    cornerPointsLessSharpMsg.header.frame_id = "camera_init";
    pubCornerPointsLessSharp.publish(cornerPointsLessSharpMsg);

    sensor_msgs::PointCloud2 surfPointsFlat2;
    pcl::toROSMsg(surfPointsFlat, surfPointsFlat2);
    surfPointsFlat2.header.stamp = laserCloudMsg->header.stamp;
    surfPointsFlat2.header.frame_id = "camera_init";
    pubSurfPointsFlat.publish(surfPointsFlat2);

    sensor_msgs::PointCloud2 surfPointsLessFlat2;
    pcl::toROSMsg(surfPointsLessFlat, surfPointsLessFlat2);
    surfPointsLessFlat2.header.stamp = laserCloudMsg->header.stamp;
    surfPointsLessFlat2.header.frame_id = "camera_init";
    pubSurfPointsLessFlat.publish(surfPointsLessFlat2);

    // pub each scam
    // 可以按照每个scan发出去，不过这里是false
    if(PUB_EACH_LINE)
    {
        for(int i = 0; i< N_SCANS; i++)
        {
            sensor_msgs::PointCloud2 scanMsg;
            pcl::toROSMsg(laserCloudScans[i], scanMsg);
            scanMsg.header.stamp = laserCloudMsg->header.stamp;
            scanMsg.header.frame_id = "camera_init";
            pubEachScan[i].publish(scanMsg);
        }
    }

    printf("scan registration time %f ms *************\n", t_whole.toc());
    if(t_whole.toc() > 100)
        ROS_WARN("scan registration process over 100ms");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scanRegistration");
    ros::NodeHandle nh;
    // 从配置文件中获取多少线的激光雷达
    nh.param<int>("scan_line", N_SCANS, 16);
    // 最小有效距离
    nh.param<double>("minimum_range", MINIMUM_RANGE, 0.1);

    printf("scan line number %d \n", N_SCANS);
    My_Create myCreate;
    // 只有线束是16 32 64的才可以继续
    if(N_SCANS != 16 && N_SCANS != 32 && N_SCANS != 64)
    {
        printf("only support velodyne with 16, 32 or 64 scan line!");
        return 0;
    }

    myCreate.init_parameters();


        ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100,  laserCloudHandler);

        pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100);

        //pubGroundPoint = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_ground", 100);

        pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100);

        pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100);

        pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100);

        pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100);

        pubRemovePoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_remove_points", 100);

        if (PUB_EACH_LINE) {
            for (int i = 0; i < N_SCANS; i++) {
                ros::Publisher tmp = nh.advertise<sensor_msgs::PointCloud2>("/laser_scanid_" + std::to_string(i), 100);
                pubEachScan.push_back(tmp);
            }
        }
        ros::spin();

    return 0;
}


