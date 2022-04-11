//
// Created by ren on 2022/4/9.
//

#ifndef ALOAM_VELODYNE_ENHANCED_H
#define ALOAM_VELODYNE_ENHANCED_H
#include <unordered_map>
#include <yaml-cpp/yaml.h>
class segInfo{
public:
    int label{-1};
    std::vector<int> index{};
};

class My_Create{
public:
    My_Create(){use_update = 1;};
    virtual ~My_Create() = default;
    void reset_parameters();//参数重置
    void init_parameters();//参数初始化
    void removeGroundPoint(std::vector<pcl::PointCloud<PointType>> &laserCloud_input);//地面点分割
    void print_pointtypes();
    void pointPrint(PointType P,int row,int col);
    void pointPrint2file(PointType P,int row,int col,std::ofstream file);
    bool use_update ;//是否使用该优化方法

};

class DCVC_Relative:My_Create{
public:
    bool DCVC(std::vector<int>& label_info);//label_info是指为点云中的第几个，即把点云分为很多个label
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> polarCor; //极坐标系坐标容器
    std::unordered_map<int, std::vector<int>> voxelMap{};//体素滤波后的map
    std::vector<std::pair<int, segInfo>> labelRecords;
    double minPitch{0.0}; //动态体素的参数设置   [-15°，15°]
    double maxPitch{0.0}; //动态体素的参数设置   [-15°，15°]
    double minPolar{5.0}; //动态体素的参数设置
    double maxPolar{5.0}; //动态体素的参数设置
    double minAzi{180.0}; //动态体素的参数设置   [0°，360°]
    double maxAzi{180.0}; //动态体素的参数设置   [0°，360°]
    double startR{0.0}; //动态体素的参数设置
    double deltaR{0.0}; //动态体素的参数设置
    double deltaP{0.0}; //动态体素的参数设置
    double deltaA{0.0}; //动态体素的参数设置
    int width{0};//偏航角方向上的栅格个数
    int height{0};// 俯仰角方向上的栅格数
    int minSeg{0};
    int polarNum{0};//极坐标数
    std::vector<double> polarBounds{};//存储点云中的极径
    std::vector<int> labelInfo{};//标签信息，每个标签是一个聚类

    void searchKNN(
            int& polar_index,
            int& pitch_index,
            int& azimuth_index,
            std::vector<int>& out_neighIndex
    ) const;

    bool createHashTable();
    int getPolarIndex(double &radius);
    void convertToPolar(pcl::PointCloud<pcl::PointXYZ> &cloud_in_);

private:


};
#endif //ALOAM_VELODYNE_ENHANCED_H
