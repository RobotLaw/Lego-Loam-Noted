#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_


#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include "cloud_msgs/cloud_info.h"

#include <opencv/cv.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

#define PI 3.14159265

using namespace std;

typedef pcl::PointXYZI  PointType;  // 三维坐标+反射强度

extern const string pointCloudTopic = "/velodyne_points";   // 接收的点云话题名称
extern const string imuTopic = "/imu/data";     // 接收的imu话题名称

// Save pcd，点云保存路径
extern const string fileDirectory = "/tmp/";

/*  激光雷达原理可参考：https://blog.csdn.net/weixin_42576673/article/details/105240372
    /velodyne_points消息的数据结构为：
    point: 坐标（x,y）
    intensity: 强度(0~255)， 反射为漫反射（Diffuse Reflector）时，反射率的数值为0～100；当反射为逆反射（Retro-Reflector）时，数值为101～255.
    ring: 第几根线(从下至上，0~15)，ring id 与 laser id 并不是一一对应的，具体见launch/VLP-16_id.png
    激光雷达首先会按照一定顺序发射激光线束，垂直方向上是跳跃发射，这样避免邻近的干扰。
*/
// Using velodyne cloud "ring" channel for image projection (other lidar may have different name for this channel, change "PointXYZIR" below)
extern const bool useCloudRing = true; // if true, ang_res_y and ang_bottom are not used

// VLP-16
extern const int N_SCAN = 16;               // 多线激光雷达线数
extern const int Horizon_SCAN = 1800;       // 单线水平扫描点数
extern const float ang_res_x = 0.2;         // 水平方向分辨率
extern const float ang_res_y = 2.0;         // 垂直方向分辨率
extern const float ang_bottom = 15.0+0.1;   // 这里应该是垂直方向的视场角(一半)
extern const int groundScanInd = 7;         // 地面检测索引

// HDL-32E
// extern const int N_SCAN = 32;
// extern const int Horizon_SCAN = 1800;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 41.33/float(N_SCAN-1);
// extern const float ang_bottom = 30.67;
// extern const int groundScanInd = 20;

// VLS-128
// extern const int N_SCAN = 128;
// extern const int Horizon_SCAN = 1800;
// extern const float ang_res_x = 0.2;
// extern const float ang_res_y = 0.3;
// extern const float ang_bottom = 25.0;
// extern const int groundScanInd = 10;

// Ouster users may need to uncomment line 159 in imageProjection.cpp
// Usage of Ouster imu data is not fully supported yet (LeGO-LOAM needs 9-DOF IMU), please just publish point cloud data
// Ouster OS1-16
// extern const int N_SCAN = 16;
// extern const int Horizon_SCAN = 1024;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 33.2/float(N_SCAN-1);
// extern const float ang_bottom = 16.6+0.1;
// extern const int groundScanInd = 7;

// Ouster OS1-64
// extern const int N_SCAN = 64;
// extern const int Horizon_SCAN = 1024;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 33.2/float(N_SCAN-1);
// extern const float ang_bottom = 16.6+0.1;
// extern const int groundScanInd = 15;

extern const bool loopClosureEnableFlag = false;    // 是否开启回环检测标志
extern const double mappingProcessInterval = 0.3;   // 建图过程的时间间隔

extern const float scanPeriod = 0.1;    // 扫描间隔 = 1 / 频率
extern const int systemDelay = 0;       // 系统延时
extern const int imuQueLength = 200;    // imu队列长度

extern const float sensorMinimumRange = 1.0;    // 激光雷达传感器最小测距范围
extern const float sensorMountAngle = 0.0;
extern const float segmentTheta = 60.0/180.0*M_PI; // 60度--->弧度，在imageProjection中用于判断平面，decrese this value may improve accuracy
extern const int segmentValidPointNum = 5;  // 分割有效点数
extern const int segmentValidLineNum = 3;   // 分割有效线数
extern const float segmentAlphaX = ang_res_x / 180.0 * M_PI;    // 分辨率对应的弧度值
extern const float segmentAlphaY = ang_res_y / 180.0 * M_PI;


extern const int edgeFeatureNum = 2;        // 边缘特征点数
extern const int surfFeatureNum = 4;        // 平面特征点数
extern const int sectionsTotal = 6;         // 分区域点总数
extern const float edgeThreshold = 0.1;     // 判断点的阈值c_th
extern const float surfThreshold = 0.1;
extern const float nearestFeatureSearchSqDist = 25; // 特征关联里面近邻特征搜索平方距离


// Mapping Params
extern const float surroundingKeyframeSearchRadius = 50.0; // key frame that is within n meters from current pose will be considerd for scan-to-map optimization (when loop closure disabled)
extern const int   surroundingKeyframeSearchNum = 50; // submap size (when loop closure enabled)
// history key frames (history submap for loop closure)
extern const float historyKeyframeSearchRadius = 7.0; // key frame that is within n meters from current pose will be considerd for loop closure
extern const int   historyKeyframeSearchNum = 25; // 2n+1 number of hostory key frames will be fused into a submap for loop closure
extern const float historyKeyframeFitnessScore = 0.3; // the smaller the better alignment

extern const float globalMapVisualizationSearchRadius = 500.0; // key frames with in n meters will be visualized

// 粗糙度(曲率)
struct smoothness_t{ 
    float value;    // 按照论文公式(1)计算出来的粗糙度值
    size_t ind;     // 当前点在分割点云中的id
};

struct by_value{ 
    bool operator()(smoothness_t const &left, smoothness_t const &right) { 
        return left.value < right.value;
    }
};

/*
    * A point cloud type that has "ring" channel，自定义新的点类型
    */
struct PointXYZIR
{
    PCL_ADD_POINT4D     // XYZ + padding
    PCL_ADD_INTENSITY;  // I
    uint16_t ring;      //ring id 
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 确保new操作符对齐
} EIGEN_ALIGN16;    //强制SSE对齐

// 注册点类型宏
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIR,  
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (uint16_t, ring, ring)
)

/*
    * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
    */
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D     // XYZ + padding
    PCL_ADD_INTENSITY;  // I
    float roll;         // 滚转角
    float pitch;        // 俯仰角
    float yaw;          // 偏航角
    double time;        // 时间
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time)
)

typedef PointXYZIRPYT  PointTypePose;

#endif
