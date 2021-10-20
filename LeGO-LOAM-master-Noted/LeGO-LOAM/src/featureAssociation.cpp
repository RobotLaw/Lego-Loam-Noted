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
//
// This is an implementation of the algorithm described in the following papers:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
//   T. Shan and B. Englot. LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain
//      IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). October 2018.

#include "utility.h"

class FeatureAssociation{

private:

	ros::NodeHandle nh;

    ros::Subscriber subLaserCloud; // 带有地面点的分割点云： 坐标 + 行列索引 (用fullCloud中的点填充的)
    ros::Subscriber subLaserCloudInfo;
    ros::Subscriber subOutlierCloud;
    ros::Subscriber subImu;

    ros::Publisher pubCornerPointsSharp;
    ros::Publisher pubCornerPointsLessSharp;
    ros::Publisher pubSurfPointsFlat;
    ros::Publisher pubSurfPointsLessFlat;

    pcl::PointCloud<PointType>::Ptr segmentedCloud; // 分割点云，去除了原始点云中的离群点以及大部分地面点
    pcl::PointCloud<PointType>::Ptr outlierCloud;   // 界外点云，即离群点

    pcl::PointCloud<PointType>::Ptr cornerPointsSharp;          // 边缘点
    pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp;      // 次边缘点，包含了边缘点
    pcl::PointCloud<PointType>::Ptr surfPointsFlat;             // 平面点(均为地面点)
    pcl::PointCloud<PointType>::Ptr surfPointsLessFlat;         // 次平面点(降采样后)，包含了为地面的平面点

    pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan; // 临时变量
    pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScanDS;

    pcl::VoxelGrid<PointType> downSizeFilter;
    // 如果是同一帧点云的话下面四个时间都是相等的
    double timeScanCur; // 当前帧sweep扫描时间
    double timeNewSegmentedCloud; // 新的分割点云到达时间 = timeScanCur
    double timeNewSegmentedCloudInfo; // 新的点云分割信息到达时间
    double timeNewOutlierCloud; // 新的界外点云到达时间

    bool newSegmentedCloud; // 接收到当前帧新数据的标志位
    bool newSegmentedCloudInfo;
    bool newOutlierCloud;

    cloud_msgs::cloud_info segInfo; // 点云分割信息
    std_msgs::Header cloudHeader; // 等于当前sweep扫描时间

    int systemInitCount; // not used
    bool systemInited; // not used

    std::vector<smoothness_t> cloudSmoothness; // 点云的粗糙度(曲率)
    float *cloudCurvature; // 当前分割点云中特征点的曲率
    int *cloudNeighborPicked; // 点筛选标记：1:筛选过 0:未筛选过
    int *cloudLabel; // 点分类标号:2-代表曲率很大，1-代表曲率比较大,-1-代表曲率很小，0-曲率比较小(其中1包含了2,0包含了1,0和1构成了点云全部的点)

    int imuPointerFront;
    int imuPointerLast; // imu最新收到的消息在数组中的位置，每次接收到imu数据回调时就会更新
    int imuPointerLastIteration;
    float imuRollStart, imuPitchStart, imuYawStart; // 每一帧sweep开始时imu在世界坐标系下的姿态，在对点云进行非匀速矫正时更新

    float cosImuRollStart, cosImuPitchStart, cosImuYawStart, sinImuRollStart, sinImuPitchStart, sinImuYawStart; // 每一帧sweep开始时imu世界姿态对应的正/余弦值
    float imuRollCur, imuPitchCur, imuYawCur; // 点云中选定点对应的imu姿态，位于世界坐标系下,到最后变成点云中最后一个点对应的imu姿态了

    float imuVeloXStart, imuVeloYStart, imuVeloZStart; // 点云sweep初始时刻imu在世界坐标下的速度
    float imuShiftXStart, imuShiftYStart, imuShiftZStart; // 点云sweep初始时刻imu在世界坐标下的位置

    float imuVeloXCur, imuVeloYCur, imuVeloZCur;    // 点云中选定点对应的imu在世界坐标下的速度
    float imuShiftXCur, imuShiftYCur, imuShiftZCur; // 点云中选定点对应的imu在世界坐标下的位移

    float imuShiftFromStartXCur, imuShiftFromStartYCur, imuShiftFromStartZCur; // 点云中选定点相对起始点由于非匀速运动产生的畸变位移，刚开始世界坐标最终转换到点云初始坐标，最终为最后一个点的畸变位移了
    float imuVeloFromStartXCur, imuVeloFromStartYCur, imuVeloFromStartZCur;    // 点云中选定点相对起始点由于非匀速运动产生的畸变速度，刚开始世界坐标最终转换到点云初始坐标，最终为最后一个点的畸变速度了

    float imuAngularRotationXCur, imuAngularRotationYCur, imuAngularRotationZCur; // 每一帧sweep开始时imu角度(在交换坐标轴前的imu坐标系下)
    float imuAngularRotationXLast, imuAngularRotationYLast, imuAngularRotationZLast; // 上一帧sweep开始时的imu角度
    float imuAngularFromStartX, imuAngularFromStartY, imuAngularFromStartZ; // 当前帧sweep开始时距离上一帧imu转动的角度值
    // IMU信息，只在接收到imu回调时更新
    double imuTime[imuQueLength];   // 时间戳
    float imuRoll[imuQueLength];    // 世界坐标下的姿态
    float imuPitch[imuQueLength];
    float imuYaw[imuQueLength];
    // 交换后的左上前imu坐标系(局部)下的加速度，只在接收到imu回调时更新
    float imuAccX[imuQueLength];
    float imuAccY[imuQueLength];
    float imuAccZ[imuQueLength];
    // imu在世界坐标系下的速度，只在接收到imu回调时更新
    float imuVeloX[imuQueLength];
    float imuVeloY[imuQueLength];
    float imuVeloZ[imuQueLength];
    // imu在世界坐标系下的坐标，只在接收到imu回调时更新
    float imuShiftX[imuQueLength];
    float imuShiftY[imuQueLength];
    float imuShiftZ[imuQueLength];
    // imu的三轴角速度(!!!交换前的前左上坐标系下)
    float imuAngularVeloX[imuQueLength];
    float imuAngularVeloY[imuQueLength];
    float imuAngularVeloZ[imuQueLength];
    // imu的三轴角度(!!!交换前的前左上坐标系下)
    float imuAngularRotationX[imuQueLength];
    float imuAngularRotationY[imuQueLength];
    float imuAngularRotationZ[imuQueLength];



    ros::Publisher pubLaserCloudCornerLast;
    ros::Publisher pubLaserCloudSurfLast;
    ros::Publisher pubLaserOdometry;
    ros::Publisher pubOutlierCloudLast;

    int skipFrameNum; // 跳帧数
    bool systemInitedLM;

    int laserCloudCornerLastNum; // 上一帧帧激光雷达点云中边缘特征点(边缘点+次边缘点)数量
    int laserCloudSurfLastNum; // 上一帧激光雷达点云中平面特征点(地面平面点+次平面点)数量

    int *pointSelCornerInd; // not used
    // 与当前帧边缘点相对应的上一帧的两个点，其中索引为当前边缘点在当前点云中的ID，值为对应的上一帧的点在上一帧点云中的ID
    float *pointSearchCornerInd1;
    float *pointSearchCornerInd2;

    int *pointSelSurfInd;           // not used
    // 与当前帧平面点相对应的上一帧的三个点，其中索引为当前平面点在当前点云中的ID，值为对应的上一帧的点在上一帧点云中的ID
    float *pointSearchSurfInd1;     // 当前平面点对应的上一帧点云(平面点+次平面点)中的最近邻点j，其中索引为当前点在点云中的索引，保存的值为上一帧对应点的索引
    float *pointSearchSurfInd2;     // 当前平面点对应的上一帧点云中的最近邻点j同线的近邻点l
    float *pointSearchSurfInd3;     // 当前平面点对应的上一帧点云中的最近邻点j相邻线的近邻点m
    // 为什么姿态是这样的顺序???对应的变换后的坐标轴 ///Q 姿态是当前帧相对于上一帧的R_1_2，位移是上一帧相对于当前帧的t_2_1(之前的笔记)
    float transformCur[6]; // [pitch,yaw,roll,tx,ty,tz] 上一帧相对于当前帧的位姿变换T_2_1(其实就是点云开始相对点云结束坐标系变换，注意此时是以点云结束为世界坐标系的T_end_start)
    float transformSum[6]; // [pitch,yaw,roll,tx,ty,tz] 当前帧相对于第一帧的位姿变换，位于世界坐标下 T_w_l

    float imuRollLast, imuPitchLast, imuYawLast; // 每一帧sweep点云结束(最后一个点)时imu在世界坐标系下的姿态
    float imuShiftFromStartX, imuShiftFromStartY, imuShiftFromStartZ; // 点云最后一个点相对于第一个点由于加减速产生的畸变位移，在点云初始start坐标系下
    float imuVeloFromStartX, imuVeloFromStartY, imuVeloFromStartZ;    // 点云最后一个点相对于第一个点由于加减速产生的畸变速度，在点云初始start坐标系下

    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;   // 上一帧的边缘特征点信息
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;     // 上一帧的边缘平面特征点信息
    pcl::PointCloud<PointType>::Ptr laserCloudOri; // 保存未经处理(畸变校正)过的特征点，位于ti时刻坐标系下
    pcl::PointCloud<PointType>::Ptr coeffSel;      // 保存带有权重的单位法向量和点到线或平面的距离值

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerLast;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfLast;

    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;

    PointType pointOri, pointSel, tripod1, tripod2, tripod3, pointProj, coeff;

    nav_msgs::Odometry laserOdometry;

    tf::TransformBroadcaster tfBroadcaster;
    tf::StampedTransform laserOdometryTrans;

    bool isDegenerate; // 退化标志
    cv::Mat matP;

    int frameCount;

public:

    FeatureAssociation():
        nh("~")
        {
        // 包含了地面点的分割点云
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/segmented_cloud", 1, &FeatureAssociation::laserCloudHandler, this);
        subLaserCloudInfo = nh.subscribe<cloud_msgs::cloud_info>("/segmented_cloud_info", 1, &FeatureAssociation::laserCloudInfoHandler, this);
        subOutlierCloud = nh.subscribe<sensor_msgs::PointCloud2>("/outlier_cloud", 1, &FeatureAssociation::outlierCloudHandler, this);
        subImu = nh.subscribe<sensor_msgs::Imu>(imuTopic, 50, &FeatureAssociation::imuHandler, this);

        pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 1);
        pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 1);
        pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 1);
        pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 1);

        pubLaserCloudCornerLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 2);
        pubLaserCloudSurfLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 2);
        pubOutlierCloudLast = nh.advertise<sensor_msgs::PointCloud2>("/outlier_cloud_last", 2);
        pubLaserOdometry = nh.advertise<nav_msgs::Odometry> ("/laser_odom_to_init", 5);
        
        initializationValue();
    }

    void initializationValue() // 各类参数的初始化
    {
        cloudCurvature = new float[N_SCAN*Horizon_SCAN]; ///Q 分割点云中的点数其实远远小于这个值，是否可以考虑缩小数组大小以缩小内存占用
        cloudNeighborPicked = new int[N_SCAN*Horizon_SCAN];
        cloudLabel = new int[N_SCAN*Horizon_SCAN];

        pointSelCornerInd = new int[N_SCAN*Horizon_SCAN];
        pointSearchCornerInd1 = new float[N_SCAN*Horizon_SCAN];
        pointSearchCornerInd2 = new float[N_SCAN*Horizon_SCAN];

        pointSelSurfInd = new int[N_SCAN*Horizon_SCAN];
        pointSearchSurfInd1 = new float[N_SCAN*Horizon_SCAN];
        pointSearchSurfInd2 = new float[N_SCAN*Horizon_SCAN];
        pointSearchSurfInd3 = new float[N_SCAN*Horizon_SCAN];

        cloudSmoothness.resize(N_SCAN*Horizon_SCAN);

        downSizeFilter.setLeafSize(0.2, 0.2, 0.2);

        segmentedCloud.reset(new pcl::PointCloud<PointType>());
        outlierCloud.reset(new pcl::PointCloud<PointType>());

        cornerPointsSharp.reset(new pcl::PointCloud<PointType>());
        cornerPointsLessSharp.reset(new pcl::PointCloud<PointType>());
        surfPointsFlat.reset(new pcl::PointCloud<PointType>());
        surfPointsLessFlat.reset(new pcl::PointCloud<PointType>());

        surfPointsLessFlatScan.reset(new pcl::PointCloud<PointType>());
        surfPointsLessFlatScanDS.reset(new pcl::PointCloud<PointType>());

        timeScanCur = 0;
        timeNewSegmentedCloud = 0;
        timeNewSegmentedCloudInfo = 0;
        timeNewOutlierCloud = 0;

        newSegmentedCloud = false;
        newSegmentedCloudInfo = false;
        newOutlierCloud = false;

        systemInitCount = 0;
        systemInited = false;

        imuPointerFront = 0;
        imuPointerLast = -1;
        imuPointerLastIteration = 0;

        imuRollStart = 0; imuPitchStart = 0; imuYawStart = 0;
        cosImuRollStart = 0; cosImuPitchStart = 0; cosImuYawStart = 0;
        sinImuRollStart = 0; sinImuPitchStart = 0; sinImuYawStart = 0;
        imuRollCur = 0; imuPitchCur = 0; imuYawCur = 0;

        imuVeloXStart = 0; imuVeloYStart = 0; imuVeloZStart = 0;
        imuShiftXStart = 0; imuShiftYStart = 0; imuShiftZStart = 0;

        imuVeloXCur = 0; imuVeloYCur = 0; imuVeloZCur = 0;
        imuShiftXCur = 0; imuShiftYCur = 0; imuShiftZCur = 0;

        imuShiftFromStartXCur = 0; imuShiftFromStartYCur = 0; imuShiftFromStartZCur = 0;
        imuVeloFromStartXCur = 0; imuVeloFromStartYCur = 0; imuVeloFromStartZCur = 0;

        imuAngularRotationXCur = 0; imuAngularRotationYCur = 0; imuAngularRotationZCur = 0;
        imuAngularRotationXLast = 0; imuAngularRotationYLast = 0; imuAngularRotationZLast = 0;
        imuAngularFromStartX = 0; imuAngularFromStartY = 0; imuAngularFromStartZ = 0;

        for (int i = 0; i < imuQueLength; ++i)
        {
            imuTime[i] = 0;
            imuRoll[i] = 0; imuPitch[i] = 0; imuYaw[i] = 0;
            imuAccX[i] = 0; imuAccY[i] = 0; imuAccZ[i] = 0;
            imuVeloX[i] = 0; imuVeloY[i] = 0; imuVeloZ[i] = 0;
            imuShiftX[i] = 0; imuShiftY[i] = 0; imuShiftZ[i] = 0;
            imuAngularVeloX[i] = 0; imuAngularVeloY[i] = 0; imuAngularVeloZ[i] = 0;
            imuAngularRotationX[i] = 0; imuAngularRotationY[i] = 0; imuAngularRotationZ[i] = 0;
        }


        skipFrameNum = 1;

        for (int i = 0; i < 6; ++i){
            transformCur[i] = 0;
            transformSum[i] = 0;
        }

        systemInitedLM = false;

        imuRollLast = 0; imuPitchLast = 0; imuYawLast = 0;
        imuShiftFromStartX = 0; imuShiftFromStartY = 0; imuShiftFromStartZ = 0;
        imuVeloFromStartX = 0; imuVeloFromStartY = 0; imuVeloFromStartZ = 0;

        laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());
        laserCloudOri.reset(new pcl::PointCloud<PointType>());
        coeffSel.reset(new pcl::PointCloud<PointType>());

        kdtreeCornerLast.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeSurfLast.reset(new pcl::KdTreeFLANN<PointType>());

        laserOdometry.header.frame_id = "/camera_init";
        laserOdometry.child_frame_id = "/laser_odom";

        laserOdometryTrans.frame_id_ = "/camera_init";
        laserOdometryTrans.child_frame_id_ = "/laser_odom";
        
        isDegenerate = false;
        matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));

        frameCount = skipFrameNum;
    }

    void updateImuRollPitchYawStartSinCos(){
        cosImuRollStart = cos(imuRollStart);
        cosImuPitchStart = cos(imuPitchStart);
        cosImuYawStart = cos(imuYawStart);
        sinImuRollStart = sin(imuRollStart);
        sinImuPitchStart = sin(imuPitchStart);
        sinImuYawStart = sin(imuYawStart);
    }

    // 将Lidar位移转到IMU起始坐标系下//计算局部坐标系下点云中的点相对第一个开始点的由于加减速运动产生的位移畸变
    void ShiftToStartIMU(float pointTime)
    {
        //计算相对于第一个点由于加减速产生的畸变位移(全局坐标系下畸变位移量delta_Tg)
        /*  为什么这样计算???这里假设点云在整个出点(即一个sweep)的过程中，机器人都是匀速运动的，即以第一个点的速度做匀速运动，
            所以后面每个点对应的imu世界坐标理想情况下应该为： sn = s0 + v0 * pointTime;
            所以实际对应的imu世界坐标减去理想的坐标就得到了运动的畸变，即 sn' - sn
        */
        //imuShiftFromStartCur = imuShiftCur - (imuShiftStart + imuVeloStart * pointTime)
        imuShiftFromStartXCur = imuShiftXCur - imuShiftXStart - imuVeloXStart * pointTime;
        imuShiftFromStartYCur = imuShiftYCur - imuShiftYStart - imuVeloYStart * pointTime;
        imuShiftFromStartZCur = imuShiftZCur - imuShiftZStart - imuVeloZStart * pointTime;

        /********************************************************************************
         R_b_n = R_ZYX(左乘) = Rx(roll)*Ry(pitch)*Rz(yaw) 这个是未交换轴前的顺序
        其中，Rx =  |   1       0         0      |  世界坐标系转换到局部坐标系用这个旋转矩阵
                    |  0   cos(phi)   sin(phi)   |
                    |  0   -sin(phi)  cos(phi)   |
        Rz(pitch).inverse * Rx(pitch).inverse * Ry(yaw).inverse * delta_Tg，这句话其实就是(RyRxRz)^-1
        transfrom from the global frame to the local frame，接下来要将这个畸变从世界坐标系转换到sweep初始时刻的imu坐标系(即点云初始坐标系下)
        *********************************************************************************/
        //绕y轴旋转(-imuYawStart)，即Ry(yaw).inverse
        float x1 = cosImuYawStart * imuShiftFromStartXCur - sinImuYawStart * imuShiftFromStartZCur;
        float y1 = imuShiftFromStartYCur;
        float z1 = sinImuYawStart * imuShiftFromStartXCur + cosImuYawStart * imuShiftFromStartZCur;

        //绕x轴旋转(-imuPitchStart)，即Rx(pitch).inverse
        float x2 = x1;
        float y2 = cosImuPitchStart * y1 + sinImuPitchStart * z1;
        float z2 = -sinImuPitchStart * y1 + cosImuPitchStart * z1;

        //绕z轴旋转(-imuRollStart)，即Rz(pitch).inverse
        imuShiftFromStartXCur = cosImuRollStart * x2 + sinImuRollStart * y2;
        imuShiftFromStartYCur = -sinImuRollStart * x2 + cosImuRollStart * y2;
        imuShiftFromStartZCur = z2;
    }

    //计算局部坐标系下点云中的点相对第一个开始点由于加减速产生的的速度畸变（增量）
    void VeloToStartIMU()
    {
        //计算相对于第一个点由于加减速产生的畸变速度(全局坐标系下畸变速度增量delta_Vg)
        // 同理，假设一个sweep中机器人做匀速运动，其速度为第一个点的速度        
        // imuVeloXStart,imuVeloYStart,imuVeloZStart是点云索引i=0时刻的速度
        // 此处计算的是相对于初始时刻i=0时的相对速度， 这个相对速度在世界坐标系下
        imuVeloFromStartXCur = imuVeloXCur - imuVeloXStart;
        imuVeloFromStartYCur = imuVeloYCur - imuVeloYStart;
        imuVeloFromStartZCur = imuVeloZCur - imuVeloZStart;
        /********************************************************************************
            R_b_n = R_ZYX(左乘) = Rx(roll)*Ry(pitch)*Rz(yaw) 这个是未交换轴前的顺序
            Rz(pitch).inverse * Rx(pitch).inverse * Ry(yaw).inverse * delta_Vg
            transfrom from the global frame to the local frame
        *********************************************************************************/
        // ！！！下面从世界坐标系转换到点云初始start坐标系，roll,pitch,yaw要取负值
        // 首先绕y轴进行旋转
        //    |cosry   0  —sinry|
        // Ry=|0       1       0|  世界坐标转换到局部坐标，用这个旋转矩阵
        //    |sinry   0   cosry|
        float x1 = cosImuYawStart * imuVeloFromStartXCur - sinImuYawStart * imuVeloFromStartZCur;
        float y1 = imuVeloFromStartYCur;
        float z1 = sinImuYawStart * imuVeloFromStartXCur + cosImuYawStart * imuVeloFromStartZCur;

        // 绕当前x轴旋转(-pitch)的角度
        //    |1     0        0|
        // Rx=|0   cosrx  sinrx|
        //    |0  -sinrx  cosrx|
        float x2 = x1;
        float y2 = cosImuPitchStart * y1 + sinImuPitchStart * z1;
        float z2 = -sinImuPitchStart * y1 + cosImuPitchStart * z1;

        // 绕当前z轴旋转(-roll)的角度
        //     |cosrz   sinrz  0|
        //  Rz=|-sinrz cosrz   0|
        //     |0       0      1|
        imuVeloFromStartXCur = cosImuRollStart * x2 + sinImuRollStart * y2;
        imuVeloFromStartYCur = -sinImuRollStart * x2 + cosImuRollStart * y2;
        imuVeloFromStartZCur = z2;
    }

    // 该函数的功能是把点云坐标变换到点云初始时刻，去除点云加减速产生的位移畸变
    // 先将选定点(激光雷达坐标系下，但不是初始坐标系下)转换到其对应的世界坐标下，然后再转换到点云初始坐标系下，再去除掉由于加减速产生的位移畸变即可
    void TransformToStartIMU(PointType *p)
    {
        /********************************************************************************
            (变换后的轴)Ry(yaw)*Rx(pitch)*Rz(roll)*Pl,原始R_n_b = Rz(yaw)*Ry(pitch)*Rx(roll), 
            transform point to the global frame，利用选定点当前的位姿将其从激光雷达坐标系转换到世界坐标系!!!
        *********************************************************************************/
        // 因为在adjustDistortion函数中有对xyz的坐标进行交换的过程
        // 交换的过程是x=原来的y，y=原来的z，z=原来的x
        // 所以下面其实是绕Z轴(原先的x轴)旋转，对应的是roll角
        //
        //     |cosrz  -sinrz  0|
        //  Rz=|sinrz  cosrz   0|  局部坐标转到到世界坐标，用这个旋转矩阵
        //     |0       0      1|
        // [x1,y1,z1]^T=Rz*[x,y,z]
        //
        // 因为在imuHandler中进行过坐标变换，
        // 所以下面的roll其实已经对应于新坐标系中(X-Y-Z)的yaw
        // 绕z轴旋转(imuRollCur)
        float x1 = cos(imuRollCur) * p->x - sin(imuRollCur) * p->y;
        float y1 = sin(imuRollCur) * p->x + cos(imuRollCur) * p->y;
        float z1 = p->z;

        // 绕X轴(原先的y轴)旋转(imuPitchCur)
        // 
        // [x2,y2,z2]^T=Rx*[x1,y1,z1]
        //    |1     0        0|
        // Rx=|0   cosrx -sinrx|
        //    |0   sinrx  cosrx|
        float x2 = x1;
        float y2 = cos(imuPitchCur) * y1 - sin(imuPitchCur) * z1;
        float z2 = sin(imuPitchCur) * y1 + cos(imuPitchCur) * z1;

        // 最后再绕Y轴(原先的Z轴)旋转(imuYawCur)
        //    |cosry   0   sinry|
        // Ry=|0       1       0|
        //    |-sinry  0   cosry|
        float x3 = cos(imuYawCur) * x2 + sin(imuYawCur) * z2;
        float y3 = y2;
        float z3 = -sin(imuYawCur) * x2 + cos(imuYawCur) * z2;

        /********************************************************************************
            Rz(pitch).inverse * Rx(pitch).inverse * Ry(yaw).inverse * Pg
            transfrom global points to the local frame，将位于世界坐标下的激光选定点数据转换到第一个点对应的imu坐标系下(即点云初始坐标系下)
        *********************************************************************************/
        // 下面部分的代码功能是从imu坐标的原点变换到i=0时imu的初始时刻(从世界坐标系变换到start坐标系)
        // 变换方式和函数VeloToStartIMU()中的类似
        // 变换顺序：Cur-->世界坐标系-->Start，这两次变换中，
        // 前一次是正变换，角度为正，后一次是逆变换，角度应该为负 ////其实就是两次旋转的旋转矩阵不一样而已
        // 可以参考：
        // https://blog.csdn.net/wykxwyc/article/details/101712524
        //绕y轴旋转(-imuYawStart)
        float x4 = cosImuYawStart * x3 - sinImuYawStart * z3;
        float y4 = y3;
        float z4 = sinImuYawStart * x3 + cosImuYawStart * z3;

        //绕x轴旋转(-imuPitchStart)
        float x5 = x4;
        float y5 = cosImuPitchStart * y4 + sinImuPitchStart * z4;
        float z5 = -sinImuPitchStart * y4 + cosImuPitchStart * z4;

        //绕z轴旋转(-imuRollStart)，然后叠加平移量
        // 绕z轴(原先的x轴)变换角度到初始imu时刻，另外需要加上imu的位移漂移
        // 后面加上的 imuShiftFromStart.. 表示从start时刻到cur时刻的漂移，(imuShiftFromStart.. 在start坐标系下)
        p->x = cosImuRollStart * x5 + sinImuRollStart * y5 + imuShiftFromStartXCur;
        p->y = -sinImuRollStart * x5 + cosImuRollStart * y5 + imuShiftFromStartYCur;
        p->z = z5 + imuShiftFromStartZCur;
    }

    // 世界坐标系下积分速度与位移，与交换前前左上坐标系下imu转动的角度
    void AccumulateIMUShiftAndRotation()
    {
        float roll = imuRoll[imuPointerLast]; // 世界坐标下的姿态，最新imu数据
        float pitch = imuPitch[imuPointerLast];
        float yaw = imuYaw[imuPointerLast];
        float accX = imuAccX[imuPointerLast]; // 交换后的左上前imu坐标系(局部)下的加速度(已经去除了重力加速度的影响)
        float accY = imuAccY[imuPointerLast];
        float accZ = imuAccZ[imuPointerLast];

        // a_g = R_g_i * a_i 正常来讲 R_g_i = Rz(yaw)*Ry(pitch)*Rx(roll)
        //将当前时刻的加速度值(交换过的)绕交换过的ZXY固定轴（原XYZ）分别旋转(roll, pitch, yaw)角，转换得到世界坐标系下的加速度值(right hand rule)
        // 因为交换过后RPY对应fixed axes ZXY(RPY---ZXY)，Now R_n_b = R_ZXY = Ry(yaw)*Rx(pitch)*Rz(roll).
        // 而且因为现在加速度是交换过后的值，所以要求世界坐标下的值的话按需要照交换前的XYZ顺序来进行转换处理
        // 这里，由于R_g_i的旋转是按照imu坐标系的轴进行旋转的，也即绕旋转后的轴进行旋转
        
        // 先绕Z轴(原x轴)旋转,下方坐标系示意imuHandler()中加速度的坐标轴交换
        //  z->Y
        //  ^  
        //  |    ^ y->X
        //  |   /
        //  |  /
        //  | /
        //  -----> x->Z
        //
        //     |cosrz  -sinrz  0|
        //  Rz=|sinrz  cosrz   0|   局部坐标系到世界坐标系的位姿变换
        //     |0       0      1|
        // [x1,y1,z1]^T=Rz*[accX,accY,accZ]
        // 因为在imuHandler中进行过坐标变换，所以下面的roll其实已经对应于新坐标系中(X-Y-Z)的yaw
        float x1 = cos(roll) * accX - sin(roll) * accY;
        float y1 = sin(roll) * accX + cos(roll) * accY;
        float z1 = accZ;

        // 绕X轴(原y轴)旋转
        // [x2,y2,z2]^T=Rx*[x1,y1,z1]
        //    |1     0        0|
        // Rx=|0   cosrx -sinrx|
        //    |0   sinrx  cosrx|
        float x2 = x1;
        float y2 = cos(pitch) * y1 - sin(pitch) * z1;
        float z2 = sin(pitch) * y1 + cos(pitch) * z1;

        // 最后再绕Y轴(原z轴)旋转
        //    |cosry   0   sinry|
        // Ry=|0       1       0|
        //    |-sinry  0   cosry|
        accX = cos(yaw) * x2 + sin(yaw) * z2;
        accY = y2;
        accZ = -sin(yaw) * x2 + cos(yaw) * z2;

        // 最新imu数据的上一个imu点对应数组位置
        int imuPointerBack = (imuPointerLast + imuQueLength - 1) % imuQueLength; // 加imuQueLength是为了防止imuPointerLast为0时产生误解
        // 上一个点到当前最新点所经历的时间，即计算imu测量周期
        double timeDiff = imuTime[imuPointerLast] - imuTime[imuPointerBack];
        //要求imu的频率至少比lidar高，这样的imu信息才使用，后面校正也才有意义
        if (timeDiff < scanPeriod) { //（隐含从静止开始运动）
            //求每个imu时间点的位移(相对于世界坐标原点)与速度(均在世界坐标下),两点之间视为匀/减加速直线运动，类似于惯性导航的样子!!!
            // IMU第一帧数据时，上一点(虚假的上一点，并不是真的上一个点)的速度，位移都为0
            imuShiftX[imuPointerLast] = imuShiftX[imuPointerBack] + imuVeloX[imuPointerBack] * timeDiff + accX * timeDiff * timeDiff / 2;
            imuShiftY[imuPointerLast] = imuShiftY[imuPointerBack] + imuVeloY[imuPointerBack] * timeDiff + accY * timeDiff * timeDiff / 2;
            imuShiftZ[imuPointerLast] = imuShiftZ[imuPointerBack] + imuVeloZ[imuPointerBack] * timeDiff + accZ * timeDiff * timeDiff / 2;
            //当前时刻imu在世界坐标下的速度
            imuVeloX[imuPointerLast] = imuVeloX[imuPointerBack] + accX * timeDiff;
            imuVeloY[imuPointerLast] = imuVeloY[imuPointerBack] + accY * timeDiff;
            imuVeloZ[imuPointerLast] = imuVeloZ[imuPointerBack] + accZ * timeDiff;
            // 当前imu绕轴旋转过的角度(在交换前的前左上坐标系下) /////Q 这里后面应该是当前时刻点的角速度
            imuAngularRotationX[imuPointerLast] = imuAngularRotationX[imuPointerBack] + imuAngularVeloX[imuPointerBack] * timeDiff;
            imuAngularRotationY[imuPointerLast] = imuAngularRotationY[imuPointerBack] + imuAngularVeloY[imuPointerBack] * timeDiff;
            imuAngularRotationZ[imuPointerLast] = imuAngularRotationZ[imuPointerBack] + imuAngularVeloZ[imuPointerBack] * timeDiff;
        }
    }

    // 接收imu消息，imu安装的坐标系为x轴向前，y轴向左，z轴向上的右手坐标系(与激光雷达的安装坐标系相同)
    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn)
    {
        //This will get the roll pitch and yaw from the matrix about fixed axes X, Y, Z respectively. 
        //Here roll pitch yaw is in the global frame，
        double roll, pitch, yaw;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(imuIn->orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw); // 全局坐标系下的姿态，这里IMU是用的一个现有的模块，可以直接输出姿态角度信息。
        
        /*
            旋转矩阵小结：
            如果按照ZYX的航空旋转次序的话，
            R_b_n = R_ZYX(左乘) = Rx(roll)*Ry(pitch)*Rz(yaw) 表示从导航坐标系(世界坐标系)到本体坐标系(局部坐标系)的旋转矩阵，(外旋)
            其中，Rx = |   1       0         0      |
                       |  0   cos(phi)   sin(phi)   |
                       |  0   -sin(phi)  cos(phi)   |
            R_n_b = R_ZYX(右乘) = Rz(yaw)*Ry(pitch)*Rx(roll) 表示从本体坐标系(局部坐标系)到导航坐标系(世界坐标系)的旋转矩阵，(内旋)
            也可以表示为b系相对于n系的姿态
            其中，Rx = |   1       0         0       |
                       |  0   cos(phi)   -sin(phi)   |
                       |  0   sin(phi)   cos(phi)    |
        
                           |  cycz          cysz        -sy   |
            R_ZYX(左乘) =  |  sxsycz-cxsz  sxsysz+cxcz  sxcy  | = R_ZYX(右乘)^T
                           |  cxsycz+sxsz  cxsysz-sxcz  cxcy  |

        */ 
        // 这里，其实隐含了两个步骤：
        // (1) IMU的测量值是一个比力，有重力的影响，需要先减去这个重力因素的影响；a_true = a_measure - R_ZYX(左乘) * g（R_ZYX：ZYX航空旋转次序，即R_b_n)
        // (2) 因为imu坐标系(前左上)与雷达坐标系(左上前)的方向不同，所以需要把他们的值做一个简单的交换即可。
        // 具体见知乎  https://zhuanlan.zhihu.com/p/263090394 ，这里R_ZYX的作用是把世界坐标的重力转换到当前的imu坐标下
        //减去重力的影响,求出xyz方向的加速度实际值，并进行坐标轴交换，统一到z轴向前,x轴向左，y轴向上的右手坐标系(imu局部坐标系), 
        // 这里这样设置坐标轴交换的目的是为了与相机坐标系一致，方便以后进行视觉的融合
        //交换过后RPY对应fixed axes ZXY(RPY---ZXY)。
        float accX = imuIn->linear_acceleration.y - sin(roll) * cos(pitch) * 9.81;
        float accY = imuIn->linear_acceleration.z - cos(roll) * cos(pitch) * 9.81;
        float accZ = imuIn->linear_acceleration.x + sin(pitch) * 9.81;
        
        //循环移位效果，形成环形数组  范围0~200
        imuPointerLast = (imuPointerLast + 1) % imuQueLength;
        
        imuTime[imuPointerLast] = imuIn->header.stamp.toSec();

        imuRoll[imuPointerLast] = roll; // 当前时刻imu的全局姿态，类似于R_w_i
        imuPitch[imuPointerLast] = pitch;
        imuYaw[imuPointerLast] = yaw;

        // 当前时刻imu去掉重力后的三轴加速度(交换后的左上前坐标系下，即局部坐标系，注意这里不是世界坐标系)
        imuAccX[imuPointerLast] = accX;
        imuAccY[imuPointerLast] = accY;
        imuAccZ[imuPointerLast] = accZ;

        // 当前时刻imu的三轴角速度(!!!交换前的前左上坐标系下，即局部坐标系，注意这里不是世界坐标系)
        imuAngularVeloX[imuPointerLast] = imuIn->angular_velocity.x;
        imuAngularVeloY[imuPointerLast] = imuIn->angular_velocity.y;
        imuAngularVeloZ[imuPointerLast] = imuIn->angular_velocity.z;

        AccumulateIMUShiftAndRotation(); // 世界坐标系下的积分速度与位移 和 交换前坐标系下imu绕各轴转动的角度
    }

    void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

        cloudHeader = laserCloudMsg->header;

        timeScanCur = cloudHeader.stamp.toSec();
        timeNewSegmentedCloud = timeScanCur;

        segmentedCloud->clear();
        pcl::fromROSMsg(*laserCloudMsg, *segmentedCloud);

        newSegmentedCloud = true;
    }

    void outlierCloudHandler(const sensor_msgs::PointCloud2ConstPtr& msgIn){

        timeNewOutlierCloud = msgIn->header.stamp.toSec();

        outlierCloud->clear();
        pcl::fromROSMsg(*msgIn, *outlierCloud);

        newOutlierCloud = true;
    }

    void laserCloudInfoHandler(const cloud_msgs::cloud_infoConstPtr& msgIn)
    {
        timeNewSegmentedCloudInfo = msgIn->header.stamp.toSec();
        segInfo = *msgIn;
        newSegmentedCloudInfo = true;
    }

    void adjustDistortion()
    {
        bool halfPassed = false;
        // 分割点云中去除了原始点云中的离群点以及大部分地面点，该数量远远小于原始点云的点数(16*1800 = 28800)
        int cloudSize = segmentedCloud->points.size();

        PointType point;

        for (int i = 0; i < cloudSize; i++) {
            // 坐标轴交换，安装好的velodyne lidar的坐标系(前左上)转换到z轴向前，x轴向左，y轴向上(左上前)的右手坐标系
            point.x = segmentedCloud->points[i].y;
            point.y = segmentedCloud->points[i].z;
            point.z = segmentedCloud->points[i].x;

            // 因为转一圈可能会超过2pi， 故角度a可能对应a或者2pi + a
            // 如何确定是a还是2pi+a呢， half_passed 利用点的顺序与时间先后相关这一点解决了这个问题
            // 见https://zhuanlan.zhihu.com/p/57351961 解析

            /******以下代码均在左上前的右手坐标系进行的******/

            // ori表示的是偏航角yaw，因为前面有负号，ori=[-M_PI,M_PI)
            // 因为segInfo.orientationDiff表示的范围是(PI,3PI)，在2PI附近
            // 下面过程的主要作用是调整ori大小，满足start<ori<end
            float ori = -atan2(point.x, point.z); // 添加负号主要是因为雷达是顺时针转动的，要转化为逆时针，结果为当前点与交换后的Z轴正方向的夹角
            if (!halfPassed) { // 根据扫描线是否旋转过半选择与起始位置还是终止位置进行差值计算，从而进行补偿
                //确保-pi/2 < ori - startOri < 3*pi/2
                if (ori < segInfo.startOrientation - M_PI / 2)
					// start-ori>M_PI/2，说明ori小于start，不合理，
					// 正常情况在前半圈的话，ori-stat范围[0,M_PI]
                    ori += 2 * M_PI;
                else if (ori > segInfo.startOrientation + M_PI * 3 / 2)
                    ori -= 2 * M_PI; // ori-start>3/2*M_PI,说明ori太大，不合理

                if (ori - segInfo.startOrientation > M_PI) //半个周期
                    halfPassed = true;
            } else {
                ori += 2 * M_PI;
                //确保-3*pi/2 < ori - endOri < pi/2
                if (ori < segInfo.endOrientation - M_PI * 3 / 2)
                    ori += 2 * M_PI; // end-ori>3/2*PI,ori太小
                else if (ori > segInfo.endOrientation + M_PI / 2)
                    ori -= 2 * M_PI; // ori-end>M_PI/2,太大
            }
            
            //-0.5 < relTime < 1.5（点旋转的角度与整个周期旋转角度的比率, 即点云中点的相对时间）// 根据水平角度可以得到获取每个点时相对于开始点的时间relTime
            float relTime = (ori - segInfo.startOrientation) / segInfo.orientationDiff;
            // 强度值存储的是当前点在投影过后的距离图像中的行列索引Point.intensity = (float)rowIdn + (float)columnIdn / 10000.0;
            // int(segmentedCloud->points[i].intensity)得到当前点在距离图像中的行号，也就是对应的线号(ring值)
            // 对点云数据格式信息的利用达到最大化：对应线号id + 相对开始点的时间，一个数据包含了两个信息！！
            //点强度=线号+点相对时间（即一个整数+一个小数，整数部分是线号，小数部分是该点的相对时间）,匀速扫描：根据当前扫描的角度和扫描周期计算相对扫描起始位置的时间
            point.intensity = int(segmentedCloud->points[i].intensity) + scanPeriod * relTime;

            //相对时间relTime用来和IMU数据一起近似去除激光的非匀速运动，构建匀速运动模型(即假设激光雷达在一次sweep期间都是匀速运动的)。
            //当前点时间 = 点云开始时间 + 周期时间(点在一次sweep周期中的相对时间) ，imuPointerLast 是imu当前最新点，变量只在imu中改变
            if (imuPointerLast >= 0) { // 如果收到IMU数据,则使用IMU矫正点云在一次sweep过程中因为加减速产生的畸变
                float pointTime = relTime * scanPeriod; //计算点的周期时间(其实就是当前点相对于点云起始点的时间)
                imuPointerFront = imuPointerLastIteration; // 增加这个变量是为了快速找到与当前点云相对应的imu时间戳
                // while循环内进行时间轴对齐，产生两种结果： 
                // 1.imu数据充足，找到点在点云中时间最近的一个imu时间，走break流程退出，接下来可利用imu数据进行插值等工作；
                // 2.imu数据不够，没有找到imu时刻，循环条件不满足退出，此时imuPointerFront = imuPointerLast，无法利用imu数据进行插值。
                while (imuPointerFront != imuPointerLast) { //寻找是否有点云的时间戳刚好小于IMU的时间戳的IMU位置:imuPointerFront
                    if (timeScanCur + pointTime < imuTime[imuPointerFront]) { //找到ti(点云中该点的时间)后的最近一个imu时刻，此时imu数据充足
                        break;
                    }
                    imuPointerFront = (imuPointerFront + 1) % imuQueLength;
                }

                // 这里是直接把imu的位姿，速度等拿来当做当前lidar的位姿等，没有像vins等框架使用外参计算出lidar真正的位姿，速度等。
                //timeScanCur + pointTime是ti时刻(第i个点扫描的时间;imuPointerFront是ti后一时刻的imu时间,imuPointerBack是ti前一时刻的imu时间
                if (timeScanCur + pointTime > imuTime[imuPointerFront]) {//上面while循环没找到合适的imu时间,只能以当前收到的最新的IMU的速度，位移，欧拉角作为当前点的速度，位移，欧拉角使用
                    // 该条件内imu数据比激光数据早，但是没有更后面的数据
                    // (打个比方,激光在9点时出现，imu现在只有8点的)
                    // 这种情况上面while循环是以imuPointerFront == imuPointerLast结束的
                    imuRollCur = imuRoll[imuPointerFront]; // 点云中选定点对应当前最新时刻的imu的世界姿态，imu模块直接输出的结果
                    imuPitchCur = imuPitch[imuPointerFront];
                    imuYawCur = imuYaw[imuPointerFront];

                    imuVeloXCur = imuVeloX[imuPointerFront]; // 选定点对应当前最新时刻imu在世界坐标下的速度
                    imuVeloYCur = imuVeloY[imuPointerFront];
                    imuVeloZCur = imuVeloZ[imuPointerFront];

                    imuShiftXCur = imuShiftX[imuPointerFront]; // 选定点对应当前最新时刻imu在世界坐标下的位移
                    imuShiftYCur = imuShiftY[imuPointerFront];
                    imuShiftZCur = imuShiftZ[imuPointerFront];   
                } else {//找到了点云时间戳小于IMU时间戳的IMU位置,则该点必处于imuPointerBack和imuPointerFront之间，据此线性插值，计算点云点的速度，位移和欧拉角
                    // 在imu数据充足的情况下可以进行插补， 当前timeScanCur + pointTime < imuTime[imuPointerFront]，
                    // 而且imuPointerFront是最早一个时间大于timeScanCur + pointTime的imu数据指针
                    //按时间距离计算权重分配比率,也即线性插值
                    int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
                    float ratioFront = (timeScanCur + pointTime - imuTime[imuPointerBack]) 
                                                     / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
                    float ratioBack = (imuTime[imuPointerFront] - timeScanCur - pointTime) 
                                                    / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);

                    // 通过上面计算的ratioFront以及ratioBack进行插补
                    // 因为imuRollCur和imuPitchCur通常都在0度左右，变化不会很大，因此不需要考虑超过2M_PI的情况
                    // imuYaw转的角度比较大，需要考虑超过2*M_PI的情况
                    imuRollCur = imuRoll[imuPointerFront] * ratioFront + imuRoll[imuPointerBack] * ratioBack;// 用ti时间前后的两个imu数据进行插值
                    imuPitchCur = imuPitch[imuPointerFront] * ratioFront + imuPitch[imuPointerBack] * ratioBack;
                    if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] > M_PI) {// 个人理解这里是对imu两个时间跨越正负pi的角度作处理，因为偏航角范围为[-pi,+pi]
                        imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] + 2 * M_PI) * ratioBack;
                    } else if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] < -M_PI) {
                        imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] - 2 * M_PI) * ratioBack;
                    } else {
                        imuYawCur = imuYaw[imuPointerFront] * ratioFront + imuYaw[imuPointerBack] * ratioBack;
                    }
                    
                    // imu速度插补 //本质:imuVeloXCur = imuVeloX[imuPointerback] + (imuVelX[imuPointerFront]-imuVelX[imuPoniterBack])*ratioFront
                    imuVeloXCur = imuVeloX[imuPointerFront] * ratioFront + imuVeloX[imuPointerBack] * ratioBack;
                    imuVeloYCur = imuVeloY[imuPointerFront] * ratioFront + imuVeloY[imuPointerBack] * ratioBack;
                    imuVeloZCur = imuVeloZ[imuPointerFront] * ratioFront + imuVeloZ[imuPointerBack] * ratioBack;

                    // imu位置插补
                    imuShiftXCur = imuShiftX[imuPointerFront] * ratioFront + imuShiftX[imuPointerBack] * ratioBack;
                    imuShiftYCur = imuShiftY[imuPointerFront] * ratioFront + imuShiftY[imuPointerBack] * ratioBack;
                    imuShiftZCur = imuShiftZ[imuPointerFront] * ratioFront + imuShiftZ[imuPointerBack] * ratioBack;
                }

                if (i == 0) { //如果是第一个点,记住点云起始位置对应的imu的速度，位移，欧拉角
                    // 此处更新过的角度值主要用在updateImuRollPitchYawStartSinCos()中, 更新每个角的正余弦值
                    imuRollStart = imuRollCur; // 每一帧sweep初始时刻imu的世界姿态，imu模块直接输出的结果
                    imuPitchStart = imuPitchCur;
                    imuYawStart = imuYawCur;

                    imuVeloXStart = imuVeloXCur; // 每一帧sweep初始时刻imu在世界坐标下的速度
                    imuVeloYStart = imuVeloYCur;
                    imuVeloZStart = imuVeloZCur;

                    imuShiftXStart = imuShiftXCur; // 每一帧sweep初始时刻imu在世界坐标下的位移
                    imuShiftYStart = imuShiftYCur;
                    imuShiftZStart = imuShiftZCur;

                    if (timeScanCur + pointTime > imuTime[imuPointerFront]) { // 该条件内imu数据比激光数据早，但是没有更后面的数据
                        imuAngularRotationXCur = imuAngularRotationX[imuPointerFront];
                        imuAngularRotationYCur = imuAngularRotationY[imuPointerFront];
                        imuAngularRotationZCur = imuAngularRotationZ[imuPointerFront];
                    }else{ // 在imu数据充足的情况下可以进行插值
                        int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
                        float ratioFront = (timeScanCur + pointTime - imuTime[imuPointerBack]) 
                                                         / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
                        float ratioBack = (imuTime[imuPointerFront] - timeScanCur - pointTime) 
                                                        / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
                        imuAngularRotationXCur = imuAngularRotationX[imuPointerFront] * ratioFront + imuAngularRotationX[imuPointerBack] * ratioBack;
                        imuAngularRotationYCur = imuAngularRotationY[imuPointerFront] * ratioFront + imuAngularRotationY[imuPointerBack] * ratioBack;
                        imuAngularRotationZCur = imuAngularRotationZ[imuPointerFront] * ratioFront + imuAngularRotationZ[imuPointerBack] * ratioBack;
                    }

                    // 距离上一帧sweep，旋转过的角度变化值
                    imuAngularFromStartX = imuAngularRotationXCur - imuAngularRotationXLast;
                    imuAngularFromStartY = imuAngularRotationYCur - imuAngularRotationYLast;
                    imuAngularFromStartZ = imuAngularRotationZCur - imuAngularRotationZLast;

                    imuAngularRotationXLast = imuAngularRotationXCur;
                    imuAngularRotationYLast = imuAngularRotationYCur;
                    imuAngularRotationZLast = imuAngularRotationZCur;

                    // 这里更新的是i=0时刻的rpy角(imu的世界姿态)，后面将速度坐标投影过来会用到i=0时刻的值
                    updateImuRollPitchYawStartSinCos();
                } else { // 计算除了第一个点之外其他每个点相对于第一个点的由于非匀速(加减速)运动产生的位移速度畸变，并对点云中的每个点位置信息重新补偿矫正
                    // ShiftToStartIMU(pointTime);// 将Lidar位移转到IMU起始坐标系下//计算局部坐标系(点云初始坐标)下点云中的点相对第一个开始点的由于加减速运动产生的位移畸变
                    VeloToStartIMU(); // 速度投影到初始i=0时刻 // 计算局部坐标系下点云中的点相对第一个开始点由于加减速产生的的速度畸变（增量）
                    TransformToStartIMU(&point); // 将点的坐标变换到初始i=0时刻
                }
            }
            // 去掉非匀速运动畸变的点(强度值换成了线号+点相对时间的形式进行填充)重新保存，现在所有点都位于点云初始坐标系下了
            segmentedCloud->points[i] = point;
        }

        imuPointerLastIteration = imuPointerLast;
    }

    // 计算光滑性，这里的计算没有完全按照公式进行， 缺少除以总点数i和r[i]
    void calculateSmoothness()
    {
        int cloudSize = segmentedCloud->points.size(); // 现在分割点云中的点都是在sweep初始坐标系下了
        for (int i = 5; i < cloudSize - 5; i++) { // 使用每个点的前后五个点计算曲率，因此前五个与最后五个点跳过
            ///Q 也可以采用 startRingIndex和 endRingIndex这两个结构来进行计算
            float diffRange = segInfo.segmentedCloudRange[i-5] + segInfo.segmentedCloudRange[i-4]
                            + segInfo.segmentedCloudRange[i-3] + segInfo.segmentedCloudRange[i-2]
                            + segInfo.segmentedCloudRange[i-1] - segInfo.segmentedCloudRange[i] * 10
                            + segInfo.segmentedCloudRange[i+1] + segInfo.segmentedCloudRange[i+2]
                            + segInfo.segmentedCloudRange[i+3] + segInfo.segmentedCloudRange[i+4]
                            + segInfo.segmentedCloudRange[i+5];            

            cloudCurvature[i] = diffRange*diffRange; // 这里没有做归一化

            // 初始时，点全未筛选过，在markOccludedPoints()函数中对该参数进行重新修改
            cloudNeighborPicked[i] = 0;
            // 在extractFeatures()函数中会对标签进行修改，
			// 初始化为0，surfPointsFlat标记为-1，surfPointsLessFlatScan为不大于0的标签
			// cornerPointsSharp标记为2，cornerPointsLessSharp标记为1
            cloudLabel[i] = 0;

            cloudSmoothness[i].value = cloudCurvature[i];
            cloudSmoothness[i].ind = i;
        }
    }

    // 阻塞点是哪种点?
    // 阻塞点指点云之间相互遮挡，而且又靠得很近的点
    void markOccludedPoints()
    {
        int cloudSize = segmentedCloud->points.size();
        //挑选点，排除容易被斜面挡住的点以及离群点，有些点容易被斜面挡住，而离群点可能出现带有偶然性，这些情况都可能导致前后两次扫描不能被同时看到
        for (int i = 5; i < cloudSize - 6; ++i){ // 当前选定点与后一个点(注意，这里只是ID相邻，在空间位置(水平角度)上并不一定是相邻的，因为分割点云是稀疏的)，所以减6

            float depth1 = segInfo.segmentedCloudRange[i];    // 当前选定点的距离值ri
            float depth2 = segInfo.segmentedCloudRange[i+1];  // 后一点的距离值r(i+1)
            int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[i+1] - segInfo.segmentedCloudColInd[i])); // 两个点在距离图像上的列数差 

            if (columnDiff < 10){ // 列数差在一定的范围以内，说明在水平角度上差距不大，则有可能存在遮挡的情况

                ///Q 可以试着和loam_velodyne比较下，看看能否有个更加好的方法来把这些点筛出去
                // 标记距离较远的那些点(此时这些点由于视角变化等等容易被遮挡)，筛选出去
                if (depth1 - depth2 > 0.3){
                    cloudNeighborPicked[i - 5] = 1;
                    cloudNeighborPicked[i - 4] = 1;
                    cloudNeighborPicked[i - 3] = 1;
                    cloudNeighborPicked[i - 2] = 1;
                    cloudNeighborPicked[i - 1] = 1;
                    cloudNeighborPicked[i] = 1;
                }else if (depth2 - depth1 > 0.3){
                    cloudNeighborPicked[i + 1] = 1;
                    cloudNeighborPicked[i + 2] = 1;
                    cloudNeighborPicked[i + 3] = 1;
                    cloudNeighborPicked[i + 4] = 1;
                    cloudNeighborPicked[i + 5] = 1;
                    cloudNeighborPicked[i + 6] = 1;
                }
            }

            float diff1 = std::abs(float(segInfo.segmentedCloudRange[i-1] - segInfo.segmentedCloudRange[i]));
            float diff2 = std::abs(float(segInfo.segmentedCloudRange[i+1] - segInfo.segmentedCloudRange[i]));

            // 选择距离变化较大的点，并将他们标记为1 ， 去掉离群点，包括陡斜面上的点，强烈凸凹点和空旷区域中的某些点，置为筛选过，弃用
            if (diff1 > 0.02 * segInfo.segmentedCloudRange[i] && diff2 > 0.02 * segInfo.segmentedCloudRange[i])
                cloudNeighborPicked[i] = 1;
        }
    }

    void extractFeatures()
    {
        cornerPointsSharp->clear();
        cornerPointsLessSharp->clear();
        surfPointsFlat->clear();
        surfPointsLessFlat->clear();

        for (int i = 0; i < N_SCAN; i++) {

            surfPointsLessFlatScan->clear();

            for (int j = 0; j < 6; j++) { // 将每线scan平均分成6等份(分区)处理
                // 六等份起点：sp = scanStartInd + (scanEndInd - scanStartInd)*j/6
                int sp = (segInfo.startRingIndex[i] * (6 - j)    + segInfo.endRingIndex[i] * j) / 6;
                // 六等份终点：ep = scanStartInd - 1 + (scanEndInd - scanStartInd)*(j+1)/6
                int ep = (segInfo.startRingIndex[i] * (5 - j)    + segInfo.endRingIndex[i] * (j + 1)) / 6 - 1;

                if (sp >= ep)
                    continue;

                // 按照粗糙度(曲率)cloudSmoothness.value从小到大排序
                std::sort(cloudSmoothness.begin()+sp, cloudSmoothness.begin()+ep, by_value());

                // 因为前面对曲率用了从小到大的排序，曲率最大的点的id肯定保存在cloudSortInd[ep]处，相反曲率最小的点的id肯定保存在cloudSortInd[sp]处
                // 1.先选取曲率比较大的特征点：提取2个边缘点 + 20个次边缘点(包含了边缘点)
                int largestPickedNum = 0;
                for (int k = ep; k >= sp; k--) {
                    int ind = cloudSmoothness[k].ind; // 最开始进入那就是当前分区中曲率最大点在分割点云中的点序(id)
                    // 该点需要满足三个特征：未被筛选过，曲率大于阈值，并且不是地面特征点
                    if (cloudNeighborPicked[ind] == 0 &&
                        cloudCurvature[ind] > edgeThreshold &&
                        segInfo.segmentedCloudGroundFlag[ind] == false) {
                        //点分类标号:2-代表曲率很大，1-代表曲率比较大,-1-代表曲率很小，0-曲率比较小(其中1包含了2,0包含了1,0和1构成了点云全部的点)
                        largestPickedNum++;
                        if (largestPickedNum <= 2) { // 挑选曲率最大的前2个点放入sharp边缘点集合
                            cloudLabel[ind] = 2;
                            cornerPointsSharp->push_back(segmentedCloud->points[ind]); // 边缘点点云
                            cornerPointsLessSharp->push_back(segmentedCloud->points[ind]); //cornerPointsLessSharp包含了label为2和1的点
                        } else if (largestPickedNum <= 20) { // 挑选曲率最大的前20个点放入less sharp点集合
                            cloudLabel[ind] = 1;
                            cornerPointsLessSharp->push_back(segmentedCloud->points[ind]); // 次边缘点点云
                        } else {
                            break;
                        }

                        cloudNeighborPicked[ind] = 1; // 进行到这一步表明前面已经把该点加入到点云中去了，置标志位
                        // 将曲率比较大的所选点的前后各5个连续距离比较近的点筛选出去，防止特征点聚集，使得特征点在每个方向上尽量分布均匀
                        for (int l = 1; l <= 5; l++) {
                            int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l - 1]));
                            // 第一眼看起来像是有问题??? 对选定点周围一定距离的一些点才做防止聚集的处理，如果大于这个值了，就不用管它了。这语句没错。
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--) {
                            int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                // 2.再选取4个曲率特别小的地面特征点  注意!!! 这里提取的是地面特征点(与loam区别)
                int smallestPickedNum = 0;
                for (int k = sp; k <= ep; k++) {
                    int ind = cloudSmoothness[k].ind; // 刚开始进入时为曲率最小点在点云中的点序(id)
                    // 该点需要满足三个特征：未被筛选过，曲率小于阈值，必须是地面特征点
                    if (cloudNeighborPicked[ind] == 0 &&
                        cloudCurvature[ind] < surfThreshold &&
                        segInfo.segmentedCloudGroundFlag[ind] == true) {

                        cloudLabel[ind] = -1;
                        surfPointsFlat->push_back(segmentedCloud->points[ind]); // 地面特征点点云

                        smallestPickedNum++;
                        if (smallestPickedNum >= 4) { //只选最小的四个，剩下的Label==0,就都是曲率比较小的
                            break;
                        }

                        cloudNeighborPicked[ind] = 1; // 置标志位
                        // 同样防止特征点聚集
                        for (int l = 1; l <= 5; l++) {

                            int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l - 1]));
                            if (columnDiff > 10)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--) {

                            int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                // 3.将剩余曲率较小的点（包括之前被标记的地面特征平面点）全部归入次平面点中less flat类别中
                for (int k = sp; k <= ep; k++) {
                    if (cloudLabel[k] <= 0) {
                        surfPointsLessFlatScan->push_back(segmentedCloud->points[k]);
                    }
                }
            }

            // surfPointsLessFlatScan中有过多的点云，如果点云太多，计算量太大
            // 进行下采样，可以大大减少计算量
            surfPointsLessFlatScanDS->clear();
            downSizeFilter.setInputCloud(surfPointsLessFlatScan);
            downSizeFilter.filter(*surfPointsLessFlatScanDS);

            *surfPointsLessFlat += *surfPointsLessFlatScanDS;
        }
    }

    void publishCloud()
    {
        sensor_msgs::PointCloud2 laserCloudOutMsg;
        // 发布边缘点
	    if (pubCornerPointsSharp.getNumSubscribers() != 0){
	        pcl::toROSMsg(*cornerPointsSharp, laserCloudOutMsg);
	        laserCloudOutMsg.header.stamp = cloudHeader.stamp;
	        laserCloudOutMsg.header.frame_id = "/camera";
	        pubCornerPointsSharp.publish(laserCloudOutMsg);
	    }
        // 次边缘点，包含了边缘点
	    if (pubCornerPointsLessSharp.getNumSubscribers() != 0){
	        pcl::toROSMsg(*cornerPointsLessSharp, laserCloudOutMsg);
	        laserCloudOutMsg.header.stamp = cloudHeader.stamp;
	        laserCloudOutMsg.header.frame_id = "/camera";
	        pubCornerPointsLessSharp.publish(laserCloudOutMsg);
	    }
        // 地面平面点
	    if (pubSurfPointsFlat.getNumSubscribers() != 0){
	        pcl::toROSMsg(*surfPointsFlat, laserCloudOutMsg);
	        laserCloudOutMsg.header.stamp = cloudHeader.stamp;
	        laserCloudOutMsg.header.frame_id = "/camera";
	        pubSurfPointsFlat.publish(laserCloudOutMsg);
	    }
        // 次平面点(降采样后)，包含了地面平面点
	    if (pubSurfPointsLessFlat.getNumSubscribers() != 0){
	        pcl::toROSMsg(*surfPointsLessFlat, laserCloudOutMsg);
	        laserCloudOutMsg.header.stamp = cloudHeader.stamp;
	        laserCloudOutMsg.header.frame_id = "/camera";
	        pubSurfPointsLessFlat.publish(laserCloudOutMsg);
	    }
    }









































    // 当前点云中的点相对第一个点去除因匀速运动产生的畸变，效果相当于得到在点云扫描开始位置静止扫描得到的点云
    // 最后得到的坐标为点云初始位置坐标系下的坐标(当然这个值是先验，后面会继续迭代)
    // pi为一帧激光点在ti时刻下的坐标，po为激光点在当前sweep开始时的坐标
    void TransformToStart(PointType const * const pi, PointType * const po)
    {
        // 在adjustDistortion() 函数中，对intensity属性进行了如下修改
        // 点强度 = 线号 + 点相对时间（即一个整数+一个小数，整数部分是线号，小数部分是该点相对初始点的时间）
        // point.intensity = int(segmentedCloud->points[i].intensity) + scanPeriod * relTime;
        float s = 10 * (pi->intensity - int(pi->intensity));

        // 首先transformCur这个值是根据上一次位姿变换预测出来的的一帧点云初始时刻跟结束时刻的位姿变化关系
        //线性插值得到选定点(ti时刻)与sweep初始点的位姿变换关系T_i_start：根据每个点在点云中的相对位置关系(或者说是时间关系)，乘以相应的旋转平移系数
        float rx = s * transformCur[0];     // pitch 
        float ry = s * transformCur[1];     // yaw
        float rz = s * transformCur[2];     // roll
        float tx = s * transformCur[3];     // tx
        float ty = s * transformCur[4];     // ty
        float tz = s * transformCur[5];     // tz

        // | cos(-rz)   -sin(-rz)    0 |    | cos(rz)    sin(rz)     0 |
        // | sin(-rz)   cos(-rz)     0 | =  | -sin(rz)   cos(rz)     0 |  运用了三角函数的一些性质
        // |    0           0        1 |    |    0          0        1 | 
        
        // 这里类似于把点从局部坐标系(ti时刻)变换到全局坐标系(点云初始时刻)下，但是由于这里transformCur是点云初始坐标系相对于选定点ti时刻坐标系的位姿变换，
        // 所以这里姿态欧拉角是点云初始时刻相对于ti时刻的姿态的，如果姿态变回点云初始坐标系时需要对角度值取反才能得到ti时刻相对于点云初始时刻的姿态欧拉角
        /*  
            具体原理如下所示：
            由于p_i = R_i_start * p_start + t_i_start ，那么所要求的 p_start = (R_i_start)^-1 * (p_i - t_i_start)
            即 p_start = (R_i_start)^T * (p_i - t_i_start) = R_start_i * (p_i - t_i_start)
            其中，
            R_b_n = R_ZYX(左乘) = Rx(roll)*Ry(pitch)*Rz(yaw)
            R_n_b = R_ZYX(右乘) = Rz(yaw)*Ry(pitch)*Rx(roll) 
        */
        //平移后绕z轴旋转（-rz）
        float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty); // 先去掉平移
        float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
        float z1 = (pi->z - tz);

        //绕x轴旋转（-rx）
        float x2 = x1;
        float y2 = cos(rx) * y1 + sin(rx) * z1;
        float z2 = -sin(rx) * y1 + cos(rx) * z1;

        //绕y轴旋转（-ry）
        po->x = cos(ry) * x2 - sin(ry) * z2;
        po->y = y2;
        po->z = sin(ry) * x2 + cos(ry) * z2;
        po->intensity = pi->intensity;
    }

    //将上一帧点云中的点相对结束位置去除因匀速运动产生的畸变，效果相当于得到在点云扫描结束位置静止扫描得到的点云
    void TransformToEnd(PointType const * const pi, PointType * const po)
    {
        //根据不同点所处位置计算插值系数
        float s = 10 * (pi->intensity - int(pi->intensity));
        // 点云开始相对于点云结束的位姿变换T_end_start，乘以系数则成为选定点(ti时刻)相对点云sweep开始的位姿变换T_i_start
        float rx = s * transformCur[0];
        float ry = s * transformCur[1];
        float rz = s * transformCur[2];
        float tx = s * transformCur[3];
        float ty = s * transformCur[4];
        float tz = s * transformCur[5];

        // p_i = R_i_s*p_start + t_i_s ====> p_start = R_i_s^-1(p_i - t_i_s)
        // 转换得到点云初始坐标系下的坐标，解释见上面的函数
        //平移后绕z轴旋转（-rz）
        float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
        float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
        float z1 = (pi->z - tz);

        //绕x轴旋转（-rx）
        float x2 = x1;
        float y2 = cos(rx) * y1 + sin(rx) * z1;
        float z2 = -sin(rx) * y1 + cos(rx) * z1;

        //绕y轴旋转（-ry）
        float x3 = cos(ry) * x2 - sin(ry) * z2;
        float y3 = y2;
        float z3 = sin(ry) * x2 + cos(ry) * z2; //求出了相对于起始点校正的坐标

        // 点云开始相对于点云结束的位姿变换T_end_start
        rx = transformCur[0];
        ry = transformCur[1];
        rz = transformCur[2];
        tx = transformCur[3];
        ty = transformCur[4];
        tz = transformCur[5];

        //转移到点云结束的局部坐标系下 p_e = R_e_s * p_s + t_e_s
        //绕y轴旋转（ry）
        float x4 = cos(ry) * x3 + sin(ry) * z3;
        float y4 = y3;
        float z4 = -sin(ry) * x3 + cos(ry) * z3;

        //绕x轴旋转（rx）
        float x5 = x4;
        float y5 = cos(rx) * y4 - sin(rx) * z4;
        float z5 = sin(rx) * y4 + cos(rx) * z4;

        //绕z轴旋转（rz），再平移
        float x6 = cos(rz) * x5 - sin(rz) * y5 + tx;
        float y6 = sin(rz) * x5 + cos(rz) * y5 + ty;
        float z6 = z5 + tz;

        // 先去掉加减速的畸变位移，再旋转至sweep的初始世界坐标下
        //平移后绕z轴旋转（imuRollStart）
        float x7 = cosImuRollStart * (x6 - imuShiftFromStartX) 
                 - sinImuRollStart * (y6 - imuShiftFromStartY);
        float y7 = sinImuRollStart * (x6 - imuShiftFromStartX) 
                 + cosImuRollStart * (y6 - imuShiftFromStartY);
        float z7 = z6 - imuShiftFromStartZ;

        //绕x轴旋转（imuPitchStart）
        float x8 = x7;
        float y8 = cosImuPitchStart * y7 - sinImuPitchStart * z7;
        float z8 = sinImuPitchStart * y7 + cosImuPitchStart * z7;

        //绕y轴旋转（imuYawStart）
        float x9 = cosImuYawStart * x8 + sinImuYawStart * z8;
        float y9 = y8;
        float z9 = -sinImuYawStart * x8 + cosImuYawStart * z8;

        // 从世界坐标系下转换到sweep结束坐标系下
        //绕y轴旋转（-imuYawLast）
        float x10 = cos(imuYawLast) * x9 - sin(imuYawLast) * z9;
        float y10 = y9;
        float z10 = sin(imuYawLast) * x9 + cos(imuYawLast) * z9;

        //绕x轴旋转（-imuPitchLast）
        float x11 = x10;
        float y11 = cos(imuPitchLast) * y10 + sin(imuPitchLast) * z10;
        float z11 = -sin(imuPitchLast) * y10 + cos(imuPitchLast) * z10;

        //绕z轴旋转（-imuRollLast）
        po->x = cos(imuRollLast) * x11 + sin(imuRollLast) * y11;
        po->y = -sin(imuRollLast) * x11 + cos(imuRollLast) * y11;
        po->z = z11;
        po->intensity = int(pi->intensity); //只保留线号
    }

    //利用IMU修正旋转量，根据起始欧拉角，当前点云的欧拉角修正
    // bcx,bcy,bcz 为当前lidar最后一个点在世界坐标系下的姿态欧拉角
    // blx,bly,blz 为点云初始点的pitch,yaw,roll 世界坐标系下
    // alx,aly,alz 为点云最后一个点的pitch,yaw,roll 世界坐标系下
    // acx,acy,acz 为经过imu补偿修正后的姿态角
    void PluginIMURotation(float bcx, float bcy, float bcz, float blx, float bly, float blz, 
                           float alx, float aly, float alz, float &acx, float &acy, float &acz)
    {
        float sbcx = sin(bcx);
        float cbcx = cos(bcx);
        float sbcy = sin(bcy);
        float cbcy = cos(bcy);
        float sbcz = sin(bcz);
        float cbcz = cos(bcz);

        float sblx = sin(blx);
        float cblx = cos(blx);
        float sbly = sin(bly);
        float cbly = cos(bly);
        float sblz = sin(blz);
        float cblz = cos(blz);

        float salx = sin(alx);
        float calx = cos(alx);
        float saly = sin(aly);
        float caly = cos(aly);
        float salz = sin(alz);
        float calz = cos(alz);

        float srx = -sbcx*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly) 
                  - cbcx*cbcz*(calx*saly*(cbly*sblz - cblz*sblx*sbly) 
                  - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx) 
                  - cbcx*sbcz*(calx*caly*(cblz*sbly - cbly*sblx*sblz) 
                  - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz);
        acx = -asin(srx);

        float srycrx = (cbcy*sbcz - cbcz*sbcx*sbcy)*(calx*saly*(cbly*sblz - cblz*sblx*sbly) 
                     - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx) 
                     - (cbcy*cbcz + sbcx*sbcy*sbcz)*(calx*caly*(cblz*sbly - cbly*sblx*sblz) 
                     - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz) 
                     + cbcx*sbcy*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly);
        float crycrx = (cbcz*sbcy - cbcy*sbcx*sbcz)*(calx*caly*(cblz*sbly - cbly*sblx*sblz) 
                     - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz) 
                     - (sbcy*sbcz + cbcy*cbcz*sbcx)*(calx*saly*(cbly*sblz - cblz*sblx*sbly) 
                     - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx) 
                     + cbcx*cbcy*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly);
        acy = atan2(srycrx / cos(acx), crycrx / cos(acx));
        
        float srzcrx = sbcx*(cblx*cbly*(calz*saly - caly*salx*salz) 
                     - cblx*sbly*(caly*calz + salx*saly*salz) + calx*salz*sblx) 
                     - cbcx*cbcz*((caly*calz + salx*saly*salz)*(cbly*sblz - cblz*sblx*sbly) 
                     + (calz*saly - caly*salx*salz)*(sbly*sblz + cbly*cblz*sblx) 
                     - calx*cblx*cblz*salz) + cbcx*sbcz*((caly*calz + salx*saly*salz)*(cbly*cblz 
                     + sblx*sbly*sblz) + (calz*saly - caly*salx*salz)*(cblz*sbly - cbly*sblx*sblz) 
                     + calx*cblx*salz*sblz);
        float crzcrx = sbcx*(cblx*sbly*(caly*salz - calz*salx*saly) 
                     - cblx*cbly*(saly*salz + caly*calz*salx) + calx*calz*sblx) 
                     + cbcx*cbcz*((saly*salz + caly*calz*salx)*(sbly*sblz + cbly*cblz*sblx) 
                     + (caly*salz - calz*salx*saly)*(cbly*sblz - cblz*sblx*sbly) 
                     + calx*calz*cblx*cblz) - cbcx*sbcz*((saly*salz + caly*calz*salx)*(cblz*sbly 
                     - cbly*sblx*sblz) + (caly*salz - calz*salx*saly)*(cbly*cblz + sblx*sbly*sblz) 
                     - calx*calz*cblx*sblz);
        acz = atan2(srzcrx / cos(acx), crzcrx / cos(acx));
    }

    // 计算第一帧点云即原点相对于当前帧的积累旋转量 R_0_k+1 = R_0_k * R_k_k+1
    // cx,cy,cz 上一帧相对于第一帧的姿态欧拉角，lx,ly,lz 当前帧相对于上一帧的欧拉角
    // ox,oy,oz 当前帧相对于第一帧点云的姿态欧拉角
    void AccumulateRotation(float cx, float cy, float cz, float lx, float ly, float lz, 
                            float &ox, float &oy, float &oz)
    {
        /*
        这里三个旋转矩阵的形式都如下所示:
            | cycz+sxsysz  -cysz+sxsycz   cxsy |
        R = |    cxsz          cxcz       -sx  | = Ry*Rx*Rz (ZYX右乘或者XYZ左乘)
            | -sycz+sxcysz  sysz+sxcycz  cxcy  |
        此时用的旋转矩阵如下：
             | 1  0   0   |
        Rx = | 0  cx  -sx | = R_w_b
             | 0  sx  cx  |
        */ 
        // R23
        float srx = cos(lx)*cos(cx)*sin(ly)*sin(cz) - cos(cx)*cos(cz)*sin(lx) - cos(lx)*cos(ly)*sin(cx);
        ox = -asin(srx);

        //R13
        float srycrx = sin(lx)*(cos(cy)*sin(cz) - cos(cz)*sin(cx)*sin(cy)) + cos(lx)*sin(ly)*(cos(cy)*cos(cz) 
                     + sin(cx)*sin(cy)*sin(cz)) + cos(lx)*cos(ly)*cos(cx)*sin(cy);
        //R33
        float crycrx = cos(lx)*cos(ly)*cos(cx)*cos(cy) - cos(lx)*sin(ly)*(cos(cz)*sin(cy) 
                     - cos(cy)*sin(cx)*sin(cz)) - sin(lx)*(sin(cy)*sin(cz) + cos(cy)*cos(cz)*sin(cx));
        oy = atan2(srycrx / cos(ox), crycrx / cos(ox));

        //R21
        float srzcrx = sin(cx)*(cos(lz)*sin(ly) - cos(ly)*sin(lx)*sin(lz)) + cos(cx)*sin(cz)*(cos(ly)*cos(lz) 
                     + sin(lx)*sin(ly)*sin(lz)) + cos(lx)*cos(cx)*cos(cz)*sin(lz);
        //R22
        float crzcrx = cos(lx)*cos(lz)*cos(cx)*cos(cz) - cos(cx)*sin(cz)*(cos(ly)*sin(lz) 
                     - cos(lz)*sin(lx)*sin(ly)) - sin(cx)*(sin(ly)*sin(lz) + cos(ly)*cos(lz)*sin(lx));
        oz = atan2(srzcrx / cos(ox), crzcrx / cos(ox));
    }

    double rad2deg(double radians)
    {
        return radians * 180.0 / M_PI;
    }

    double deg2rad(double degrees)
    {
        return degrees * M_PI / 180.0;
    }

    void findCorrespondingCornerFeatures(int iterCount){

        //处理当前点云中的曲率最大的特征点(边缘点),从上个点云中曲率比较大的特征点中找两个最近距离点，一个点使用kd-tree查找，另一个根据找到的点在其相邻线找另外一个最近距离的点
        int cornerPointsSharpNum = cornerPointsSharp->points.size();

        for (int i = 0; i < cornerPointsSharpNum; i++) {
            //TODO 利用前一次位姿的计算结果，按照匀速模型内插消除畸变，将每个点转换到点云起始坐标系并去除畸变，并保存在pointSel中
            // 每一次迭代都将特征点都要利用当前预测的坐标转换转换至k+1 sweep的初始位置处对应于函数 TransformToStart()
            // 每一次迭代都将特征点都要利用当前预测的坐标转换转换至k+1 sweep的结束位置处对应于函数 TransformToEnd()
            // pointSel是当前时刻t+1的cornerPointsSharp转换到初始点云坐标系后的点坐标，对角点一个一个做处理，设为i点
            TransformToStart(&cornerPointsSharp->points[i], &pointSel);
            
            // 优化算法每迭代五次，就重新查找最近点，否则沿用上次迭代的最近点
            if (iterCount % 5 == 0) {
                // std::vector<int> indices;
                // pcl::removeNaNFromPointCloud(*laserCloudCornerLast,*laserCloudCornerLast, indices); // 上一帧的边缘点剔除异常值

                // nearestKSearch是PCL中的K近邻域搜索，搜索上一时刻LessCornor的K邻域点
                // 搜索结果: pointSearchInd是搜索到的最近点在kdtreeCornerLast的索引; pointSearchSqDis是近邻对应的平方距离(以25作为阈值)
                //这里，只在kd-tree查找一个最近距离点，边缘点未经过体素栅格滤波，一般边缘点本来就比较少，不做滤波
                /*
                搜索给定查询点的k近邻。该函数不返回距离，返回值为找到的邻居数量
                int nearestKSearch( const PointT &p_q, //给定的查询点(必须先有setInputCloud步骤!)
                                    int k, //要搜索的邻居数量(仅当水平和垂直窗口还没有给出时使用!)
                                    Indices &k_indices, //搜索到的结果点索引(必须事先调整为k !)
                                    std::vector< float > &k_sqr_distances //结果点对于查询点的平方距离
                                    )		const
                */
               /******** 在上一时刻t的LessSharp中用KD树寻找和点i最近的点j ********/
                kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
                int closestPointInd = -1, minPointInd2 = -1;

                /***** 在最邻近点附近(临近扫描线scan上寻找)找到距离当前给定边缘点的次临近点 *****/
                //寻找相邻线距离目标点距离最小的点
                //再次提醒：velodyne是2度一线，scanID相邻并不代表线号相邻，相邻线度数相差2度，也即线号scanID相差2
                //这里作者没有直接nearKSearch找最近的两个点，也为了考虑这两个点要有效的吧，                
                if (pointSearchSqDis[0] < nearestFeatureSearchSqDist) {
                    closestPointInd = pointSearchInd[0]; // 找到的距离当前时刻k+1点云中某边缘点最近的上一帧点云k的点的ID(kdtreeCornerLast里的ID)
                    int closestPointScan = int(laserCloudCornerLast->points[closestPointInd].intensity); // 提取最近点线号scan

                    float pointSqDis, minPointSqDis2 = nearestFeatureSearchSqDist; //初始门槛值5米，可大致过滤掉scanID相邻，但实际线不相邻的值
                    
                    /******** 在上一时刻t的LessSharp中点j附近2层中最近的点l ********/
                    //寻找距离目标点最近距离的平方和最小的另一个点
                    // 注意这里的j是待查找点在laserCloudCornerLast的ID
                    for (int j = closestPointInd + 1; j < cornerPointsSharpNum; j++) { //向scanID增大的方向查找
                        if (int(laserCloudCornerLast->points[j].intensity) > closestPointScan + 2.5) { //非相邻线scan
                            break;
                        }

                        pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) * 
                                     (laserCloudCornerLast->points[j].x - pointSel.x) + 
                                     (laserCloudCornerLast->points[j].y - pointSel.y) * 
                                     (laserCloudCornerLast->points[j].y - pointSel.y) + 
                                     (laserCloudCornerLast->points[j].z - pointSel.z) * 
                                     (laserCloudCornerLast->points[j].z - pointSel.z);

                        //确保两个点不在同一条scan上（注意，相邻线查找应该不能用scanID == closestPointScan +/- 1 来做）
                        if (int(laserCloudCornerLast->points[j].intensity) > closestPointScan) {
                            if (pointSqDis < minPointSqDis2) { //距离更近，要小于初始值5米
                                //更新最小距离与点序
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j; // 先在id增大的方向找到一个最小距离的点
                            }
                        }
                    }
                    //同理
                    for (int j = closestPointInd - 1; j >= 0; j--) { //向scanID减小的方向查找
                        if (int(laserCloudCornerLast->points[j].intensity) < closestPointScan - 2.5) {
                            break;
                        }

                        pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) * 
                                     (laserCloudCornerLast->points[j].x - pointSel.x) + 
                                     (laserCloudCornerLast->points[j].y - pointSel.y) * 
                                     (laserCloudCornerLast->points[j].y - pointSel.y) + 
                                     (laserCloudCornerLast->points[j].z - pointSel.z) * 
                                     (laserCloudCornerLast->points[j].z - pointSel.z);

                        if (int(laserCloudCornerLast->points[j].intensity) < closestPointScan) {
                            if (pointSqDis < minPointSqDis2) {
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j; // 如果在id减小的方向还能找到比上一步ID增大方向上距离给定点还小的点，就更新
                            }
                        }
                    }
                }

                pointSearchCornerInd1[i] = closestPointInd; //kd-tree最近距离点，论文中的点j，-1表示未找到满足的点
                pointSearchCornerInd2[i] = minPointInd2; //另一个相邻scan最近的点，论文中的点l，-1表示未找到满足的点
            }

            if (pointSearchCornerInd2[i] >= 0) { //大于等于0，不等于-1，说明两个点都找到了 //pointSearchCornerInd2[i]的条件呢???

                tripod1 = laserCloudCornerLast->points[pointSearchCornerInd1[i]]; // loam论文中的点j
                tripod2 = laserCloudCornerLast->points[pointSearchCornerInd2[i]]; // loam论文中的点l

                //选择的特征点记为O(论文i)，kd-tree最近距离点记为A(论文j)，另一个最近距离点记为B(论文l)
                float x0 = pointSel.x;
                float y0 = pointSel.y;
                float z0 = pointSel.z;
                float x1 = tripod1.x;
                float y1 = tripod1.y;
                float z1 = tripod1.z;
                float x2 = tripod2.x;
                float y2 = tripod2.y;
                float z2 = tripod2.z;

                float m11 = ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1));
                float m22 = ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1));
                float m33 = ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1));
                //向量OA = (x0 - x1, y0 - y1, z0 - z1), 向量OB = (x0 - x2, y0 - y2, z0 - z2)，向量AB = （x1 - x2, y1 - y2, z1 - z2）
                //向量OA OB的向量积(即叉乘)为：
                //|  i      j      k  |
                //|x0-x1  y0-y1  z0-z1|       OA × OB = [OA]× OB 结果是一个3*1的向量(其实也就是OAB平面的法向量)
                //|x0-x2  y0-y2  z0-z2|
                //模为：
                float a012 = sqrt(m11 * m11  + m22 * m22 + m33 * m33);

                //两个最近距离点之间的距离，即向量AB的模
                float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));
                
                //AB方向的单位向量与OAB平面的单位法向量的向量积(叉积)在各轴上的分量（d的方向）
                // 距离方程f对初始坐标下的偏导，即△偏导符号求导
                // T = [rx,ry,rz,tx,ty,tz]^T 待优化位姿变量
                // pt = R_0_t * p0 + T_t_0 ,其中pt为点云原始坐标，p0为点云在起始坐标系下的坐标
                // δf / δrx = δf/δpt * δp0/δrx = [la,lb,lc] * δp0/δrx
                // δf/δpt为梯度方向，对于点和直线，自然是垂直直线的方向，对于点和平面，自然是法向量方向
                // 具体细节见 https://zhuanlan.zhihu.com/p/333668148
                //////Q暂时不是很理解
                //x轴分量i
                float la =  ((y1 - y2)*m11 + (z1 - z2)*m22) / a012 / l12;

                //y轴分量j
                float lb = -((x1 - x2)*m11 - (z1 - z2)*m33) / a012 / l12;

                //z轴分量k
                float lc = -((x1 - x2)*m22 + (y1 - y2)*m33) / a012 / l12;

                //点到线的距离，d = |向量OA 叉乘 向量OB|/|AB|
                float ld2 = a012 / l12;

                //权重计算，距离越大权重越小，距离越小权重越大，得到的权重范围<=1
                float s = 1;
                if (iterCount >= 5) { //5次迭代之后开始增加权重因素
                    s = 1 - 1.8 * fabs(ld2);
                }

                if (s > 0.1 && ld2 != 0) { //只保留权重大的，也即距离比较小的点，同时也舍弃距离为零的
                    coeff.x = s * la; 
                    coeff.y = s * lb;
                    coeff.z = s * lc;
                    coeff.intensity = s * ld2;
                  
                    laserCloudOri->push_back(cornerPointsSharp->points[i]); // 原始点云，未经过匀速畸变校正
                    coeffSel->push_back(coeff);
                }
            }
        }
    }

    void findCorrespondingSurfFeatures(int iterCount){ // iterCount 算法迭代次数

        int surfPointsFlatNum = surfPointsFlat->points.size(); // 当前帧平面特征点(其实就是地面点)个数

        for (int i = 0; i < surfPointsFlatNum; i++) {
            // 利用前一次位姿的计算结果，按照匀速模型内插消除畸变，将每个点转换到点云起始坐标系并去除畸变，并保存在pointSel中
            // 每一次迭代都将特征点利用当前预测的坐标转换 转换至k+1 sweep的初始位置处，对应于函数 TransformToStart()
            // 每一次迭代都将特征点利用当前预测的坐标转换 转换至k+1 sweep的结束位置处，对应于函数 TransformToEnd()

            // 当前时刻K+1转换到点云初始坐标系下(利用匀速模型去掉畸变)，对平面点做处理，设为点i
            TransformToStart(&surfPointsFlat->points[i], &pointSel);

            // 优化算法每迭代五次，就重新查找最近点，否则沿用上次迭代的最近点
            if (iterCount % 5 == 0) {
                
                // nearestKSearch是PCL中的K近邻域搜索，搜索上一时刻kdtreeSurfLast的K邻域点
                // 搜索结果: pointSearchInd是搜索到的最近点在kdtreeCornerLast的索引; pointSearchSqDis是近邻对应的平方距离
                // 这里k=1， 只在kd-tree查找一个最近距离点
                //kd-tree最近点查找，在经过体素栅格滤波之后的地面平面点+次平面点中查找，因为一般平面点太多，滤波后最近点查找数据量小
                /*
                搜索给定查询点的k近邻。该函数不返回距离，返回值为找到的邻居数量
                int nearestKSearch( const PointT &p_q, //给定的查询点(必须先有setInputCloud步骤!)
                                    int k, //要搜索的邻居数量(仅当水平和垂直窗口还没有给出时使用!)
                                    Indices &k_indices, //搜索到的结果点索引(必须事先调整为k !)
                                    std::vector< float > &k_sqr_distances //结果点对于查询点的平方距离
                                    )		const
                */
                // 在上一时刻t的LessFlat中用KD树找距离点i最近的点j
                kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
                int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;

                if (pointSearchSqDis[0] < nearestFeatureSearchSqDist) {
                    closestPointInd = pointSearchInd[0]; // 上一帧点云(地面平面点+次平面点)中最近点的ID，loam论文中的点j
                    // 在adjustDistortion() 函数中，对intensity属性进行了如下修改
                    // 点强度 = 线号 + 点相对时间（即一个整数+一个小数，整数部分是线号，小数部分是该点相对初始点的时间）
                    // point.intensity = int(segmentedCloud->points[i].intensity) + scanPeriod * relTime;
                    int closestPointScan = int(laserCloudSurfLast->points[closestPointInd].intensity); // 搜索到的最近点所在的线号

                    // 主要功能是在上一帧点云中找到另外2个相邻scan之内与选定点的距离最近点(其中一个点与j同线，另一个点在相邻的线上)，并将找到的最近点及其序号保存
                    ///Q 有个疑问?需不需要对边界线号进行判断处理(比如线号为0,15这两个)是否有必要  ---感觉没有必要
                    float pointSqDis, minPointSqDis2 = nearestFeatureSearchSqDist, minPointSqDis3 = nearestFeatureSearchSqDist;
                    for (int j = closestPointInd + 1; j < surfPointsFlatNum; j++) { //向scanID增大的方向查找
                        ///Q 由于之前作了在距离图像的投影，我觉得大于1就可以排除不是相邻scan的情况了(因为loam他的id跟线号并不是一一对应的)
                        if (int(laserCloudSurfLast->points[j].intensity) > closestPointScan + 2.5) { // 排除非相邻线scan
                            break;
                        }

                        pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) * 
                                     (laserCloudSurfLast->points[j].x - pointSel.x) + 
                                     (laserCloudSurfLast->points[j].y - pointSel.y) * 
                                     (laserCloudSurfLast->points[j].y - pointSel.y) + 
                                     (laserCloudSurfLast->points[j].z - pointSel.z) * 
                                     (laserCloudSurfLast->points[j].z - pointSel.z);

                        // 如果点的线号小于等于最近点的线号，这里 为什么要加一个小于的判断???
                        // 这是为了方便用if条件一次判断同一线上的点，因为这个是id增大的方向，不存在线号小于找到的最近点i的线号的情况
                        if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScan) {
                            if (pointSqDis < minPointSqDis2) {
                              minPointSqDis2 = pointSqDis; // loam论文中的点l，与点j同线，最后保留的是往ID增大与减小两个方向上距离最小的那个值
                              minPointInd2 = j;
                            }
                        } else { //如果点所在的线号大于找到的最近点j的线号(其实就是相邻线scan)，即loam论文中的点m
                            if (pointSqDis < minPointSqDis3) {
                                minPointSqDis3 = pointSqDis;
                                minPointInd3 = j;
                            }
                        }
                    }
                    // 同理，向scanID减小的方向查找
                    for (int j = closestPointInd - 1; j >= 0; j--) {
                        if (int(laserCloudSurfLast->points[j].intensity) < closestPointScan - 2.5) { // 排除非相邻线scan
                            break;
                        }

                        pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) * 
                                     (laserCloudSurfLast->points[j].x - pointSel.x) + 
                                     (laserCloudSurfLast->points[j].y - pointSel.y) * 
                                     (laserCloudSurfLast->points[j].y - pointSel.y) + 
                                     (laserCloudSurfLast->points[j].z - pointSel.z) * 
                                     (laserCloudSurfLast->points[j].z - pointSel.z);

                        if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScan) { // 找相同线scan的最近点
                            if (pointSqDis < minPointSqDis2) { // 如果有更小的，就保留
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j;
                            }
                        } else { // 找相邻线scan的最近点
                            if (pointSqDis < minPointSqDis3) {
                                minPointSqDis3 = pointSqDis;
                                minPointInd3 = j;
                            }
                        }
                    }
                }

                pointSearchSurfInd1[i] = closestPointInd; //kd-tree最近距离点，loam论文中的点j,-1表示未找到满足要求的点
                pointSearchSurfInd2[i] = minPointInd2; //同一线号上的距离最近的点，loam论文中的点l，-1表示未找到满足要求的点
                pointSearchSurfInd3[i] = minPointInd3; //不同线号上的距离最近的点，loam论文中的点m，-1表示未找到满足要求的点
            }

            /*******计算点i到平面jlm的距离dh 理想情况下ijlm共面 *******/
            if (pointSearchSurfInd2[i] >= 0 && pointSearchSurfInd3[i] >= 0) { //确实找到了这样三个点

                tripod1 = laserCloudSurfLast->points[pointSearchSurfInd1[i]]; //A点，论文中的点j
                tripod2 = laserCloudSurfLast->points[pointSearchSurfInd2[i]]; //B点，论文中的点l
                tripod3 = laserCloudSurfLast->points[pointSearchSurfInd3[i]]; //C点，论文中的点m
                //向量AB = (tripod2.x - tripod1.x, tripod2.y - tripod1.y, tripod2.z - tripod1.z)
                //向量AC = (tripod3.x - tripod1.x, tripod3.y - tripod1.y, tripod3.z - tripod1.z)

                // pa,pb,pc既为偏导函数的分子部分也为距离函数分母行列式内各方向分量值，ps为分母部分
                // 向量AB，AC的向量积（即叉乘），得到的是jlm平面的法向量 ，其模值为以jlm为三个顶点组成的的平行四面边的面积
                // 假设当前帧选中点为点O ，则以OABC为顶点的平行六面体的体积为：| AO · (AB × AC) |
                // 则 点O到ABC平面的距离为：| AO · (AB × AC) | / | AB × AC |，具体公式见loam论文
                
                // AB × AC = [AB]× AC = [pa,pb,pc]^T，通过叉乘计算出来的jlm平面的法向量，各个分量为在各个轴上的坐标(投影值)
                //法向量在x轴方向分向量i
                float pa = (tripod2.y - tripod1.y) * (tripod3.z - tripod1.z) // 向量[pa；pb；pc] = 点到面的距离对x0 y0 z0的偏导
                         - (tripod3.y - tripod1.y) * (tripod2.z - tripod1.z);
                //法向量在y轴方向分向量j
                float pb = (tripod2.z - tripod1.z) * (tripod3.x - tripod1.x) 
                         - (tripod3.z - tripod1.z) * (tripod2.x - tripod1.x);
                //法向量在z轴方向分向量k
                float pc = (tripod2.x - tripod1.x) * (tripod3.y - tripod1.y) 
                         - (tripod3.x - tripod1.x) * (tripod2.y - tripod1.y);
                float pd = -(pa * tripod1.x + pb * tripod1.y + pc * tripod1.z); // 体积计算产生的中间变量，也可以看做个点积值

                float ps = sqrt(pa * pa + pb * pb + pc * pc); //ps 法向量的模 ，也是ABC三个顶点所组成的平行四边形的面积

                //pa pb pc为法向量各方向上的单位向量(归一化了)
                pa /= ps;
                pb /= ps;
                pc /= ps;
                pd /= ps;

                // 距离没有取绝对值
                // 两个向量的点乘，分母除以ps中已经除掉了，
                // 加pd原因:pointSel与tripod1构成的线段需要相减

                //点到面的距离：向量AO与与法向量的点积除以法向量的模
                float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                // 计算权重
                //这里的s是个权重, 表示s在这个least-square问题中的置信度, 每个点的置信度不一样
		        //理论上, 这个权重, 与点到面距离负相关, 距离越大, 置信度越低, 这里相当于是一个在loss之外加了一个鲁棒性函数, 用来过减弱离群值的影响
		        //源代码中"sqrt(sqrt(point_sel.x * point_sel.x + point_sel.y * point_sel.y + point_sel.z * point_sel.z)" 这部分, 并没有什么逻辑性可言
		        //你可以设计自己的鲁棒性函数来替代这一行代码
                float s = 1;
                if (iterCount >= 5) {
                    s = 1 - 1.8 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x
                            + pointSel.y * pointSel.y + pointSel.z * pointSel.z));///Q 这个表达式有何依据???
                }

                if (s > 0.1 && pd2 != 0) {
                    // [x,y,z]是整个平面的单位法向量乘以权重系数
                    // intensity是平面外一点到该平面的距离
                    coeff.x = s * pa;
                    coeff.y = s * pb;
                    coeff.z = s * pc;
                    coeff.intensity = s * pd2;
                    // 未经变换的点放入laserCloudOri队列，距离，法向量值放入coeffSel
                    laserCloudOri->push_back(surfPointsFlat->points[i]); //保存原始点(未处于sweep初始坐标系中)与相应的系数
                    coeffSel->push_back(coeff);
                }//至此Jaccobian矩阵就构建完毕了。每个特征点对应的Jaccobian矩阵的三个元素都保存在coeffSel中，后面采用L-M方法解算的时候直接调用就行了
            }
        }
    }

    bool calculateTransformationSurf(int iterCount){
        // 开始迭代计算了
        // pointSelNum是有多少个对应约束(加入了多少个特征点)
        int pointSelNum = laserCloudOri->points.size();

        cv::Mat matA(pointSelNum, 3, CV_32F, cv::Scalar::all(0)); // 因为是两步优化，现在这一步只是优化[tz,roll,pitch]三个量，所以列数是3
        cv::Mat matAt(3, pointSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(3, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(pointSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(3, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(3, 1, CV_32F, cv::Scalar::all(0));

        float srx = sin(transformCur[0]); //pitch, X axis
        float crx = cos(transformCur[0]);
        float sry = sin(transformCur[1]); //yaw, Y axis
        float cry = cos(transformCur[1]);
        float srz = sin(transformCur[2]); //roll, Z axis
        float crz = cos(transformCur[2]);
        float tx = transformCur[3];
        float ty = transformCur[4];
        float tz = transformCur[5];
        /*
            ///Q 需要好好推导一下这里的方程
            优化方程为： f(p_start)=f(R_start_end(p_end-t_end_start)) = d
            求偏导
            梯度方向，对于点和直线，自然是垂直直线的方向，对于点和平面，自然是法向量方向
            距离方程f对初始坐标下的偏导，即△偏导符号求导
            T = [rx,ry,tz]^T 待优化位姿变量
            pk = R_k_0 * p0 + t_k_0 ,其中pk为k时刻(即点云中任一点)点云原始坐标，p0为点云在起始坐标系下的坐标
            ===> p0 = R_k_0 ^-1 (pk-t_k_0)

            δf / δrx = δf/δp0 * δp0/δrx = [la,lb,lc] * δp0/δrx = [la,lb,lc] * δR_k_0^-1/δrx *(pk-t_k_0)
            δf / δtx = δf/δp0 * δp0/δtx = [la,lb,lc] * δp0/δtx = [la,lb,lc] * δ[R_k_0^-1(pk-t_k_0)]/δtx = [la,lb,lc] * R_k_0^-1 * [-1;0;0]^T
            其中，
                                    | cycz-sxsysz  cysz+sxsycz  -sycx |
            R(待优化) = R_k_0 ^-1 =  |    -cxsz        cxcz        sx  | = Ry*Rx*Rz (ZYX右乘)
                                    | sycz+sxcysz  sysz-sxcycz  cycx  |
            
            这里,       | 1   0    0 |
                Rx =    | 0  cx   sx |  为什么是这个形式??因为这里的角度是点云开始相对于点云结束时的角度(先忽略掉他的符号)，是以点云结束坐标系为参考(世界坐标系)的。
                        | 0  -sx  cx |  然后，我们这里优化的姿态矩阵是R_k_0^-1 = R_0_k，相当于从世界坐标系到局部坐标系，所以用的是这种旋转矩阵
        */

        float a1 = crx*sry*srz; float a2 = crx*crz*sry; float a3 = srx*sry; float a4 = tx*a1 - ty*a2 - tz*a3;
        float a5 = srx*srz; float a6 = crz*srx; float a7 = ty*a6 - tz*crx - tx*a5;
        float a8 = crx*cry*srz; float a9 = crx*cry*crz; float a10 = cry*srx; float a11 = tz*a10 + ty*a9 - tx*a8;

        float b1 = -crz*sry - cry*srx*srz; float b2 = cry*crz*srx - sry*srz;
        float b5 = cry*crz - srx*sry*srz; float b6 = cry*srz + crz*srx*sry;

        float c1 = -b6; float c2 = b5; float c3 = tx*b6 - ty*b5; float c4 = -crx*crz; float c5 = crx*srz; float c6 = ty*c5 + tx*-c4;
        float c7 = b2; float c8 = -b1; float c9 = tx*-b2 - ty*-b1;

        for (int i = 0; i < pointSelNum; i++) { // 构建单个特征点的代价方程，并把所有特征点进行叠加

            pointOri = laserCloudOri->points[i]; // 当前时刻点坐标(未去除匀速畸变，因为后面会用迭代优化的位姿继续计算)
            coeff = coeffSel->points[i]; // 该点所对应的偏导数，其实就是单位法向量
            //对第一步优化的变量求雅克比矩阵 [la,lb,lc] * [δR_k_0^-1/δrx *(pk-t_k_0)]
            float arx = (-a1*pointOri.x + a2*pointOri.y + a3*pointOri.z + a4) * coeff.x
                      + (a5*pointOri.x - a6*pointOri.y + crx*pointOri.z + a7) * coeff.y
                      + (a8*pointOri.x - a9*pointOri.y - a10*pointOri.z + a11) * coeff.z;

            float arz = (c1*pointOri.x + c2*pointOri.y + c3) * coeff.x
                      + (c4*pointOri.x - c5*pointOri.y + c6) * coeff.y
                      + (c7*pointOri.x + c8*pointOri.y + c9) * coeff.z;

            float aty = -b6 * coeff.x + c4 * coeff.y + b2 * coeff.z;

            float d2 = coeff.intensity;
            // A=[J的偏导]; B=[权重系数*(点到直线的距离 or 点到平面的距离)] 求解公式: AX=B
            // 为了让左边满秩，同乘At-> At*A*X = At*B
            matA.at<float>(i, 0) = arx;
            matA.at<float>(i, 1) = arz;
            matA.at<float>(i, 2) = aty;
            matB.at<float>(i, 0) = -0.05 * d2;//matB是代价向量  //-0.05 :猜测：防止求得的增量过大，使得算法震荡
        }

        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        //求解matAtA * matX = matAtB
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);//QR分解得到X
        
        //接下来有一个迭代第一步的处理，猜测是出现退化进行修正，然后更新位姿之后进行收敛判断
        if (iterCount == 0) {
            cv::Mat matE(1, 3, CV_32F, cv::Scalar::all(0)); //特征值1*3矩阵
            cv::Mat matV(3, 3, CV_32F, cv::Scalar::all(0)); //特征向量3*3矩阵
            cv::Mat matV2(3, 3, CV_32F, cv::Scalar::all(0));

            cv::eigen(matAtA, matE, matV); //求解特征值/特征向量
            matV.copyTo(matV2);

            isDegenerate = false;
            float eignThre[3] = {10, 10, 10}; //特征值取值门槛
            for (int i = 2; i >= 0; i--) { //从小到大查找
                if (matE.at<float>(0, i) < eignThre[i]) { //特征值太小，则认为处在兼并环境中，发生了退化
                    for (int j = 0; j < 3; j++) { //对应的特征向量置为0
                        matV2.at<float>(i, j) = 0;
                    }
                    isDegenerate = true;
                } else {
                    break;
                }
            }
            matP = matV.inv() * matV2; //计算P矩阵
        }

        //如果发生退化，只使用预测矩阵P计算
        if (isDegenerate) {
            cv::Mat matX2(3, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = matP * matX2;
        }
        //------- (bug fix: sometime the L-M optimization result matX contains NaN, which will break the whole node)
        /*if (isnan(matX.at<float>(0, 0)) || isnan(matX.at<float>(1, 0)) || isnan(matX.at<float>(2, 0)) || isnan(matX.at<float>(3, 0)) || isnan(matX.at<float>(4, 0)) || isnan(matX.at<float>(5, 0)))
        {
        printf("[USER WARN]laser Odometry: NaN found in var \"matX\", this L-M optimization step is going to be ignored.\n");
        }
        */ // 原版的loam这里有个bug修正

        // 更新第一步优化的结果[tz,roll,pitch]
        /*—————— matX代表的是每一次迭代的变化值detX,tranform代表累计后的迭代最新结果 ——————*/
        transformCur[0] += matX.at<float>(0, 0);    //pitch 
        transformCur[2] += matX.at<float>(1, 0);    // roll 
        transformCur[4] += matX.at<float>(2, 0);    // tz

        for(int i=0; i<6; i++){
            if(isnan(transformCur[i]))//判断是否非数字
                transformCur[i]=0;
        }
        //计算旋转平移量，如果很小就停止迭代
        float deltaR = sqrt(
                            pow(rad2deg(matX.at<float>(0, 0)), 2) +
                            pow(rad2deg(matX.at<float>(1, 0)), 2));
        float deltaT = sqrt(
                            pow(matX.at<float>(2, 0) * 100, 2));

        if (deltaR < 0.1 && deltaT < 0.1) {//迭代终止条件
            return false;
        }
        return true;
    }

    bool calculateTransformationCorner(int iterCount){

        int pointSelNum = laserCloudOri->points.size();

        cv::Mat matA(pointSelNum, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(3, pointSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(3, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(pointSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(3, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(3, 1, CV_32F, cv::Scalar::all(0));

        float srx = sin(transformCur[0]); // pitch 
        float crx = cos(transformCur[0]);
        float sry = sin(transformCur[1]); //yaw
        float cry = cos(transformCur[1]);
        float srz = sin(transformCur[2]); //roll
        float crz = cos(transformCur[2]);
        float tx = transformCur[3];
        float ty = transformCur[4];
        float tz = transformCur[5];

        float b1 = -crz*sry - cry*srx*srz; float b2 = cry*crz*srx - sry*srz; float b3 = crx*cry; float b4 = tx*-b1 + ty*-b2 + tz*b3;
        float b5 = cry*crz - srx*sry*srz; float b6 = cry*srz + crz*srx*sry; float b7 = crx*sry; float b8 = tz*b7 - ty*b6 - tx*b5;

        float c5 = crx*srz;

        for (int i = 0; i < pointSelNum; i++) {

            pointOri = laserCloudOri->points[i];
            coeff = coeffSel->points[i];

            float ary = (b1*pointOri.x + b2*pointOri.y - b3*pointOri.z + b4) * coeff.x
                      + (b5*pointOri.x + b6*pointOri.y - b7*pointOri.z + b8) * coeff.z;

            float atx = -b5 * coeff.x + c5 * coeff.y + b1 * coeff.z;

            float atz = b7 * coeff.x - srx * coeff.y - b3 * coeff.z;

            float d2 = coeff.intensity;

            matA.at<float>(i, 0) = ary;
            matA.at<float>(i, 1) = atx;
            matA.at<float>(i, 2) = atz;
            matB.at<float>(i, 0) = -0.05 * d2;
        }

        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        if (iterCount == 0) {
            cv::Mat matE(1, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matV(3, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matV2(3, 3, CV_32F, cv::Scalar::all(0));

            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);

            isDegenerate = false;
            float eignThre[3] = {10, 10, 10};
            for (int i = 2; i >= 0; i--) {
                if (matE.at<float>(0, i) < eignThre[i]) {
                    for (int j = 0; j < 3; j++) {
                        matV2.at<float>(i, j) = 0;
                    }
                    isDegenerate = true;
                } else {
                    break;
                }
            }
            matP = matV.inv() * matV2;
        }

        if (isDegenerate) {
            cv::Mat matX2(3, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = matP * matX2;
        }

        transformCur[1] += matX.at<float>(0, 0); // yaw 
        transformCur[3] += matX.at<float>(1, 0); // tx
        transformCur[5] += matX.at<float>(2, 0); // tz

        for(int i=0; i<6; i++){
            if(isnan(transformCur[i]))
                transformCur[i]=0;
        }

        float deltaR = sqrt(
                            pow(rad2deg(matX.at<float>(0, 0)), 2));
        float deltaT = sqrt(
                            pow(matX.at<float>(1, 0) * 100, 2) +
                            pow(matX.at<float>(2, 0) * 100, 2));

        if (deltaR < 0.1 && deltaT < 0.1) {
            return false;
        }
        return true;
    }

    bool calculateTransformation(int iterCount){

        int pointSelNum = laserCloudOri->points.size();

        cv::Mat matA(pointSelNum, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(6, pointSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(pointSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));

        float srx = sin(transformCur[0]);
        float crx = cos(transformCur[0]);
        float sry = sin(transformCur[1]);
        float cry = cos(transformCur[1]);
        float srz = sin(transformCur[2]);
        float crz = cos(transformCur[2]);
        float tx = transformCur[3];
        float ty = transformCur[4];
        float tz = transformCur[5];

        float a1 = crx*sry*srz; float a2 = crx*crz*sry; float a3 = srx*sry; float a4 = tx*a1 - ty*a2 - tz*a3;
        float a5 = srx*srz; float a6 = crz*srx; float a7 = ty*a6 - tz*crx - tx*a5;
        float a8 = crx*cry*srz; float a9 = crx*cry*crz; float a10 = cry*srx; float a11 = tz*a10 + ty*a9 - tx*a8;

        float b1 = -crz*sry - cry*srx*srz; float b2 = cry*crz*srx - sry*srz; float b3 = crx*cry; float b4 = tx*-b1 + ty*-b2 + tz*b3;
        float b5 = cry*crz - srx*sry*srz; float b6 = cry*srz + crz*srx*sry; float b7 = crx*sry; float b8 = tz*b7 - ty*b6 - tx*b5;

        float c1 = -b6; float c2 = b5; float c3 = tx*b6 - ty*b5; float c4 = -crx*crz; float c5 = crx*srz; float c6 = ty*c5 + tx*-c4;
        float c7 = b2; float c8 = -b1; float c9 = tx*-b2 - ty*-b1;

        for (int i = 0; i < pointSelNum; i++) {

            pointOri = laserCloudOri->points[i];
            coeff = coeffSel->points[i];

            float arx = (-a1*pointOri.x + a2*pointOri.y + a3*pointOri.z + a4) * coeff.x
                      + (a5*pointOri.x - a6*pointOri.y + crx*pointOri.z + a7) * coeff.y
                      + (a8*pointOri.x - a9*pointOri.y - a10*pointOri.z + a11) * coeff.z;

            float ary = (b1*pointOri.x + b2*pointOri.y - b3*pointOri.z + b4) * coeff.x
                      + (b5*pointOri.x + b6*pointOri.y - b7*pointOri.z + b8) * coeff.z;

            float arz = (c1*pointOri.x + c2*pointOri.y + c3) * coeff.x
                      + (c4*pointOri.x - c5*pointOri.y + c6) * coeff.y
                      + (c7*pointOri.x + c8*pointOri.y + c9) * coeff.z;

            float atx = -b5 * coeff.x + c5 * coeff.y + b1 * coeff.z;

            float aty = -b6 * coeff.x + c4 * coeff.y + b2 * coeff.z;

            float atz = b7 * coeff.x - srx * coeff.y - b3 * coeff.z;

            float d2 = coeff.intensity;

            matA.at<float>(i, 0) = arx;
            matA.at<float>(i, 1) = ary;
            matA.at<float>(i, 2) = arz;
            matA.at<float>(i, 3) = atx;
            matA.at<float>(i, 4) = aty;
            matA.at<float>(i, 5) = atz;
            matB.at<float>(i, 0) = -0.05 * d2;
        }

        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        if (iterCount == 0) {
            cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);

            isDegenerate = false;
            float eignThre[6] = {10, 10, 10, 10, 10, 10};
            for (int i = 5; i >= 0; i--) {
                if (matE.at<float>(0, i) < eignThre[i]) {
                    for (int j = 0; j < 6; j++) {
                        matV2.at<float>(i, j) = 0;
                    }
                    isDegenerate = true;
                } else {
                    break;
                }
            }
            matP = matV.inv() * matV2;
        }

        if (isDegenerate) {
            cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = matP * matX2;
        }

        transformCur[0] += matX.at<float>(0, 0);
        transformCur[1] += matX.at<float>(1, 0);
        transformCur[2] += matX.at<float>(2, 0);
        transformCur[3] += matX.at<float>(3, 0);
        transformCur[4] += matX.at<float>(4, 0);
        transformCur[5] += matX.at<float>(5, 0);

        for(int i=0; i<6; i++){
            if(isnan(transformCur[i]))
                transformCur[i]=0;
        }

        float deltaR = sqrt(
                            pow(rad2deg(matX.at<float>(0, 0)), 2) +
                            pow(rad2deg(matX.at<float>(1, 0)), 2) +
                            pow(rad2deg(matX.at<float>(2, 0)), 2));
        float deltaT = sqrt(
                            pow(matX.at<float>(3, 0) * 100, 2) +
                            pow(matX.at<float>(4, 0) * 100, 2) +
                            pow(matX.at<float>(5, 0) * 100, 2));

        if (deltaR < 0.1 && deltaT < 0.1) {
            return false;
        }
        return true;
    }

    void checkSystemInitialization(){ //将第一个点云数据集发送给laserMapping,从下一个点云数据开始处理

        //将cornerPointsLessSharp与laserCloudCornerLast交换,目的保存cornerPointsLessSharp的值下轮使用
        pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp; // 包含了边缘点的次边缘点
        cornerPointsLessSharp = laserCloudCornerLast; // 初始化时该指针没有指向的对象
        laserCloudCornerLast = laserCloudTemp; // 保存初始时刻的边缘点+次边缘点

        //将surfPointLessFlat与laserCloudSurfLast交换，目的保存surfPointsLessFlat的值下轮使用
        laserCloudTemp = surfPointsLessFlat; // 包含了地面平面点的次平面点
        surfPointsLessFlat = laserCloudSurfLast;
        laserCloudSurfLast = laserCloudTemp; // 保存初始时刻的地面平面点+次平面点

        //初始化时使用第一帧的特征点构建kd-tree，为了方便寻找最近的点
        kdtreeCornerLast->setInputCloud(laserCloudCornerLast); //所有的边缘点+次边缘点集合
        kdtreeSurfLast->setInputCloud(laserCloudSurfLast); //所有的地面平面点+次平面点集合

        laserCloudCornerLastNum = laserCloudCornerLast->points.size();
        laserCloudSurfLastNum = laserCloudSurfLast->points.size();

        //将cornerPointsLessSharp和surfPointLessFlat点也即边缘点+次边缘点和地面平面点+次平面点分别发送给laserMapping
        sensor_msgs::PointCloud2 laserCloudCornerLast2;
        pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
        laserCloudCornerLast2.header.stamp = cloudHeader.stamp; //这个时间是接收到点云的时间
        laserCloudCornerLast2.header.frame_id = "/camera";
        pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

        sensor_msgs::PointCloud2 laserCloudSurfLast2;
        pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
        laserCloudSurfLast2.header.stamp = cloudHeader.stamp;
        laserCloudSurfLast2.header.frame_id = "/camera";
        pubLaserCloudSurfLast.publish(laserCloudSurfLast2);

        //记录第一帧的翻滚角和俯仰角，位于世界坐标系下
        transformSum[0] += imuPitchStart;
        transformSum[2] += imuRollStart;

        systemInitedLM = true;
    }

    void updateInitialGuess(){

        imuPitchLast = imuPitchCur; // 此时为点云中最后一个点对应imu在世界坐标系下的姿态，R_w_i
        imuYawLast = imuYawCur;
        imuRollLast = imuRollCur;

        imuShiftFromStartX = imuShiftFromStartXCur; // 最后一个点相对于点云初始时刻(第一个点)的畸变位移，位于点云初始坐标系下
        imuShiftFromStartY = imuShiftFromStartYCur;
        imuShiftFromStartZ = imuShiftFromStartZCur;

        imuVeloFromStartX = imuVeloFromStartXCur; // 最后一个点相对于点云初始时刻(第一个点)的畸变速度，位于点云初始坐标系下
        imuVeloFromStartY = imuVeloFromStartYCur;
        imuVeloFromStartZ = imuVeloFromStartZCur;

        // 关于下面负号的说明：
        // transformCur是在Cur坐标系下的 p_start=R*p_cur+t
        // R和t是在Cur坐标系下的
        // 而imuAngularFromStart是在start坐标系下的，所以需要加负号
        if (imuAngularFromStartX != 0 || imuAngularFromStartY != 0 || imuAngularFromStartZ != 0){ // 角度有所变化
            transformCur[0] = - imuAngularFromStartY; // 变成负号就成了上一帧相对于当前帧转动的欧拉角，此时以当前帧为参照
            transformCur[1] = - imuAngularFromStartZ;
            transformCur[2] = - imuAngularFromStartX;
        }
        
        // 平移量的初值赋值为加减速的位移量，为其梯度下降的方向（沿用上次转换的T（一个sweep匀速模型），同时在其基础上减去匀速运动位移，即只考虑加减速的位移量）
        // 速度乘以时间，当前变换中的位移
        if (imuVeloFromStartX != 0 || imuVeloFromStartY != 0 || imuVeloFromStartZ != 0){
            transformCur[3] -= imuVeloFromStartX * scanPeriod;
            transformCur[4] -= imuVeloFromStartY * scanPeriod;
            transformCur[5] -= imuVeloFromStartZ * scanPeriod;
        }
    }

    void updateTransformation(){

        // 上一帧激光雷达点云数据中特征点的数量足够多再开始匹配
        if (laserCloudCornerLastNum < 10 || laserCloudSurfLastNum < 100)
            return;

        //Levenberg-Marquardt算法(L-M method)，非线性最小二乘算法迭代求解前后两帧的位姿变化
        // 这里采用的是两步LM优化方法：
        // 1. 通过匹配平面特征来估计出[tz,roll,pitch];
        // 2. 使用第一步估计值作为约束，匹配边缘特征来估计剩下的[tx,ty,yaw]

        for (int iterCount1 = 0; iterCount1 < 25; iterCount1++) {
            laserCloudOri->clear();
            coeffSel->clear();

            // 找到对应的特征平面
            // 然后计算协方差矩阵，保存在coeffSel队列中
            // laserCloudOri中保存的是对应于coeffSel的未转换到开始时刻的原始点云数据
            findCorrespondingSurfFeatures(iterCount1);

            if (laserCloudOri->points.size() < 10)
                continue;
            // 通过面特征的匹配，计算变换矩阵
            if (calculateTransformationSurf(iterCount1) == false)
                break;
        }

        for (int iterCount2 = 0; iterCount2 < 25; iterCount2++) {

            laserCloudOri->clear();
            coeffSel->clear();

            findCorrespondingCornerFeatures(iterCount2);

            if (laserCloudOri->points.size() < 10)
                continue;
            if (calculateTransformationCorner(iterCount2) == false)
                break;
        }
    }

    void integrateTransformation(){
        float rx, ry, rz, tx, ty, tz; //rx, ry, rz 为当前lidar在世界坐标系下的姿态欧拉角
        // AccumulateRotation作用
        // 将计算的两帧之间的位姿“累加”起来，获得相对于第一帧(世界坐标系)的旋转矩阵  transformSum + (-transformCur) =(rx,ry,rz)

        // transformSum 代表的是前一帧的终点到世界坐标系的位姿
        // transformCur表示当前帧起始点到终点的位姿，不是终点相对于起始点的位姿，这也是为什么参数里面transform要用负号的原因
        // AccumulateRotation计算的是当前帧的终点相对于世界坐标系的欧拉角rx,ry,rz
        // 要计算当前点相对于世界坐标系的欧拉角，先得计算出当前帧终点相对于世界坐标系的旋转矩阵R=Rtransformsum*RtransformCur

        //求相对于原点的旋转量,原版loam垂直方向上1.05倍修正
        // AccumulateRotation作用将局部旋转坐标转换至全局旋转坐标,转换方法为先计算R＝(Rcy*Rcx*Rcz)(Rly*Rlx*Rlz),然后对R中的元素解三角函数(此处仅仅求解旋转)
        AccumulateRotation(transformSum[0], transformSum[1], transformSum[2], 
                           -transformCur[0], -transformCur[1], -transformCur[2], rx, ry, rz);

        /******** 接着把lidar坐标系下的平移转移到世界坐标系下 *******/
        // 据转换矩阵 T = Ttransformsum  * TtransformCur  来求出平移偏量
        /*
        transformCur[i]的数组表示的当前帧的起点相对于终点的位姿，平移量也是一样，也是表示的当前帧的起点相对于终点的平移，
        实际上根据transformCur[i]里面的欧拉角计算出来的矩阵R应该是绕Y，X，Z的顺序计算出来的矩阵，而前面计算出来的RtransformCur表示的是当前帧的终点相对于起点旋转矩阵
        所以应该是R的转置(逆)，同样Ttransform就是T的逆，这样问题的第一步就变成了已知T计算T的逆
        又由于：
            T^-1 = | R^T  -R^T*t |
                   | 0^T    1    |

        T_w_l = | R_transformSum t_sum | * | R^T  -R^T*t | = | R_transformSum*R^T  -R_transformSum*R^T*t+t_sum | 
                |       0^T         1  |   | 0^T     1   |   |        0^T                       1              |
        其中，R_transformSum = R_transformSum*R^T
        */
        // 进行平移分量的更新
        float x1 = cos(rz) * (transformCur[3] - imuShiftFromStartX) 
                 - sin(rz) * (transformCur[4] - imuShiftFromStartY);
        float y1 = sin(rz) * (transformCur[3] - imuShiftFromStartX) 
                 + cos(rz) * (transformCur[4] - imuShiftFromStartY);
        float z1 = transformCur[5] - imuShiftFromStartZ; // 绕Z轴旋转

        float x2 = x1;
        float y2 = cos(rx) * y1 - sin(rx) * z1;
        float z2 = sin(rx) * y1 + cos(rx) * z1; // 绕X轴旋转

        //求相对于原点的平移量，其实就是世界坐标系下的平移
        tx = transformSum[3] - (cos(ry) * x2 + sin(ry) * z2); //1.先绕Y轴转 2. 减去相对平移
        ty = transformSum[4] - y2;
        tz = transformSum[5] - (-sin(ry) * x2 + cos(ry) * z2);

        // 与accumulateRotatio联合起来更新transformSum的rotation部分的工作
        // 可视为transformToEnd的下部分的逆过程

        //根据IMU修正旋转量 /////Q其实这里就可以用数据融合来做啦!!!比如预积分啊啥的
        //rx, ry, rz存放累计的到tend的旋转，imu分别存放到开始和到结束的旋转，是imu的测量数据。
        //目的：纠正精确化rx, ry, rz的值，imuEnd*inv(imuStart)*Sum表示在估计的里程计中附加IMU测量的旋转量。
        PluginIMURotation(rx, ry, rz, imuPitchStart, imuYawStart, imuRollStart, 
                          imuPitchLast, imuYawLast, imuRollLast, rx, ry, rz);

        //得到激光雷达(最后一个点)在世界坐标系下的位姿
        transformSum[0] = rx;
        transformSum[1] = ry;
        transformSum[2] = rz;
        transformSum[3] = tx;
        transformSum[4] = ty;
        transformSum[5] = tz;
    }

    void publishOdometry(){
        /* rz,rx,ry分别对应着标准右手坐标系中的roll,pitch,yaw角,通过查看createQuaternionMsgFromRollPitchYaw()的函数定义可以发现.
        * 当pitch和yaw角给负值后,四元数中的y和z会变成负值,x和w不受影响.由四元数定义可以知道,x,y,z是指旋转轴在三个轴上的投影,w影响
        * 旋转角度,所以由createQuaternionMsgFromRollPitchYaw()计算得到四元数后,其在一般右手坐标系中的x,y,z分量对应到该应用场景下
        * 的坐标系中,geoQuat.x对应实际坐标系下的z轴分量,geoQuat.y对应x轴分量,geoQuat.z对应实际的y轴分量,而由于rx和ry在计算四元数
        * 时给的是负值,所以geoQuat.y和geoQuat.z取负值,这样就等于没变(别人的解释)
        */
        //欧拉角转换成四元数
        geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(transformSum[2], -transformSum[0], -transformSum[1]);

        // rx,ry,rz转化为四元数发布
        laserOdometry.header.stamp = cloudHeader.stamp;
        laserOdometry.pose.pose.orientation.x = -geoQuat.y;
        laserOdometry.pose.pose.orientation.y = -geoQuat.z;
        laserOdometry.pose.pose.orientation.z = geoQuat.x;
        laserOdometry.pose.pose.orientation.w = geoQuat.w;
        laserOdometry.pose.pose.position.x = transformSum[3];
        laserOdometry.pose.pose.position.y = transformSum[4];
        laserOdometry.pose.pose.position.z = transformSum[5];
        pubLaserOdometry.publish(laserOdometry);

        // laserOdometryTrans 是用于tf广播
        laserOdometryTrans.stamp_ = cloudHeader.stamp;
        laserOdometryTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
        laserOdometryTrans.setOrigin(tf::Vector3(transformSum[3], transformSum[4], transformSum[5]));
        tfBroadcaster.sendTransform(laserOdometryTrans);
    }

    void adjustOutlierCloud(){
        PointType point;
        int cloudSize = outlierCloud->points.size();
        for (int i = 0; i < cloudSize; ++i)
        {
            point.x = outlierCloud->points[i].y;
            point.y = outlierCloud->points[i].z;
            point.z = outlierCloud->points[i].x;
            point.intensity = outlierCloud->points[i].intensity;
            outlierCloud->points[i] = point;
        }
    }

    void publishCloudsLast(){

        updateImuRollPitchYawStartSinCos();

        //对点云的曲率比较大和比较小的点投影到扫描结束位置
        int cornerPointsLessSharpNum = cornerPointsLessSharp->points.size();
        for (int i = 0; i < cornerPointsLessSharpNum; i++) {
            // TransformToEnd的作用是将k+1时刻的less特征点转移至k+1时刻的sweep的结束位置处的雷达坐标系下
            TransformToEnd(&cornerPointsLessSharp->points[i], &cornerPointsLessSharp->points[i]);
        }


        int surfPointsLessFlatNum = surfPointsLessFlat->points.size();
        for (int i = 0; i < surfPointsLessFlatNum; i++) {
            TransformToEnd(&surfPointsLessFlat->points[i], &surfPointsLessFlat->points[i]);
        }

        //畸变校正之后的点(投影至扫描终点)作为last点保存等下个点云进来进行匹配
        pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
        cornerPointsLessSharp = laserCloudCornerLast;
        laserCloudCornerLast = laserCloudTemp;

        laserCloudTemp = surfPointsLessFlat;
        surfPointsLessFlat = laserCloudSurfLast;
        laserCloudSurfLast = laserCloudTemp;

        laserCloudCornerLastNum = laserCloudCornerLast->points.size();
        laserCloudSurfLastNum = laserCloudSurfLast->points.size();
        
        //点足够多就构建kd-tree，否则弃用此帧，沿用上一帧数据的kd-tree
        if (laserCloudCornerLastNum > 10 && laserCloudSurfLastNum > 100) {
            kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
            kdtreeSurfLast->setInputCloud(laserCloudSurfLast);
        }

        frameCount++;
        //按照跳帧数publich边缘点，平面点以及全部点给laserMapping(每隔一帧发一次)
        if (frameCount >= skipFrameNum + 1) {

            frameCount = 0;
            // 调整坐标系，调整回来原始的样子
            adjustOutlierCloud();
            sensor_msgs::PointCloud2 outlierCloudLast2;
            pcl::toROSMsg(*outlierCloud, outlierCloudLast2);
            outlierCloudLast2.header.stamp = cloudHeader.stamp;
            outlierCloudLast2.header.frame_id = "/camera";
            pubOutlierCloudLast.publish(outlierCloudLast2);

            sensor_msgs::PointCloud2 laserCloudCornerLast2;
            pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
            laserCloudCornerLast2.header.stamp = cloudHeader.stamp;
            laserCloudCornerLast2.header.frame_id = "/camera";
            pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

            sensor_msgs::PointCloud2 laserCloudSurfLast2;
            pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
            laserCloudSurfLast2.header.stamp = cloudHeader.stamp;
            laserCloudSurfLast2.header.frame_id = "/camera";
            pubLaserCloudSurfLast.publish(laserCloudSurfLast2);
        }
    }

    void runFeatureAssociation()
    {
        // 如果有新数据进来则执行，否则不执行任何操作
        if (newSegmentedCloud && newSegmentedCloudInfo && newOutlierCloud &&
            std::abs(timeNewSegmentedCloudInfo - timeNewSegmentedCloud) < 0.05 &&
            std::abs(timeNewOutlierCloud - timeNewSegmentedCloud) < 0.05){ //同步作用，确保同时收到同一个点云的特征点信息才进入

            newSegmentedCloud = false;
            newSegmentedCloudInfo = false;
            newOutlierCloud = false;
        }else{
            return;
        }
        /**
        	1. Feature Extraction
        */
        // 主要进行的处理去除点云数据由于非匀速运动产生的畸变，并且将所有点都投影至sweep初始时刻
        // 注意!!! 这里去掉的只是因为不满足匀速模型(加减速运动)而产生的畸变，在后面TransformToStart()才会利用匀速模型去掉每个点的畸变 
        adjustDistortion();

        // 不完全按照公式进行光滑性计算，并保存结果
        calculateSmoothness();

        // 标记阻塞点??? 阻塞点是什么点???
        // 参考了csdn若愚maimai大佬的博客，这里的阻塞点指过近的点
        // 指在点云中可能出现的互相遮挡的情况
        markOccludedPoints();

        // 特征抽取，然后分别保存到cornerPointsSharp等等队列中去
        // 保存到不同的队列是不同类型的点云，进行了标记的工作，
        // 这一步中减少了点云数量，使计算量减少
        extractFeatures();

        // 发布cornerPointsSharp等4种类型的点云数据
        publishCloud(); // cloud for visualization
	
        /**
		2. Feature Association
        */
        if (!systemInitedLM) {
            checkSystemInitialization();
            return;
        }
         
        // 预测位姿
        updateInitialGuess();

        // 更新变换
        updateTransformation();

        // 积分总变换
        integrateTransformation();

        publishOdometry();

        publishCloudsLast(); // cloud to mapOptimization
    }
};



// 该节点只接收分割后的点云、离群点、及imu信息，然后对其进行处理
int main(int argc, char** argv)
{
    ros::init(argc, argv, "lego_loam");

    ROS_INFO("\033[1;32m---->\033[0m Feature Association Started.");

    FeatureAssociation FA;

    ros::Rate rate(200);
    while (ros::ok())
    {
        ros::spinOnce();

        FA.runFeatureAssociation();

        rate.sleep();
    }
    
    ros::spin();
    return 0;
}
