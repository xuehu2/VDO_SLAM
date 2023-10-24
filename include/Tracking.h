/**
* This file is part of VDO-SLAM.
*
* Copyright (C) 2019-2020 Jun Zhang <jun doc zhang2 at anu dot edu doc au> (The Australian National University)
* For more information see <https://github.com/halajun/VDO_SLAM>
*
**/


#ifndef TRACKING_H
#define TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

// eigen
#include <Eigen/Core>

#include"Map.h"
#include"Frame.h"
#include"ORBextractor.h"
#include "System.h"

#include <mutex>

namespace VDO_SLAM {

    using namespace std;

    class Map;

    class System;

    class Tracking {

        struct BirdEyeVizProperties {
            // Bird-eye visualization
            float birdeye_scale_factor_;
            int birdeye_far_plane_;
            int birdeye_left_plane_;
            int birdeye_right_plane_;

            BirdEyeVizProperties() {
                // Bird-eye visualization module params
                birdeye_scale_factor_ = 13.0;
                birdeye_far_plane_ = 50;
                birdeye_left_plane_ = -20;
                birdeye_right_plane_ = 20;
            }
        };

        struct LessPoint2f {
            bool operator()(const cv::Point2f &lhs, const cv::Point2f &rhs) const {
                return (lhs.x == rhs.x) ? (lhs.y < rhs.y) : (lhs.x < rhs.x);
            }
        };

    public:

        Tracking(System *pSys, Map *pMap, const string &strSettingPath, const int sensor);

        // Preprocess the input and call Track(). Extract features and performs stereo matching.
        cv::Mat GrabImageRGBD(const cv::Mat &imRGB, cv::Mat &imD, const cv::Mat &imFlow, const cv::Mat &maskSEM,
                              const cv::Mat &mTcw_gt, const vector<vector<float> > &vObjPose_gt,
                              const double &timestamp, cv::Mat &imTraj, const int &nImage);

        // Sparse Scene Flow Vector
        void GetSceneFlowObj();

        // Dynamic Object Tracking 动态目标跟踪
        std::vector<std::vector<int> > DynObjTracking();

        // For flow display on 2d plane
        void
        DrawLine(cv::KeyPoint &keys, cv::Point2f &flow, cv::Mat &ref_image, const cv::Scalar &color, int thickness = 2,
                 int line_type = 1, const cv::Point2i &offset = cv::Point2i(0, 0));

        void DrawTransparentSquare(cv::Point center, cv::Vec3b color, int radius, double alpha, cv::Mat &ref_image);

        void DrawGridBirdeye(double res_x, double res_z, const BirdEyeVizProperties &viz_props, cv::Mat &ref_image);

        void DrawSparseFlowBirdeye(const std::vector<Eigen::Vector3d> &pts, const std::vector<Eigen::Vector3d> &vel,
                                   const cv::Mat &camera, const BirdEyeVizProperties &viz_props, cv::Mat &ref_image);

        void TransformPointToScaledFrustum(double &pose_x, double &pose_z, const BirdEyeVizProperties &viz_props);

        cv::Mat ObjPoseParsingKT(const std::vector<float> &vObjPose_gt);

        cv::Mat ObjPoseParsingOX(const std::vector<float> &vObjPose_gt);

        cv::Mat GetInitModelCam(const std::vector<int> &MatchId, std::vector<int> &MatchId_sub);

        cv::Mat GetInitModelObj(const std::vector<int> &ObjId, std::vector<int> &ObjId_sub, const int objid);

        void StackObjInfo(std::vector<cv::KeyPoint> &FeatDynObj, std::vector<float> &DepDynObj,
                          std::vector<int> &FeatLabObj);

        std::vector<std::vector<std::pair<int, int> > > GetStaticTrack();

        std::vector<std::vector<std::pair<int, int> > > GetDynamicTrack();

        std::vector<std::vector<std::pair<int, int> > > GetDynamicTrackNew();

        std::vector<std::vector<int> >
        GetObjTrackTime(std::vector<std::vector<int> > &ObjTrackLab, std::vector<std::vector<int> > &ObjSemanticLab,
                        std::vector<std::vector<int> > &vnSMLabGT);

        void GetMetricError(const std::vector<cv::Mat> &CamPose, const std::vector<std::vector<cv::Mat> > &RigMot,
                            const std::vector<std::vector<cv::Mat> > &ObjPosePre,
                            const std::vector<cv::Mat> &CamPose_gt, const std::vector<std::vector<cv::Mat> > &RigMot_gt,
                            const std::vector<std::vector<bool> > &ObjStat);

        void PlotMetricError(const std::vector<cv::Mat> &CamPose, const std::vector<std::vector<cv::Mat> > &RigMot,
                             const std::vector<std::vector<cv::Mat> > &ObjPosePre,
                             const std::vector<cv::Mat> &CamPose_gt,
                             const std::vector<std::vector<cv::Mat> > &RigMot_gt,
                             const std::vector<std::vector<bool> > &ObjStat);

        void GetVelocityError(const std::vector<std::vector<cv::Mat> > &RigMot,
                              const std::vector<std::vector<cv::Mat> > &PointDyn,
                              const std::vector<std::vector<int> > &FeaLab, const std::vector<std::vector<int> > &RMLab,
                              const std::vector<std::vector<float> > &Velo_gt,
                              const std::vector<std::vector<int> > &TmpMatch,
                              const std::vector<std::vector<bool> > &ObjStat);

        void RenewFrameInfo(const std::vector<int> &TM_sta);

        void UpdateMask();


    public:

        // Tracking states 跟踪状态
        enum eTrackingState {
            NO_IMAGES_YET = 0, NOT_INITIALIZED = 1, OK = 2,
        };

        eTrackingState mState;  //跟踪状态
        eTrackingState mLastProcessedState;

        // Dataset Selection  数据集选择
        enum eDataState {
            OMD = 1, KITTI = 2, VirtualKITTI = 3,
        };

        eDataState mTestData;  //测试数据集选择，yaml文件配置

        // Input sensor 输入传感器类型
        int mSensor;

        // Current Frame 当前帧
        Frame mCurrentFrame;
        cv::Mat mImGray;  // 左图 or 灰度图

        // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        // todo 上一帧的左图 or 灰度图
        cv::Mat mImGrayLast;  // ++++++ new added

        // new added (Nov 14 2019)
        cv::Mat mFlowMap, mFlowMapLast; // 当前的光流图和上一帧的光流图
        cv::Mat mDepthMap; //深度地图
        cv::Mat mSegMap, mSegMapLast;   // 当前语义分割图和上一帧语义分割图

        // transfer the ground truth to use identity matrix as origin 原点
        cv::Mat mOriginInv;      // 起始帧的世界坐标 groundtruth

        // 判断是初始帧还是跟踪过程中的帧到帧
        bool bFrame2Frame, bFirstFrame;
        // Store temperal matching feature index
        // 存储匹配的关键点索引 和 匹配的关键点中内点的索引
        std::vector<int> TemperalMatch, TemperalMatch_subset;  // ++++++ new added
        // 前一帧的特征点，当前帧的特征点
        std::vector<cv::KeyPoint> mvKeysLastFrame, mvKeysCurrentFrame;  // ++++++ new added

        std::vector<cv::KeyPoint> mvTmpObjKeys;     // 由光流跟踪的物体的关键点
        std::vector<float> mvTmpObjDepth;           // 由光流跟踪的物体的关键点的深度
        std::vector<int> mvTmpSemObjLabel;          // 由光流跟踪的物体的关键点的语义标签
        std::vector<cv::Point2f> mvTmpObjFlowNext;  //
        std::vector<cv::KeyPoint> mvTmpObjCorres;   //

        // re-projection error   重投影误差
        std::vector<float> repro_e;

        // save current frame ID   保存当前帧的id
        int f_id;

        // save the global Tracking ID    保存全局跟踪id
        int max_id;

        // save stop frame   保存停止帧的id
        int StopFrame;

        // save optimization decision
        bool bLocalBatch;   //是否启用局部ba
        bool bGlobalBatch;  //是否启用全局ba
        // whether use joint optic-flow formulation
        bool bJoint;

        // Window Size and Overlapping Size for Local Batch Optimization
        int nWINDOW_SIZE, nOVERLAP_SIZE;

        // Max Tracking Points on Background and Object in each frame
        int nMaxTrackPointBG, nMaxTrackPointOBJ;    //最大跟踪点数量

        // Scene Flow Magnitude and Distribution Threshold
        float fSFMgThres, fSFDsThres;               //场景流阈值

        // save timing values
        std::vector<float> all_timing; // 计时器

        // use sampled feature or detected feature for background
        int nUseSampleFea; // 是否使用采样的关键点

        // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    protected:

        // Main tracking function. It is independent of the input sensor.
        void Track();

        // 跟踪的新方法
        void TrackBack();

        // 初始化
        void Initialization();

        // 新的初始化方法
        void InitializationBack();

        //ORB特征提取器
        ORBextractor *mpORBextractorLeft, *mpORBextractorRight;

        // System
        System *mpSystem;

        // 地图
        Map *mpMap;

        //Calibration matrix 标定的内参矩阵
        cv::Mat mK;
        cv::Mat mDistCoef;
        float mbf;

        // Threshold close/far points
        // Points seen as close by the stereo/RGBD sensor are considered reliable
        float mThDepth; // 近远点判断阈值
        float mThDepthObj;  // 物体的近远点判断阈值

        // The depth map scale factor.
        float mDepthMapFactor;  //深度的尺度因子

        //Current matches in frame
        int mnMatchesInliers;  // 当前的匹配内点

        //Last Frame Info
        Frame mLastFrame;  // 前1帧

        //Motion Model 运动模型
        cv::Mat mVelocity;  // 帧间的变换(速度)

        //Color order (true RGB, false BGR, ignored if grayscale)
        bool mbRGB;         //图像格式
    };

} //namespace VDO_SLAM

#endif // TRACKING_H
