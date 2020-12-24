#ifndef POINTCLOUD_GENERATOR_H
#define POINTCLOUD_GENERATOR_H

#include "System.h"
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <condition_variable>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace ORB_SLAM3;

class PointCloudStruct
{
    typedef pcl::PointXYZRGBA PointType;
    typedef pcl::PointCloud<PointType> PointCloud;
public:
    PointCloud::Ptr mPC;
    Eigen::Isometry3d mT;
    int mID; 
};

class PointCloudGenerator
{
public:
    typedef pcl::PointXYZRGBA PointType;
    typedef pcl::PointCloud<PointType> PointCloud;  
    
    PointCloudGenerator(double resolution, double meank, double thresh);
    ~PointCloudGenerator();
    void InsertKeyFrame(KeyFrame* NewKeyFrame, cv::Mat& Color, cv::Mat& Depth, vector<KeyFrame*> vKeyFrames);
    void PCGenerator();
    void PCLoopClose();
protected:
    PointCloud::Ptr GenerateKeyFramePC(KeyFrame* NewKeyFrame, cv::Mat& Color, cv::Mat& Depth);

    double mResolution = 0.04;
    double mMeanK = 50;
    double mThresh = 1;
    pcl::VoxelGrid<PointType> mVoxel;
    pcl::StatisticalOutlierRemoval<PointType> mStatisticalFilter;

    PointCloud::Ptr mGlobalMap;
    vector<KeyFrame*> mKeyFrames;
    vector<KeyFrame*> mCurrentKeyFrames;
    vector<PointCloudStruct> mPointClouds;
    uint16_t mLastKeyFrameSize = 0;

    condition_variable mGlobalMapCondition;
    mutex mGlobalMapMutex;
    mutex mLoopCloseMutex;
    mutex mKeyFrameMutex;
};

#endif // POINTCLOUD_GENERATOR_H
