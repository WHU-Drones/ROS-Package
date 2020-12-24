#include "PointCloudGenerator.h"
#include "System.h"
#include "Converter.h"
#include "KeyFrame.h"

PointCloudGenerator::PointCloudGenerator(double resolution, double meank, double thresh):
    mResolution(resolution), mMeanK(meank), mThresh(thresh)
{
    mStatisticalFilter.setMeanK(mMeanK);
    mStatisticalFilter.setStddevMulThresh(mThresh);
    mVoxel.setLeafSize(mResolution, mResolution, mResolution);
    mGlobalMap = boost::make_shared<PointCloud>();
}

PointCloudGenerator::~PointCloudGenerator()
{
}

void PointCloudGenerator::InsertKeyFrame(KeyFrame* NewKeyFrame, cv::Mat& Color, cv::Mat& Depth, vector<KeyFrame*> CurrentKeyFrames)
{
    unique_lock<mutex> lock(mKeyFrameMutex);
    mKeyFrames.push_back(NewKeyFrame);
    mCurrentKeyFrames = CurrentKeyFrames;

    PointCloudStruct CurrentFramePC;
    CurrentFramePC.mID = NewKeyFrame->mnFrameId;
    CurrentFramePC.mT = ORB_SLAM3::Converter::toSE3Quat(NewKeyFrame->GetPose());
    CurrentFramePC.mPC = GenerateKeyFramePC(NewKeyFrame, Color, Depth);
    mPointClouds.push_back(CurrentFramePC);
    mGlobalMapCondition.notify_one();
}

PointCloudGenerator::PointCloud::Ptr GenerateKeyFramePC(KeyFrame* NewKeyFrame, cv::Mat& Color, cv::Mat& Depth)
{
    PointCloudGenerator::PointCloud::Ptr NewKeyFramePC(new PointCloudGenerator::PointCloud());
    for (int m = 0; m < Depth.rows; m += 3)
    {
        for (int n = 0; n < Depth.cols; n += 3)
        {
            float d = Depth.ptr<float>(m)[n];
            if (d < 0.01 || d > 5)
                continue;

            PointCloudGenerator::PointType point;
            point.z = d;
            point.x = (n - NewKeyFrame->cx) * point.z / NewKeyFrame->fx;
            point.y = (m - NewKeyFrame->cy) * point.z / NewKeyFrame->fy;
            point.b = Color.ptr<uchar>(m)[n*3];
            point.g = Color.ptr<uchar>(m)[n*3+1];
            point.r = Color.ptr<uchar>(m)[n*3+2];

            NewKeyFramePC->push_back(point);
        }
    }
    return NewKeyFramePC;
}

void PointCloudGenerator::PCGenerator()
{
    while (1)
    {
        {
            unique_lock<mutex> lock1(mGlobalMapMutex);
            mGlobalMapCondition.wait(lock1);
        }
        if (mLoopCloseMutex.try_lock())
        {
            unique_lock<mutex> lock2(mKeyFrameMutex);
            size_t N = mKeyFrames.size();
            if(N != mLastKeyFrameSize)
            {
                for (size_t i = mLastKeyFrameSize; i < N; i++)
                {
                    PointCloud::Ptr pc(new PointCloud);
                    pcl::transformPointCloud(*(mPointClouds[i].mPC), *pc, mPointClouds[i].mT.inverse().matrix());
                    *mGlobalMap += *pc;
                }

                PointCloud::Ptr tmp(new PointCloud);
                mStatisticalFilter.setInputCloud(mGlobalMap);
                mStatisticalFilter.filter(*tmp);
                mVoxel.setInputCloud(tmp);
                mVoxel.filter(*mGlobalMap);
                mLastKeyFrameSize = N;
            }
            mLoopCloseMutex.unlock();
        }
    }
}

void PointCloudGenerator::PCLoopClose()
{
    if (mLoopCloseMutex.try_lock())
    {
        PointCloud::Ptr tmp1(new PointCloud);
        for (int i = 0; i < mCurrentKeyFrames.size(); i++)
        {
            for (int j = 0; j < mPointClouds.size(); j++)
            {
                if (mPointClouds[j].mID == mCurrentKeyFrames[i]->mnFrameId)
                {
                    Eigen::Isometry3d T = ORB_SLAM3::Converter::toSE3Quat(mCurrentKeyFrames[i]->GetPose());
                    PointCloud::Ptr pc(new PointCloud);
                    pcl::transformPointCloud(*(mPointClouds[i].mPC), *pc, T.inverse().matrix());
                    *tmp1 += *pc;
                    break;
                }
            }
        }
        PointCloud::Ptr tmp2(new PointCloud);
        mVoxel.setInputCloud(tmp1);
        mVoxel.filter(*tmp2);
        mGlobalMap->swap(*tmp2);
        
        mLoopCloseMutex.unlock();
    }
}