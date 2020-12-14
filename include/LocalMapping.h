/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H
#include "KeyFrame.h"
#include "Atlas.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"
#include "Initializer.h"

#include <mutex>


namespace ORB_SLAM3
{

class System;
class Tracking;
class LoopClosing;
class Atlas;

class LocalMapping
{
public:
    /**
     * @brief LocalMaping构造函数 Construct a new Local Mapping:: Local Mapping object
     * 
     * @param pSys 
     * @param pAtlas 
     * @param bMonocular 
     * @param bInertial 
     * @param _strSeqName 
     */
    LocalMapping(System* pSys, Atlas* pAtlas, const float bMonocular, bool bInertial, const string &_strSeqName=std::string());

    void SetLoopCloser(LoopClosing* pLoopCloser);

    void SetTracker(Tracking* pTracker);

    // Main function
    /**
     * @brief LocalMapping线程主函数
     * 
     */
    void Run();
    /**
     * @brief 被Tracking线程调用，向LocalMapping中插入关键帧，
     * 
     * @param pKF 被插入的关键帧
     */
    void InsertKeyFrame(KeyFrame* pKF);
    void EmptyQueue();

    // Thread Synch
    /**
     * @brief 外部线程调用,请求停止当前线程的工作 
     * 
     */
    void RequestStop();
    /**
     * @brief 请求当前线程复位,由外部线程调用
     * 
     */
    void RequestReset();
    void RequestResetActiveMap(Map* pMap);
    bool Stop();
    void Release();
    bool isStopped();
    bool stopRequested();
    bool AcceptKeyFrames();
    void SetAcceptKeyFrames(bool flag);
    bool SetNotStop(bool flag);

    void InterruptBA();

    void RequestFinish();
    bool isFinished();

    int KeyframesInQueue(){
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

    bool IsInitializing();
    double GetCurrKFTime();
    KeyFrame* GetCurrKF();

    std::mutex mMutexImuInit;

    Eigen::MatrixXd mcovInertial;
    Eigen::Matrix3d mRwg;
    Eigen::Vector3d mbg;
    Eigen::Vector3d mba;
    double mScale;
    double mInitTime;
    double mCostTime;
    bool mbNewInit;
    unsigned int mInitSect;
    unsigned int mIdxInit;
    unsigned int mnKFs;
    double mFirstTs;
    int mnMatchesInliers;

    // For debugging (erase in normal mode)
    int mInitFr;
    int mIdxIteration;
    string strSequence;

    bool mbNotBA1;
    bool mbNotBA2;
    bool mbBadImu;

    bool mbWriteStats;

    // not consider far points (clouds)
    bool mbFarPoints;
    float mThFarPoints;
protected:
    /**
     * @brief 检查队列中是否有关键帧
     * 
     * @return true 有关键帧
     * @return false 没有关键帧
     */
    bool CheckNewKeyFrames();
    /**
     * @brief 处理缓冲关键帧列表中的关键帧
     * 
     */
    void ProcessNewKeyFrame();
    /**
     * @brief 当前关键帧和其共视程度较高关键帧之间通过三角化生成一些MapPoints
     * 
     */
    void CreateNewMapPoints();
    /**
     * @brief  剔除ProcessNewKeyFrame()中引入的不合格的地图点
     * 
     */
    void MapPointCulling();
    /**
     * @brief 检查并融合当前关键帧与相邻帧（两级相邻）重复的MapPoints
     * 
     */
    void SearchInNeighbors();
    void KeyFrameCulling();

    /**
     * @brief 通过两个关键帧的位姿计算两个关键帧之间的基本矩阵
     * 
     * @param pKF1 关键帧1
     * @param pKF2 关键帧2
     * @return cv::Mat 基本矩阵
     */
    cv::Mat ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);

    cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

    System *mpSystem;

    bool mbMonocular;
    bool mbInertial;

    void ResetIfRequested();
    bool mbResetRequested;
    bool mbResetRequestedActiveMap;
    Map* mpMapToReset;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;
    // 指向Atlas句柄
    Atlas* mpAtlas;
    // 指向回环检测句柄
    LoopClosing* mpLoopCloser;
    // 指向Tracking线程句柄
    Tracking* mpTracker;

    //Tracking线程插入的关键帧保存在这个列表中
    std::list<KeyFrame*> mlNewKeyFrames;
    // 当前处理的关键帧
    KeyFrame* mpCurrentKeyFrame;
    
    // 最近添加的地图点列表
    std::list<MapPoint*> mlpRecentAddedMapPoints;

    // 操作关键帧列表时使用的互斥量 
    std::mutex mMutexNewKFs;

    //终止BA的标志
    bool mbAbortBA;
    // 当前线程是否已经真正地终止了
    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;

    bool mbAcceptKeyFrames;
    std::mutex mMutexAccept;

    void InitializeIMU(float priorG = 1e2, float priorA = 1e6, bool bFirst = false);
    void ScaleRefinement();

    bool bInitializing;

    Eigen::MatrixXd infoInertial;
    int mNumLM;
    int mNumKFCulling;

    float mTinit;

    int countRefinement;

    //DEBUG
    ofstream f_lm;
};

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
