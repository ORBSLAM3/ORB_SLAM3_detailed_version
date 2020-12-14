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


#include "LocalMapping.h"
#include "LoopClosing.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include "Converter.h"

#include<mutex>
#include<chrono>

namespace ORB_SLAM3
{
/**
 * @brief LocalMaping构造函数 Construct a new Local Mapping:: Local Mapping object
 * 
 * @param pSys 
 * @param pAtlas 
 * @param bMonocular 
 * @param bInertial 
 * @param _strSeqName 
 */
LocalMapping::LocalMapping(System* pSys, Atlas *pAtlas, const float bMonocular, bool bInertial, const string &_strSeqName):
    mpSystem(pSys), mbMonocular(bMonocular), mbInertial(bInertial), mbResetRequested(false), mbResetRequestedActiveMap(false), mbFinishRequested(false), mbFinished(true), mpAtlas(pAtlas), bInitializing(false),
    mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true),
    mbNewInit(false), mIdxInit(0), mScale(1.0), mInitSect(0), mbNotBA1(true), mbNotBA2(true), mIdxIteration(0), infoInertial(Eigen::MatrixXd::Zero(9,9))
{
    mnMatchesInliers = 0;

    mbBadImu = false;

    mTinit = 0.f;

    mNumLM = 0;
    mNumKFCulling=0;

    //DEBUG: times and data from LocalMapping in each frame

    strSequence = "";//_strSeqName;
    //f_lm.open("localMapping_times" + strSequence + ".txt");
    /*f_lm.open("localMapping_times.txt");

    f_lm << "# Timestamp KF, Num CovKFs, Num KFs, Num RecentMPs, Num MPs, processKF, MPCulling, CreateMP, SearchNeigh, BA, KFCulling, [numFixKF_LBA]" << endl;
    f_lm << fixed;*/
}

void LocalMapping::SetLoopCloser(LoopClosing* pLoopCloser)
{
    mpLoopCloser = pLoopCloser;
}

void LocalMapping::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}
/**
 * @brief LocalMapping线程主函数
 * 
 */
void LocalMapping::Run()
{
    //mbFinished为fasel，表示当前run函数正在运行
    mbFinished = false;

    while(1)
    {
        //告诉Tracking线程，LocalMapping正在忙，不能接收关键帧
        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(false);

        //1.检查队列中是否有关键帧
        // Check if there are keyframes in the queue
        if(CheckNewKeyFrames() && !mbBadImu)
        {
            // std::cout << "LM" << std::endl;
            std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
            
            //2.计算关键帧特征点的BoW映射，将关键帧插入地图
            // BoW conversion and insertion in Map
            ProcessNewKeyFrame();
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
            //3.剔除上一步ProcessNewKeyFrame()中引入的不合格的地图点
            // Check recent MapPoints
            MapPointCulling();
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
            //4.三角化生成新的地图点
            // Triangulate new MapPoints
            CreateNewMapPoints();
            std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();

            // Save here:
            // # Cov KFs
            // # tot Kfs
            // # recent added MPs
            // # tot MPs
            // # localMPs in LBA
            // # fixedKFs in LBA

            mbAbortBA = false;//终止BA的标志
            
            // 如果已经处理完队列中最后一个关键帧
            if(!CheckNewKeyFrames())
            {
                //检查并融合当前关键帧与相邻帧（两级相邻）重复的MapPoints
                // Find more matches in neighbor keyframes and fuse point duplications
                SearchInNeighbors();
            }

            std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
            std::chrono::steady_clock::time_point t5 = t4, t6 = t4;
            // mbAbortBA = false;

            //DEBUG--
            /*f_lm << setprecision(0);
            f_lm << mpCurrentKeyFrame->mTimeStamp*1e9 << ",";
            f_lm << mpCurrentKeyFrame->GetVectorCovisibleKeyFrames().size() << ",";
            f_lm << mpCurrentKeyFrame->GetMap()->GetAllKeyFrames().size() << ",";
            f_lm << mlpRecentAddedMapPoints.size() << ",";
            f_lm << mpCurrentKeyFrame->GetMap()->GetAllMapPoints().size() << ",";*/
            //--
            int num_FixedKF_BA = 0;

            if(!CheckNewKeyFrames() && !stopRequested())
            {
                if(mpAtlas->KeyFramesInMap()>2)
                {
                    if(mbInertial && mpCurrentKeyFrame->GetMap()->isImuInitialized())
                    {
                        float dist = cv::norm(mpCurrentKeyFrame->mPrevKF->GetCameraCenter() - mpCurrentKeyFrame->GetCameraCenter()) +
                                cv::norm(mpCurrentKeyFrame->mPrevKF->mPrevKF->GetCameraCenter() - mpCurrentKeyFrame->mPrevKF->GetCameraCenter());

                        if(dist>0.05)
                            mTinit += mpCurrentKeyFrame->mTimeStamp - mpCurrentKeyFrame->mPrevKF->mTimeStamp;
                        if(!mpCurrentKeyFrame->GetMap()->GetIniertialBA2())
                        {
                            if((mTinit<10.f) && (dist<0.02))
                            {
                                cout << "Not enough motion for initializing. Reseting..." << endl;
                                unique_lock<mutex> lock(mMutexReset);
                                mbResetRequestedActiveMap = true;
                                mpMapToReset = mpCurrentKeyFrame->GetMap();
                                mbBadImu = true;
                            }
                        }

                        bool bLarge = ((mpTracker->GetMatchesInliers()>75)&&mbMonocular)||((mpTracker->GetMatchesInliers()>100)&&!mbMonocular);
                        Optimizer::LocalInertialBA(mpCurrentKeyFrame, &mbAbortBA, mpCurrentKeyFrame->GetMap(), bLarge, !mpCurrentKeyFrame->GetMap()->GetIniertialBA2());
                    }
                    else
                    {
                        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
                        Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame,&mbAbortBA, mpCurrentKeyFrame->GetMap(),num_FixedKF_BA);
                        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
                    }
                }

                t5 = std::chrono::steady_clock::now();

                // Initialize IMU here
                if(!mpCurrentKeyFrame->GetMap()->isImuInitialized() && mbInertial)
                {
                    if (mbMonocular)
                        InitializeIMU(1e2, 1e10, true);
                    else
                        InitializeIMU(1e2, 1e5, true);
                }


                // Check redundant local Keyframes
                KeyFrameCulling();

                t6 = std::chrono::steady_clock::now();

                if ((mTinit<100.0f) && mbInertial)
                {
                    if(mpCurrentKeyFrame->GetMap()->isImuInitialized() && mpTracker->mState==Tracking::OK) // Enter here everytime local-mapping is called
                    {
                        if(!mpCurrentKeyFrame->GetMap()->GetIniertialBA1()){
                            if (mTinit>5.0f)
                            {
                                cout << "start VIBA 1" << endl;
                                mpCurrentKeyFrame->GetMap()->SetIniertialBA1();
                                if (mbMonocular)
                                    InitializeIMU(1.f, 1e5, true); // 1.f, 1e5
                                else
                                    InitializeIMU(1.f, 1e5, true); // 1.f, 1e5

                                cout << "end VIBA 1" << endl;
                            }
                        }
                        //else if (mbNotBA2){
                        else if(!mpCurrentKeyFrame->GetMap()->GetIniertialBA2()){
                            if (mTinit>15.0f){ // 15.0f
                                cout << "start VIBA 2" << endl;
                                mpCurrentKeyFrame->GetMap()->SetIniertialBA2();
                                if (mbMonocular)
                                    InitializeIMU(0.f, 0.f, true); // 0.f, 0.f
                                else
                                    InitializeIMU(0.f, 0.f, true);

                                cout << "end VIBA 2" << endl;
                            }
                        }

                        // scale refinement
                        if (((mpAtlas->KeyFramesInMap())<=100) &&
                                ((mTinit>25.0f && mTinit<25.5f)||
                                (mTinit>35.0f && mTinit<35.5f)||
                                (mTinit>45.0f && mTinit<45.5f)||
                                (mTinit>55.0f && mTinit<55.5f)||
                                (mTinit>65.0f && mTinit<65.5f)||
                                (mTinit>75.0f && mTinit<75.5f))){
                            cout << "start scale ref" << endl;
                            if (mbMonocular)
                                ScaleRefinement();
                            cout << "end scale ref" << endl;
                        }
                    }
                }
            }

            std::chrono::steady_clock::time_point t7 = std::chrono::steady_clock::now();

            mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);
            std::chrono::steady_clock::time_point t8 = std::chrono::steady_clock::now();

            double t_procKF = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t1 - t0).count();
            double t_MPcull = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
            double t_CheckMP = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t3 - t2).count();
            double t_searchNeigh = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t4 - t3).count();
            double t_Opt = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t5 - t4).count();
            double t_KF_cull = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t6 - t5).count();
            double t_Insert = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t8 - t7).count();

            //DEBUG--
            /*f_lm << setprecision(6);
            f_lm << t_procKF << ",";
            f_lm << t_MPcull << ",";
            f_lm << t_CheckMP << ",";
            f_lm << t_searchNeigh << ",";
            f_lm << t_Opt << ",";
            f_lm << t_KF_cull << ",";
            f_lm << setprecision(0) << num_FixedKF_BA << "\n";*/
            //--

        }
        else if(Stop() && !mbBadImu)
        {
            // Safe area to stop
            while(isStopped() && !CheckFinish())
            {
                // cout << "LM: usleep if is stopped" << endl;
                usleep(3000);
            }
            if(CheckFinish())
                break;
        }

        ResetIfRequested();

        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(true);

        if(CheckFinish())
            break;

        // cout << "LM: normal usleep" << endl;
        usleep(3000);
    }

    //f_lm.close();

    SetFinish();
}
/**
 * @brief 被Tracking线程调用，向LocalMapping中插入关键帧，
 * 
 * @param pKF 被插入的关键帧
 */
void LocalMapping::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexNewKFs);
    mlNewKeyFrames.push_back(pKF);
    mbAbortBA=true;
}

/**
 * @brief 检查队列中是否有关键帧
 * 
 * @return true 有关键帧
 * @return false 没有关键帧
 */
bool LocalMapping::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    return(!mlNewKeyFrames.empty());
}
/**
 * @brief 处理缓冲关键帧列表中的关键帧
 * 
 */
void LocalMapping::ProcessNewKeyFrame()
{
    //1.从缓冲关键帧队列中取出一帧
    //cout << "ProcessNewKeyFrame: " << mlNewKeyFrames.size() << endl;
    {
        unique_lock<mutex> lock(mMutexNewKFs);
        mpCurrentKeyFrame = mlNewKeyFrames.front();
        mlNewKeyFrames.pop_front();
    }
    //2.计算关键帧特征的的Bow向量,
    // Compute Bags of Words structures
    mpCurrentKeyFrame->ComputeBoW();

    //3.跟踪局部地图过程中新匹配上的MapPoints和当前关键帧绑定
    // Associate MapPoints to the new keyframe and update normal and descriptor
    const vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    //遍历当前关键帧地图点
    for(size_t i=0; i<vpMapPointMatches.size(); i++)
    {
        MapPoint* pMP = vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())//由于某些原因,当前关键帧观测到的某个地图点会被删除
            {
                if(!pMP->IsInKeyFrame(mpCurrentKeyFrame))// 地图点不是当前帧生成的，则更新观测
                {
                    //^ todo 添加观测
                    pMP->AddObservation(mpCurrentKeyFrame, i);
                    //^todo 更新该点的评价观测放心和观测距离范围
                    pMP->UpdateNormalAndDepth();
                    //^todo 计算更新地图点最佳描述子
                    pMP->ComputeDistinctiveDescriptors();
                }
                else // this can only happen for new stereo points inserted by the Tracking
                {
                    // 当前帧中已经包含这个地图点，这些由当前帧生成的地图点不靠谱，将其加入到待检查的地图点列表中
                    // 这些地图点不靠谱是为什么，我是这样理解的因为如果这个地图点是当前关键帧生成的，则就是新插入的，就需要检查一下，也就是不是部分点需要检查，而是只要是新生成的点都需要检查一下。
                    mlpRecentAddedMapPoints.push_back(pMP);
                }
            }
        }
    }
    //^todo 4.更新关键帧间的连接关系，Covisibility图和Essential图(tree)
    // Update links in the Covisibility Graph
    mpCurrentKeyFrame->UpdateConnections();
    //^todo  5.将关键帧插入地图中
    // Insert Keyframe in Map
    mpAtlas->AddKeyFrame(mpCurrentKeyFrame);
}

void LocalMapping::EmptyQueue()
{
    while(CheckNewKeyFrames())
        ProcessNewKeyFrame();
}
/**
 * @brief  剔除ProcessNewKeyFrame()中引入的不合格的地图点
 * 
 */
void LocalMapping::MapPointCulling()
{
    // 待检查地图点队列进行遍历
    // Check Recent Added MapPoints
    list<MapPoint*>::iterator lit = mlpRecentAddedMapPoints.begin();
    const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

    int nThObs;
    if(mbMonocular)
        nThObs = 2;
    else
        nThObs = 3; // MODIFICATION_STEREO_IMU here 3
    const int cnThObs = nThObs;

    int borrar = mlpRecentAddedMapPoints.size();
    //开始遍历待检查的地图点
    while(lit!=mlpRecentAddedMapPoints.end())
    {
        MapPoint* pMP = *lit;

        if(pMP->isBad())// 1.如果地图点是坏点，则从队列中剔除
            lit = mlpRecentAddedMapPoints.erase(lit);
        else if(pMP->GetFoundRatio()<0.25f)//2.地图点能和帧中特征点匹配上的图像帧数/地图点在多少图像帧视野范围内(不只关键帧)要大于25%
        {
            pMP->SetBadFlag();//设置成坏点
            lit = mlpRecentAddedMapPoints.erase(lit);// 从待检查队列中剔除出去
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=2 && pMP->Observations()<=cnThObs)//3.当前关键帧和生成该地图点的关键帧已经相距大于等于2，并且观测到该点的关键帧小于阈值，则剔除
        {
            pMP->SetBadFlag();//设置成坏点
            lit = mlpRecentAddedMapPoints.erase(lit);// 从待检查队列中剔除出去
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=3)//4.从建立该点开始，已经过了3个关键帧而没有被剔除，则认为是质量高的点
            lit = mlpRecentAddedMapPoints.erase(lit);
        else//其他情况，则在待检查队列中保留该点，下一个关键帧的时候还要检查该点。
        {
            lit++;
            borrar--;//todo 应该是没用
        }
    }
    //cout << "erase MP: " << borrar << endl;
}
/**
 * @brief 当前关键帧和其共视程度较高关键帧之间通过三角化生成一些MapPoints
 * 
 */
void LocalMapping::CreateNewMapPoints()
{
    // Retrieve neighbor keyframes in covisibility graph
    // 用于获取共视帧时的参数，获取共视程度最高的nn帧，
    // 不同传感器不同，双目获取10帧，单目获取20帧，单目需要更好的共视关系的关键帧来建立地图
    int nn = 10;
    // For stereo inertial case
    if(mbMonocular)
        nn=20;
    //1.在当前关键帧的共视关键帧中取出共视程度最高的nn帧
    vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
    //如果有IMU
    if (mbInertial)
    {
        KeyFrame* pKF = mpCurrentKeyFrame;
        int count=0;
        while((vpNeighKFs.size()<=nn)&&(pKF->mPrevKF)&&(count++<nn))//如果共视程度高的帧不够nn帧，则依次把之前的关键帧放入，直到凑够nn帧
        {
            vector<KeyFrame*>::iterator it = std::find(vpNeighKFs.begin(), vpNeighKFs.end(), pKF->mPrevKF);//在共视程度高的帧中查找当前帧的上一关键帧
            if(it==vpNeighKFs.end())// 上一关键帧不在共视关键帧中，则加入到其中
                vpNeighKFs.push_back(pKF->mPrevKF);
            pKF = pKF->mPrevKF;
        }
    }

    float th = 0.6f;

    ORBmatcher matcher(th,false);

    cv::Mat Rcw1 = mpCurrentKeyFrame->GetRotation();
    cv::Mat Rwc1 = Rcw1.t();
    cv::Mat tcw1 = mpCurrentKeyFrame->GetTranslation();
    cv::Mat Tcw1(3,4,CV_32F);
    Rcw1.copyTo(Tcw1.colRange(0,3));
    tcw1.copyTo(Tcw1.col(3));

    //获取当前关键帧在世界坐标系下的坐标
    cv::Mat Ow1 = mpCurrentKeyFrame->GetCameraCenter();
    //获取当前帧的位姿
    const float &fx1 = mpCurrentKeyFrame->fx;
    const float &fy1 = mpCurrentKeyFrame->fy;
    const float &cx1 = mpCurrentKeyFrame->cx;
    const float &cy1 = mpCurrentKeyFrame->cy;
    const float &invfx1 = mpCurrentKeyFrame->invfx;
    const float &invfy1 = mpCurrentKeyFrame->invfy;
    
    // 用于后面的点深度的验证;这里为什么选1.5也不清楚,猜测还是各种经验值吧
    const float ratioFactor = 1.5f*mpCurrentKeyFrame->mfScaleFactor;
    
    // 遍历相邻关键帧
    // Search matches with epipolar restriction and triangulate
    for(size_t i=0; i<vpNeighKFs.size(); i++)
    {
        //下面生成地图点操作可能会比较耗时，所以如果有新的关键帧需要处理，就先去处理新的关键帧
        if(i>0 && CheckNewKeyFrames())// && (mnMatchesInliers>50))
            return;

        KeyFrame* pKF2 = vpNeighKFs[i];

        GeometricCamera* pCamera1 = mpCurrentKeyFrame->mpCamera, *pCamera2 = pKF2->mpCamera;
        // 获取相邻帧在世界坐标系下的坐标
        // Check first that baseline is not too short
        cv::Mat Ow2 = pKF2->GetCameraCenter();
        // 通过帧之间的坐标计算基线向量
        cv::Mat vBaseline = Ow2-Ow1;
        // 计算基线范数，即基线长度
        const float baseline = cv::norm(vBaseline);

        if(!mbMonocular)
        {
            if(baseline<pKF2->mb)// 两个关键帧之间基线长度不能小于双目相机基线长度
            continue;
        }
        else
        {
            //获取当前相邻关键帧场景深度中值
            const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
            //baseline和景深的比例
            const float ratioBaselineDepth = baseline/medianDepthKF2;
            //比值太大则不适合生成3D点，可能性1是两个关键帧之间基线太短，基线太短能够恢复的地图点深度十分有限；可能性2是相邻帧看到的空间深度非常大，这时生成的3D点不准确
            if(ratioBaselineDepth<0.01)
                continue;
        }
        // 计算当前关键帧到相邻关键帧的基础矩阵
        // Compute Fundamental Matrix
        cv::Mat F12 = ComputeF12(mpCurrentKeyFrame,pKF2);
        // 通过极线约束限制匹配时的搜索范围，进行特征点匹配
        // Search matches that fullfil epipolar constraint
        vector<pair<size_t,size_t> > vMatchedIndices;
        bool bCoarse = mbInertial &&
                ((!mpCurrentKeyFrame->GetMap()->GetIniertialBA2() && mpCurrentKeyFrame->GetMap()->GetIniertialBA1())||
                 mpTracker->mState==Tracking::RECENTLY_LOST);
        matcher.SearchForTriangulation(mpCurrentKeyFrame,pKF2,F12,vMatchedIndices,false,bCoarse);

        cv::Mat Rcw2 = pKF2->GetRotation();
        cv::Mat Rwc2 = Rcw2.t();
        cv::Mat tcw2 = pKF2->GetTranslation();
        cv::Mat Tcw2(3,4,CV_32F);
        Rcw2.copyTo(Tcw2.colRange(0,3));
        tcw2.copyTo(Tcw2.col(3));

        const float &fx2 = pKF2->fx;
        const float &fy2 = pKF2->fy;
        const float &cx2 = pKF2->cx;
        const float &cy2 = pKF2->cy;
        const float &invfx2 = pKF2->invfx;
        const float &invfy2 = pKF2->invfy;
        
        //对每对匹配点进行三角化，生成3D点 
        // Triangulate each match
        const int nmatches = vMatchedIndices.size();
        for(int ikp=0; ikp<nmatches; ikp++)
        {
            // 获取匹配点在两关键帧中的索引
            const int &idx1 = vMatchedIndices[ikp].first;
            const int &idx2 = vMatchedIndices[ikp].second;

            // 获取当前匹配点在当前关键帧中的特征点(为什么这么写没看懂呢)
            const cv::KeyPoint &kp1 = (mpCurrentKeyFrame -> NLeft == -1) ? mpCurrentKeyFrame->mvKeysUn[idx1]
                                                                         : (idx1 < mpCurrentKeyFrame -> NLeft) ? mpCurrentKeyFrame -> mvKeys[idx1]
                                                                                                               : mpCurrentKeyFrame -> mvKeysRight[idx1 - mpCurrentKeyFrame -> NLeft];
            // mvuRight中存放着双目的深度值，如果不是双目，其值将为-1
            const float kp1_ur=mpCurrentKeyFrame->mvuRight[idx1];
            bool bStereo1 = (!mpCurrentKeyFrame->mpCamera2 && kp1_ur>=0);
            const bool bRight1 = (mpCurrentKeyFrame -> NLeft == -1 || idx1 < mpCurrentKeyFrame -> NLeft) ? false
                                                                               : true;
            // 获取当前匹配点在相邻关键帧中的特征点
            const cv::KeyPoint &kp2 = (pKF2 -> NLeft == -1) ? pKF2->mvKeysUn[idx2]
                                                            : (idx2 < pKF2 -> NLeft) ? pKF2 -> mvKeys[idx2]
                                                                                     : pKF2 -> mvKeysRight[idx2 - pKF2 -> NLeft];

            const float kp2_ur = pKF2->mvuRight[idx2];
            bool bStereo2 = (!pKF2->mpCamera2 && kp2_ur>=0);
            const bool bRight2 = (pKF2 -> NLeft == -1 || idx2 < pKF2 -> NLeft) ? false
                                                                               : true;

            if(mpCurrentKeyFrame->mpCamera2 && pKF2->mpCamera2){
                if(bRight1 && bRight2){
                    Rcw1 = mpCurrentKeyFrame->GetRightRotation();
                    Rwc1 = Rcw1.t();
                    tcw1 = mpCurrentKeyFrame->GetRightTranslation();
                    Tcw1 = mpCurrentKeyFrame->GetRightPose();
                    Ow1 = mpCurrentKeyFrame->GetRightCameraCenter();

                    Rcw2 = pKF2->GetRightRotation();
                    Rwc2 = Rcw2.t();
                    tcw2 = pKF2->GetRightTranslation();
                    Tcw2 = pKF2->GetRightPose();
                    Ow2 = pKF2->GetRightCameraCenter();

                    pCamera1 = mpCurrentKeyFrame->mpCamera2;
                    pCamera2 = pKF2->mpCamera2;
                }
                else if(bRight1 && !bRight2){
                    Rcw1 = mpCurrentKeyFrame->GetRightRotation();
                    Rwc1 = Rcw1.t();
                    tcw1 = mpCurrentKeyFrame->GetRightTranslation();
                    Tcw1 = mpCurrentKeyFrame->GetRightPose();
                    Ow1 = mpCurrentKeyFrame->GetRightCameraCenter();

                    Rcw2 = pKF2->GetRotation();
                    Rwc2 = Rcw2.t();
                    tcw2 = pKF2->GetTranslation();
                    Tcw2 = pKF2->GetPose();
                    Ow2 = pKF2->GetCameraCenter();

                    pCamera1 = mpCurrentKeyFrame->mpCamera2;
                    pCamera2 = pKF2->mpCamera;
                }
                else if(!bRight1 && bRight2){
                    Rcw1 = mpCurrentKeyFrame->GetRotation();
                    Rwc1 = Rcw1.t();
                    tcw1 = mpCurrentKeyFrame->GetTranslation();
                    Tcw1 = mpCurrentKeyFrame->GetPose();
                    Ow1 = mpCurrentKeyFrame->GetCameraCenter();

                    Rcw2 = pKF2->GetRightRotation();
                    Rwc2 = Rcw2.t();
                    tcw2 = pKF2->GetRightTranslation();
                    Tcw2 = pKF2->GetRightPose();
                    Ow2 = pKF2->GetRightCameraCenter();

                    pCamera1 = mpCurrentKeyFrame->mpCamera;
                    pCamera2 = pKF2->mpCamera2;
                }
                else{
                    Rcw1 = mpCurrentKeyFrame->GetRotation();
                    Rwc1 = Rcw1.t();
                    tcw1 = mpCurrentKeyFrame->GetTranslation();
                    Tcw1 = mpCurrentKeyFrame->GetPose();
                    Ow1 = mpCurrentKeyFrame->GetCameraCenter();

                    Rcw2 = pKF2->GetRotation();
                    Rwc2 = Rcw2.t();
                    tcw2 = pKF2->GetTranslation();
                    Tcw2 = pKF2->GetPose();
                    Ow2 = pKF2->GetCameraCenter();

                    pCamera1 = mpCurrentKeyFrame->mpCamera;
                    pCamera2 = pKF2->mpCamera;
                }
            }
            // 反投影：根据相机模型将像素进行投影，得到的是一个方向向量（未被归一化）
            // Check parallax between rays
            // 分别对匹配好的特征点（分属当前关键帧和相邻关键帧）的特征点进行反投影，得到投影向量
            cv::Mat xn1 = pCamera1->unprojectMat(kp1.pt);
            cv::Mat xn2 = pCamera2->unprojectMat(kp2.pt);

            // 将投影向量转换到世界坐标系下
            cv::Mat ray1 = Rwc1*xn1;
            cv::Mat ray2 = Rwc2*xn2;
            // 计算两个反投影向量之间的角度
            const float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));

            // 接下来几个量是用了判断反投影向量之间夹角大小的
            // 单目时：最大阈值为夹角+1（大于90度）
            float cosParallaxStereo = cosParallaxRays+1;
            float cosParallaxStereo1 = cosParallaxStereo;
            float cosParallaxStereo2 = cosParallaxStereo;
            
            // 双目时,根据双目成像原理（等腰三角形成像原理），计算双目左右帧图像特征点和地图点向量的夹角作为阈值
            if(bStereo1)//当前关键帧
                cosParallaxStereo1 = cos(2*atan2(mpCurrentKeyFrame->mb/2,mpCurrentKeyFrame->mvDepth[idx1]));
            else if(bStereo2)//相邻关键帧
                cosParallaxStereo2 = cos(2*atan2(pKF2->mb/2,pKF2->mvDepth[idx2]));
            // 用小阈值
            cosParallaxStereo = min(cosParallaxStereo1,cosParallaxStereo2);
            // 三角化恢复3D点，如果视差小则用三角法恢复3D点，视差角大时用双目恢复3D点（双目以及深度有效）
            cv::Mat x3D;
            if(cosParallaxRays<cosParallaxStereo && cosParallaxRays>0 && (bStereo1 || bStereo2 ||
               (cosParallaxRays<0.9998 && mbInertial) || (cosParallaxRays<0.9998 && !mbInertial)))//视差小
            {
                //线性三角化
                // Linear Triangulation Method
                cv::Mat A(4,4,CV_32F);
                A.row(0) = xn1.at<float>(0)*Tcw1.row(2)-Tcw1.row(0);
                A.row(1) = xn1.at<float>(1)*Tcw1.row(2)-Tcw1.row(1);
                A.row(2) = xn2.at<float>(0)*Tcw2.row(2)-Tcw2.row(0);
                A.row(3) = xn2.at<float>(1)*Tcw2.row(2)-Tcw2.row(1);

                // SVD分解
                cv::Mat w,u,vt;
                cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
                //得到(X,Y,Z,W)
                x3D = vt.row(3).t();
                //如果W=0,则不要这个点
                if(x3D.at<float>(3)==0)
                    continue;
                // 归一化到欧式空间 (X/W,Y/W,Z/W,W/W)=(x,y,z,1)
                // Euclidean coordinates
                x3D = x3D.rowRange(0,3)/x3D.at<float>(3);

            }
            else if(bStereo1 && cosParallaxStereo1<cosParallaxStereo2)
            {
                x3D = mpCurrentKeyFrame->UnprojectStereo(idx1);//视差大的时候使用双目信息来恢复 - 直接反投影了
            }
            else if(bStereo2 && cosParallaxStereo2<cosParallaxStereo1)
            {
                x3D = pKF2->UnprojectStereo(idx2);// 同上
            }
            else
            {
                continue; //No stereo and very low parallax
            }

            cv::Mat x3Dt = x3D.t();

            if(x3Dt.empty()) continue;
            // 检查3D点是否在相机前面
            //Check triangulation in front of cameras
            // 验证3D点和当前关键帧
            float z1 = Rcw1.row(2).dot(x3Dt)+tcw1.at<float>(2);
            if(z1<=0)
                continue;
            // 验证3D点和相邻关键帧
            float z2 = Rcw2.row(2).dot(x3Dt)+tcw2.at<float>(2);
            if(z2<=0)
                continue;
            // 计算3D点在当前关键帧下的重投影误差
            //Check reprojection error in first keyframe
            const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
            const float x1 = Rcw1.row(0).dot(x3Dt)+tcw1.at<float>(0);
            const float y1 = Rcw1.row(1).dot(x3Dt)+tcw1.at<float>(1);
            const float invz1 = 1.0/z1;

            if(!bStereo1)
            {
                cv::Point2f uv1 = pCamera1->project(cv::Point3f(x1,y1,z1));
                float errX1 = uv1.x - kp1.pt.x;
                float errY1 = uv1.y - kp1.pt.y;
                //基于卡方检验计算出的阈值（假设测量有一个像素的偏差）自由度2(X,Y)
                if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1)
                    continue;

            }
            else //有右目，卡方检验计算出的阈值会多一个自由度(X,Y,X_R)
            {
                float u1 = fx1*x1*invz1+cx1;
                float u1_r = u1 - mpCurrentKeyFrame->mbf*invz1;// 根据视差公式计算假想的右目坐标
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                float errX1_r = u1_r - kp1_ur;
                if((errX1*errX1+errY1*errY1+errX1_r*errX1_r)>7.8*sigmaSquare1)
                    continue;
            }
            // 计算3D点在另一个关键帧下的重投影误差
            //Check reprojection error in second keyframe
            const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
            const float x2 = Rcw2.row(0).dot(x3Dt)+tcw2.at<float>(0);
            const float y2 = Rcw2.row(1).dot(x3Dt)+tcw2.at<float>(1);
            const float invz2 = 1.0/z2;
            if(!bStereo2)
            {
                cv::Point2f uv2 = pCamera2->project(cv::Point3f(x2,y2,z2));
                float errX2 = uv2.x - kp2.pt.x;
                float errY2 = uv2.y - kp2.pt.y;
                if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2)
                    continue;
            }
            else
            {
                float u2 = fx2*x2*invz2+cx2;
                float u2_r = u2 - mpCurrentKeyFrame->mbf*invz2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                float errX2_r = u2_r - kp2_ur;
                if((errX2*errX2+errY2*errY2+errX2_r*errX2_r)>7.8*sigmaSquare2)
                    continue;
            }
            // 世界坐标系下，3D点与相机间的向量，方向由相机指向3D点
            //Check scale consistency
            cv::Mat normal1 = x3D-Ow1;
            float dist1 = cv::norm(normal1);//距离范数

            cv::Mat normal2 = x3D-Ow2;
            float dist2 = cv::norm(normal2);

            if(dist1==0 || dist2==0)
                continue;
            if(mbFarPoints && (dist1>=mThFarPoints||dist2>=mThFarPoints)) // MODIFICATION
                continue;
            //ratioDist是不考虑金字塔尺度下的距离比例
            const float ratioDist = dist2/dist1;
            // 金字塔尺度因子的比例
            const float ratioOctave = mpCurrentKeyFrame->mvScaleFactors[kp1.octave]/pKF2->mvScaleFactors[kp2.octave];
            //^ 还不是非常明白,感觉这里的意思大致应该就是, 深度值的比例和图像金字塔的比例不应该差太多
            if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
                continue;

            // Triangulation is succesfull
            // 三角化成功，构造新的地图点
            MapPoint* pMP = new MapPoint(x3D,mpCurrentKeyFrame,mpAtlas->GetCurrentMap());
            // 添加观测
            // 可观测到该点的关键帧（即当前帧和相邻帧）
            pMP->AddObservation(mpCurrentKeyFrame,idx1);            
            pMP->AddObservation(pKF2,idx2);
            // 关键帧和相邻关键帧可以观测到该点
            mpCurrentKeyFrame->AddMapPoint(pMP,idx1);
            pKF2->AddMapPoint(pMP,idx2);
            // 计算描述子
            pMP->ComputeDistinctiveDescriptors();
            // 更新该地图点的平均观测方向和深度范围
            pMP->UpdateNormalAndDepth();
            // 向地图中添加该点
            mpAtlas->AddMapPoint(pMP);
            //将新产生的点放入检测队列
            // 这些MapPoints都会经过MapPointCulling函数的检验
            mlpRecentAddedMapPoints.push_back(pMP);
        }
    }
}

/**
 * @brief 检查并融合当前关键帧与相邻帧（两级相邻）重复的MapPoints
 * 
 */
void LocalMapping::SearchInNeighbors()
{
    // Retrieve neighbor keyframes
    int nn = 10;
    if(mbMonocular)
        nn=20;
    // 获取一级相邻帧和二级相邻帧
    // 获取一级相邻帧：获取当前关键帧在共视图中共视程度最高的前nn帧作为一级相邻帧
    const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
    vector<KeyFrame*> vpTargetKFs;
    // 遍历一级相邻帧，进行筛选（如果帧是坏帧或已经和当前帧进行过融合则不作为一级相邻帧）
    for(vector<KeyFrame*>::const_iterator vit=vpNeighKFs.begin(), vend=vpNeighKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        // 判断是否和当前帧进行过融合
        if(pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId)
            continue;
        vpTargetKFs.push_back(pKFi);
        pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;
    }

    // Add some covisible of covisible
    // Extend to some second neighbors if abort is not requested
    // 获取二级相邻帧：遍历一级相邻帧，一级相邻帧的共视帧为二级相邻帧
    for(int i=0, imax=vpTargetKFs.size(); i<imax; i++)
    {
        // 获取当前一级相邻帧共视程度最好的20帧作为二级相邻帧候选
        const vector<KeyFrame*> vpSecondNeighKFs = vpTargetKFs[i]->GetBestCovisibilityKeyFrames(20);
        // 对二级相邻帧候选进行遍历并筛选（和一级相邻帧类似）
        for(vector<KeyFrame*>::const_iterator vit2=vpSecondNeighKFs.begin(), vend2=vpSecondNeighKFs.end(); vit2!=vend2; vit2++)
        {
            KeyFrame* pKFi2 = *vit2;
            if(pKFi2->isBad() || pKFi2->mnFuseTargetForKF==mpCurrentKeyFrame->mnId || pKFi2->mnId==mpCurrentKeyFrame->mnId)
                continue;
            vpTargetKFs.push_back(pKFi2);// 合格的二级相邻帧也加到相邻帧列表中
            pKFi2->mnFuseTargetForKF=mpCurrentKeyFrame->mnId;
        }
        if (mbAbortBA)
            break;
    }
    // 如果是有惯导，并且一级加二级相邻帧不足20，则把之前的关键帧加进来
    // Extend to temporal neighbors
    if(mbInertial)
    {
        KeyFrame* pKFi = mpCurrentKeyFrame->mPrevKF;//指向上一关键帧
        while(vpTargetKFs.size()<20 && pKFi)
        {
            if(pKFi->isBad() || pKFi->mnFuseTargetForKF==mpCurrentKeyFrame->mnId)// 上一帧和当前帧融合过，这里不用单独判断是否已经在相邻帧列表中，因为判断是否融合过就可以顺便判断
            {
                pKFi = pKFi->mPrevKF;
                continue;
            }
            vpTargetKFs.push_back(pKFi);//加入到杏林子列表中
            pKFi->mnFuseTargetForKF=mpCurrentKeyFrame->mnId;
            pKFi = pKFi->mPrevKF;//指向上一关键帧
        }
    }
    // 将当前帧的MapPoints分别与相邻帧(的MapPoints)进行融合 -- 正向
    // Search matches by projection from current KF in target KFs
    ORBmatcher matcher;
    // 获取当前帧能匹配的地图点
    vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    // 遍历相邻帧
    for(vector<KeyFrame*>::iterator vit=vpTargetKFs.begin(), vend=vpTargetKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        // 投影当前帧的MapPoints到相邻关键帧pKFi中，并判断是否有重复的MapPoints
        // 1.如果MapPoint能匹配关键帧的特征点，并且该点有对应的MapPoint，那么将两个MapPoint合并（选择观测数多的）
        // 2.如果MapPoint能匹配关键帧的特征点，并且该点没有对应的MapPoint，那么为该点添加MapPoint
        // 注意这个时候对地图点融合的操作是立即生效的
        matcher.Fuse(pKFi,vpMapPointMatches);
        // todo
        if(pKFi->NLeft != -1) matcher.Fuse(pKFi,vpMapPointMatches,true);
    }

    if (mbAbortBA)
        return;
    // 相邻帧MapPoints分别与当前帧(的MapPoints)进行融合 -- 反向
    // Search matches by projection from target KFs in current KF
    vector<MapPoint*> vpFuseCandidates;
    vpFuseCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size());
    // 对相邻帧进行遍历
    for(vector<KeyFrame*>::iterator vitKF=vpTargetKFs.begin(), vendKF=vpTargetKFs.end(); vitKF!=vendKF; vitKF++)
    {
        KeyFrame* pKFi = *vitKF;
        // 获取相邻帧能匹配的地图点
        vector<MapPoint*> vpMapPointsKFi = pKFi->GetMapPointMatches();
        // 遍历当前相邻帧所有地图点，
        for(vector<MapPoint*>::iterator vitMP=vpMapPointsKFi.begin(), vendMP=vpMapPointsKFi.end(); vitMP!=vendMP; vitMP++)
        {
            MapPoint* pMP = *vitMP;
            if(!pMP)
                continue;
            // 如果该点是坏点或者已经加入待融合地图点集合中了，则跳过该点
            if(pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->mnId)
                continue;
            pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->mnId;
            vpFuseCandidates.push_back(pMP);//加入到待融合地图点集合中
        }
    }
    // 当前帧和待融合地图点进行融合
    matcher.Fuse(mpCurrentKeyFrame,vpFuseCandidates);
    if(mpCurrentKeyFrame->NLeft != -1) matcher.Fuse(mpCurrentKeyFrame,vpFuseCandidates,true);


    // Update points
    // 更新地图点描述子，观测方向和平均深度
    vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
    {
        MapPoint* pMP=vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                pMP->ComputeDistinctiveDescriptors();
                pMP->UpdateNormalAndDepth();
            }
        }
    }
    // 更新当前帧的MapPoints后更新与其它帧的连接关系,地图点和帧之间的观测发生了变换，所有共视图之间的连接关系要发生变换
    // Update connections in covisibility graph
    mpCurrentKeyFrame->UpdateConnections();
}
/**
 * @brief 通过两个关键帧的位姿计算两个关键帧之间的基本矩阵
 * 
 * @param pKF1 关键帧1
 * @param pKF2 关键帧2
 * @return cv::Mat 基本矩阵
 */
cv::Mat LocalMapping::ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2)
{
    // 获取关键帧1在世界坐标系下的旋转矩阵和平移向量
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();
    // 获取关键帧2在世界坐标系下的旋转矩阵和平移向量
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();
    
    // 计算关键帧1到关键帧2的旋转矩阵和平移向量
    cv::Mat R12 = R1w*R2w.t();
    cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;
    // 把向量转成反对称矩阵的形式
    cv::Mat t12x = SkewSymmetricMatrix(t12);

    // 获取两帧图像的内参矩阵
    const cv::Mat &K1 = pKF1->mpCamera->toK();
    const cv::Mat &K2 = pKF2->mpCamera->toK();

    // 计算基础矩阵
    return K1.t().inv()*t12x*R12*K2.inv();
}

void LocalMapping::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true;
    unique_lock<mutex> lock2(mMutexNewKFs);
    mbAbortBA = true;
}

bool LocalMapping::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(mbStopRequested && !mbNotStop)
    {
        mbStopped = true;
        cout << "Local Mapping STOP" << endl;
        return true;
    }

    return false;
}

bool LocalMapping::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool LocalMapping::stopRequested()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
}

void LocalMapping::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);
    if(mbFinished)
        return;
    mbStopped = false;
    mbStopRequested = false;
    for(list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
        delete *lit;
    mlNewKeyFrames.clear();

    cout << "Local Mapping RELEASE" << endl;
}

bool LocalMapping::AcceptKeyFrames()
{
    unique_lock<mutex> lock(mMutexAccept);
    return mbAcceptKeyFrames;
}

void LocalMapping::SetAcceptKeyFrames(bool flag)
{
    unique_lock<mutex> lock(mMutexAccept);
    mbAcceptKeyFrames=flag;
}

bool LocalMapping::SetNotStop(bool flag)
{
    unique_lock<mutex> lock(mMutexStop);

    if(flag && mbStopped)
        return false;

    mbNotStop = flag;

    return true;
}

void LocalMapping::InterruptBA()
{
    mbAbortBA = true;
}

void LocalMapping::KeyFrameCulling()
{
    // Check redundant keyframes (only local keyframes)
    // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
    // in at least other 3 keyframes (in the same or finer scale)
    // We only consider close stereo points
    const int Nd = 21; // MODIFICATION_STEREO_IMU 20 This should be the same than that one from LIBA
    mpCurrentKeyFrame->UpdateBestCovisibles();
    vector<KeyFrame*> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

    float redundant_th;
    if(!mbInertial)
        redundant_th = 0.9;
    else if (mbMonocular)
        redundant_th = 0.9;
    else
        redundant_th = 0.5;

    const bool bInitImu = mpAtlas->isImuInitialized();
    int count=0;

    // Compoute last KF from optimizable window:
    unsigned int last_ID;
    if (mbInertial)
    {
        int count = 0;
        KeyFrame* aux_KF = mpCurrentKeyFrame;
        while(count<Nd && aux_KF->mPrevKF)
        {
            aux_KF = aux_KF->mPrevKF;
            count++;
        }
        last_ID = aux_KF->mnId;
    }



    for(vector<KeyFrame*>::iterator vit=vpLocalKeyFrames.begin(), vend=vpLocalKeyFrames.end(); vit!=vend; vit++)
    {
        count++;
        KeyFrame* pKF = *vit;

        if((pKF->mnId==pKF->GetMap()->GetInitKFid()) || pKF->isBad())
            continue;
        const vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();

        int nObs = 3;
        const int thObs=nObs;
        int nRedundantObservations=0;
        int nMPs=0;
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    if(!mbMonocular)
                    {
                        if(pKF->mvDepth[i]>pKF->mThDepth || pKF->mvDepth[i]<0)
                            continue;
                    }

                    nMPs++;
                    if(pMP->Observations()>thObs)
                    {
                        const int &scaleLevel = (pKF -> NLeft == -1) ? pKF->mvKeysUn[i].octave
                                                                     : (i < pKF -> NLeft) ? pKF -> mvKeys[i].octave
                                                                                          : pKF -> mvKeysRight[i].octave;
                        const map<KeyFrame*, tuple<int,int>> observations = pMP->GetObservations();
                        int nObs=0;
                        for(map<KeyFrame*, tuple<int,int>>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                        {
                            KeyFrame* pKFi = mit->first;
                            if(pKFi==pKF)
                                continue;
                            tuple<int,int> indexes = mit->second;
                            int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
                            int scaleLeveli = -1;
                            if(pKFi -> NLeft == -1)
                                scaleLeveli = pKFi->mvKeysUn[leftIndex].octave;
                            else {
                                if (leftIndex != -1) {
                                    scaleLeveli = pKFi->mvKeys[leftIndex].octave;
                                }
                                if (rightIndex != -1) {
                                    int rightLevel = pKFi->mvKeysRight[rightIndex - pKFi->NLeft].octave;
                                    scaleLeveli = (scaleLeveli == -1 || scaleLeveli > rightLevel) ? rightLevel
                                                                                                  : scaleLeveli;
                                }
                            }

                            if(scaleLeveli<=scaleLevel+1)
                            {
                                nObs++;
                                if(nObs>thObs)
                                    break;
                            }
                        }
                        if(nObs>thObs)
                        {
                            nRedundantObservations++;
                        }
                    }
                }
            }
        }

        if(nRedundantObservations>redundant_th*nMPs)
        {
            if (mbInertial)
            {
                if (mpAtlas->KeyFramesInMap()<=Nd)
                    continue;

                if(pKF->mnId>(mpCurrentKeyFrame->mnId-2))
                    continue;

                if(pKF->mPrevKF && pKF->mNextKF)
                {
                    const float t = pKF->mNextKF->mTimeStamp-pKF->mPrevKF->mTimeStamp;

                    if((bInitImu && (pKF->mnId<last_ID) && t<3.) || (t<0.5))
                    {
                        pKF->mNextKF->mpImuPreintegrated->MergePrevious(pKF->mpImuPreintegrated);
                        pKF->mNextKF->mPrevKF = pKF->mPrevKF;
                        pKF->mPrevKF->mNextKF = pKF->mNextKF;
                        pKF->mNextKF = NULL;
                        pKF->mPrevKF = NULL;
                        pKF->SetBadFlag();
                    }
                    else if(!mpCurrentKeyFrame->GetMap()->GetIniertialBA2() && (cv::norm(pKF->GetImuPosition()-pKF->mPrevKF->GetImuPosition())<0.02) && (t<3))
                    {
                        pKF->mNextKF->mpImuPreintegrated->MergePrevious(pKF->mpImuPreintegrated);
                        pKF->mNextKF->mPrevKF = pKF->mPrevKF;
                        pKF->mPrevKF->mNextKF = pKF->mNextKF;
                        pKF->mNextKF = NULL;
                        pKF->mPrevKF = NULL;
                        pKF->SetBadFlag();
                    }
                }
            }
            else
            {
                pKF->SetBadFlag();
            }
        }
        if((count > 20 && mbAbortBA) || count>100) // MODIFICATION originally 20 for mbabortBA check just 10 keyframes
        {
            break;
        }
    }
}

/**
 * @brief 将向量转成反对称矩阵形式
 * 
 * @param v 输入的向量
 * @return cv::Mat 反对称矩阵
 */
cv::Mat LocalMapping::SkewSymmetricMatrix(const cv::Mat &v)
{
    return (cv::Mat_<float>(3,3) <<             0, -v.at<float>(2), v.at<float>(1),
            v.at<float>(2),               0,-v.at<float>(0),
            -v.at<float>(1),  v.at<float>(0),              0);
}

void LocalMapping::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        cout << "LM: Map reset recieved" << endl;
        mbResetRequested = true;
    }
    cout << "LM: Map reset, waiting..." << endl;

    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetRequested)
                break;
        }
        usleep(3000);
    }
    cout << "LM: Map reset, Done!!!" << endl;
}

void LocalMapping::RequestResetActiveMap(Map* pMap)
{
    {
        unique_lock<mutex> lock(mMutexReset);
        cout << "LM: Active map reset recieved" << endl;
        mbResetRequestedActiveMap = true;
        mpMapToReset = pMap;
    }
    cout << "LM: Active map reset, waiting..." << endl;

    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetRequestedActiveMap)
                break;
        }
        usleep(3000);
    }
    cout << "LM: Active map reset, Done!!!" << endl;
}

void LocalMapping::ResetIfRequested()
{
    bool executed_reset = false;
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbResetRequested)
        {
            executed_reset = true;

            cout << "LM: Reseting Atlas in Local Mapping..." << endl;
            mlNewKeyFrames.clear();
            mlpRecentAddedMapPoints.clear();
            mbResetRequested=false;
            mbResetRequestedActiveMap = false;

            // Inertial parameters
            mTinit = 0.f;
            mbNotBA2 = true;
            mbNotBA1 = true;
            mbBadImu=false;

            mIdxInit=0;

            cout << "LM: End reseting Local Mapping..." << endl;
        }

        if(mbResetRequestedActiveMap) {
            executed_reset = true;
            cout << "LM: Reseting current map in Local Mapping..." << endl;
            mlNewKeyFrames.clear();
            mlpRecentAddedMapPoints.clear();

            // Inertial parameters
            mTinit = 0.f;
            mbNotBA2 = true;
            mbNotBA1 = true;
            mbBadImu=false;

            mbResetRequestedActiveMap = false;
            cout << "LM: End reseting Local Mapping..." << endl;
        }
    }
    if(executed_reset)
        cout << "LM: Reset free the mutex" << endl;

}

void LocalMapping::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LocalMapping::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LocalMapping::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;    
    unique_lock<mutex> lock2(mMutexStop);
    mbStopped = true;
}

bool LocalMapping::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void LocalMapping::InitializeIMU(float priorG, float priorA, bool bFIBA)
{
    if (mbResetRequested)
        return;

    float minTime;
    int nMinKF;
    if (mbMonocular)
    {
        minTime = 2.0;
        nMinKF = 10;
    }
    else
    {
        minTime = 1.0;
        nMinKF = 10;
    }


    if(mpAtlas->KeyFramesInMap()<nMinKF)
        return;

    // Retrieve all keyframe in temporal order
    list<KeyFrame*> lpKF;
    KeyFrame* pKF = mpCurrentKeyFrame;
    while(pKF->mPrevKF)
    {
        lpKF.push_front(pKF);
        pKF = pKF->mPrevKF;
    }
    lpKF.push_front(pKF);
    vector<KeyFrame*> vpKF(lpKF.begin(),lpKF.end());

    if(vpKF.size()<nMinKF)
        return;

    mFirstTs=vpKF.front()->mTimeStamp;
    if(mpCurrentKeyFrame->mTimeStamp-mFirstTs<minTime)
        return;

    bInitializing = true;

    while(CheckNewKeyFrames())
    {
        ProcessNewKeyFrame();
        vpKF.push_back(mpCurrentKeyFrame);
        lpKF.push_back(mpCurrentKeyFrame);
    }

    const int N = vpKF.size();
    IMU::Bias b(0,0,0,0,0,0);

    // Compute and KF velocities mRwg estimation
    if (!mpCurrentKeyFrame->GetMap()->isImuInitialized())
    {
        cv::Mat cvRwg;
        cv::Mat dirG = cv::Mat::zeros(3,1,CV_32F);
        for(vector<KeyFrame*>::iterator itKF = vpKF.begin(); itKF!=vpKF.end(); itKF++)
        {
            if (!(*itKF)->mpImuPreintegrated)
                continue;
            if (!(*itKF)->mPrevKF)
                continue;

            dirG -= (*itKF)->mPrevKF->GetImuRotation()*(*itKF)->mpImuPreintegrated->GetUpdatedDeltaVelocity();
            cv::Mat _vel = ((*itKF)->GetImuPosition() - (*itKF)->mPrevKF->GetImuPosition())/(*itKF)->mpImuPreintegrated->dT;
            (*itKF)->SetVelocity(_vel);
            (*itKF)->mPrevKF->SetVelocity(_vel);
        }

        dirG = dirG/cv::norm(dirG);
        cv::Mat gI = (cv::Mat_<float>(3,1) << 0.0f, 0.0f, -1.0f);
        cv::Mat v = gI.cross(dirG);
        const float nv = cv::norm(v);
        const float cosg = gI.dot(dirG);
        const float ang = acos(cosg);
        cv::Mat vzg = v*ang/nv;
        cvRwg = IMU::ExpSO3(vzg);
        mRwg = Converter::toMatrix3d(cvRwg);
        mTinit = mpCurrentKeyFrame->mTimeStamp-mFirstTs;
    }
    else
    {
        mRwg = Eigen::Matrix3d::Identity();
        mbg = Converter::toVector3d(mpCurrentKeyFrame->GetGyroBias());
        mba = Converter::toVector3d(mpCurrentKeyFrame->GetAccBias());
    }

    mScale=1.0;

    mInitTime = mpTracker->mLastFrame.mTimeStamp-vpKF.front()->mTimeStamp;

    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    Optimizer::InertialOptimization(mpAtlas->GetCurrentMap(), mRwg, mScale, mbg, mba, mbMonocular, infoInertial, false, false, priorG, priorA);
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    /*cout << "scale after inertial-only optimization: " << mScale << endl;
    cout << "bg after inertial-only optimization: " << mbg << endl;
    cout << "ba after inertial-only optimization: " << mba << endl;*/


    if (mScale<1e-1)
    {
        cout << "scale too small" << endl;
        bInitializing=false;
        return;
    }



    // Before this line we are not changing the map

    unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    if ((fabs(mScale-1.f)>0.00001)||!mbMonocular)
    {
        mpAtlas->GetCurrentMap()->ApplyScaledRotation(Converter::toCvMat(mRwg).t(),mScale,true);
        mpTracker->UpdateFrameIMU(mScale,vpKF[0]->GetImuBias(),mpCurrentKeyFrame);
    }
    std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();

    // Check if initialization OK
    if (!mpAtlas->isImuInitialized())
        for(int i=0;i<N;i++)
        {
            KeyFrame* pKF2 = vpKF[i];
            pKF2->bImu = true;
        }

    /*cout << "Before GIBA: " << endl;
    cout << "ba: " << mpCurrentKeyFrame->GetAccBias() << endl;
    cout << "bg: " << mpCurrentKeyFrame->GetGyroBias() << endl;*/

    std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
    if (bFIBA)
    {
        if (priorA!=0.f)
            Optimizer::FullInertialBA(mpAtlas->GetCurrentMap(), 100, false, 0, NULL, true, priorG, priorA);
        else
            Optimizer::FullInertialBA(mpAtlas->GetCurrentMap(), 100, false, 0, NULL, false);
    }

    std::chrono::steady_clock::time_point t5 = std::chrono::steady_clock::now();

    // If initialization is OK
    mpTracker->UpdateFrameIMU(1.0,vpKF[0]->GetImuBias(),mpCurrentKeyFrame);
    if (!mpAtlas->isImuInitialized())
    {
        cout << "IMU in Map " << mpAtlas->GetCurrentMap()->GetId() << " is initialized" << endl;
        mpAtlas->SetImuInitialized();
        mpTracker->t0IMU = mpTracker->mCurrentFrame.mTimeStamp;
        mpCurrentKeyFrame->bImu = true;
    }


    mbNewInit=true;
    mnKFs=vpKF.size();
    mIdxInit++;

    for(list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
    {
        (*lit)->SetBadFlag();
        delete *lit;
    }
    mlNewKeyFrames.clear();

    mpTracker->mState=Tracking::OK;
    bInitializing = false;


    /*cout << "After GIBA: " << endl;
    cout << "ba: " << mpCurrentKeyFrame->GetAccBias() << endl;
    cout << "bg: " << mpCurrentKeyFrame->GetGyroBias() << endl;
    double t_inertial_only = std::chrono::duration_cast<std::chrono::duration<double> >(t1 - t0).count();
    double t_update = std::chrono::duration_cast<std::chrono::duration<double> >(t3 - t2).count();
    double t_viba = std::chrono::duration_cast<std::chrono::duration<double> >(t5 - t4).count();
    cout << t_inertial_only << ", " << t_update << ", " << t_viba << endl;*/

    mpCurrentKeyFrame->GetMap()->IncreaseChangeIndex();

    return;
}

void LocalMapping::ScaleRefinement()
{
    // Minimum number of keyframes to compute a solution
    // Minimum time (seconds) between first and last keyframe to compute a solution. Make the difference between monocular and stereo
    // unique_lock<mutex> lock0(mMutexImuInit);
    if (mbResetRequested)
        return;

    // Retrieve all keyframes in temporal order
    list<KeyFrame*> lpKF;
    KeyFrame* pKF = mpCurrentKeyFrame;
    while(pKF->mPrevKF)
    {
        lpKF.push_front(pKF);
        pKF = pKF->mPrevKF;
    }
    lpKF.push_front(pKF);
    vector<KeyFrame*> vpKF(lpKF.begin(),lpKF.end());

    while(CheckNewKeyFrames())
    {
        ProcessNewKeyFrame();
        vpKF.push_back(mpCurrentKeyFrame);
        lpKF.push_back(mpCurrentKeyFrame);
    }

    const int N = vpKF.size();

    mRwg = Eigen::Matrix3d::Identity();
    mScale=1.0;

    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    Optimizer::InertialOptimization(mpAtlas->GetCurrentMap(), mRwg, mScale);
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    if (mScale<1e-1) // 1e-1
    {
        cout << "scale too small" << endl;
        bInitializing=false;
        return;
    }

    // Before this line we are not changing the map
    unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    if ((fabs(mScale-1.f)>0.00001)||!mbMonocular)
    {
        mpAtlas->GetCurrentMap()->ApplyScaledRotation(Converter::toCvMat(mRwg).t(),mScale,true);
        mpTracker->UpdateFrameIMU(mScale,mpCurrentKeyFrame->GetImuBias(),mpCurrentKeyFrame);
    }
    std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();

    for(list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
    {
        (*lit)->SetBadFlag();
        delete *lit;
    }
    mlNewKeyFrames.clear();

    double t_inertial_only = std::chrono::duration_cast<std::chrono::duration<double> >(t1 - t0).count();

    // To perform pose-inertial opt w.r.t. last keyframe
    mpCurrentKeyFrame->GetMap()->IncreaseChangeIndex();

    return;
}



bool LocalMapping::IsInitializing()
{
    return bInitializing;
}


double LocalMapping::GetCurrKFTime()
{

    if (mpCurrentKeyFrame)
    {
        return mpCurrentKeyFrame->mTimeStamp;
    }
    else
        return 0.0;
}

KeyFrame* LocalMapping::GetCurrKF()
{
    return mpCurrentKeyFrame;
}

} //namespace ORB_SLAM
