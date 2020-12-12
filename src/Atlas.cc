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

#include "Atlas.h"
#include "Viewer.h"

#include "GeometricCamera.h"
#include "Pinhole.h"
#include "KannalaBrandt8.h"

namespace ORB_SLAM3
{

Atlas::Atlas(){
    mpCurrentMap = static_cast<Map*>(NULL);
}

Atlas::Atlas(int initKFid): mnLastInitKFidMap(initKFid), mHasViewer(false)
{
    mpCurrentMap = static_cast<Map*>(NULL);
    CreateNewMap();
}

Atlas::~Atlas()
{
    for(std::set<Map*>::iterator it = mspMaps.begin(), end = mspMaps.end(); it != end;)
    {
        Map* pMi = *it;

        if(pMi)
        {
            delete pMi;
            pMi = static_cast<Map*>(NULL);  // 删除指针后赋值NULL，避免指向内存随机位置

            it = mspMaps.erase(it);
        }
        else
            ++it;

    }
}
/**  创建新地图集
 * @调用：在Tracking开始时，或Tracking丢失时重新创建一个新的子地图。
 * 如果是刚开始，此时mpCurrentMap不存在，所以不执行if的代码段，而直接进行创建
 * 如果mpCurrentMap存在，则意味着需要开始一个新的子地图，此时需要记录当前地图id，赋予原子地图最大id并将下一个id设置为新的子地图起始id
 * 在创建新子地图时，将这一段子地图设置为active(mIsInUse=true)，并将之前的子地图设置为non-active
 * */
void Atlas::CreateNewMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    cout << "Creation of new map with id: " << Map::nNextId << endl;
    if(mpCurrentMap){
        cout << "Exits current map " << endl;
        if(!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
            mnLastInitKFidMap = mpCurrentMap->GetMaxKFid()+1; //The init KF is the next of current maximum

        mpCurrentMap->SetStoredMap();
        cout << "Saved map with ID: " << mpCurrentMap->GetId() << endl;

        //if(mHasViewer)
        //    mpViewer->AddMapToCreateThumbnail(mpCurrentMap);
    }
    cout << "Creation of new map with last KF id: " << mnLastInitKFidMap << endl;

    mpCurrentMap = new Map(mnLastInitKFidMap);
    mpCurrentMap->SetCurrentMap();
    mspMaps.insert(mpCurrentMap);
}


/** 修改Map
 * @调用：在LoopClosing的MergeLocal，融合两个子地图后调用
 * @param pMap：融合后的地图
 * @作用：即在active map和matched map融合后，将当前map设置为融合后的map
 * */
void Atlas::ChangeMap(Map* pMap)
{
    unique_lock<mutex> lock(mMutexAtlas);
    cout << "Chage to map with id: " << pMap->GetId() << endl;
    if(mpCurrentMap){
        mpCurrentMap->SetStoredMap();
    }

    mpCurrentMap = pMap;
    mpCurrentMap->SetCurrentMap();
}

unsigned long int Atlas::GetLastInitKFid()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mnLastInitKFidMap;
}

void Atlas::SetViewer(Viewer* pViewer)
{
    mpViewer = pViewer;
    mHasViewer = true;
}

void Atlas::AddKeyFrame(KeyFrame* pKF)
{
    Map* pMapKF = pKF->GetMap();
    pMapKF->AddKeyFrame(pKF);
}

void Atlas::AddMapPoint(MapPoint* pMP)
{
    Map* pMapMP = pMP->GetMap();
    pMapMP->AddMapPoint(pMP);
}

void Atlas::AddCamera(GeometricCamera* pCam)
{
    mvpCameras.push_back(pCam);
}

void Atlas::SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs)
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->SetReferenceMapPoints(vpMPs);
}

void Atlas::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->InformNewBigChange();
}

/** 获取地图集大变化的次数 
 * @return：变化的次数
 * @作用：每当地图集的当前子地图发生了回环、全局BA后，地图的变化次数会+1
 * 在System运行时会判断子地图是否发生了重要变换，如果发生了，在显示时会进行更新。
 **/
int Atlas::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetLastBigChangeIdx();
}

/** 计算当前地图中，地图点的数量，用于输出调试信息 **/
long unsigned int Atlas::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->MapPointsInMap();
}
/** 计算当前地图中，关键帧的数量，用于输出调试信息，以及一些其他判断 **/
long unsigned Atlas::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->KeyFramesInMap();
}

std::vector<KeyFrame*> Atlas::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllKeyFrames();
}

std::vector<MapPoint*> Atlas::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllMapPoints();
}

std::vector<MapPoint*> Atlas::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetReferenceMapPoints();
}

/** 按照子地图id从小到达顺序，获取全部子地图 **/
vector<Map*> Atlas::GetAllMaps()
{
    unique_lock<mutex> lock(mMutexAtlas);
    struct compFunctor
    {
        inline bool operator()(Map* elem1 ,Map* elem2)
        {
            return elem1->GetId() < elem2->GetId();
        }
    };
    vector<Map*> vMaps(mspMaps.begin(),mspMaps.end());
    sort(vMaps.begin(), vMaps.end(), compFunctor());    // 使用自定义的方法进行sort
    return vMaps;
}

int Atlas::CountMaps()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mspMaps.size();
}

void Atlas::clearMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->clear();
}

void Atlas::clearAtlas()
{
    unique_lock<mutex> lock(mMutexAtlas);
    /*for(std::set<Map*>::iterator it=mspMaps.begin(), send=mspMaps.end(); it!=send; it++)
    {
        (*it)->clear();
        delete *it;
    }*/
    mspMaps.clear();
    mpCurrentMap = static_cast<Map*>(NULL);
    mnLastInitKFidMap = 0;
}

Map* Atlas::GetCurrentMap()     // TODO: 调用次数较多，还没搞清楚具体用途
{
    unique_lock<mutex> lock(mMutexAtlas);
    if(!mpCurrentMap)
        CreateNewMap();
    while(mpCurrentMap->IsBad())
        usleep(3000);

    return mpCurrentMap;
}

/** 设置为bad地图
 * @调用：LoopClosing中两个子地图融合之后
 * @param：pMap：将这个地图设置为bad
 * @作用：两个子地图融合之后，首先将当前地图设置为融合后的地图，之后将融合前的active map设置为bad
 * 并存储bad的子地图，在merge最后删掉全部bad的子地图
 **/
void Atlas::SetMapBad(Map* pMap)
{
    mspMaps.erase(pMap);
    pMap->SetBad();

    mspBadMaps.insert(pMap);
}

/**  删除所有bad的子地图
 * @调用：LoopClosing中完成一次LocalMerge的最后
**/
void Atlas::RemoveBadMaps()
{
    /*for(Map* pMap : mspBadMaps)
    {
        delete pMap;
        pMap = static_cast<Map*>(NULL);
    }*/
    mspBadMaps.clear();
}

bool Atlas::isInertial()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->IsInertial();
}

void Atlas::SetInertialSensor()
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->SetInertialSensor();
}

void Atlas::SetImuInitialized()
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->SetImuInitialized();
}

bool Atlas::isImuInitialized()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->isImuInitialized();
}

/** 这个函数并没有用到 **/
void Atlas::PreSave()
{
    if(mpCurrentMap){
        if(!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
            mnLastInitKFidMap = mpCurrentMap->GetMaxKFid()+1; //The init KF is the next of current maximum
    }

    struct compFunctor
    {
        inline bool operator()(Map* elem1 ,Map* elem2)
        {
            return elem1->GetId() < elem2->GetId();
        }
    };

    // 在mvpBackupMaps后面加入mspMpas的全部数据
    // 关于back_inserter的详细用法参考：http://www.cplusplus.com/reference/iterator/back_inserter/
    std::copy(mspMaps.begin(), mspMaps.end(), std::back_inserter(mvpBackupMaps));
    sort(mvpBackupMaps.begin(), mvpBackupMaps.end(), compFunctor());

    std::set<GeometricCamera*> spCams(mvpCameras.begin(), mvpCameras.end());
    cout << "There are " << spCams.size() << " cameras in the atlas" << endl;
    for(Map* pMi : mvpBackupMaps)
    {
        cout << "Pre-save of map " << pMi->GetId() << endl;
        pMi->PreSave(spCams);
    }
    cout << "Maps stored" << endl;
    for(GeometricCamera* pCam : mvpCameras)
    {
        cout << "Pre-save of camera " << pCam->GetId() << endl;
        if(pCam->GetType() == pCam->CAM_PINHOLE)
        {
            mvpBackupCamPin.push_back((Pinhole*) pCam);
        }
        else if(pCam->GetType() == pCam->CAM_FISHEYE)
        {
            mvpBackupCamKan.push_back((KannalaBrandt8*) pCam);
        }
    }

}

/** 这个函数并没有用到 **/
void Atlas::PostLoad()
{
    mvpCameras.clear();
    map<unsigned int,GeometricCamera*> mpCams;
    for(Pinhole* pCam : mvpBackupCamPin)
    {
        //mvpCameras.push_back((GeometricCamera*)pCam);
        mvpCameras.push_back(pCam);
        mpCams[pCam->GetId()] = pCam;
    }
    for(KannalaBrandt8* pCam : mvpBackupCamKan)
    {
        //mvpCameras.push_back((GeometricCamera*)pCam);
        mvpCameras.push_back(pCam);
        mpCams[pCam->GetId()] = pCam;
    }

    mspMaps.clear();
    unsigned long int numKF = 0, numMP = 0;
    map<long unsigned int, KeyFrame*> mpAllKeyFrameId;
    for(Map* pMi : mvpBackupMaps)
    {
        cout << "Map id:" << pMi->GetId() << endl;
        mspMaps.insert(pMi);
        map<long unsigned int, KeyFrame*> mpKeyFrameId;
        pMi->PostLoad(mpKeyFrameDB, mpORBVocabulary, mpKeyFrameId, mpCams);
        mpAllKeyFrameId.insert(mpKeyFrameId.begin(), mpKeyFrameId.end());
        numKF += pMi->GetAllKeyFrames().size();
        numMP += pMi->GetAllMapPoints().size();
    }

    cout << "Number KF:" << numKF << "; number MP:" << numMP << endl;
    mvpBackupMaps.clear();
}



/** 从这里开始，往下的函数也都没有用到 **/

void Atlas::SetKeyFrameDababase(KeyFrameDatabase* pKFDB)
{
    mpKeyFrameDB = pKFDB;
}

KeyFrameDatabase* Atlas::GetKeyFrameDatabase()
{
    return mpKeyFrameDB;
}

void Atlas::SetORBVocabulary(ORBVocabulary* pORBVoc)
{
    mpORBVocabulary = pORBVoc;
}

ORBVocabulary* Atlas::GetORBVocabulary()
{
    return mpORBVocabulary;
}

long unsigned int Atlas::GetNumLivedKF()
{
    unique_lock<mutex> lock(mMutexAtlas);
    long unsigned int num = 0;
    for(Map* mMAPi : mspMaps)
    {
        num += mMAPi->GetAllKeyFrames().size();
    }

    return num;
}

long unsigned int Atlas::GetNumLivedMP() {
    unique_lock<mutex> lock(mMutexAtlas);
    long unsigned int num = 0;
    for (Map *mMAPi : mspMaps) {
        num += mMAPi->GetAllMapPoints().size();
    }

    return num;
}

} //namespace ORB_SLAM3
