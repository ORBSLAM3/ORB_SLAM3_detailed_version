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

//注释@qxiaofan
//email:vision3d@yeah.net

//对于ORBextractor模块的流程简单梳理如下：
//step-1:构造图像金字塔。
//step-2:提取FAST角点，并对其进行均匀化分布，同时计算特征点方向。
//step-3：高斯滤波。
//step-4:计算描述子。

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv/cv.h>


namespace ORB_SLAM3
{

class ExtractorNode
{
public:
    /** @brief 构造函数 */
    ExtractorNode():bNoMore(false){}

    /**
    * @brief 在八叉树分配特征点的过程中，实现一个节点分裂为4个节点的操作
    *
    * @param[out] n1   分裂的节点1
    * @param[out] n2   分裂的节点2
    * @param[out] n3   分裂的节点3
    * @param[out] n4   分裂的节点4
    */
    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    std::vector<cv::KeyPoint> vKeys;
    cv::Point2i UL, UR, BL, BR;
    std::list<ExtractorNode>::iterator lit;
    bool bNoMore;
};

class ORBextractor
{
public:
    
    enum {HARRIS_SCORE=0, FAST_SCORE=1 };

   /**
   * @brief 构造函数
   *
   * @param[in] nfeatures   特征点数目
   * @param[in] scaleFactor 图像金字塔缩放系数
   * @param[in] nlevels     金字塔层数
   * @param[in] iniThFAST   最初FAST特征点提取参数
   * @param[in] minThFAST   使用iniThFAST阈值，提取的特征点数目达不到要求时，启用阈值minThFAST
   */
    ORBextractor(int nfeatures, float scaleFactor, int nlevels,
                 int iniThFAST, int minThFAST);

    ~ORBextractor(){}

    // Compute the ORB features and descriptors on an image.
    // ORB are dispersed on the image using an octree.
    // Mask is ignored in the current implementation.
    int operator()( cv::InputArray _image, cv::InputArray _mask,
                    std::vector<cv::KeyPoint>& _keypoints,
                    cv::OutputArray _descriptors, std::vector<int> &vLappingArea);

    int inline GetLevels(){
        return nlevels;}

    float inline GetScaleFactor(){
        return scaleFactor;}

    std::vector<float> inline GetScaleFactors(){
        return mvScaleFactor;
    }

    std::vector<float> inline GetInverseScaleFactors(){
        return mvInvScaleFactor;
    }

    std::vector<float> inline GetScaleSigmaSquares(){
        return mvLevelSigma2;
    }

    std::vector<float> inline GetInverseScaleSigmaSquares(){
        return mvInvLevelSigma2;
    }

    std::vector<cv::Mat> mvImagePyramid;

protected:
    /**
    * @brief 计算图像金字塔
    * @param[in] image 输入图片
    */
    void ComputePyramid(cv::Mat image);

   /**
   * @brief 按八叉树分配特征点方式，计算图像金字塔中的特征点
   * @details 第一层vector：存储image的所有特征点；第二层vector:每层金字塔对应的keypoints
   * @param[out] allKeypoints 提取得到的所有特征点
   */
    void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);

   /**
   * @brief 对于某一图层，分配其特征点，通过八叉树的方式
   * @param[in] vToDistributeKeys         等待分配的特征点
   * @param[in] minX                      分发的图像范围
   * @param[in] maxX                      分发的图像范围
   * @param[in] minY                      分发的图像范围
   * @param[in] maxY                      分发的图像范围
   * @param[in] nFeatures                 设定的、本图层中想要提取的特征点数目
   * @param[in] level                     要提取的图像所在的金字塔层
   * @return std::vector<cv::KeyPoint>
   */
    std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                           const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);
   /**
   * @brief 这个函数实际中没有使用，不作介绍
   * @param[out] allKeypoints 提取到的特征点
   */
    void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);
    std::vector<cv::Point> pattern;      //计算描述子的随机采样点

    int nfeatures;                       //整个图像(囊括每层金字塔）应提取的特征点数目
    double scaleFactor;                  //图像金字塔缩放因子
    int nlevels;                         //图像金字塔层数
    int iniThFAST;                       //初始FAST响应阈值
    int minThFAST;                       //最小FAST响应阈值

    std::vector<int> mnFeaturesPerLevel; //每层图像中该提取的特征点数目

    std::vector<int> umax;

    std::vector<float> mvScaleFactor;    //每层缩放比例
    std::vector<float> mvInvScaleFactor; //每层缩放比例的倒数
    std::vector<float> mvLevelSigma2;    //每层的sigma^2
    std::vector<float> mvInvLevelSigma2; //1/(sigma^2)
};

} //namespace ORB_SLAM

#endif

