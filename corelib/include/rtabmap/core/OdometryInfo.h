/*
Copyright (c) 2010-2025, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef ODOMETRYINFO_H_
#define ODOMETRYINFO_H_

#include <rtabmap/core/rtabmap_core_export.h>
#include <map>
#include "rtabmap/core/Transform.h"
#include "rtabmap/core/RegistrationInfo.h"
#include "rtabmap/core/CameraModel.h"
#include "rtabmap/core/LaserScan.h"
#include <opencv2/features2d/features2d.hpp>

namespace rtabmap {

class RTABMAP_CORE_EXPORT OdometryInfo
{
public:
	OdometryInfo();
	OdometryInfo copyWithoutData() const;
	std::map<std::string, float> statistics(const Transform & pose = Transform());

	bool lost;  // true：里程计跟踪失败
	RegistrationInfo reg;  // 点云 / 特征匹配信息，用于判断当前估计是否可信
	// 特征 & 局部地图统计
	int features;  // 当前帧提取的特征数
	int localMapSize;  // 局部视觉地图点数
	int localScanMapSize;  // 局部激光地图大小
	int localKeyFrames;  // 当前参与优化的关键帧数
	// 局部 BA（Bundle Adjustment）信息
	int localBundleOutliers;  // 被剔除的外点
	int localBundleConstraints;  // 当前局部 BA 的约束数量
	float localBundleTime;
	std::map<int, Transform> localBundlePoses;
	std::map<int, std::vector<CameraModel> > localBundleModels;
	float localBundleAvgInlierDistance;
	int localBundleMaxKeyFramesForInlier;
	std::vector<int> localBundleOutliersPerCam;
	bool keyFrameAdded;
	// 时间消耗统计（性能分析）
	float timeDeskewing;
	float timeEstimation;
	float timeParticleFiltering;
	double stamp;
	double interval;  // 与上一帧时间差
	// 位姿相关（核心输出）
	Transform transform;  // 原始 odom 估计
	Transform transformFiltered;  // 滤波后的
	Transform transformGroundTruth;
	// 预测位姿（先验）
	Transform guessVelocity; // deprecated, will be removed. Use guess and interval instead.
	Transform guess;
	float distanceTravelled;
	int memoryUsage; //MB
	double gravityRollError;
	double gravityPitchError;

	int type;

	// F2M
	std::multimap<int, cv::KeyPoint> words;
	std::map<int, cv::Point3f> localMap;
	LaserScan localScanMap;

	// F2F
	std::vector<cv::Point2f> refCorners;
	std::vector<cv::Point2f> newCorners;
	std::vector<int> cornerInliers;
};

}

#endif /* ODOMETRYINFO_H_ */
