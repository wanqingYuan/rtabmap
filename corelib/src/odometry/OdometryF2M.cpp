/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/core/Memory.h"
#include "rtabmap/core/Signature.h"
#include "rtabmap/core/RegistrationVis.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/core/util3d_registration.h"
#include "rtabmap/core/util3d_motion_estimation.h"
#include "rtabmap/core/util3d_filtering.h"
#include "rtabmap/core/util3d_surface.h"
#include "rtabmap/core/Optimizer.h"
#include "rtabmap/core/VWDictionary.h"
#include "rtabmap/core/Graph.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UTimer.h"
#include "rtabmap/utilite/UMath.h"
#include "rtabmap/utilite/UConversion.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <rtabmap/core/odometry/OdometryF2M.h>
#include <pcl/common/io.h>

#if _MSC_VER
	#define ISFINITE(value) _finite(value)
#else
	#define ISFINITE(value) std::isfinite(value)
#endif

namespace rtabmap {

OdometryF2M::OdometryF2M(const ParametersMap & parameters) :
	Odometry(parameters),
	maximumMapSize_(Parameters::defaultOdomF2MMaxSize()),
	keyFrameThr_(Parameters::defaultOdomKeyFrameThr()),
	visKeyFrameThr_(Parameters::defaultOdomVisKeyFrameThr()),
	maxNewFeatures_(Parameters::defaultOdomF2MMaxNewFeatures()),
	initDepthFactor_(Parameters::defaultOdomF2MInitDepthFactor()),
	floorThreshold_(Parameters::defaultOdomF2MFloorThreshold()),
	scanKeyFrameThr_(Parameters::defaultOdomScanKeyFrameThr()),
	scanMaximumMapSize_(Parameters::defaultOdomF2MScanMaxSize()),
	scanSubtractRadius_(Parameters::defaultOdomF2MScanSubtractRadius()),
	scanSubtractAngle_(Parameters::defaultOdomF2MScanSubtractAngle()),
	scanMapMaxRange_(Parameters::defaultOdomF2MScanRange()),
	bundleAdjustment_(Parameters::defaultOdomF2MBundleAdjustment()),
	bundleMaxFrames_(Parameters::defaultOdomF2MBundleAdjustmentMaxFrames()),
	bundleMinMotion_(Parameters::defaultOdomF2MBundleAdjustmentMinMotion()),
	bundleMaxKeyFramesPerFeature_(Parameters::defaultOdomF2MBundleAdjustmentMaxKeyFramesPerFeature()),
	bundleUpdateFeatureMapOnAllFrames_(Parameters::defaultOdomF2MBundleUpdateFeatureMapOnAllFrames()),
	validDepthRatio_(Parameters::defaultOdomF2MValidDepthRatio()),
	pointToPlaneK_(Parameters::defaultIcpPointToPlaneK()),
	pointToPlaneRadius_(Parameters::defaultIcpPointToPlaneRadius()),
	map_(new Signature(-1)),
	lastFrame_(new Signature(1)),
	lastFrameOldestNewId_(0),
	bundleSeq_(0),
	sba_(0)
{
	UDEBUG("");
	Parameters::parse(parameters, Parameters::kOdomF2MMaxSize(), maximumMapSize_);
	Parameters::parse(parameters, Parameters::kOdomKeyFrameThr(), keyFrameThr_);
	Parameters::parse(parameters, Parameters::kOdomVisKeyFrameThr(), visKeyFrameThr_);
	Parameters::parse(parameters, Parameters::kOdomF2MMaxNewFeatures(), maxNewFeatures_);
	Parameters::parse(parameters, Parameters::kOdomF2MInitDepthFactor(), initDepthFactor_);
	Parameters::parse(parameters, Parameters::kOdomF2MFloorThreshold(), floorThreshold_);
	Parameters::parse(parameters, Parameters::kOdomScanKeyFrameThr(), scanKeyFrameThr_);
	Parameters::parse(parameters, Parameters::kOdomF2MScanMaxSize(), scanMaximumMapSize_);
	Parameters::parse(parameters, Parameters::kOdomF2MScanSubtractRadius(), scanSubtractRadius_);
	if(Parameters::parse(parameters, Parameters::kOdomF2MScanSubtractAngle(), scanSubtractAngle_))
	{
		scanSubtractAngle_ *= M_PI/180.0f;
	}
	Parameters::parse(parameters, Parameters::kOdomF2MScanRange(), scanMapMaxRange_);
	Parameters::parse(parameters, Parameters::kOdomF2MBundleAdjustment(), bundleAdjustment_);
	Parameters::parse(parameters, Parameters::kOdomF2MBundleAdjustmentMaxFrames(), bundleMaxFrames_);
	Parameters::parse(parameters, Parameters::kOdomF2MBundleAdjustmentMinMotion(), bundleMinMotion_);
	Parameters::parse(parameters, Parameters::kOdomF2MBundleAdjustmentMaxKeyFramesPerFeature(), bundleMaxKeyFramesPerFeature_);
	Parameters::parse(parameters, Parameters::kOdomF2MBundleUpdateFeatureMapOnAllFrames(), bundleUpdateFeatureMapOnAllFrames_);
	Parameters::parse(parameters, Parameters::kOdomF2MValidDepthRatio(), validDepthRatio_);

	Parameters::parse(parameters, Parameters::kIcpPointToPlaneK(), pointToPlaneK_);
	Parameters::parse(parameters, Parameters::kIcpPointToPlaneRadius(), pointToPlaneRadius_);

	UASSERT(bundleMaxFrames_ >= 0);
	ParametersMap bundleParameters = parameters;
	if(bundleAdjustment_ > 0)
	{
		if((bundleAdjustment_==1 && Optimizer::isAvailable(Optimizer::kTypeG2O)) ||
		(bundleAdjustment_==2 && Optimizer::isAvailable(Optimizer::kTypeCVSBA)) ||
		(bundleAdjustment_==3 && Optimizer::isAvailable(Optimizer::kTypeCeres)))
		{
			// disable bundle in RegistrationVis as we do it already here
			uInsert(bundleParameters, ParametersPair(Parameters::kVisBundleAdjustment(), "0"));
			sba_ = Optimizer::create(bundleAdjustment_==3?Optimizer::kTypeCeres:bundleAdjustment_==2?Optimizer::kTypeCVSBA:Optimizer::kTypeG2O, bundleParameters);
		}
		else
		{
			UWARN("Selected bundle adjustment approach (\"%s\"=\"%d\") is not available, "
					"local bundle adjustment is then disabled.", Parameters::kOdomF2MBundleAdjustment().c_str(), bundleAdjustment_);
			bundleAdjustment_ = 0;
		}
	}
	UASSERT(maximumMapSize_ >= 0);
	UASSERT(keyFrameThr_ >= 0.0f && keyFrameThr_<=1.0f);
	UASSERT(visKeyFrameThr_>=0);
	UASSERT(scanKeyFrameThr_ >= 0.0f && scanKeyFrameThr_<=1.0f);
	UASSERT(maxNewFeatures_ >= 0);
	UASSERT(initDepthFactor_>0.0f);

	int corType = Parameters::defaultVisCorType();
	Parameters::parse(parameters, Parameters::kVisCorType(), corType);
	if(corType != 0)
	{
		UWARN("%s=%d is not supported by OdometryF2M, using Features matching approach instead (type=0).",
				Parameters::kVisCorType().c_str(),
				corType);
		corType = 0;
	}
	uInsert(bundleParameters, ParametersPair(Parameters::kVisCorType(), uNumber2Str(corType)));

	int estType = Parameters::defaultVisEstimationType();
	Parameters::parse(parameters, Parameters::kVisEstimationType(), estType);
	if(estType > 1)
	{
		UWARN("%s=%d is not supported by OdometryF2M, using 2D->3D approach instead (type=1).",
				Parameters::kVisEstimationType().c_str(),
				estType);
		estType = 1;
	}
	uInsert(bundleParameters, ParametersPair(Parameters::kVisEstimationType(), uNumber2Str(estType)));

	regPipeline_ = Registration::create(bundleParameters);
	if(bundleAdjustment_>0 && regPipeline_->isScanRequired())
	{
		if(regPipeline_->isImageRequired())
		{
			UWARN("%s=%d cannot be used with registration not done only with images (%s=%s), disabling bundle adjustment.",
					Parameters::kOdomF2MBundleAdjustment().c_str(),
					bundleAdjustment_,
					Parameters::kRegStrategy().c_str(),
					uValue(bundleParameters, Parameters::kRegStrategy(), uNumber2Str(Parameters::defaultRegStrategy())).c_str());
		}
		bundleAdjustment_ = 0;
	}

	parameters_ = bundleParameters;
}

OdometryF2M::~OdometryF2M()
{
	delete map_;
	delete lastFrame_;
	delete sba_;
	delete regPipeline_;
	UDEBUG("");
}


void OdometryF2M::reset(const Transform & initialPose)
{
	Odometry::reset(initialPose);

	UDEBUG("initialPose=%s", initialPose.prettyPrint().c_str());
	Odometry::reset(initialPose);
	*lastFrame_ = Signature(1);
	*map_ = Signature(-1);
	scansBuffer_.clear();
	bundleWordReferences_.clear();
	bundlePoses_.clear();
	bundleLinks_.clear();
	bundleModels_.clear();
	bundlePoseReferences_.clear();
	bundleSeq_ = 0;
	lastFrameOldestNewId_ = 0;
}

// return not null transform if odometry is correctly computed
// 基于 Frame-to-Map（F2M）的方式，计算当前传感器帧相对于上一时刻的里程计增量，并维护一个局部地图（视觉 + 激光 + IMU，可选局部 BA）
// | F2F（Frame-to-Frame）   | 上一帧       | 快，但漂移快  |
// | F2M（Frame-to-Map） | 多帧组成的局部地图 | 稳定，适合真车 |

Transform OdometryF2M::computeTransform(
		SensorData & data,
		const Transform & guessIn,
		OdometryInfo * info)
{
	// 1 初始化与 IMU 处理
	// guessIn：外部给的运动先验（例如 IMU / 轮速）
	// output：最终输出的 相对位姿
	Transform guess = guessIn;
	UTimer timer;
	Transform output;

	if(info)
	{
		info->type = 0;
	}

	// IMU 重力约束
	Transform imuT;
	// imus() {return imus_;} 在process函数中已经将imu放入imus_变量中了
	if(sba_ && sba_->gravitySigma() > 0.0f && !imus().empty())
	{
		// 如果启用了 SBA 且 IMU 可用：插值 IMU 姿态；后续 BA 中作为 gravity 约束
		imuT = Transform::getTransform(imus(), data.stamp());
		// 若当前帧没有 IMU 数据 → 补进去；
		if(data.imu().empty())
		{
			Eigen::Quaternionf q = imuT.getQuaternionf();
			data.setIMU(IMU(cv::Vec4d(q.x(), q.y(), q.z(), q.w()), cv::Mat(), cv::Vec3d(), cv::Mat(), cv::Vec3d(), cv::Mat()));
		}
	}

	RegistrationInfo regInfo;
	int nFeatures = 0;

	// 2 构建当前帧（Signature）
	// 把 SensorData 包装成 Signature
	delete lastFrame_;
	int id = data.id();
	data.setId(++bundleSeq_); // 使用内部 bundleSeq_ 强制生成唯一 ID（防止重复）
	lastFrame_ = new Signature(data);
	data.setId(id);

	bool addKeyFrame = false;
	int totalBundleWordReferencesUsed = 0;
	int totalBundleOutliers = 0;
	float bundleTime = 0.0f;
	bool visDepthAsMask = Parameters::defaultVisDepthAsMask();
	Parameters::parse(parameters_, Parameters::kVisDepthAsMask(), visDepthAsMask);

	// 3 获取相机模型（单目 / RGB-D / 双目）后面 BA / 像素误差 / 多相机索引 都依赖它
	std::vector<CameraModel> lastFrameModels;
	if(!lastFrame_->sensorData().cameraModels().empty() &&
		lastFrame_->sensorData().cameraModels().at(0).isValidForProjection())
	{
		lastFrameModels = lastFrame_->sensorData().cameraModels();
	}
	else if(!lastFrame_->sensorData().stereoCameraModels().empty() &&
			lastFrame_->sensorData().stereoCameraModels().at(0).isValidForProjection())
	{
		for(size_t i=0; i<lastFrame_->sensorData().stereoCameraModels().size(); ++i)
		{
			CameraModel model = lastFrame_->sensorData().stereoCameraModels()[i].left();
			// Set Tx for stereo BA
			model = CameraModel(model.fx(),
					model.fy(),
					model.cx(),
					model.cy(),
					model.localTransform(),
					-lastFrame_->sensorData().stereoCameraModels()[i].baseline()*model.fx(),
					model.imageSize());
			lastFrameModels.push_back(model);
		}
	}
	UDEBUG("lastFrameModels=%ld", lastFrameModels.size());

	// Generate keypoints from the new data
	// 4 计算核心，用F2M方式确定位姿
	// 从新数据中生成关键点，要求传感器数据必须有效；否则丢弃此帧
	if(lastFrame_->sensorData().isValid())
	{
		// 条件判断：是否能做 Frame-to-Map
		// 必须满足两个条件之一：视觉地图存在（map_->getWords3().size() > 0）或激光地图存在（laserScanRaw 非空）
		// 否则：这是第一帧 / 地图未初始化；不能做 F2M，只能初始化
		if((map_->getWords3().size() || !map_->sensorData().laserScanRaw().isEmpty()) &&
			lastFrame_->sensorData().isValid())
		{
			// 4.1 准备临时数据结构
			// 拷贝当前局部地图,后续所有注册 / BA 都在 tmpMap 上进行,成功后再回写到 map_
			Signature tmpMap;  
			Transform transform;
			UDEBUG("guess=%s frames=%d image required=%d", guess.prettyPrint().c_str(), this->framesProcessed(), regPipeline_->isImageRequired()?1:0);

			// bundle adjustment stuff if used
			// Bundle Adjustment 临时容器
			std::map<int, cv::Point3f> points3DMap; // 参与 BA 的 3D 点
			std::map<int, Transform> bundlePoses; // 关键帧位姿
			std::multimap<int, Link> bundleLinks; // 位姿约束（邻接 / IMU）
			std::map<int, std::vector<CameraModel> > bundleModels; // 相机模型
			float bundleAvgInlierDistance = 0.0f;

			// 4.2 双重尝试机制（guess / no guess）
			// 有 guess + 视觉, 尝试2 次。因为 错误但不算太离谱的 guess，比完全没有 guess 更容易把视觉配准带进局部最优甚至失败。
			// 无 guess 或 ICP-only, 尝试1 次。ICP（激光）不需要 因为ICP 本身就是局部搜索，没 guess → 很容易直接不收敛
			// 所以 ICP 通常：要么给 guess；要么扩大 correspondence distance
			for(int guessIteration=0;
					guessIteration<(!guess.isNull()&&regPipeline_->isImageRequired()?2:1) && transform.isNull();
					++guessIteration)
			{
				tmpMap = *map_;
				// reset matches, but keep already extracted features in lastFrame_->sensorData()
				// 清除 上一次注册留下的匹配关系, 但 不删除已提取的 keypoints / descriptors
				// 让 registration 重新匹配 map ↔ frame
				lastFrame_->removeAllWords();

				points3DMap.clear();
				bundlePoses.clear();
				bundleLinks.clear();
				bundleModels.clear();

				float maxCorrespondenceDistance = 0.0f;
				float outlierRatio = 0.0f;
				//  4.3 guess的纯激光里程计场景，并且处于第一帧->第二帧之间。此阶段需要初始化
				if(guess.isNull() &&
					!regPipeline_->isImageRequired() &&
					regPipeline_->isScanRequired() &&
					this->framesProcessed() < 2)
				{
					// only on initialization (first frame to register), increase icp max correspondences in case the robot is already moving
					// 机器人可能已经在运动
					maxCorrespondenceDistance = Parameters::defaultIcpMaxCorrespondenceDistance();
					outlierRatio = Parameters::defaultIcpOutlierRatio();
					Parameters::parse(parameters_, Parameters::kIcpMaxCorrespondenceDistance(), maxCorrespondenceDistance);
					Parameters::parse(parameters_, Parameters::kIcpOutlierRatio(), outlierRatio);
					ParametersMap params;
					// 扩大 ICP 搜索范围，防止初始失败
					// 只在初始化阶段生效，后面会恢复参数
					params.insert(ParametersPair(Parameters::kIcpMaxCorrespondenceDistance(), uNumber2Str(maxCorrespondenceDistance*3.0f)));
					params.insert(ParametersPair(Parameters::kIcpOutlierRatio(), uNumber2Str(0.95f)));
					regPipeline_->parseParameters(params);
				}

				if(guessIteration == 1)
				{
					UWARN("Failed to find a transformation with the provided guess (%s), trying again without a guess.", guess.prettyPrint().c_str());
				}

				// 4.4 Frame-to-Map 配准（核心
				// 地图：tmpMap（当前局部 map 的拷贝）
				// 当前帧：lastFrame_
				// 初始猜测： guess
				// 输出的是 当前帧在 map 坐标系下的位姿
				transform = regPipeline_->computeTransformationMod(
						tmpMap,
						*lastFrame_,
						// special case for ICP-only odom, set guess to identity if we just started or reset
						// 第一次尝试，并且有guess：使用guess作为初始猜测
						// 如果是纯激光场景，并且是前两帧：使用当前位姿作为初始猜测
						// 否则：使用空的初始位姿，让算法自己寻找匹配
						guessIteration==0 && !guess.isNull()?this->getPose()*guess:!regPipeline_->isImageRequired()&&this->framesProcessed()<2?this->getPose():Transform(),
						&regInfo);

				// 恢复 ICP 参数（很容易被忽略）
				// 确保：只有初始化阶段使用“宽松 ICP”；不影响后续正常帧
				if(maxCorrespondenceDistance>0.0f)
				{
					// set it back
					ParametersMap params;
					params.insert(ParametersPair(Parameters::kIcpMaxCorrespondenceDistance(), uNumber2Str(maxCorrespondenceDistance)));
					params.insert(ParametersPair(Parameters::kIcpOutlierRatio(), uNumber2Str(outlierRatio)));
					regPipeline_->parseParameters(params);
				}

				// 把注册过程中提取的特征写回 SensorData
				data.setFeatures(lastFrame_->sensorData().keypoints(), lastFrame_->sensorData().keypoints3D(), lastFrame_->sensorData().descriptors());
				data.setLaserScan(lastFrame_->sensorData().laserScanRaw());

				UDEBUG("Registration time = %fs", regInfo.totalTime);
				if(!transform.isNull())
				{
					// local bundle adjustment
					// 4.5 局部 Bundle Adjustment（可选）
					if(bundleAdjustment_>0 && sba_ &&
					   regPipeline_->isImageRequired() &&
					   !lastFrameModels.empty() &&
					   regInfo.inliersIDs.size())
					{
						UDEBUG("Local Bundle Adjustment");

						// 安全检查不通过：
						// 检查当前地图 map_ 的特征点数量和临时地图 tmpMap 的数量是否一致。如果数量不一致，说明特征可能被重新计算或丢失了。
						// 检查特征点集合中第一个 key（id）是否一致。用于判断特征点的 ID 顺序是否保持一致。

						// 如果 数量或首尾 id 不一致，说明：使用的注册方法可能 重新计算了特征点（如 Optical Flow）；原来的特征 ID 会被改变
						// 这样就无法安全地执行 Bundle Adjustment，因为 BA 依赖于 全局特征 ID 不变 来优化相机/点云位姿
						// 因此，代码里直接报错并禁用 BA：
						UASSERT(map_->getWords().size() && tmpMap.getWords().size());
						if(map_->getWords().size() != tmpMap.getWords().size() ||
						   map_->getWords().begin()->first != tmpMap.getWords().begin()->first ||
						   map_->getWords().rbegin()->first != tmpMap.getWords().rbegin()->first)
						{
							UERROR("Bundle Adjustment cannot be used with a registration approach recomputing "
									"features from the \"from\" signature (e.g., Optical Flow) that would change "
									"their ids (size=old=%ld new=%ld first/last: old=%d->%d new=%d->%d).",
									map_->getWords().size(), tmpMap.getWords().size(),
									map_->getWords().begin()->first, map_->getWords().rbegin()->first,
									tmpMap.getWords().begin()->first, tmpMap.getWords().rbegin()->first);
							bundleAdjustment_ = 0;
						}
						// 安全检查通过：
						else
						{
							// 断言检查，确保 BA 输入数据完整。
							UASSERT(bundlePoses_.size());
							UASSERT_MSG(bundlePoses_.size()-1 == bundleLinks_.size(), uFormat("poses=%d links=%d", (int)bundlePoses_.size(), (int)bundleLinks_.size()).c_str());
							UASSERT(bundlePoses_.size() == bundleModels_.size());

							// 准备 BA 数据
							// bundlePoses：关键帧位姿；bundleLinks：位姿约束（视觉 / IMU / 邻接）；points3DMap：3D 地图点；wordReferences：每个特征在哪些帧被看到
							// 将已有的 BA 数据拷贝到本地变量，用于优化。IMU 约束也加入 bundleLinks。
							bundlePoses = bundlePoses_;
							bundleLinks = bundleLinks_;
							bundleModels = bundleModels_;
							bundleLinks.insert(bundleIMUOrientations_.begin(), bundleIMUOrientations_.end());

							UASSERT_MSG(bundlePoses.find(lastFrame_->id()) == bundlePoses.end(),
									uFormat("Frame %d already added! Make sure the input frames have unique IDs!", lastFrame_->id()).c_str());
							// 加入相邻帧约束（邻接）
							bundleLinks.insert(std::make_pair(bundlePoses_.rbegin()->first, Link(bundlePoses_.rbegin()->first, lastFrame_->id(), Link::kNeighbor, bundlePoses_.rbegin()->second.inverse()*transform, regInfo.covariance.inv())));
							// 添加当前帧
							bundlePoses.insert(std::make_pair(lastFrame_->id(), transform));

							// IMU 重力约束。
							if(!imuT.isNull())
							{
								bundleLinks.insert(std::make_pair(lastFrame_->id(), Link(lastFrame_->id(), lastFrame_->id(), Link::kGravity, imuT)));
							}
							// 加入相机模型。
							bundleModels.insert(std::make_pair(lastFrame_->id(), lastFrameModels));

							UDEBUG("Fill matches (%d)", (int)regInfo.inliersIDs.size());
							// 收集特征观测（wordReferences）
							std::map<int, std::map<int, FeatureBA> > wordReferences; // 记录每个特征点在哪些关键帧被观测到
							size_t maxKeyFramesForInlier = 0;
							// 为“局部 Bundle Adjustment”构建观测数据, 把 每一个视觉 inlier 变成一个：「3D 点 + 多帧 2D 观测」的 BA 约束单元。
							// 遍历 当前帧的所有视觉 inliers 。 regInfo.inliersIDs = 当前帧与 map 成功匹配、用于位姿估计的视觉特征 ID
							for(unsigned int i=0; i<regInfo.inliersIDs.size(); ++i)
							{
								int wordId =regInfo.inliersIDs[i];

								// 3D point
								// A 找到该特征对应的 3D 点（map 中）
								std::multimap<int, int>::const_iterator iter3D = tmpMap.getWords().find(wordId);
								UASSERT(iter3D!=tmpMap.getWords().end() && !tmpMap.getWords3().empty());
								// tmpMap.getWords3()  index → cv::Point3f
								points3DMap.insert(std::make_pair(wordId, tmpMap.getWords3()[iter3D->second]));

								// all other references
								// B 找历史帧中对该特征的观测（references）
								// 长期维护的 BA 记忆：bundleWordReferences_[wordId][frameId] = FeatureBA
								std::map<int, std::map<int, FeatureBA> >::iterator refIter = bundleWordReferences_.find(wordId);
								// 如果找不到，说明BA 的历史数据已经不一致，直接报错。
								UASSERT_MSG(refIter != bundleWordReferences_.end(), uFormat("wordId=%d", wordId).c_str());
								if(info && refIter->second.size() > maxKeyFramesForInlier)
								{
									maxKeyFramesForInlier = refIter->second.size();
								}

								std::map<int, FeatureBA> references;
								// C 限制历史观测数量（bundleMaxFrames_）
								// bundleMaxFrames_ 会控制使用的历史帧数，避免 BA 太大
								// 为什么要 step？
								// 假设：一个特征被 50 个关键帧看到，你只想用最近 10 个
								// step = 50 / 10 = 5 → 每 5 个取一个
								int step = bundleMaxFrames_>0?(refIter->second.size() / bundleMaxFrames_):1;
								if(step == 0)
								{
									step = 1;
								}
								int oi=0;
								for(std::map<int, FeatureBA>::iterator jter=refIter->second.begin(); jter!=refIter->second.end(); ++jter)
								{
									// 稀疏采样历史观测
									if(oi++ % step == 0 && bundlePoses.find(jter->first)!=bundlePoses.end())
									{
										references.insert(*jter);
										++totalBundleWordReferencesUsed;
									}
								}
								//make sure the last reference is here
								// D 确保最后一次观测一定被加入
								if(refIter->second.size() > 1)
								{
									if(references.insert(*refIter->second.rbegin()).second)
									{
										++totalBundleWordReferencesUsed;
									}
								}

								// E 从当前帧中（lastFrame_中）找到该特征对应的 2D 观测
								std::multimap<int, int>::const_iterator iter2D = lastFrame_->getWords().find(wordId);
								if(iter2D!=lastFrame_->getWords().end())
								{
									UASSERT(!lastFrame_->getWordsKpts().empty());
									// 取关键点
									cv::KeyPoint kpt = lastFrame_->getWordsKpts()[iter2D->second];

									// 多相机情况：RTAB-Map 把多相机图像横向拼接
									int cameraIndex = 0;
									if(lastFrameModels.size()>1)
									{
										UASSERT(lastFrameModels[0].imageWidth()>0);
										float subImageWidth = lastFrameModels[0].imageWidth();
										// 计算 cameraIndex，算出来自哪个相机
										cameraIndex = int(kpt.pt.x / subImageWidth);
										UASSERT(cameraIndex < (int)lastFrameModels.size());
										// 把 keypoint 坐标还原到该子图
										kpt.pt.x = kpt.pt.x - (subImageWidth*float(cameraIndex));
									}

									//get depth
									// 计算深度（如果有）
									float d = 0.0f;
									if( !lastFrame_->getWords3().empty() &&
										util3d::isFinite(lastFrame_->getWords3()[iter2D->second]))
									{
										//move back point in camera frame (to get depth along z)
										d = util3d::transformPoint(lastFrame_->getWords3()[iter2D->second], lastFrameModels[cameraIndex].localTransform().inverse()).z;
									}
									// 插入当前帧观测
									references.insert(std::make_pair(lastFrame_->id(), FeatureBA(kpt, d, cv::Mat(), cameraIndex)));
								}
								// 把该特征的所有观测加入 BA
								wordReferences.insert(std::make_pair(wordId, references));

								//UDEBUG("%d (%f,%f,%f)", iter3D->first, iter3D->second.x, iter3D->second.y, iter3D->second.z);
								//for(std::map<int, cv::Point2f>::iterator iter=inserted.first->second.begin(); iter!=inserted.first->second.end(); ++iter)
								//{
								//	UDEBUG("%d (%f,%f)", iter->first, iter->second.x, iter->second.y);
								//}
							}

							UDEBUG("sba...start");
							// set root negative to fix all other poses
							std::set<int> sbaOutliers;
							UTimer bundleTimer;
							// F 调用BA优化
							// 结果：更新当前帧位姿；剔除 BA outliers；缩小协方差；估计像素平均运动（用于关键帧判定）
							// BA 后 transform 更准，但更慢
							bundlePoses = sba_->optimizeBA(-lastFrame_->id(), bundlePoses, bundleLinks, bundleModels, points3DMap, wordReferences, &sbaOutliers);
							bundleTime = bundleTimer.ticks();
							UDEBUG("sba...end");
							totalBundleOutliers = (int)sbaOutliers.size();

							UDEBUG("bundleTime=%fs (poses=%d wordRef=%d outliers=%d)", bundleTime, (int)bundlePoses.size(), (int)bundleWordReferences_.size(), (int)sbaOutliers.size());
							if(info)
							{
								info->localBundlePoses = bundlePoses;
								info->localBundleModels = bundleModels;
								info->localBundleMaxKeyFramesForInlier = maxKeyFramesForInlier;
							}

							UDEBUG("Local Bundle Adjustment Before: %s", transform.prettyPrint().c_str());
							// G 如果优化成功，需要更新 transform 和剔除 outliers
							if(bundlePoses.size() == bundlePoses_.size()+1)
							{
								// 优化后的结果有效说明优化成功
								if(!bundlePoses.rbegin()->second.isNull())
								{
									if(info)
									{
										info->localBundleOutliersPerCam = std::vector<int>(lastFrameModels.size(),0);
									}
									// 更新 transform 和剔除 outliers
									if(sbaOutliers.size())
									{
										regInfo.inliersPerCam = std::vector<int>(lastFrameModels.size(),0);
										std::vector<int> newInliers(regInfo.inliersIDs.size());
										int oi=0;
										for(unsigned int i=0; i<regInfo.inliersIDs.size(); ++i)
										{
											if(sbaOutliers.find(regInfo.inliersIDs[i]) == sbaOutliers.end())
											{
												newInliers[oi++] = regInfo.inliersIDs[i];
												regInfo.inliersPerCam[wordReferences.at(regInfo.inliersIDs[i]).at(lastFrame_->id()).cameraIndex] += 1;
											}
											else if(info)
											{
												info->localBundleOutliersPerCam[wordReferences.at(regInfo.inliersIDs[i]).at(lastFrame_->id()).cameraIndex] += 1;
											}
										}
										newInliers.resize(oi);
										UDEBUG("BA outliers ratio %f", float(sbaOutliers.size())/float(regInfo.inliersIDs.size()));
										// 更新 inliers
										regInfo.inliers = (int)newInliers.size();
										regInfo.inliersIDs = newInliers;
									}
									// 如果 BA 剔除了太多 inliers，注册失败，transform 设为 null。
									if(regInfo.inliers < regPipeline_->getMinVisualCorrespondences())
									{
										regInfo.rejectedMsg = uFormat("Too low inliers after bundle adjustment: %d<%d", regInfo.inliers, regPipeline_->getMinVisualCorrespondences());
										transform.setNull();
									}
									// 否则：当前帧位姿取 BA 优化后的结果;更新关键帧之间的相对变换;更新 IMU 重力误差（roll/pitch）
									else
									{
										transform = bundlePoses.rbegin()->second;
										std::multimap<int, Link>::iterator iter = graph::findLink(bundleLinks, bundlePoses_.rbegin()->first, lastFrame_->id(), false);
										UASSERT(iter != bundleLinks.end());
										// 更新相邻帧的相对变换
										iter->second.setTransform(bundlePoses_.rbegin()->second.inverse()*transform);

										iter = graph::findLink(bundleLinks, lastFrame_->id(), lastFrame_->id(), false);
										if(info && iter!=bundleLinks.end() && iter->second.type() == Link::kGravity)
										{
											// 更新 IMU 重力误差
											float rollImu,pitchImu,yaw;
											iter->second.transform().getEulerAngles(rollImu, pitchImu, yaw);
											float roll,pitch;
											transform.getEulerAngles(roll, pitch, yaw);
											info->gravityRollError = fabs(rollImu - roll);
											info->gravityPitchError = fabs(pitchImu - pitch);
										}

										// With bundle adjustment, scale down covariance by 10
										// BA 优化后，位姿更准确, 协方差矩阵缩小，表示对位姿估计更有信心
										UASSERT(regInfo.covariance.cols==6 && regInfo.covariance.rows == 6 && regInfo.covariance.type() == CV_64FC1);
										double thrLin = Registration::COVARIANCE_LINEAR_EPSILON*10.0;
										double thrAng = Registration::COVARIANCE_ANGULAR_EPSILON*10.0;
										if(regInfo.covariance.at<double>(0,0)>thrLin)
											regInfo.covariance.at<double>(0,0) *= 0.1;
										if(regInfo.covariance.at<double>(1,1)>thrLin)
											regInfo.covariance.at<double>(1,1) *= 0.1;
										if(regInfo.covariance.at<double>(2,2)>thrLin)
											regInfo.covariance.at<double>(2,2) *= 0.1;
										if(regInfo.covariance.at<double>(3,3)>thrAng)
											regInfo.covariance.at<double>(3,3) *= 0.1;
										if(regInfo.covariance.at<double>(4,4)>thrAng)
											regInfo.covariance.at<double>(4,4) *= 0.1;
										if(regInfo.covariance.at<double>(5,5)>thrAng)
											regInfo.covariance.at<double>(5,5) *= 0.1;

										// Estimate how much the new frame moved from previous frame in term of pixels
										// 计算当前帧与上一个关键帧的平均像素运动
										if(bundleMinMotion_ > 0.0f)
										{
											UASSERT(!bundlePoses_.empty());
											int count = 0;
											// 估计帧间运动
											for(unsigned int i=0; i<regInfo.inliersIDs.size(); ++i)
											{
												std::map<int, std::map<int, FeatureBA> >::iterator wter = wordReferences.find(regInfo.inliersIDs[i]);
												if(wter != wordReferences.end())
												{
													std::map<int, FeatureBA>::iterator fter = wter->second.find(bundlePoses_.rbegin()->first);
													if(fter != wter->second.end())
													{
														const FeatureBA & f1 = fter->second; // previous key-frame
														const FeatureBA & f2 = wter->second.find(lastFrame_->id())->second; // current key-frame
														float dx = f1.kpt.pt.x - f2.kpt.pt.x;
														float dy = f1.kpt.pt.y - f2.kpt.pt.y;
														bundleAvgInlierDistance += sqrt(dx*dx + dy*dy);
														++count;
													}
												}
											}
											if(count)
											{
												// 用于：判断是否生成新关键帧, 评估局部运动大小
												bundleAvgInlierDistance /= count;
											}
											UDEBUG("Average pixel distance between %d inliers: %f", count, bundleAvgInlierDistance);
											if(info)
											{
												info->localBundleAvgInlierDistance = bundleAvgInlierDistance;
											}
										}
									}
									UDEBUG("Local Bundle Adjustment After : %s", transform.prettyPrint().c_str());
								}
								// 优化后的结果为null 说明优化失败，设为null
								else
								{
									regInfo.rejectedMsg = "Last bundle pose is null?!";
									transform.setNull();
								}
							}
							// H 优化失败，设为null
							else
							{
								regInfo.rejectedMsg = "Local bundle adjustment failed!";
								transform.setNull();
							}
						}
					}

					if(!transform.isNull())
					{
						// make it incremental
						// 4.6 优化结果处理成功，转成增量里程计
						// Registration 得到的是 map → 当前帧；Odometry 需要的是 上一帧 → 当前帧
						transform = this->getPose().inverse() * transform;
					}
				}

				// 4.7 优化结果处理失败，打印错误日志
				if(transform.isNull())
				{
					if(guessIteration == 1)
					{
						UWARN("Trial with no guess still fail.");
					}
					if(!regInfo.rejectedMsg.empty())
					{
						if(guess.isNull())
						{
							UWARN("Registration failed: \"%s\"", regInfo.rejectedMsg.c_str());
						}
						else
						{
							UWARN("Registration failed: \"%s\" (guess=%s)", regInfo.rejectedMsg.c_str(), guess.prettyPrint().c_str());
						}
					}
					else
					{
						UWARN("Unknown registration error");
					}
				}
				else if(guessIteration == 1)
				{
					UWARN("Trial with no guess succeeded!");
				}
			}

			// 4.8 判断是否更新地图map
			if(!transform.isNull())
			{
				// 把 transform 变成里程计输出 & 绝对位姿
				// transform：增量里程计（上一帧 → 当前帧）
				output = transform;

				// 用于标记：局部地图是否被 BA 修改过
				bool modified = false;
				// newFramePose：当前帧在 map/world 坐标系下的绝对位姿
				Transform newFramePose = this->getPose()*output;

				// fields to update
				// 从 tmpMap 拷贝当前“局部地图内容”
				LaserScan mapScan = tmpMap.sensorData().laserScanRaw();
				std::multimap<int, int> mapWords = tmpMap.getWords();
				std::vector<cv::KeyPoint> mapWordsKpts = tmpMap.getWordsKpts();
				std::vector<cv::Point3f> mapPoints = tmpMap.getWords3();
				cv::Mat mapDescriptors = tmpMap.getWordsDescriptors();

				// update last frame features without depth (if bundle adjustment was done)
				// Do this before adding bundle frames to keep mono observations without depth
				// 用 BA 结果“补全当前帧的单目深度”
				bool lastFrameWords3Updated = false;
				std::vector<cv::Point3f> lastFrameWords3;
				// 触发条件：视觉里程计 & 不是 RGB-D 深度 mask & 启用了 BA & 当前帧有视觉特征 & 当前帧 存在“无效 3D 点”（NaN）& BA 里有优化后的 3D 点
				// 典型场景：单目视觉
				if( regPipeline_->isImageRequired() &&
					!visDepthAsMask &&
				    bundleAdjustment_>0 &&
					!lastFrame_->getWords().empty() &&
					lastFrame_->getWords().size() == lastFrame_->getWords3().size() &&
					!points3DMap.empty())
				{
					// 单目帧本身是“没有深度”的 但 BA 可能已经通过多视几何恢复了 3D 点
					// 这一步就是：把 BA 恢复的 3D 点反投影回当前帧，补上深度
					lastFrameWords3 = lastFrame_->getWords3();
					Transform newFramePoseInv = newFramePose.inverse();
					for(std::multimap<int, int>::const_iterator iter=lastFrame_->getWords().begin();
					    iter!=lastFrame_->getWords().end();
						++iter)
					{
						cv::Point3f & pt = lastFrameWords3.at(iter->second);
						if(!util3d::isFinite(pt))
						{
							std::map<int, cv::Point3f>::iterator mapIter = points3DMap.find(iter->first);
							if(mapIter != points3DMap.end())
							{
								// in base frame
								pt = util3d::transformPoint(mapIter->second, newFramePoseInv);
								lastFrameWords3Updated = true;
							}
						}
					}
				}

				// 是否用 BA 更新“整个局部地图的 3D 点”
				// 触发条件：视觉里程计 & 启用了 BA & 参数允许“回写 BA 优化后的 3D 点到 map” & BA 实际产生了 3D 点更新
				if( regPipeline_->isImageRequired() &&
					bundleAdjustment_>0 &&
					bundleUpdateFeatureMapOnAllFrames_ &&
					!points3DMap.empty())
				{
					// update local map 3D points (if bundle adjustment was done)
					for(std::map<int, cv::Point3f>::iterator iter=points3DMap.begin(); iter!=points3DMap.end(); ++iter)
					{
						UASSERT(mapWords.count(iter->first) == 1);
						mapPoints[mapWords.find(iter->first)->second] = iter->second;
					}
					// map 被修改了 → modified = true
					modified = true;
				}

				// 4.9 关键帧判定（非常重要）
				// 视觉关键帧条件：inliers 太少 || 匹配比例太低 || 像素运动够大
				bool addVisualKeyFrame = regPipeline_->isImageRequired() &&
						 (keyFrameThr_ == 0.0f ||
						  visKeyFrameThr_ == 0 ||
						  float(regInfo.inliers) <= (keyFrameThr_*float(lastFrame_->getWords().size())) ||
						  regInfo.inliers <= visKeyFrameThr_) &&
						  (bundleAdjustment_==0 || bundleAvgInlierDistance >= bundleMinMotion_);

				// 激光关键帧条件：icpInliersRatio <= scanKeyFrameThr_
				bool addGeometricKeyFrame = regPipeline_->isScanRequired() &&
					(scanKeyFrameThr_==0 || regInfo.icpInliersRatio <= scanKeyFrameThr_);

				addKeyFrame = addVisualKeyFrame || addGeometricKeyFrame;

				UDEBUG("keyframeThr=%f visKeyFrameThr_=%d matches=%d inliers=%d (avg dist=%f, min=%f) features=%d mp=%d",
					keyFrameThr_,
					visKeyFrameThr_,
					regInfo.matches,
					regInfo.inliers,
					bundleAvgInlierDistance,
					bundleMinMotion_,
					(int)lastFrame_->sensorData().keypoints().size(),
					(int)mapPoints.size());

				// 4.10 关键帧被接纳以后，更新局部地图（视觉 + 激光）
				// 如果是关键帧：添加新特征点（带深度 / 无深度）；剔除旧特征（超过 maximumMapSize_）
				// 更新：mapWords；mapPoints；mapDescriptors
				// 同时维护 BA 相关数据结构 bundleWordReferences_；bundlePoseReferences_；bundlePoses_
				if(addKeyFrame)
				{
					//Visual
					// 视觉部分
					int added = 0;
					int removed = 0;
					UTimer tmpTimer;

					UDEBUG("Update local map");
					// 初始化 & 约束检查
					// 只要 BA 开启，reference 关系一定会变，所以默认认为 map 会被修改
					modified = bundleAdjustment_>0; // We always add new references even if we don't add/remove points

					// update local map
					UASSERT(mapWords.size() == mapPoints.size());
					UASSERT(mapWords.size() == mapWordsKpts.size());
					UASSERT((int)mapPoints.size() == mapDescriptors.rows);
					UASSERT_MSG(lastFrame_->getWordsDescriptors().rows == (int)lastFrame_->getWords3().size(), uFormat("%d vs %d", lastFrame_->getWordsDescriptors().rows, (int)lastFrame_->getWords3().size()).c_str());

					std::map<int, int>::iterator iterBundlePosesRef = bundlePoseReferences_.end();
					if(bundleAdjustment_>0)
					{
						// BA：把当前关键帧正式加入 BA 图
						bundlePoseReferences_.insert(std::make_pair(lastFrame_->id(), 0));
						std::multimap<int, Link>::iterator iter = graph::findLink(bundleLinks, bundlePoses_.rbegin()->first, lastFrame_->id(), false);
						UASSERT(iter != bundleLinks.end());
						// 邻边加入
						bundleLinks_.insert(*iter);
						// 寻找当前帧->当前帧的边，这种是imu的重力约束边，加入BA图
						iter = graph::findLink(bundleLinks, lastFrame_->id(), lastFrame_->id(), false);
						if(iter != bundleLinks.end())
						{
							bundleIMUOrientations_.insert(*iter);
						}
						// 位姿加入
						uInsert(bundlePoses_, bundlePoses);
						UASSERT(bundleModels.find(lastFrame_->id()) != bundleModels.end());
						// 相机模型加入
						bundleModels_.insert(*bundleModels.find(lastFrame_->id()));
						iterBundlePosesRef = bundlePoseReferences_.find(lastFrame_->id());

						// 如果 不允许 BA 全量更新 map
						// 那么：只更新 points3DMap 中涉及的点，避免“地图几何频繁抖动
						if(!bundleUpdateFeatureMapOnAllFrames_)
						{
							// update local map 3D points (if bundle adjustment was done)
							for(std::map<int, cv::Point3f>::iterator iter=points3DMap.begin(); iter!=points3DMap.end(); ++iter)
							{
								UASSERT(mapWords.count(iter->first) == 1);
								mapPoints[mapWords.find(iter->first)->second] = iter->second;
							}
						}
					}

					// sort by feature response
					// 筛选“新特征候选”（newIds）：从当前关键帧中，选出“值得加入 map 的新特征”
					std::multimap<float, std::pair<int, std::pair<cv::KeyPoint, std::pair<cv::Point3f, std::pair<cv::Mat, int> > > > > newIds;
					UASSERT(lastFrame_->getWords3().size() == lastFrame_->getWords().size());
					UDEBUG("new frame words3=%d", (int)lastFrame_->getWords3().size());
					std::set<int> seenStatusUpdated;

					// add points without depth only if the local map has reached its maximum size
					// addPointsWithoutDepth 是否允许“无深度特征”
					bool addPointsWithoutDepth = false;
					// addPointsWithoutDepth的判断条件：单目 / 深度不完整 & 有足够比例的点有真实深度
					// 否则：拒绝加入无深度点，防止 map 退化
					if(!visDepthAsMask && validDepthRatio_ < 1.0f && !lastFrame_->getWords3().empty())
					{
						int ptsWithDepth = 0;
						for (std::vector<cv::Point3f>::const_iterator iter = lastFrame_->getWords3().begin();
							iter != lastFrame_->getWords3().end();
							++iter)
						{
							// 有真实深度
							if(util3d::isFinite(*iter))
							{
								++ptsWithDepth;
							}
						}
						float r = float(ptsWithDepth) / float(lastFrame_->getWords3().size());
						addPointsWithoutDepth = r > validDepthRatio_;
						if(!addPointsWithoutDepth)
						{
							UWARN("Not enough points with valid depth in current frame (%d/%d=%f < %s=%f), points without depth are not added to map.",
									ptsWithDepth, (int)lastFrame_->getWords3().size(), r, Parameters::kOdomF2MValidDepthRatio().c_str(), validDepthRatio_);
						}
					}

					if(!lastFrameModels.empty())
					{
						// 遍历当前帧所有 feature
						for(std::multimap<int, int>::const_iterator iter = lastFrame_->getWords().begin(); iter!=lastFrame_->getWords().end(); ++iter)
						{
							const cv::Point3f & pt = lastFrame_->getWords3()[iter->second];
							cv::KeyPoint kpt = lastFrame_->getWordsKpts()[iter->second];

							// 多相机情况下：把 keypoint 坐标还原到该子图
							int cameraIndex = 0;
							if(lastFrameModels.size()>1)
							{
								UASSERT(lastFrameModels[0].imageWidth()>0);
								float subImageWidth = lastFrameModels[0].imageWidth();
								cameraIndex = int(kpt.pt.x / subImageWidth);
								UASSERT(cameraIndex < (int)lastFrameModels.size());
								kpt.pt.x = kpt.pt.x - (subImageWidth*float(cameraIndex));
							}

							// 情况 1：这个特征不在 map 里 → 新点候选
							if(mapWords.find(iter->first) == mapWords.end()) // Point not in map
							{
								// 有深度 → OK
								// 没深度但允许 → OK
								// 否则 → 忽略
								if(util3d::isFinite(pt) || addPointsWithoutDepth)
								{
									newIds.insert(
											std::make_pair(kpt.response>0?1.0f/kpt.response:0.0f,
													std::make_pair(iter->first,
															std::make_pair(kpt,
																	std::make_pair(pt,
																			std::make_pair(lastFrame_->getWordsDescriptors().row(iter->second), cameraIndex))))));
								}
							}
							// 情况 2：这个特征已在 map 里 & BA 开启
							else if(bundleAdjustment_>0)
							{
								// 更新 BA 的 feature 观测
								if(lastFrame_->getWords().count(iter->first) == 1)
								{
									// 1 更新关键点 octave
									std::multimap<int, int>::iterator iterKpts = mapWords.find(iter->first);
									if(iterKpts!=mapWords.end() && !mapWordsKpts.empty())
									{
										mapWordsKpts[iterKpts->second].octave = kpt.octave;
									}

									UASSERT(iterBundlePosesRef!=bundlePoseReferences_.end());
									// 2 当前 keyframe 的引用计数 +1
									iterBundlePosesRef->second += 1;

									//move back point in camera frame (to get depth along z)
									// 3 计算该点在相机坐标系下的 depth
									float depth = 0.0f;
									if(util3d::isFinite(pt))
									{
										depth = util3d::transformPoint(pt, lastFrameModels[cameraIndex].localTransform().inverse()).z;
									}
									// 4 更新 bundleWordReferences_
									if(bundleWordReferences_.find(iter->first) == bundleWordReferences_.end())
									{
										std::map<int, FeatureBA> framePt;
										framePt.insert(std::make_pair(lastFrame_->id(), FeatureBA(kpt, depth, cv::Mat(), cameraIndex)));
										bundleWordReferences_.insert(std::make_pair(iter->first, framePt));
									}
									else
									{
										std::map<int, rtabmap::FeatureBA> & keyframes = bundleWordReferences_.find(iter->first)->second;
										// bundleMaxKeyFramesPerFeature_ 防止一个 feature 被太多 keyframe 观察，BA 规模爆炸
										if(bundleMaxKeyFramesPerFeature_ != 0 && (int)keyframes.size() > bundleMaxKeyFramesPerFeature_)
										{
											// To keep number of keyframes looking at same feature bounded
											int frameId = keyframes.rbegin()->first;
											UASSERT(bundlePoseReferences_.find(frameId) != bundlePoseReferences_.end());
											bundlePoseReferences_.at(frameId) -= 1;
											keyframes.erase(frameId);
										}
										keyframes.insert(std::make_pair(lastFrame_->id(), FeatureBA(kpt, depth, cv::Mat(), cameraIndex)));
									}
								}
							}
						}
						UDEBUG("newIds=%d", (int)newIds.size());
					}

					int lastFrameOldestNewId = lastFrameOldestNewId_;
					lastFrameOldestNewId_ = lastFrame_->getWords().size()?lastFrame_->getWords().rbegin()->first:0;
					// 真正把新特征加入 map
					for(std::multimap<float, std::pair<int, std::pair<cv::KeyPoint, std::pair<cv::Point3f, std::pair<cv::Mat, int> > > > >::reverse_iterator iter=newIds.rbegin();
						iter!=newIds.rend();
						++iter)
					{
						// 限制条件 maxNewFeatures_
						if(maxNewFeatures_ == 0  || added < maxNewFeatures_)
						{
							int cameraIndex = iter->second.second.second.second.second;
							cv::Point3f pt = iter->second.second.second.first;
							// 如果是“无深度点”
							if(!util3d::isFinite(pt))
							{
								// get the ray instead
								// 构造一条 ray，给一个 初始化深度
								// 本质是：“我不知道多远，但给你一个可 BA 优化的初值”
								float x = iter->second.second.first.pt.x; //subImageWidth should be already removed
								float y = iter->second.second.first.pt.y;
								Eigen::Vector3f ray = util3d::projectDepthTo3DRay(
										lastFrameModels[cameraIndex].imageSize(),
										x,
										y,
										lastFrameModels[cameraIndex].cx(),
										lastFrameModels[cameraIndex].cy(),
										lastFrameModels[cameraIndex].fx(),
										lastFrameModels[cameraIndex].fy());
								float scaleInf = initDepthFactor_ * lastFrameModels[cameraIndex].fx();
								pt = util3d::transformPoint(cv::Point3f(ray[0]*scaleInf, ray[1]*scaleInf, ray[2]*scaleInf), lastFrameModels[cameraIndex].localTransform()); // in base_link frame
							}
							// 地面点过滤 floorThreshold_
							if(floorThreshold_ != 0.0f && pt.z < floorThreshold_)
							{
								continue;
							}

							// 更新 BA 的 feature 观测
							if(bundleAdjustment_>0)
							{
								if(lastFrame_->getWords().count(iter->second.first) == 1)
								{
									UASSERT(iterBundlePosesRef!=bundlePoseReferences_.end());
									iterBundlePosesRef->second += 1;

									//move back point in camera frame (to get depth along z)
									float depth = 0.0f;
									if(util3d::isFinite(iter->second.second.second.first))
									{
										depth = util3d::transformPoint(iter->second.second.second.first, lastFrameModels[cameraIndex].localTransform().inverse()).z;
									}
									if(bundleWordReferences_.find(iter->second.first) == bundleWordReferences_.end())
									{
										std::map<int, FeatureBA> framePt;
										framePt.insert(std::make_pair(lastFrame_->id(), FeatureBA(iter->second.second.first, depth, cv::Mat(), cameraIndex)));
										bundleWordReferences_.insert(std::make_pair(iter->second.first, framePt));
									}
									else
									{
										bundleWordReferences_.find(iter->second.first)->second.insert(std::make_pair(lastFrame_->id(), FeatureBA(iter->second.second.first, depth, cv::Mat(), cameraIndex)));
									}
								}
							}

							// 加入 map
							mapWords.insert(mapWords.end(), std::make_pair(iter->second.first, mapWords.size()));
							mapWordsKpts.push_back(iter->second.second.first);
							mapPoints.push_back(util3d::transformPoint(pt, newFramePose));
							mapDescriptors.push_back(iter->second.second.second.second.first);
							if(lastFrameOldestNewId_ > iter->second.first)
							{
								lastFrameOldestNewId_ = iter->second.first;
							}
							++added;
						}
						else
						{
							break;
						}
					}
					UDEBUG("");

					// remove words in map if max size is reached
					// map 超限 → 删除旧特征
					if((int)mapWords.size() > maximumMapSize_)
					{
						// remove oldest outliers first
						// 优先删：不是 inlier 的；不是当前帧 / 新帧投影的；最老的
						// 第一步：构建“当前帧的 inlier 集合”
						std::set<int> inliers(regInfo.inliersIDs.begin(), regInfo.inliersIDs.end());
						// 第二步：构造“优先删除候选列表 ids”
						// matchesIDs 是 当前帧尝试匹配过的 map 点，包含：inliers，outliers（匹配失败的），这些点是“最不新鲜、最容易被淘汰”的候选
						std::vector<int> ids = regInfo.matchesIDs;
						// 第三步：把 projectedIDs 也加入候选
						// projectedIDs 是当前帧通过 投影方式（motion model / ICP / guess）预测出来的 map points，不一定真的被观测到
						if(regInfo.projectedIDs.size())
						{
							ids.resize(ids.size() + regInfo.projectedIDs.size());
							int oi=0;
							for(unsigned int i=0; i<regInfo.projectedIDs.size(); ++i)
							{
								// lastFrameOldestNewId 当前帧新加入 feature 的最小 wordId
								// 判断条件：只考虑“老于当前帧”的 projected 点
								// 设计动机：刚加入的新点，哪怕暂时没被匹配，也不能立刻删
								if(regInfo.projectedIDs[i]>=lastFrameOldestNewId)
								{
									ids[regInfo.matchesIDs.size()+oi++] = regInfo.projectedIDs[i];
								}
							}
							ids.resize(regInfo.matchesIDs.size()+oi);
							UDEBUG("projected added=%d/%d minLastFrameId=%d", oi, (int)regInfo.projectedIDs.size(), lastFrameOldestNewId);
						}
						// 第四步：真正开始删点（第一轮）
						for(unsigned int i=0; i<ids.size() && (int)mapWords.size() > maximumMapSize_ && mapWords.size() >= newIds.size(); ++i)
						{
							int id = ids.at(i);
							// 删除条件：不是 inlier 的点
							if(inliers.find(id) == inliers.end())
							{
								// 删除一个 map point 时，必须同步清理 BA 数据
								std::map<int, std::map<int, FeatureBA> >::iterator iterRef = bundleWordReferences_.find(id);
								// 1 查这个点是否参与 BA
								if(iterRef != bundleWordReferences_.end())
								{
									// 这个点正参与BA（曾被多个 keyframe 观测），因此需要遍历所有相关的引用
									for(std::map<int, FeatureBA>::iterator iterFrame = iterRef->second.begin(); iterFrame != iterRef->second.end(); ++iterFrame)
									{
										if(bundlePoseReferences_.find(iterFrame->first) != bundlePoseReferences_.end())
										{
											// 2 更新每个 keyframe 的引用计数
											// bundlePoseReferences_ 表示：这个 keyframe 还剩多少个 feature 在 BA 里引用它
											// 如果不减：BA 会以为这个 pose 仍然有约束，后面就不会被正确清理
											bundlePoseReferences_.at(iterFrame->first) -= 1;
										}
									}
									// 3 从 BA 中彻底移除这个 feature
									bundleWordReferences_.erase(iterRef);
								}

								// 最后：从 map 中删除该 feature
								mapWords.erase(id);
								++removed;
							}
						}

						// remove oldest first
						// 第五步：真正开始删点（第二轮），按时间顺序删
						// mapWords.begin() 最老的 wordId 在前面
						// mapWords.size() > maximumMapSize_  只要还超容量就继续删
						// mapWords.size() >= newIds.size()  保证新加的点不会被全删光
						// 从最老的点开始删，直到 map 回到安全大小
						for(std::multimap<int, int>::iterator iter = mapWords.begin();
							iter!=mapWords.end() && (int)mapWords.size() > maximumMapSize_ && mapWords.size() >= newIds.size();)
						{
							// 删除条件仍然是：不是 inlier
							if(inliers.find(iter->first) == inliers.end())
							{
								std::map<int, std::map<int, FeatureBA> >::iterator iterRef = bundleWordReferences_.find(iter->first);
								// 1 查这个点是否参与 BA
								if(iterRef != bundleWordReferences_.end())
								{
									for(std::map<int, FeatureBA>::iterator iterFrame = iterRef->second.begin(); iterFrame != iterRef->second.end(); ++iterFrame)
									{
										if(bundlePoseReferences_.find(iterFrame->first) != bundlePoseReferences_.end())
										{
											bundlePoseReferences_.at(iterFrame->first) -= 1;
										}
									}
									bundleWordReferences_.erase(iterRef);
								}

								mapWords.erase(iter++);
								++removed;
							}
							else
							{
								++iter;
							}
						}

						// 压缩 & 重建 map 数据结构
						// 前面 mapWords.erase(...) 但 mapPoints / mapWordsKpts / mapDescriptors 还没删！
						if(mapWords.size() != mapPoints.size())
						{
							UDEBUG("Remove points");
							std::vector<cv::KeyPoint> mapWordsKptsClean(mapWords.size());
							std::vector<cv::Point3f> mapPointsClean(mapWords.size());
							cv::Mat mapDescriptorsClean(mapWords.size(), mapDescriptors.cols, mapDescriptors.type());
							int index = 0;
							// 重建逻辑（本质）
							// 按 mapWords 的顺序（wordId 从小到大）
							// 从旧数组里取数据
							// 填入新的紧凑数组
							// 重写 mapWords[wordId] = newIndex
							for(std::multimap<int, int>::iterator iter = mapWords.begin(); iter!=mapWords.end(); ++iter, ++index)
							{
								mapWordsKptsClean[index] = mapWordsKpts[iter->second];
								mapPointsClean[index] = mapPoints[iter->second];
								mapDescriptors.row(iter->second).copyTo(mapDescriptorsClean.row(index));
								iter->second = index;
							}
							mapWordsKpts = mapWordsKptsClean;
							mapWordsKptsClean.clear();
							mapPoints = mapPointsClean;
							mapPointsClean.clear();
							mapDescriptors = mapDescriptorsClean;
						}

						// 清理 BA 中“已无 feature 引用的关键帧”
						Link * previousLink = 0;
						for(std::map<int, int>::iterator iter=bundlePoseReferences_.begin(); iter!=bundlePoseReferences_.end();)
						{
							// bundlePoseReferences_[id] <= 0：这个 keyframe 已经不再贡献任何 BA 约束
							if(iter->second <= 0)
							{
								if(previousLink == 0 || bundleLinks_.find(iter->first) != bundleLinks_.end())
								{
									if(previousLink)
									{
										UASSERT(previousLink->to() == iter->first);
										*previousLink = previousLink->merge(bundleLinks_.find(iter->first)->second, previousLink->type());
									}
									UASSERT(bundlePoses_.erase(iter->first) == 1);
									bundleLinks_.erase(iter->first);
									bundleModels_.erase(iter->first);
									bundleIMUOrientations_.erase(iter->first);
									bundlePoseReferences_.erase(iter++);
								}
							}
							else
							{
								previousLink=0;
								if(bundleLinks_.find(iter->first) != bundleLinks_.end())
								{
									previousLink = &bundleLinks_.find(iter->first)->second;
								}
								++iter;
							}
						}
					}

					if(added || removed)
					{
						modified = true;
					}
					UDEBUG("Update local features map = %fs", tmpTimer.ticks());

					// Geometric
					// 激光雷达 / 深度点云 SLAM 中“局部扫描地图（local scan map）更新
					// 当 ICP 内点率低于阈值时，将当前激光帧中“新出现的点”融合进局部扫描地图 mapScan，同时限制地图范围和点数，并重建法线，最终生成新的 LaserScan。
					UDEBUG("scankeyframeThr=%f icpInliersRatio=%f", scanKeyFrameThr_, regInfo.icpInliersRatio);
					UINFO("Update local scan map %d (ratio=%f < %f)", lastFrame_->id(), regInfo.icpInliersRatio, scanKeyFrameThr_);

					// 前提条件：必须有激光数据
					if(lastFrame_->sensorData().laserScanRaw().size())
					{
						// 构建地图点云 & 当前帧点云（带法线）
						// 将已有的 mapScan 转成 世界坐标系下的点云
						pcl::PointCloud<pcl::PointXYZINormal>::Ptr mapCloudNormals = util3d::laserScanToPointCloudINormal(mapScan, tmpMap.sensorData().laserScanRaw().localTransform());
						// 当前帧视角（viewpoint）
						Transform viewpoint =  newFramePose * lastFrame_->sensorData().laserScanRaw().localTransform();
						pcl::PointCloud<pcl::PointXYZINormal>::Ptr frameCloudNormals (new pcl::PointCloud<pcl::PointXYZINormal>());
						
						// 当前帧点云构建（是否裁剪）
						if(scanMapMaxRange_ > 0)
						{
							// 情况 A：限制局部地图大小（scanMapMaxRange_ > 0）
							frameCloudNormals = util3d::laserScanToPointCloudINormal(lastFrame_->sensorData().laserScanRaw());
							frameCloudNormals = util3d::cropBox(frameCloudNormals,
									Eigen::Vector4f(-scanMapMaxRange_ / 2, -scanMapMaxRange_ / 2,-scanMapMaxRange_ / 2, 0),
									Eigen::Vector4f(scanMapMaxRange_ / 2,scanMapMaxRange_ / 2,scanMapMaxRange_ / 2, 0)
									);
							frameCloudNormals = util3d::transformPointCloud(frameCloudNormals, viewpoint);
						} else
						{
							// 情况 B：不限制范围
							frameCloudNormals = util3d::laserScanToPointCloudINormal(lastFrame_->sensorData().laserScanRaw(), viewpoint);
						}
						
						pcl::IndicesPtr frameCloudNormalsIndices(new std::vector<int>);
						int newPoints;
						// 去重：只保留“新点”
						if(mapCloudNormals->size() && scanSubtractRadius_ > 0.0f)
						{
							// remove points that overlap (the ones found in both clouds)
							// subtractFiltering 的作用：删除与已有地图重叠的点
							// 判断条件：距离 < scanSubtractRadius_，法线夹角 < scanSubtractAngle_（若有法线）
							// 结果：frameCloudNormalsIndices → 真正的新点索引
							frameCloudNormalsIndices = util3d::subtractFiltering(
									frameCloudNormals,
									pcl::IndicesPtr(new std::vector<int>),
									mapCloudNormals,
									pcl::IndicesPtr(new std::vector<int>),
									scanSubtractRadius_,
									lastFrame_->sensorData().laserScanRaw().hasNormals()&&mapScan.hasNormals()?scanSubtractAngle_:0.0f);
							newPoints = frameCloudNormalsIndices->size();
						}
						else
						{
							newPoints = frameCloudNormals->size();
						}

						// 如果确实有新点 → 融合进地图
						if(newPoints)
						{
							// 两种地图管理策略
							// A. scanMapMaxRange_ > 0（滑动窗口地图）
							if (scanMapMaxRange_ > 0) {
								// Copying new points to tmp cloud
								// These are the points that have no overlap between mapScan and lastFrame
								// 添加新点，使用tmp作为临时点云
								pcl::PointCloud<pcl::PointXYZINormal> tmp;
								pcl::copyPointCloud(*frameCloudNormals, *frameCloudNormalsIndices, tmp);

								// 地图太大？裁剪空间范围
								// 裁剪到当前帧周围的 立方体窗口，形成“局部子地图”
								if (int(mapCloudNormals->size() + newPoints) > scanMaximumMapSize_) // 20 000 points
								{
									// Print mapSize
									UINFO("mapSize=%d newPoints=%d maxPoints=%d",
										  int(mapCloudNormals->size()),
										  newPoints,
										  scanMaximumMapSize_);

									*mapCloudNormals += tmp;
									cv::Point3f boxMin (-scanMapMaxRange_/2, -scanMapMaxRange_/2, -scanMapMaxRange_/2);
									cv::Point3f boxMax (scanMapMaxRange_/2, scanMapMaxRange_/2, scanMapMaxRange_/2);

									boxMin = util3d::transformPoint(boxMin, viewpoint.translation());
									boxMax = util3d::transformPoint(boxMax, viewpoint.translation());

									mapCloudNormals = util3d::cropBox(mapCloudNormals, Eigen::Vector4f(boxMin.x, boxMin.y, boxMin.z, 0 ), Eigen::Vector4f(boxMax.x, boxMax.y, boxMax.z, 0 ));

								} else {
									*mapCloudNormals += tmp;
								}

								// 体素滤波
								mapCloudNormals = util3d::voxelize(mapCloudNormals, scanSubtractRadius_);
								pcl::PointCloud<pcl::PointXYZI>::Ptr mapCloud (new pcl::PointCloud<pcl::PointXYZI> ());
								copyPointCloud(*mapCloudNormals, *mapCloud);
								// 重新计算法线
								pcl::PointCloud<pcl::Normal>::Ptr normals = util3d::computeNormals(mapCloud, pointToPlaneK_, pointToPlaneRadius_, Eigen::Vector3f(viewpoint.x(), viewpoint.y(), viewpoint.z()));
								copyPointCloud(*normals, *mapCloudNormals);

							}
							// B. scanMapMaxRange_ == 0（缓冲区方式） 
							else 
							{
								// 保存最近若干帧扫描
								scansBuffer_.push_back(std::make_pair(frameCloudNormals, frameCloudNormalsIndices));

								//remove points if too big
								UDEBUG("scansBuffer=%d, mapSize=%d newPoints=%d maxPoints=%d",
									   (int)scansBuffer_.size(),
									   int(mapCloudNormals->size()),
									   newPoints,
									   scanMaximumMapSize_);

								if(scansBuffer_.size() > 1 &&
								   int(mapCloudNormals->size() + newPoints) > scanMaximumMapSize_)
								{
									//regenerate the local map
									// 地图太大时：重建
									mapCloudNormals->clear();
									std::list<int> toRemove;
									int i = int(scansBuffer_.size())-1;
									for(; i>=0; --i)
									{
										int pointsToAdd = scansBuffer_[i].second->size()?scansBuffer_[i].second->size():scansBuffer_[i].first->size();
										if((int)mapCloudNormals->size() + pointsToAdd > scanMaximumMapSize_ ||
										   i == 0)
										{
											*mapCloudNormals += *scansBuffer_[i].first;
											break;
										}
										else
										{
											if(scansBuffer_[i].second->size())
											{
												pcl::PointCloud<pcl::PointXYZINormal> tmp;
												pcl::copyPointCloud(*scansBuffer_[i].first, *scansBuffer_[i].second, tmp);
												*mapCloudNormals += tmp;
											}
											else
											{
												*mapCloudNormals += *scansBuffer_[i].first;
											}
										}
									}
									// remove old clouds
									// 丢弃最旧扫描
									if(i > 0)
									{
										std::vector<std::pair<pcl::PointCloud<pcl::PointXYZINormal>::Ptr, pcl::IndicesPtr> > scansTmp(scansBuffer_.size()-i);
										int oi = 0;
										for(; i<(int)scansBuffer_.size(); ++i)
										{
											UASSERT(oi < (int)scansTmp.size());
											scansTmp[oi++] = scansBuffer_[i];
										}
										scansBuffer_ = scansTmp;
									}
								}
								else
								{
									// just append the last cloud
									// 保留最新扫描
									if(scansBuffer_.back().second->size())
									{
										pcl::PointCloud<pcl::PointXYZINormal> tmp;
										pcl::copyPointCloud(*scansBuffer_.back().first, *scansBuffer_.back().second, tmp);
										*mapCloudNormals += tmp;
									}
									else
									{
										*mapCloudNormals += *scansBuffer_.back().first;
									}
								}
							}

							// 点云 → LaserScan（非常关键）
							// 局部地图最终仍以 LaserScan 形式存在
							// 为下一轮 ICP 提供输入
							if(mapScan.is2d())
							{
								Transform mapViewpoint(-newFramePose.x(), -newFramePose.y(),0,0,0,0);
								mapScan = LaserScan(util3d::laserScan2dFromPointCloud(*mapCloudNormals, mapViewpoint), 0, 0.0f);
							}
							else
							{
								Transform mapViewpoint(-newFramePose.x(), -newFramePose.y(), -newFramePose.z(),0,0,0);
								mapScan = LaserScan(util3d::laserScanFromPointCloud(*mapCloudNormals, mapViewpoint), 0, 0.0f);
							}
							// 标记局部地图发生变化
							// 通知系统更新缓存 / 重新配准
							modified=true;
						}
					}
					UDEBUG("Update local scan map = %fs", tmpTimer.ticks());
				}

				// 4.11 更新局部激光地图（Scan Map）
				if(modified)
				{
					*map_ = tmpMap;

					if(mapScan.is2d())
					{

						map_->sensorData().setLaserScan(
								LaserScan(
										mapScan.data(),
										0,
										0.0f,
										mapScan.format(),
										Transform(newFramePose.x(), newFramePose.y(), lastFrame_->sensorData().laserScanRaw().localTransform().z(),0,0,0)));
					}
					else
					{
						map_->sensorData().setLaserScan(
								LaserScan(
										mapScan.data(),
										0,
										0.0f,
										mapScan.format(),
										newFramePose.translation()));
					}

					map_->setWords(mapWords, mapWordsKpts, mapPoints, mapDescriptors);
				}

				if(lastFrameWords3Updated)
				{
					// update output with refined 3d points from bundle adjustment
					data.setFeatures(lastFrame_->getWordsKpts(), lastFrameWords3, lastFrame_->getWordsDescriptors());
				}
			}

			if(info)
			{
				// use tmpMap instead of map_ to make sure that correspondences with the new frame matches
				info->localMapSize = (int)tmpMap.getWords3().size();
				info->localScanMapSize = tmpMap.sensorData().laserScanRaw().size();
				if(this->isInfoDataFilled())
				{
					info->localMap.clear();
					if(!tmpMap.getWords3().empty())
					{
						for(std::multimap<int, int>::const_iterator iter=tmpMap.getWords().begin(); iter!=tmpMap.getWords().end(); ++iter)
						{
							info->localMap.insert(std::make_pair(iter->first, tmpMap.getWords3()[iter->second]));
						}
					}
					info->localScanMap = tmpMap.sensorData().laserScanRaw();
				}
			}
		}
		// 不满足F2M条件，当前还没有地图：只做初始化
		else
		{
			// Just generate keypoints for the new signature
			// For scan, we want to use reading filters, so set dummy's scan and set back to reference afterwards
			// 这段代码出现在 RTAB-Map 前端里“生成新节点（Signature）的特征，但不把它和上一帧建立几何约束” 的场景中。
			// 不做位姿约束 只生成特征，激光扫描要走 reading filters（滤波流程）
			
			// Signature：图优化里的一个节点（一帧）
			Signature dummy;
			dummy.sensorData().setLaserScan(lastFrame_->sensorData().laserScanRaw());
			// 临时移走激光数据
			lastFrame_->sensorData().setLaserScan(LaserScan());
			// regPipeline_：配准流水线（视觉 + 激光）
			// computeTransformationMod()：会提取特征，也可能做几何/ICP。如果当前帧有激光 → 会尝试用它做几何配准
			// 但这里你不想让 lastFrame_ 真正参与几何约束，你只想：生成 keypoints, 使用激光滤波器. 因此需要临时移走激光数据
			regPipeline_->computeTransformationMod(
					*lastFrame_,
					dummy);
			// 把激光数据还回 lastFrame_
			lastFrame_->sensorData().setLaserScan(dummy.sensorData().laserScanRaw());

			// 把特征拷贝到 Data 结构中
			data.setFeatures(lastFrame_->sensorData().keypoints(), lastFrame_->sensorData().keypoints3D(), lastFrame_->sensorData().descriptors());
			data.setLaserScan(lastFrame_->sensorData().laserScanRaw());

			// a very high variance tells that the new pose is not linked with the previous one
			// covariance 很大：表示“不要信这个约束”,当前帧相当于“漂浮节点”
			regInfo.covariance = cv::Mat::eye(6,6,CV_64FC1)*9999.0;

			// 初始化状态变量 表示：当前帧 还没有被成功注册, 需要等待回环 / 后续对齐
			bool frameValid = false;
			Transform newFramePose = this->getPose(); // 获取当前位姿（不一定是 Identity）

			// 视觉处理
			if(regPipeline_->isImageRequired())
			{
				// 用当前帧“初始化地图 + 位姿”，并为后续 Bundle Adjustment 做准备；如果条件不满足，就认为里程计初始化失败。
				int ptsWithDepth = 0;
				// 统计“有深度的视觉特征点”数量
				for (std::multimap<int, int>::const_iterator iter = lastFrame_->getWords().begin();
					iter != lastFrame_->getWords().end();
					++iter)
				{
					if(!lastFrame_->getWords3().empty() && 
					   util3d::isFinite(lastFrame_->getWords3()[iter->second]))
					{
						++ptsWithDepth;
					}
				}

				// 是否 >= 最小视觉对应数
				// 若满足条件：用当前帧初始化视觉 / 激光地图；output = Identity
				if (ptsWithDepth >= regPipeline_->getMinVisualCorrespondences())
				{
					// 标记帧有效 & 数据一致性检查
					// 确保：每个 3D 点都有描述子 & 每个 word 都有对应的 3D 点
					// 防止特征 / 描述子 / 3D 点错位
					frameValid = true;
					// update local map
					UASSERT_MSG(lastFrame_->getWordsDescriptors().rows == (int)lastFrame_->getWords3().size(), uFormat("%d vs %d", lastFrame_->getWordsDescriptors().rows, (int)lastFrame_->getWords3().size()).c_str());
					UASSERT(lastFrame_->getWords3().size() == lastFrame_->getWords().size());

					// 构建局部地图所需的数据结构
					std::multimap<int, int> words;
					std::vector<cv::KeyPoint> wordsKpts;
					std::vector<cv::Point3f> transformedPoints;
					std::multimap<int, int> mapPointWeights;
					cv::Mat descriptors;
					// 遍历特征点，构建地图点
					if(!lastFrame_->getWords3().empty() && !lastFrameModels.empty())
					{
						for (std::multimap<int, int>::const_iterator iter = lastFrame_->getWords().begin();
							iter != lastFrame_->getWords().end();
							++iter)
						{
							const cv::Point3f & pt = lastFrame_->getWords3()[iter->second];
							// 该 3D 点有效,则保存该点
							if (util3d::isFinite(pt))
							{
								words.insert(words.end(), std::make_pair(iter->first, words.size()));
								wordsKpts.push_back(lastFrame_->getWordsKpts()[iter->second]);
								// 把 相机坐标系下的 3D 点 变换到 当前世界坐标系
								// 这是“地图初始化”的核心动作。
								transformedPoints.push_back(util3d::transformPoint(pt, newFramePose));
								mapPointWeights.insert(std::make_pair(iter->first, 0));
								descriptors.push_back(lastFrame_->getWordsDescriptors().row(iter->second));
							}
						}
					}

					// Bundle Adjustment 初始化（如果开启）
					if(bundleAdjustment_>0)
					{
						// update bundleWordReferences_: used for bundle adjustment
						if(!wordsKpts.empty())
						{
							// 初始化 BA 的“特征观测关系”
							for(std::multimap<int, int>::const_iterator iter=words.begin(); iter!=words.end(); ++iter)
							{
								// 只处理 唯一出现的特征点
								// 避免一个 word 在同一帧多次出现，造成 BA 约束混乱。
								if(words.count(iter->first) == 1)
								{
									UASSERT(bundleWordReferences_.find(iter->first) == bundleWordReferences_.end());
									std::map<int, FeatureBA> framePt;

									cv::KeyPoint kpt = wordsKpts[iter->second];

									// 多相机（多目）支持
									int cameraIndex = 0;
									if(lastFrameModels.size()>1)
									{
										UASSERT(lastFrameModels[0].imageWidth()>0);
										float subImageWidth = lastFrameModels[0].imageWidth();
										cameraIndex = int(kpt.pt.x / subImageWidth);
										kpt.pt.x = kpt.pt.x - (subImageWidth*float(cameraIndex));
									}

									//get depth
									// 计算深度 d（用于 BA）
									float d = 0.0f;
									if(lastFrame_->getWords().count(iter->first) == 1 &&
									  !lastFrame_->getWords3().empty() &&
										util3d::isFinite(lastFrame_->getWords3()[lastFrame_->getWords().find(iter->first)->second]))
									{
										//move back point in camera frame (to get depth along z)
										d = util3d::transformPoint(lastFrame_->getWords3()[lastFrame_->getWords().find(iter->first)->second], lastFrameModels[cameraIndex].localTransform().inverse()).z;
									}


									framePt.insert(std::make_pair(lastFrame_->id(), FeatureBA(kpt, d, cv::Mat(), cameraIndex)));
									bundleWordReferences_.insert(std::make_pair(iter->first, framePt));
								}
							}
						}

						// 初始化 BA 的位姿、模型、IMU
						bundlePoseReferences_.insert(std::make_pair(lastFrame_->id(), (int)bundleWordReferences_.size()));
						bundleModels_.insert(std::make_pair(lastFrame_->id(), lastFrameModels));
						bundlePoses_.insert(std::make_pair(lastFrame_->id(), newFramePose));

						if(!imuT.isNull())
						{
							bundleIMUOrientations_.insert(std::make_pair(lastFrame_->id(), Link(lastFrame_->id(), lastFrame_->id(), Link::kGravity, newFramePose)));
						}
					}

					// 真正“写入地图”
					map_->setWords(words, wordsKpts, transformedPoints, descriptors);
					addKeyFrame = true;
				}
				// 否则：认为里程计初始化失败, odom 丢失
				else
				{
					UWARN("%d visual features required to initialize the odometry (only %d extracted).", regPipeline_->getMinVisualCorrespondences(), (int)lastFrame_->getWords3().size());
				}
			}
			// 激光处理
			if(regPipeline_->isScanRequired())
			{
				if (lastFrame_->sensorData().laserScanRaw().size())
				{
					pcl::PointCloud<pcl::PointXYZINormal>::Ptr mapCloudNormals = util3d::laserScanToPointCloudINormal(lastFrame_->sensorData().laserScanRaw(), newFramePose * lastFrame_->sensorData().laserScanRaw().localTransform());

					double complexity = 0.0;;
					if(!frameValid)
					{
						float minComplexity = Parameters::defaultIcpPointToPlaneMinComplexity();
						bool p2n = Parameters::defaultIcpPointToPlane();
						Parameters::parse(parameters_, Parameters::kIcpPointToPlane(), p2n);
						Parameters::parse(parameters_, Parameters::kIcpPointToPlaneMinComplexity(), minComplexity);
						if(p2n && minComplexity>0.0f)
						{
							if(lastFrame_->sensorData().laserScanRaw().hasNormals())
							{
								complexity = util3d::computeNormalsComplexity(*mapCloudNormals, Transform::getIdentity(), lastFrame_->sensorData().laserScanRaw().is2d());
								if(complexity > minComplexity)
								{
									frameValid = true;
								}
								else if(!guess.isNull() && !guess.isIdentity())
								{
									UWARN("Scan complexity too low (%f) to init robustly the first "
											"keyframe. Make sure the lidar is seeing enough "
											"geometry in all axes for good initialization. "
											"Accepting as an initial guess (%s) is provided.",
											complexity,
											guess.prettyPrint().c_str());
									frameValid = true;
								}
							}
							else
							{
								UWARN("Input raw scan doesn't have normals, complexity check on first frame is not done.");
								frameValid = true;
							}
						}
						else
						{
							frameValid = true;
						}
					}

					if(frameValid)
					{
						if (scanMapMaxRange_ > 0 ){
							UINFO("Local map will be updated using range instead of time with range threshold set at %f", scanMapMaxRange_);
						} else {
							scansBuffer_.push_back(std::make_pair(mapCloudNormals, pcl::IndicesPtr(new std::vector<int>)));
						}
						if(lastFrame_->sensorData().laserScanRaw().is2d())
						{
							Transform mapViewpoint(-newFramePose.x(), -newFramePose.y(),0,0,0,0);
							map_->sensorData().setLaserScan(
									LaserScan(
											util3d::laserScan2dFromPointCloud(*mapCloudNormals, mapViewpoint),
											0,
											0.0f,
											Transform(newFramePose.x(), newFramePose.y(), lastFrame_->sensorData().laserScanRaw().localTransform().z(),0,0,0)));
						}
						else
						{
							Transform mapViewpoint(-newFramePose.x(), -newFramePose.y(), -newFramePose.z(),0,0,0);
							map_->sensorData().setLaserScan(
									LaserScan(
											util3d::laserScanFromPointCloud(*mapCloudNormals, mapViewpoint),
											0,
											0.0f,
											newFramePose.translation()));
						}

						addKeyFrame = true;
					}
					else
					{
						UWARN("Scan complexity too low (%f) to init first keyframe.", complexity);
					}
				}
				else
				{
					UWARN("Missing scan to initialize odometry.");
				}
			}

			if (frameValid)
			{
				// We initialized the local map
				// 如果帧无效,设置output初值
				output.setIdentity();
			}

			// 数据记录
			if(info)
			{
				info->localMapSize = (int)map_->getWords3().size();
				info->localScanMapSize = map_->sensorData().laserScanRaw().size();

				if(this->isInfoDataFilled())
				{
					info->localMap.clear();
					if(!map_->getWords3().empty())
					{
						for(std::multimap<int, int>::const_iterator iter=map_->getWords().begin(); iter!=map_->getWords().end(); ++iter)
						{
							info->localMap.insert(std::make_pair(iter->first, map_->getWords3()[iter->second]));
						}
					}
					info->localScanMap = map_->sensorData().laserScanRaw();
				}
			}
		}

		// 清空 map 中的 sensorData 特征
		// Map 的 words / map points	用于定位、建图、BA（核心数据）
		// sensorData 里的 features	临时缓存，用于发布 / 可视化 / debug
		// 为什么要清空？sensorData 往往只保存 最近一帧 的特征
		// 避免：重复发布旧特征, 可视化显示错误, 内存堆积
		// 这一步 不会影响里程计或地图，只是“清理展示层数据”。
		map_->sensorData().setFeatures(std::vector<cv::KeyPoint>(), std::vector<cv::Point3f>(), cv::Mat()); // clear sensorData features
		// 统计当前帧特征数量
		// 通常用于：状态信息, 调试输出, UI / ROS topic 发布
		nFeatures = lastFrame_->getWords().size();
		if(this->isInfoDataFilled() && info)
		{
			// 仅在需要图像时才填充视觉信息
			if(regPipeline_->isImageRequired())
			{
				info->words.clear();
				if(!lastFrame_->getWordsKpts().empty())
				{
					for(std::multimap<int, int>::const_iterator iter=lastFrame_->getWords().begin(); iter!=lastFrame_->getWords().end(); ++iter)
					{
						info->words.insert(std::make_pair(iter->first, lastFrame_->getWordsKpts()[iter->second]));
					}
				}
			}
		}
	}
	// 没有有效传感器数据时丢弃此帧
	else
	{
		UERROR("SensorData not valid!");
	}

	// 11 填充 OdometryInfo & 返回
	if(info)
	{
		info->features = nFeatures;
		info->localKeyFrames = (int)bundlePoses_.size();
		info->keyFrameAdded = addKeyFrame;
		info->localBundleOutliers = totalBundleOutliers;
		info->localBundleConstraints = totalBundleWordReferencesUsed;
		info->localBundleTime = bundleTime;

		if(this->isInfoDataFilled())
		{
			info->reg = regInfo;
		}
		else
		{
			info->reg = regInfo.copyWithoutData();
		}
	}

	UINFO("Odom update time = %fs lost=%s features=%d inliers=%d/%d variance:lin=%f, ang=%f local_map=%d local_scan_map=%d",
			timer.elapsed(),
			output.isNull()?"true":"false",
			nFeatures,
			regInfo.inliers,
			regInfo.matches,
			!regInfo.covariance.empty()?regInfo.covariance.at<double>(0,0):0,
			!regInfo.covariance.empty()?regInfo.covariance.at<double>(5,5):0,
			regPipeline_->isImageRequired()?(int)map_->getWords3().size():0,
			regPipeline_->isScanRequired()?(int)map_->sensorData().laserScanRaw().size():0);
	return output;
}

} // namespace rtabmap
