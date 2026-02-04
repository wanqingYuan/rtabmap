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

#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/core/Version.h"
#include "rtabmap/core/Features2d.h"
#include "rtabmap/core/Optimizer.h"
#include "rtabmap/core/Graph.h"
#include "rtabmap/core/Signature.h"

#include "rtabmap/core/EpipolarGeometry.h"

#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/core/util3d_filtering.h"
#include "rtabmap/core/util3d_surface.h"

#include "rtabmap/core/DBDriver.h"
#include "rtabmap/core/Memory.h"
#include "rtabmap/core/VWDictionary.h"
#include "rtabmap/core/BayesFilter.h"
#include "rtabmap/core/Compression.h"
#include "rtabmap/core/Registration.h"
#include "rtabmap/core/RegistrationInfo.h"

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UProcessInfo.h>

#ifdef RTABMAP_PYTHON
#include "rtabmap/core/PythonInterface.h"
#endif

#ifdef RTABMAP_MRPT
// Used for odometry error propagation
#include <mrpt/poses/CPose3DPDFGaussian.h>
#endif

#include <pcl/search/kdtree.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/TextureMesh.h>

#include <stdlib.h>
#include <set>

#define LOG_F "LogF.txt"
#define LOG_I "LogI.txt"

#define GRAPH_FILE_NAME "Graph.dot"


//
//
//
// =======================================================
// MAIN LOOP, see method "void Rtabmap::process();" below.
// =======================================================
//
//
//

namespace rtabmap
{

Rtabmap::Rtabmap() :
	_publishStats(Parameters::defaultRtabmapPublishStats()),
	_publishLastSignatureData(Parameters::defaultRtabmapPublishLastSignature()),
	_publishPdf(Parameters::defaultRtabmapPublishPdf()),
	_publishLikelihood(Parameters::defaultRtabmapPublishLikelihood()),
	_publishRAMUsage(Parameters::defaultRtabmapPublishRAMUsage()),
	_computeRMSE(Parameters::defaultRtabmapComputeRMSE()),
	_saveWMState(Parameters::defaultRtabmapSaveWMState()),
	_maxTimeAllowed(Parameters::defaultRtabmapTimeThr()), // 700 ms
	_maxMemoryAllowed(Parameters::defaultRtabmapMemoryThr()), // 0=inf
	_loopThr(Parameters::defaultRtabmapLoopThr()),
	_loopRatio(Parameters::defaultRtabmapLoopRatio()),
	_aggressiveLoopThr(Parameters::defaultRGBDAggressiveLoopThr()),
	_virtualPlaceLikelihoodRatio(Parameters::defaultRtabmapVirtualPlaceLikelihoodRatio()),
	_maxLoopClosureDistance(Parameters::defaultRGBDMaxLoopClosureDistance()),
	_verifyLoopClosureHypothesis(Parameters::defaultVhEpEnabled()),
	_maxRetrieved(Parameters::defaultRtabmapMaxRetrieved()),
	_maxLocalRetrieved(Parameters::defaultRGBDMaxLocalRetrieved()),
	_maxRepublished(Parameters::defaultRtabmapMaxRepublished()),
	_rawDataKept(Parameters::defaultMemImageKept()),
	_statisticLogsBufferedInRAM(Parameters::defaultRtabmapStatisticLogsBufferedInRAM()),
	_statisticLogged(Parameters::defaultRtabmapStatisticLogged()),
	_statisticLoggedHeaders(Parameters::defaultRtabmapStatisticLoggedHeaders()),
	_rgbdSlamMode(Parameters::defaultRGBDEnabled()),
	_rgbdLinearUpdate(Parameters::defaultRGBDLinearUpdate()),
	_rgbdAngularUpdate(Parameters::defaultRGBDAngularUpdate()),
	_rgbdLinearSpeedUpdate(Parameters::defaultRGBDLinearSpeedUpdate()),
	_rgbdAngularSpeedUpdate(Parameters::defaultRGBDAngularSpeedUpdate()),
	_newMapOdomChangeDistance(Parameters::defaultRGBDNewMapOdomChangeDistance()),
	_neighborLinkRefining(Parameters::defaultRGBDNeighborLinkRefining()),
	_proximityByTime(Parameters::defaultRGBDProximityByTime()),
	_proximityBySpace(Parameters::defaultRGBDProximityBySpace()),
	_scanMatchingIdsSavedInLinks(Parameters::defaultRGBDScanMatchingIdsSavedInLinks()),
	_loopClosureIdentityGuess(Parameters::defaultRGBDLoopClosureIdentityGuess()),
	_localRadius(Parameters::defaultRGBDLocalRadius()),
	_localImmunizationRatio(Parameters::defaultRGBDLocalImmunizationRatio()),
	_proximityMaxGraphDepth(Parameters::defaultRGBDProximityMaxGraphDepth()),
	_proximityMaxPaths(Parameters::defaultRGBDProximityMaxPaths()),
	_proximityMaxNeighbors(Parameters::defaultRGBDProximityPathMaxNeighbors()),
	_proximityFilteringRadius(Parameters::defaultRGBDProximityPathFilteringRadius()),
	_proximityRawPosesUsed(Parameters::defaultRGBDProximityPathRawPosesUsed()),
	_proximityAngle(Parameters::defaultRGBDProximityAngle()*M_PI/180.0f),
	_proximityOdomGuess(Parameters::defaultRGBDProximityOdomGuess()),
	_proximityMergedScanCovFactor(Parameters::defaultRGBDProximityMergedScanCovFactor()),
	_databasePath(""),
	_optimizeFromGraphEnd(Parameters::defaultRGBDOptimizeFromGraphEnd()),
	_optimizationMaxError(Parameters::defaultRGBDOptimizeMaxError()),
	_startNewMapOnLoopClosure(Parameters::defaultRtabmapStartNewMapOnLoopClosure()),
	_startNewMapOnGoodSignature(Parameters::defaultRtabmapStartNewMapOnGoodSignature()),
	_goalReachedRadius(Parameters::defaultRGBDGoalReachedRadius()),
	_goalsSavedInUserData(Parameters::defaultRGBDGoalsSavedInUserData()),
	_pathStuckIterations(Parameters::defaultRGBDPlanStuckIterations()),
	_pathLinearVelocity(Parameters::defaultRGBDPlanLinearVelocity()),
	_pathAngularVelocity(Parameters::defaultRGBDPlanAngularVelocity()),
	_forceOdom3doF(Parameters::defaultRGBDForceOdom3DoF()),
	_restartAtOrigin(Parameters::defaultRGBDStartAtOrigin()),
	_loopCovLimited(Parameters::defaultRGBDLoopCovLimited()),
	_loopGPS(Parameters::defaultRtabmapLoopGPS()),
	_maxOdomCacheSize(Parameters::defaultRGBDMaxOdomCacheSize()),
	_localizationSmoothing(Parameters::defaultRGBDLocalizationSmoothing()),
	_localizationPriorInf(1.0/(Parameters::defaultRGBDLocalizationPriorError()*Parameters::defaultRGBDLocalizationPriorError())),
	_localizationSecondTryWithoutProximityLinks(Parameters::defaultRGBDLocalizationSecondTryWithoutProximityLinks()),
	_createGlobalScanMap(Parameters::defaultRGBDProximityGlobalScanMap()),
	_markerPriorsLinearVariance(Parameters::defaultMarkerPriorsVarianceLinear()),
	_markerPriorsAngularVariance(Parameters::defaultMarkerPriorsVarianceAngular()),
	_loopClosureHypothesis(0,0.0f),
	_highestHypothesis(0,0.0f),
	_lastProcessTime(0.0),
	_someNodesHaveBeenTransferred(false),
	_distanceTravelled(0.0f),
	_distanceTravelledSinceLastLocalization(0.0f),
	_optimizeFromGraphEndChanged(false),
	_epipolarGeometry(0),
	_bayesFilter(0),
	_graphOptimizer(0),
	_memory(0),
	_foutFloat(0),
	_foutInt(0),
	_wDir(""),
	_mapCorrection(Transform::getIdentity()),
	_lastLocalizationNodeId(0),
	_currentSessionHasGPS(false),
	_pathStatus(0),
	_pathCurrentIndex(0),
	_pathGoalIndex(0),
	_pathTransformToGoal(Transform::getIdentity()),
	_pathStuckCount(0),
	_pathStuckDistance(0.0f)
#ifdef RTABMAP_PYTHON
	,_python(new PythonInterface())
#endif
{
}

Rtabmap::~Rtabmap() {
	UDEBUG("");
	this->close();
}

void Rtabmap::setupLogFiles(bool overwrite)
{
	flushStatisticLogs();
	// Log files
	if(_foutFloat)
	{
		fclose(_foutFloat);
		_foutFloat = 0;
	}
	if(_foutInt)
	{
		fclose(_foutInt);
		_foutInt = 0;
	}

	if(_statisticLogged && !_wDir.empty())
	{
		std::string attributes = "a+"; // append to log files
		if(overwrite)
		{
			// If a file with the same name already exists
			// its content is erased and the file is treated
			// as a new empty file.
			attributes = "w";
		}

		bool addLogFHeader = overwrite || !UFile::exists(_wDir+"/"+LOG_F);
		bool addLogIHeader = overwrite || !UFile::exists(_wDir+"/"+LOG_I);

	#ifdef _MSC_VER
		fopen_s(&_foutFloat, (_wDir+"/"+LOG_F).c_str(), attributes.c_str());
		fopen_s(&_foutInt, (_wDir+"/"+LOG_I).c_str(), attributes.c_str());
	#else
		_foutFloat = fopen((_wDir+"/"+LOG_F).c_str(), attributes.c_str());
		_foutInt = fopen((_wDir+"/"+LOG_I).c_str(), attributes.c_str());
	#endif
		// add header (column identification)
		if(_statisticLoggedHeaders && addLogFHeader && _foutFloat)
		{
			fprintf(_foutFloat, "Column headers:\n");
			fprintf(_foutFloat, " 1-Total iteration time (s)\n");
			fprintf(_foutFloat, " 2-Memory update time (s)\n");
			fprintf(_foutFloat, " 3-Retrieval time (s)\n");
			fprintf(_foutFloat, " 4-Likelihood time (s)\n");
			fprintf(_foutFloat, " 5-Posterior time (s)\n");
			fprintf(_foutFloat, " 6-Hypothesis selection time (s)\n");
			fprintf(_foutFloat, " 7-Hypothesis validation time (s)\n");
			fprintf(_foutFloat, " 8-Transfer time (s)\n");
			fprintf(_foutFloat, " 9-Statistics creation time (s)\n");
			fprintf(_foutFloat, " 10-Loop closure hypothesis value\n");
			fprintf(_foutFloat, " 11-NAN\n");
			fprintf(_foutFloat, " 12-NAN\n");
			fprintf(_foutFloat, " 13-NAN\n");
			fprintf(_foutFloat, " 14-NAN\n");
			fprintf(_foutFloat, " 15-NAN\n");
			fprintf(_foutFloat, " 16-Virtual place hypothesis\n");
			fprintf(_foutFloat, " 17-Join trash time (s)\n");
			fprintf(_foutFloat, " 18-Weight Update (rehearsal) similarity\n");
			fprintf(_foutFloat, " 19-Empty trash time (s)\n");
			fprintf(_foutFloat, " 20-Retrieval database access time (s)\n");
			fprintf(_foutFloat, " 21-Add loop closure link time (s)\n");
			fprintf(_foutFloat, " 22-Memory cleanup time (s)\n");
			fprintf(_foutFloat, " 23-Scan matching (odometry correction) time (s)\n");
			fprintf(_foutFloat, " 24-Local time loop closure detection time (s)\n");
			fprintf(_foutFloat, " 25-Local space loop closure detection time (s)\n");
			fprintf(_foutFloat, " 26-Map optimization (s)\n");
		}
		if(_statisticLoggedHeaders && addLogIHeader && _foutInt)
		{
			fprintf(_foutInt, "Column headers:\n");
			fprintf(_foutInt, " 1-Loop closure ID\n");
			fprintf(_foutInt, " 2-Highest loop closure hypothesis\n");
			fprintf(_foutInt, " 3-Locations transferred\n");
			fprintf(_foutInt, " 4-NAN\n");
			fprintf(_foutInt, " 5-Words extracted from the last image\n");
			fprintf(_foutInt, " 6-Vocabulary size\n");
			fprintf(_foutInt, " 7-Working memory size\n");
			fprintf(_foutInt, " 8-Is loop closure hypothesis rejected?\n");
			fprintf(_foutInt, " 9-NAN\n");
			fprintf(_foutInt, " 10-NAN\n");
			fprintf(_foutInt, " 11-Locations retrieved\n");
			fprintf(_foutInt, " 12-Retrieval location ID\n");
			fprintf(_foutInt, " 13-Unique words extraced from last image\n");
			fprintf(_foutInt, " 14-Retrieval ID\n");
			fprintf(_foutInt, " 15-Non-null likelihood values\n");
			fprintf(_foutInt, " 16-Weight Update ID\n");
			fprintf(_foutInt, " 17-Is last location merged through Weight Update?\n");
			fprintf(_foutInt, " 18-Local graph size\n");
			fprintf(_foutInt, " 19-Sensor data id\n");
			fprintf(_foutInt, " 20-Indexed words\n");
			fprintf(_foutInt, " 21-Index memory usage (KB)\n");
		}

		ULOGGER_DEBUG("Log file (int)=%s", (_wDir+"/"+LOG_I).c_str());
		ULOGGER_DEBUG("Log file (float)=%s", (_wDir+"/"+LOG_F).c_str());
	}
	else
	{
		if(_statisticLogged)
		{
			UWARN("Working directory is not set, log disabled!");
		}
		UDEBUG("Log disabled!");
	}
}

void Rtabmap::flushStatisticLogs()
{
	if(_foutFloat && _bufferedLogsF.size())
	{
		UDEBUG("_bufferedLogsF.size=%d", _bufferedLogsF.size());
		for(std::list<std::string>::iterator iter = _bufferedLogsF.begin(); iter!=_bufferedLogsF.end(); ++iter)
		{
			fprintf(_foutFloat, "%s", iter->c_str());
		}
		_bufferedLogsF.clear();
	}
	if(_foutInt && _bufferedLogsI.size())
	{
		UDEBUG("_bufferedLogsI.size=%d", _bufferedLogsI.size());
		for(std::list<std::string>::iterator iter = _bufferedLogsI.begin(); iter!=_bufferedLogsI.end(); ++iter)
		{
			fprintf(_foutInt, "%s", iter->c_str());
		}
		_bufferedLogsI.clear();
	}
}

/**
 * 这个初始化函数主要做以下几件事：
 * 1 确定数据库路径与是否创建新数据库 
 * 2 根据是否加载数据库参数，合并配置参数 
 * 3 创建 Memory 对象并加载数据库内容（优化后的位姿图等）
 * 4 解析参数，初始化内部变量 
 * 5 从数据库中恢复 SLAM 状态（优化图、约束、全局地图等） 
 * 6 初始化回环检测的贝叶斯预测模型 
 * 7 创建全局激光地图（可选）
 * 8 初始化日志系统
 */
void Rtabmap::init(const ParametersMap & parameters, const std::string & databasePath, bool loadDatabaseParameters)
{
	// 1. 打开/检查数据库路径
	UDEBUG("path=%s", databasePath.c_str());
	// 保存数据库路径
	_databasePath = databasePath;
	if(!_databasePath.empty())
	{
		// 如果路径不为空，则必须是 .db 后缀，否则断言失败：
		UASSERT(UFile::getExtension(_databasePath).compare("db") == 0);
		UINFO("Using database \"%s\".", _databasePath.c_str());
	}
	else
	{
		// 如果未提供数据库路径，则警告：
		// 意味着：如果用 RAM 数据库，除非 close() 时指定新的输出路径，否则所有地图不会保存。
		UWARN("Using empty database. Mapping session will not be saved unless it is closed with an output database path.");
	}

	// 2. 判断是否是新数据库
	bool newDatabase = _databasePath.empty() || !UFile::exists(_databasePath);

	// 3. 处理参数来源（程序参数 + 数据库参数）
	ParametersMap allParameters;
	if(!newDatabase && loadDatabaseParameters)
	{
		// 如果 [不是新数据库] 且 [要求加载数据库参数]：
		DBDriver * driver = DBDriver::create();
		if(driver->openConnection(_databasePath, false))
		{
			// 这些参数是之前 SLAM 运行时保存的。
			allParameters = driver->getLastParameters();
			// ignore working directory (we may be on a different computer)
			// 特别地，删除工作目录参数（因为可能在另一电脑上运行）
			allParameters.erase(Parameters::kRtabmapWorkingDirectory());
		}
		delete driver;
	}

	// 然后把用户传入参数 parameters 合并进来（覆盖数据库参数）
	uInsert(allParameters, parameters);
	ParametersMap::const_iterator iter;
	// 4. 设置当前工作目录
	if((iter=allParameters.find(Parameters::kRtabmapWorkingDirectory())) != allParameters.end())
	{
		this->setWorkingDirectory(iter->second.c_str());
	}

	// If doesn't exist, create a memory
	// 5. 创建 Memory 对象（如果还没创建）
	if(!_memory)
	{
		// Memory 负责：
		// 1 读写数据库（节点、图片、词袋词典、约束等）
		// 2 管理 STM/WM 
		// 3 提供回环候选
		_memory = new Memory(allParameters);
		_memory->init(_databasePath, false, allParameters, true);
	}

	// 6. 清空与初始化内部运行数据
	_optimizedPoses.clear();
	_constraints.clear();
	_globalScanMap.clear();
	_globalScanMapPoses.clear();
	_odomCachePoses.clear();
	_odomCacheConstraints.clear();
	_nodesToRepublish.clear();

	// Parse all parameters
	// 7. 解析参数并设置内部变量
	// 这个函数会读所有 RTAB-Map 参数（如是否使用 ICP、最大距离、loop ratio 等）并设置内部变量。
	this->parseParameters(allParameters);

	// 8. 尝试从数据库中恢复优化后的位姿图
	Transform lastPose;
	// 由 loadOptimizedPoses(&lastPose) 填写，表示“最后的 localization pose”（通常是数据库里记录的最后一个定位位姿
	_optimizedPoses = _memory->loadOptimizedPoses(&lastPose);
	// 如果不是 增量式优化 Mem/IncrementalMemory 配置项
	if(!_memory->isIncremental())
	{
		// 当初始化时没有现成的优化图但 WM 里有多个节点时，尝试基于这些节点计算一次优化图并得到约束，从而恢复出一个可用的地图状态。
		if(_optimizedPoses.empty() &&
			_memory->getWorkingMem().size()>1 &&
			_memory->getWorkingMem().lower_bound(1)!=_memory->getWorkingMem().end())
		{
			cv::Mat cov;
			// 尝试补算优化图：
			// 第一个参数_optimizeFromGraphEnd：bool，决定在补算图时使用工作内存（WM）中的哪个节点作为起点（graph end 或 graph begin）。
			// 第二个参数 false：代表 lookInDatabase=false（在 optimizeCurrentMap 的签名里，这通常表示不去数据库额外查找节点/约束，只使用内存内容或当前提供的数据进行优化）
			// 第三个参数 _optimizedPoses：传出参数，用于接收优化结果（位姿 map）。
			// cov：用于接收优化后的协方差矩阵（局部或全局）。
			// &_constraints：将被填充为优化过程中或之后得到的约束集合（links）。
			this->optimizeCurrentMap(
					!_optimizeFromGraphEnd?_memory->getWorkingMem().lower_bound(1)->first:_memory->getWorkingMem().rbegin()->first,
					false, _optimizedPoses, cov, &_constraints);
		}
		// 如果恢复出了优化图：
		if(!_optimizedPoses.empty())
		{
			// 忽略数据库里记录的 “上次定位位姿”（可能来自不同实验/不同坐标系），而强制认为当前起点为地图原点（即第一个节点）。
			// 常用于希望从头开始但使用已有地图的场景。
			if(_restartAtOrigin)
			{
				UWARN("last localization pose is ignored (%s=true), assuming we start at the origin of the map.", Parameters::kRGBDStartAtOrigin().c_str());
				lastPose = _optimizedPoses.begin()->second;
			}
			// 设置初始 localization pose（仅 localization 模式）
			_lastLocalizationPose = lastPose;

			UINFO("Loaded optimizedPoses=%d firstPose %d=%s lastLocalizationPose=%s",
					_optimizedPoses.size(),
					_optimizedPoses.begin()->first,
					_optimizedPoses.begin()->second.prettyPrint().c_str(),
					_lastLocalizationPose.prettyPrint().c_str());

			// 9. 如果没有约束信息，从 memory 里提取
			if(_constraints.empty())
			{
				std::map<int, Transform> tmp;
				// Get just the links
				// 从 Memory 中重新获取约束
				// uKeysSet 是一个帮助函数，返回 map 的 key 集合, 取出 _optimizedPoses 的节点 ID 集合作为请求对象
				_memory->getMetricConstraints(uKeysSet(_optimizedPoses), tmp, _constraints, false, true);
			}

			// Initialize Bayes' prediction matrix
			// 10. 初始化贝叶斯滤波器（回环检测预测模型）
			UTimer time;
			// 构建一个初始 likelihood(先验、似然) 分布：
			std::map<int, float> likelihood;
			// Memory::kIdVirtual：这是一个虚拟节点 ID，常用于贝叶斯滤波器中表示“未知/新地标”的概率；
			// 将其初始值设为 1，代表默认把概率放在虚拟节点上（保证归一性或作为占位）。
			likelihood.insert(std::make_pair(Memory::kIdVirtual, 1));
			for(std::map<int, Transform>::iterator iter=_optimizedPoses.begin(); iter!=_optimizedPoses.end(); ++iter)
			{
				// 对于_optimizedPoses中的每个节点
				// 如果 Memory 中含有对应的 Signature（签名/描述子/图像等），则向 likelihood 中插入该节点并赋初值 0。
				if(_memory->getSignature(iter->first))
				{
					likelihood.insert(std::make_pair(iter->first, 0));
				}
			}
			// 用于回环检测：
			// 贝叶斯滤波器根据传入的先验 likelihood 以及 Memory 中的统计信息来计算后验概率分布，
			// 这个后验会被用于后续的 loop closure 候选筛选（即预测哪些节点更可能是回环匹配目标）。
			_bayesFilter->computePosterior(_memory, likelihood);
			UINFO("Time initializing Bayes' prediction with %ld nodes: %fs", _optimizedPoses.size(), time.ticks());

			// 11. 创建全局激光地图（如果参数启用）
			// 通常用于：多 session SLAM\位姿图全局一致性检查\导航地图构建
			if(_createGlobalScanMap)
				createGlobalScanMap();
		}
		else
		{
			UINFO("Loaded optimizedPoses=0, last localization pose is ignored!");
		}
	}
	else
	{
		// 若 Memory 为增量式，那么 loadOptimizedPoses(&lastPose) 返回的结果通常已经是最新的优化图（或至少是增量维护的结果），
		// 所以不需要再次调用 optimizeCurrentMap() 补算。
		_lastLocalizationPose = lastPose;
		if(!_optimizedPoses.empty())
		{
			// 如果 _optimizedPoses 非空而 _constraints 尚未填充，仍然调用 getMetricConstraints(...) 来确保约束信息就绪。
			std::map<int, Transform> tmp;
			// Get just the links
			_memory->getMetricConstraints(uKeysSet(_optimizedPoses), tmp, _constraints, false, true);
		}
	}

	if(_databasePath.empty())
	{
		_statisticLogged = false;
	}
	// 12. 初始化日志文件系统
	setupLogFiles(newDatabase);
}

void Rtabmap::init(const std::string & configFile, const std::string & databasePath, bool loadDatabaseParameters)
{
	// fill ctrl struct with values from the configuration file
	ParametersMap param;// = Parameters::defaultParameters;

	if(!configFile.empty())
	{
		ULOGGER_DEBUG("Read parameters from = %s", configFile.c_str());
		Parameters::readINI(configFile, param);
	}

	this->init(param, databasePath, loadDatabaseParameters);
}

void Rtabmap::close(bool databaseSaved, const std::string & ouputDatabasePath)
{
	UINFO("databaseSaved=%d", databaseSaved?1:0);
	_highestHypothesis = std::make_pair(0,0.0f);
	_loopClosureHypothesis = std::make_pair(0,0.0f);
	_lastProcessTime = 0.0;
	_someNodesHaveBeenTransferred = false;
	_constraints.clear();
	_mapCorrection.setIdentity();
	_mapCorrectionBackup.setNull();

	_localizationCovariance = cv::Mat();
	_lastLocalizationNodeId = 0;
	_odomCachePoses.clear();
	_odomCacheConstraints.clear();
	_distanceTravelled = 0.0f;
	_distanceTravelledSinceLastLocalization = 0.0f;
	_optimizeFromGraphEndChanged = false;
	this->clearPath(0);
	_gpsGeocentricCache.clear();
	_currentSessionHasGPS = false;

	_globalScanMap.clear();
	_globalScanMapPoses.clear();

	_nodesToRepublish.clear();

	flushStatisticLogs();
	if(_foutFloat)
	{
		fclose(_foutFloat);
		_foutFloat = 0;
	}
	if(_foutInt)
	{
		fclose(_foutInt);
		_foutInt = 0;
	}

	if(_epipolarGeometry)
	{
		delete _epipolarGeometry;
		_epipolarGeometry = 0;
	}
	if(_memory)
	{
		if(databaseSaved)
		{
			if(_memory->isGraphReduced() && _memory->isIncremental())
			{
				// Force reducing graph, then remove filtered nodes from the optimized poses
				std::map<int, int> reducedIds;
				_memory->incrementMapId(&reducedIds);
				for(std::map<int, int>::iterator iter=reducedIds.begin(); iter!=reducedIds.end(); ++iter)
				{
					_optimizedPoses.erase(iter->first);
				}
			}
			// 保存优化后的位姿图和最新的定位姿态。
			_memory->saveOptimizedPoses(_optimizedPoses, _lastLocalizationPose);
		}
		_memory->close(databaseSaved, true, ouputDatabasePath);
		delete _memory;
		_memory = 0;
	}
	_optimizedPoses.clear();
	_lastLocalizationPose.setNull();

	if(_bayesFilter)
	{
		delete _bayesFilter;
		_bayesFilter = 0;
	}
	if(_graphOptimizer)
	{
		delete _graphOptimizer;
		_graphOptimizer = 0;
	}
	_databasePath.clear();
	parseParameters(Parameters::getDefaultParameters()); // reset to default parameters
	_parameters.clear();
}

void Rtabmap::parseParameters(const ParametersMap & parameters)
{
	uInsert(_parameters, parameters);

	// place this before changing working directory
	Parameters::parse(parameters, Parameters::kRtabmapStatisticLogsBufferedInRAM(), _statisticLogsBufferedInRAM);
	Parameters::parse(parameters, Parameters::kRtabmapStatisticLogged(), _statisticLogged);
	Parameters::parse(parameters, Parameters::kRtabmapStatisticLoggedHeaders(), _statisticLoggedHeaders);

	ULOGGER_DEBUG("");
	ParametersMap::const_iterator iter;
	if((iter=parameters.find(Parameters::kRtabmapWorkingDirectory())) != parameters.end())
	{
		this->setWorkingDirectory(iter->second.c_str());
	}

	Parameters::parse(parameters, Parameters::kRtabmapPublishStats(), _publishStats);
	Parameters::parse(parameters, Parameters::kRtabmapPublishLastSignature(), _publishLastSignatureData);
	Parameters::parse(parameters, Parameters::kRtabmapPublishPdf(), _publishPdf);
	Parameters::parse(parameters, Parameters::kRtabmapPublishLikelihood(), _publishLikelihood);
	Parameters::parse(parameters, Parameters::kRtabmapPublishRAMUsage(), _publishRAMUsage);
	Parameters::parse(parameters, Parameters::kRtabmapComputeRMSE(), _computeRMSE);
	Parameters::parse(parameters, Parameters::kRtabmapSaveWMState(), _saveWMState);
	Parameters::parse(parameters, Parameters::kRtabmapTimeThr(), _maxTimeAllowed);
	Parameters::parse(parameters, Parameters::kRtabmapMemoryThr(), _maxMemoryAllowed);
	Parameters::parse(parameters, Parameters::kRtabmapLoopThr(), _loopThr);
	Parameters::parse(parameters, Parameters::kRtabmapLoopRatio(), _loopRatio);
	Parameters::parse(parameters, Parameters::kRGBDAggressiveLoopThr(), _aggressiveLoopThr);
	Parameters::parse(parameters, Parameters::kRtabmapVirtualPlaceLikelihoodRatio(), _virtualPlaceLikelihoodRatio);

	Parameters::parse(parameters, Parameters::kRGBDMaxLoopClosureDistance(), _maxLoopClosureDistance);
	Parameters::parse(parameters, Parameters::kVhEpEnabled(), _verifyLoopClosureHypothesis);
	Parameters::parse(parameters, Parameters::kRtabmapMaxRetrieved(), _maxRetrieved);
	Parameters::parse(parameters, Parameters::kRGBDMaxLocalRetrieved(), _maxLocalRetrieved);
	Parameters::parse(parameters, Parameters::kRtabmapMaxRepublished(), _maxRepublished);
	if(_maxRepublished == 0 || !_publishLastSignatureData)
	{
		_nodesToRepublish.clear();
	}
	Parameters::parse(parameters, Parameters::kMemImageKept(), _rawDataKept);
	Parameters::parse(parameters, Parameters::kRGBDEnabled(), _rgbdSlamMode);
	Parameters::parse(parameters, Parameters::kRGBDLinearUpdate(), _rgbdLinearUpdate);
	Parameters::parse(parameters, Parameters::kRGBDAngularUpdate(), _rgbdAngularUpdate);
	Parameters::parse(parameters, Parameters::kRGBDLinearSpeedUpdate(), _rgbdLinearSpeedUpdate);
	Parameters::parse(parameters, Parameters::kRGBDAngularSpeedUpdate(), _rgbdAngularSpeedUpdate);
	Parameters::parse(parameters, Parameters::kRGBDNewMapOdomChangeDistance(), _newMapOdomChangeDistance);
	Parameters::parse(parameters, Parameters::kRGBDNeighborLinkRefining(), _neighborLinkRefining);
	Parameters::parse(parameters, Parameters::kRGBDProximityByTime(), _proximityByTime);
	Parameters::parse(parameters, Parameters::kRGBDProximityBySpace(), _proximityBySpace);
	Parameters::parse(parameters, Parameters::kRGBDScanMatchingIdsSavedInLinks(), _scanMatchingIdsSavedInLinks);
	Parameters::parse(parameters, Parameters::kRGBDLoopClosureIdentityGuess(), _loopClosureIdentityGuess);
	Parameters::parse(parameters, Parameters::kRGBDLocalRadius(), _localRadius);
	Parameters::parse(parameters, Parameters::kRGBDLocalImmunizationRatio(), _localImmunizationRatio);
	Parameters::parse(parameters, Parameters::kRGBDProximityMaxGraphDepth(), _proximityMaxGraphDepth);
	Parameters::parse(parameters, Parameters::kRGBDProximityMaxPaths(), _proximityMaxPaths);
	Parameters::parse(parameters, Parameters::kRGBDProximityPathMaxNeighbors(), _proximityMaxNeighbors);
	Parameters::parse(parameters, Parameters::kRGBDProximityPathFilteringRadius(), _proximityFilteringRadius);
	Parameters::parse(parameters, Parameters::kRGBDProximityPathRawPosesUsed(), _proximityRawPosesUsed);
	if(Parameters::parse(parameters, Parameters::kRGBDProximityAngle(), _proximityAngle))
	{
		_proximityAngle *= M_PI/180.0f;
	}
	Parameters::parse(parameters, Parameters::kRGBDProximityOdomGuess(), _proximityOdomGuess);
	Parameters::parse(parameters, Parameters::kRGBDProximityMergedScanCovFactor(), _proximityMergedScanCovFactor);
	UASSERT(_proximityMergedScanCovFactor>0.0);

	bool optimizeFromGraphEndPrevious = _optimizeFromGraphEnd;
	Parameters::parse(parameters, Parameters::kRGBDOptimizeFromGraphEnd(), _optimizeFromGraphEnd);
	if(optimizeFromGraphEndPrevious != _optimizeFromGraphEnd && !_optimizedPoses.empty())
	{
		_optimizeFromGraphEndChanged = true;
	}
	Parameters::parse(parameters, Parameters::kRGBDOptimizeMaxError(), _optimizationMaxError);
	Parameters::parse(parameters, Parameters::kRtabmapStartNewMapOnLoopClosure(), _startNewMapOnLoopClosure);
	Parameters::parse(parameters, Parameters::kRtabmapStartNewMapOnGoodSignature(), _startNewMapOnGoodSignature);
	Parameters::parse(parameters, Parameters::kRGBDGoalReachedRadius(), _goalReachedRadius);
	Parameters::parse(parameters, Parameters::kRGBDGoalsSavedInUserData(), _goalsSavedInUserData);
	Parameters::parse(parameters, Parameters::kRGBDPlanStuckIterations(), _pathStuckIterations);
	Parameters::parse(parameters, Parameters::kRGBDPlanLinearVelocity(), _pathLinearVelocity);
	Parameters::parse(parameters, Parameters::kRGBDPlanAngularVelocity(), _pathAngularVelocity);
	Parameters::parse(parameters, Parameters::kRGBDForceOdom3DoF(), _forceOdom3doF);
	Parameters::parse(parameters, Parameters::kRGBDStartAtOrigin(), _restartAtOrigin);
	Parameters::parse(parameters, Parameters::kRGBDLoopCovLimited(), _loopCovLimited);
	Parameters::parse(parameters, Parameters::kRtabmapLoopGPS(), _loopGPS);
	Parameters::parse(parameters, Parameters::kRGBDMaxOdomCacheSize(), _maxOdomCacheSize);
	Parameters::parse(parameters, Parameters::kRGBDLocalizationSmoothing(), _localizationSmoothing);
	double localizationPriorError = Parameters::defaultRGBDLocalizationPriorError();
	Parameters::parse(parameters, Parameters::kRGBDLocalizationPriorError(), localizationPriorError);
	UASSERT(localizationPriorError>0.0);
	_localizationPriorInf = 1.0/(localizationPriorError*localizationPriorError);
	Parameters::parse(parameters, Parameters::kRGBDLocalizationSecondTryWithoutProximityLinks(), _localizationSecondTryWithoutProximityLinks);
	Parameters::parse(parameters, Parameters::kRGBDProximityGlobalScanMap(), _createGlobalScanMap);

	Parameters::parse(parameters, Parameters::kMarkerPriorsVarianceLinear(), _markerPriorsLinearVariance);
	UASSERT(_markerPriorsLinearVariance>0.0f);
	Parameters::parse(parameters, Parameters::kMarkerPriorsVarianceAngular(), _markerPriorsAngularVariance);
	UASSERT(_markerPriorsAngularVariance>0.0f);
	std::string markerPriorsStr;
	if(Parameters::parse(parameters, Parameters::kMarkerPriors(), markerPriorsStr))
	{
		_markerPriors.clear();
		std::list<std::string> strList = uSplit(markerPriorsStr, '|');
		for(std::list<std::string>::iterator iter=strList.begin(); iter!=strList.end(); ++iter)
		{
			std::string markerStr = *iter;
			while(!markerStr.empty() && !uIsDigit(markerStr[0]))
			{
				markerStr.erase(markerStr.begin());
			}
			if(!markerStr.empty())
			{
				std::string idStr = uSplitNumChar(markerStr).front();
				int id = uStr2Int(idStr);
				Transform prior = Transform::fromString(markerStr.substr(idStr.size()));
				if(!prior.isNull() && id>0)
				{
					_markerPriors.insert(std::make_pair(-id, prior));
					UDEBUG("Added landmark prior %d: %s", id, prior.prettyPrint().c_str());
				}
				else
				{
					UERROR("Failed to parse element \"%s\" in parameter %s", markerStr.c_str(), Parameters::kMarkerPriors().c_str());
				}
			}
			else if(!iter->empty())
			{
				UERROR("Failed to parse parameter %s, value=\"%s\"", Parameters::kMarkerPriors().c_str(), iter->c_str());
			}
		}
	}

	UASSERT(_rgbdLinearUpdate >= 0.0f);
	UASSERT(_rgbdAngularUpdate >= 0.0f);
	UASSERT(_rgbdLinearSpeedUpdate >= 0.0f);
	UASSERT(_rgbdAngularSpeedUpdate >= 0.0f);
	UASSERT(_maxOdomCacheSize >= 0);

	// By default, we create our strategies if they are not already created.
	// If they already exists, we check the parameters if a change is requested

	// Graph optimizer
	Optimizer::Type optimizerType = Optimizer::kTypeUndef;
	if((iter=parameters.find(Parameters::kOptimizerStrategy())) != parameters.end())
	{
		optimizerType = (Optimizer::Type)std::atoi((*iter).second.c_str());
	}
	if(optimizerType!=Optimizer::kTypeUndef)
	{
		UDEBUG("new detector strategy %d", int(optimizerType));
		if(_graphOptimizer)
		{
			delete _graphOptimizer;
			_graphOptimizer = 0;
		}

		_graphOptimizer = Optimizer::create(optimizerType, _parameters);
	}
	else if(_graphOptimizer)
	{
		_graphOptimizer->parseParameters(parameters);
	}
	else
	{
		optimizerType = (Optimizer::Type)Parameters::defaultOptimizerStrategy();
		_graphOptimizer = Optimizer::create(optimizerType, parameters);
	}

	if(!_createGlobalScanMap)
	{
		_globalScanMap.clear();
		_globalScanMapPoses.clear();
	}

	if(_memory)
	{
		bool isMemIncremental = _memory->isIncremental();
		if(Parameters::parse(parameters, Parameters::kMemIncrementalMemory(), isMemIncremental) &&
			isMemIncremental != _memory->isIncremental())
		{
			// Mode has changed from Mapping to Localization, cleanup the local graph
			if(_memory->isGraphReduced() && _memory->isIncremental())
			{
				// Force reducing graph, then remove filtered nodes from the optimized poses
				std::map<int, int> reducedIds;
				_memory->incrementMapId(&reducedIds);
				for(std::map<int, int>::iterator iter=reducedIds.begin(); iter!=reducedIds.end(); ++iter)
				{
					_optimizedPoses.erase(iter->first);
				}
			}

			// In both cases, we save the latest optimized graph and latest localization pose
			// 保存优化后的位姿图和最新的定位姿态。
			_memory->saveOptimizedPoses(_optimizedPoses, _lastLocalizationPose);

			// Mode changed from Localization to Mapping, clear local graph
			if(!_memory->isIncremental()) {
				_optimizedPoses.clear();
				_lastLocalizationPose.setNull();
				_mapCorrection.setIdentity();
				_mapCorrectionBackup.setNull();
				_localizationCovariance = cv::Mat();
				_lastLocalizationNodeId = 0;
			}
		}

		_memory->parseParameters(parameters);
		if(_memory->isIncremental() && !_globalScanMap.empty())
		{
			UWARN("Map is now incremental, clearing global scan map...");
			_globalScanMap.clear();
			_globalScanMapPoses.clear();
		}

		if(_createGlobalScanMap && !_memory->isIncremental() && _globalScanMap.empty() && !_optimizedPoses.empty())
		{
			this->createGlobalScanMap();
		}

		if(_memory->isIncremental())
		{
			_odomCachePoses.clear();
			_odomCacheConstraints.clear();
		}
	}

	if(!_epipolarGeometry)
	{
		_epipolarGeometry = new EpipolarGeometry(_parameters);
	}
	else
	{
		_epipolarGeometry->parseParameters(parameters);
	}

	// Bayes filter, create one if not exists
	if(!_bayesFilter)
	{
		_bayesFilter = new BayesFilter(_parameters);
	}
	else
	{
		_bayesFilter->parseParameters(parameters);
	}
}

int Rtabmap::getLastLocationId() const
{
	int id = 0;
	if(_memory)
	{
		id = _memory->getLastSignatureId();
	}
	return id;
}

std::list<int> Rtabmap::getWM() const
{
	std::list<int> mem;
	if(_memory)
	{
		mem = uKeysList(_memory->getWorkingMem());
		mem.remove(-1);// Ignore the virtual signature (if here)
	}
	return mem;
}

int Rtabmap::getWMSize() const
{
	if(_memory)
	{
		return (int)_memory->getWorkingMem().size()-1; // remove virtual place
	}
	return 0;
}

std::map<int, int> Rtabmap::getWeights() const
{
	std::map<int, int> weights;
	if(_memory)
	{
		weights = _memory->getWeights();
		weights.erase(-1);// Ignore the virtual signature (if here)
	}
	return weights;
}

std::set<int> Rtabmap::getSTM() const
{
	if(_memory)
	{
		return _memory->getStMem();
	}
	return std::set<int>();
}

int Rtabmap::getSTMSize() const
{
	if(_memory)
	{
		return (int)_memory->getStMem().size();
	}
	return 0;
}

int Rtabmap::getTotalMemSize() const
{
	if(_memory)
	{
		const Signature * s  =_memory->getLastWorkingSignature();
		if(s)
		{
			return s->id();
		}
	}
	return 0;
}

bool Rtabmap::isInSTM(int locationId) const
{
	if(_memory)
	{
		return _memory->isInSTM(locationId);
	}
	return false;
}

bool Rtabmap::isIDsGenerated() const
{
	if(_memory)
	{
		return _memory->isIDsGenerated();
	}
	return Parameters::defaultMemGenerateIds();
}

const Statistics & Rtabmap::getStatistics() const
{
	return statistics_;
}

Transform Rtabmap::getPose(int locationId) const
{
	return uValue(_optimizedPoses, locationId, Transform());
}

void Rtabmap::setInitialPose(const Transform & initialPose)
{
	if(_memory)
	{
		if(!_memory->isIncremental())
		{
			_lastLocalizationPose = initialPose;
			_localizationCovariance = cv::Mat();
			_lastLocalizationNodeId = 0;
			_odomCachePoses.clear();
			_odomCacheConstraints.clear();
			_mapCorrection.setIdentity();
			_mapCorrectionBackup.setNull();

			if(_memory->getLastWorkingSignature()->id() &&
				_optimizedPoses.empty())
			{
				cv::Mat covariance;
				this->optimizeCurrentMap(_memory->getLastWorkingSignature()->id(), false, _optimizedPoses, covariance, &_constraints);
			}
		}
		else
		{
			UWARN("Initial pose can only be set in localization mode (%s=false), ignoring it...", Parameters::kMemIncrementalMemory().c_str());
		}
	}
}

int Rtabmap::triggerNewMap()
{
	int mapId = -1;
	if(_memory)
	{
		_localizationCovariance = cv::Mat();
		_lastLocalizationNodeId = 0;
		_odomCachePoses.clear();
		_odomCacheConstraints.clear();
		_distanceTravelled = 0.0f;
		_distanceTravelledSinceLastLocalization = 0.0f;

		if(!_memory->isIncremental())
		{
			_mapCorrection.setIdentity();
			if(_restartAtOrigin)
			{
				_lastLocalizationPose.setIdentity();
			}
			return mapId;
		}
		std::map<int, int> reducedIds;
		mapId = _memory->incrementMapId(&reducedIds);
		UINFO("New map triggered, new map = %d", mapId);
		_optimizedPoses.clear();
		_constraints.clear();

		if(_bayesFilter)
		{
			_bayesFilter->reset();
		}

		//Verify if there are nodes that were merged through graph reduction
		if(reducedIds.size() && _path.size())
		{
			for(unsigned int i=0; i<_path.size(); ++i)
			{
				std::map<int, int>::const_iterator iter = reducedIds.find(_path[i].first);
				if(iter!= reducedIds.end())
				{
					// change path ID to loop closure ID
					_path[i].first = iter->second;
				}
			}
		}
	}
	return mapId;
}

bool Rtabmap::labelLocation(int id, const std::string & label)
{
	if(_memory)
	{
		if(id > 0)
		{
			return _memory->labelSignature(id, label);
		}
		else if(_memory->isIncremental() && _memory->getLastWorkingSignature())
		{
			return _memory->labelSignature(_memory->getLastWorkingSignature()->id(), label);
		}
		else if(!_memory->isIncremental() && !_lastLocalizationPose.isNull() && !_lastLocalizationPose.isIdentity())
		{
			std::map<int, Transform> nearestNodes = getNodesInRadius(_lastLocalizationPose, _localRadius, 1);
			if(!nearestNodes.empty())
			{
				return _memory->labelSignature(nearestNodes.begin()->first, label);
			}
			else
			{
				UERROR("No nodes found inside %s=%fm of the current pose (%s). Cannot set label \"%s\"",
						Parameters::kRGBDLocalRadius().c_str(),
						_localRadius,
						_lastLocalizationPose.prettyPrint().c_str(),
						label.c_str());
			}
		}
		else
		{
			UERROR("Last signature is null! Cannot set label \"%s\"", label.c_str());
		}
	}
	return false;
}

bool Rtabmap::setUserData(int id, const cv::Mat & data)
{
	if(_memory)
	{
		if(id > 0)
		{
			return _memory->setUserData(id, data);
		}
		else if(_memory->getLastWorkingSignature())
		{
			return _memory->setUserData(_memory->getLastWorkingSignature()->id(), data);
		}
		else
		{
			UERROR("Last signature is null! Cannot set user data!");
		}
	}
	return false;
}

void Rtabmap::generateDOTGraph(const std::string & path, int id, int margin)
{
	if(_memory)
	{
		_memory->joinTrashThread(); // make sure the trash is flushed

		if(id > 0)
		{
			std::map<int, int> ids = _memory->getNeighborsId(id, margin, -1, false);

			if(ids.size() > 0)
			{
				ids.insert(std::pair<int,int>(id, 0));
				std::set<int> idsSet;
				for(std::map<int, int>::iterator iter = ids.begin(); iter!=ids.end(); ++iter)
				{
					idsSet.insert(idsSet.end(), iter->first);
				}
				_memory->generateGraph(path, idsSet);
			}
			else
			{
				UERROR("No neighbors found for signature %d.", id);
			}
		}
		else
		{
			_memory->generateGraph(path);
		}
	}
}

void Rtabmap::exportPoses(const std::string & path, bool optimized, bool global, int format)
{
	if(_memory && _memory->getLastWorkingSignature())
	{
		std::map<int, Transform> poses;
		std::multimap<int, Link> constraints;

		if(optimized)
		{
			cv::Mat covariance;
			this->optimizeCurrentMap(_memory->getLastWorkingSignature()->id(), global, poses, covariance, &constraints);
		}
		else
		{
			std::map<int, int> ids = _memory->getNeighborsId(_memory->getLastWorkingSignature()->id(), 0, global?-1:0, true);
			_memory->getMetricConstraints(uKeysSet(ids), poses, constraints, global);
		}

		std::map<int, double> stamps;
		if(format == 1 || format == 10 || format == 11)
		{
			for(std::map<int, Transform>::iterator iter=poses.begin(); iter!=poses.end(); ++iter)
			{
				Transform o,g;
				int m, w;
				std::string l;
				double stamp = 0.0;
				std::vector<float> v;
				GPS gps;
				EnvSensors sensors;
				_memory->getNodeInfo(iter->first, o, m, w, l, stamp, g, v, gps, sensors, true);
				stamps.insert(std::make_pair(iter->first, stamp));
			}
		}

		graph::exportPoses(path, format, poses, constraints, stamps, _parameters);
	}
}

void Rtabmap::resetMemory()
{
	UDEBUG("");
	_highestHypothesis = std::make_pair(0,0.0f);
	_loopClosureHypothesis = std::make_pair(0,0.0f);
	_lastProcessTime = 0.0;
	_someNodesHaveBeenTransferred = false;
	_optimizedPoses.clear();
	_constraints.clear();
	_mapCorrection.setIdentity();
	_mapCorrectionBackup.setNull();
	_lastLocalizationPose.setNull();
	_localizationCovariance = cv::Mat();
	_lastLocalizationNodeId = 0;
	_odomCachePoses.clear();
	_odomCacheConstraints.clear();
	_distanceTravelled = 0.0f;
	_distanceTravelledSinceLastLocalization = 0.0f;
	_optimizeFromGraphEndChanged = false;
	_globalScanMap.clear();
	_globalScanMapPoses.clear();
	_nodesToRepublish.clear();
	this->clearPath(0);

	if(_memory)
	{
		_memory->init(_databasePath, true, _parameters, true);
		if(_memory->getLastWorkingSignature())
		{
			cv::Mat covariance;
			optimizeCurrentMap(_memory->getLastWorkingSignature()->id(), false, _optimizedPoses, covariance, &_constraints);
		}
		if(_bayesFilter)
		{
			_bayesFilter->reset();
		}
	}
	else
	{
		UERROR("RTAB-Map is not initialized. No memory to reset...");
	}

	if(_graphOptimizer)
	{
		delete _graphOptimizer;
		_graphOptimizer = Optimizer::create(_parameters);
	}

	this->setupLogFiles(true);
}

class NearestPathKey
{
public:
	NearestPathKey(float l, int i, float d) :
		likelihood(l),
		id(i),
		distance(d){}
	bool operator<(const NearestPathKey & k) const
	{
		if(likelihood < k.likelihood)
		{
			return true;
		}
		else if(likelihood == k.likelihood)
		{
			if(distance > k.distance)
			{
				return true;
			}
			else if(distance == k.distance && id < k.id)
			{
				return true;
			}
		}
		return false;
	}
	float likelihood;
	int id;
	float distance;
};

//============================================================
// MAIN LOOP
//============================================================
bool Rtabmap::process(
		const cv::Mat & image,
		int id,
		const std::map<std::string, float> & externalStats)
{
	// 调用主循环函数
	return this->process(SensorData(image, id), Transform());
}
bool Rtabmap::process(
			const SensorData & data,
			Transform odomPose,
			float odomLinearVariance,
			float odomAngularVariance,
			const std::vector<float> & odomVelocity,
			const std::map<std::string, float> & externalStats)
{
	if(!odomPose.isNull())
	{
		UASSERT(odomLinearVariance>0.0f);
		UASSERT(odomAngularVariance>0.0f);
	}
	// 构造协方差矩阵
	cv::Mat covariance = cv::Mat::eye(6,6,CV_64FC1);
	covariance.at<double>(0,0) = odomLinearVariance;
	covariance.at<double>(1,1) = odomLinearVariance;
	covariance.at<double>(2,2) = odomLinearVariance;
	covariance.at<double>(3,3) = odomAngularVariance;
	covariance.at<double>(4,4) = odomAngularVariance;
	covariance.at<double>(5,5) = odomAngularVariance;
	return process(data, odomPose, covariance, odomVelocity, externalStats);
}
bool Rtabmap::process(
		const SensorData & data,
		Transform odomPose,
		const cv::Mat & odomCovariance,
		const std::vector<float> & odomVelocity,
		const std::map<std::string, float> & externalStats)
{
	UDEBUG("");

	//============================================================
	// Initialization 初始化
	//============================================================
	UTimer timer;  // 用于测量某一局部步骤的耗时（短时间段）
	UTimer timerTotal;  // 测量整个 process() 的总耗时
	double timeMemoryUpdate = 0;  // 更新记忆/图
	double timeNeighborLinkRefining = 0;  // 邻接边修正
	double timeProximityByTimeDetection = 0;  // 时间邻近检测
	double timeProximityBySpaceSearch = 0;  // 空间邻近搜索（最近邻）
	double timeProximityBySpaceVisualDetection = 0;  // 视觉靠近检测（回环）
	double timeProximityBySpaceDetection = 0;
	double timeCleaningNeighbors = 0;  // 删除邻接边
	double timeReactivations = 0;
	double timeAddLoopClosureLink = 0;  // 建立回环闭环约束
	double timeMapOptimization = 0;  // 图优化
	double timeRetrievalDbAccess = 0;  // 从数据库检索候选节点
	double timeLikelihoodCalculation = 0;  // 计算似然度
	double timePosteriorCalculation = 0;  // 计算后验概率
	double timeHypothesesCreation = 0;  // 产生回环假设
	double timeHypothesesValidation = 0;  // 验证回环假设
	double timeRealTimeLimitReachedProcess = 0;  // 实时限制检查
	double timeMemoryCleanup = 0;  // 清理旧记忆
	double timeEmptyingTrash = 0;  // 垃圾清除（删除节点/链接）
	double timeJoiningTrash = 0;
	double timeFinalizingStatistics = 0;  // 生成统计数据
	double timeStatsCreation = 0;

	float hypothesisRatio = 0.0f; // Only used for statistics 候选回环的得分比率（用于判断是否有效）
	bool rejectedLoopClosure = false;  // 是否拒绝本帧的回环检测结果

	std::map<int, float> rawLikelihood;  // 原始似然 (从视觉词包匹配计算出)
	std::map<int, float> adjustedLikelihood;  // 调整后的似然（根据策略校正）
	std::map<int, float> likelihood;  // 最终用于 Bayes 计算的似然
	std::map<int, int> weights;  // 匹配权重（反映匹配强度）
	std::map<int, float> posterior;  // 后验概率：用于判断最可能的回环
	std::list<std::pair<int, float> > reactivateHypotheses;  // RTAB-Map 会在适当时重新激活老节点（长期记忆 → 短期记忆）的候选列表。

	std::map<int, int> childCount;  // 统计子节点数
	std::set<int> signaturesRetrieved;  // 存储从数据库读取的 signature（节点）。
	int proximityDetectionsInTimeFound = 0;  // 统计基于时间（而不是距离）检测到的“邻近候选”（如连贯帧间相似度）。

	const Signature * signature = 0;  // 当前帧的 Signature
	const Signature * sLoop = 0;  // 回环对应的 Signature（若检测到回环）

	_loopClosureHypothesis = std::make_pair(0,0.0f);
	std::pair<int, float> lastHighestHypothesis = _highestHypothesis;  // 上一帧的最高候选，用于比较稳定性
	_highestHypothesis = std::make_pair(0,0.0f);  // 本次处理中的最高回环候选 (id, score)

	std::set<int> immunizedLocations;  // 在局部图优化或临近节点中，有些节点会“免疫”（不参与某些操作）以避免重复检测。

	statistics_ = Statistics(); // reset 清空统计对象
	// 将外部提供的统计信息 externalStats 写入其中
	for(std::map<std::string, float>::const_iterator iter=externalStats.begin(); iter!=externalStats.end(); ++iter)
	{
		statistics_.addStatistic(iter->first, iter->second);
	}

	//============================================================
	// Wait for an image...
	//============================================================
	ULOGGER_INFO("getting data...");

	// 开始计时
	timer.start();
	timerTotal.start();

	UASSERT_MSG(_memory, "RTAB-Map is not initialized!");
	UASSERT_MSG(_bayesFilter, "RTAB-Map is not initialized!");
	UASSERT_MSG(_graphOptimizer, "RTAB-Map is not initialized!");

	//============================================================
	// If RGBD SLAM is enabled, a pose must be set.
	// RGBD模式，系统要求odometry输入
	//============================================================
	// rtabmap_odom生成odom数据。建图模式下必须有Odom数据；定位模式下可以没有odom数据（允许只获取map定位结果）
	// 如果当前没有 odometry，系统会生成一个假 odomPose，并用该标记记录。
	bool fakeOdom = false;
	if(_rgbdSlamMode)
	{
		// 如果有 odometry，则检查并修正
		if(!odomPose.isNull())
		{
			// If we are doing 2D mapping, make sure the pose is 3DoF so that landmark logic works.
			// 若强制 3DoF odom 且 Optimizer 是 2D SLAM 。如果odom不是三维，需要降维至 3DoF
			if(_forceOdom3doF && _graphOptimizer->isSlam2d() && !odomPose.is3DoF())
			{
				odomPose = odomPose.to3DoF();
			}

			// this will make sure that all inverse operations will work!
			// 检查 odometry 是否可逆（矩阵是否奇异）
			// RTAB-Map 进行大量 pose.inverse() 操作; 奇异矩阵将导致 SLAM 崩溃
			if(!odomPose.isInvertible())
			{
				UWARN("Input odometry is not invertible! pose = %s\n"
						"[%f %f %f %f;\n"
						" %f %f %f %f;\n"
						" %f %f %f %f;\n"
						" 0 0 0 1]\n"
						"Trying to normalize rotation to see if it makes it invertible...",
						odomPose.prettyPrint().c_str(),
						odomPose.r11(), odomPose.r12(), odomPose.r13(), odomPose.o14(),
						odomPose.r21(), odomPose.r22(), odomPose.r23(), odomPose.o24(),
						odomPose.r31(), odomPose.r32(), odomPose.r33(), odomPose.o34());
				// 尝试进行旋转矩阵正交化（normalizeRotation）
				odomPose.normalizeRotation();
				// 若仍然不可逆，报 fatal 错误
				UASSERT_MSG(odomPose.isInvertible(), uFormat("Odometry pose is not invertible!\n"
						"[%f %f %f %f;\n"
						" %f %f %f %f;\n"
						" %f %f %f %f;\n"
						" 0 0 0 1]", odomPose.prettyPrint().c_str(),
						odomPose.r11(), odomPose.r12(), odomPose.r13(), odomPose.o14(),
						odomPose.r21(), odomPose.r22(), odomPose.r23(), odomPose.o24(),
						odomPose.r31(), odomPose.r32(), odomPose.r33(), odomPose.o34()).c_str());
				UWARN("Normalizing rotation succeeded! fixed pose = %s\n"
						"[%f %f %f %f;\n"
						" %f %f %f %f;\n"
						" %f %f %f %f;\n"
						" 0 0 0 1]\n"
						"If the resulting rotation is very different from original one, try to fix the odometry or TF.",
						odomPose.prettyPrint().c_str(),
						odomPose.r11(), odomPose.r12(), odomPose.r13(), odomPose.o14(),
						odomPose.r21(), odomPose.r22(), odomPose.r23(), odomPose.o24(),
						odomPose.r31(), odomPose.r32(), odomPose.r33(), odomPose.o34());
			}
		}

		UDEBUG("incremental=%d odomPose=%s optimizedPoses=%d mapCorrection=%s lastLocalizationPose=%s lastLocalizationNodeId=%d",
				_memory->isIncremental()?1:0,
				odomPose.prettyPrint().c_str(),
				(int)_optimizedPoses.size(),
				_mapCorrection.prettyPrint().c_str(),
				_lastLocalizationPose.prettyPrint().c_str(),
				_lastLocalizationNodeId);

		if(!_memory->isIncremental() &&
			!odomPose.isNull() &&
			_optimizedPoses.size() &&
			_mapCorrection.isIdentity() &&
			!_lastLocalizationPose.isNull() &&
			_lastLocalizationNodeId == 0)
		{
			// Localization mode
			// 进入Localization模式（不是 SLAM 建图，而是使用已有地图进行定位）。
			// 此时系统必须建立：map->odom 之间的转换，使实时 odom 坐标与已优化地图坐标对齐。
			if(!_optimizeFromGraphEnd)  // odom和map的对齐策略
			{
				// _optimizeFromGraphEnd=false 不从图末端优化（默认）
				// set map->odom so that odom is moved back to last saved localization
				// 计算并设置 map->odom，以便将 odom 与新的定位结果对齐。
				// 使用矩阵计算 _mapCorrection 结构为[R|T] R旋转 T平移
				if(_graphOptimizer->isSlam2d())
				{
					_mapCorrection = _lastLocalizationPose.to3DoF() * odomPose.to3DoF().inverse();
				}
				else if((!data.imu().empty() || _memory->isOdomGravityUsed()) && _graphOptimizer->gravitySigma()>0.0f)
				{
					_mapCorrection = _lastLocalizationPose.to4DoF() * odomPose.to4DoF().inverse();
				}
				else
				{
					_mapCorrection = _lastLocalizationPose * odomPose.inverse();
				}
				// 不从图末端优化，因此只需记录map->odom而不需要对优化后的位姿进行处理
				std::map<int, Transform> nodesOnly(_optimizedPoses.lower_bound(1), _optimizedPoses.end());
				_lastLocalizationNodeId = graph::findNearestNode(nodesOnly, _lastLocalizationPose);
				UWARN("Update map correction based on last localization saved in database! correction = %s, nearest id = %d of last pose = %s, odom = %s",
						_mapCorrection.prettyPrint().c_str(),
						_lastLocalizationNodeId,
						_lastLocalizationPose.prettyPrint().c_str(),
						odomPose.prettyPrint().c_str());
			}
			else
			{
				// 从图末端优化
				// move optimized poses accordingly to last saved localization
				// 将整个 optimized map 根据新的本地化位置做平移
				// odom = mapCorrectionInv * lastLocalizationPose 将新的map位姿转换到odom下，而不是调整 map→odom
				Transform mapCorrectionInv;
				if(_graphOptimizer->isSlam2d())
				{
					mapCorrectionInv = odomPose.to3DoF() * _lastLocalizationPose.to3DoF().inverse();
				}
				else if((!data.imu().empty() || _memory->isOdomGravityUsed()) && _graphOptimizer->gravitySigma()>0.0f)
				{
					mapCorrectionInv = odomPose.to4DoF() * _lastLocalizationPose.to4DoF().inverse();
				}
				else
				{
					mapCorrectionInv = odomPose * _lastLocalizationPose.inverse();
				}
				// 从图末端优化，因此需要将优化后的map位姿转换到最新的odom坐标系下
				for(std::map<int, Transform>::iterator iter=_optimizedPoses.begin(); iter!=_optimizedPoses.end(); ++iter)
				{
					iter->second = mapCorrectionInv * iter->second;
				}

				std::map<int, Transform> nodesOnly(_optimizedPoses.lower_bound(1), _optimizedPoses.end());
				_lastLocalizationNodeId = graph::findNearestNode(nodesOnly, _lastLocalizationPose);
				UWARN("Transformed map accordingly to last localization pose saved in database (%s=true)! nearest id = %d of last pose = %s",
						Parameters::kRGBDOptimizeFromGraphEnd().c_str(),
						_lastLocalizationNodeId,
						_lastLocalizationPose.prettyPrint().c_str());
			}
		}

		// 若没有odometry
		if(odomPose.isNull())
		{
			if(_memory->isIncremental())
			{
				// 建图模式 (incremental = true) 没有 odom → 无法继续 → 丢弃帧
				UERROR("RGB-D SLAM mode is enabled, memory is incremental but no odometry is provided. "
					   "Image %d is ignored!", data.id());
				return false;
			}
			else // fake localization
			{
				// 定位模式 (incremental = false) 可用 mapCorrection 和 lastLocalizationPose 生成假 odom
				if(!_mapCorrectionBackup.isNull())
				{
					_mapCorrection = _mapCorrectionBackup;
					_mapCorrectionBackup.setNull();
				}
				if(_lastLocalizationPose.isNull())
				{
					_lastLocalizationPose = Transform::getIdentity();
				}
				fakeOdom = true;
				odomPose = _mapCorrection.inverse() * _lastLocalizationPose;
				UDEBUG("Map correction = %s", _mapCorrection.prettyPrint().c_str());
				UDEBUG("Last localization pose: %s", _lastLocalizationPose.prettyPrint().c_str());
				UDEBUG("Fake odom: %s", odomPose.prettyPrint().c_str());
			}
		}
		else if(_memory->isIncremental()) // only in mapping mode
		{
			// 建图模式下：检测 odometry 是否重置
			// Detect if the odometry is reset. If yes, trigger a new map.
			if(_memory->getLastWorkingSignature())
			{
				const Transform & lastPose = _memory->getLastWorkingSignature()->getPose(); // use raw odometry

				// look for identity
				// odom pose 突然变回 identity（0,0,0）常见于 VIO/VO 失败后的重置。
				if(!lastPose.isIdentity() && odomPose.isIdentity())
				{
					int mapId = triggerNewMap();
					UWARN("Odometry is reset (identity pose detected). Increment map id to %d!", mapId);
				}
				else if(_newMapOdomChangeDistance > 0.0)
				{
					// look for large change
					Transform lastPoseToNewPose = lastPose.inverse() * odomPose;
					float x,y,z, roll,pitch,yaw;
					lastPoseToNewPose.getTranslationAndEulerAngles(x,y,z, roll,pitch,yaw);
					// odometry 跳变过大判断（超过 _newMapOdomChangeDistance）
					if((x*x + y*y + z*z) > _newMapOdomChangeDistance*_newMapOdomChangeDistance)
					{
						// 如果跳变过大，新建地图，用于避免因跳变导致图拓扑断裂。
						int mapId = triggerNewMap();
						UWARN("Odometry is reset (large odometry change detected > %f). A new map (%d) is created! Last pose = %s, new pose = %s",
								_newMapOdomChangeDistance,
								mapId,
								lastPose.prettyPrint().c_str(),
								odomPose.prettyPrint().c_str());
					}
				}
			}
		}
	}

	//============================================================
	// Memory Update : Location creation + Add to STM + Weight Update (Rehearsal)
	// 更新memory: 定位创建 + 加入短期记忆 + 权重更新
	//============================================================
	ULOGGER_INFO("Updating memory...");
	if(_rgbdSlamMode)
	{
		// 用 odometry 更新 Memory
		// 传感器数据 → Memory.update() → Signature → 回环检测 → 图优化 → 地图输出
		// update() 决定了：1当前节点是否要加入图（建立邻接边） 2当前节点是否要与旧节点匹配（回环检测入口）
		// 3 odom pose 是否正确 4 特征提取是否成功 5 数据是否要压缩/保存 6 是否要产生新的回环候选
		// 没有 Memory update，后续所有 SLAM 步骤没有输入。
		if(!_memory->update(data, odomPose, odomCovariance, odomVelocity, &statistics_))
		{
			return false;
		}
	}
	else
	{
		// 在非 RGBD-SLAM 模式下：使用空的 odometry 表示无 odom
		if(!_memory->update(data, Transform(), cv::Mat(), std::vector<float>(), &statistics_))
		{
			return false;
		}
	}

	// 获取最新的“工作签名”
	signature = _memory->getLastWorkingSignature();
	// 检测是否存在 GPS 信息
	_currentSessionHasGPS = _currentSessionHasGPS || signature->sensorData().gps().stamp() > 0.0;
	if(!signature)
	{
		UFATAL("Not supposed to be here...last signature is null?!?");
	}

	ULOGGER_INFO("Processing signature %d w=%d map=%d", signature->id(), signature->getWeight(), signature->mapId());
	// 记录 Memory update 的计算时间
	timeMemoryUpdate = timer.ticks();
	ULOGGER_INFO("timeMemoryUpdate=%fs", timeMemoryUpdate);

	//============================================================
	// Metric
	// RGBD模式下：运动检测、小位移过滤、速度过滤、邻居链路优化（Odometry refining）、图优化、姿态更新、landmark 管理、约束图更新、里程管理。
	//============================================================
	// 1. 变量初始化
	bool smallDisplacement = false;  // 机器人移动太小
	bool tooFastMovement = false;  // 机器人移动太快
	std::list<int> signaturesRemoved;  // 被删除节点
	bool neighborLinkRefined = false;   // 是否执行了 ICP refine
	bool addedNewLandmark = false;  // 新增 landmark
	float distanceToClosestNodeInTheGraph = 0;  // 最近关键帧距离
	float angleToClosestNodeInTheGraph = 0;  // 最近关键帧角度差
	if(_rgbdSlamMode)
	{
		// 2. 处理 odometry 协方差
		double linVar = odomCovariance.empty()?1.0f:uMax3(odomCovariance.at<double>(0,0), odomCovariance.at<double>(1,1)>=9999?0:odomCovariance.at<double>(1,1), odomCovariance.at<double>(2,2)>=9999?0:odomCovariance.at<double>(2,2));
		double angVar = odomCovariance.empty()?1.0f:uMax3(odomCovariance.at<double>(3,3)>=9999?0:odomCovariance.at<double>(3,3), odomCovariance.at<double>(4,4)>=9999?0:odomCovariance.at<double>(4,4), odomCovariance.at<double>(5,5));
		statistics_.addStatistic(Statistics::kMemoryOdometry_variance_lin(), (float)linVar);
		statistics_.addStatistic(Statistics::kMemoryOdometry_variance_ang(), (float)angVar);

		//Verify if there was a rehearsal
		// rehearsal 检查（节点是否与上一帧合并）'MemoryRehearsal/merged'
		int rehearsedId = (int)uValue(statistics_.data(), Statistics::kMemoryRehearsal_merged(), 0.0f);
		if(rehearsedId > 0)
		{
			// 如果两帧太相似，则不创建新节点，而是合并。
			// 这时旧节点的 optimized pose 要清除。
			_optimizedPoses.erase(rehearsedId);
		}
		else
		{
			if(_rgbdLinearUpdate > 0.0f || _rgbdAngularUpdate > 0.0f)
			{
				//============================================================
				// Minimum displacement required to add to Memory
				// 运动过滤：是否是“小位移”帧, 若机器人根本没动多少，则该帧不会加入图结构。
				//============================================================
				Transform t;

				if(_memory->isIncremental())
				{
					const std::multimap<int, Link> & links = signature->getLinks();
					if(links.size() && links.begin()->second.type() == Link::kNeighbor)
					{
						const Signature * s = _memory->getSignature(links.begin()->second.to());
						UASSERT(s!=0);
						// don't filter if the new node is not intermediate but previous one is
						if(signature->getWeight() < 0 || s->getWeight() >= 0)
						{
							// 获取当前节点与第一帧的转换
							t = links.begin()->second.transform();
						}
					}
				}
				else if(!_odomCachePoses.empty())
				{
					// 从缓存中获取第一帧与当前节点的转换
					t = _odomCachePoses.rbegin()->second.inverse() * signature->getPose();
				}
				if(!t.isNull())
				{
					// 如果变换 t 很小
					float x,y,z, roll,pitch,yaw;
					t.getTranslationAndEulerAngles(x,y,z, roll,pitch,yaw);
					bool isMoving = fabs(x) > _rgbdLinearUpdate ||
									fabs(y) > _rgbdLinearUpdate ||
									fabs(z) > _rgbdLinearUpdate ||
									(_rgbdAngularUpdate>0.0f && (
										fabs(roll) > _rgbdAngularUpdate ||
										fabs(pitch) > _rgbdAngularUpdate ||
										fabs(yaw) > _rgbdAngularUpdate));
					if(!isMoving)
					{
						// This will disable global loop closure detection, only retrieval will be done.
						// The location will also be deleted at the end.
						// 不进行全局回环检测; 该节点最终会被移除; 只进行外观检索（retrieval）
						smallDisplacement = true;
						UDEBUG("smallDisplacement: %f %f %f %f %f %f", x,y,z, roll,pitch,yaw);
					}
				}
			}
			if(odomVelocity.size() == 6)
			{
				// This will disable global loop closure detection, only retrieval will be done.
				// The location will also be deleted at the end.
				// 速度太快意味着：视觉特征失效; ICP 可能失败; 回环检测不可靠
				// 速度过快过滤: 禁止执行 neighbor link refining（ICP）; 该帧只用于外观检索，不加入图优化
				tooFastMovement =
						(_rgbdLinearSpeedUpdate>0.0f && uMax3(fabs(odomVelocity[0]), fabs(odomVelocity[1]), fabs(odomVelocity[2])) > _rgbdLinearSpeedUpdate) ||
						(_rgbdAngularSpeedUpdate>0.0f && uMax3(fabs(odomVelocity[3]), fabs(odomVelocity[4]), fabs(odomVelocity[5])) > _rgbdAngularSpeedUpdate);
			}
		}

		// Update optimizedPoses with the newly added node
		// 邻居链路优化 Neighbor Link Refining（最关键）
		Transform newPose;
		bool intermediateNodeRefining = false;
		if(_neighborLinkRefining && // 是否对相邻节点（neighbor links）之间的约束关系进行二次精细化（refine）优化。
			signature->getLinks().size() &&
			signature->getLinks().begin()->second.type() == Link::kNeighbor &&
		   _memory->isIncremental() && // ignore pose matching in localization mode 定位模式不执行
		   rehearsedId == 0 && // don't do it if rehearsal happened 没有 rehearsal
		   !tooFastMovement) // ignore if too fast movement has been detected 没有 tooFastMovement
		{
			int oldId = signature->getLinks().begin()->first;
			const Signature * oldS = _memory->getSignature(oldId);
			UASSERT(oldS != 0);

			if(signature->getWeight() >= 0 && oldS->getWeight()>=0) // 忽略非关键节点
			{
				// link 表示 A→B，那我们求逆得到 B→A 的初步估计（guess）
				Transform guess = signature->getLinks().begin()->second.transform().inverse();

				if(smallDisplacement)  // 微小移动时
				{
					if(signature->getLinks().begin()->second.transVariance() == 1)
					{
						// set small variance
						// 设置很大的协方差 → 表示该测量不可靠
						UDEBUG("Set small variance. The robot is not moving.");
						_memory->updateLink(Link(oldId, signature->id(), signature->getLinks().begin()->second.type(), guess, cv::Mat::eye(6,6,CV_64FC1)*1000));
					}
				}
				else
				{
					//============================================================
					// Refine neighbor links
					// 执行 ICP refine（核心）
					//============================================================
					UINFO("Odometry refining: guess = %s", guess.prettyPrint().c_str());
					// 使用特征匹配＋ICP 计算 refined transform
					RegistrationInfo info;
					Transform t = _memory->computeTransform(oldId, signature->id(), guess, &info);
					if(!t.isNull())
					{
						// 计算成功将邻近边添加到memory中
						UINFO("Odometry refining: update neighbor link (%d->%d, variance:lin=%f, ang=%f) from %s to %s",
								oldId,
								signature->id(),
								info.covariance.at<double>(0,0),
								info.covariance.at<double>(5,5),
								guess.prettyPrint().c_str(),
								t.prettyPrint().c_str());
						UASSERT(info.covariance.at<double>(0,0) > 0.0 && info.covariance.at<double>(5,5) > 0.0);
						_memory->updateLink(Link(oldId, signature->id(), signature->getLinks().begin()->second.type(), t, info.covariance.inv()));

						if(_optimizeFromGraphEnd)
						{
							// update all previous nodes
							// Normally _mapCorrection should be identity, but if _optimizeFromGraphEnd
							// parameters just changed state, we should put back all poses without map correction.
							// 
							Transform u = guess * t.inverse();
							std::map<int, Transform>::iterator jter = _optimizedPoses.find(oldId);
							UASSERT(jter!=_optimizedPoses.end());
							Transform up = jter->second * u * jter->second.inverse();
							Transform mapCorrectionInv = _mapCorrection.inverse();
							for(std::map<int, Transform>::iterator iter=_optimizedPoses.begin(); iter!=_optimizedPoses.end(); ++iter)
							{
								iter->second = mapCorrectionInv * up * iter->second;
							}
						}
					}
					else
					{
						// 计算失败设置很大的协方差
						UINFO("Odometry refining rejected: %s", info.rejectedMsg.c_str());
						if(!info.covariance.empty() && info.covariance.at<double>(0,0) > 0.0 && info.covariance.at<double>(0,0) != 1.0 && info.covariance.at<double>(5,5) > 0.0 && info.covariance.at<double>(5,5) != 1.0)
						{
							if(ULogger::level() <= ULogger::kInfo)
							{
								std::cout << info.covariance << std::endl;
							}
							_memory->updateLink(Link(oldId, signature->id(), signature->getLinks().begin()->second.type(), guess, (info.covariance*100.0).inv()));
						}
					}
					neighborLinkRefined = !t.isNull();
					statistics_.addStatistic(Statistics::kNeighborLinkRefiningAccepted(),neighborLinkRefined?1.0f:0);
					statistics_.addStatistic(Statistics::kNeighborLinkRefiningInliers(), info.inliers);
					statistics_.addStatistic(Statistics::kNeighborLinkRefiningICP_inliers_ratio(), info.icpInliersRatio);
					statistics_.addStatistic(Statistics::kNeighborLinkRefiningICP_rotation(), info.icpRotation);
					statistics_.addStatistic(Statistics::kNeighborLinkRefiningICP_translation(), info.icpTranslation);
					statistics_.addStatistic(Statistics::kNeighborLinkRefiningICP_complexity(), info.icpStructuralComplexity);
					statistics_.addStatistic(Statistics::kNeighborLinkRefiningPts(), signature->sensorData().laserScanRaw().size());
				}
				// ICP耗时计算
				timeNeighborLinkRefining = timer.ticks();
				ULOGGER_INFO("timeOdometryRefining=%fs", timeNeighborLinkRefining);

				UASSERT(oldS->hasLink(signature->id()));
				UASSERT(uContains(_optimizedPoses, oldId));

				// 统计信息记录
				statistics_.addStatistic(Statistics::kNeighborLinkRefiningVariance(), oldS->getLinks().find(signature->id())->second.transVariance());

				// 更新 optimizedPoses（图优化后的位姿）
				newPose = _optimizedPoses.at(oldId) * oldS->getLinks().find(signature->id())->second.transform();
				_mapCorrection = newPose * signature->getPose().inverse();
				if(_mapCorrection.getNormSquared() > 0.001f && _optimizeFromGraphEnd)
				{
					UERROR("Map correction should be identity when optimizing from the last node. T=%s NewPose=%s OldPose=%s",
							_mapCorrection.prettyPrint().c_str(),
							newPose.prettyPrint().c_str(),
							signature->getPose().prettyPrint().c_str());
				}
			}
			else
			{
				newPose = _mapCorrection * signature->getPose();
				intermediateNodeRefining = true;
			}
		}
		else
		{
			newPose = _mapCorrection * signature->getPose();
		}

		// Get statistics about the closest node in the graph
		// 定位模式下，找到地图上最靠近当前位置的节点
		if(!_memory->isIncremental())
		{
			int closestNode = 0;
			float sqrdDistance = 0.0f;
			if(_optimizedPoses.begin()->first < 0)
			{
				std::map<int, Transform> poses(_optimizedPoses.lower_bound(1), _optimizedPoses.end());
				closestNode = graph::findNearestNode(poses, newPose, &sqrdDistance);
			}
			else
			{
				closestNode = graph::findNearestNode(_optimizedPoses, newPose, &sqrdDistance);
			}
			if(closestNode>0 && sqrdDistance>0.0f)
			{
				distanceToClosestNodeInTheGraph = sqrt(sqrdDistance);
				UDEBUG("Last localization pose = %s, closest node=%d (%f m)", newPose.prettyPrint().c_str(), closestNode, distanceToClosestNodeInTheGraph);
				angleToClosestNodeInTheGraph = newPose.getAngle(_optimizedPoses.at(closestNode));
			}
		}

		UDEBUG("Added pose %s (odom=%s)", newPose.prettyPrint().c_str(), signature->getPose().prettyPrint().c_str());
		// Update Poses and Constraints
		// 添加 Landmarks（AprilTags、特征点地图等）约束
		_optimizedPoses.insert(std::make_pair(signature->id(), newPose));
		if(_memory->isIncremental() && signature->getWeight() >= 0)
		{
			// 建图模式下添加landmark 约束
			for(std::map<int, Link>::const_iterator iter = signature->getLandmarks().begin(); iter!=signature->getLandmarks().end(); ++iter)
			{
				if(_optimizedPoses.find(iter->first) == _optimizedPoses.end())
				{
					_optimizedPoses.insert(std::make_pair(iter->first, newPose*iter->second.transform()));
					UDEBUG("Added landmark %d : %s", iter->first, (newPose*iter->second.transform()).prettyPrint().c_str());
					addedNewLandmark = true;
				}
				_constraints.insert(std::make_pair(iter->first, iter->second.inverse()));
			}
		}

		float distanceTravelledOld = _distanceTravelled;

		// only in mapping mode we add a neighbor link
		// 更新 graph constraints（图边）
		if(signature->getLinks().size() &&
		   signature->getLinks().begin()->second.type() == Link::kNeighbor)
		{
			// link should be old to new
			UASSERT_MSG(signature->id() > signature->getLinks().begin()->second.to(),
					"Only forward links should be added.");

			// 从 signature 的第一个 Link 中取出它的逆向 Link，并存到 tmp。
			Link tmp = signature->getLinks().begin()->second.inverse();

			if(!smallDisplacement)
			{
				_distanceTravelled += tmp.transform().getNorm();
			}

			// if the previous node is an intermediate node, remove it from the local graph
			// 如果前一个节点是 intermediate（非关键节点）：合并中间节点 删除中间节点 图拓扑简化
			if(_constraints.size() &&
			   _constraints.rbegin()->second.to() == signature->getLinks().begin()->second.to())
			{
				const Signature * s = _memory->getSignature(signature->getLinks().begin()->second.to());
				UASSERT(s!=0);
				if(s->getWeight() == -1)
				{
					tmp = _constraints.rbegin()->second.merge(tmp, tmp.type());
					_optimizedPoses.erase(s->id());
					_constraints.erase(--_constraints.end());
				}
			}

			_constraints.insert(std::make_pair(tmp.from(), tmp));
		}
		// Localization mode stuff
		if( signature->getWeight() >= 0 &&
			!smallDisplacement &&
		    odomCovariance.cols == 6 &&
			odomCovariance.rows == 6 &&
			odomCovariance.type() == CV_64FC1 &&
			odomCovariance.at<double>(0,0) < 1)
		{
			if( _memory->isIncremental() && _localizationCovariance.empty())
			{
				_localizationCovariance = cv::Mat::zeros(6,6,CV_64FC1);
			}
			if(_localizationCovariance.total() == 36)
			{
#ifdef RTABMAP_MRPT
				// Transform odometry covariance (which in base frame) into global frame
				// "odometry error propagation law"
				Eigen::Quaterniond rotation = _lastLocalizationPose.getQuaterniond();
				mrpt::poses::CPose3D pose = mrpt::poses::CPose3D::FromQuaternion(mrpt::math::CQuaternionDouble(rotation.w(), rotation.x(), rotation.y(), rotation.z()));
				mrpt::math::CMatrixDouble66 gaussian;
				gaussian.loadFromRawPointer((const double*)odomCovariance.data);
				mrpt::poses::CPose3DPDFGaussian gaussianTransformed(mrpt::poses::CPose3D(), gaussian);
				gaussianTransformed.changeCoordinatesReference(pose);
				_localizationCovariance += cv::Mat(6,6,CV_64FC1, gaussianTransformed.cov.data());
#else
				// Assuming diagonal uniform covariance matrix!
				// If variance is different for each axis,
				// build rtabmap with MRPT to use approach above.
				_localizationCovariance += odomCovariance;
#endif
			}
		}
		_lastLocalizationPose = newPose; // keep in cache the latest corrected pose
		// Localization 模式相关缓存
		if(!_memory->isIncremental() && signature->getWeight() >= 0)
		{
			UDEBUG("Update odometry localization cache (size=%d/%d)", (int)_odomCachePoses.size(), _maxOdomCacheSize);
			if(!_odomCachePoses.empty())
			{
				// 更新里程
				float odomDistance = (_odomCachePoses.rbegin()->second.inverse() * signature->getPose()).getNorm();
				if(!smallDisplacement)
				{
					_distanceTravelled += odomDistance;
				}

				// 限制缓存大小,过大清除最早的缓存
				while(!_odomCachePoses.empty() && (int)_odomCachePoses.size() > _maxOdomCacheSize)
				{
					_odomCacheConstraints.erase(_odomCachePoses.begin()->first);
					_odomCachePoses.erase(_odomCachePoses.begin());
				}
				// 缓存 constraints
				if(!_odomCachePoses.empty())
				{
					Link odomLink(_odomCachePoses.rbegin()->first,
							signature->id(),
							Link::kNeighbor,
							_odomCachePoses.rbegin()->second.inverse() * signature->getPose(),
							odomCovariance.inv());
					_odomCacheConstraints.insert(std::make_pair(_odomCachePoses.rbegin()->first, odomLink));
					UDEBUG("Added odom cov = %f %f", odomLink.transVariance(), odomLink.rotVariance());
				}
			}

			// 缓存 odometry poses
			_odomCachePoses.insert(std::make_pair(signature->id(), signature->getPose()));
		}
		// 里程累加
		_distanceTravelledSinceLastLocalization += _distanceTravelled - distanceTravelledOld;

		//============================================================
		// Reduced graph
		//============================================================
		//Verify if there are nodes that were merged through graph reduction
		// 这段代码做了三件事：
		// 1 更新路径中节点的 ID（因为有些 ID 被替换成回环闭合后的新 ID）
		// 2 删除已经被替换掉的旧 ID 对应的位姿 
		// 3 删除与这些旧 ID 有关的约束（Links）
		// 这样保证优化后的图（poses + constraints + path）保持一致性。
		if(statistics_.reducedIds().size())
		{
			// 确认 reducedIds 是否非空
			// 如果存在 ID 映射，则需要对路径、位姿、约束进行更新。
			for(unsigned int i=0; i<_path.size(); ++i)
			{
				// 更新路径 _path 中的节点 ID
				std::map<int, int>::const_iterator iter = statistics_.reducedIds().find(_path[i].first);
				if(iter!= statistics_.reducedIds().end())
				{
					// change path ID to loop closure ID
					_path[i].first = iter->second;
				}
			}

			// 处理被移除的原始 ID：删除其位姿和相关约束
			for(std::map<int, int>::const_iterator iter=statistics_.reducedIds().begin();
				iter!=statistics_.reducedIds().end();
				++iter)
			{
				int erased = (int)_optimizedPoses.erase(iter->first);
				if(erased)
				{
					for(std::multimap<int, Link>::iterator jter = _constraints.begin(); jter!=_constraints.end();)
					{
						if(jter->second.from() == iter->first || jter->second.to() == iter->first)
						{
							_constraints.erase(jter++);
						}
						else
						{
							++jter;
						}
					}
				}
			}
		}

		//============================================================
		// Local loop closure in TIME
		// RTAB-Map 中 local loop closure（局部回环）检测逻辑
		// 当前节点与时间上相邻的节点足够接近时，尝试计算两者的相对变换，如果成功则添加一个“局部时间回环（local time closure）”约束。
		//============================================================
		// _proximityByTime = true → 开启“基于时间的邻近回环” 
		// intermediateNodeRefining = true → 中间节点需要优化，也会触发类似逻辑
		if((_proximityByTime || intermediateNodeRefining) &&
		   rehearsedId == 0 && // 如果当前节点是“复习节点（rehearsed）”就跳过（因为它已经与其他节点强关联过）。
		   _memory->isIncremental() && // 必须是 SLAM 模式，不能是只定位模式
		   signature->getWeight()>=0) // 权重小于 0 的节点一般不参与图优化，如虚拟节点／无效节点。
		{
			const std::set<int> & stm = _memory->getStMem();
			// 遍历 STM（短期记忆）中的节点
			for(std::set<int>::const_reverse_iterator iter = stm.rbegin(); iter!=stm.rend(); ++iter)
			{
				// 筛选可用于做局部回环的 target 节点
				// 必须满足： 
				// 1 不是当前节点本身 
				// 2 当前节点与该节点之间还没有 Link（避免重复添加） 
				// 3 两者属于同一个地图（mapId） 
				// 4 权重有效
				if(*iter != signature->id() &&
				   signature->getLinks().find(*iter) == signature->getLinks().end() &&
				   _memory->getSignature(*iter)->mapId() == signature->mapId() &&
				   _memory->getSignature(*iter)->getWeight()>=0)
				{
					std::string rejectedMsg;
					UDEBUG("Check local transform between %d and %d", signature->id(), *iter);
					RegistrationInfo info;
					Transform guess;
					// 计算两节点之间的相对位姿 transform
					if(_optimizedPoses.find(*iter) != _optimizedPoses.end())
					{
						guess = _optimizedPoses.at(*iter).inverse() * newPose;
					}

					// For proximity by time, correspondences should be already enough precise, so don't recompute them
					// 使用视觉或匹配方法计算 transform
					Transform transform = _memory->computeTransform(*iter, signature->id(), guess, &info, true);

					// 若成功返回非空 transform，就认为找到局部时间回环。
					if(!transform.isNull())
					{
						transform = transform.inverse();
						UDEBUG("Add local loop closure in TIME (%d->%d) %s",
								signature->id(),
								*iter,
								transform.prettyPrint().c_str());
						// Add a loop constraint
						// 回环成功：添加一个类型为 kLocalTimeClosure（局部时间回环） 的边
						UASSERT(info.covariance.at<double>(0,0) > 0.0 && info.covariance.at<double>(5,5) > 0.0);
						if(_memory->addLink(Link(signature->id(), *iter, Link::kLocalTimeClosure, transform, getInformation(info.covariance))))
						{
							++proximityDetectionsInTimeFound;
							UINFO("Local loop closure found between %d and %d with t=%s",
									*iter, signature->id(), transform.prettyPrint().c_str());
						}
						else
						{
							UWARN("Cannot add local loop closure between %d and %d ?!?",
									*iter, signature->id());
						}
					}
					else
					{
						UINFO("Local loop closure (time) between %d and %d rejected: %s",
								*iter, signature->id(), rejectedMsg.c_str());
					}

					// 特殊逻辑：只对第一个非中间节点执行
					// 如果只是为了中间节点细化，并非 proximity-by-time，则找到第一个匹配对象后就停止
					if(!_proximityByTime && intermediateNodeRefining)
					{
						// Do it only with the latest non-intermediate node
						break;
					}
				}
			}
		}
	}

	timeProximityByTimeDetection = timer.ticks();
	UINFO("timeProximityByTimeDetection=%fs", timeProximityByTimeDetection);

	//============================================================
	// Bayes filter update
	// 这段代码是 RTAB-Map 中最核心的部分之一：似然计算 + Bayes 过滤 + 回环（loop closure）假设选择与验证。
	//============================================================
	// 判断 上一次更新是否发生过定位回环
	bool localizationOnPreviousUpdate = false;
	if(_memory->isIncremental())
	{
		// SLAM模式 如果上一帧有过 loop closure： 
		// 当前 signature 有 link , link 指向别的节点 , 该节点存在 loop closure links → 则认为“上一帧发生过定位回环”
		localizationOnPreviousUpdate =
			signature->getLinks().size() &&
			signature->getLinks().begin()->first!=signature->id() &&
			_memory->getLoopClosureLinks(signature->getLinks().begin()->first, false).size() != 0;
	}
	else
	{
		// localization mode
		// Count how many localization links are in the constraints

		// Localization 模式 遍历 odomCacheConstraints，看是否存在超过 1 个定位相关的边（为了过滤延迟检测）。
		int localizationLinks = 0;
		int previousIdWithLocalizationLink = 0;
		for(std::multimap<int, Link>::iterator iter=_odomCacheConstraints.begin();
				iter!=_odomCacheConstraints.end(); ++iter)
		{
			if(previousIdWithLocalizationLink == iter->first)
			{
				// ignore links with node already counted
				continue;
			}
			if(iter->second.type() == Link::kGlobalClosure ||
			   iter->second.type() == Link::kLocalSpaceClosure ||
			   iter->second.type() == Link::kLocalTimeClosure ||
			   iter->second.type() == Link::kUserClosure ||
			   iter->second.type() == Link::kNeighborMerged ||
			   iter->second.type() == Link::kLandmark)
			{
				++localizationLinks;
				previousIdWithLocalizationLink = iter->first;
			}
		}

		localizationOnPreviousUpdate = localizationLinks > 1; // need two links in case we have delayed localization
	}

	// Not a bad signature, not an intermediate node, not a small displacement unless the previous signature didn't have a loop closure, not too fast movement
	// 判断是否允许进行回环检测,要求：图像质量足够 & 不是中间节点、虚拟节点 & 不小位移 或 上一帧没有定位回环 & 不是太快移动
	if(!signature->isBadSignature() && signature->getWeight()>=0 && (!smallDisplacement || !localizationOnPreviousUpdate) && !tooFastMovement)
	{
		// If the working memory is empty, don't do the detection. It happens when it
		// is the first time the detector is started (there needs some images to
		// fill the short-time memory before a signature is added to the working memory).
		// 必须 Working Memory 非空 第一次启动时，WM 未满 → 不做回环
		if(_memory->getWorkingMem().size())
		{
			//============================================================
			// Likelihood computation
			// Get the likelihood of the new signature
			// with all images contained in the working memory + reactivated.
			// 计算似然用的候选节点（signaturesToCompare）
			//============================================================
			ULOGGER_INFO("computing likelihood...");

			std::list<int> signaturesToCompare; // 存储 待比较回环的节点 ID 列表
			GPS originGPS; // 当前节点的 参考 GPS 坐标,如果当前节点没有 GPS，会从附近节点推断
			Transform originOffsetENU = Transform::getIdentity(); // 局部坐标系 → ENU 坐标系的偏移变换
			// 利用 GPS 将地图节点转换到 ENU 坐标系，并过滤掉距离过远的回环候选节点。
			// 当启用 GPS 回环约束（_loopGPS）时，计算当前节点在 ENU 坐标系中的位置，并为后续的回环检测过滤提供参考坐标。
			if(_loopGPS)
			{
				// 尝试直接使用当前节点的 GPS
				originGPS = signature->sensorData().gps();
				// stamp() == 0.0 → 当前节点 没有 GPS 
				// _currentSessionHasGPS → 当前会话中 曾经有 GPS 数据 
				// 说明：可以从历史节点推算当前 GPS
				if(originGPS.stamp() == 0.0 && _currentSessionHasGPS)
				{
					UTimer tmpT;
					// 已有优化后的位姿图 & 处于增量建图模式（非纯定位）
					if(_optimizedPoses.size() && _memory->isIncremental())
					{
						//Search for latest node having GPS linked to current signature not too far.
						// 在附近节点中查找最近的 GPS 节点 <key：节点 ID,value：图中距离（或空间距离）>
						std::map<int, float> nearestIds = graph::findNearestNodes(signature->id(), _optimizedPoses, _localRadius);
						for(std::map<int, float>::reverse_iterator iter=nearestIds.rbegin(); iter!=nearestIds.rend() && iter->first>0; ++iter)
						{
							const Signature * s = _memory->getSignature(iter->first);
							UASSERT(s!=0);
							if(s->sensorData().gps().stamp() > 0.0)
							{
								// 找到第一个有 GPS 的节点, 作为参考节点
								// 获取该节点的 GPS
								originGPS = s->sensorData().gps();
								// 获取该节点在优化图中的位姿
								const Transform & sPose = _optimizedPoses.at(s->id());
								// 构造 ENU 方向旋转,最终得到 局部坐标对齐到 ENU 的旋转
								Transform localToENU(0,0,(float)((-(originGPS.bearing()-90))*M_PI/180.0) - sPose.theta());
								// 计算当前节点的 ENU 偏移
								originOffsetENU = localToENU * (sPose.rotation()*(sPose.inverse()*_optimizedPoses.at(signature->id())));
								break;
							}
						}
					}
					//else if(!_memory->isIncremental()) // TODO, how can we estimate current GPS position in localization?
					//{
					//}
				}
				if(originGPS.stamp() > 0.0)
				{
					// no need to save it if it is in localization mode
					_gpsGeocentricCache.insert(std::make_pair(signature->id(), std::make_pair(originGPS.toGeodeticCoords().toGeocentric_WGS84(), originOffsetENU)));
				}
			}

			// 遍历工作记忆中的所有节点，根据 GPS 计算它们与当前节点的 ENU 空间距离，只保留在 _localRadius 范围内的节点作为回环候选。
			// 如果没有 GPS，则不进行过滤，全部接受。
			// _workingMem <id,age>
			for(std::map<int, double>::const_iterator iter=_memory->getWorkingMem().begin();
				iter!=_memory->getWorkingMem().end();
				++iter)
			{
				// id > 0：真实节点
				if(iter->first > 0)
				{
					const Signature * s = _memory->getSignature(iter->first);
					UASSERT(s!=0);
					if(s->getWeight() != -1) // 忽略中间节点，weight == -1表示 中间节点 / 临时节点,不参与回环
					{
						bool accept = true;
						// 如果 当前节点有 GPS → 才启用 GPS 过滤
						if(originGPS.stamp()>0.0)
						{
							// 查找/构建候选节点的 GPS 缓存
							std::map<int, std::pair<cv::Point3d, Transform> >::iterator cacheIter = _gpsGeocentricCache.find(s->id());
							//  GPS 缓存不存在 → 尝试构建
							if(cacheIter == _gpsGeocentricCache.end())
							{
								// 读取节点 GPS
								GPS gps = s->sensorData().gps();
								Transform offsetENU = Transform::getIdentity();
								// 如果节点本身没有 GPS
								if(gps.stamp()==0.0)
								{
									// 推算该节点的 GPS 和 ENU 偏移
									_memory->getGPS(s->id(), gps, offsetENU, false);
								}
								// 成功获取 GPS → 存入缓存
								if(gps.stamp() > 0.0)
								{
									cacheIter = _gpsGeocentricCache.insert(
											std::make_pair(s->id(),
													std::make_pair(gps.toGeodeticCoords().toGeocentric_WGS84(), offsetENU))).first;
								}
							}

							// 如果该节点最终有 GPS
							if(cacheIter != _gpsGeocentricCache.end())
							{
								// 找到当前节点（origin）的缓存
								std::map<int, std::pair<cv::Point3d, Transform> >::iterator originIter = _gpsGeocentricCache.find(signature->id());
								UASSERT(originIter != _gpsGeocentricCache.end());
								// 计算 ENU 相对位移
								cv::Point3d relativePose = GeodeticCoords::Geocentric_WGS84ToENU_WGS84(cacheIter->second.first, originIter->second.first, originGPS.toGeodeticCoords());
								// 处理 GPS 误差
								const double & error = originGPS.error();
								// 应用 ENU 偏移补偿
								const Transform & offsetENU = cacheIter->second.second;
								relativePose.x += offsetENU.x() - originOffsetENU.x();
								relativePose.y += offsetENU.y() - originOffsetENU.y();
								relativePose.z += offsetENU.z() - originOffsetENU.z();
								// ignore altitude if difference is under GPS error 如果高度差 小于误差 → 忽略
								//  Z 轴（高度）特殊处理
								if(relativePose.z>error)
								{
									relativePose.z -= error;
								}
								else if(relativePose.z < -error)
								{
									relativePose.z += error;
								}
								else
								{
									relativePose.z = 0;
								}
								// 在 GPS 空间距离内 → 接受
								accept = uNormSquared(relativePose.x, relativePose.y, relativePose.z) < _localRadius*_localRadius;
							}
						}

						// 否则：全部节点直接接受（当前节点没有GPS时）
						if(accept)
						{
							signaturesToCompare.push_back(iter->first);
						}
					}
				}
				// id <= 0：虚拟节点（Virtual Signature）
				else
				{
					// virtual signature should be added
					signaturesToCompare.push_back(iter->first);
				}
			}

			// Likelihood（似然）计算
			// RTAB-Map 回环检测中“外观相似度计算”的核心实现之一。
			// 这是 RTAB-Map 的核心：使用 Bag-of-Words / visual features & 对新节点与所有候选节点计算匹配得分（似然）
			rawLikelihood = _memory->computeLikelihood(signature, signaturesToCompare);

			// Adjust the likelihood (with mean and std dev)
			// 再做标准化（adjustLikelihood）
			likelihood = rawLikelihood;
			this->adjustLikelihood(likelihood);

			timeLikelihoodCalculation = timer.ticks();
			ULOGGER_INFO("timeLikelihoodCalculation=%fs",timeLikelihoodCalculation);

			//============================================================
			// Apply the Bayes filter
			//  Posterior = Likelihood x Prior 其中 Prior 是时间相关的转移概率（Markov chain）
			// Bayes Filter：后验计算
			//============================================================
			ULOGGER_INFO("getting posterior...");

			// Compute the posterior
			posterior = _bayesFilter->computePosterior(_memory, likelihood);
			timePosteriorCalculation = timer.ticks();
			ULOGGER_INFO("timePosteriorCalculation=%fs",timePosteriorCalculation);

			// For statistics, copy weights
			if(_publishStats && (_publishLikelihood || _publishPdf))
			{
				weights = _memory->getWeights();
			}

			//============================================================
			// Select the highest hypothesis
			// 从 posterior 中选出最高假设
			//============================================================
			ULOGGER_INFO("creating hypotheses...");
			if(posterior.size())
			{
				for(std::map<int, float>::const_reverse_iterator iter = posterior.rbegin(); iter != posterior.rend(); ++iter)
				{
					if(iter->first > 0 && iter->second > _highestHypothesis.second)
					{
						_highestHypothesis = *iter;
					}
				}
				// With the virtual place, use sum of LC probabilities (1 - virtual place hypothesis).
				// 把“虚拟场所”的概率减掉
				_highestHypothesis.second = 1-posterior.begin()->second;
			}
			timeHypothesesCreation = timer.ticks();
			ULOGGER_INFO("Highest hypothesis=%d, value=%f, timeHypothesesCreation=%fs", _highestHypothesis.first, _highestHypothesis.second, timeHypothesesCreation);

			if(_highestHypothesis.first > 0)
			{
				// 综合计算阈值
				float loopThr = _loopThr;
				bool hasLoopClosureConstraints = false;
				// 判断是否存在回环约束
				for(std::multimap<int, Link>::iterator iter=_odomCacheConstraints.begin(); iter!=_odomCacheConstraints.end() && !hasLoopClosureConstraints; ++iter)
				{
					hasLoopClosureConstraints =
							iter->second.type() == Link::kGlobalClosure ||
							iter->second.type() == Link::kLocalSpaceClosure ||
							iter->second.type() == Link::kLandmark;
				}
				if(	(( _memory->isIncremental() && !uContains(_optimizedPoses, _highestHypothesis.first)) || // not linked to previous map of that hypothesis
					 (!_memory->isIncremental() && !hasLoopClosureConstraints)) && // not yet localized to any previous sessions
					_memory->getWorkingMem().size()>1 && // should have an old map (beside virtual signature)
					_rgbdSlamMode &&
					loopThr > _aggressiveLoopThr)
				{
					// use the best hypothesis directly.
					UDEBUG("Using %s=%f", Parameters::kRGBDAggressiveLoopThr().c_str(), _aggressiveLoopThr);
					loopThr = _aggressiveLoopThr;
				}

				// Loop closure Threshold
				// 验证回环假设
				if(_highestHypothesis.second >= loopThr)
				{
					rejectedLoopClosure = true;
					if(posterior.size() <= 2 && loopThr>0.0f)
					{
						// Ignore loop closure if there is only one loop closure hypothesis
						// 只有一个闭环假设，过于单一 → 可能属于自相似 → 拒绝
						UDEBUG("rejected hypothesis: single hypothesis");
					}
					else if(_verifyLoopClosureHypothesis && !_epipolarGeometry->check(signature, _memory->getSignature(_highestHypothesis.first)))
					{
						// 极线几何检查（Epipolar Geometry）
						UWARN("rejected hypothesis: by epipolar geometry");
					}
					else if(_loopRatio > 0.0f && lastHighestHypothesis.second && _highestHypothesis.second < _loopRatio*lastHighestHypothesis.second)
					{
						UWARN("rejected hypothesis: not satisfying hypothesis ratio (%f < %f * %f)",
								_highestHypothesis.second, _loopRatio, lastHighestHypothesis.second);
					}
					else if(_loopRatio > 0.0f && lastHighestHypothesis.second == 0)
					{
						UWARN("rejected hypothesis: last closure hypothesis is null (loop ratio is on)");
					}
					else
					{
						// 储存当前最好的闭环匹配节点 ID + posterior 评分
						_loopClosureHypothesis = _highestHypothesis;
						// 通过验证
						rejectedLoopClosure = false;
					}

					timeHypothesesValidation = timer.ticks();
					ULOGGER_INFO("timeHypothesesValidation=%fs",timeHypothesesValidation);
				}
				else if(_highestHypothesis.second < _loopRatio*lastHighestHypothesis.second)
				{
					// Used for Precision-Recall computation.
					// When analyzing logs, it's convenient to know
					// if the hypothesis would be rejected if T_loop would be lower.
					rejectedLoopClosure = true;
					UDEBUG("rejected hypothesis: under loop ratio %f < %f", _highestHypothesis.second, _loopRatio*lastHighestHypothesis.second);
				}

				//for statistic...
				hypothesisRatio = _loopClosureHypothesis.second>0?_highestHypothesis.second/_loopClosureHypothesis.second:0;
			}
		} // if(_memory->getWorkingMemSize())
	}// !isBadSignature
	else if(!signature->isBadSignature() && (smallDisplacement || tooFastMovement))
	{
		_highestHypothesis = lastHighestHypothesis;
		UDEBUG("smallDisplacement=%d tooFastMovement=%d", smallDisplacement?1:0, tooFastMovement?1:0);
	}
	else
	{
		UDEBUG("Ignoring likelihood and loop closure hypotheses as current signature doesn't have enough visual features.");
	}

	//============================================================
	// Before retrieval, make sure the trash has finished
	// 在进行“检索或回环检测”之前，确保内存垃圾（trash memory）已经清理完毕。
	//============================================================
	// 阻塞等待后台“垃圾处理线程”完成
	_memory->joinTrashThread();
	// 获取垃圾线程中真正用于数据库保存（写入磁盘）的时间
	timeEmptyingTrash = _memory->getDbSavingTime();
	// 记录主线程“等待 trashThread 的实际开销”
	timeJoiningTrash = timer.ticks();
	ULOGGER_INFO("Time emptying memory trash = %fs,  joining (actual overhead) = %fs", timeEmptyingTrash, timeJoiningTrash);

	//============================================================
	// RETRIEVAL 1/3 : Loop closure neighbors reactivation
	// 当检测到回环候选节点后，从数据库或长期记忆中重新激活该节点周围的邻居节点，并进行免疫（防止它们被转移到长期内存）
	// 目标： 1 找到回环候选节点的邻居（时间邻居 + 空间邻居）
	// 2 重新加载必要的节点到 Working Memory（WM）
	// 3 给这些节点加“免疫标记”（immunization），使它们不会被 Memory Management 从 WM 中移除
	// 4 后续步骤（Retrieval 2/3 与 3/3）才能顺利进行视觉回环验证与图优化
	//============================================================
	// 基本变量与内存管理检查
	int retrievalId = _highestHypothesis.first;  // 当前回环假设的节点 ID（由 Bayes 计算出的最佳候选）
	std::list<int> reactivatedIds;
	double timeGetNeighborsTimeDb = 0.0;
	double timeGetNeighborsSpaceDb = 0.0;
	int immunizedGlobally = 0;
	int immunizedLocally = 0;
	int maxLocalLocationsImmunized = 0;
	if(_maxTimeAllowed != 0 || _maxMemoryAllowed != 0)
	{
		// with memory management, we have to immunize some nodes
		maxLocalLocationsImmunized = _localImmunizationRatio * float(_memory->getWorkingMem().size());
	}
	// no need to do retrieval or immunization of locations if memory management
	// is disabled and all nodes are in WM
	// 判断是否需要 retrieval（重新激活）
	// 只有在：WM 不是满载所有节点 或 memory management 启用 才需要 retrieval。
	if(!(_memory->allNodesInWM() && maxLocalLocationsImmunized == 0))
	{
		if(retrievalId > 0)
		{
			//Load neighbors
			ULOGGER_INFO("Retrieving locations... around id=%d", retrievalId);
			int neighborhoodSize = (int)_bayesFilter->getPredictionLC().size()-1;
			UASSERT(neighborhoodSize >= 0);
			ULOGGER_DEBUG("margin=%d maxRetieved=%d", neighborhoodSize, _maxRetrieved);

			UTimer timeGetN;
			unsigned int nbLoadedFromDb = 0;
			std::set<int> reactivatedIdsSet;
			std::map<int, int> neighbors;
			int nbDirectNeighborsInDb = 0;

			// priority in time
			// Direct neighbors TIME
			// 获取“时间邻居” TIME 邻居 = 连续的 Odom chain（比如：ID 300 → 301 → 302 → …）
			ULOGGER_DEBUG("In TIME");
			// neighbors <nodeId,拓扑距离>
			neighbors = _memory->getNeighborsId(retrievalId,
					neighborhoodSize,
					_maxRetrieved,
					true,
					true,
					false,
					true,
					std::set<int>(),
					&timeGetNeighborsTimeDb);
			ULOGGER_DEBUG("neighbors of %d in time = %d", retrievalId, (int)neighbors.size());
			//Priority to locations near in time (direct neighbor) then by space (loop closure)
			// 排序与处理 TIME 邻居
			// 作用： 1 按 m 分层处理（由近到远）2 避免加载短期记忆（STM）中的节点（因为它们已经在内存中）
			// 3 记录被重新激活的 ID 4 记录被免疫的 ID
			bool firstPassDone = false; // just to avoid checking to STM after the first pass
			int m = 0;
			// m = 邻域层级（hop distance） 0：自身 1：直接邻居 2：二跳邻居
			// neighborhoodSize：免疫/激活的最大拓扑半径
			while(m < neighborhoodSize)
			{
				// 自动排序（升序） 后面会 反向插入 到列表，保证确定性顺序
				std::set<int> idsSorted;
				for(std::map<int, int>::iterator iter=neighbors.begin(); iter!=neighbors.end();)
				{
					// 第一层特殊处理：排除 STM,也就是排除自身节点
					if(!firstPassDone && _memory->isInSTM(iter->first))
					{
						neighbors.erase(iter++);
					}
					// 匹配当前层 m 的节点
					else if(iter->second == m)
					{
						// 防止重复激活
						if(reactivatedIdsSet.find(iter->first) == reactivatedIdsSet.end())
						{
							// reactivatedIdsSet中不存在此节点
							// 加入激活集合
							idsSorted.insert(iter->first);
							reactivatedIdsSet.insert(iter->first);

							// 统计直接邻居（m == 1）
							if(m == 1 && _memory->getSignature(iter->first) == 0)
							{
								++nbDirectNeighborsInDb;
							}

							//immunized locations in the neighborhood from being transferred
							// 将该节点加入 免疫集合 
							// 被免疫的节点：不会被记忆管理算法转移/清除
							if(immunizedLocations.insert(iter->first).second)
							{
								++immunizedGlobally;
							}

							//UDEBUG("nt=%d m=%d immunized=1", iter->first, iter->second);
						}
						// 当前节点已处理， 防止后续层重复处理， 从 neighbors 中移除
						neighbors.erase(iter++);
					}
					// 不是当前层 → 跳过
					else
					{
						++iter;
					}
				}
				firstPassDone = true;
				reactivatedIds.insert(reactivatedIds.end(), idsSorted.rbegin(), idsSorted.rend());
				++m;
			}

			// neighbors SPACE, already added direct neighbors will be ignored
			// 获取“空间邻居”（Loop closure neighbors） SPACE 邻居 = 通过 loop closure 连接的节点
			ULOGGER_DEBUG("In SPACE");
			neighbors = _memory->getNeighborsId(retrievalId,
					neighborhoodSize,
					_maxRetrieved,
					true,
					false,
					false,
					false,
					std::set<int>(),
					&timeGetNeighborsSpaceDb);
			ULOGGER_DEBUG("neighbors of %d in space = %d", retrievalId, (int)neighbors.size());
			firstPassDone = false;
			m = 0;
			while(m < neighborhoodSize)
			{
				std::set<int> idsSorted;
				for(std::map<int, int>::iterator iter=neighbors.begin(); iter!=neighbors.end();)
				{
					if(!firstPassDone && _memory->isInSTM(iter->first))
					{
						neighbors.erase(iter++);
					}
					else if(iter->second == m)
					{
						if(reactivatedIdsSet.find(iter->first) == reactivatedIdsSet.end())
						{
							idsSorted.insert(iter->first);
							reactivatedIdsSet.insert(iter->first);

							if(m == 1 && _memory->getSignature(iter->first) == 0)
							{
								++nbDirectNeighborsInDb;
							}
							//UDEBUG("nt=%d m=%d", iter->first, iter->second);
						}
						neighbors.erase(iter++);
					}
					else
					{
						++iter;
					}
				}
				firstPassDone = true;
				reactivatedIds.insert(reactivatedIds.end(), idsSorted.rbegin(), idsSorted.rend());
				++m;
			}
			// 最终打印状态
			ULOGGER_INFO("neighborhoodSize=%d, "
					"reactivatedIds.size=%d, "
					"nbLoadedFromDb=%d, "
					"nbDirectNeighborsInDb=%d, "
					"time=%fs (%fs %fs)",
					neighborhoodSize,
					reactivatedIds.size(),
					(int)nbLoadedFromDb,
					nbDirectNeighborsInDb,
					timeGetN.ticks(),
					timeGetNeighborsTimeDb,
					timeGetNeighborsSpaceDb);

		}
	}

	//============================================================
	// RETRIEVAL 2/3 : Update planned path and get next nodes to retrieve
	// 在局部范围内优先保持或加载与当前位姿相关的关键帧，尤其是路径上的关键帧与最近邻关键帧。
	//============================================================
	std::list<int> retrievalLocalIds;
	if(_rgbdSlamMode)
	{
		// Priority on locations on the planned path
		// 依据当前路径（_path）加载或免疫节点
		if(_path.size())
		{
			// 先更新当前路径索引 会更新机器人当前在路径上的位置。
			updateGoalIndex();

			float distanceSoFar = 0.0f;
			// immunize all nodes after current node and
			// retrieve nodes after current node in the maximum radius from the current node
			// 沿路径向前累积距离，找到局部半径 _localRadius 内的路径节点
			for(unsigned int i=_pathCurrentIndex; i<_path.size(); ++i)
			{
				if(_localRadius > 0.0f && i != _pathCurrentIndex)
				{
					distanceSoFar += _path[i-1].second.getDistance(_path[i].second);
				}

				// 如果距离 ≤ _localRadius： 
				// 若该节点已在 WM 中 → 免疫该节点 
				// 若该节点未在 WM 中 → 加入重新加载列表 retrievalLocalIds
				if(distanceSoFar <= _localRadius)
				{
					if(_memory->getSignature(_path[i].first) != 0)
					{
						if(immunizedLocations.insert(_path[i].first).second)
						{
							++immunizedLocally;
						}
						UDEBUG("Path immunization: node %d (dist=%fm)", _path[i].first, distanceSoFar);
					}
					else if(retrievalLocalIds.size() < _maxLocalRetrieved)
					{
						UINFO("retrieval of node %d on path (dist=%fm)", _path[i].first, distanceSoFar);
						retrievalLocalIds.push_back(_path[i].first);
						// retrieved locations are automatically immunized
					}
				}
				else
				{
					UDEBUG("Stop on node %d (dist=%fm > %fm)",
							_path[i].first, distanceSoFar, _localRadius);
					break;
				}
			}
		}

		// 如果：所有节点都已经在 WM 且不允许任何局部免疫 
		// 那么 整个逻辑没有意义，直接跳过
		if(!(_memory->allNodesInWM() && maxLocalLocationsImmunized == 0))
		{
			// immunize the path from the nearest local location to the current location
			// 免疫“最近局部节点 → 当前节点”的路径
			// 免疫数量未达上限 & 增量建图模式
			if(immunizedLocally < maxLocalLocationsImmunized &&
				_memory->isIncremental()) // Can only work in mapping mode
			{
				std::map<int ,Transform> poses;
				// remove poses from STM
				// 构建可用位姿集合（去掉 STM）STM 节点：不稳定 可能马上被移除
				for(std::map<int, Transform>::iterator iter=_optimizedPoses.begin(); iter!=_optimizedPoses.end(); ++iter)
				{
					if(iter->first > 0 && !_memory->isInSTM(iter->first))
					{
						poses.insert(*iter);
					}
				}
				
				// 找出距离当前节点最近的“局部”节点
				int nearestId = graph::findNearestNode(poses, _optimizedPoses.at(signature->id()));
				// 距离阈值检查
				if(nearestId > 0 &&
					(_localRadius==0 ||
					 _optimizedPoses.at(signature->id()).getDistance(_optimizedPoses.at(nearestId)) < _localRadius))
				{
					std::multimap<int, int> links;
					// 构建局部图（约束 → 无向图）
					for(std::multimap<int, Link>::iterator iter=_constraints.begin(); iter!=_constraints.end(); ++iter)
					{
						// if(from,to 都在 optimizedPoses)
						if(uContains(_optimizedPoses, iter->second.from()) && uContains(_optimizedPoses, iter->second.to()))
						{
							links.insert(std::make_pair(iter->second.from(), iter->second.to()));
							links.insert(std::make_pair(iter->second.to(), iter->second.from())); // <->
						}
					}

					// 如果 nearestId 在局部范围内，计算二者间路径
					std::list<std::pair<int, Transform> > path = graph::computePath(_optimizedPoses, links, nearestId, signature->id());
					if(path.size() == 0)
					{
						UWARN("Could not compute a path between %d and %d", nearestId, signature->id());
					}
					else
					{
						// 沿路径免疫节点，直到达到最大允许免疫数量 maxLocalLocationsImmunized
						for(std::list<std::pair<int, Transform> >::iterator iter=path.begin();
							iter!=path.end();
							++iter)
						{
							if(iter->first>0)
							{
								if(immunizedLocally >= maxLocalLocationsImmunized)
								{
									// set 20 to avoid this warning when starting mapping
									if(maxLocalLocationsImmunized > 20 && _someNodesHaveBeenTransferred)
									{
										UWARN("Could not immunize the whole local path (%d) between "
											  "%d and %d (max location immunized=%d). You may want "
											  "to increase RGBD/LocalImmunizationRatio (current=%f (%d of WM=%d)) "
											  "to be able to immunize longer paths.",
												(int)path.size(),
												nearestId,
												signature->id(),
												maxLocalLocationsImmunized,
												_localImmunizationRatio,
												maxLocalLocationsImmunized,
												(int)_memory->getWorkingMem().size());
									}
									break;
								}
								else if(!_memory->isInSTM(iter->first))
								{
									if(immunizedLocations.insert(iter->first).second)
									{
										++immunizedLocally;
									}
									//UDEBUG("local node %d on path immunized=1", iter->first);
								}
							}
						}
					}
				}
			}

			// retrieval based on the nodes close the the nearest pose in WM
			// immunize closest nodes
			// 基于最近邻节点进行检索和免疫
			std::map<int, float> nearNodes = graph::findNearestNodes(signature->id(), _optimizedPoses, _localRadius);
			// sort by distance
			// 按距离排序。
			std::multimap<float, int> nearNodesByDist;
			for(std::map<int, float>::iterator iter=nearNodes.lower_bound(1); iter!=nearNodes.end(); ++iter)
			{
				nearNodesByDist.insert(std::make_pair(iter->second, iter->first));
			}
			UINFO("near nodes=%d, max local immunized=%d, ratio=%f WM=%d",
					(int)nearNodesByDist.size(),
					maxLocalLocationsImmunized,
					_localImmunizationRatio,
					(int)_memory->getWorkingMem().size());
			// 对邻节点进行检索和免疫，直到达到最大检索数量
			for(std::multimap<float, int>::iterator iter=nearNodesByDist.begin();
				iter!=nearNodesByDist.end() && (retrievalLocalIds.size() < _maxLocalRetrieved || immunizedLocally < maxLocalLocationsImmunized);
				++iter)
			{
				const Signature * s = _memory->getSignature(iter->second);
				if(s!=0)
				{
					// If there is a change of direction, better to be retrieving
					// ALL nearest signatures than only newest neighbors
					const std::multimap<int, Link> & links = s->getLinks();
					for(std::multimap<int, Link>::const_reverse_iterator jter=links.rbegin();
						jter!=links.rend() && retrievalLocalIds.size() < _maxLocalRetrieved;
						++jter)
					{
						// 若某个节点的邻居未在 WM 中 → 加入检索列表
						if(_memory->getSignature(jter->first) == 0)
						{
							UINFO("retrieval of node %d on local map", jter->first);
							retrievalLocalIds.push_back(jter->first);
						}
					}
					// 若该节点不在 STM 且仍需免疫 → 将其免疫
					if(!_memory->isInSTM(s->id()) && immunizedLocally < maxLocalLocationsImmunized)
					{
						if(immunizedLocations.insert(s->id()).second)
						{
							++immunizedLocally;
						}
						//UDEBUG("local node %d (%f m) immunized=1", iter->second, iter->first);
					}
				}
			}
			// well, if the maximum retrieved is not reached, look for neighbors in database
			// 如果未达到最大检索数量，再查数据库邻居
			if(retrievalLocalIds.size() < _maxLocalRetrieved)
			{
				std::set<int> retrievalLocalIdsSet(retrievalLocalIds.begin(), retrievalLocalIds.end());
				for(std::list<int>::iterator iter=retrievalLocalIds.begin();
					iter!=retrievalLocalIds.end() && retrievalLocalIds.size() < _maxLocalRetrieved;
					++iter)
				{
					// 通过 memory->getNeighborsId 获取更远邻居
					std::map<int, int> ids = _memory->getNeighborsId(*iter, 2, _maxLocalRetrieved - (unsigned int)retrievalLocalIds.size() + 1, true, false);
					for(std::map<int, int>::reverse_iterator jter=ids.rbegin();
						jter!=ids.rend() && retrievalLocalIds.size() < _maxLocalRetrieved;
						++jter)
					{
						if(_memory->getSignature(jter->first) == 0 &&
						   retrievalLocalIdsSet.find(jter->first) == retrievalLocalIdsSet.end())
						{
							UINFO("retrieval of node %d on local map", jter->first);
							retrievalLocalIds.push_back(jter->first);
							retrievalLocalIdsSet.insert(jter->first);
						}
					}
				}
			}

			// update Age of the close signatures (oldest the farthest)
			// 更新这些局部节点的“年龄”
			for(std::multimap<float, int>::reverse_iterator iter=nearNodesByDist.rbegin(); iter!=nearNodesByDist.rend(); ++iter)
			{
				_memory->updateAge(iter->second);
			}

			// insert them first to make sure they are loaded.
			// 将 retrievalLocalIds 插入 reactivatedIds，确保这些节点优先被加载：
			reactivatedIds.insert(reactivatedIds.begin(), retrievalLocalIds.begin(), retrievalLocalIds.end());
		}
	}

	//============================================================
	// RETRIEVAL 3/3 : Load signatures from the database
	//============================================================
	if(reactivatedIds.size())
	{
		// Not important if the loop closure hypothesis don't have all its neighbors loaded,
		// only a loop closure link is added...
		// 1) 从数据库中重新激活（载入）那些标记为需要重新激活的 signature（节点/特征签名）
		signaturesRetrieved = _memory->reactivateSignatures(
				reactivatedIds,
				_maxRetrieved+(unsigned int)retrievalLocalIds.size(), // add path retrieved
				timeRetrievalDbAccess);

		ULOGGER_INFO("retrieval of %d (db time = %fs)", (int)signaturesRetrieved.size(), timeRetrievalDbAccess);

		// 2) 把另外两类与“获取邻居”相关的 DB 时间加到总的检索 DB 时间中
		timeRetrievalDbAccess += timeGetNeighborsTimeDb + timeGetNeighborsSpaceDb;
		UINFO("total timeRetrievalDbAccess=%fs", timeRetrievalDbAccess);

		// Immunize just retrieved signatures
		// 3) 将刚检索到的签名“免疫化”——防止它们被内存管理再次移出
		immunizedLocations.insert(signaturesRetrieved.begin(), signaturesRetrieved.end());

		// 4) 如果确实检索到了签名，并且全局扫描映射（global scan map）不为空，
    	//    则清空全局扫描映射及其位姿。理由：全局扫描地图在重新引入节点后可能不一致。
		if(!signaturesRetrieved.empty() && !_globalScanMap.empty())
		{
			UWARN("Some signatures have been retrieved from memory management, clearing global scan map...");
			_globalScanMap.clear();
			_globalScanMapPoses.clear();
		}
	}
	timeReactivations = timer.ticks();
	ULOGGER_INFO("timeReactivations=%fs", timeReactivations);

	//============================================================
	// Proximity detections
	// RTAB-Map 的局部空间回环检测
	//============================================================
	std::list<std::pair<int, int> > loopClosureLinksAdded;
	// 统计视觉回环质量（inliers, variance, matches）
	int loopClosureVisualInliers = 0; // for statistics
	float loopClosureVisualInliersRatio = 0.0f;
	int loopClosureVisualMatches = 0;
	float loopClosureLinearVariance = 0.0f;
	float loopClosureAngularVariance = 0.0f;
	float loopClosureVisualInliersMeanDist = 0;
	float loopClosureVisualInliersDistribution = 0;

	int proximityDetectionsAddedVisually = 0;
	// 统计 ICP 匹配添加的回环数量
	int proximityDetectionsAddedByICPMulti = 0;
	int proximityDetectionsAddedByICPGlobal = 0;
	int lastProximitySpaceClosureId = 0;
	int proximitySpacePaths = 0;
	int localVisualPathsChecked = 0;
	int localScanPathsChecked = 0;
	int loopIdSuppressedByProximity = 0;

	// 是否允许空间近邻检测的条件检查
	if(_proximityBySpace &&
	   _localRadius > 0 &&
	   _rgbdSlamMode &&
	   signature->getWeight() >= 0) // 要求不是中间节点
	{
		// 一些特殊情况禁止执行 proximity detection
		// 1 当前 session 刚开始，还没有建立与旧地图的回环，不允许局部检测。
		if(_startNewMapOnLoopClosure &&
			_memory->getWorkingMem().size()>=2 && // must have an old map (+1 virtual place)
			_localizationCovariance.empty() && // if we didn't localize yet
			graph::filterLinks(signature->getLinks(), Link::kSelfRefLink).size() == 0) // alone in new session
		{
			UINFO("Proximity detection by space disabled as if we force to have a global loop "
					"closure with previous map before doing proximity detections (%s=true).",
					Parameters::kRtabmapStartNewMapOnLoopClosure().c_str());
		}
		// 2 若图优化器被关闭（iterations()==0）
		else if(_graphOptimizer->iterations() == 0)
		{
			UWARN("Cannot do local loop closure detection in space if graph optimization is disabled!");
		}
		// 3 运动太小或太快时跳过检测
		else
		{
			// In localization mode, no need to check local loop
			// closures if we are already localized by a landmark.

			// don't do it if it is a small displacement unless the previous signature didn't have a loop closure
			// don't do it if there is a too fast movement
			// 移动速度适中并且（上一次更新没有发生定位回环或移动距离适中）
			if((!smallDisplacement || !localizationOnPreviousUpdate) && !tooFastMovement)
			{

				//============================================================
				// LOCAL LOOP CLOSURE SPACE
				// 进入真正的局部回环检测流程
				//============================================================

				//
				// 1) compare visually with nearest locations
				// 阶段 1：视觉近邻检测
				//
				UDEBUG("Proximity detection (local loop closure in SPACE using matching images, local radius=%fm)", _localRadius);
				// 寻找局部邻居节点，从 _optimizedPoses 中找出距离当前节点在 _localRadius 半径内的所有节点
				// nearestIds 的 value 是距离或排序指标
				std::map<int, float> nearestIds = graph::findNearestNodes(signature->id(), _optimizedPoses, _localRadius);
				UDEBUG("nearestIds=%d/%d", (int)nearestIds.size(), (int)_optimizedPoses.size());
				std::map<int, Transform> nearestPoses;
				std::multimap<int, int> links;
				// 受到 max depth 约束，则按图深度过滤
				if(_memory->isIncremental() && _proximityMaxGraphDepth>0)
				{
					// get bidirectional links 构建双向 links
					for(std::multimap<int, Link>::iterator iter=_constraints.begin(); iter!=_constraints.end(); ++iter)
					{
						if(uContains(_optimizedPoses, iter->second.from()) && uContains(_optimizedPoses, iter->second.to()))
						{
							links.insert(std::make_pair(iter->second.from(), iter->second.to()));
							links.insert(std::make_pair(iter->second.to(), iter->second.from())); // <->
						}
					}
				}
				// 根据深度过滤 nearestIds，形成 nearestPoses。
				for(std::map<int, float>::iterator iter=nearestIds.lower_bound(1); iter!=nearestIds.end(); ++iter)
				{
					if(_memory->getStMem().find(iter->first) == _memory->getStMem().end())
					{
						if(_memory->isIncremental() && _proximityMaxGraphDepth > 0)
						{
							std::list<std::pair<int, Transform> > path = graph::computePath(_optimizedPoses, links, signature->id(), iter->first);
							UDEBUG("Graph depth to %d = %ld", iter->first, path.size());
							if(!path.empty() && (int)path.size() <= _proximityMaxGraphDepth)
							{
								nearestPoses.insert(std::make_pair(iter->first, _optimizedPoses.at(iter->first)));
							}
						}
						else
						{
							nearestPoses.insert(std::make_pair(iter->first, _optimizedPoses.at(iter->first)));
						}
					}
				}
				UDEBUG("nearestPoses=%d", (int)nearestPoses.size());

				// segment poses by paths, only one detection per path, landmarks are ignored
				// 获取未排序的最近路径集合
				// key (int)：路径起点（某个候选近邻节点）
				// value (map<int, Transform>)：从当前节点出发 到该起点节点的一条拓扑路径 
				// map 中包含路径上的所有节点及其位姿
				std::map<int, std::map<int, Transform> > nearestPathsNotSorted = getPaths(nearestPoses, _optimizedPoses.at(signature->id()), _proximityMaxGraphDepth);
				UDEBUG("got %d paths", (int)nearestPathsNotSorted.size());
				// sort nearest paths by highest likelihood (if two have same likelihood, sort by id)
				// 按路径的 “最高似然” 排序，一条路径的优先级 = 路径上最“像当前节点”的那个节点
				std::map<NearestPathKey, std::map<int, Transform> > nearestPaths;
				Transform currentPoseInv = _optimizedPoses.at(signature->id()).inverse();
				for(std::map<int, std::map<int, Transform> >::const_iterator iter=nearestPathsNotSorted.begin();iter!=nearestPathsNotSorted.end(); ++iter)
				{
					const std::map<int, Transform> & path = iter->second;
					float highestLikelihood = 0.0f;
					int highestLikelihoodId = iter->first;
					float smallestDistanceSqr = -1;
					for(std::map<int, Transform>::const_iterator jter=path.begin(); jter!=path.end(); ++jter)
					{
						// 读取外观似然，从前面计算好的 likelihood 中读取：
						float v = uValue(likelihood, jter->first, 0.0f);
						// 计算空间距离
						// jter->second：路径节点在世界坐标系下的位姿
						// currentPoseInv * pose：得到该节点在当前坐标系下的相对位姿
						float distance = (currentPoseInv * jter->second).getNormSquared();
						// 选择路径“最优节点”
						// 外观似然更高 → 优先
						// 如果似然相同：空间距离更近 → 优先
						if(v > highestLikelihood || (v == highestLikelihood && (smallestDistanceSqr < 0 || distance < smallestDistanceSqr)))
						{
							highestLikelihood = v;
							highestLikelihoodId = jter->first;
							smallestDistanceSqr = distance;
						}
					}
					// 更新路径评分
					nearestPaths.insert(std::make_pair(NearestPathKey(highestLikelihood, highestLikelihoodId, smallestDistanceSqr), path));
				}
				UDEBUG("nearestPaths=%d proximityMaxPaths=%d", (int)nearestPaths.size(), _proximityMaxPaths);

				timeProximityBySpaceSearch = timer.ticks();
				ULOGGER_INFO("timeProximityBySpaceSearch=%fs", timeProximityBySpaceSearch);

				float proximityFilteringRadius = _proximityFilteringRadius;
				if(_maxLoopClosureDistance>0.0f && (proximityFilteringRadius <= 0.0f || _maxLoopClosureDistance<proximityFilteringRadius))
				{
					proximityFilteringRadius = _maxLoopClosureDistance;
				}
				// 对每条路径尝试视觉回环检测
				for(std::map<NearestPathKey, std::map<int, Transform> >::const_reverse_iterator iter=nearestPaths.rbegin();
					iter!=nearestPaths.rend() &&
					(_proximityMaxPaths <= 0 || localVisualPathsChecked < _proximityMaxPaths);
					++iter)
				{
					std::map<int, Transform> path = iter->second;
					UASSERT(path.size());

					//find the nearest pose on the path looking in the same direction
					// 找到这条路径上在同一方向上距离当前最近的节点
					path.insert(std::make_pair(signature->id(), _optimizedPoses.at(signature->id())));
					path = graph::findNearestPoses(signature->id(), path, _localRadius, _proximityAngle);
					//take the one with highest likelihood if not null
					// 找最近的候选节点 nearestId
					int nearestId = 0;
					if(iter->first.likelihood > 0.0f &&
					   path.find(iter->first.id)!=path.end())
					{
						nearestId = iter->first.id;
					}
					else
					{
						nearestId = rtabmap::graph::findNearestNode(path, _optimizedPoses.at(signature->id()));
					}

					if(nearestId > 0)
					{
						// nearest pose must not be linked to current location and enough close
						// 最近的候选节点 nearestId 未与当前节点建立链接，并且要求候选节点与当前节点距离足够近
						if(!signature->hasLink(nearestId) &&
							(proximityFilteringRadius <= 0.0f ||
							 _optimizedPoses.at(signature->id()).getDistanceSquared(_optimizedPoses.at(nearestId)) < proximityFilteringRadius*proximityFilteringRadius))
						{
							++localVisualPathsChecked;
							RegistrationInfo info;
							Transform guess;
							// 有 odom guess 则使用 odometry
							if(_proximityOdomGuess)
							{
								// Use odometry as guess so that correspondences can be computed by projection
								guess = _optimizedPoses.at(nearestId).inverse()*_optimizedPoses.at(signature->id());
							} //else: guess is null to make sure visual correspondences are globally computed
							// 通过视觉 computeTransform， transform = 从 nearestId → 当前节点 的位姿变换
							Transform transform = _memory->computeTransform(nearestId, signature->id(), guess, &info);
							if(!transform.isNull())
							{
								// 成功
								transform = transform.inverse();
								if(proximityFilteringRadius <= 0 || transform.getNormSquared() <= proximityFilteringRadius*proximityFilteringRadius)
								{
									UINFO("[Visual] Add local loop closure in SPACE (%d->%d) %s",
											signature->id(),
											nearestId,
											transform.prettyPrint().c_str());
									UASSERT(info.covariance.at<double>(0,0) > 0.0 && info.covariance.at<double>(5,5) > 0.0);

									//for statistics
									// 记录数据
									loopClosureVisualInliersMeanDist = info.inliersMeanDistance;
									loopClosureVisualInliersDistribution = info.inliersDistribution;

									++proximityDetectionsAddedVisually;
									lastProximitySpaceClosureId = nearestId;

									loopClosureVisualInliers = info.inliers;
									loopClosureVisualInliersRatio = info.inliersRatio;
									loopClosureVisualMatches = info.matches;

									cv::Mat information = getInformation(info.covariance);
									loopClosureLinearVariance = 1.0/information.at<double>(0,0);
									loopClosureAngularVariance = 1.0/information.at<double>(5,5);

									Link::Type type = Link::kLocalSpaceClosure;
									if(_loopClosureHypothesis.first>0 &&
										nearestIds.find(_loopClosureHypothesis.first)!=nearestIds.end())
									{
										// Avoid transform computation on the global loop closure if a visual proximity
										// one has been detected close (inside proximity radius) to that hypothesis.
										UDEBUG("Proximity detection on %d is close to loop closure %d, ignoring loop closure transform estimation...",
												nearestId, _loopClosureHypothesis.first);

										if(nearestId == _loopClosureHypothesis.first)
										{
											type = Link::kGlobalClosure;
											loopIdSuppressedByProximity = nearestId;
										}
										else if(loopIdSuppressedByProximity == 0)
										{
											loopIdSuppressedByProximity = nearestId;
										}
									}

									// 会被加入图中作为 Link::kLocalSpaceClosure
									_memory->addLink(Link(signature->id(), nearestId, type, transform, information));
									loopClosureLinksAdded.push_back(std::make_pair(signature->id(), nearestId));
								}
								else
								{
									UWARN("Ignoring local loop closure with %d because resulting "
										  "transform is too large!? (%fm > %fm)",
											nearestId, transform.getNorm(), proximityFilteringRadius);
								}
							}
						}
						else if(!signature->hasLink(nearestId) && proximityFilteringRadius>0.0f)
						{
							UDEBUG("Skipping path %d as most likely ID %d is too far %f > %f (%s)",
								iter->first.id,
								nearestId,
								_optimizedPoses.at(signature->id()).getDistance(_optimizedPoses.at(nearestId)),
								proximityFilteringRadius,
								Parameters::kRGBDProximityPathFilteringRadius().c_str());
						}
					}
				}

				timeProximityBySpaceVisualDetection = timer.ticks();
				ULOGGER_INFO("timeProximityBySpaceVisualDetection=%fs", timeProximityBySpaceVisualDetection);

				//
				// 2) compare locally with nearest locations by scan matching
				// 阶段 2：激光扫描近邻检测
				//
				UDEBUG("Proximity detection (local loop closure in SPACE with scan matching)");
				if( _proximityMaxNeighbors <= 0)
				{
					UDEBUG("Proximity by scan matching is disabled (%s=%d).", Parameters::kRGBDProximityPathMaxNeighbors().c_str(), _proximityMaxNeighbors);
				}
				// 只有当前节点含激光数据时才进行
				else if(!signature->sensorData().laserScanCompressed().isEmpty())
				{
					proximitySpacePaths = (int)nearestPaths.size();
					for(std::map<NearestPathKey, std::map<int, Transform> >::const_reverse_iterator iter=nearestPaths.rbegin();
							iter!=nearestPaths.rend() &&
							(_proximityMaxPaths <= 0 || localScanPathsChecked < _proximityMaxPaths);
							++iter)
					{
						std::map<int, Transform> path = iter->second; // should contain only nodes (no landmarks)
						UASSERT(path.size());
						UASSERT(path.begin()->first > 0);

						//find the nearest pose on the path
						// 查找路径中最近节点
						int nearestId = rtabmap::graph::findNearestNode(path, _optimizedPoses.at(signature->id()));
						UASSERT(nearestId > 0);
						//UDEBUG("Path %d (size=%d) distance=%fm", nearestId, (int)path.size(), _optimizedPoses.at(signature->id()).getDistance(_optimizedPoses.at(nearestId)));

						// nearest pose must be close and not linked to current location
						if(!signature->hasLink(nearestId))
						{
							if(_proximityMaxNeighbors < _proximityMaxGraphDepth || _proximityMaxGraphDepth == 0)
							{
								std::map<int, Transform> filteredPath;
								int i=0;
								std::map<int, Transform>::iterator nearestIdIter = path.find(nearestId);
								// "_proximityMaxNeighbors-1" means that if _proximityMaxNeighbors=1,
								// only nearest node on the path is taken (no scan merging). Useful to find
								// proximity detection between only 2 nodes with 360x360 lidar scans.
								// 根据配置限制邻居数量（scan merging）
								// 例如 _proximityMaxNeighbors = 1 表示不 merge 扫描，只匹配两个节点。
								for(std::map<int, Transform>::iterator iter=nearestIdIter; iter!=path.end() && i<=_proximityMaxNeighbors-1; ++iter, ++i)
								{
									filteredPath.insert(*iter);
								}
								i=1;
								for(std::map<int, Transform>::reverse_iterator iter(nearestIdIter); iter!=path.rend() && i<=_proximityMaxNeighbors-1; ++iter, ++i)
								{
									filteredPath.insert(*iter);
								}
								path = filteredPath;
							}

							// Assemble scans in the path and do ICP only
							// 使用局部地图执行 ICP 分两种模式
							std::map<int, Transform> optimizedLocalPath;
							if(_globalScanMap.empty() && _proximityRawPosesUsed)
							{
								//optimize the path's poses locally
								cv::Mat covariance;
								path = optimizeGraph(nearestId, uKeysSet(path), std::map<int, Transform>(), false, covariance);
								// transform local poses in optimized graph referential
								if(!uContains(path, nearestId))
								{
									UERROR("Proximity path not containing nearest ID ?! Skipping this path.");
									continue;
								}
								Transform t = _optimizedPoses.at(nearestId) * path.at(nearestId).inverse();

								for(std::map<int, Transform>::iterator jter=path.lower_bound(1); jter!=path.end(); ++jter)
								{
									optimizedLocalPath.insert(std::make_pair(jter->first, t * jter->second));
								}
							}
							else
							{
								optimizedLocalPath = path;
							}

							std::map<int, Transform> filteredPath;
							if(_globalScanMap.empty() && optimizedLocalPath.size() > 2 && proximityFilteringRadius > 0.0f)
							{
								// path filtering
								filteredPath = graph::radiusPosesFiltering(optimizedLocalPath, proximityFilteringRadius, 0, true);
								// make sure the current pose is still here
								filteredPath.insert(*optimizedLocalPath.find(nearestId));
							}
							else
							{
								filteredPath = optimizedLocalPath;
							}

							if(filteredPath.size() > 0)
							{
								// add current node to poses
								filteredPath.insert(std::make_pair(signature->id(), _optimizedPoses.at(signature->id())));
								//The nearest will be the reference for a loop closure transform
								if(signature->getLinks().find(nearestId) == signature->getLinks().end())
								{
									++localScanPathsChecked;
									RegistrationInfo info;
									Transform transform;
									bool icpMulti = true;
									// Multi-ICP 模式（无 global scan map）
									if(_globalScanMap.empty())
									{
										transform = _memory->computeIcpTransformMulti(signature->id(), nearestId, filteredPath, &info);
									}
									// Global scan map 模式（已有全局点云）
									else
									{
										UASSERT_MSG(_globalScanMapPoses.find(nearestId) != _globalScanMapPoses.end(), uFormat("Pose of %d not found in global scan poses", nearestId).c_str());
										icpMulti = false;
										// use pre-assembled scan map
										SensorData assembledData;
										assembledData.setId(nearestId);
										assembledData.setLaserScan(
												LaserScan(_globalScanMap,
													signature->sensorData().laserScanCompressed().maxPoints(),
													signature->sensorData().laserScanCompressed().rangeMax(),
													_globalScanMapPoses.at(nearestId).inverse() * (signature->sensorData().laserScanCompressed().is2d()?Transform(0,0,signature->sensorData().laserScanCompressed().localTransform().z(),0,0,0):Transform::getIdentity())));
										Signature nearestNode(assembledData);
										Transform guess = filteredPath.at(nearestId).inverse() * filteredPath.at(signature->id());
										transform = _memory->computeIcpTransform(nearestNode, *signature, guess, &info);
										if(!transform.isNull())
										{
											transform = transform.inverse();
										}
									}

									// 成功后添加回环边
									if(!transform.isNull())
									{
										UINFO("[Scan matching] Add local loop closure in SPACE (%d->%d) %s",
												signature->id(),
												nearestId,
												transform.prettyPrint().c_str());

										cv::Mat scanMatchingIds;
										if(_scanMatchingIdsSavedInLinks)
										{
											std::stringstream stream;
											stream << "SCANS:";
											for(std::map<int, Transform>::iterator iter=optimizedLocalPath.begin(); iter!=optimizedLocalPath.end(); ++iter)
											{
												if(iter != optimizedLocalPath.begin())
												{
													stream << ";";
												}
												stream << uNumber2Str(iter->first);
											}
											std::string scansStr = stream.str();
											scanMatchingIds = cv::Mat(1, int(scansStr.size()+1), CV_8SC1, (void *)scansStr.c_str());
											scanMatchingIds = compressData2(scanMatchingIds); // compressed
										}

										// set Identify covariance for laser scan matching only
										UASSERT(info.covariance.at<double>(0,0) > 0.0 && info.covariance.at<double>(5,5) > 0.0);
										_memory->addLink(Link(signature->id(), nearestId, Link::kLocalSpaceClosure, transform, getInformation(info.covariance)/_proximityMergedScanCovFactor, scanMatchingIds));
										loopClosureLinksAdded.push_back(std::make_pair(signature->id(), nearestId));

										if(icpMulti)
										{
											++proximityDetectionsAddedByICPMulti;
										}
										else
										{
											++proximityDetectionsAddedByICPGlobal;
										}

										// no local loop closure added visually
										if(proximityDetectionsAddedVisually == 0)
										{
											lastProximitySpaceClosureId = nearestId;
										}
									}
									else
									{
										UINFO("Local scan matching rejected: %s", info.rejectedMsg.c_str());
									}
									if(!_globalScanMap.empty())
									{
										break;
									}
								}
							}
						}
						else
						{
							//UDEBUG("Path %d ignored", nearestId);
						}
					}
				}
			}
		}
	}
	timeProximityBySpaceDetection = timer.ticks();
	ULOGGER_INFO("timeProximityBySpaceDetection=%fs", timeProximityBySpaceDetection);

	//=============================================================
	// Global loop closure detection
	// (updated: place this after retrieval to be sure that neighbors of the loop closure are in RAM)
	// 全局回环检测
	// 更新：将此操作放在检索之后，以确保闭环闭合的相邻元素位于 RAM 中。
	//=============================================================
	// 若存在回环候选（来自词袋）表示词袋候选（BOW matching）已经找到一个潜在回环节点 ID 为_loopClosureHypothesis.first
	if(_loopClosureHypothesis.first>0)
	{
		// 若未被局部 proximity 检测抑制，才能执行全局回环
		// 若上一阶段局部空间检测已经对该区域建立了回环，则避免重复计算全局回环。
		if(loopIdSuppressedByProximity==0)
		{
			//Compute transform if metric data are present
			// 计算全局回环变换（视觉匹配）
			Transform transform;
			RegistrationInfo info;
			info.covariance = cv::Mat::eye(6,6,CV_64FC1);
			if(_rgbdSlamMode)
			{
				// 从候选回环节点 → 当前节点；使用视觉特征匹配+RANSAC+PnP 或 DEPTH 方法
				// 若 _loopClosureIdentityGuess 为 true，则使用单位变换作为初始猜测
				transform = _memory->computeTransform(
						_loopClosureHypothesis.first,
						signature->id(),
						_loopClosureIdentityGuess?Transform::getIdentity():Transform(),
						&info);

				loopClosureVisualInliersMeanDist = info.inliersMeanDistance;
				loopClosureVisualInliersDistribution = info.inliersDistribution;

				loopClosureVisualInliers = info.inliers;
				loopClosureVisualInliersRatio = info.inliersRatio;
				loopClosureVisualMatches = info.matches;
				// 处理视觉匹配结果
				rejectedLoopClosure = transform.isNull();
				if(rejectedLoopClosure)
				{
					UWARN("Rejected loop closure %d -> %d: %s",
							_loopClosureHypothesis.first, signature->id(), info.rejectedMsg.c_str());
				}
				else if(_maxLoopClosureDistance>0.0f && transform.getNorm() > _maxLoopClosureDistance)
				{
					// 若 transform 距离过大 → 拒绝
					rejectedLoopClosure = true;
					UWARN("Rejected localization %d -> %d because distance to map (%fm) is over %s=%fm.",
							_loopClosureHypothesis.first, signature->id(), transform.getNorm(), Parameters::kRGBDMaxLoopClosureDistance().c_str(), _maxLoopClosureDistance);
				}
				else
				{
					// 若成功，则取 transform.inverse()
					transform = transform.inverse();
				}
			}
			// 若 transform 可用，添加全局回环边
			if(!rejectedLoopClosure)
			{
				// Make the new one the parent of the old one
				UASSERT(info.covariance.at<double>(0,0) > 0.0 && info.covariance.at<double>(5,5) > 0.0);
				
				loopClosureLinearVariance = uMax3(info.covariance.at<double>(0,0), info.covariance.at<double>(1,1)>=9999?0:info.covariance.at<double>(1,1), info.covariance.at<double>(2,2)>=9999?0:info.covariance.at<double>(2,2));
				loopClosureAngularVariance = uMax3(info.covariance.at<double>(3,3)>=9999?0:info.covariance.at<double>(3,3), info.covariance.at<double>(4,4)>=9999?0:info.covariance.at<double>(4,4), info.covariance.at<double>(5,5));
				cv::Mat information = getInformation(info.covariance);
				rejectedLoopClosure = !_memory->addLink(Link(signature->id(), _loopClosureHypothesis.first, Link::kGlobalClosure, transform, information));
				if(!rejectedLoopClosure)
				{
					loopClosureLinksAdded.push_back(std::make_pair(signature->id(), _loopClosureHypothesis.first));
				}
			}

			// 若任何错误发生 → 清除回环假设
			if(rejectedLoopClosure)
			{
				_loopClosureHypothesis.first = 0;
			}
		}
		else if(loopIdSuppressedByProximity != _loopClosureHypothesis.first)
		{
			// _loopClosureHypothesis.first 这是 词袋（BoW）全局回环检测 得到的候选节点 ID。表示 BoW 判断：当前节点可能与节点 150 是全局回环（视觉相似）。
			// loopIdSuppressedByProximity这是 局部空间接近（proximity by space）检测 设置的一个“抑制 ID”。表示 proximity 检测已经处理过 ID 附近的区域，不需要再对它做全局回环。
			// 这个被抑制的区域 不是当前候选的回环 ID 说明这是「另一个区域的抑制」，不属于当前候选
			// RTAB-Map 的策略是：若当前 proximity 抑制不是针对这个候选回环节点，则认为当前这个候选也不安全，于是直接清除回环候选
			// 因为 proximity 检测比 BoW 更可靠（基于几何/位姿/ICP）。
			// RTAB-Map 的原则是：只要 proximity 检测正在影响地图中的某一区域，BoW 的全局回环就不要“越权”到其他区域乱匹配
			_loopClosureHypothesis.first = 0;
		}
	}

	timeAddLoopClosureLink = timer.ticks();
	ULOGGER_INFO("timeAddLoopClosureLink=%fs", timeAddLoopClosureLink);

	//============================================================
	// Landmark
	// Landmark（地标）检测与处理
	// RTAB-Map 中 landmark（地标）是 非 SLAM 节点，通常是 ArUco 标签 / fiducial markers / AprilTag / QRCode / 手工地标。
	// Landmark 形成图优化中的特殊节点（anchor node），由相机观测得到位置约束。
	//============================================================
	std::map<int, std::set<int> > landmarksDetected; // <Landmark ID, list of nodes that saw this landmark>
	// 检测当前 signature 是否观测到地标
	if(!signature->getLandmarks().empty() && !_graphOptimizer->landmarksIgnored())
	{
		// odom cache 中是否已经包含全局闭环？
		bool hasGlobalLoopClosuresInOdomCache = !graph::filterLinks(_odomCacheConstraints, Link::kGlobalClosure, true).empty() || _loopClosureHypothesis.first != 0;
		UDEBUG("hasGlobalLoopClosuresInOdomCache=%d", hasGlobalLoopClosuresInOdomCache?1:0);
		// 遍历所有观测到的地标
		for(std::map<int, Link>::const_iterator iter=signature->getLandmarks().begin(); iter!=signature->getLandmarks().end(); ++iter)
		{
			// 判断这个地标是否曾被看到超过一次
			// 若该地标只出现过 1 次，则不能用于图优化（单点无法提供相对位姿）。
			if(uContains(_memory->getLandmarksIndex(), iter->first) &&
					_memory->getLandmarksIndex().find(iter->first)->second.size()>1)
			{
				// 判断是否允许在“定位模式”使用远距离地标
				// 在 定位模式（memory incremental= false） 中：如果没有全局回环，里程计（odom）是漂移的，远距离 landmark 观测可能会导致地图错位
				// 因此，如果 landmark 太远，则应忽略
				if(!_memory->isIncremental() &&          // In localization mode
					!hasGlobalLoopClosuresInOdomCache && // If there are global loop closures in odom cache, we can keep far landmarks
					_localRadius>0.0 &&
					iter->second.transform().getNormSquared() > _localRadius*_localRadius)
				{
					// Ignore landmark detections over local radius
					UWARN("Ignoring landmark %d for localization as it is too far (%fm > %s=%f) "
							"and odom cache doesn't contain global loop closure(s).",
							iter->first,
							iter->second.transform().getNorm(),
							Parameters::kRGBDLocalRadius().c_str(),
							_localRadius);
				}
				else
				{
					// 接受这个地标观测
					UINFO("Landmark %d observed again! Seen the first time by node %d.", -iter->first, *_memory->getLandmarksIndex().find(iter->first)->second.begin());
					// 记录该地标再次被看到 （在 graph optimization 中使用）
					landmarksDetected.insert(std::make_pair(iter->first, _memory->getLandmarksIndex().find(iter->first)->second));
					rejectedLoopClosure = false; // If it was true, it will be set back to false if landmarks are rejected on graph optimization
					// 将此 landmark 视为一个 loop closure（因为它将旧数据与当前节点连接）
					loopClosureLinksAdded.push_back(std::make_pair(signature->id(), iter->first));
				}
			}
		}
	}

	//============================================================
	// Add virtual links if a path is activated
	// 虚拟闭环
	// 这部分是 路径跟踪模式 / 导航模式（path following） 的专属逻辑。
	//============================================================
	// 若机器人正在执行路径（规划）：
	if(_path.size())
	{
		// Add a virtual loop closure link to keep the path linked to local map
		// 机器人当前位置并非路径上的当前目标节点 并且两者之间还没有图优化连接 则需要创建一个“虚拟闭环”。
		if( signature->id() != _path[_pathCurrentIndex].first &&
			!signature->hasLink(_path[_pathCurrentIndex].first))
		{
			UASSERT(uContains(_optimizedPoses, signature->id()));
			UASSERT_MSG(uContains(_optimizedPoses, _path[_pathCurrentIndex].first), uFormat("id=%d", _path[_pathCurrentIndex].first).c_str());
			// Virtual closure 是一种 弱约束(edge)：仅用于保持路径中的目标节点与图中当前节点连接 其约束权重（信息矩阵）非常小（方差很大）
			// 计算虚拟闭环的 transform
			Transform virtualLoop = _optimizedPoses.at(signature->id()).inverse() * _optimizedPoses.at(_path[_pathCurrentIndex].first);

			if(_localRadius == 0.0f || virtualLoop.getNorm() < _localRadius)
			{
				_memory->addLink(Link(signature->id(), _path[_pathCurrentIndex].first, Link::kVirtualClosure, virtualLoop, cv::Mat::eye(6,6,CV_64FC1)*0.01)); // set high variance
			}
			// 若虚拟闭环太远（违反 localRadius），则中止路径执行
			else
			{
				UERROR("Virtual link larger than local radius (%fm > %fm). Aborting the plan!",
						virtualLoop.getNorm(), _localRadius);
				this->clearPath(-1);
			}
		}
	}

	//============================================================
	// Optimize map graph
	// 图优化
	//============================================================
	// 阶段 0 — 变量/统计初始化
	float maxLinearError = 0.0f;
	float maxLinearErrorRatio = 0.0f;
	float maxAngularError = 0.0f;
	float maxAngularErrorRatio = 0.0f;
	double optimizationError = 0.0;
	int optimizationIterations = 0;
	Transform previousMapCorrection;
	bool delayedLocalization = false;
	int odomCacheProximityLinksCleared = 0;
	UDEBUG("RGB-D SLAM mode: %d", _rgbdSlamMode?1:0);
	UDEBUG("Incremental: %d", _memory->isIncremental());
	UDEBUG("Loop hyp: %d", _loopClosureHypothesis.first);
	UDEBUG("Last prox: %d", lastProximitySpaceClosureId);
	UDEBUG("Reduced ids: %d", (int)statistics_.reducedIds().size());
	UDEBUG("Has prior: %d (prior ignored=%d)", signature->hasLink(signature->id(), Link::kPosePrior)?1:0, _graphOptimizer->priorsIgnored()?1:0);
	UDEBUG("Has gravity: %d (sigma=%f, odomGravity=%d, refined=%d)", signature->hasLink(signature->id(), Link::kGravity)?1:0, _graphOptimizer->gravitySigma(), _memory->isOdomGravityUsed()?1:0, neighborLinkRefined?1:0);
	UDEBUG("Has virtual link: %d", (int)graph::filterLinks(signature->getLinks(), Link::kVirtualClosure, true).size());
	UDEBUG("Prox Time: %d", proximityDetectionsInTimeFound);
	UDEBUG("Landmarks: %d", (int)landmarksDetected.size());
	UDEBUG("Retrieved: %d", (int)signaturesRetrieved.size());
	UDEBUG("Not self ref links: %d", (int)graph::filterLinks(signature->getLinks(), Link::kSelfRefLink).size());

	// 阶段 1 — 决策：是否要做图优化
	if(_rgbdSlamMode // 仅在 RGB-D SLAM 模式时才做
		&&
		(_loopClosureHypothesis.first>0 || // 词袋找到 global loop
	     lastProximitySpaceClosureId>0 || // can be different map of the current one
	     statistics_.reducedIds().size() ||
		 (signature->hasLink(signature->id(), Link::kPosePrior) && !_graphOptimizer->priorsIgnored()) || // prior/gravity 链增加先验信息
		 (signature->hasLink(signature->id(), Link::kGravity) && _graphOptimizer->gravitySigma()>0.0f && (!_memory->isOdomGravityUsed() || neighborLinkRefined)) || // gravity edge
	     proximityDetectionsInTimeFound>0 || // 时间邻近的 proximity
		 !landmarksDetected.empty() || // 地标
		 signaturesRetrieved.size()) // 被检索回的签名（从数据库回来）会影响图,可以是不同地图的节点
		 &&
		 (_memory->isIncremental() ||
		  // In localization mode, the new node should be linked to another node or a landmark already in the working memory
		  // 若处于 localization 模式，只有当该新节点与工作内存已有连接（virtual link or landmark）时才继续。
		  // 也就是：定位模式下不随意对全图做优化，只有当新节点能连接到 working memory 才会调整。
		  graph::filterLinks(graph::filterLinks(signature->getLinks(), Link::kVirtualClosure), Link::kSelfRefLink).size() ||
		  !landmarksDetected.empty()))
	{
		UASSERT(uContains(_optimizedPoses, signature->id()));

		// used in localization mode: filter virtual links
		// 从当前 节点 中找出“定位约束”，排除Virtual类型
		// filterLinks 默认为从links中去除filterType；如果inverted为true，则是只保留filterType
		std::multimap<int, Link> localizationLinks = graph::filterLinks(signature->getLinks(), Link::kVirtualClosure);
		// 从 localizationLinks 排除 self-ref links（自己 → 自己 的 link）
		localizationLinks = graph::filterLinks(localizationLinks, Link::kSelfRefLink);
		// 定位模式下检测到landmarks时
		if(!landmarksDetected.empty() && !_memory->isIncremental())
		{
			for(std::map<int, std::set<int> >::iterator iter=landmarksDetected.begin(); iter!=landmarksDetected.end(); ++iter)
			{
				// iter->first：landmark 的 node id
				// 如果landmark 本身 已经在地图里
				if(_optimizedPoses.find(iter->first)!=_optimizedPoses.end())
				{
					UASSERT(uContains(signature->getLandmarks(), iter->first));
					// 把 landmark 约束“补充进定位约束集合”
					localizationLinks.insert(std::make_pair(iter->first, signature->getLandmarks().at(iter->first)));
				}
			}
		}

		// 检查：这些定位约束是否都在图里？
		bool allLocalizationLinksInGraph = !localizationLinks.empty();
		for(std::multimap<int, Link>::iterator iter=localizationLinks.begin(); iter!=localizationLinks.end(); ++iter)
		{
			if(!uContains(_optimizedPoses, iter->first))
			{
				allLocalizationLinksInGraph = false;
				break;
			}
		}

		// Note that in localization mode, we don't re-optimize the graph
		// if:
		//  1- there are no signatures retrieved,
		//  2- we are relocalizing on a node already in the optimized graph
		// 阶段 2 — 准备 localization 专用的约束（localizationLinks）与检查
		if(!_memory->isIncremental() &&
		   signaturesRetrieved.empty() &&
		   !localizationLinks.empty() &&
		   allLocalizationLinksInGraph)
		{
			// 阶段 3 — Localization 模式下的快速验证（使用 odom cache）
			bool rejectLocalization = _odomCachePoses.empty();
			// 如果 odom cache 为空，没有过去的 odom 信息，无法验证连续性，故暂拒绝。
			if(!_odomCachePoses.empty())
			{
				// Verify if the new localization is valid by checking if there is
				// not too much deformation using current odometry poses
				// This will also refine localization links

				// 初始化一个“可优化的子图”
				// _odomCachePoses：最近 odom 累积的一小段轨迹，是可调整的（变量节点）
				// _odomCacheConstraints：这些节点之间的里程计约束
				std::map<int, Transform> poses = _odomCachePoses;
				std::multimap<int, Link> constraints = _odomCacheConstraints;
				// add self referring links (e.g., gravity)
				// 筛选出 self-ref links（重力 / 先验/IMU 姿态约束）
				std::multimap<int, Link> selfLinks = graph::filterLinks(signature->getLinks(), Link::kSelfRefLink, true);
				// 如果 optimizer 忽略 prior，则删掉
				if(_graphOptimizer->priorsIgnored())
				{
					selfLinks = graph::filterLinks(selfLinks, Link::kPosePrior);
				}
				// 将self-ref links加入约束集合,保证姿态稳定性（尤其是 roll / pitch）
				constraints.insert(selfLinks.begin(), selfLinks.end());
				// 将 localizationLinks（来自当前 signature 指向图中节点或 landmark 的链接）加入 constraints。
				for(std::multimap<int, Link>::iterator iter=localizationLinks.begin(); iter!=localizationLinks.end(); ++iter)
				{
					constraints.insert(std::make_pair(iter->second.from(), iter->second));
				}
				// 把“地图节点”加入 poses，并用 PosePrior 固定住
				// prior 信息矩阵 ,_localizationPriorInf 通常很大 ⇒ 极小方差, 近似“硬约束”
				cv::Mat priorInfMat = cv::Mat::eye(6,6, CV_64FC1)*_localizationPriorInf;
				// 遍历所有约束，找出涉及的 map 节点
				for(std::multimap<int, Link>::iterator iter=constraints.begin(); iter!=constraints.end(); ++iter)
				{
					// _optimizedPoses 是 已经优化过的全局地图位姿 ：不允许被改, 只能当锚点
					std::map<int, Transform>::iterator iterPose = _optimizedPoses.find(iter->second.to());
					// 如果地图节点还不在 poses 中
					if(iterPose != _optimizedPoses.end() && poses.find(iterPose->first) == poses.end())
					{
						// 把该地图节点加入 poses, 但还不够 因为 optimizer 会把它当变量节点！
						poses.insert(*iterPose);
						// make the poses in the map fixed
						// 给该节点加一个 PosePrior（固定它）通过 self-prior 把地图节点“钉死”
						// 所以优化时只能动的是：odom cache 中的节点 和 当前帧位姿
						constraints.insert(std::make_pair(iterPose->first, Link(iterPose->first, iterPose->first, Link::kPosePrior, iterPose->second, priorInfMat)));
						UDEBUG("Constraint %d->%d: %s (type=%s, var=%f)", iterPose->first, iterPose->first, iterPose->second.prettyPrint().c_str(), Link::typeName(Link::kPosePrior).c_str(), 1./_localizationPriorInf);
					}
					UDEBUG("Constraint %d->%d: %s (type=%s, var = %f %f)", iter->second.from(), iter->second.to(), iter->second.transform().prettyPrint().c_str(), iter->second.typeName().c_str(), iter->second.transVariance(), iter->second.rotVariance());
				}

				// 从刚才构造的 poses / constraints 中，提取一个与当前帧连通的子图，
				// 利用 pose prior 把地图节点固定住，在这个子图上跑一次 graph optimization，
				// 得到用于定位的最优位姿（optPoses）和协方差
				std::map<int, Transform> posesOut;
				std::multimap<int, Link> edgeConstraintsOut;
				// 备份并强制启用 prior（非常关键）
				bool priorsIgnored = _graphOptimizer->priorsIgnored();
				UDEBUG("priorsIgnored was %s", priorsIgnored?"true":"false");
				// 前面专门给地图节点加了 kPosePrior, 如果 optimizer 当前配置是：Optimizer/PriorsIgnored=true
				// 那这些 prior 就会被直接忽略
				// 这里强制启用 prior 的目的, 确保地图节点被“钉死”，不参与优化
				_graphOptimizer->setPriorsIgnored(false);
				// 从 poses + constraints 构成的大图中, 只保留：与当前节点 signature->id() 连通的部分
				// 输出：posesOut, edgeConstraintsOut
				// 为什么一定要提取连通子图？
				// 防止优化无关节点（节省计算）;避免奇异矩阵（不连通图不可解）;保证 root 节点可达 
				// 尤其在定位模式：odom cache 很小; 地图节点很多 ;只优化“当前相关的那一小撮”
				_graphOptimizer->getConnectedGraph(signature->id(), poses, constraints, posesOut, edgeConstraintsOut);
				if(ULogger::level() == ULogger::kDebug)
				{
					// Debug：打印子图的初始位姿
					for(std::map<int, Transform>::iterator iter=posesOut.begin(); iter!=posesOut.end(); ++iter)
					{
						UDEBUG("Pose %d %s", iter->first, iter->second.prettyPrint().c_str());
					}
				}
				cv::Mat locOptCovariance;
				std::map<int, Transform> optPoses;
				// 判断子图是否合法（非常关键的安全检查）
				// posesOut.begin()->first 子图中最小的 node id
				// 判断条件：子图里 必须包含“地图节点”（老节点）。RTAB-Map 的 node id 是严格单调递增的 地图节点的 id 永远小于 odom cache 节点的 id
				// 否则说明：所有约束都只发生在 odom cache 内, 根本没有锚点, 优化结果无意义（整块漂）
				if(!posesOut.empty() && posesOut.begin()->first < _odomCachePoses.begin()->first)
				{
					// 在 2D SLAM：只优化 x / y / yaw, 保留 roll / pitch / z
					// 这才是 localizationLinks / landmark / prior 真正“生效”的地方
					// 地图节点：被 prior 固定, odom cache：被拉向地图
					// 在提取出的子图上做优化，得到 optPoses（优化后的位姿）和 locOptCovariance（定位的协方差,定位不确定度）。
					// 这里不会写回 _optimizedPoses, 不会改变地图, 只是临时结果
					optPoses = _graphOptimizer->optimize(posesOut.begin()->first, posesOut, edgeConstraintsOut, locOptCovariance, 0, &optimizationError, &optimizationIterations);
				}
				else
				{
					// 如果不满足，直接报错：
					UERROR("Invalid localization constraints");
				}
				// 恢复 optimizer 原配置
				_graphOptimizer->setPriorsIgnored(priorsIgnored);
				for(std::map<int, Transform>::iterator iter=optPoses.begin(); iter!=optPoses.end(); ++iter)
				{
					UDEBUG("Opt  %d %s", iter->first, iter->second.prettyPrint().c_str());
				}

				// 若 optPoses.empty() 则优化失败，rejectLocalization = true。
				if(optPoses.empty())
				{
					UWARN("Optimization failed, rejecting localization!");
					rejectLocalization = true;
				}
				else
				{
					UINFO("Compute max graph errors...");
					const Link * maxLinearLink = 0;
					const Link * maxAngularLink = 0;
					// 计算最大线性与角度误差及对应 link，这里会输出 maxLinearErrorRatio / maxAngularErrorRatio 等。
					graph::computeMaxGraphErrors(
							optPoses,
							edgeConstraintsOut,
							maxLinearErrorRatio,
							maxAngularErrorRatio,
							maxLinearError,
							maxAngularError,
							&maxLinearLink,
							&maxAngularLink,
							_graphOptimizer->isSlam2d());
					if(maxLinearLink == 0 && maxAngularLink==0)
					{
						UWARN("Could not compute graph errors! Rejecting localization!");
						rejectLocalization = true;
					}

					if(maxLinearLink)
					{
						UINFO("Max optimization linear error = %f m (link %d->%d, var=%f, ratio error/std=%f, thr=%f)",
								maxLinearError,
								maxLinearLink->from(),
								maxLinearLink->to(),
								maxLinearLink->transVariance(),
								maxLinearError/sqrt(maxLinearLink->transVariance()),
								_optimizationMaxError);
						if(_optimizationMaxError > 0.0f && maxLinearErrorRatio > _optimizationMaxError)
						{
							// 若任一超过 _optimizationMaxError（或极端异常且没有 robust 优化器），则拒绝本次定位
							UWARN("Rejecting localization (%d <-> %d) in this "
									"iteration because a wrong loop closure has been "
									"detected after graph optimization, resulting in "
									"a maximum graph error ratio of %f (edge %d->%d, type=%d, abs error=%f m, stddev=%f). The "
									"maximum error ratio parameter \"%s\" is %f of std deviation.",
									localizationLinks.rbegin()->second.from(),
									localizationLinks.rbegin()->second.to(),
									maxLinearErrorRatio,
									maxLinearLink->from(),
									maxLinearLink->to(),
									maxLinearLink->type(),
									maxLinearError,
									sqrt(maxLinearLink->transVariance()),
									Parameters::kRGBDOptimizeMaxError().c_str(),
									_optimizationMaxError);
							rejectLocalization = true;
						}
						else if(_optimizationMaxError == 0.0f && maxLinearErrorRatio>100 && !_graphOptimizer->isRobust())
						{
							UERROR("Huge optimization error detected!"
									"Linear error ratio of %f (edge %d->%d, type=%d, abs error=%f m, stddev=%f). You may consider "
									"enabling \"%s\" to reject those bad optimizations by setting it to a non null value!",
									maxLinearErrorRatio,
									maxLinearLink->from(),
									maxLinearLink->to(),
									maxLinearLink->type(),
									maxLinearError,
									sqrt(maxLinearLink->transVariance()),
									Parameters::kRGBDOptimizeMaxError().c_str());
						}
					}
					if(maxAngularLink)
					{
						UINFO("Max optimization angular error = %f deg (link %d->%d, var=%f, ratio error/std=%f, thr=%f)",
								maxAngularError*180.0f/CV_PI,
								maxAngularLink->from(),
								maxAngularLink->to(),
								maxAngularLink->rotVariance(),
								maxAngularError/sqrt(maxAngularLink->rotVariance()),
								_optimizationMaxError);
						if(_optimizationMaxError > 0.0f && maxAngularErrorRatio > _optimizationMaxError)
						{
							// 若任一超过 _optimizationMaxError（或极端异常且没有 robust 优化器），则拒绝本次定位
							UWARN("Rejecting localization (%d <-> %d) in this "
									"iteration because a wrong loop closure has been "
									"detected after graph optimization, resulting in "
									"a maximum graph error ratio of %f (edge %d->%d, type=%d, abs error=%f deg, stddev=%f). The "
									"maximum error ratio parameter \"%s\" is %f of std deviation.",
									localizationLinks.rbegin()->second.from(),
									localizationLinks.rbegin()->second.to(),
									maxAngularErrorRatio,
									maxAngularLink->from(),
									maxAngularLink->to(),
									maxAngularLink->type(),
									maxAngularError*180.0f/CV_PI,
									sqrt(maxAngularLink->rotVariance()),
									Parameters::kRGBDOptimizeMaxError().c_str(),
									_optimizationMaxError);
							rejectLocalization = true;
						}
						else if(_optimizationMaxError == 0.0f && maxAngularErrorRatio>100 && !_graphOptimizer->isRobust())
						{
							UERROR("Huge optimization error detected!"
									"Angular error ratio of %f (edge %d->%d, type=%d, abs error=%f m, stddev=%f). You may consider "
									"enabling \"%s\" to reject those bad optimizations by setting it to a non null value!",
									maxAngularErrorRatio,
									maxAngularLink->from(),
									maxAngularLink->to(),
									maxAngularLink->type(),
									maxAngularError*180.0f/CV_PI,
									sqrt(maxAngularLink->rotVariance()),
									Parameters::kRGBDOptimizeMaxError().c_str());
						}
					}
				}

				// localization 的回退尝试：去掉 local loop closures 再试一次
				// 如果初次尝试失败，但有全局回环或 landmarks 存在，
				// 则去掉本地空间闭环（proximity local closures），
				// 再重试优化（有时 local proximity closures 错误会和 global closures 冲突，去掉它们可能得到合理解）
				// 提高鲁棒性，避免本地的错误 ICP/visual proximity 覆盖掉全局正确回环。
				bool hasGlobalLoopClosuresOrLandmarks = false;
				if(rejectLocalization && 
					(_localizationSecondTryWithoutProximityLinks && !graph::filterLinks(constraints, Link::kLocalSpaceClosure, true).empty()))
				{
					// Let's try again without local loop closures
					localizationLinks = graph::filterLinks(localizationLinks, Link::kLocalSpaceClosure);
					constraints = graph::filterLinks(constraints, Link::kLocalSpaceClosure);
					for(std::multimap<int, Link>::iterator iter=constraints.begin(); iter!=constraints.end() && !hasGlobalLoopClosuresOrLandmarks; ++iter)
					{
						hasGlobalLoopClosuresOrLandmarks =
								iter->second.type() == Link::kGlobalClosure ||
								iter->second.type() == Link::kLandmark;
					}
					if(hasGlobalLoopClosuresOrLandmarks && !localizationLinks.empty())
					{
						rejectLocalization = false;
						UWARN("Global and loop closures seem not tallying together, try again to optimize without local loop closures...");
						priorsIgnored = _graphOptimizer->priorsIgnored();
						UDEBUG("priorsIgnored was %s", priorsIgnored?"true":"false");
						_graphOptimizer->setPriorsIgnored(false); //temporary set false to use priors above to fix nodes of the map
						// If slam2d: get connected graph while keeping original roll,pitch,z values.
						_graphOptimizer->getConnectedGraph(signature->id(), poses, constraints, posesOut, edgeConstraintsOut);
						optPoses.clear();
						if(!posesOut.empty() &&
						   posesOut.begin()->first < _odomCachePoses.begin()->first)
						{
							optPoses = _graphOptimizer->optimize(posesOut.begin()->first, posesOut, edgeConstraintsOut, locOptCovariance, 0, &optimizationError, &optimizationIterations);
						}
						else
						{
							UERROR("Invalid localization constraints");
						}
						_graphOptimizer->setPriorsIgnored(priorsIgnored); // set back
						for(std::map<int, Transform>::iterator iter=optPoses.begin(); iter!=optPoses.end(); ++iter)
						{
							UDEBUG("Opt2  %d %s", iter->first, iter->second.prettyPrint().c_str());
						}

						if(optPoses.empty())
						{
							UWARN("Optimization failed, rejecting localization!");
							rejectLocalization = true;
						}
						else
						{
							UINFO("Compute max graph errors...");
							const Link * maxLinearLink = 0;
							const Link * maxAngularLink = 0;
							graph::computeMaxGraphErrors(
									optPoses,
									edgeConstraintsOut,
									maxLinearErrorRatio,
									maxAngularErrorRatio,
									maxLinearError,
									maxAngularError,
									&maxLinearLink,
									&maxAngularLink,
									_graphOptimizer->isSlam2d());
							if(maxLinearLink == 0 && maxAngularLink==0)
							{
								UWARN("Could not compute graph errors! Rejecting localization!");
								rejectLocalization = true;
							}

							if(maxLinearLink)
							{
								UINFO("Max optimization linear error = %f m (link %d->%d, var=%f, ratio error/std=%f, thr=%f)",
										maxLinearError,
										maxLinearLink->from(),
										maxLinearLink->to(),
										maxLinearLink->transVariance(),
										maxLinearError/sqrt(maxLinearLink->transVariance()),
										_optimizationMaxError);
								if(_optimizationMaxError > 0.0f && maxLinearErrorRatio > _optimizationMaxError)
								{
									UWARN("Rejecting localization (%d <-> %d) in this "
											"iteration because a wrong loop closure has been "
											"detected after graph optimization, resulting in "
											"a maximum graph error ratio of %f (edge %d->%d, type=%d, abs error=%f m, stddev=%f). The "
											"maximum error ratio parameter \"%s\" is %f of std deviation.",
											localizationLinks.rbegin()->second.from(),
											localizationLinks.rbegin()->second.to(),
											maxLinearErrorRatio,
											maxLinearLink->from(),
											maxLinearLink->to(),
											maxLinearLink->type(),
											maxLinearError,
											sqrt(maxLinearLink->transVariance()),
											Parameters::kRGBDOptimizeMaxError().c_str(),
											_optimizationMaxError);
									rejectLocalization = true;
								}
								else if(_optimizationMaxError == 0.0f && maxLinearErrorRatio>100 && !_graphOptimizer->isRobust())
								{
									UERROR("Huge optimization error detected!"
											"Linear error ratio of %f (edge %d->%d, type=%d, abs error=%f m, stddev=%f). You may consider "
											"enabling \"%s\" to reject those bad optimizations by setting it to a non null value!",
											maxLinearErrorRatio,
											maxLinearLink->from(),
											maxLinearLink->to(),
											maxLinearLink->type(),
											maxLinearError,
											sqrt(maxLinearLink->transVariance()),
											Parameters::kRGBDOptimizeMaxError().c_str());
								}
							}
							if(maxAngularLink)
							{
								UINFO("Max optimization angular error = %f deg (link %d->%d, var=%f, ratio error/std=%f, thr=%f)",
										maxAngularError*180.0f/CV_PI,
										maxAngularLink->from(),
										maxAngularLink->to(),
										maxAngularLink->rotVariance(),
										maxAngularError/sqrt(maxAngularLink->rotVariance()),
										_optimizationMaxError);
								if(_optimizationMaxError > 0.0f && maxAngularErrorRatio > _optimizationMaxError)
								{
									UWARN("Rejecting localization (%d <-> %d) in this "
											"iteration because a wrong loop closure has been "
											"detected after graph optimization, resulting in "
											"a maximum graph error ratio of %f (edge %d->%d, type=%d, abs error=%f deg, stddev=%f). The "
											"maximum error ratio parameter \"%s\" is %f of std deviation.",
											localizationLinks.rbegin()->second.from(),
											localizationLinks.rbegin()->second.to(),
											maxAngularErrorRatio,
											maxAngularLink->from(),
											maxAngularLink->to(),
											maxAngularLink->type(),
											maxAngularError*180.0f/CV_PI,
											sqrt(maxAngularLink->rotVariance()),
											Parameters::kRGBDOptimizeMaxError().c_str(),
											_optimizationMaxError);
									rejectLocalization = true;
								}
								else if(_optimizationMaxError == 0.0f && maxAngularErrorRatio>100 && !_graphOptimizer->isRobust())
								{
									UERROR("Huge optimization error detected!"
											"Angular error ratio of %f (edge %d->%d, type=%d, abs error=%f m, stddev=%f). You may consider "
											"enabling \"%s\" to reject those bad optimizations by setting it to a non null value!",
											maxAngularErrorRatio,
											maxAngularLink->from(),
											maxAngularLink->to(),
											maxAngularLink->type(),
											maxAngularError*180.0f/CV_PI,
											sqrt(maxAngularLink->rotVariance()),
											Parameters::kRGBDOptimizeMaxError().c_str());
								}
							}
						}
					}
				}

				// localization 验证通过后的处理：接受/更新 odom cache 等
				// 将小子图优化结果写回 odom cache 与 optimized poses，使系统中对定位的认知一致，并为后续的路径/定位决策提供更稳健的数据。
				if(!rejectLocalization)
				{
					if(hasGlobalLoopClosuresOrLandmarks)
					{
						// We successfully optimize the graph without local loop closures,
						// clear them as some of them may be wrong.
						// 如果是在 remove local loop closures 后成功优化（hasGlobalLoopClosuresOrLandmarks），
						// 则把 odom cache 中的 local space closure links 清掉（它们可能是不可靠的）。
						size_t before = _odomCacheConstraints.size();
						_odomCacheConstraints = graph::filterLinks(_odomCacheConstraints, Link::kLocalSpaceClosure);
						if(before != _odomCacheConstraints.size())
						{
							UWARN("Successfully optimized without local loop closures! Clearing them from local odometry cache. %ld/%ld have been removed.",
									before - _odomCacheConstraints.size(), before);
						}
						else
						{
							UWARN("Successfully optimized without local loop closures!");
						}
						odomCacheProximityLinksCleared = before - _odomCacheConstraints.size();
					}

					// Count how many localization links are in the constraints
					bool hadAlreadyLocalizationLinks = false;
					for(std::multimap<int, Link>::iterator iter=_odomCacheConstraints.begin();
							iter!=_odomCacheConstraints.end(); ++iter)
					{
						if(iter->second.type() == Link::kGlobalClosure ||
						   iter->second.type() == Link::kLocalSpaceClosure ||
						   iter->second.type() == Link::kLocalTimeClosure ||
						   iter->second.type() == Link::kUserClosure ||
						   iter->second.type() == Link::kNeighborMerged ||
						   iter->second.type() == Link::kLandmark)
						{
							hadAlreadyLocalizationLinks = true;
							break;
						}
					}

					// update localization links
					UASSERT(uContains(optPoses, signature->id()));
					Transform newOptPoseInv = optPoses.at(signature->id()).inverse();
					// 把 localizationLinks 加入 _odomCacheConstraints
					for(std::multimap<int, Link>::iterator iter=localizationLinks.begin(); iter!=localizationLinks.end(); ++iter)
					{
						if(!_localizationSmoothing)
						{
							// Add original link without optimization
							UDEBUG("Adding new odom cache constraint %d->%d (%s)", 
								iter->second.from(), iter->second.to(), iter->second.transform().prettyPrint().c_str());
						}
						else
						{
							// Adjust with optimized poses, this will smooth the localization
							// 如果 _localizationSmoothing 启用，则根据 optPoses 调整这些 link 的变换，使得 localization 更平滑；
							UASSERT(uContains(optPoses, iter->first));
							Transform newT = newOptPoseInv * optPoses.at(iter->first);
							UDEBUG("Adjusted localization link %d->%d after optimization", iter->second.from(), iter->second.to());
							UDEBUG("from %s", iter->second.transform().prettyPrint().c_str());
							UDEBUG("  to %s", newT.prettyPrint().c_str());
							iter->second.setTransform(newT);

							// Update link in the referred signatures
							if(iter->first > 0)
								_memory->updateLink(iter->second, false);
						}
						
						_odomCacheConstraints.insert(std::make_pair(signature->id(), iter->second));
					}

					_odomCacheConstraints.insert(selfLinks.begin(), selfLinks.end());

					// At least 2 localizations at 2 different time required
					if(hadAlreadyLocalizationLinks || _maxOdomCacheSize == 0)
					{
						UINFO("Update localization");

						// update odomCachePoses with optimized poses (but make sure to put them back in odom frame)
						// 同时更新 _odomCachePoses 中所有 pose（用 mapToOdomCache * optPoses.at(iter->first) 把 map 优化后的 pose 转回 odom cache 坐标系）。
						Transform mapToOdomCache = signature->getPose() * newOptPoseInv;
						for(std::map<int, Transform>::iterator iter = _odomCachePoses.begin(); iter!=_odomCachePoses.end(); ++iter)
						{
							iter->second = mapToOdomCache * optPoses.at(iter->first);
						}

						if(_optimizeFromGraphEnd)
						{
							// 更新 _optimizedPoses.at(signature->id())
							// 若 _optimizeFromGraphEnd=true 可能按特定策略调整（保持 map correction 为 identity）
							// update all previous nodes
							// Normally _mapCorrection should be identity, but if _optimizeFromGraphEnd
							// parameters just changed state, we should put back all poses without map correction.
							Transform oldPose = _optimizedPoses.at(localizationLinks.rbegin()->first);
							Transform mapCorrectionInv = _mapCorrection.inverse();
							Transform u = signature->getPose() * localizationLinks.rbegin()->second.transform();
							if(_graphOptimizer->gravitySigma() > 0)
							{
                                // Adjust transform with gravity
                                Transform transform = localizationLinks.rbegin()->second.transform();
                                int loopId = localizationLinks.rbegin()->first;
                                int landmarkId = 0;
                                if(loopId<0)
                                {
                                    //For landmarks, use transform against other node looking the landmark
                                    // (because we don't assume that landmarks are aligned with gravity)
                                    landmarkId = loopId;
                                    UASSERT(landmarksDetected.find(landmarkId) != landmarksDetected.end() &&
                                            !landmarksDetected.at(landmarkId).empty());
                                    loopId = *landmarksDetected.at(landmarkId).begin();
                                }

                                const Signature * loopS = _memory->getSignature(loopId);
                                UASSERT(loopS !=0);
                                std::multimap<int, Link>::const_iterator iterGravityLoop = graph::findLink(loopS->getLinks(), loopS->id(), loopS->id(), false, Link::kGravity);
                                std::multimap<int, Link>::const_iterator iterGravitySign = graph::findLink(signature->getLinks(), signature->id(), signature->id(), false, Link::kGravity);
                                if(iterGravityLoop!=loopS->getLinks().end() &&
                                   iterGravitySign!=signature->getLinks().end())
                                {
                                    float roll,pitch,yaw;
                                    if(landmarkId < 0)
                                    {
                                        iterGravityLoop->second.transform().getEulerAngles(roll, pitch, yaw);
                                        Transform gravityCorr = Transform(_optimizedPoses.at(loopS->id()).x(),
                                                                              _optimizedPoses.at(loopS->id()).y(),
                                                                              _optimizedPoses.at(loopS->id()).z(),
                                                                              roll, pitch, _optimizedPoses.at(loopS->id()).theta()) * _optimizedPoses.at(loopS->id()).inverse();
                                        (gravityCorr * _optimizedPoses.at(landmarkId)).getEulerAngles(roll,pitch,yaw);
                                    }
                                    else
                                    {
                                        iterGravityLoop->second.transform().getEulerAngles(roll, pitch, yaw);
                                    }
                                    Transform targetRotation = iterGravitySign->second.transform().rotation()*transform.rotation();
                                    targetRotation = Transform(0,0,0,roll,pitch,targetRotation.theta());
                                    Transform error = transform.rotation().inverse() * iterGravitySign->second.transform().rotation().inverse() * targetRotation;
                                    transform *= error;
                                    u  = signature->getPose() * transform;
                                }
                                else if(iterGravityLoop!=loopS->getLinks().end() ||
                                        iterGravitySign!=signature->getLinks().end())
                                {
                                    UWARN("Gravity link not found for %d or %d, localization won't be corrected with gravity.", loopId, signature->id());
                                }
							}
							Transform up = u * oldPose.inverse();
							if(_graphOptimizer->isSlam2d())
							{
								// in case of 3d landmarks, transform constraint to 2D
								up.to3DoF();
							}
							for(std::map<int, Transform>::iterator iter=_optimizedPoses.begin(); iter!=_optimizedPoses.end(); ++iter)
							{
								iter->second = mapCorrectionInv * up * iter->second;
							}
							_optimizedPoses.at(signature->id()) = signature->getPose();
						}
						else
						{
							// 否则用 newPose 或基于 gravity 的校正后的 newPose 更新当前签名的 pose。
							Transform newPose = _optimizedPoses.at(localizationLinks.rbegin()->first) * localizationLinks.rbegin()->second.transform().inverse();
							UDEBUG("newPose=%s", newPose.prettyPrint().c_str());
							if(_graphOptimizer->isSlam2d() && signature->getPose().is3DoF())
							{
								// in case of 3d landmarks, transform constraint to 2D
								newPose = newPose.to3DoF();
								UDEBUG("newPose 2D=%s", newPose.prettyPrint().c_str());
							}
							else if(_graphOptimizer->gravitySigma() > 0)
							{
								// Adjust transform with gravity
								std::multimap<int, Link>::const_iterator iterGravitySign = graph::findLink(signature->getLinks(), signature->id(), signature->id(), false, Link::kGravity);
								if(iterGravitySign!=signature->getLinks().end())
								{
                                    Transform transform = localizationLinks.rbegin()->second.transform();
									float roll,pitch,yaw;
									float tmp1,tmp2;
									UDEBUG("Gravity link = %s", iterGravitySign->second.transform().prettyPrint().c_str());
                                    _optimizedPoses.at(localizationLinks.rbegin()->first).getEulerAngles(roll, pitch, yaw);
                                    Transform targetRotation = iterGravitySign->second.transform().rotation()*transform.rotation();
                                    targetRotation = Transform(0,0,0,roll,pitch,targetRotation.theta());
                                    Transform error = transform.rotation().inverse() * iterGravitySign->second.transform().rotation().inverse() * targetRotation;
                                    transform *= error;
                                    newPose = _optimizedPoses.at(localizationLinks.rbegin()->first) * transform.inverse();
                                    iterGravitySign->second.transform().getEulerAngles(roll, pitch, tmp1);
                                    newPose.getEulerAngles(tmp1, tmp2, yaw);
                                    newPose = Transform(newPose.x(), newPose.y(), newPose.z(), roll, pitch, yaw);
									UDEBUG("newPose gravity=%s", newPose.prettyPrint().c_str());
								}
								else if(iterGravitySign!=signature->getLinks().end())
								{
									UWARN("Gravity link not found for %d, localization won't be corrected with gravity.", signature->id());
								}
							}
							_optimizedPoses.at(signature->id()) = newPose;
						}
						_localizationCovariance = locOptCovariance.empty()?localizationLinks.rbegin()->second.infMatrix().inv():locOptCovariance;
					}
					else //如果只有一条 localization link（即还不够稳定），会触发“延迟定位”（delayedLocalization = true），拒绝立即采纳。
					{
						UWARN("Localization was good, but waiting for another one to be more accurate (%s>0)", Parameters::kRGBDMaxOdomCacheSize().c_str());
						delayedLocalization = true;
						rejectLocalization = true;
					}
				}
			}

			if(rejectLocalization)
			{
				// 放弃当前的 loop hypothesis（BoW）与 proximity closure；避免错误的回环污染图；在后续循环中会重新寻找或等待新的线索。
				_loopClosureHypothesis.first = 0;
				lastProximitySpaceClosureId = 0;
				rejectedLoopClosure = true;
			}
		}
		else
		{
			// 阶段 4 — 增量/常规模式：调用 optimizeCurrentMap
			// 如果不是上面那种快速 localization 流程，则进入常规优化路径：
			// 该分支实际做了图的全局/局部优化，并在优化后用误差检验以剔除坏回环，保护地图一致性。
			UINFO("Update map correction");
			std::map<int, Transform> poses = _optimizedPoses;  // 初始猜测

			// if _optimizeFromGraphEnd parameter just changed state, don't use optimized poses as guess
			// 若刚改变这个参数，则清除初始猜测避免用旧猜测产生偏差
			// _optimizeFromGraphEndChanged = false（默认）：从 root（最老节点）开始优化，true：固定历史节点， 只优化最近一段
			// OptimizeFromGraphEnd = false → 全图自由优化 | OptimizeFromGraphEnd = true → 历史节点被固定
			// 如果继续用旧 _optimizedPoses 作为初值，初值中： 历史节点可能已经被移动 新约束中： 它们被 强制固定 => 初值与约束语义冲突
			// 所以作者选择了最安全的方式：参数语义变化 → 清空初始猜测 → 让优化器从约束本身解
			if(_optimizeFromGraphEndChanged)
			{
				UWARN("Optimization: clearing guess poses as %s has changed state, now %s",
						Parameters::kRGBDOptimizeFromGraphEnd().c_str(), _optimizeFromGraphEnd?"true":"false");
				poses.clear();
				_optimizeFromGraphEndChanged = false;
			}

			// 从当前 graph 和 constraints 中提取出要优化的子图并调用图优化器得到 poses（新的 optimized poses）
			// 和 constraints（新的约束集合，可能包含新加入的 loop closures）。
			std::multimap<int, Link> constraints;
			cv::Mat covariance;
			// 这是一个 高层封装函数，内部会：
			// 1 从 Memory 中：取出当前可达子图, 收集所有约束（odom / loop / prior）
			// 2 根据参数：决定 root 节点, 是否 optimize from end
			// 3 调用：_graphOptimizer->optimize()
			// 4 返回：新的 poses（即 _optimizedPoses）和 constraints（最终用于优化的边）
			optimizeCurrentMap(signature->id(), false, poses, covariance, &constraints, &optimizationError, &optimizationIterations);

			// Check added loop closures have broken the graph
			// (in case of wrong loop closures).
			// 若 poses 为空 -> 优化失败
			bool updateConstraints = true;
			if(poses.empty())
			{
				UWARN("Graph optimization failed! Rejecting last loop closures added.");
				// 移除刚刚加入的 loopClosureLinksAdded（这些边被认为导致了错误）
				for(std::list<std::pair<int, int> >::iterator iter=loopClosureLinksAdded.begin(); iter!=loopClosureLinksAdded.end(); ++iter)
				{
					_memory->removeLink(iter->first, iter->second);
					UWARN("Loop closure %d->%d rejected!", iter->first, iter->second);
				}
				updateConstraints = false;
				_loopClosureHypothesis.first = 0;
				lastProximitySpaceClosureId = 0;
				rejectedLoopClosure = true;
			}
			// 若优化成功且处于增量模式且添加了回环：
			else if(_memory->isIncremental() &&
			  loopClosureLinksAdded.size() &&
			  optimizationIterations > 0 &&
			  constraints.size())
			{
				UINFO("Compute max graph errors...");
				const Link * maxLinearLink = 0;
				const Link * maxAngularLink = 0;
				// 计算最大图误差
				graph::computeMaxGraphErrors(
						poses,
						constraints,
						maxLinearErrorRatio,
						maxAngularErrorRatio,
						maxLinearError,
						maxAngularError,
						&maxLinearLink,
						&maxAngularLink);
				if(maxLinearLink == 0 && maxAngularLink==0)
				{
					UWARN("Could not compute graph errors! Wrong loop closures could be accepted!");
				}

				bool reject = false;
				// 若 maxLinearErrorRatio 或 maxAngularErrorRatio 超阈值 _optimizationMaxError，
				// 则把本次刚添加的所有 loop closures 全部移除（回退）并标记 rejectedLoopClosure = true
				if(maxLinearLink)
				{
					UINFO("Max optimization linear error = %f m (link %d->%d, var=%f, ratio error/std=%f)", maxLinearError, maxLinearLink->from(), maxLinearLink->to(), maxLinearLink->transVariance(), maxLinearError/sqrt(maxLinearLink->transVariance()));
					if(_optimizationMaxError > 0.0f && maxLinearErrorRatio > _optimizationMaxError)
					{
						UWARN("Rejecting all added loop closures (%d, first is %d <-> %d) in this "
							  "iteration because a wrong loop closure has been "
							  "detected after graph optimization, resulting in "
							  "a maximum graph error ratio of %f (edge %d->%d, type=%d, abs error=%f m, stddev=%f). The "
							  "maximum error ratio parameter \"%s\" is %f of std deviation.",
							  (int)loopClosureLinksAdded.size(),
							  loopClosureLinksAdded.front().first,
							  loopClosureLinksAdded.front().second,
							  maxLinearErrorRatio,
							  maxLinearLink->from(),
							  maxLinearLink->to(),
							  maxLinearLink->type(),
							  maxLinearError,
							  sqrt(maxLinearLink->transVariance()),
							  Parameters::kRGBDOptimizeMaxError().c_str(),
							  _optimizationMaxError);
						reject = true;
					}
					else if(_optimizationMaxError == 0.0f && maxLinearErrorRatio>100 && !_graphOptimizer->isRobust())
					{
						UERROR("Huge optimization error detected!"
								"Linear error ratio of %f (edge %d->%d, type=%d, abs error=%f m, stddev=%f). You may consider "
								"enabling \"%s\" to reject those bad optimizations by setting it to a non null value!",
								maxLinearErrorRatio,
								maxLinearLink->from(),
								maxLinearLink->to(),
								maxLinearLink->type(),
								maxLinearError,
								sqrt(maxLinearLink->transVariance()),
								Parameters::kRGBDOptimizeMaxError().c_str());
					}
				}
				if(maxAngularLink)
				{
					UINFO("Max optimization angular error = %f deg (link %d->%d, var=%f, ratio error/std=%f)", maxAngularError*180.0f/CV_PI, maxAngularLink->from(), maxAngularLink->to(), maxAngularLink->rotVariance(), maxAngularError/sqrt(maxAngularLink->rotVariance()));
					if(_optimizationMaxError > 0.0f && maxAngularErrorRatio > _optimizationMaxError)
					{
						UWARN("Rejecting all added loop closures (%d, first is %d <-> %d) in this "
							  "iteration because a wrong loop closure has been "
							  "detected after graph optimization, resulting in "
							  "a maximum graph error ratio of %f (edge %d->%d, type=%d, abs error=%f deg, stddev=%f). The "
							  "maximum error ratio parameter \"%s\" is %f of std deviation.",
							  (int)loopClosureLinksAdded.size(),
							  loopClosureLinksAdded.front().first,
							  loopClosureLinksAdded.front().second,
							  maxAngularErrorRatio,
							  maxAngularLink->from(),
							  maxAngularLink->to(),
							  maxAngularLink->type(),
							  maxAngularError*180.0f/CV_PI,
							  sqrt(maxAngularLink->rotVariance()),
							  Parameters::kRGBDOptimizeMaxError().c_str(),
							  _optimizationMaxError);
						reject = true;
					}
					else if(_optimizationMaxError == 0.0f && maxAngularErrorRatio>100 && !_graphOptimizer->isRobust())
					{
						UERROR("Huge optimization error detected!"
								"Angular error ratio of %f (edge %d->%d, type=%d, abs error=%f m, stddev=%f). You may consider "
								"enabling \"%s\" to reject those bad optimizations by setting it to a non null value!",
								maxAngularErrorRatio,
								maxAngularLink->from(),
								maxAngularLink->to(),
								maxAngularLink->type(),
								maxAngularError*180.0f/CV_PI,
								sqrt(maxAngularLink->rotVariance()),
								Parameters::kRGBDOptimizeMaxError().c_str());
					}
				}

				if(reject)
				{
					for(std::list<std::pair<int, int> >::iterator iter=loopClosureLinksAdded.begin(); iter!=loopClosureLinksAdded.end(); ++iter)
					{
						_memory->removeLink(iter->first, iter->second);
						UWARN("Loop closure %d->%d rejected!", iter->first, iter->second);
					}
					updateConstraints = false;
					_loopClosureHypothesis.first = 0;
					lastProximitySpaceClosureId = 0;
					rejectedLoopClosure = true;
				}
			}

			// 阶段 5 — 更新约束与优化结果写回（接受优化）
			if(updateConstraints)
			{
				UINFO("Updated local map (old size=%d, new size=%d)", (int)_optimizedPoses.size(), (int)poses.size());
				_optimizedPoses = poses;
				_constraints = constraints;
				_localizationCovariance = covariance;
			}
		}

		// Update map correction, it should be identify when optimizing from the last node
		// 阶段 6 — 地图校正（map correction）与统计更新
		// 保存并维护从 odom 到 map 的变换，使系统能够将传感器/里程计读数与地图约束对齐。
		UASSERT(_optimizedPoses.find(signature->id()) != _optimizedPoses.end());
		if(fakeOdom && _mapCorrectionBackup.isNull())
		{
			_mapCorrectionBackup = _mapCorrection;
		}
		previousMapCorrection = _mapCorrection;
		_mapCorrection = _optimizedPoses.at(signature->id()) * signature->getPose().inverse();
		// Update statistics about the closest node in the graph using the actual loop closure
		// 阶段 7 — 最后清理与时间统计
		if(!_memory->isIncremental() && !_lastLocalizationPose.isNull())
		{
			int closestNode = _loopClosureHypothesis.first>0?_loopClosureHypothesis.first:lastProximitySpaceClosureId;
			if(closestNode>0)
			{
				distanceToClosestNodeInTheGraph = _lastLocalizationPose.getDistance(_optimizedPoses.at(closestNode));
				UDEBUG("Last localization pose = %s, updated closest node=%d (%f m)", _lastLocalizationPose.prettyPrint().c_str(), closestNode, distanceToClosestNodeInTheGraph);
				angleToClosestNodeInTheGraph = _lastLocalizationPose.getAngle(_optimizedPoses.at(closestNode));
			}
		}
		_lastLocalizationPose = _optimizedPoses.at(signature->id()); // update
		if(_mapCorrection.getNormSquared() > 0.1f && _optimizeFromGraphEnd)
		{
			bool hasPrior = signature->hasLink(signature->id());
			if(!_graphOptimizer->priorsIgnored())
			{
				for(std::multimap<int, Link>::reverse_iterator iter=_constraints.rbegin(); !hasPrior && iter!=_constraints.rend(); ++iter)
				{
					if(iter->second.type() == Link::kPosePrior)
					{
						hasPrior = true;
					}
				}
			}
			if((!hasPrior || _graphOptimizer->priorsIgnored()) && _graphOptimizer->gravitySigma()==0.0f)
			{
				UERROR("Map correction should be identity when optimizing from the last node. T=%s", _mapCorrection.prettyPrint().c_str());
			}
		}
	}
	// newLocId 优先用 loop hypothesis（BoW 全局回环），其次用 proximity（local space closure）。
	// 若都没有，则有可能用 landmark 的原始观测者来更新 _lastLocalizationNodeId。
	int newLocId = _loopClosureHypothesis.first>0?_loopClosureHypothesis.first:lastProximitySpaceClosureId>0?lastProximitySpaceClosureId:0;
	// _lastLocalizationNodeId 用于统计/决策，例如判断与最近定位节点的距离、路径规划参考等。
	_lastLocalizationNodeId = newLocId!=0?newLocId:_lastLocalizationNodeId;
	if(newLocId==0 && !landmarksDetected.empty())
	{
		std::map<int, std::set<int> >::const_iterator iter = _memory->getLandmarksIndex().find(landmarksDetected.begin()->first);
		if(iter!=_memory->getLandmarksIndex().end())
		{
			if(iter->second.size() && *iter->second.begin()!=signature->id())
			{
				_lastLocalizationNodeId = *iter->second.begin();
			}
		}
	}

	timeMapOptimization = timer.ticks();
	ULOGGER_INFO("timeMapOptimization=%fs", timeMapOptimization);

	//============================================================
	// Prepare statistics
	//============================================================
	// Data used for the statistics event and for the log files
	int dictionarySize = 0;
	int refWordsCount = 0;
	int refUniqueWordsCount = 0;
	int lcHypothesisReactivated = 0;
	float rehearsalValue = uValue(statistics_.data(), Statistics::kMemoryRehearsal_sim(), 0.0f);
	int rehearsalMaxId = (int)uValue(statistics_.data(), Statistics::kMemoryRehearsal_merged(), 0.0f);
	sLoop = _memory->getSignature(_loopClosureHypothesis.first?_loopClosureHypothesis.first:lastProximitySpaceClosureId?lastProximitySpaceClosureId:_highestHypothesis.first);
	if(sLoop)
	{
		lcHypothesisReactivated = sLoop->isSaved()?1.0f:0.0f;
	}
	dictionarySize = (int)_memory->getVWDictionary()->getVisualWords().size();
	refWordsCount = (int)signature->getWords().size();
	refUniqueWordsCount = (int)uUniqueKeys(signature->getWords()).size();

	if(_graphOptimizer->isSlam2d() && _localizationCovariance.total() == 36)
	{
		// set very small
		_localizationCovariance.at<double>(2,2) = Registration::COVARIANCE_LINEAR_EPSILON;
		_localizationCovariance.at<double>(3,3) = Registration::COVARIANCE_ANGULAR_EPSILON;
		_localizationCovariance.at<double>(4,4) = Registration::COVARIANCE_ANGULAR_EPSILON;
	}

	// Posterior is empty if a bad signature is detected
	float vpHypothesis = posterior.size()?posterior.at(Memory::kIdVirtual):0.0f;
	int loopId = _loopClosureHypothesis.first>0?_loopClosureHypothesis.first:lastProximitySpaceClosureId;

	// prepare statistics
	if(_loopClosureHypothesis.first || _publishStats)
	{
		ULOGGER_INFO("sending stats...");
		statistics_.setRefImageId(_memory->getLastSignatureId()); // Use last id from Memory (in case of rehearsal)
		statistics_.setRefImageMapId(signature->mapId());
		statistics_.setStamp(data.stamp());
		if(_loopClosureHypothesis.first != Memory::kIdInvalid)
		{
			statistics_.setLoopClosureId(_loopClosureHypothesis.first);
			statistics_.setLoopClosureMapId(_memory->getMapId(_loopClosureHypothesis.first));
			ULOGGER_INFO("Loop closure detected! With id=%d", _loopClosureHypothesis.first);
		}

		if(_publishStats)
		{
			ULOGGER_INFO("send all stats...");
			statistics_.setExtended(1);

			statistics_.addStatistic(Statistics::kLoopAccepted_hypothesis_id(), _loopClosureHypothesis.first);
			statistics_.addStatistic(Statistics::kLoopSuppressed_hypothesis_id(), loopIdSuppressedByProximity);
			// 本次处理中的最高回环候选 <id, score>
			statistics_.addStatistic(Statistics::kLoopHighest_hypothesis_id(), _highestHypothesis.first);
			statistics_.addStatistic(Statistics::kLoopHighest_hypothesis_value(), _highestHypothesis.second);
			statistics_.addStatistic(Statistics::kLoopHypothesis_reactivated(), lcHypothesisReactivated);
			statistics_.addStatistic(Statistics::kLoopVp_hypothesis(), vpHypothesis);
			statistics_.addStatistic(Statistics::kLoopReactivate_id(), retrievalId);
			statistics_.addStatistic(Statistics::kLoopHypothesis_ratio(), hypothesisRatio);
			statistics_.addStatistic(Statistics::kLoopVisual_inliers(), loopClosureVisualInliers);
			statistics_.addStatistic(Statistics::kLoopVisual_inliers_ratio(), loopClosureVisualInliersRatio);
			statistics_.addStatistic(Statistics::kLoopVisual_matches(), loopClosureVisualMatches);
			statistics_.addStatistic(Statistics::kLoopLinear_variance(), loopClosureLinearVariance);
			statistics_.addStatistic(Statistics::kLoopAngular_variance(), loopClosureAngularVariance);
			statistics_.addStatistic(Statistics::kLoopLast_id(), _memory->getLastGlobalLoopClosureId());
			statistics_.addStatistic(Statistics::kLoopOptimization_max_error(), maxLinearError);
			statistics_.addStatistic(Statistics::kLoopOptimization_max_error_ratio(), maxLinearErrorRatio);
			statistics_.addStatistic(Statistics::kLoopOptimization_max_ang_error(), maxAngularError*180.0f/M_PI);
			statistics_.addStatistic(Statistics::kLoopOptimization_max_ang_error_ratio(), maxAngularErrorRatio);
			statistics_.addStatistic(Statistics::kLoopOptimization_error(), optimizationError);
			statistics_.addStatistic(Statistics::kLoopOptimization_iterations(), optimizationIterations);
			statistics_.addStatistic(Statistics::kLoopLandmark_detected(), landmarksDetected.empty()?0:-landmarksDetected.begin()->first);
			statistics_.addStatistic(Statistics::kLoopLandmark_detected_node_ref(), landmarksDetected.empty() || landmarksDetected.begin()->second.empty()?0:*landmarksDetected.begin()->second.begin());
			statistics_.addStatistic(Statistics::kLoopVisual_inliers_mean_dist(), loopClosureVisualInliersMeanDist);
			statistics_.addStatistic(Statistics::kLoopVisual_inliers_distribution(), loopClosureVisualInliersDistribution);

			statistics_.addStatistic(Statistics::kProximityTime_detections(), proximityDetectionsInTimeFound);
			statistics_.addStatistic(Statistics::kProximitySpace_detections_added_visually(), proximityDetectionsAddedVisually);
			statistics_.addStatistic(Statistics::kProximitySpace_detections_added_icp_multi(), proximityDetectionsAddedByICPMulti);
			statistics_.addStatistic(Statistics::kProximitySpace_detections_added_icp_global(), proximityDetectionsAddedByICPGlobal);
			statistics_.addStatistic(Statistics::kProximitySpace_paths(), proximitySpacePaths);
			statistics_.addStatistic(Statistics::kProximitySpace_visual_paths_checked(), localVisualPathsChecked);
			statistics_.addStatistic(Statistics::kProximitySpace_scan_paths_checked(), localScanPathsChecked);
			statistics_.addStatistic(Statistics::kProximitySpace_last_detection_id(), lastProximitySpaceClosureId);
			statistics_.setProximityDetectionId(lastProximitySpaceClosureId);
			statistics_.setProximityDetectionMapId(_memory->getMapId(lastProximitySpaceClosureId));

			statistics_.addStatistic(Statistics::kLoopId(), loopId);
			statistics_.addStatistic(Statistics::kLoopMap_id(), (loopId>0 && sLoop)?sLoop->mapId():-1);

			statistics_.addStatistic(Statistics::kLoopDistance_since_last_loc(), _distanceTravelledSinceLastLocalization);

			float x,y,z,roll,pitch,yaw;
			if(_loopClosureHypothesis.first || lastProximitySpaceClosureId || (!rejectedLoopClosure && !landmarksDetected.empty()))
			{
				if(_loopClosureHypothesis.first || lastProximitySpaceClosureId)
				{
					// Loop closure transform
					UASSERT(sLoop);
					std::multimap<int, Link>::const_iterator loopIter =  sLoop->getLinks().find(signature->id());
					UASSERT(loopIter!=sLoop->getLinks().end());
					UINFO("Set loop closure transform = %s", loopIter->second.transform().prettyPrint().c_str());
					statistics_.setLoopClosureTransform(loopIter->second.transform());

					statistics_.addStatistic(Statistics::kLoopVisual_words(), sLoop->getWords().size());

					// if ground truth exists, compute localization error
					if(!sLoop->getGroundTruthPose().isNull() && !signature->getGroundTruthPose().isNull())
					{
						Transform transformGT = sLoop->getGroundTruthPose().inverse() * signature->getGroundTruthPose();
						statistics_.addStatistic(Statistics::kGtLocalization_linear_error(), loopIter->second.transform().getDistance(transformGT));
						statistics_.addStatistic(Statistics::kGtLocalization_angular_error(), loopIter->second.transform().getAngle(transformGT)*180/M_PI);
					}
				}

				_distanceTravelledSinceLastLocalization = 0.0f;

				statistics_.addStatistic(Statistics::kLoopMapToOdom_norm(), _mapCorrection.getNorm());
				statistics_.addStatistic(Statistics::kLoopMapToOdom_angle(), _mapCorrection.getAngle(Transform::getIdentity())*180.0f/M_PI);
				_mapCorrection.getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
				statistics_.addStatistic(Statistics::kLoopMapToOdom_x(), x);
				statistics_.addStatistic(Statistics::kLoopMapToOdom_y(), y);
				statistics_.addStatistic(Statistics::kLoopMapToOdom_z(), z);
				statistics_.addStatistic(Statistics::kLoopMapToOdom_roll(),  roll*180.0f/M_PI);
				statistics_.addStatistic(Statistics::kLoopMapToOdom_pitch(),  pitch*180.0f/M_PI);
				statistics_.addStatistic(Statistics::kLoopMapToOdom_yaw(), yaw*180.0f/M_PI);

				// Odom correction (actual odometry pose change), ignore correction from first localization
				if(!odomPose.isNull() && !previousMapCorrection.isNull() && !previousMapCorrection.isIdentity())
				{
					Transform odomCorrection = (previousMapCorrection*odomPose).inverse()*_mapCorrection*odomPose;
					statistics_.addStatistic(Statistics::kLoopOdom_correction_norm(), odomCorrection.getNorm());
					statistics_.addStatistic(Statistics::kLoopOdom_correction_angle(), odomCorrection.getAngle(Transform::getIdentity())*180.0f/M_PI);
					odomCorrection.getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
					statistics_.addStatistic(Statistics::kLoopOdom_correction_x(), x);
					statistics_.addStatistic(Statistics::kLoopOdom_correction_y(), y);
					statistics_.addStatistic(Statistics::kLoopOdom_correction_z(), z);
					statistics_.addStatistic(Statistics::kLoopOdom_correction_roll(),  roll*180.0f/M_PI);
					statistics_.addStatistic(Statistics::kLoopOdom_correction_pitch(),  pitch*180.0f/M_PI);
					statistics_.addStatistic(Statistics::kLoopOdom_correction_yaw(), yaw*180.0f/M_PI);
				}
			}
			if(!_lastLocalizationPose.isNull() && !_lastLocalizationPose.isIdentity())
			{
				_lastLocalizationPose.getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
				statistics_.addStatistic(Statistics::kLoopMapToBase_x(), x);
				statistics_.addStatistic(Statistics::kLoopMapToBase_y(), y);
				statistics_.addStatistic(Statistics::kLoopMapToBase_z(), z);
				statistics_.addStatistic(Statistics::kLoopMapToBase_roll(),  roll*180.0f/M_PI);
				statistics_.addStatistic(Statistics::kLoopMapToBase_pitch(),  pitch*180.0f/M_PI);
				statistics_.addStatistic(Statistics::kLoopMapToBase_yaw(), yaw*180.0f/M_PI);
				UINFO("Localization pose = %s", _lastLocalizationPose.prettyPrint().c_str());

				if(_localizationSecondTryWithoutProximityLinks) {
					statistics_.addStatistic(Statistics::kLoopProximity_links_cleared(), (float)odomCacheProximityLinksCleared);
				}

				if(_localizationCovariance.total()==36)
				{
					double varLin = _graphOptimizer->isSlam2d()?
							std::max(_localizationCovariance.at<double>(0,0), _localizationCovariance.at<double>(1,1)):
							uMax3(_localizationCovariance.at<double>(0,0), _localizationCovariance.at<double>(1,1), _localizationCovariance.at<double>(2,2));

					statistics_.addStatistic(Statistics::kLoopMapToBase_lin_std(), sqrt(varLin));
					statistics_.addStatistic(Statistics::kLoopMapToBase_lin_var(), varLin);
				}
			}

			statistics_.setMapCorrection(_mapCorrection);
			UINFO("Set map correction = %s", _mapCorrection.prettyPrint().c_str());
			statistics_.setLocalizationCovariance(_localizationCovariance);

			// timings...
			statistics_.addStatistic(Statistics::kTimingMemory_update(), timeMemoryUpdate*1000);
			statistics_.addStatistic(Statistics::kTimingNeighbor_link_refining(), timeNeighborLinkRefining*1000);
			statistics_.addStatistic(Statistics::kTimingProximity_by_time(), timeProximityByTimeDetection*1000);
			statistics_.addStatistic(Statistics::kTimingProximity_by_space_search(), timeProximityBySpaceSearch*1000);
			statistics_.addStatistic(Statistics::kTimingProximity_by_space_visual(), timeProximityBySpaceVisualDetection*1000);
			statistics_.addStatistic(Statistics::kTimingProximity_by_space(), timeProximityBySpaceDetection*1000);
			statistics_.addStatistic(Statistics::kTimingReactivation(), timeReactivations*1000);
			statistics_.addStatistic(Statistics::kTimingAdd_loop_closure_link(), timeAddLoopClosureLink*1000);
			statistics_.addStatistic(Statistics::kTimingMap_optimization(), timeMapOptimization*1000);
			statistics_.addStatistic(Statistics::kTimingLikelihood_computation(), timeLikelihoodCalculation*1000);
			statistics_.addStatistic(Statistics::kTimingPosterior_computation(), timePosteriorCalculation*1000);
			statistics_.addStatistic(Statistics::kTimingHypotheses_creation(), timeHypothesesCreation*1000);
			statistics_.addStatistic(Statistics::kTimingHypotheses_validation(), timeHypothesesValidation*1000);
			statistics_.addStatistic(Statistics::kTimingCleaning_neighbors(), timeCleaningNeighbors*1000);

			// retrieval
			statistics_.addStatistic(Statistics::kMemorySignatures_retrieved(), (float)signaturesRetrieved.size());

			// Feature specific parameters
			statistics_.addStatistic(Statistics::kKeypointDictionary_size(), dictionarySize);
			statistics_.addStatistic(Statistics::kKeypointCurrent_frame(), refWordsCount);
			statistics_.addStatistic(Statistics::kKeypointIndexed_words(), _memory->getVWDictionary()->getIndexedWordsCount());
			statistics_.addStatistic(Statistics::kKeypointIndex_memory_usage(), _memory->getVWDictionary()->getIndexMemoryUsed());

			//Epipolar geometry constraint
			// 是否拒绝本次回环检测结果 加入统计数据 传送到窗口显示
			// 如果拒绝，窗口会显示 Loop hypothesis %1(highestHypothesisId) rejected!
			statistics_.addStatistic(Statistics::kLoopRejectedHypothesis(), rejectedLoopClosure?1.0f:0);

			statistics_.addStatistic(Statistics::kMemorySmall_movement(), smallDisplacement?1.0f:0);
			statistics_.addStatistic(Statistics::kMemoryDistance_travelled(), _distanceTravelled);
			statistics_.addStatistic(Statistics::kMemoryFast_movement(), tooFastMovement?1.0f:0);
			statistics_.addStatistic(Statistics::kMemoryNew_landmark(), addedNewLandmark?1.0f:0);

			if(distanceToClosestNodeInTheGraph>0.0)
			{
				statistics_.addStatistic(Statistics::kMemoryClosest_node_distance(), distanceToClosestNodeInTheGraph);
				statistics_.addStatistic(Statistics::kMemoryClosest_node_angle(), angleToClosestNodeInTheGraph);
			}

			if(_publishRAMUsage)
			{
				UTimer ramTimer;
				statistics_.addStatistic(Statistics::kMemoryRAM_usage(), UProcessInfo::getMemoryUsage()/(1024*1024));
				long estimatedMemoryUsage = sizeof(Rtabmap);
				estimatedMemoryUsage += _optimizedPoses.size() * (sizeof(int) + sizeof(Transform) + 12 * sizeof(float) + sizeof(std::map<int, Transform>::iterator)) + sizeof(std::map<int, Transform>);
				estimatedMemoryUsage += _constraints.size() * (sizeof(int) + sizeof(Transform) + 12 * sizeof(float) + sizeof(cv::Mat) + 36 * sizeof(double) + sizeof(std::map<int, Link>::iterator)) + sizeof(std::map<int, Link>);
				estimatedMemoryUsage += _memory->getMemoryUsed();
				estimatedMemoryUsage += _bayesFilter->getMemoryUsed();
				estimatedMemoryUsage += _parameters.size()*(sizeof(std::string)*2+sizeof(ParametersMap::iterator)) + sizeof(ParametersMap);
				statistics_.addStatistic(Statistics::kMemoryRAM_estimated(), (float)(estimatedMemoryUsage/(1024*1024)));//MB
				statistics_.addStatistic(Statistics::kTimingRAM_estimation(), ramTimer.ticks()*1000);
			}

			if(_publishLikelihood || _publishPdf)
			{
				// Child count by parent signature on the root of the memory ... for statistics
				statistics_.setWeights(weights);
				if(_publishPdf)
				{
					statistics_.setPosterior(posterior);
				}
				if(_publishLikelihood)
				{
					statistics_.setLikelihood(likelihood);
					statistics_.setRawLikelihood(rawLikelihood);
				}
			}

			statistics_.setLabels(_memory->getAllLabels());

			// Path
			if(_path.size())
			{
				statistics_.setLocalPath(this->getPathNextNodes());
				statistics_.setCurrentGoalId(this->getPathCurrentGoalId());
			}
		}

		timeStatsCreation = timer.ticks();
		ULOGGER_INFO("Time creating stats = %f...", timeStatsCreation);
	}

	Signature lastSignatureData = *signature;
	Transform lastSignatureLocalizedPose;
	if(_optimizedPoses.find(signature->id()) != _optimizedPoses.end())
	{
		lastSignatureLocalizedPose = _optimizedPoses.at(signature->id());
	}
	if(!_publishLastSignatureData)
	{
		lastSignatureData.sensorData().clearCompressedData();
		lastSignatureData.sensorData().clearRawData();
	}
	if(!_rawDataKept)
	{
		_memory->removeRawData(signature->id(), true, !_neighborLinkRefining && !_proximityBySpace, true);
	}

	// Localization mode and saving localization data: save odometry covariance in a prior link
	// so that DBReader can republish the covariance of localization data
	if(!_memory->isIncremental() && _memory->isLocalizationDataSaved() && !odomCovariance.empty())
	{
		_memory->addLink(Link(signature->id(), signature->id(), Link::kPosePrior, odomPose, odomCovariance.inv()));
	}

	// remove last signature if the memory is not incremental or is a bad signature (if bad signatures are ignored)
	int signatureRemoved = _memory->cleanup();
	if(signatureRemoved)
	{
		signaturesRemoved.push_back(signatureRemoved);
	}

	// If this option activated, add new nodes only if there are linked with a previous map.
	// Used when rtabmap is first started, it will wait a
	// global loop closure detection before starting the new map,
	// otherwise it deletes the current node.
	if(signatureRemoved != lastSignatureData.id())
	{
		if(_startNewMapOnLoopClosure &&
			_memory->isIncremental() &&              // only in mapping mode
			graph::filterLinks(signature->getLinks(), Link::kSelfRefLink).size() == 0 &&      // alone in the current map
			(landmarksDetected.empty() || rejectedLoopClosure) &&     // 没有检测到地标或回环（意味着无法与旧地图建立联系）
			_memory->getWorkingMem().size()>=2)      // 内存不为空（避免刚启动时把第一帧删了）
		{
			UWARN("Ignoring location %d because a global loop closure is required before starting a new map!",
					signature->id());
			signaturesRemoved.push_back(signature->id());
			_memory->deleteLocation(signature->id());
		}
		else if(_startNewMapOnGoodSignature &&
				(signature->getLandmarks().empty() && signature->isBadSignature()) &&  // 特征点太少，且没看到地标
				graph::filterLinks(signature->getLinks(), Link::kSelfRefLink).size() == 0)     // alone in the current map
		{
			UWARN("Ignoring location %d because a good signature (with enough features or with a landmark detected) is required before starting a new map!",
					signature->id());
			signaturesRemoved.push_back(signature->id());
			_memory->deleteLocation(signature->id());
		}
		else if((smallDisplacement || tooFastMovement) &&   // 1. 没怎么动 或者 动太快(模糊)
				_loopClosureHypothesis.first == 0 &&    // 2. 没检测到全局回环 (BoW)
				lastProximitySpaceClosureId == 0 &&     // 3. 没检测到局部回环 (Proximity)
				(rejectedLoopClosure || landmarksDetected.empty()) && // 4. 回环被拒绝 或者 没看到地标
				!addedNewLandmark) // 5. 没发现新地标
		{
			// Don't delete the location if a loop closure is detected
			UINFO("Ignoring location %d because the displacement is too small! (d=%f a=%f)",
				  signature->id(), _rgbdLinearUpdate, _rgbdAngularUpdate);
			// If there is a too small displacement, remove the node
			signaturesRemoved.push_back(signature->id());
			_memory->deleteLocation(signature->id());
		}
		else
		{
			// 将数据持久化到数据库中。
			// 场景：如果不满足上述任何删除条件。
			// 移动了足够距离（正常的关键帧）。
			// 或者，虽然没动，但是检测到了回环（为了闭环必须保存）。
			// 或者，这是地图的第一帧（且质量尚可）。
			_memory->saveLocationData(signature->id());
		}
	}
	else if(!_memory->isIncremental() &&
			(smallDisplacement || tooFastMovement) &&
			_loopClosureHypothesis.first == 0 &&
			lastProximitySpaceClosureId == 0 &&
			!delayedLocalization &&
			(rejectedLoopClosure || landmarksDetected.empty()))
	{
		_odomCachePoses.erase(signatureRemoved);
		for(std::multimap<int, Link>::iterator iter=_odomCacheConstraints.begin(); iter!=_odomCacheConstraints.end();)
		{
			if(iter->second.from() == signatureRemoved || iter->second.to() == signatureRemoved)
			{
				_odomCacheConstraints.erase(iter++);
			}
			else
			{
				++iter;
			}
		}
	}

	// Pass this point signature should not be used, since it could have been transferred...
	signature = 0;

	timeMemoryCleanup = timer.ticks();
	ULOGGER_INFO("timeMemoryCleanup = %fs... %d signatures removed", timeMemoryCleanup, (int)signaturesRemoved.size());



	//============================================================
	// TRANSFER
	//============================================================
	// TRANSFER 阶段是 RTAB-Map 的 内存管理核心机制：
	// 当处理一帧图像花费太多时间、或工作内存（Working Memory）变太大时， 
	// 系统会 把最老、最不常访问的节点移到长期记忆（Long-Term Memory, LTM）。
	//============================================================
	// 目的 1：保证系统实时性
	// 目的 2：保持地图局部区域完整
	double totalTime = timerTotal.ticks();
	ULOGGER_INFO("Total time processing = %fs...", totalTime);
	// 判断是否触发 TRANSFER（移出工作内存）
	// 若：当前帧处理时间 totalTime > maxAllowedTime，或者工作内存中的节点数量超过 _maxMemoryAllowed
	if((_maxTimeAllowed != 0 && totalTime*1000>_maxTimeAllowed) ||
		(_maxMemoryAllowed != 0 && _memory->getWorkingMem().size() > _maxMemoryAllowed))
	{
		ULOGGER_INFO("Removing old signatures because time limit is reached %f>%f or memory is reached %d>%d...", totalTime*1000, _maxTimeAllowed, _memory->getWorkingMem().size(), _maxMemoryAllowed);
		// 表示“这些节点免疫，不允许被转移出工作内存”。
		immunizedLocations.insert(_lastLocalizationNodeId); // keep the latest localization in working memory
		// _memory->forget() 会选择性地把最不重要的节点（低频、老）转移到 LTM, 返回删除的节点列表
		// 转移（forget）一些不重要的老节点到 LTM 以释放内存、提高实时性。
		std::list<int> transferred = _memory->forget(immunizedLocations);
		signaturesRemoved.insert(signaturesRemoved.end(), transferred.begin(), transferred.end());
		if(!_someNodesHaveBeenTransferred && transferred.size())
		{
			_someNodesHaveBeenTransferred = true; // only used to hide a warning on close nodes immunization
		}
	}
	_lastProcessTime = totalTime;

	// cleanup cached gps values
	// 更新 GPS 缓存
	for(std::list<int>::iterator iter=signaturesRemoved.begin(); iter!=signaturesRemoved.end() && _gpsGeocentricCache.size(); ++iter)
	{
		_gpsGeocentricCache.erase(*iter);
	}

	//Remove optimized poses from signatures transferred
	// 若删除节点，同时存在优化图（_optimizedPoses 或 _constraints），需要刷新局部地图
	if(signaturesRemoved.size() && (_optimizedPoses.size() || _constraints.size()))
	{
		//refresh the local map because some transferred nodes may have broken the tree
		// 选取局部地图重建的起点 ID
		int id = 0;
		if(!_memory->isIncremental() && (_lastLocalizationNodeId > 0 || _path.size()))
		{
			if(_path.size())
			{
				// priority on node on the path
				// 优先选择：path 路径节点（如果正在导航）
				UASSERT(_pathCurrentIndex < _path.size());
				UASSERT_MSG(uContains(_optimizedPoses, _path.at(_pathCurrentIndex).first), uFormat("id=%d", _path.at(_pathCurrentIndex).first).c_str());
				id = _path.at(_pathCurrentIndex).first;
				UDEBUG("Refresh local map from %d", id);
			}
			else
			{
				// 选择：最近的 localization 节点 _lastLocalizationNodeId
				if(uContains(_optimizedPoses, _lastLocalizationNodeId))
				{
					id = _lastLocalizationNodeId;
					UDEBUG("Refresh local map from %d", id);
				}
				else
				{
					// 否则：_lastLocalizationNodeId 无效 → 清零
					UDEBUG("Clearing _lastLocalizationNodeId(%d)", _lastLocalizationNodeId);
					_lastLocalizationNodeId = 0;
				}
			}
		}
		else if(_memory->isIncremental() &&
				_optimizedPoses.size() &&
				_memory->getLastWorkingSignature())
		{
			// 使用 last working signature → 当前图像对应的节点 ID。
			// 需要从一个“仍存在于工作内存中的有效节点”重新提取局部子图。
			id = _memory->getLastWorkingSignature()->id();
			UDEBUG("Refresh local map from %d", id);
		}
		UDEBUG("id=%d _optimizedPoses=%d", id, (int)_optimizedPoses.size());
		if(id > 0)
		{
			if(_lastLocalizationNodeId != 0)
			{
				_lastLocalizationNodeId = id;
			}
			UASSERT_MSG(_memory->getSignature(id) != 0, uFormat("id=%d", id).c_str());

			// 处理只删除了最后一个签名的情况
			if(signaturesRemoved.size() == 1 && signaturesRemoved.front() == lastSignatureData.id())
			{
				// 特殊优化：若被删除的是当前帧前的最后一个节点
				// 则只需从 _optimizedPoses 和 _constraints 中删除它，不需要重建整个局部地图
				int lastId = signaturesRemoved.front();
				UDEBUG("Detected that only last signature has been removed (lastId=%d)", lastId);
				_optimizedPoses.erase(lastId);
				for(std::multimap<int, Link>::iterator iter=_constraints.find(lastId); iter!=_constraints.end() && iter->first==lastId;++iter)
				{
					if(iter->second.to() != iter->second.from())
					{
						std::multimap<int, Link>::iterator jter = graph::findLink(_constraints, iter->second.to(), iter->second.from(), false);
						if(jter != _constraints.end())
						{
							_constraints.erase(jter);
						}
					}
				}
				_constraints.erase(lastId);
			}
			else
			{
				// 否则，一般情况：需要根据邻居关系刷新局部地图
				// 保持局部 SLAM 图只包含当前可用、未被转移出工作内存的节点。
				std::map<int, int> ids = _memory->getNeighborsId(id, 0, 0, true);
				for(std::map<int, Transform>::iterator iter=_optimizedPoses.begin(); iter!=_optimizedPoses.end();)
				{
					if(iter->first > 0 && !uContains(ids, iter->first))
					{
						UDEBUG("Removed %d from local map", iter->first);
						UASSERT(iter->first != _lastLocalizationNodeId);
						// 删除 _optimizedPoses 中不属于局部图的节点
						_optimizedPoses.erase(iter++);

						if(!_globalScanMap.empty())
						{
							UWARN("optimized poses have been modified, clearing global scan map...");
							_globalScanMap.clear();
							_globalScanMapPoses.clear();
						}
					}
					else
					{
						++iter;
					}
				}
				for(std::multimap<int, Link>::iterator iter=_constraints.begin(); iter!=_constraints.end();)
				{
					if(iter->first > 0 && (!uContains(ids, iter->second.from()) || !uContains(ids, iter->second.to())))
					{
						// 删除 _constraints 中不属于局部图的边
						_constraints.erase(iter++);
					}
					else
					{
						++iter;
					}
				}
			}
		}
		else
		{
			// 若无法选定有效 node id → 清空优化图
			if(!_optimizedPoses.empty())
				UDEBUG("Optimized poses cleared!");
			_optimizedPoses.clear();
			_constraints.clear();
		}
	}
	// just some verifications to make sure that planning path is still in the local map!
	// 确保路径规划上的“当前点”和“目标点”仍在局部地图中。
	// 如果不在，路径应该被视为无效（ASSERT 会直接提示问题）。
	if(_path.size())
	{
		UASSERT(_pathCurrentIndex < _path.size());
		UASSERT(_pathGoalIndex < _path.size());
		UASSERT_MSG(uContains(_optimizedPoses, _path.at(_pathCurrentIndex).first), uFormat("local map size=%d, id=%d", (int)_optimizedPoses.size(), _path.at(_pathCurrentIndex).first).c_str());
		UASSERT_MSG(uContains(_optimizedPoses, _path.at(_pathGoalIndex).first), uFormat("local map size=%d, id=%d", (int)_optimizedPoses.size(), _path.at(_pathGoalIndex).first).c_str());
	}


	timeRealTimeLimitReachedProcess = timer.ticks();
	ULOGGER_INFO("Time limit reached processing = %f...", timeRealTimeLimitReachedProcess);

	//==============================================================
	// Finalize statistics and log files
	//==============================================================
	int localGraphSize = 0;
	if(_publishStats)
	{
		statistics_.addStatistic(Statistics::kTimingStatistics_creation(), timeStatsCreation*1000);
		statistics_.addStatistic(Statistics::kTimingTotal(), totalTime*1000);
		statistics_.addStatistic(Statistics::kTimingForgetting(), timeRealTimeLimitReachedProcess*1000);
		statistics_.addStatistic(Statistics::kTimingJoining_trash(), timeJoiningTrash*1000);
		statistics_.addStatistic(Statistics::kTimingEmptying_trash(), timeEmptyingTrash*1000);
		statistics_.addStatistic(Statistics::kTimingMemory_cleanup(), timeMemoryCleanup*1000);

		// Transfer
		statistics_.addStatistic(Statistics::kMemorySignatures_removed(), signaturesRemoved.size());
		statistics_.addStatistic(Statistics::kMemoryImmunized_globally(), immunizedGlobally);
		statistics_.addStatistic(Statistics::kMemoryImmunized_locally(), immunizedLocally);
		statistics_.addStatistic(Statistics::kMemoryImmunized_locally_max(), maxLocalLocationsImmunized);

		// place after transfer because the memory/local graph may have changed
		statistics_.addStatistic(Statistics::kMemoryWorking_memory_size(), _memory->getWorkingMem().size());
		statistics_.addStatistic(Statistics::kMemoryShort_time_memory_size(), _memory->getStMem().size());
		statistics_.addStatistic(Statistics::kMemoryDatabase_memory_used(), _memory->getDatabaseMemoryUsed());

		// Set local graph
		std::map<int, Transform> poses;
		std::multimap<int, Link> constraints;
		if(!_rgbdSlamMode)
		{
			UDEBUG("");
			// no optimization on appearance-only mode, create a local graph
			std::map<int, int> ids = _memory->getNeighborsId(lastSignatureData.id(), 0, 0, true);
			_memory->getMetricConstraints(uKeysSet(ids), poses, constraints, false);
		}
		else // RGBD-SLAM mode
		{
			poses = _optimizedPoses;
			constraints = _constraints;
		}
		UINFO("Adding data %d [%d] (rgb/left=%d depth/right=%d)", lastSignatureData.id(), lastSignatureData.mapId(), lastSignatureData.sensorData().imageRaw().empty()?0:1, lastSignatureData.sensorData().depthOrRightRaw().empty()?0:1);
			statistics_.addSignatureData(lastSignatureData);

		if(_nodesToRepublish.size())
		{
			std::multimap<int, int> missingIds;

			// priority to loopId
			int tmpId = loopId>0?loopId:_highestHypothesis.first;
			if(tmpId>0 && _nodesToRepublish.find(tmpId) != _nodesToRepublish.end())
			{
				missingIds.insert(std::make_pair(-1, tmpId));
			}

			if(!_lastLocalizationPose.isNull())
			{
				// Republish data from closest nodes of the current localization
				std::map<int, Transform> nodesOnly(_optimizedPoses.lower_bound(1), _optimizedPoses.end());
				int id = rtabmap::graph::findNearestNode(nodesOnly, _lastLocalizationPose);
				if(id>0)
				{
					std::map<int, int> ids = _memory->getNeighborsId(id, 0, 0, true, false, true);
					for(std::map<int, int>::iterator iter=ids.begin(); iter!=ids.end(); ++iter)
					{
						if(iter->first != loopId &&
								_nodesToRepublish.find(iter->first) != _nodesToRepublish.end())
						{
							missingIds.insert(std::make_pair(iter->second, iter->first));
						}
					}

					if(_nodesToRepublish.size() != missingIds.size())
					{
						// remove requested nodes not anymore in the graph
						for(std::set<int>::iterator iter=_nodesToRepublish.begin(); iter!=_nodesToRepublish.end();)
						{
							if(ids.find(*iter) == ids.end())
							{
								iter = _nodesToRepublish.erase(iter);
							}
							else
							{
								++iter;
							}
						}
					}
				}
			}

			int loaded = 0;
			std::stringstream stream;
			for(std::multimap<int, int>::iterator iter=missingIds.begin(); iter!=missingIds.end() && loaded<(int)_maxRepublished; ++iter)
			{
				statistics_.addSignatureData(getSignatureCopy(iter->second, true, true, true, true, true, true));
				_nodesToRepublish.erase(iter->second);
				++loaded;
				stream << iter->second << " ";
			}
			if(loaded)
			{
				UWARN("Republishing data of requested node(s) %s(%s=%d)",
						stream.str().c_str(),
						Parameters::kRtabmapMaxRepublished().c_str(),
						_maxRepublished);
			}
		}

		UDEBUG("");
		localGraphSize = (int)poses.size();
		if(!lastSignatureLocalizedPose.isNull())
		{
			poses.insert(std::make_pair(lastSignatureData.id(), lastSignatureLocalizedPose)); // in case we are in localization
		}
		statistics_.setPoses(poses);
		statistics_.setConstraints(constraints);

		statistics_.addStatistic(Statistics::kMemoryLocal_graph_size(), poses.size());

		statistics_.setOdomCachePoses(_odomCachePoses);
		statistics_.setOdomCacheConstraints(_odomCacheConstraints);
		statistics_.addStatistic(Statistics::kMemoryOdom_cache_poses(), _odomCachePoses.size());
		statistics_.addStatistic(Statistics::kMemoryOdom_cache_links(), _odomCacheConstraints.size());

		if(_computeRMSE && _memory->getGroundTruths().size())
		{
			UDEBUG("Computing RMSE...");
			float translational_rmse = 0.0f;
			float translational_mean = 0.0f;
			float translational_median = 0.0f;
			float translational_std = 0.0f;
			float translational_min = 0.0f;
			float translational_max = 0.0f;
			float rotational_rmse = 0.0f;
			float rotational_mean = 0.0f;
			float rotational_median = 0.0f;
			float rotational_std = 0.0f;
			float rotational_min = 0.0f;
			float rotational_max = 0.0f;

			graph::calcRMSE(
					_memory->getGroundTruths(),
					poses,
					translational_rmse,
					translational_mean,
					translational_median,
					translational_std,
					translational_min,
					translational_max,
					rotational_rmse,
					rotational_mean,
					rotational_median,
					rotational_std,
					rotational_min,
					rotational_max);

			statistics_.addStatistic(Statistics::kGtTranslational_rmse(), translational_rmse);
			statistics_.addStatistic(Statistics::kGtTranslational_mean(), translational_mean);
			statistics_.addStatistic(Statistics::kGtTranslational_median(), translational_median);
			statistics_.addStatistic(Statistics::kGtTranslational_std(), translational_std);
			statistics_.addStatistic(Statistics::kGtTranslational_min(), translational_min);
			statistics_.addStatistic(Statistics::kGtTranslational_max(), translational_max);
			statistics_.addStatistic(Statistics::kGtRotational_rmse(), rotational_rmse);
			statistics_.addStatistic(Statistics::kGtRotational_mean(), rotational_mean);
			statistics_.addStatistic(Statistics::kGtRotational_median(), rotational_median);
			statistics_.addStatistic(Statistics::kGtRotational_std(), rotational_std);
			statistics_.addStatistic(Statistics::kGtRotational_min(), rotational_min);
			statistics_.addStatistic(Statistics::kGtRotational_max(), rotational_max);
			UDEBUG("Computing RMSE...done!");
		}

		std::vector<int> ids;
		ids.reserve(_memory->getWorkingMem().size() + _memory->getStMem().size());
		for(std::set<int>::const_iterator iter=_memory->getStMem().begin(); iter!=_memory->getStMem().end(); ++iter)
		{
			ids.push_back(*iter);
		}
		for(std::map<int, double>::const_iterator iter=_memory->getWorkingMem().lower_bound(0); iter!=_memory->getWorkingMem().end(); ++iter)
		{
			ids.push_back(iter->first);
		}
		statistics_.setWmState(ids);
		UDEBUG("wmState=%d", (int)ids.size());
	}

	//Save statistics to database
	if(_memory->isIncremental() || _memory->isLocalizationDataSaved())
	{
		_memory->saveStatistics(statistics_, _saveWMState);
	}

	//Start trashing
	UDEBUG("Empty trash...");
	_memory->emptyTrash();

	// Log info...
	// TODO : use a specific class which will handle the RtabmapEvent
	if(_foutFloat && _foutInt)
	{
		UDEBUG("Logging...");
		std::string logF = uFormat("%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
									totalTime,
									timeMemoryUpdate,
									timeReactivations,
									timeLikelihoodCalculation,
									timePosteriorCalculation,
									timeHypothesesCreation,
									timeHypothesesValidation,
									timeRealTimeLimitReachedProcess,
									timeStatsCreation,
									_highestHypothesis.second,
									0.0f,
									0.0f,
									0.0f,
									0.0f,
									0.0f,
									vpHypothesis,
									timeJoiningTrash,
									rehearsalValue,
									timeEmptyingTrash,
									timeRetrievalDbAccess,
									timeAddLoopClosureLink,
									timeMemoryCleanup,
									timeNeighborLinkRefining,
									timeProximityByTimeDetection,
									timeProximityBySpaceDetection,
									timeMapOptimization);
		std::string logI = uFormat("%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
									_loopClosureHypothesis.first,
									_highestHypothesis.first,
									(int)signaturesRemoved.size(),
									0,
									refWordsCount,
									dictionarySize,
									int(_memory->getWorkingMem().size()),
									rejectedLoopClosure?1:0,
									0,
									0,
									int(signaturesRetrieved.size()),
									lcHypothesisReactivated,
									refUniqueWordsCount,
									retrievalId,
									0,
									rehearsalMaxId,
									rehearsalMaxId>0?1:0,
									localGraphSize,
									data.id(),
									_memory->getVWDictionary()->getIndexedWordsCount(),
									_memory->getVWDictionary()->getIndexMemoryUsed());
		if(_statisticLogsBufferedInRAM)
		{
			_bufferedLogsF.push_back(logF);
			_bufferedLogsI.push_back(logI);
		}
		else
		{
			if(_foutFloat)
			{
				fprintf(_foutFloat, "%s", logF.c_str());
			}
			if(_foutInt)
			{
				fprintf(_foutInt, "%s", logI.c_str());
			}
		}
		UINFO("Time logging = %f...", timer.ticks());
		//ULogger::flush();
	}
	timeFinalizingStatistics = timer.ticks();
	UDEBUG("End process, timeFinalizingStatistics=%fs", timeFinalizingStatistics);
	if(_publishStats)
	{
		statistics_.addStatistic(Statistics::kTimingFinalizing_statistics(), timeFinalizingStatistics*1000);
	}

	return true;
}

// SETTERS
void Rtabmap::setTimeThreshold(float maxTimeAllowed)
{
	//must be positive, 0 mean inf time allowed (no time limit)
	_maxTimeAllowed = maxTimeAllowed;
	if(_maxTimeAllowed < 0)
	{
		ULOGGER_WARN("maxTimeAllowed < 0, then setting it to 0 (inf).");
		_maxTimeAllowed = 0;
	}
	else if(_maxTimeAllowed > 0.0f && _maxTimeAllowed < 1.0f)
	{
		ULOGGER_WARN("Time threshold set to %fms, it is not in seconds!", _maxTimeAllowed);
	}
}
void Rtabmap::setMemoryThreshold(int maxMemoryAllowed)
{
	//must be positive, 0 mean inf memory allowed (no memory limit)
	_maxMemoryAllowed = maxMemoryAllowed;
	if(maxMemoryAllowed < 0)
	{
		ULOGGER_WARN("maxMemoryAllowed < 0, then setting it to 0 (inf).");
		_maxMemoryAllowed = 0;
	}
}

void Rtabmap::setWorkingDirectory(std::string path)
{
	path = uReplaceChar(path, '~', UDirectory::homeDir());
	if(!path.empty() && UDirectory::exists(path))
	{
		ULOGGER_DEBUG("Comparing new working directory path \"%s\" with \"%s\"", path.c_str(), _wDir.c_str());
		if(path.compare(_wDir) != 0)
		{
			if (_foutFloat || _foutInt)
			{
				UWARN("Working directory has been changed from \"%s\" with \"%s\", new log files will be created.",
					path.c_str(), _wDir.c_str());
			}
			_wDir = path;
			setupLogFiles();
		}
	}
	else if(path.empty())
	{
		_wDir.clear();
		setupLogFiles();
	}
	else
	{
		ULOGGER_ERROR("Directory \"%s\" doesn't exist!", path.c_str());
	}
}

void Rtabmap::rejectLastLoopClosure()
{
	if(_memory && _memory->getStMem().find(getLastLocationId())!=_memory->getStMem().end())
	{
		std::multimap<int, Link> links = _memory->getLinks(getLastLocationId(), false);
		bool linksRemoved = false;
		for(std::multimap<int, Link>::iterator iter = links.begin(); iter!=links.end(); ++iter)
		{
			if(iter->second.type() == Link::kGlobalClosure ||
				iter->second.type() == Link::kLocalSpaceClosure ||
				iter->second.type() == Link::kLocalTimeClosure ||
				iter->second.type() == Link::kUserClosure)
			{
				_memory->removeLink(iter->second.from(), iter->second.to());
				std::multimap<int, Link>::iterator jter = graph::findLink(_constraints, iter->second.from(), iter->second.to(), true);
				if(jter!=_constraints.end())
				{
					_constraints.erase(jter);
					// second time if link is also inverted
					jter = graph::findLink(_constraints, iter->second.from(), iter->second.to(), true);
					if(jter!=_constraints.end())
					{
						_constraints.erase(jter);
					}
				}
				linksRemoved = true;
			}
		}

		if(linksRemoved)
		{
			_loopClosureHypothesis.first = 0;

			// we have to re-optimize the graph without the rejected links
			if(_memory->isIncremental() && _optimizedPoses.size())
			{
				UINFO("Update graph");
				std::map<int, Transform> poses = _optimizedPoses;
				std::multimap<int, Link> constraints;
				cv::Mat covariance;
				optimizeCurrentMap(getLastLocationId(), false, poses, covariance, &constraints);

				if(poses.empty())
				{
					UWARN("Graph optimization failed after removing loop closure links from last location!");
				}
				else
				{
					UINFO("Updated local map (old size=%d, new size=%d)", (int)_optimizedPoses.size(), (int)poses.size());
					_optimizedPoses = poses;
					_constraints = constraints;
					_mapCorrection = _optimizedPoses.at(_memory->getLastWorkingSignature()->id()) * _memory->getLastWorkingSignature()->getPose().inverse();
				}
			}
		}
	}
}

void Rtabmap::deleteLastLocation()
{
	if(_memory && _memory->getStMem().size())
	{
		int lastId = *_memory->getStMem().rbegin();
		_memory->deleteLocation(lastId);
		// we have to re-optimize the graph without the deleted location
		if(_memory->isIncremental() && _optimizedPoses.size())
		{
			UINFO("Update graph");
			_optimizedPoses.erase(lastId);
			std::map<int, Transform> poses = _optimizedPoses;
			//remove all constraints with last localization id
			for(std::multimap<int, Link>::iterator iter=_constraints.begin(); iter!=_constraints.end();)
			{
				if(iter->second.from() == lastId || iter->second.to() == lastId)
				{
					_constraints.erase(iter++);
				}
				else
				{
					++iter;
				}
			}

			if(poses.empty())
			{
				_mapCorrection.setIdentity();
			}
			else
			{
				std::multimap<int, Link> constraints;
				cv::Mat covariance;
				optimizeCurrentMap(_memory->getLastWorkingSignature()->id(), false, poses, covariance, &constraints);

				if(poses.empty())
				{
					UWARN("Graph optimization failed after deleting the last location!");
				}
				else
				{
					_optimizedPoses = poses;
					_constraints = constraints;
					_mapCorrection = _optimizedPoses.at(_memory->getLastWorkingSignature()->id()) * _memory->getLastWorkingSignature()->getPose().inverse();
				}
			}
		}
	}
}

void Rtabmap::setOptimizedPoses(const std::map<int, Transform> & poses, const std::multimap<int, Link> & constraints)
{
	_optimizedPoses = poses;
    _constraints = constraints;
}

void Rtabmap::dumpData() const
{
	UDEBUG("");
	if(_memory)
	{
		if(this->getWorkingDir().empty())
		{
			UERROR("Working directory not set.");
		}
		else
		{
			_memory->dumpMemory(this->getWorkingDir());
		}
	}
}

// fromId must be in _memory and in _optimizedPoses
// Get poses in front of the robot, return optimized poses
std::map<int, Transform> Rtabmap::getForwardWMPoses(
		int fromId,
		int maxNearestNeighbors,
		float radius,
		int maxGraphDepth // 0 means ignore
		) const
{
	std::map<int, Transform> poses;
	if(_memory && fromId > 0)
	{
		UDEBUG("");
		const Signature * fromS = _memory->getSignature(fromId);
		UASSERT(fromS != 0);
		UASSERT(_optimizedPoses.find(fromId) != _optimizedPoses.end());

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud->resize(_optimizedPoses.size());
		std::vector<int> ids(_optimizedPoses.size());
		int oi = 0;
		const std::set<int> & stm = _memory->getStMem();
		//get distances
		std::map<int, float> foundIds;
		if(_memory->isIncremental())
		{
			foundIds = _memory->getNeighborsIdRadius(fromId, radius, _optimizedPoses, maxGraphDepth);
		}
		else
		{
			foundIds = graph::findNearestNodes(fromId, _optimizedPoses, radius);
		}

		float radiusSqrd = radius * radius;
		for(std::map<int, Transform>::const_iterator iter = _optimizedPoses.begin(); iter!=_optimizedPoses.end(); ++iter)
		{
			if(iter->first != fromId)
			{
				if(stm.find(iter->first) == stm.end() &&
				   uContains(foundIds, iter->first) &&
				   (radiusSqrd==0 || foundIds.at(iter->first) <= radiusSqrd))
				{
					(*cloud)[oi] = pcl::PointXYZ(iter->second.x(), iter->second.y(), iter->second.z());
					ids[oi++] = iter->first;
				}
			}
		}

		cloud->resize(oi);
		ids.resize(oi);

		Transform fromT = _optimizedPoses.at(fromId);

		if(cloud->size())
		{
			//if(cloud->size())
			//{
			//	pcl::io::savePCDFile("radiusPoses.pcd", *cloud);
			//	UWARN("Saved radiusPoses.pcd");
			//}

			//filter poses in front of the fromId
			float x,y,z, roll,pitch,yaw;
			fromT.getTranslationAndEulerAngles(x,y,z, roll,pitch,yaw);

			pcl::CropBox<pcl::PointXYZ> cropbox;
			cropbox.setInputCloud(cloud);
			cropbox.setMin(Eigen::Vector4f(-1, -radius, -999999, 0));
			cropbox.setMax(Eigen::Vector4f(radius, radius, 999999, 0));
			cropbox.setRotation(Eigen::Vector3f(roll, pitch, yaw));
			cropbox.setTranslation(Eigen::Vector3f(x, y, z));
			cropbox.setRotation(Eigen::Vector3f(roll,pitch,yaw));
			pcl::IndicesPtr indices(new std::vector<int>());
			cropbox.filter(*indices);

			//if(indices->size())
			//{
			//	pcl::io::savePCDFile("radiusCrop.pcd", *cloud, *indices);
			//	UWARN("Saved radiusCrop.pcd");
			//}

			if(indices->size())
			{
				pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZ>);
				kdTree->setInputCloud(cloud, indices);
				std::vector<int> ind;
				std::vector<float> dist;
				pcl::PointXYZ pt(fromT.x(), fromT.y(), fromT.z());
				kdTree->radiusSearch(pt, radius, ind, dist, maxNearestNeighbors);
				//pcl::PointCloud<pcl::PointXYZ> inliers;
				for(unsigned int i=0; i<ind.size(); ++i)
				{
					if(ind[i] >=0)
					{
						Transform tmp = _optimizedPoses.find(ids[ind[i]])->second;
						//inliers.push_back(pcl::PointXYZ(tmp.x(), tmp.y(), tmp.z()));
						UDEBUG("Inlier %d: %s", ids[ind[i]], tmp.prettyPrint().c_str());
						poses.insert(std::make_pair(ids[ind[i]], tmp));
					}
				}

				//if(inliers.size())
				//{
				//	pcl::io::savePCDFile("radiusInliers.pcd", inliers);
				//}
				//if(nearestId >0)
				//{
				//	pcl::PointCloud<pcl::PointXYZ> c;
				//	Transform ct = _optimizedPoses.find(nearestId)->second;
				//	c.push_back(pcl::PointXYZ(ct.x(), ct.y(), ct.z()));
				//	pcl::io::savePCDFile("radiusNearestPt.pcd", c);
				//}
			}
		}
	}
	return poses;
}

std::map<int, std::map<int, Transform> > Rtabmap::getPaths(const std::map<int, Transform> & posesIn, const Transform & target, int maxGraphDepth) const
{
	std::map<int, std::map<int, Transform> > paths;
	std::set<int> nodesSet;
	std::map<int, Transform> poses;
	for(std::map<int, Transform>::const_iterator iter=posesIn.lower_bound(1); iter!=posesIn.end(); ++iter)
	{
		nodesSet.insert(iter->first);
		poses.insert(*iter);
	}
	if(_memory && nodesSet.size() && !target.isNull())
	{
		double e0=0,e1=0,e2=0,e3=0,e4=0;
		UTimer t;
		e0 = t.ticks();
		// Segment poses connected only by neighbor links
		while(poses.size())
		{
			std::map<int, Transform> path;
			// select nearest pose and iterate neighbors from there
			int nearestId = rtabmap::graph::findNearestNode(poses, target);

			e1+=t.ticks();

			if(nearestId == 0)
			{
				UWARN("Nearest id of %s in %d poses is 0 !? Returning empty path.", target.prettyPrint().c_str(), (int)poses.size());
				break;
			}
			std::map<int, int> ids = _memory->getNeighborsId(nearestId, maxGraphDepth, 0, true, true, true, true, nodesSet);

			e2+=t.ticks();

			for(std::map<int, int>::iterator iter=ids.begin(); iter!=ids.end(); ++iter)
			{
				std::map<int, Transform>::iterator jter = poses.find(iter->first);
				if(jter != poses.end())
				{
					bool valid = path.empty();
					if(!valid)
					{
						// make sure it has a neighbor added to path
						std::multimap<int, Link> links = _memory->getNeighborLinks(iter->first);
						for(std::multimap<int, Link>::iterator kter=links.begin(); kter!=links.end() && !valid; ++kter)
						{
							valid = path.find(kter->first) != path.end();
						}
					}

					if(valid)
					{
						//UDEBUG("%d <- %d", nearestId, jter->first);
						path.insert(*jter);
						poses.erase(jter);
					}
				}
			}

			e3+=t.ticks();

			if (path.size())
			{
				if (maxGraphDepth > 0 && !_memory->isGraphReduced() && (int)path.size() > maxGraphDepth * 2 + 1)
				{
					UWARN("%s=Off but path(%d) > maxGraphDepth(%d)*2+1, nearestId=%d ids=%d. Is reduce graph activated before?",
						Parameters::kMemReduceGraph().c_str(), (int)path.size(), maxGraphDepth, nearestId, (int)ids.size());
				}
				paths.insert(std::make_pair(nearestId, path));
			}
			else
			{
				UWARN("path.size()=0!? nearestId=%d ids=%d, aborting...", nearestId, (int)ids.size());
				break;
			}

			e4+=t.ticks();
		}
		UDEBUG("e0=%fs e1=%fs e2=%fs e3=%fs e4=%fs", e0, e1, e2, e3, e4);
	}
	return paths;
}

/**
 * 以当前节点 id 为参考，找出与之连通的整张（或局部）图， 
 * 根据 OptimizeFromGraphEnd 决定 root， 
 * 调用 optimizeGraph() 做真正的 pose graph optimization， 
 * 并把结果写回 optimizedPoses
 */
void Rtabmap::optimizeCurrentMap(
		int id,
		bool lookInDatabase,
		std::map<int, Transform> & optimizedPoses,
		cv::Mat & covariance,
		std::multimap<int, Link> * constraints,
		double * error,
		int * iterationsDone) const
{
	//Optimize the map
	UINFO("Optimize map: around location %d (lookInDatabase=%s)", id, lookInDatabase?"true":"false");
	// 必须有 memory（否则没图）
	// id > 0：RTAB-Map 的合法 node id 从 1 开始; id<=0 表示无效输入
	if(_memory && id > 0)
	{
		UTimer timer;
		// 获取“连通图节点集合”（非常关键）
		// id: 起始节点 
		// 0: depth = 0 → 不限深度 
		// lookInDatabase?-1:0: 是否从数据库加载老节点 
		// true: include loop closures 
		// false: 不只取邻接节点
		std::map<int, int> ids = _memory->getNeighborsId(id, 0, lookInDatabase?-1:0, true, false);
		// _optimizeFromGraphEnd == false 从图的起点优化 全图可动
		// _optimizeFromGraphEnd == true root 仍是传入的 id（通常是最新节点） 历史节点近似固定
		if(!_optimizeFromGraphEnd && ids.size() > 1)
		{
			// ids.begin()->first 是 最小 id = 最老的节点
			id = ids.begin()->first;
		}
		UINFO("get %d ids time %f s", (int)ids.size(), timer.ticks());

		// 真正的“核心调用”：optimizeGraph()
		std::map<int, Transform> poses = Rtabmap::optimizeGraph(id, uKeysSet(ids), optimizedPoses, lookInDatabase, covariance, constraints, error, iterationsDone);
		UINFO("optimize time %f s", timer.ticks());

		// 优化成功后的处理
		if(poses.size())
		{
			optimizedPoses = poses;

			if(_memory->getSignature(id) && uContains(optimizedPoses, id))
			{
				Transform t = optimizedPoses.at(id) * _memory->getSignature(id)->getPose().inverse();
				UINFO("Correction (from node %d) %s", id, t.prettyPrint().c_str());
			}
		}
		// 失败就“全清”
		else
		{
			UWARN("Failed to optimize the graph! returning empty optimized poses...");
			optimizedPoses.clear();
			if(constraints)
			{
				constraints->clear();
			}
		}
	}
}

std::map<int, Transform> Rtabmap::optimizeGraph(
		int fromId,
		const std::set<int> & ids,
		const std::map<int, Transform> & guessPoses,
		bool lookInDatabase,
		cv::Mat & covariance,
		std::multimap<int, Link> * constraints,
		double * error,
		int * iterationsDone) const
{
	UTimer timer;
	std::map<int, Transform> optimizedPoses;
	std::map<int, Transform> poses;
	std::multimap<int, Link> edgeConstraints;
	UDEBUG("ids=%d", (int)ids.size());
	_memory->getMetricConstraints(ids, poses, edgeConstraints, lookInDatabase, !_graphOptimizer->landmarksIgnored());
	UINFO("get constraints (ids=%d, %d poses, %d edges) time %f s", (int)ids.size(), (int)poses.size(), (int)edgeConstraints.size(), timer.ticks());

	// add landmark priors if there are some
	for(std::map<int, Transform>::iterator iter=poses.begin(); iter!=poses.end() && iter->first < 0; ++iter)
	{
		if(_markerPriors.find(iter->first) != _markerPriors.end())
		{
			cv::Mat infMatrix = cv::Mat::eye(6, 6, CV_64FC1);
			infMatrix(cv::Range(0,3), cv::Range(0,3)) /= _markerPriorsLinearVariance;
			infMatrix(cv::Range(3,6), cv::Range(3,6)) /= _markerPriorsAngularVariance;
			edgeConstraints.insert(std::make_pair(iter->first, Link(iter->first, iter->first, Link::kPosePrior, _markerPriors.at(iter->first), infMatrix)));
			UDEBUG("Added prior %d : %s (variance: lin=%f ang=%f)", iter->first, _markerPriors.at(iter->first).prettyPrint().c_str(),
					_markerPriorsLinearVariance, _markerPriorsAngularVariance);
		}
	}

	if(_graphOptimizer->iterations() > 0)
	{
		for(std::map<int, Transform>::iterator iter=poses.begin(); iter!=poses.end(); ++iter)
		{
			// Apply guess poses (if some), ignore for rootid to avoid origin drifting
			std::map<int, Transform>::const_iterator foundGuess = guessPoses.find(iter->first);
			if(foundGuess!=guessPoses.end() && iter->first != fromId)
			{
				iter->second = foundGuess->second;
			}
		}
	}


	UASSERT(_graphOptimizer!=0);
	if(_graphOptimizer->iterations() == 0)
	{
		// Optimization disabled! Return not optimized poses.
		optimizedPoses = poses;
		if(constraints)
		{
			*constraints = edgeConstraints;
		}
	}
	else
	{
		bool hasLandmarks = !edgeConstraints.empty() && edgeConstraints.begin()->first < 0;
		if(poses.size() != guessPoses.size() || hasLandmarks)
		{
			UDEBUG("recompute poses using only links (robust to multi-session)");
			std::map<int, Transform> posesOut;
			std::multimap<int, Link> edgeConstraintsOut;
			_graphOptimizer->getConnectedGraph(fromId, poses, edgeConstraints, posesOut, edgeConstraintsOut);
			optimizedPoses = _graphOptimizer->optimize(fromId, posesOut, edgeConstraintsOut, covariance, 0, error, iterationsDone);
			if(constraints)
			{
				*constraints = edgeConstraintsOut;
			}
		}
		else
		{
			UDEBUG("use input guess poses");
			optimizedPoses = _graphOptimizer->optimize(fromId, poses, edgeConstraints, covariance, 0, error, iterationsDone);
			if(constraints)
			{
				*constraints = edgeConstraints;
			}
		}

		if(!poses.empty() && optimizedPoses.empty())
		{
			UWARN("Optimization has failed (poses=%d, guess=%d, links=%d)...",
				  (int)poses.size(), (int)guessPoses.size(), (int)edgeConstraints.size());
		}
	}

	UINFO("Optimization time %f s", timer.ticks());

	return optimizedPoses;
}

void Rtabmap::adjustLikelihood(std::map<int, float> & likelihood) const
{
	ULOGGER_DEBUG("likelihood.size()=%d", likelihood.size());
	UTimer timer;
	timer.start();
	if(likelihood.size()==0)
	{
		return;
	}

	// Use only non-null values (ignore virtual place)
	std::list<float> values;
	bool likelihoodNullValuesIgnored = true;
	for(std::map<int, float>::iterator iter = ++likelihood.begin(); iter!=likelihood.end(); ++iter)
	{
		if((iter->second >= 0 && !likelihoodNullValuesIgnored) ||
		   (iter->second > 0 && likelihoodNullValuesIgnored))
		{
			values.push_back(iter->second);
		}
	}
	UDEBUG("values.size=%d", values.size());

	float mean = uMean(values);
	float stdDev = std::sqrt(uVariance(values, mean));


	//Adjust likelihood with mean and standard deviation (see Angeli phd)
	float epsilon = 0.0001;
	float max = 0.0f;
	int maxId = 0;
	for(std::map<int, float>::iterator iter=++likelihood.begin(); iter!= likelihood.end(); ++iter)
	{
		float value = iter->second;
		iter->second = 1.0f;
		if(value > mean+stdDev)
		{
			if(_virtualPlaceLikelihoodRatio==0 && mean)
			{
				iter->second = (value-(stdDev-epsilon))/mean;
			}
			else if(_virtualPlaceLikelihoodRatio!=0 && stdDev)
			{
				iter->second = (value-mean)/stdDev;
			}
		}

		if(value > max)
		{
			max = value;
			maxId = iter->first;
		}
	}

	if(_virtualPlaceLikelihoodRatio==0 && stdDev > epsilon && max)
	{
		likelihood.begin()->second = mean/stdDev + 1.0f;
	}
	else if(_virtualPlaceLikelihoodRatio!=0 && max > mean)
	{
		likelihood.begin()->second = stdDev/(max-mean) + 1.0f;
	}
	else
	{
		likelihood.begin()->second = 2.0f; //2 * std dev
	}

	double time = timer.ticks();
	UDEBUG("mean=%f, stdDev=%f, max=%f, maxId=%d, time=%fs", mean, stdDev, max, maxId, time);
}

void Rtabmap::dumpPrediction() const
{
	if(_memory && _bayesFilter)
	{
		if(this->getWorkingDir().empty())
		{
			UERROR("Working directory not set.");
			return;
		}
		std::list<int> signaturesToCompare;
		for(std::map<int, double>::const_iterator iter=_memory->getWorkingMem().begin();
			iter!=_memory->getWorkingMem().end();
			++iter)
		{
			if(iter->first > 0)
			{
				const Signature * s = _memory->getSignature(iter->first);
				UASSERT(s!=0);
				if(s->getWeight() != -1) // ignore intermediate nodes
				{
					signaturesToCompare.push_back(iter->first);
				}
			}
			else
			{
				// virtual signature should be added
				signaturesToCompare.push_back(iter->first);
			}
		}
		cv::Mat prediction = _bayesFilter->generatePrediction(_memory, uListToVector(signaturesToCompare));

		FILE* fout = 0;
		std::string fileName = this->getWorkingDir() + "/DumpPrediction.txt";
		#ifdef _MSC_VER
			fopen_s(&fout, fileName.c_str(), "w");
		#else
			fout = fopen(fileName.c_str(), "w");
		#endif

		if(fout)
		{
			for(int i=0; i<prediction.rows; ++i)
			{
				for(int j=0; j<prediction.cols; ++j)
				{
					fprintf(fout, "%f ",((float*)prediction.data)[j + i*prediction.cols]);
				}
				fprintf(fout, "\n");
			}
			fclose(fout);
		}
	}
	else
	{
		UWARN("Memory and/or the Bayes filter are not created");
	}
}

Signature Rtabmap::getSignatureCopy(int id, bool images, bool scan, bool userData, bool occupancyGrid, bool withWords, bool withGlobalDescriptors) const
{
	Signature s;
	if(_memory)
	{
		Transform odomPoseLocal;
		int weight = -1;
		int mapId = -1;
		std::string label;
		double stamp = 0;
		Transform groundTruth;
		std::vector<float> velocity;
		GPS gps;
		EnvSensors sensors;
		_memory->getNodeInfo(id, odomPoseLocal, mapId, weight, label, stamp, groundTruth, velocity, gps, sensors, true);
		SensorData data;
		data.setId(id);
		if(images || scan || userData || occupancyGrid)
		{
			data = _memory->getNodeData(id, images, scan, userData, occupancyGrid);
		}
		if(!images && withWords)
		{
			std::vector<CameraModel> models;
			std::vector<StereoCameraModel> stereoModels;
			_memory->getNodeCalibration(id, models, stereoModels);
			data.setCameraModels(models);
			data.setStereoCameraModels(stereoModels);
		}

		s=Signature(id,
				mapId,
				weight,
				stamp,
				label,
				odomPoseLocal,
				groundTruth,
				data);

		std::multimap<int, Link> links = _memory->getLinks(id, true, true);
		for(std::multimap<int, Link>::iterator iter=links.begin(); iter!=links.end(); ++iter)
		{
			if(iter->second.type() == Link::kLandmark)
			{
				s.addLandmark(iter->second);
			}
			else
			{
				s.addLink(iter->second);
			}
		}

		if(withWords || withGlobalDescriptors)
		{
			std::multimap<int, int> words;
			std::vector<cv::KeyPoint> wordsKpts;
			std::vector<cv::Point3f> words3;
			cv::Mat wordsDescriptors;
			std::vector<rtabmap::GlobalDescriptor> globalDescriptors;
			_memory->getNodeWordsAndGlobalDescriptors(id, words, wordsKpts, words3, wordsDescriptors, globalDescriptors);
			if(withWords)
			{
				s.setWords(words, wordsKpts, words3, wordsDescriptors);
			}
			if(withGlobalDescriptors)
			{
				s.sensorData().setGlobalDescriptors(globalDescriptors);
			}
		}
		if(velocity.size()==6)
		{
			s.setVelocity(velocity[0], velocity[1], velocity[2], velocity[3], velocity[4], velocity[5]);
		}
		s.sensorData().setGPS(gps);
		s.sensorData().setEnvSensors(sensors);
	}
	return s;
}

void Rtabmap::get3DMap(
		std::map<int, Signature> & signatures,
		std::map<int, Transform> & poses,
		std::multimap<int, Link> & constraints,
		bool optimized,
		bool global) const
{
	UDEBUG("");
	return getGraph(poses, constraints, optimized, global, &signatures, true, true, true, true);
}

void Rtabmap::getGraph(
		std::map<int, Transform> & poses,
		std::multimap<int, Link> & constraints,
		bool optimized,
		bool global,
		std::map<int, Signature> * signatures,
		bool withImages,
		bool withScan,
		bool withUserData,
		bool withGrid,
		bool withWords,
		bool withGlobalDescriptors) const
{
	if(_memory && _memory->getLastWorkingSignature())
	{
		if(_rgbdSlamMode)
		{
			if(optimized)
			{
				poses = _optimizedPoses; // guess
				cv::Mat covariance;
				this->optimizeCurrentMap(_memory->getLastWorkingSignature()->id(), global, poses, covariance, &constraints);
				if(!global && !_optimizedPoses.empty())
				{
					// We send directly the already optimized poses if they are set
					UDEBUG("_optimizedPoses=%ld poses=%ld", _optimizedPoses.size(), poses.size());
					poses = _optimizedPoses;
				}
			}
			else
			{
				std::map<int, int> ids = _memory->getNeighborsId(_memory->getLastWorkingSignature()->id(), 0, global?-1:0, true);
				_memory->getMetricConstraints(uKeysSet(ids), poses, constraints, global);
			}
		}
		else
		{
			// no optimization on appearance-only mode
			std::map<int, int> ids = _memory->getNeighborsId(_memory->getLastWorkingSignature()->id(), 0, global?-1:0, true);
			_memory->getMetricConstraints(uKeysSet(ids), poses, constraints, global);
		}

		if(signatures)
		{
			// Get data
			std::set<int> ids = uKeysSet(_memory->getWorkingMem()); // WM

			//remove virtual signature
			ids.erase(Memory::kIdVirtual);

			ids.insert(_memory->getStMem().begin(), _memory->getStMem().end()); // STM + WM
			if(global)
			{
				ids = _memory->getAllSignatureIds(); // STM + WM + LTM, ignoreChildren=true
			}

			for(std::set<int>::iterator iter = ids.begin(); iter!=ids.end(); ++iter)
			{
				signatures->insert(std::make_pair(*iter, getSignatureCopy(*iter, withImages, withScan, withUserData, withGrid, withWords, withGlobalDescriptors)));
			}
		}
	}
	else if(_memory && (_memory->getStMem().size() || _memory->getWorkingMem().size() > 1))
	{
		UERROR("Last working signature is null!?");
	}
	else if(_memory == 0)
	{
		UWARN("Memory not initialized...");
	}
}

std::map<int, Transform> Rtabmap::getNodesInRadius(const Transform & pose, float radius, int k, std::map<int, float> * distsSqr)
{
	std::map<int, float> nearestNodesTmp;
	std::map<int, float> * nearestNodesPtr = distsSqr == 0? &nearestNodesTmp : distsSqr;
	*nearestNodesPtr = graph::findNearestNodes(pose, _optimizedPoses, radius<=0?_localRadius:radius, 0, k);
	std::map<int, Transform> nearestPoses;
	for(std::map<int, float>::iterator iter=nearestNodesPtr->begin(); iter!=nearestNodesPtr->end(); ++iter)
	{
		nearestPoses.insert(*_optimizedPoses.find(iter->first));
	}
	return nearestPoses;
}

std::map<int, Transform> Rtabmap::getNodesInRadius(int nodeId, float radius, int k, std::map<int, float> * distsSqr)
{
	UDEBUG("nodeId=%d, radius=%f", nodeId, radius);
	std::map<int, float> nearestNodesTmp;
	std::map<int, float> * nearestNodesPtr = distsSqr == 0? &nearestNodesTmp : distsSqr;
	if(nodeId==0 && !_lastLocalizationPose.isNull() && !_lastLocalizationPose.isIdentity())
	{
		*nearestNodesPtr = graph::findNearestNodes(_lastLocalizationPose, _optimizedPoses, radius<=0?_localRadius:radius, 0, k);
	}
	else
	{
		if(nodeId==0 && !_optimizedPoses.empty())
		{
			nodeId = _optimizedPoses.rbegin()->first;
		}

		if(_optimizedPoses.find(nodeId) != _optimizedPoses.end())
		{
			*nearestNodesPtr = graph::findNearestNodes(nodeId, _optimizedPoses, radius<=0?_localRadius:radius, 0, k);
		}
	}

	std::map<int, Transform> nearestPoses;
	for(std::map<int, float>::iterator iter=nearestNodesPtr->begin(); iter!=nearestNodesPtr->end(); ++iter)
	{
		nearestPoses.insert(*_optimizedPoses.find(iter->first));
	}

	return nearestPoses;
}

int Rtabmap::detectMoreLoopClosures(
		float clusterRadiusMax,
		float clusterAngle,
		int iterations,
		bool intraSession,
		bool interSession,
		const ProgressState * processState,
		float clusterRadiusMin)
{
	UASSERT(iterations>0);

	if(_graphOptimizer->iterations() <= 0)
	{
		UERROR("Cannot detect more loop closures if graph optimization iterations = 0");
		return -1;
	}
	if(!_rgbdSlamMode)
	{
		UERROR("Detecting more loop closures can be done only in RGBD-SLAM mode.");
		return -1;
	}
	if(!intraSession && !interSession)
	{
		UERROR("Intra and/or inter session argument should be true.");
		return -1;
	}

	std::list<Link> loopClosuresAdded;
	std::multimap<int, int> checkedLoopClosures;

	std::map<int, Transform> posesToCheckLoopClosures;
	std::map<int, Transform> poses;
	std::multimap<int, Link> links;
	std::map<int, Signature> signatures; // some signatures may be in LTM, get them all
	this->getGraph(poses, links, true, true, &signatures);

	std::map<int, int> mapIds;
	UDEBUG("remove all invalid or intermediate nodes, fill mapIds");
	for(std::map<int, Transform>::iterator iter=poses.upper_bound(0); iter!=poses.end();++iter)
	{
		if(signatures.at(iter->first).getWeight() >= 0)
		{
			posesToCheckLoopClosures.insert(*iter);
			mapIds.insert(std::make_pair(iter->first, signatures.at(iter->first).mapId()));
		}
	}

	for(int n=0; n<iterations; ++n)
	{
		UINFO("Looking for more loop closures, clustering poses... (iteration=%d/%d, radius=%f m angle=%f rad)",
				n+1, iterations, clusterRadiusMax, clusterAngle);

		std::multimap<int, int> clusters = graph::radiusPosesClustering(
				posesToCheckLoopClosures,
				clusterRadiusMax,
				clusterAngle);

		UINFO("Looking for more loop closures, clustering poses... found %d clusters.", (int)clusters.size());

		int i=0;
		std::set<int> addedLinks;
		for(std::multimap<int, int>::iterator iter=clusters.begin(); iter!= clusters.end(); ++iter, ++i)
		{
			if(processState && processState->isCanceled())
			{
				return -1;
				break;
			}

			int from = iter->first;
			int to = iter->second;
			if(from > to)
			{
				from = iter->second;
				to = iter->first;
			}

			int mapIdFrom = uValue(mapIds, from, 0);
			int mapIdTo = uValue(mapIds, to, 0);

			if((interSession && mapIdFrom != mapIdTo) ||
			   (intraSession && mapIdFrom == mapIdTo))
			{

				bool alreadyChecked = false;
				for(std::multimap<int, int>::iterator jter = checkedLoopClosures.lower_bound(from);
					!alreadyChecked && jter!=checkedLoopClosures.end() && jter->first == from;
					++jter)
				{
					if(to == jter->second)
					{
						alreadyChecked = true;
					}
				}

				if(!alreadyChecked)
				{
					// only add new links and one per cluster per iteration
					if(addedLinks.find(from) == addedLinks.end() &&
					   addedLinks.find(to) == addedLinks.end() &&
					   rtabmap::graph::findLink(links, from, to) == links.end())
					{
						// Reverify if in the bounds with the current optimized graph
						Transform delta = poses.at(from).inverse() * poses.at(to);
						if(delta.getNorm() < clusterRadiusMax &&
						   delta.getNorm() >= clusterRadiusMin)
						{
							checkedLoopClosures.insert(std::make_pair(from, to));

							UASSERT(signatures.find(from) != signatures.end());
							UASSERT(signatures.find(to) != signatures.end());

							Transform guess;
							if(_proximityBySpace && uContains(poses, from) && uContains(poses, to))
							{
								guess = poses.at(from).inverse() * poses.at(to);
							}

							RegistrationInfo info;
							// use signatures instead of IDs because some signatures may not be in WM
							Transform t = _memory->computeTransform(signatures.at(from), signatures.at(to), guess, &info);

							if(!t.isNull())
							{
								bool updateConstraints = true;

								//optimize the graph to see if the new constraint is globally valid

								int fromId = from;
								int mapId = signatures.at(from).mapId();
								// use first node of the map containing from
								for(std::map<int, Signature>::iterator ster=signatures.begin(); ster!=signatures.end(); ++ster)
								{
									if(ster->second.mapId() == mapId)
									{
										fromId = ster->first;
										break;
									}
								}
								std::multimap<int, Link> linksIn = links;
								linksIn.insert(std::make_pair(from, Link(from, to, Link::kUserClosure, t, getInformation(info.covariance))));
								const Link * maxLinearLink = 0;
								const Link * maxAngularLink = 0;
								float maxLinearError = 0.0f;
								float maxAngularError = 0.0f;
								float maxLinearErrorRatio = 0.0f;
								float maxAngularErrorRatio = 0.0f;
								std::map<int, Transform> optimizedPoses;
								std::multimap<int, Link> links;
								UASSERT(poses.find(fromId) != poses.end());
								UASSERT_MSG(poses.find(from) != poses.end(), uFormat("id=%d poses=%d links=%d", from, (int)poses.size(), (int)links.size()).c_str());
								UASSERT_MSG(poses.find(to) != poses.end(), uFormat("id=%d poses=%d links=%d", to, (int)poses.size(), (int)links.size()).c_str());
								_graphOptimizer->getConnectedGraph(fromId, poses, linksIn, optimizedPoses, links);
								UASSERT(optimizedPoses.find(fromId) != optimizedPoses.end());
								UASSERT_MSG(optimizedPoses.find(from) != optimizedPoses.end(), uFormat("id=%d poses=%d links=%d", from, (int)optimizedPoses.size(), (int)links.size()).c_str());
								UASSERT_MSG(optimizedPoses.find(to) != optimizedPoses.end(), uFormat("id=%d poses=%d links=%d", to, (int)optimizedPoses.size(), (int)links.size()).c_str());
								UASSERT(graph::findLink(links, from, to) != links.end());
								optimizedPoses = _graphOptimizer->optimize(fromId, optimizedPoses, links);
								std::string msg;
								if(optimizedPoses.size())
								{
									graph::computeMaxGraphErrors(
											optimizedPoses,
											links,
											maxLinearErrorRatio,
											maxAngularErrorRatio,
											maxLinearError,
											maxAngularError,
											&maxLinearLink,
											&maxAngularLink);
									if(maxLinearLink)
									{
										UINFO("Max optimization linear error = %f m (link %d->%d)", maxLinearError, maxLinearLink->from(), maxLinearLink->to());
										if(_optimizationMaxError > 0.0f && maxLinearErrorRatio > _optimizationMaxError)
										{
											msg = uFormat("Rejecting edge %d->%d because "
														"graph error is too large after optimization (%f m for edge %d->%d with ratio %f > std=%f m). "
														"\"%s\" is %f.",
														from,
														to,
														maxLinearError,
														maxLinearLink->from(),
														maxLinearLink->to(),
														maxLinearErrorRatio,
														sqrt(maxLinearLink->transVariance()),
														Parameters::kRGBDOptimizeMaxError().c_str(),
														_optimizationMaxError);
										}
										else if(_optimizationMaxError == 0.0f && maxLinearErrorRatio>100 && !_graphOptimizer->isRobust())
										{
											UERROR("Huge optimization error detected!"
													"Linear error ratio of %f (edge %d->%d, type=%d, abs error=%f m, stddev=%f). You may consider "
													"enabling \"%s\" to reject those bad optimizations by setting it to a non null value!",
													maxLinearErrorRatio,
													maxLinearLink->from(),
													maxLinearLink->to(),
													maxLinearLink->type(),
													maxLinearError,
													sqrt(maxLinearLink->transVariance()),
													Parameters::kRGBDOptimizeMaxError().c_str());
										}
									}
									else if(maxAngularLink)
									{
										UINFO("Max optimization angular error = %f deg (link %d->%d)", maxAngularError*180.0f/M_PI, maxAngularLink->from(), maxAngularLink->to());
										if(_optimizationMaxError > 0.0f && maxAngularErrorRatio > _optimizationMaxError)
										{
											msg = uFormat("Rejecting edge %d->%d because "
														"graph error is too large after optimization (%f deg for edge %d->%d with ratio %f > std=%f deg). "
														"\"%s\" is %f m.",
														from,
														to,
														maxAngularError*180.0f/M_PI,
														maxAngularLink->from(),
														maxAngularLink->to(),
														maxAngularErrorRatio,
														sqrt(maxAngularLink->rotVariance()),
														Parameters::kRGBDOptimizeMaxError().c_str(),
														_optimizationMaxError);
										}
										else if(_optimizationMaxError == 0.0f && maxAngularErrorRatio>100 && !_graphOptimizer->isRobust())
										{
											UERROR("Huge optimization error detected!"
													"Angular error ratio of %f (edge %d->%d, type=%d, abs error=%f m, stddev=%f). You may consider "
													"enabling \"%s\" to reject those bad optimizations by setting it to a non null value!",
													maxAngularErrorRatio,
													maxAngularLink->from(),
													maxAngularLink->to(),
													maxAngularLink->type(),
													maxAngularError*180.0f/CV_PI,
													sqrt(maxAngularLink->rotVariance()),
													Parameters::kRGBDOptimizeMaxError().c_str());
										}
									}
								}
								else
								{
									msg = uFormat("Rejecting edge %d->%d because graph optimization has failed!",
												from,
												to);
								}
								if(!msg.empty())
								{
									UWARN("%s", msg.c_str());
									updateConstraints = false;
								}
								else
								{
									poses = optimizedPoses;
								}

								if(updateConstraints)
								{
									addedLinks.insert(from);
									addedLinks.insert(to);
									cv::Mat inf = getInformation(info.covariance);
									links.insert(std::make_pair(from, Link(from, to, Link::kUserClosure, t, inf)));
									loopClosuresAdded.push_back(Link(from, to, Link::kUserClosure, t, inf));
									std::string msg = uFormat("Iteration %d/%d: Added loop closure %d->%d! (%d/%d)", n+1, iterations, from, to, i+1, (int)clusters.size());
									UINFO(msg.c_str());

									if(processState)
									{
										UINFO(msg.c_str());
										if(!processState->callback(msg))
										{
											return -1;
										}
									}
								}
							}
						}
					}
				}
			}
		}

		if(processState)
		{
			std::string msg = uFormat("Iteration %d/%d: Detected %d total loop closures!", n+1, iterations, (int)addedLinks.size()/2);
			UINFO(msg.c_str());
			if(!processState->callback(msg))
			{
				return -1;
			}
		}
		else
		{
			UINFO("Iteration %d/%d: Detected %d total loop closures!", n+1, iterations, (int)addedLinks.size()/2);
		}

		if(addedLinks.size() == 0)
		{
			break;
		}

		UINFO("Optimizing graph with new links (%d nodes, %d constraints)...",
				(int)poses.size(), (int)links.size());
		int fromId = _optimizeFromGraphEnd?poses.rbegin()->first:poses.begin()->first;
		poses = _graphOptimizer->optimize(fromId, poses, links, 0);
		if(poses.size() == 0)
		{
			UERROR("Optimization failed! Rejecting all loop closures...");
			loopClosuresAdded.clear();
			return -1;
		}
		UINFO("Optimizing graph with new links... done!");
	}
	UINFO("Total added %d loop closures.", (int)loopClosuresAdded.size());

	if(loopClosuresAdded.size())
	{
		for(std::list<Link>::iterator iter=loopClosuresAdded.begin(); iter!=loopClosuresAdded.end(); ++iter)
		{
			_memory->addLink(*iter, true);
		}
		// Update optimized poses
		for(std::map<int, Transform>::iterator iter=_optimizedPoses.begin(); iter!=_optimizedPoses.end(); ++iter)
		{
			std::map<int, Transform>::iterator jter = poses.find(iter->first);
			if(jter != poses.end())
			{
				iter->second = jter->second;
			}
		}
		std::map<int, Transform> tmp;
		// Update also the links if some have been added in WM
		_memory->getMetricConstraints(uKeysSet(_optimizedPoses), tmp, _constraints, false);
		// This will force rtabmap_ros to regenerate the global occupancy grid if there was one
		_memory->save2DMap(cv::Mat(), 0, 0, 0);
	}
	return (int)loopClosuresAdded.size();
}

bool Rtabmap::globalBundleAdjustment(
		int optimizerType,
		bool rematchFeatures,
		int iterations,
		float pixelVariance)
{
	if(!_optimizedPoses.empty() && !_constraints.empty())
	{
		int iterations = Parameters::defaultOptimizerIterations();
		float pixelVariance = Parameters::defaultg2oPixelVariance();
		ParametersMap params = _parameters;
		Parameters::parse(params, Parameters::kOptimizerIterations(), iterations);
		Parameters::parse(params, Parameters::kg2oPixelVariance(), pixelVariance);
		if(iterations > 0)
		{
			uInsert(params, ParametersPair(Parameters::kOptimizerIterations(), uNumber2Str(iterations)));
		}
		if(pixelVariance > 0.0f)
		{
			uInsert(params, ParametersPair(Parameters::kg2oPixelVariance(), uNumber2Str(pixelVariance)));
		}

		std::map<int, Signature> signatures;
		for(std::map<int, Transform>::iterator iter=_optimizedPoses.lower_bound(1); iter!=_optimizedPoses.end(); ++iter)
		{
			if(_memory->getSignature(iter->first))
			{
				signatures.insert(std::make_pair(iter->first, *_memory->getSignature(iter->first)));
			}
		}

		Optimizer * optimizer = Optimizer::create((Optimizer::Type)optimizerType, params);
		std::map<int, Transform> poses = optimizer->optimizeBA(
				_optimizeFromGraphEnd?_optimizedPoses.lower_bound(1)->first:_optimizedPoses.rbegin()->first,
				_optimizedPoses,
				_constraints,
				signatures,
				rematchFeatures);
		delete optimizer;

		if(poses.empty())
		{
			UERROR("Optimization failed!");
		}
		else
		{
			_optimizedPoses = poses;
			// This will force rtabmap_ros to regenerate the global occupancy grid if there was one
			_memory->save2DMap(cv::Mat(), 0, 0, 0);
			return true;
		}
	}
	else
	{
		UERROR("Optimized poses (%ld) or constraints (%ld) are empty!", _optimizedPoses.size(), _constraints.size());
	}
	return false;
}

int Rtabmap::cleanupLocalGrids(
		const std::map<int, Transform> & poses,
		const cv::Mat & map,
		float xMin,
		float yMin,
		float cellSize,
		int cropRadius,
		bool filterScans)
{
	if(_memory)
	{
		return _memory->cleanupLocalGrids(
				poses,
				map,
				xMin,
				yMin,
				cellSize,
				cropRadius,
				filterScans);
	}
	return -1;
}

int Rtabmap::refineLinks()
{
	if(!_rgbdSlamMode)
	{
		UERROR("Refining links can be done only in RGBD-SLAM mode.");
		return -1;
	}

	std::list<Link> linksRefined;

	std::map<int, Transform> poses;
	std::multimap<int, Link> links;
	std::map<int, Signature> signatures;
	this->getGraph(poses, links, false, true, &signatures);

	int i=0;
	for(std::multimap<int, Link>::iterator iter=links.lower_bound(1); iter!= links.end(); ++iter)
	{
		int from = iter->second.from();
		int to = iter->second.to();

		UASSERT(signatures.find(from) != signatures.end());
		UASSERT(signatures.find(to) != signatures.end());

		RegistrationInfo info;
		// use signatures instead of IDs because some signatures may not be in WM
		Transform t = _memory->computeTransform(signatures.at(from), signatures.at(to), iter->second.transform(), &info);

		if(!t.isNull())
		{
			linksRefined.push_back(Link(from, to, iter->second.type(), t, info.covariance.inv()));
			UINFO("Refined link %d->%d! (%d/%d)", from, to, ++i, (int)links.size());
		}
	}
	UINFO("Total refined %d links.", (int)linksRefined.size());

	if(linksRefined.size())
	{
		for(std::list<Link>::iterator iter=linksRefined.begin(); iter!=linksRefined.end(); ++iter)
		{
			_memory->updateLink(*iter, true);
		}
	}
	return (int)linksRefined.size();
}

bool Rtabmap::addLink(const Link & link)
{
	const Transform & t = link.transform();
	if(!_rgbdSlamMode)
	{
		UERROR("Adding new link can be done only in RGBD-SLAM mode.");
		return false;
	}
	if(!_memory)
	{
		UERROR("Memory is not initialized.");
		return false;
	}
	if(t.isNull())
	{
		UERROR("Link's transform is null! (%d->%d type=%s)", link.from(), link.to(), link.typeName().c_str());
		return false;
	}
	if(_memory->isIncremental())
	{
		if(_memory->getSignature(link.from()) == 0)
		{
			UERROR("Link's \"from id\" %d is not in working memory", link.from());
			return false;
		}
		if(_memory->getSignature(link.to()) == 0)
		{
			UERROR("Link's \"to id\" %d is not in working memory", link.to());
			return false;
		}

		if(_optimizedPoses.find(link.from()) == _optimizedPoses.end() &&
		   _optimizedPoses.find(link.to()) == _optimizedPoses.end())
		{
			UERROR("Neither nodes %d or %d are in the local graph (size=%d). One of the 2 nodes should be in the local graph.", (int)_optimizedPoses.size(), link.from(), link.to());
			return false;
		}

		// add temporary the link
		if(!_memory->addLink(link))
		{
			UERROR("Cannot add new link %d->%d to memory", link.from(), link.to());
			return false;
		}

		// optimize with new link
		std::map<int, Transform> poses = _optimizedPoses;
		std::multimap<int, Link> links;
		cv::Mat covariance;
		optimizeCurrentMap(this->getLastLocationId(), false, poses, covariance, &links);

		if(poses.find(link.from()) == poses.end())
		{
			UERROR("Link's \"from id\" %d is not in the graph (size=%d)", link.from(), (int)poses.size());
			_memory->removeLink(link.from(), link.to());
			return false;
		}
		if(poses.find(link.to()) == poses.end())
		{
			UERROR("Link's \"to id\" %d is not in the graph (size=%d)", link.to(), (int)poses.size());
			_memory->removeLink(link.from(), link.to());
			return false;
		}

		std::string msg;
		if(poses.empty())
		{
			msg = uFormat("Rejecting edge %d->%d because graph optimization has failed!", link.from(), link.to());
		}
		else
		{
			float maxLinearError = 0.0f;
			float maxLinearErrorRatio = 0.0f;
			float maxAngularError = 0.0f;
			float maxAngularErrorRatio = 0.0f;
			const Link * maxLinearLink = 0;
			const Link * maxAngularLink = 0;

			graph::computeMaxGraphErrors(
					poses,
					links,
					maxLinearErrorRatio,
					maxAngularErrorRatio,
					maxLinearError,
					maxAngularError,
					&maxLinearLink,
					&maxAngularLink);
			if(maxLinearLink)
			{
				UINFO("Max optimization linear error = %f m (link %d->%d)", maxLinearError, maxLinearLink->from(), maxLinearLink->to());
				if(_optimizationMaxError > 0.0f && maxLinearErrorRatio > _optimizationMaxError)
				{
					msg = uFormat("Rejecting edge %d->%d because "
							  "graph error is too large after optimization (%f m for edge %d->%d with ratio %f > std=%f m). "
							  "\"%s\" is %f.",
							  link.from(),
							  link.to(),
							  maxLinearError,
							  maxLinearLink->from(),
							  maxLinearLink->to(),
							  maxLinearErrorRatio,
							  sqrt(maxLinearLink->transVariance()),
							  Parameters::kRGBDOptimizeMaxError().c_str(),
							  _optimizationMaxError);
				}
				else if(_optimizationMaxError == 0.0f && maxLinearErrorRatio>100 && !_graphOptimizer->isRobust())
				{
					UERROR("Huge optimization error detected!"
							"Linear error ratio of %f (edge %d->%d, type=%d, abs error=%f m, stddev=%f). You may consider "
							"enabling \"%s\" to reject those bad optimizations by setting it to a non null value!",
							maxLinearErrorRatio,
							maxLinearLink->from(),
							maxLinearLink->to(),
							maxLinearLink->type(),
							maxLinearError,
							sqrt(maxLinearLink->transVariance()),
							Parameters::kRGBDOptimizeMaxError().c_str());
				}
			}
			else if(maxAngularLink)
			{
				UINFO("Max optimization angular error = %f deg (link %d->%d)", maxAngularError*180.0f/M_PI, maxAngularLink->from(), maxAngularLink->to());
				if(_optimizationMaxError > 0.0f && maxAngularErrorRatio > _optimizationMaxError)
				{
					msg = uFormat("Rejecting edge %d->%d because "
							  "graph error is too large after optimization (%f deg for edge %d->%d with ratio %f > std=%f deg). "
							  "\"%s\" is %f m.",
							  link.from(),
							  link.to(),
							  maxAngularError*180.0f/M_PI,
							  maxAngularLink->from(),
							  maxAngularLink->to(),
							  maxAngularErrorRatio,
							  sqrt(maxAngularLink->rotVariance()),
							  Parameters::kRGBDOptimizeMaxError().c_str(),
							  _optimizationMaxError);
				}
				else if(_optimizationMaxError == 0.0f && maxAngularErrorRatio>100 && !_graphOptimizer->isRobust())
				{
					UERROR("Huge optimization error detected!"
							"Angular error ratio of %f (edge %d->%d, type=%d, abs error=%f m, stddev=%f). You may consider "
							"enabling \"%s\" to reject those bad optimizations by setting it to a non null value!",
							maxAngularErrorRatio,
							maxAngularLink->from(),
							maxAngularLink->to(),
							maxAngularLink->type(),
							maxAngularError*180.0f/CV_PI,
							sqrt(maxAngularLink->rotVariance()),
							Parameters::kRGBDOptimizeMaxError().c_str());
				}
			}
		}
		if(!msg.empty())
		{
			UERROR("%s", msg.c_str());
			_memory->removeLink(link.from(), link.to());
			return false;
		}

		// Update optimized poses
		for(std::map<int, Transform>::iterator iter=_optimizedPoses.begin(); iter!=_optimizedPoses.end(); ++iter)
		{
			std::map<int, Transform>::iterator jter = poses.find(iter->first);
			if(jter != poses.end())
			{
				iter->second = jter->second;
			}
		}
		if(!_optimizeFromGraphEnd)
		{
			_mapCorrection = _optimizedPoses.rbegin()->second * _memory->getSignature(_optimizedPoses.rbegin()->first)->getPose().inverse();
		}

		std::map<int, Transform> tmp;
		// Update also the links if some have been added in WM
		_memory->getMetricConstraints(uKeysSet(_optimizedPoses), tmp, _constraints, false);
		// This will force rtabmap_ros to regenerate the global occupancy grid if there was one
		_memory->save2DMap(cv::Mat(), 0, 0, 0);

		return true;
	}
	else // localization mode
	{
		int oldestId = link.from()>link.to()?link.to():link.from();
		int newestId = link.from()<link.to()?link.to():link.from();

		if(_memory->getSignature(oldestId) == 0)
		{
			UERROR("Link's id %d is not in working memory", oldestId);
			return false;
		}
		if(_optimizedPoses.find(oldestId) == _optimizedPoses.end())
		{
			UERROR("Link's id %d is not in the optimized graph (_optimizedPoses=%d)", oldestId, (int)_optimizedPoses.size());
			return false;
		}
		if(_optimizeFromGraphEnd)
		{
			UERROR("Adding link with %s=true in localization mode is not supported.", Parameters::kRGBDOptimizeFromGraphEnd().c_str());
			return false;
		}
		if(_odomCachePoses.find(newestId) == _odomCachePoses.end())
		{
			if(!_odomCachePoses.empty())
			{
				UERROR("Link's id %d is not in the odometry cache (oldest=%d, newest=%d, %s=%d)",
						newestId,
						_odomCachePoses.begin()->first,
						_odomCachePoses.rbegin()->first,
						Parameters::kRGBDMaxOdomCacheSize().c_str(),
						_maxOdomCacheSize);
			}
			else
			{
				UERROR("Link's id %d is not in the odometry cache (%s=%d).",
						newestId,
						Parameters::kRGBDMaxOdomCacheSize().c_str(),
						_maxOdomCacheSize);
			}
			return false;
		}

		// Verify if the new localization is valid by checking if there is
		// not too much deformation using current odometry poses
		// This will also refine localization links

		std::map<int, Transform> poses = _odomCachePoses;
		std::multimap<int, Link> constraints = _odomCacheConstraints;
		constraints.insert(std::make_pair(link.from(), link));
		for(std::multimap<int, Link>::iterator iter=constraints.begin(); iter!=constraints.end(); ++iter)
		{
			std::map<int, Transform>::iterator iterPose = _optimizedPoses.find(iter->second.to());
			if(iterPose != _optimizedPoses.end() && poses.find(iterPose->first) == poses.end())
			{
				poses.insert(*iterPose);
				// make the poses in the map fixed
				constraints.insert(std::make_pair(iterPose->first, Link(iterPose->first, iterPose->first, Link::kPosePrior, iterPose->second, cv::Mat::eye(6,6, CV_64FC1)*999999)));
			}
		}

		std::map<int, Transform> posesOut;
		std::multimap<int, Link> edgeConstraintsOut;
		bool priorsIgnored = _graphOptimizer->priorsIgnored();
		_graphOptimizer->setPriorsIgnored(false); //temporary set false to use priors above to fix nodes of the map
		_graphOptimizer->getConnectedGraph(newestId, poses, constraints, posesOut, edgeConstraintsOut);
		std::map<int, Transform> optPoses = _graphOptimizer->optimize(poses.begin()->first, posesOut, edgeConstraintsOut);
		_graphOptimizer->setPriorsIgnored(priorsIgnored); // set back

		bool rejectLocalization = false;
		if(optPoses.empty())
		{
			UWARN("Optimization failed, rejecting localization!");
			rejectLocalization = true;
		}
		else
		{
			UINFO("Compute max graph errors...");
			float maxLinearError = 0.0f;
			float maxLinearErrorRatio = 0.0f;
			float maxAngularError = 0.0f;
			float maxAngularErrorRatio = 0.0f;
			const Link * maxLinearLink = 0;
			const Link * maxAngularLink = 0;
			graph::computeMaxGraphErrors(
					optPoses,
					edgeConstraintsOut,
					maxLinearErrorRatio,
					maxAngularErrorRatio,
					maxLinearError,
					maxAngularError,
					&maxLinearLink,
					&maxAngularLink,
					_graphOptimizer->isSlam2d());
			if(maxLinearLink == 0 && maxAngularLink==0)
			{
				UWARN("Could not compute graph errors! Wrong loop closures could be accepted!");
			}

			if(maxLinearLink)
			{
				UINFO("Max optimization linear error = %f m (link %d->%d, var=%f, ratio error/std=%f)", maxLinearError, maxLinearLink->from(), maxLinearLink->to(), maxLinearLink->transVariance(), maxLinearError/sqrt(maxLinearLink->transVariance()));
				if(_optimizationMaxError > 0.0f && maxLinearErrorRatio > _optimizationMaxError)
				{
					UWARN("Rejecting localization (%d <-> %d) in this "
							"iteration because a wrong loop closure has been "
							"detected after graph optimization, resulting in "
							"a maximum graph error ratio of %f (edge %d->%d, type=%d, abs error=%f m, stddev=%f). The "
							"maximum error ratio parameter \"%s\" is %f of std deviation.",
							link.from(),
							link.to(),
							maxLinearErrorRatio,
							maxLinearLink->from(),
							maxLinearLink->to(),
							maxLinearLink->type(),
							maxLinearError,
							sqrt(maxLinearLink->transVariance()),
							Parameters::kRGBDOptimizeMaxError().c_str(),
							_optimizationMaxError);
					rejectLocalization = true;
				}
				else if(_optimizationMaxError == 0.0f && maxLinearErrorRatio>100 && !_graphOptimizer->isRobust())
				{
					UERROR("Huge optimization error detected!"
							"Linear error ratio of %f (edge %d->%d, type=%d, abs error=%f m, stddev=%f). You may consider "
							"enabling \"%s\" to reject those bad optimizations by setting it to a non null value!",
							maxLinearErrorRatio,
							maxLinearLink->from(),
							maxLinearLink->to(),
							maxLinearLink->type(),
							maxLinearError,
							sqrt(maxLinearLink->transVariance()),
							Parameters::kRGBDOptimizeMaxError().c_str());
				}
			}
			if(maxAngularLink)
			{
				UINFO("Max optimization angular error = %f deg (link %d->%d, var=%f, ratio error/std=%f)", maxAngularError*180.0f/CV_PI, maxAngularLink->from(), maxAngularLink->to(), maxAngularLink->rotVariance(), maxAngularError/sqrt(maxAngularLink->rotVariance()));
				if(_optimizationMaxError > 0.0f && maxAngularErrorRatio > _optimizationMaxError)
				{
					UWARN("Rejecting localization (%d <-> %d) in this "
							"iteration because a wrong loop closure has been "
							"detected after graph optimization, resulting in "
							"a maximum graph error ratio of %f (edge %d->%d, type=%d, abs error=%f deg, stddev=%f). The "
							"maximum error ratio parameter \"%s\" is %f of std deviation.",
							link.from(),
							link.to(),
							maxAngularErrorRatio,
							maxAngularLink->from(),
							maxAngularLink->to(),
							maxAngularLink->type(),
							maxAngularError*180.0f/CV_PI,
							sqrt(maxAngularLink->rotVariance()),
							Parameters::kRGBDOptimizeMaxError().c_str(),
							_optimizationMaxError);
					rejectLocalization = true;
				}
				else if(_optimizationMaxError == 0.0f && maxAngularErrorRatio>100 && !_graphOptimizer->isRobust())
				{
					UERROR("Huge optimization error detected!"
							"Angular error ratio of %f (edge %d->%d, type=%d, abs error=%f m, stddev=%f). You may consider "
							"enabling \"%s\" to reject those bad optimizations by setting it to a non null value!",
							maxAngularErrorRatio,
							maxAngularLink->from(),
							maxAngularLink->to(),
							maxAngularLink->type(),
							maxAngularError*180.0f/CV_PI,
							sqrt(maxAngularLink->rotVariance()),
							Parameters::kRGBDOptimizeMaxError().c_str());
				}
			}
		}

		if(!rejectLocalization)
		{
			Transform newOptPoseInv = optPoses.at(link.from()).inverse();
			Transform newT = newOptPoseInv * optPoses.at(link.to());
			Link linkTmp = link;
			linkTmp.setTransform(newT);
			if(oldestId == link.from())
			{
				_lastLocalizationPose = _optimizedPoses.at(link.from()) * linkTmp.transform();
				_odomCacheConstraints.insert(std::make_pair(linkTmp.to(), linkTmp.inverse()));
			}
			else
			{
				_lastLocalizationPose = _optimizedPoses.at(link.to()) * linkTmp.transform().inverse();
				_odomCacheConstraints.insert(std::make_pair(linkTmp.from(), linkTmp));
			}
			UINFO("Set _lastLocalizationPose=%s", _lastLocalizationPose.prettyPrint().c_str());
			if(_graphOptimizer->isSlam2d())
			{
				// transform constraint to 2D
				_lastLocalizationPose = _lastLocalizationPose.to3DoF();
			}
			Transform odomPose = _odomCachePoses.find(newestId)->second;
			_mapCorrection = _lastLocalizationPose * odomPose.inverse();
			_lastLocalizationNodeId = oldestId;
			return true;
		}
	}
	return false;
}

cv::Mat Rtabmap::getInformation(const cv::Mat & covariance) const
{
	cv::Mat information = covariance.inv();
	if(_loopCovLimited)
	{
		const std::vector<double> & odomMaxInf = _memory->getOdomMaxInf();
		if(odomMaxInf.size() == 6)
		{
			for(int i=0; i<6; ++i)
			{
				if(information.at<double>(i,i) > odomMaxInf[i])
				{
					information.at<double>(i,i) = odomMaxInf[i];
				}
			}
		}
	}
	return information;
}

void Rtabmap::addNodesToRepublish(const std::vector<int> & ids)
{
	if(ids.empty())
	{
		_nodesToRepublish.clear();
	}
	else if(_maxRepublished > 0 && _publishLastSignatureData)
	{
		_nodesToRepublish.insert(ids.begin(), ids.end());
	}
	else if(_maxRepublished == 0)
	{
		UWARN("%s=0, so cannot republish the %d requested nodes.", Parameters::kRtabmapMaxRepublished().c_str(), (int)ids.size());
	}
	else //_publishLastSignatureData=false
	{
		UWARN("%s=false, so cannot republish the %d requested nodes.", Parameters::kRtabmapPublishLastSignature().c_str(), (int)ids.size());
	}
}

void Rtabmap::clearPath(int status)
{
	UINFO("status=%d", status);
	_pathStatus = status;
	_path.clear();
	_pathCurrentIndex=0;
	_pathGoalIndex = 0;
	_pathTransformToGoal.setIdentity();
	_pathUnreachableNodes.clear();
	_pathStuckCount = 0;
	_pathStuckDistance = 0.0f;
	if(_memory)
	{
		_memory->removeAllVirtualLinks();
	}
}

// return true if path is updated
bool Rtabmap::computePath(int targetNode, bool global)
{
	this->clearPath(0);

	if(targetNode>0)
	{
		UINFO("Planning a path to node %d (global=%d)", targetNode, global?1:0);
	}
	else
	{
		UINFO("Planning a path to landmark %d (global=%d)", -targetNode, global?1:0);
	}

	if(!_rgbdSlamMode)
	{
		UWARN("A path can only be computed in RGBD-SLAM mode");
		return false;
	}

	UTimer totalTimer;
	UTimer timer;
	Transform transformToLandmark = Transform::getIdentity();

	// No need to optimize the graph
	if(_memory)
	{
		int currentNode = 0;
		if(_memory->isIncremental())
		{
			if(!_memory->getLastWorkingSignature())
			{
				UWARN("Working memory is empty... cannot compute a path");
				return false;
			}
			currentNode = _memory->getLastWorkingSignature()->id();
		}
		else
		{
			if(_lastLocalizationPose.isNull() || _optimizedPoses.empty())
			{
				UWARN("Last localization pose is null or optimized graph is empty... cannot compute a path");
				return false;
			}
			if(_optimizedPoses.begin()->first < 0)
			{
				std::map<int, Transform> poses(_optimizedPoses.lower_bound(1), _optimizedPoses.end());
				currentNode = graph::findNearestNode(poses, _lastLocalizationPose);
			}
			else
			{
				currentNode = graph::findNearestNode(_optimizedPoses, _lastLocalizationPose);
			}
		}
		if(currentNode && targetNode)
		{
			std::list<std::pair<int, Transform> > path = graph::computePath(
					currentNode,
					targetNode,
					_memory,
					global,
					false,
					_pathLinearVelocity,
					_pathAngularVelocity);

			//transform in current referential
			Transform t = uValue(_optimizedPoses, currentNode, Transform::getIdentity());
			_path.resize(path.size());
			int oi = 0;
			for(std::list<std::pair<int, Transform> >::iterator iter=path.begin(); iter!=path.end();++iter)
			{
				if(iter->first > 0)
				{
					// just keep nodes in the path
					_path[oi].first = iter->first;
					_path[oi++].second = t * iter->second;
				}
			}
			_path.resize(oi);
			if(!_path.empty() && !path.empty() && path.rbegin()->first < 0)
			{
				transformToLandmark = _path.back().second.inverse() * t * path.rbegin()->second;
			}
		}
		else if(currentNode == 0)
		{
			UWARN("We should be localized before planning.");
		}
	}
	UINFO("Total planning time = %fs (%d nodes, %f m long)", totalTimer.ticks(), (int)_path.size(), graph::computePathLength(_path));

	if(_path.size() == 0)
	{
		_path.clear();
		UWARN("Cannot compute a path!");
		return false;
	}
	else
	{
		UINFO("Path generated! Size=%d", (int)_path.size());
		if(ULogger::level() == ULogger::kInfo)
		{
			std::stringstream stream;
			for(unsigned int i=0; i<_path.size(); ++i)
			{
				stream << _path[i].first;
				if(i+1 < _path.size())
				{
					stream << " ";
				}
			}
			UINFO("Path = [%s]", stream.str().c_str());
		}
		if(_goalsSavedInUserData)
		{
			// set goal to latest signature
			std::string goalStr = uFormat("GOAL:%d", targetNode);

			// use label is exist
			if(_memory->getSignature(targetNode))
			{
				if(!_memory->getSignature(targetNode)->getLabel().empty())
				{
					goalStr = std::string("GOAL:")+_memory->getSignature(targetNode)->getLabel();
				}
			}
			else if(global)
			{
				std::map<int, std::string> labels = _memory->getAllLabels();
				std::map<int, std::string>::iterator iter = labels.find(targetNode);
				if(iter != labels.end() && !iter->second.empty())
				{
					goalStr = std::string("GOAL:")+labels.at(targetNode);
				}
			}
			setUserData(0, cv::Mat(1, int(goalStr.size()+1), CV_8SC1, (void *)goalStr.c_str()).clone());
		}
		_pathTransformToGoal = transformToLandmark;

		updateGoalIndex();
		return _path.size() || _pathStatus > 0;
	}

	return false;
}

bool Rtabmap::computePath(const Transform & targetPose, float tolerance)
{
	this->clearPath(0);

	UINFO("Planning a path to pose %s ", targetPose.prettyPrint().c_str());
	if(tolerance < 0.0f)
	{
		tolerance = _localRadius;
	}

	std::list<std::pair<int, Transform> > pathPoses;

	if(!_rgbdSlamMode)
	{
		UWARN("This method can only be used in RGBD-SLAM mode");
		return false;
	}

	//Find the nearest node
	UTimer timer;
	std::map<int, Transform> nodes = _optimizedPoses;
	std::multimap<int, int> links;
	for(std::map<int, Transform>::iterator iter=nodes.upper_bound(0); iter!=nodes.end(); ++iter)
	{
		const Signature * s = _memory->getSignature(iter->first);
		UASSERT(s);
		for(std::map<int, Link>::const_iterator jter=s->getLinks().begin(); jter!=s->getLinks().end(); ++jter)
		{
			// only add links for which poses are in "nodes"
			if(jter->second.from() != jter->second.to() && uContains(nodes, jter->second.to()))
			{
				links.insert(std::make_pair(jter->second.from(), jter->second.to()));
				//links.insert(std::make_pair(jter->second.to(), jter->second.from())); // <-> (commented: already added when iterating in nodes)
			}
		}
	}
	UINFO("Time getting links = %fs", timer.ticks());

	int currentNode = 0;
	if(_memory->isIncremental())
	{
		if(!_memory->getLastWorkingSignature())
		{
			UWARN("Working memory is empty... cannot compute a path");
			return false;
		}
		currentNode = _memory->getLastWorkingSignature()->id();
	}
	else
	{
		if(_lastLocalizationPose.isNull() || _optimizedPoses.empty())
		{
			UWARN("Last localization pose is null... cannot compute a path");
			return false;
		}
		if(_optimizedPoses.begin()->first < 0)
		{
			std::map<int, Transform> poses(_optimizedPoses.lower_bound(1), _optimizedPoses.end());
			currentNode = graph::findNearestNode(poses, _lastLocalizationPose);
		}
		else
		{
			currentNode = graph::findNearestNode(_optimizedPoses, _lastLocalizationPose);
		}
	}

	int nearestId;
	if(!_lastLocalizationPose.isNull() && _lastLocalizationPose.getDistance(targetPose) < tolerance)
	{
		// target can be reached from the current node
		nearestId = currentNode;
	}
	else
	{
		nearestId = rtabmap::graph::findNearestNode(nodes, targetPose);
	}
	UINFO("Nearest node found=%d ,%fs", nearestId, timer.ticks());
	if(nearestId > 0)
	{
		if(tolerance != 0.0f && targetPose.getDistance(nodes.at(nearestId)) > tolerance)
		{
			UWARN("Cannot plan farther than %f m from the graph! (distance=%f m from node %d)",
					tolerance, targetPose.getDistance(nodes.at(nearestId)), nearestId);
		}
		else
		{
			UINFO("Computing path from location %d to %d", currentNode, nearestId);
			UTimer timer;
			_path = uListToVector(rtabmap::graph::computePath(nodes, links, currentNode, nearestId));
			UINFO("A* time = %fs", timer.ticks());

			if(_path.size() == 0)
			{
				UWARN("Cannot compute a path!");
			}
			else
			{
				UINFO("Path generated! Size=%d", (int)_path.size());
				if(ULogger::level() == ULogger::kInfo)
				{
					std::stringstream stream;
					for(unsigned int i=0; i<_path.size(); ++i)
					{
						stream << _path[i].first;
						if(i+1 < _path.size())
						{
							stream << " ";
						}
					}
					UINFO("Path = [%s]", stream.str().c_str());
				}

				UASSERT(uContains(nodes, _path.back().first));
				_pathTransformToGoal = nodes.at(_path.back().first).inverse() * targetPose;

				updateGoalIndex();

				return true;
			}
		}
	}
	else
	{
		UWARN("Nearest node not found in graph (size=%d) for pose %s", (int)nodes.size(), targetPose.prettyPrint().c_str());
	}

	return false;
}

std::vector<std::pair<int, Transform> > Rtabmap::getPathNextPoses() const
{
	std::vector<std::pair<int, Transform> > poses;
	if(_path.size())
	{
		UASSERT(_pathCurrentIndex < _path.size() && _pathGoalIndex < _path.size());
		poses.resize(_pathGoalIndex-_pathCurrentIndex+1);
		int oi=0;
		for(unsigned int i=_pathCurrentIndex; i<=_pathGoalIndex; ++i)
		{
			std::map<int, Transform>::const_iterator iter = _optimizedPoses.find(_path[i].first);
			if(iter != _optimizedPoses.end())
			{
				poses[oi++] = *iter;
			}
			else
			{
				break;
			}
		}
		poses.resize(oi);
	}
	return poses;
}

std::vector<int> Rtabmap::getPathNextNodes() const
{
	std::vector<int> ids;
	if(_path.size())
	{
		UASSERT(_pathCurrentIndex < _path.size() && _pathGoalIndex < _path.size());
		ids.resize(_pathGoalIndex-_pathCurrentIndex+1);
		int oi = 0;
		for(unsigned int i=_pathCurrentIndex; i<=_pathGoalIndex; ++i)
		{
			std::map<int, Transform>::const_iterator iter = _optimizedPoses.find(_path[i].first);
			if(iter != _optimizedPoses.end())
			{
				ids[oi++] = iter->first;
			}
			else
			{
				break;
			}
		}
		ids.resize(oi);
	}
	return ids;
}

int Rtabmap::getPathCurrentGoalId() const
{
	if(_path.size())
	{
		UASSERT(_pathGoalIndex <= _path.size());
		return _path[_pathGoalIndex].first;
	}
	return 0;
}

void Rtabmap::updateGoalIndex()
{
	if(!_rgbdSlamMode)
	{
		UWARN("This method can on be used in RGBD-SLAM mode!");
		return;
	}

	if( _memory && _path.size())
	{
		// remove all previous virtual links
		for(unsigned int i=0; i<_pathCurrentIndex && i<_path.size(); ++i)
		{
			const Signature * s = _memory->getSignature(_path[i].first);
			if(s)
			{
				_memory->removeVirtualLinks(s->id());
			}
		}

		// for the current index, only keep the newest virtual link
		// This will make sure that the path is still connected even
		// if the new signature is removed (e.g., because of a small displacement)
		UASSERT(_pathCurrentIndex < _path.size());
		const Signature * currentIndexS = _memory->getSignature(_path[_pathCurrentIndex].first);
		UASSERT_MSG(currentIndexS != 0, uFormat("_path[%d].first=%d", _pathCurrentIndex, _path[_pathCurrentIndex].first).c_str());
		std::multimap<int, Link> links = currentIndexS->getLinks(); // make a copy
		bool latestVirtualLinkFound = false;
		for(std::multimap<int, Link>::reverse_iterator iter=links.rbegin(); iter!=links.rend(); ++iter)
		{
			if(iter->second.type() == Link::kVirtualClosure)
			{
				if(latestVirtualLinkFound)
				{
					_memory->removeLink(currentIndexS->id(), iter->first);
				}
				else
				{
					latestVirtualLinkFound = true;
				}
			}
		}

		// Make sure the next signatures on the path are linked together
		float distanceSoFar = 0.0f;
		for(unsigned int i=_pathCurrentIndex+1;
			i<_path.size();
			++i)
		{
			if(i>0)
			{
				if(_localRadius > 0.0f)
				{
					distanceSoFar += _path[i-1].second.getDistance(_path[i].second);
				}
				if(distanceSoFar <= _localRadius)
				{
					if(_path[i].first != _path[i-1].first)
					{
						const Signature * s = _memory->getSignature(_path[i].first);
						if(s)
						{
							if(!s->hasLink(_path[i-1].first) && _memory->getSignature(_path[i-1].first) != 0)
							{
								Transform virtualLoop = _path[i].second.inverse() * _path[i-1].second;
								_memory->addLink(Link(_path[i].first, _path[i-1].first, Link::kVirtualClosure, virtualLoop, cv::Mat::eye(6,6,CV_64FC1)*0.01)); // on the optimized path
								UINFO("Added Virtual link between %d and %d", _path[i-1].first, _path[i].first);
							}
						}
					}
				}
				else
				{
					break;
				}
			}
		}

		UDEBUG("current node = %d current goal = %d", _path[_pathCurrentIndex].first, _path[_pathGoalIndex].first);
		Transform currentPose;
		if(_memory->isIncremental())
		{
			if(_memory->getLastWorkingSignature() == 0 ||
			   !uContains(_optimizedPoses, _memory->getLastWorkingSignature()->id()))
			{
				UERROR("Last node is null in memory or not in optimized poses. Aborting the plan...");
				this->clearPath(-1);
				return;
			}
			currentPose = _optimizedPoses.at(_memory->getLastWorkingSignature()->id());
		}
		else
		{
			if(_lastLocalizationPose.isNull())
			{
				UERROR("Last localization pose is null. Aborting the plan...");
				this->clearPath(-1);
				return;
			}
			currentPose = _lastLocalizationPose;
		}

		int goalId = _path.back().first;
		if(uContains(_optimizedPoses, goalId))
		{
			//use local position to know if the goal is reached
			float d = currentPose.getDistance(_optimizedPoses.at(goalId)*_pathTransformToGoal);
			if(d < _goalReachedRadius)
			{
				UINFO("Goal %d reached!", goalId);
				this->clearPath(1);
			}
		}

		if(_path.size())
		{
			//Always check if the farthest node is accessible in local map (max to local space radius if set)
			unsigned int goalIndex = _pathCurrentIndex;
			float distanceFromCurrentNode = 0.0f;
			bool sameGoalIndex = false;
			for(unsigned int i=_pathCurrentIndex+1; i<_path.size(); ++i)
			{
				if(uContains(_optimizedPoses, _path[i].first))
				{
					if(_localRadius > 0.0f)
					{
						distanceFromCurrentNode = _path[_pathCurrentIndex].second.getDistance(_path[i].second);
					}

					if((goalIndex == _pathCurrentIndex && i == _path.size()-1) ||
					   _pathUnreachableNodes.find(i) == _pathUnreachableNodes.end())
					{
						if(distanceFromCurrentNode <= _localRadius)
						{
							goalIndex = i;
						}
						else
						{
							break;
						}
					}
				}
				else
				{
					break;
				}
			}
			UASSERT(_pathGoalIndex < _path.size() && goalIndex < _path.size());
			if(_pathGoalIndex != goalIndex)
			{
				UINFO("Updated current goal from %d to %d (%d/%d)",
						(int)_path[_pathGoalIndex].first, _path[goalIndex].first, (int)goalIndex+1, (int)_path.size());
				_pathGoalIndex = goalIndex;
			}
			else
			{
				sameGoalIndex = true;
			}

			// update nearest pose in the path
			unsigned int nearestNodeIndex = 0;
			float distance = -1.0f;
			bool sameCurrentIndex = false;
			UASSERT(_pathGoalIndex < _path.size());
			for(unsigned int i=_pathCurrentIndex; i<=_pathGoalIndex; ++i)
			{
				std::map<int, Transform>::iterator iter = _optimizedPoses.find(_path[i].first);
				if(iter != _optimizedPoses.end())
				{
					float d = currentPose.getDistanceSquared(iter->second);
					if(distance == -1.0f || distance > d)
					{
						distance = d;
						nearestNodeIndex = i;
					}
				}
			}
			if(distance < 0)
			{
				UERROR("The nearest pose on the path not found! Aborting the plan...");
				this->clearPath(-1);
			}
			else
			{
				UDEBUG("Nearest node = %d", _path[nearestNodeIndex].first);
			}
			if(distance >= 0 && nearestNodeIndex != _pathCurrentIndex)
			{
				_pathCurrentIndex = nearestNodeIndex;
				_pathUnreachableNodes.erase(nearestNodeIndex); // if we are on it, it is reachable
			}
			else
			{
				sameCurrentIndex = true;
			}

			bool isStuck = false;
			if(sameGoalIndex && sameCurrentIndex && _pathStuckIterations>0)
			{
				float distanceToCurrentGoal = 0.0f;
				std::map<int, Transform>::iterator iter = _optimizedPoses.find(_path[_pathGoalIndex].first);
				if(iter != _optimizedPoses.end())
				{
					if(_pathGoalIndex == _pathCurrentIndex &&
						_pathGoalIndex == _path.size()-1)
					{
						distanceToCurrentGoal = currentPose.getDistanceSquared(iter->second*_pathTransformToGoal);
					}
					else
					{
						distanceToCurrentGoal = currentPose.getDistanceSquared(iter->second);
					}
				}

				if(distanceToCurrentGoal > 0.0f)
				{
					if(distanceToCurrentGoal >= _pathStuckDistance)
					{
						// we are not approaching the goal
						isStuck = true;
						if(_pathStuckDistance == 0.0f)
						{
							_pathStuckDistance = distanceToCurrentGoal;
						}
					}
				}
				else
				{
					// no nodes available, cannot plan
					isStuck = true;
				}
			}

			if(isStuck && ++_pathStuckCount > _pathStuckIterations)
			{
				UWARN("Current goal %d not reached since %d iterations (\"RGBD/PlanStuckIterations\"=%d), mark that node as unreachable.",
						_path[_pathGoalIndex].first,
						_pathStuckCount,
						_pathStuckIterations);
				_pathStuckCount = 0;
				_pathStuckDistance = 0.0;
				_pathUnreachableNodes.insert(_pathGoalIndex);
				// select previous reachable one
				while(_pathUnreachableNodes.find(_pathGoalIndex) != _pathUnreachableNodes.end())
				{
					if(_pathGoalIndex == 0 || --_pathGoalIndex <= _pathCurrentIndex)
					{
						// plan failed!
						UERROR("No upcoming nodes on the path are reachable! Aborting the plan...");
						this->clearPath(-1);
						return;
					}
				}
			}
			else if(!isStuck)
			{
				_pathStuckCount = 0;
				_pathStuckDistance = 0.0;
			}
		}
	}
}

void Rtabmap::createGlobalScanMap()
{
	UDEBUG("Creating global scan map (if scans are available)");
	_globalScanMap.clear();
	_globalScanMapPoses.clear();
	std::vector<int> scanIndices;
	std::map<int, Transform> scanViewpoints;
	for(std::map<int, Transform>::iterator iter=_optimizedPoses.begin(); iter!=_optimizedPoses.end(); ++iter)
	{
		SensorData data = _memory->getNodeData(iter->first, false, true, false, false);
		if(!data.laserScanCompressed().empty())
		{
			LaserScan scan;
			data.uncompressDataConst(0, 0, &scan, 0, 0, 0, 0);
			if(!scan.empty())
			{
				UDEBUG("Adding scan %d (format=%s, points=%d)", iter->first, scan.formatName().c_str(), scan.size());
				scan = util3d::transformLaserScan(scan, iter->second*scan.localTransform());
				if(_globalScanMap.empty() || _globalScanMap.format() == scan.format())
				{
					_globalScanMap += scan;
					_globalScanMapPoses.insert(*iter);
					scanViewpoints.insert(std::make_pair(iter->first, iter->second * scan.localTransform()));
					scanIndices.resize(_globalScanMap.size(), iter->first);
				}
				else
				{
					UWARN("Incompatible scan formats (%s vs %s), cannot create global scan map.",
							_globalScanMap.formatName().c_str(),
							scan.formatName().c_str());
					_globalScanMap.clear();
					_globalScanMapPoses.clear();
					break;
				}
			}
			else
			{
				UDEBUG("Ignored %d (scan is empty), pose still added.", iter->first);
				_globalScanMapPoses.insert(*iter);
			}
		}
		else
		{
			UDEBUG("Ignored %d (no scan), pose still added.", iter->first);
			_globalScanMapPoses.insert(*iter);
		}
	}
	if(_globalScanMap.size() > 3)
	{
		float voxelSize = 0.0f;
		int normalK = 0;
		float normalRadius = 0.0f;
		Parameters::parse(_parameters, Parameters::kMemLaserScanVoxelSize(), voxelSize);
		Parameters::parse(_parameters, Parameters::kMemLaserScanNormalK(), normalK);
		Parameters::parse(_parameters, Parameters::kMemLaserScanNormalRadius(), normalRadius);

		if(voxelSize > 0.0f)
		{
			LaserScan voxelScan = util3d::commonFiltering(_globalScanMap, 1, 0, 0, voxelSize, normalK, normalRadius);
			if(voxelScan.hasNormals())
			{
				// adjust with point of views
				util3d::adjustNormalsToViewPoints(
						scanViewpoints,
						_globalScanMap,
						scanIndices,
						voxelScan);
			}
			_globalScanMap = voxelScan;
		}

		UINFO("Global scan map has been assembled (size=%d points, %d poses) "
				"for proximity detection (only in localization mode %s=false and with %s=true)",
				(int)_globalScanMap.size(),
				(int)_globalScanMapPoses.size(),
				Parameters::kMemIncrementalMemory().c_str(),
				Parameters::kRGBDProximityGlobalScanMap().c_str());

		//for debugging...
		if(!_globalScanMap.empty() && ULogger::level() == ULogger::kDebug)
		{
			if(!_wDir.empty())
			{
				UWARN("Saving %s/rtabmap_global_scan_map.pcd (only saved when logger level is debug)", _wDir.c_str());
				pcl::PCLPointCloud2::Ptr cloud2 = util3d::laserScanToPointCloud2(_globalScanMap);
				pcl::io::savePCDFile(_wDir+"/rtabmap_global_scan_map.pcd", *cloud2);
			}
			else
			{
				UWARN("%s is enabled and logger is debug, but %s is not set, cannot save global scan map for debugging.",
						Parameters::kRGBDProximityGlobalScanMap().c_str(), Parameters::kRtabmapWorkingDirectory().c_str());
			}
		}
	}
	if(!_globalScanMap.empty() && _globalScanMap.size()<100)
	{
		UWARN("Ignoring global scan map because it is too small (%d points).", (int)_globalScanMap.size());
		_globalScanMap.clear();
		_globalScanMapPoses.clear();
	}
}

} // namespace rtabmap
