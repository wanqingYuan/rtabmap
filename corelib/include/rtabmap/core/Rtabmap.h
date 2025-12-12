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
/**
 * RTAB-Map 是一种 图优化 + 回环检测 (loop closure) 的 SLAM 系统，这个类是主要的 SLAM 控制器。
 */
#ifndef RTABMAP_H_
#define RTABMAP_H_

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines

#include "rtabmap/core/Parameters.h"
#include "rtabmap/core/SensorData.h"
#include "rtabmap/core/Statistics.h"
#include "rtabmap/core/Link.h"
#include "rtabmap/core/ProgressState.h"

#include <opencv2/core/core.hpp>
#include <list>
#include <stack>
#include <set>

namespace rtabmap
{

class EpipolarGeometry;
class Memory;
class BayesFilter;
class Signature;
class Optimizer;
class PythonInterface;

class RTABMAP_CORE_EXPORT Rtabmap
{
public:
	enum VhStrategy {kVhNone, kVhEpipolar, kVhUndef};

public:
	/* 创建/销毁 rtabmap主体对象，包含所有内存、回环检测、图优化等模块 */ 
	Rtabmap();
	virtual ~Rtabmap();

	/**
	 * @brief 主循环函数，用于输入一帧传感器数据（图像、深度、IMU、激光等），并使用提供的里程计姿态 odomPose 来更新地图。
	 * @param data 当前帧的传感器数据（包含图像、深度、激光等）。
	 * @param odomPose 当前帧的里程计位姿（对于RGB-D SLAM mode，此值必须非空）。
	 * @param odomCovariance 6×6 协方差矩阵（默认单位矩阵）
	 * @param odomVelocity 速度信息（可选）
	 * @param externalStats External statistics to be saved in the database for convenience
	 * 用户注入到数据库的自定义统计信息
	 * @return true = 有新的关键帧/节点加入地图。
	 */
	bool process(
			const SensorData & data,
			Transform odomPose,
			const cv::Mat & odomCovariance = cv::Mat::eye(6,6,CV_64FC1),
			const std::vector<float> & odomVelocity = std::vector<float>(),
			const std::map<std::string, float> & externalStats = std::map<std::string, float>());
	// for convenience
	// 使用两个方差值（线速度方差和角速度方差）代替协方差矩阵
	bool process(
			const SensorData & data,
			Transform odomPose,
			float odomLinearVariance,
			float odomAngularVariance,
			const std::vector<float> & odomVelocity = std::vector<float>(),
			const std::map<std::string, float> & externalStats = std::map<std::string, float>());
	// for convenience, loop closure detection only
	// 只进行回环检测，不需要里程计和深度
	bool process(
			const cv::Mat & image,
			int id=0, const std::map<std::string, float> & externalStats = std::map<std::string, float>());

	/**
	 * Initialize Rtabmap with parameters and a database
	 * 初始化slam系统
	 * @param parameters Parameters overriding default parameters and database parameters
	 *                   (@see loadDatabaseParameters) 待设置的参数（覆盖默认值）
	 * @param databasePath 数据库路径。 文件不存在->创建；文件存在->自动加载词袋、历史节点。
	 * @param loadDatabaseParameters 如果数据库存在(@see databasePath),选择是否从数据库中读取参数覆盖当前参数
	 */
	void init(const ParametersMap & parameters, const std::string & databasePath = "", bool loadDatabaseParameters = false);
	/**
	 * Initialize Rtabmap with parameters from a configuration file and a database
	 * 初始化，从.ini配置文件中加载参数
	 * @param configFile Configuration file (*.ini) overriding default parameters and database parameters
	 *                   (@see loadDatabaseParameters)
	 * @param databasePath The database input/output path. If not set, an
	 *                     empty database is used in RAM. If set and the file doesn't exist,
	 *                     it will be created empty. If the database exists, nodes and
	 *                     vocabulary will be loaded in working memory.
	 * @param loadDatabaseParameters If an existing database is used (@see databasePath),
	 *                               the parameters inside are loaded and set to current
	 *                               Rtabmap instance.
	 */
	void init(const std::string & configFile = "", const std::string & databasePath = "", bool loadDatabaseParameters = false);

	/**
	 * Close rtabmap. This will delete rtabmap object if set.
	 * 关闭 SLAM，保存或丢弃数据库。
	 * @param databaseSaved true=database saved, false=database discarded.
	 * @param databasePath output database file name, ignored if
	 *                     Db/Sqlite3InMemory=false (opened database is
	 *                     then overwritten).
	 */
	void close(bool databaseSaved = true, const std::string & ouputDatabasePath = "");

	// 返回当前工作目录。
	const std::string & getWorkingDir() const {return _wDir;}
	// 判断是否为 RGB-D SLAM 模式。
	bool isRGBDMode() const { return _rgbdSlamMode; }
	// 返回最近检测到的回环闭环 ID 和置信度。
	int getLoopClosureId() const {return _loopClosureHypothesis.first;} // 回环ID
	float getLoopClosureValue() const {return _loopClosureHypothesis.second;} // 回环置信度
	// 最高回环判别结果（不一定被接受）。
	int getHighestHypothesisId() const {return _highestHypothesis.first;}
	float getHighestHypothesisValue() const {return _highestHypothesis.second;}
	// 获取最新关键帧 ID。
	int getLastLocationId() const;
	// 返回工作内存（WM）和短期记忆（STM）节点 ID 列表。
	std::list<int> getWM() const; // 工作内存
	std::set<int> getSTM() const; // 短期记忆
	// 各内存大小。
	int getWMSize() const; // 工作内存大小
	int getSTMSize() const; // 短期记忆大小
	std::map<int, int> getWeights() const; // 工作内存中签名id的权重 <signature id, weight>
	int getTotalMemSize() const; // 总内存大小
	double getLastProcessTime() const {return _lastProcessTime;};
	// 是否存在短期记忆中
	bool isInSTM(int locationId) const;
	bool isIDsGenerated() const;
	// 获取最新统计信息
	const Statistics & getStatistics() const;
	// 当前局部图优化后的位姿。
	const std::map<int, Transform> & getLocalOptimizedPoses() const {return _optimizedPoses;}
	// 局部约束（邻居边、回环边、地标边等）。
	const std::multimap<int, Link> & getLocalConstraints() const {return _constraints;}
	// 返回某节点的优化后位姿。
	Transform getPose(int locationId) const;
	// 
	Transform getMapCorrection() const {return _mapCorrection;}
	const Memory * getMemory() const {return _memory;}
	float getGoalReachedRadius() const {return _goalReachedRadius;}
	float getLocalRadius() const {return _localRadius;}
	const Transform & getLastLocalizationPose() const {return _lastLocalizationPose;}

	// SLAM 每次处理时间上限
	float getTimeThreshold() const {return _maxTimeAllowed;} // in ms
	void setTimeThreshold(float maxTimeAllowed); // in ms
	// 工作内存最大节点数。
	int getMemoryThreshold() const {return _maxMemoryAllowed;} // in nodes
	void setMemoryThreshold(int maxMemoryAllowed); // in nodes

	// 设置SLAM初始姿态
	void setInitialPose(const Transform & initialPose);
	// 强制开始新地图（例如漂移太大）
	int triggerNewMap();
	// 为某节点添加文本标签。
	bool labelLocation(int id, const std::string & label);
	/**
	 * Set user data. Detect automatically if raw or compressed. If raw, the data is
	 * compressed too. A matrix of type CV_8UC1 with 1 row is considered as compressed.
	 * If you have one dimension unsigned 8 bits raw data, make sure to transpose it
	 * (to have multiple rows instead of multiple columns) in order to be detected as
	 * not compressed.
	 * 为某节点附加用户自定义数据（自动压缩）。
	 */
	bool setUserData(int id, const cv::Mat & data);
	// 导出 DOT 图文件，用于 GraphViz 可视化。
	void generateDOTGraph(const std::string & path, int id=0, int margin=5);
	// 输出位姿图（支持 g2o、toro、kitti 等格式）。
	void exportPoses(
			const std::string & path,
			bool optimized,
			bool global,
			int format // 0=raw, 1=rgbd-slam format, 2=KITTI format, 3=TORO, 4=g2o
	);
	// 重置 SLAM 内存（清空 WM 和 STM）。
	void resetMemory();
	// 调试输出。
	void dumpPrediction() const;
	void dumpData() const;
	// 读写参数。
	void parseParameters(const ParametersMap & parameters);
	const ParametersMap & getParameters() const {return _parameters;}
	// 设置工作目录
	void setWorkingDirectory(std::string path);
	// 撤销最近一次闭环检测。
	void rejectLastLoopClosure();
	// 删除最后一个节点（调试用途）。
	void deleteLastLocation();
	// 直接设置优化图结果。
	void setOptimizedPoses(const std::map<int, Transform> & poses, const std::multimap<int, Link> & constraints);
	Signature getSignatureCopy(int id, bool images, bool scan, bool userData, bool occupancyGrid, bool withWords, bool withGlobalDescriptors) const;
	// Use getGraph() instead with withImages=true, withScan=true, withUserData=true and withGrid=true.
	// 旧版，已弃用
	RTABMAP_DEPRECATED
		void get3DMap(std::map<int, Signature> & signatures,
				std::map<int, Transform> & poses,
				std::multimap<int, Link> & constraints,
				bool optimized,
				bool global) const;
	// 一次性导出：位姿图 约束 签名数据（图像、深度、栅格等），比旧版get3DMap更强大
	void getGraph(std::map<int, Transform> & poses,
			std::multimap<int, Link> & constraints,
			bool optimized,
			bool global,
			std::map<int, Signature> * signatures = 0,
			bool withImages = false,
			bool withScan = false,
			bool withUserData = false,
			bool withGrid = false,
			bool withWords = true,
			bool withGlobalDescriptors = true) const;
	// 以某位姿或节点为中心搜索半径内节点。
	std::map<int, Transform> getNodesInRadius(const Transform & pose, float radius, int k=0, std::map<int, float> * distsSqr=0); // If radius=0 and k=0, RGBD/LocalRadius is used. Can return landmarks.
	std::map<int, Transform> getNodesInRadius(int nodeId, float radius, int k=0, std::map<int, float> * distsSqr=0); // If nodeId==0, return poses around latest node. If radius=0 and k=0, RGBD/LocalRadius is used. Can return landmarks and use landmark id (negative) as request.
	// 尝试对图中更多节点做扩展回环检测。
	int detectMoreLoopClosures(
			float clusterRadiusMax = 0.5f,
			float clusterAngle = M_PI/6.0f,
			int iterations = 1,
			bool intraSession = true,
			bool interSession = true,
			const ProgressState * state = 0,
			float clusterRadiusMin = 0.0f);
	// 对整张图执行全局 BA（Bundle Adjustment）。
	bool globalBundleAdjustment(
			int optimizerType = 1 /*g2o*/,
			bool rematchFeatures = true,
			int iterations = 0,
			float pixelVariance = 0.0f);
	// 清理局部栅格地图中不再需要部分。
	int cleanupLocalGrids(
			const std::map<int, Transform> & mapPoses,
			const cv::Mat & map,
			float xMin,
			float yMin,
			float cellSize,
			int cropRadius = 1,
			bool filterScans = false);
	// 重新估计图中某些约束（例如 ICP）。
	int refineLinks();
	// 手动添加一条图约束。
	bool addLink(const Link & link);
	// 根据协方差矩阵生成信息矩阵（逆矩阵）。
	cv::Mat getInformation(const cv::Mat & covariance) const;
	void addNodesToRepublish(const std::vector<int> & ids);

	// 获取路径及状态。
	int getPathStatus() const {return _pathStatus;} // -1=failed 0=idle/executing 1=success
	void clearPath(int status); // -1=failed 0=idle/executing 1=success
	// 基于节点 ID 规划路径。
	bool computePath(int targetNode, bool global);
	// 基于任意目标坐标规划路径。
	bool computePath(const Transform & targetPose, float tolerance = -1.0f); // only in current optimized map, tolerance (m) < 0 means RGBD/LocalRadius, 0 means infinite
	const std::vector<std::pair<int, Transform> > & getPath() const {return _path;}
	std::vector<std::pair<int, Transform> > getPathNextPoses() const;
	std::vector<int> getPathNextNodes() const;
	int getPathCurrentGoalId() const;
	unsigned int getPathCurrentIndex() const {return _pathCurrentIndex;}
	unsigned int getPathCurrentGoalIndex() const {return _pathGoalIndex;}
	const Transform & getPathTransformToGoal() const {return _pathTransformToGoal;}

	std::map<int, Transform> getForwardWMPoses(int fromId, int maxNearestNeighbors, float radius, int maxDiffID) const;
	std::map<int, std::map<int, Transform> > getPaths(const std::map<int, Transform> & poses, const Transform & target, int maxGraphDepth = 0) const;
	void adjustLikelihood(std::map<int, float> & likelihood) const;
	std::pair<int, float> selectHypothesis(const std::map<int, float> & posterior,
											const std::map<int, float> & likelihood) const;

private:
	void optimizeCurrentMap(int id,
			bool lookInDatabase,
			std::map<int, Transform> & optimizedPoses,
			cv::Mat & covariance,
			std::multimap<int, Link> * constraints = 0,
			double * error = 0,
			int * iterationsDone = 0) const;
	std::map<int, Transform> optimizeGraph(
			int fromId,
			const std::set<int> & ids,
			const std::map<int, Transform> & guessPoses,
			bool lookInDatabase,
			cv::Mat & covariance,
			std::multimap<int, Link> * constraints = 0,
			double * error = 0,
			int * iterationsDone = 0) const;
	void updateGoalIndex();
	bool computePath(int targetNode, std::map<int, Transform> nodes, const std::multimap<int, rtabmap::Link> & constraints);

	void createGlobalScanMap();

	void setupLogFiles(bool overwrite = false);
	void flushStatisticLogs();

private:
	// Modifiable parameters
	bool _publishStats;
	bool _publishLastSignatureData;
	bool _publishPdf;
	bool _publishLikelihood;
	bool _publishRAMUsage;
	bool _computeRMSE;
	bool _saveWMState;
	float _maxTimeAllowed; // in ms
	unsigned int _maxMemoryAllowed; // signatures count in WM
	float _loopThr;
	float _loopRatio;
	float _aggressiveLoopThr;
	int _virtualPlaceLikelihoodRatio;
	float _maxLoopClosureDistance;
	bool _verifyLoopClosureHypothesis;
	unsigned int _maxRetrieved;
	unsigned int _maxLocalRetrieved;
	unsigned int _maxRepublished;
	bool _rawDataKept;
	bool _statisticLogsBufferedInRAM;
	bool _statisticLogged;
	bool _statisticLoggedHeaders;
	bool _rgbdSlamMode;
	float _rgbdLinearUpdate;
	float _rgbdAngularUpdate;
	float _rgbdLinearSpeedUpdate;
	float _rgbdAngularSpeedUpdate;
	float _newMapOdomChangeDistance;
	bool _neighborLinkRefining;
	bool _proximityByTime;
	bool _proximityBySpace;
	bool _scanMatchingIdsSavedInLinks;
	bool _loopClosureIdentityGuess;
	float _localRadius;
	float _localImmunizationRatio;
	int _proximityMaxGraphDepth;
	int _proximityMaxPaths;
	int _proximityMaxNeighbors;
	float _proximityFilteringRadius;
	bool _proximityRawPosesUsed;
	float _proximityAngle;
	bool _proximityOdomGuess;
	double _proximityMergedScanCovFactor;
	std::string _databasePath;  // 地图数据路径
	bool _optimizeFromGraphEnd;  // 决定在补算图时使用工作内存（WM）中的哪个节点作为起点（graph end 或 graph begin）。
	float _optimizationMaxError;
	bool _startNewMapOnLoopClosure;
	bool _startNewMapOnGoodSignature;
	float _goalReachedRadius; // meters
	bool _goalsSavedInUserData;
	int _pathStuckIterations;
	float _pathLinearVelocity;
	float _pathAngularVelocity;
	bool _forceOdom3doF;
	bool _restartAtOrigin;
	bool _loopCovLimited;
	bool _loopGPS;
	int _maxOdomCacheSize;
	bool _localizationSmoothing;
	double _localizationPriorInf;
	bool _localizationSecondTryWithoutProximityLinks;
	bool _createGlobalScanMap;
	float _markerPriorsLinearVariance;
	float _markerPriorsAngularVariance;

	std::pair<int, float> _loopClosureHypothesis;
	std::pair<int, float> _highestHypothesis;
	double _lastProcessTime;
	bool _someNodesHaveBeenTransferred;
	float _distanceTravelled;
	float _distanceTravelledSinceLastLocalization;
	bool _optimizeFromGraphEndChanged;

	// Abstract classes containing all loop closure
	// strategies for a type of signature or configuration.
	EpipolarGeometry * _epipolarGeometry;
	BayesFilter * _bayesFilter;
	Optimizer * _graphOptimizer;
	ParametersMap _parameters;

	Memory * _memory;

	FILE* _foutFloat;
	FILE* _foutInt;
	std::list<std::string> _bufferedLogsF;
	std::list<std::string> _bufferedLogsI;

	Statistics statistics_;

	std::string _wDir;

	std::map<int, Transform> _optimizedPoses;  // 存放从数据库或后端得到的（全局或局部）优化后节点
	std::multimap<int, Link> _constraints;
	Transform _mapCorrection;
	Transform _mapCorrectionBackup; // used in localization mode when odom is lost
	Transform _lastLocalizationPose; // Corrected odometry pose. In mapping mode, this corresponds to last pose return by getLocalOptimizedPoses().
	int _lastLocalizationNodeId; // for localization mode
	cv::Mat _localizationCovariance;
	std::map<int, std::pair<cv::Point3d, Transform> > _gpsGeocentricCache;
	bool _currentSessionHasGPS;
	LaserScan _globalScanMap;
	std::map<int, Transform> _globalScanMapPoses;
	std::map<int, Transform> _odomCachePoses;       // used in localization mode to reject loop closures
	std::multimap<int, Link> _odomCacheConstraints; // used in localization mode to reject loop closures
	std::map<int, Transform> _markerPriors;

	std::set<int> _nodesToRepublish;

	// Planning stuff
	int _pathStatus;
	std::vector<std::pair<int,Transform> > _path;
	std::set<unsigned int> _pathUnreachableNodes;
	unsigned int _pathCurrentIndex;
	unsigned int _pathGoalIndex;
	Transform _pathTransformToGoal;
	int _pathStuckCount;
	float _pathStuckDistance;

#ifdef RTABMAP_PYTHON
	PythonInterface * _python;
#endif

};

} // namespace rtabmap
#endif /* RTABMAP_H_ */
