#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/surface/mls.h>

namespace PCLFilterHelper {
	using namespace std;

	typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;
	typedef pcl::ConditionBase<pcl::PointXYZ> ConditionBaseT;

	PointCloudT::Ptr loadPointCloud(char * cloud_file);

	PointCloudT::Ptr toPtr(PointCloudT cloud);

	/// <summary>
	/// 立方体滤波
	/// </summary>
	/// <param name="cloud_in">输入点云</param>
	/// <param name="cloud_out">输出点云</param>
	/// <param name="min">最小点位 x y z 1</param>
	/// <param name="max">最大点位 x y z 1</param>
	/// <param name="negative">默认值为 false，输出点云为在设定字段的设定范围内的点集，如果设置为 true 则刚好相反</param>
	void cropFilter(PointCloudT::Ptr cloud_in, PointCloudT::Ptr &cloud_out, Eigen::Vector4f min, Eigen::Vector4f max, bool negative = false);

	/// <summary>
	/// 立方体滤波
	/// </summary>
	/// <param name="cloud_in">输入点云</param>
	/// <param name="cloud_out">输出点云</param>
	/// <param name="minx">最小X</param>
	/// <param name="maxx">最大X</param>
	/// <param name="miny">最小Y</param>
	/// <param name="maxy">最大Y</param>
	/// <param name="minz">最小Z</param>
	/// <param name="maxz">最大Z</param>
	/// <param name="negative">默认值为 false，输出点云为在设定字段的设定范围内的点集，如果设置为 true 则刚好相反</param>
	void cropFilter(PointCloudT::Ptr cloud_in, PointCloudT::Ptr &cloud_out, float minx, float maxx, float miny, float maxy, float minz, float maxz, bool negative = false);

	/// <summary>
	/// 凸包滤波
	/// </summary>
	/// <param name="cloud_in">输入点云</param>
	/// <param name="cloud_out">输出点云</param>
	/// <param name="boundingbox">封闭区域顶点</param>
	/// <param name="dimension">设置凸包维度</param>
	/// <param name="negative">默认值为 false，输出点云为在设定字段的设定范围内的点集，如果设置为 true 则刚好相反</param>
	/// <param name="keepOrganized"> false=删除点（默认），true=重新定义点，保留结构</param>
	void cropHullFilter(PointCloudT::Ptr cloud_in, PointCloudT::Ptr& cloud_out, PointCloudT::Ptr boundingbox, int dimension, bool negative, bool keepOrganized);
	
	/// 近似体素重心采样滤波
	/// </summary>
	/// <param name="cloud_in">输入点云</param>
	/// <param name="cloud_out">输出点云</param>
	/// <param name="lx">立方体X轴长度</param>
	/// <param name="ly">立方体Y轴长度</param>
	/// <param name="lz">立方体Z轴长度</param>
/// <param name="downsampleAllData">如果只有XYZ字段，则设置为false，如果对所有字段，如intensity，都进行下采样，则设置为true</param>
	void approximateVoxelGridFilter(PointCloudT::Ptr cloud_in, PointCloudT::Ptr& cloud_out, float lx, float ly, float lz, bool downsampleAllData);

	/// <summary>
	/// 均匀采样滤波
	/// </summary>
	/// <param name="cloud_in">输入点云</param>
	/// <param name="cloud_out">输出点云</param>
	/// <param name="radius_seach">半径范围内搜寻邻居点</param>
	void uniformSamplingFilter(PointCloudT::Ptr cloud_in, PointCloudT::Ptr& cloud_out, double radius_seach);

	/// <summary>
	/// 随机采样滤波
	/// </summary>
	/// <param name="cloud_in">输入点云</param>
	/// <param name="cloud_out">输出点云</param>
	/// <param name="sample">设置下采样点云的点数</param>
	/// <param name="seed">随机函数种子点</param>
	/// <param name="negative">默认值为 false，输出点云为在设定字段的设定范围内的点集，如果设置为 true 则刚好相反</param>
	/// <param name="keepOrganized"> false=删除点（默认），true=重新定义点，保留结构</param>
	void randomSamplingFilter(PointCloudT::Ptr cloud_in, PointCloudT::Ptr& cloud_out, unsigned int sample, unsigned int seed, bool negative, bool keepOrganized);

	/// <summary>
	/// 移动最小二乘增采样滤波
	/// </summary>
	/// <param name="cloud_in">输入点云</param>
	/// <param name="cloud_out">输出点云</param>
	/// <param name="tree">提取搜索方法的树对象，一般用近邻的kdTree即可</param>
	/// <param name="upsamplingMethod">Upsampling 采样的方法有 DISTINCT_CLOUD, RANDOM_UNIFORM_DENSITY</param>
	/// <param name="searchRadius">设置搜索邻域的半径</param>
	/// <param name="upsamplingRadius">采样的半径</param>
	/// <param name="upsamplingStepSize">采样步数的大小</param>
	void movingLeastSquaresFilter(PointCloudT::Ptr cloud_in, PointCloudT::Ptr &cloud_out, pcl::search::Search<pcl::PointXYZ>::Ptr tree, pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::UpsamplingMethod upsamplingMethod, double searchRadius, double upsamplingRadius, double upsamplingStepSize);

	/// <summary>
	/// 体素滤波
	/// </summary>
	/// <param name="cloud_in">输入点云</param>
	/// <param name="cloud_out">输出点云</param>
	/// <param name="lx">立方体X轴长度</param>
	/// <param name="ly">立方体Y轴长度</param>
	/// <param name="lz">立方体Z轴长度</param>
	void voxelFilter(PointCloudT::Ptr cloud_in, PointCloudT::Ptr &cloud_out, float lx, float ly, float lz);
	
	/// <summary>
	/// 统计滤波
	/// </summary>
	/// <param name="cloud_in">输入点云</param>
	/// <param name="cloud_out">输出点云</param>
	/// <param name="meanK">K近邻搜索点个数</param>
	/// <param name="std_dev_mul">标准差倍数</param>
	/// <param name="negative">默认值为 false，输出点云为在设定字段的设定范围内的点集，如果设置为 true 则刚好相反</param>
	/// <param name="keepOrganized"> false=删除点（默认），true=重新定义点，保留结构</param>
	void sorFilter(PointCloudT::Ptr cloud_in, PointCloudT::Ptr &cloud_out, int meanK, float std_dev_mul, bool negative = false, bool keepOrganized = false);
	
	/// <summary>
	/// 半径滤波
	/// </summary>
	/// <param name="cloud_in">输入点云</param>
	/// <param name="cloud_out">输出点云</param>
	/// <param name="radiusSearch">半径范围内搜寻邻居点</param>
	/// <param name="minNeighborsInRadius">邻居少于点数认为是离群点</param>
	/// <param name="negative">默认值为 false，输出点云为在设定字段的设定范围内的点集，如果设置为 true 则刚好相反</param>
	/// <param name="keepOrganized"> false=删除点（默认），true=重新定义点，保留结构</param>
	void rorFilter(PointCloudT::Ptr cloud_in, PointCloudT::Ptr &cloud_out, int radiusSearch, float minNeighborsInRadius, bool negative = false, bool keepOrganized = false);

	/// <summary>
	/// 直通滤波
	/// </summary>
	/// <param name="cloud_in">输入点云</param>
	/// <param name="cloud_out">输出点云</param>
	/// <param name="field_name">坐标轴名称:x ,y ,z</param>
	/// <param name="limit_min">最大值</param>
	/// <param name="limit_max">最小值</param>
	/// <param name="negative">默认值为 false，输出点云为在设定字段的设定范围内的点集，如果设置为 true 则刚好相反</param>
	void passThroughFilter(PointCloudT::Ptr cloud_in, PointCloudT::Ptr &cloud_out, std::string field_name, float limit_min, float limit_max, bool negative = false);
	
	/// <summary>
	/// 投影滤波
	/// </summary>
	/// <param name="cloud_in">输入点云</param>
	/// <param name="cloud_out">输出点云</param>
	/// <param name="coefficients">设置模型对应的系数,类似数组提供coefficients->values.resize(n)定义大小,coefficients->values[0]=赋值,平面的话值对应ax+by+cz+d=0</param>
	/// <param name="modelType">设置对应的投影模型</param>
	void projectInliersFilter(PointCloudT::Ptr cloud_in, PointCloudT::Ptr &cloud_out, pcl::ModelCoefficients::Ptr coefficients, int modelType);
	
	/// <summary>
	/// 双边滤波
	/// </summary>
	/// <param name="cloud_in">输入点云</param>
	/// <param name="cloud_out">输出点云</param>
	/// <param name="sigma_s">高斯双边滤波窗口大小</param>
	/// <param name="sigma_r">高斯标准差表示强度差异</param>
	/// <param name="tree">提取搜索方法的树对象，一般用近邻的kdTree即可</param>
	void bilateralFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_out, float sigma_s, float sigma_r, pcl::search::Search<pcl::PointXYZI>::Ptr tree);

	/// <summary>
	/// 快速双边滤波
	/// </summary>
	/// <param name="cloud_in">输入点云</param>
	/// <param name="cloud_out">输出点云</param>
	/// <param name="sigma_s">高斯双边滤波窗口大小</param>
	/// <param name="sigma_r">高斯标准差表示强度差异</param>
	void fastBilateralFilter(PointCloudT::Ptr cloud_in, PointCloudT::Ptr& cloud_out, float sigma_s, float sigma_r);

	/// <summary>
	/// 索引滤波
	/// </summary>
	/// <param name="cloud_in">输入点云</param>
	/// <param name="cloud_out">输出点云</param>
	/// <param name="indices">过滤的点云索引</param>
	/// <param name="negative">默认值为 false，输出点云为在设定字段的设定范围内的点集，如果设置为 true 则刚好相反</param>
	void extractIndicesFilter(PointCloudT::Ptr cloud_in,  PointCloudT::Ptr &cloud_out, pcl::PointIndices::Ptr indices, bool negative = false);
	
	/// <summary>
	/// 条件滤波
	/// </summary>
	/// <param name="cloud_in">输入点云</param>
	/// <param name="cloud_out">输出点云</param>
	/// <param name="condition">条件范围 pcl::ConditionAnd pcl::ConditionOr</param>
	/// <param name="keepOrganized">false=删除点（默认），true=重新定义点，保留结构</param>
	void conditionalRemovalFilter(PointCloudT::Ptr cloud_in, PointCloudT::Ptr& cloud_out, ConditionBaseT::Ptr condition, bool keepOrganized = false);
	
	/// <summary>
	/// 欧式距离分类
	/// </summary>
	/// <param name="cloud_in">输入点云</param>
	/// <param name="cluster_indices_out">输出索引</param>
	/// <param name="tolerance">近邻搜索的搜索半径 m</param>
	/// <param name="min_cluster_size">一个聚类需要的最少点数目</param>
	/// <param name="max_cluster_size">一个聚类需要的最大点数目</param>
	void euclideanClusterExtraction(PointCloudT::Ptr cloud_in, std::vector<pcl::PointIndices> &cluster_indices_out, double tolerance, int min_cluster_size, int max_cluster_size);

	/// <summary>
	/// 采样一致性分割
	/// </summary>
	/// <param name="cloud_in">输入点云</param>
	/// <param name="cluster_indices_out">输出索引</param>
	/// <param name="coefficients_out">输出模型</param>
	/// <param name="methodType">采样一致性方法的类型</param>
	/// <param name="modelType">样一致性所构造的几何模型的类型</param>
	/// <param name="distance">点到模型的距离阈值</param>
	/// <param name="max_iterations">迭代次数的上限</param>
	/// <param name="probability">选取至少一个局内点的概率</param>
	void sacSegmentationExtraction(PointCloudT::Ptr cloud_in, int methodType, pcl::SacModel modelType, double distance, int max_iterations, double probability, pcl::PointIndices &cluster_indices_out, pcl::ModelCoefficients &coefficients_out);

	/// <summary>
	/// 凸包分割
	/// </summary>
	/// <param name="cloud_in">输入点云</param>
	/// <param name="hull_in">凸包点云</param>
	/// <param name="cluster_indices_out">输出索引</param>
	/// <param name="height_min">最大高度</param>
	/// <param name="height_max">最小高度</param>
	/// <param name="vpx">视点坐标x</param>
	/// <param name="vpy">视点坐标x</param>
	/// <param name="vpz">视点坐标z</param>
	void extractPolygonalPrismDataExtraction(PointCloudT::Ptr cloud_in, PointCloudT::Ptr hull_in, pcl::PointIndices& cluster_indices_out, double height_min, double height_max, float vpx, float vpy, float vpz);
}

namespace PCLOCTreeHelper {
	using namespace std;

	typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;

	/// <summary>
	/// 点云解压缩
	/// </summary>
	/// <param name="filepath">文件路径</param>
	/// <param name="cloud_out">输出点云</param>
	void compressionDecode(std::string filepath, PointCloudT::Ptr& cloud_out);

	/// <summary>
	/// 点云压缩
	/// </summary>
	/// <param name="cloud_in">输入点云</param>
	/// <param name="filepath">文件路径</param>
	void compressionEncode(std::string filepath, PointCloudT::Ptr cloud_in);

	/// <summary>
	/// 八叉树点云变化检测
	/// </summary>
	/// <param name="cloud_in1">输入点云1</param>
	/// <param name="cloud_in2">输入点云2</param>
	/// <param name="resolution">八叉树分辨率，体素边长</param>
	/// <param name="point_out">新的点的索引</param>
	void octreeChangeDetector(PointCloudT::Ptr cloud_in1, PointCloudT::Ptr cloud_in2, float resolution, std::vector<int>& indices_out);
}

namespace PCLFeatureHelper {
	using namespace std;

	typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;

	/// <summary>
	/// 法线计算
	/// </summary>
	/// <param name="cloud_in">输入点云</param>
	/// <param name="radius">半径</param>
	/// <param name="normals_out">输出点云</param>
	void normalEstimationFeatures(PointCloudT::Ptr cloud_in, double radius, pcl::PointCloud<pcl::Normal>::Ptr& normals_out);
	
	/// <summary>
	/// fpfh计算
	/// </summary>
	/// <param name="cloud_in">输入点云</param>
	/// <param name="normals_in">输入法线</param>
	/// <param name="radius">半径</param>
	/// <param name="features_out">输出特征</param>
	void fpfhEstimationFeatures(PointCloudT::Ptr cloud_in, pcl::PointCloud<pcl::Normal>::Ptr normals_in, pcl::PointCloud<pcl::FPFHSignature33>::Ptr& features_out, double radius);
	
	/// <summary>
	/// 轮廓计算
	/// </summary>
	/// <param name="cloud_in">输入点云</param>
	/// <param name="normals_in">输入法线</param>
	/// <param name="radius">半径</param>
	/// <param name="angle">角度</param>
	/// <param name="cloud_out">输出点云</param>
	void boundaryEstimationFeatures(PointCloudT::Ptr cloud_in, pcl::PointCloud<pcl::Normal>::Ptr normals_in, PointCloudT::Ptr& cloud_out, double radius, float angle);

	/// <summary>
	/// 粗配准矩阵
	/// </summary>
	/// <param name="cloud_src_in">源点云</param>
	/// <param name="cloud_tgt_in">目标点云</param>
	/// <param name="fpfhs_src_in">源点云特征</param>
	/// <param name="fpfhs_tgt_in">目标点云特征</param>
	/// <param name="sac_trans_out">输出粗配准矩阵</param>
	/// <param name="maximum_iterations">最大迭代数</param>
	/// <param name="distance">样本间的最小距离</param>
	/// <param name="number">使用的样本数量</param>
	/// <param name="k">k邻</param>
	void sacRegistration(
		PointCloudT::Ptr cloud_src_in,
		PointCloudT::Ptr cloud_tgt_in,
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_src_in,
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_tgt_in,
		Eigen::Matrix4f& sac_trans_out,
		int maximum_iterations,
		float distance,
		int number,
		int k);

	/// <summary>
	/// 精配准
	/// </summary>
	/// <param name="cloud_src_in">源点云</param>
	/// <param name="cloud_tgt_in">目标点云</param>
	/// <param name="icp_trans_out">输出配准矩阵</param>
	/// <param name="naximum_iterations">最大迭代数</param>
	/// <param name="transformation_epsilon">两次变化矩阵之间的差值</param>
	/// <param name="euclidean_fitness_epsilon">均方误差</param>
	/// <param name="use_reciprocal">设置是否使用倒数对应</param>
	/// <param name="sac_trans">粗配准矩阵</param>
	void icpRegistration(
		PointCloudT::Ptr cloud_src_in,
		PointCloudT::Ptr cloud_tgt_in,
		Eigen::Matrix4f& icp_trans_out,
		int naximum_iterations,
		double transformation_epsilon = 1e-10,
		double euclidean_fitness_epsilon = 0.2,
		bool use_reciprocal = false,
		Eigen::Matrix4f sac_trans = {});
}