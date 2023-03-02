#include "pcl_filter_helper.h"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/bilateral.h>

namespace PCLFilterHelper {
	using namespace std;

	PointCloudT::Ptr loadPointCloud(char * cloud_file)
	{
		PointCloudT::Ptr cloud(new PointCloudT);

		string cloud_file_str = cloud_file;
		string suffixStr = cloud_file_str.substr(cloud_file_str.find_last_of('.') + 1);

		if (strcmp(suffixStr.c_str(), "pcd") == 0) {
			pcl::io::loadPCDFile(cloud_file, *cloud);
		}
		else if (strcmp(suffixStr.c_str(), "ply") == 0) {
			pcl::io::loadPLYFile(cloud_file, *cloud);
		}
		else {
			cout << "not a pcd or ply file" << endl;
		}

		return cloud;
	}

	PointCloudT::Ptr toPtr(PointCloudT cloud) {
		PointCloudT::Ptr cloudPtr(new PointCloudT);
		pcl::copyPointCloud(cloud, *cloudPtr);
		return cloudPtr;
	}

	/// <summary>
	/// 立方体滤波
	/// </summary>
	/// <param name="cloud_in">输入点云</param>
	/// <param name="cloud_out">输出点云</param>
	/// <param name="min">最小点位 x y z 1</param>
	/// <param name="max">最大点位 x y z 1</param>
	/// <param name="negative">默认值为 false，输出点云为在设定字段的设定范围内的点集，如果设置为 true 则刚好相反</param>
	void cropFilter(PointCloudT::Ptr cloud_in, PointCloudT::Ptr &cloud_out, Eigen::Vector4f min, Eigen::Vector4f max, bool negative) {
		pcl::CropBox<pcl::PointXYZ> box_filter;						//滤波器对象
		box_filter.setMin(min);	//Min和Max是指立方体的两个对角点。每个点由一个四维向量表示，通常最后一个是1.
		box_filter.setMax(max);
		//box_filter.setRotation(rotation); //旋转 rx ry rz
		//box_filter.setTranslation(tanslation);//仿射矩阵 Affine3f 旋转加平移
		//box_filter.setTransform(transformax);//平移 tx ty tz
		box_filter.setNegative(negative);
		box_filter.setInputCloud(cloud_in);
		box_filter.filter(*cloud_out);
	}

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
	void cropFilter(PointCloudT::Ptr cloud_in, PointCloudT::Ptr &cloud_out, float minx, float maxx, float miny, float maxy, float minz, float maxz, bool negative) {
		Eigen::Vector4f min = Eigen::Vector4f(minx, miny, minz, 1.0);
		Eigen::Vector4f max = Eigen::Vector4f(maxx, maxy, maxz, 1.0);
		cropFilter(cloud_in, cloud_out, min, max, negative);
	}

	/// <summary>
	/// 凸包滤波
	/// </summary>
	/// <param name="cloud_in">输入点云</param>
	/// <param name="cloud_out">输出点云</param>
	/// <param name="boundingbox">封闭区域顶点</param>
	/// <param name="dimension">设置凸包维度</param>
	/// <param name="negative">默认值为 false，输出点云为在设定字段的设定范围内的点集，如果设置为 true 则刚好相反</param>
	/// <param name="keepOrganized"> false=删除点（默认），true=重新定义点，保留结构</param>
	void cropHullFilter(PointCloudT::Ptr cloud_in, PointCloudT::Ptr& cloud_out, PointCloudT::Ptr boundingbox, int dimension, bool negative, bool keepOrganized) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull(new pcl::PointCloud<pcl::PointXYZ>);	//描述凸包形状的点云
		std::vector<pcl::Vertices> polygons;//设置Vertices类型的向量，用于保存凸包顶点

		pcl::ConvexHull<pcl::PointXYZ> hull;
		hull.setInputCloud(boundingbox);	//设置输入点云：封闭区域顶点点云
		hull.setDimension(dimension);				//设置凸包维度
		hull.reconstruct(*surface_hull, polygons);//计算凸包结果

		pcl::CropHull<pcl::PointXYZ> CH;//创建CropHull滤波对象
		CH.setDim(dimension);					//设置维度，与凸包维度一致
		CH.setInputCloud(cloud_in);		//设置需要滤波的点云
		CH.setHullIndices(polygons);	//输入封闭区域的顶点
		CH.setHullCloud(surface_hull);	//输入封闭区域的形状
		CH.setNegative(negative);
		CH.setKeepOrganized(keepOrganized);
		CH.filter(*cloud_out);
	}

	/// <summary>
	/// 体素采样滤波
	/// </summary>
	/// <param name="cloud_in">输入点云</param>
	/// <param name="cloud_out">输出点云</param>
	/// <param name="lx">立方体X轴长度</param>
	/// <param name="ly">立方体Y轴长度</param>
	/// <param name="lz">立方体Z轴长度</param>
	void voxelFilter(PointCloudT::Ptr cloud_in, PointCloudT::Ptr &cloud_out, float lx, float ly, float lz) {
		std::cout << "size before voxelFilter: " << cloud_in->size() << std::endl;
		pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
		vox_grid.setLeafSize(lx, ly, lz);//设置滤波时创建的体素大小为lx m*ly m*lz m的立方体
		vox_grid.setInputCloud(cloud_in);
		vox_grid.filter(*cloud_out);
		std::cout << "size after voxelFilter: " << cloud_out->size() << std::endl;
	}

	/// <summary>
	/// 近似体素重心采样滤波
	/// </summary>
	/// <param name="cloud_in">输入点云</param>
	/// <param name="cloud_out">输出点云</param>
	/// <param name="lx">立方体X轴长度</param>
	/// <param name="ly">立方体Y轴长度</param>
	/// <param name="lz">立方体Z轴长度</param>
	/// <param name="downsampleAllData">如果只有XYZ字段，则设置为false，如果对所有字段，如intensity，都进行下采样，则设置为true</param>
	void approximateVoxelGridFilter(PointCloudT::Ptr cloud_in, PointCloudT::Ptr& cloud_out, float lx, float ly, float lz, bool downsampleAllData) {
		std::cout << "size before voxelFilter: " << cloud_in->size() << std::endl;
		pcl::ApproximateVoxelGrid<pcl::PointXYZ> vox_grid;
		vox_grid.setLeafSize(lx, ly, lz);//设置滤波时创建的体素大小为lx m*ly m*lz m的立方体
		vox_grid.setInputCloud(cloud_in);
		vox_grid.setDownsampleAllData(downsampleAllData);
		vox_grid.filter(*cloud_out);
		std::cout << "size after voxelFilter: " << cloud_out->size() << std::endl;
	}
	/// <summary>
	/// 均匀采样滤波
	/// </summary>
	/// <param name="cloud_in">输入点云</param>
	/// <param name="cloud_out">输出点云</param>
	/// <param name="radius_seach">半径范围内搜寻邻居点</param>
	void uniformSamplingFilter(PointCloudT::Ptr cloud_in, PointCloudT::Ptr& cloud_out, double radius_seach) {
		pcl::UniformSampling<pcl::PointXYZ> filter;
		filter.setRadiusSearch(radius_seach);
		filter.setInputCloud(cloud_in);
		filter.filter(*cloud_out);
	}

	/// <summary>
	/// 随机采样滤波
	/// </summary>
	/// <param name="cloud_in">输入点云</param>
	/// <param name="cloud_out">输出点云</param>
	/// <param name="sample">设置下采样点云的点数</param>
	/// <param name="seed">随机函数种子点</param>
	/// <param name="negative">默认值为 false，输出点云为在设定字段的设定范围内的点集，如果设置为 true 则刚好相反</param>
	/// <param name="keepOrganized"> false=删除点（默认），true=重新定义点，保留结构</param>
	void randomSamplingFilter(PointCloudT::Ptr cloud_in, PointCloudT::Ptr& cloud_out, unsigned int sample, unsigned int seed, bool negative, bool keepOrganized) {
		pcl::RandomSample<pcl::PointXYZ> filter;
		filter.setSample(sample);
		filter.setSeed(seed);
		filter.setNegative(negative);
		filter.setKeepOrganized(keepOrganized);
		filter.setInputCloud(cloud_in);
		filter.filter(*cloud_out);
	}

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
	void movingLeastSquaresFilter(PointCloudT::Ptr cloud_in, PointCloudT::Ptr& cloud_out, pcl::search::Search<pcl::PointXYZ>::Ptr tree, pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::UpsamplingMethod upsamplingMethod, double searchRadius, double upsamplingRadius, double upsamplingStepSize) {
		pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> filter;
		filter.setInputCloud(cloud_in);
		filter.setSearchMethod(tree);
		//设置搜索邻域的半径
		filter.setSearchRadius(searchRadius);
		// Upsampling 采样的方法有 DISTINCT_CLOUD, RANDOM_UNIFORM_DENSITY
		filter.setUpsamplingMethod(upsamplingMethod);
		// 采样的半径
		filter.setUpsamplingRadius(upsamplingRadius);
		// 采样步数的大小
		filter.setUpsamplingStepSize(upsamplingStepSize);
		filter.process(*cloud_out);
	}
	/// <summary>
	/// 统计滤波
	/// </summary>
	/// <param name="cloud_in">输入点云</param>
	/// <param name="cloud_out">输出点云</param>
	/// <param name="meanK">K近邻搜索点个数</param>
	/// <param name="std_dev_mul">标准差倍数</param>
	/// <param name="negative">默认值为 false，输出点云为在设定字段的设定范围内的点集，如果设置为 true 则刚好相反</param>
	/// <param name="keepOrganized"> false=删除点（默认），true=重新定义点，保留结构</param>
	void sorFilter(PointCloudT::Ptr cloud_in, PointCloudT::Ptr &cloud_out, int meanK, float std_dev_mul,bool negative, bool keepOrganized) {
		std::cout << "size before statisticalOutlierRemoval: " << cloud_in->size() << std::endl;
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter(true);
		filter.setInputCloud(cloud_in);
		filter.setMeanK(meanK);
		filter.setStddevMulThresh(std_dev_mul);
		filter.setNegative(negative);
		filter.setKeepOrganized(keepOrganized);
		filter.filter(*cloud_out);
		std::cout << "size after statisticalOutlierRemoval: " << cloud_out->size() << std::endl;
	}

	/// <summary>
	/// 半径滤波
	/// </summary>
	/// <param name="cloud_in">输入点云</param>
	/// <param name="cloud_out">输出点云</param>
	/// <param name="radiusSearch">半径范围内搜寻邻居点</param>
	/// <param name="minNeighborsInRadius">邻居少于点数认为是离群点</param>
	/// <param name="negative">默认值为 false，输出点云为在设定字段的设定范围内的点集，如果设置为 true 则刚好相反</param>
	/// <param name="keepOrganized"> false=删除点（默认），true=重新定义点，保留结构</param>
	void rorFilter(PointCloudT::Ptr cloud_in, PointCloudT::Ptr &cloud_out, int radiusSearch, float minNeighborsInRadius, bool negative, bool keepOrganized) {
		std::cout << "size before radiusOutlierRemoval: " << cloud_in->size() << std::endl;

		pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
		ror.setInputCloud(cloud_in);
		ror.setRadiusSearch(radiusSearch);
		ror.setMinNeighborsInRadius(minNeighborsInRadius);
		ror.setNegative(negative);
		ror.setKeepOrganized(keepOrganized);
		ror.filter(*cloud_out);

		std::cout << "size after radiusOutlierRemoval: " << cloud_out->size() << std::endl;

	}
	
	/// <summary>
	/// 直通滤波
	/// </summary>
	/// <param name="cloud_in">输入点云</param>
	/// <param name="cloud_out">输出点云</param>
	/// <param name="field_name">坐标轴名称:x ,y ,z</param>
	/// <param name="limit_min">最大值</param>
	/// <param name="limit_max">最小值</param>
	/// <param name="negative">默认值为 false，输出点云为在设定字段的设定范围内的点集，如果设置为 true 则刚好相反</param>
	void passThroughFilter(PointCloudT::Ptr cloud_in, PointCloudT::Ptr &cloud_out, std::string field_name, float limit_min, float limit_max, bool negative) {
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud(cloud_in);            //设置输入点云
		pass.setFilterFieldName(field_name);         //设置过滤时所需要点云类型的Z字段
		pass.setFilterLimits(limit_min, limit_max);        //设置在过滤字段的范围
		pass.setFilterLimitsNegative (negative);   //设置保留范围内还是过滤掉范围内
		pass.filter(*cloud_out);            //执行滤波，保存过滤结果在cloud_out

	}

	/// <summary>
	/// 条件滤波
	/// </summary>
	/// <param name="cloud_in">输入点云</param>
	/// <param name="cloud_out">输出点云</param>
	/// <param name="range_cond">条件范围</param>
	/// <param name="keepOrganized">false=删除点（默认），true=重新定义点，保留结构</param>
	void conditionalRemovalFilter(PointCloudT::Ptr cloud_in, PointCloudT::Ptr& cloud_out, ConditionBaseT::Ptr condition, bool keepOrganized) {
		//pcl::ConditionAnd<pcl::PointXYZ>::Ptr condition(new pcl::ConditionAnd<pcl::PointXYZ>());//创建与条件
		//pcl::ConditionOr表示或条件
		//condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
		//	pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 0.0)));
		//condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
		//	pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 0.8)));
		//GT表示大于等于，LT表示小于等于，EQ表示等于，GE表示大于，LE表示小于
		pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
		condrem.setCondition(condition);
		condrem.setInputCloud(cloud_in);
		condrem.setKeepOrganized(keepOrganized);
		condrem.filter(*cloud_out);
	}

	/// <summary>
	/// 投影滤波
	/// </summary>
	/// <param name="cloud_in">输入点云</param>
	/// <param name="cloud_out">输出点云</param>
	/// <param name="coefficients">设置模型对应的系数,类似数组提供coefficients->values.resize(n)定义大小,coefficients->values[0]=赋值,平面的话值对应ax+by+cz+d=0</param>
	/// <param name="modelType">设置对应的投影模型</param>
	void projectInliersFilter(PointCloudT::Ptr cloud_in, PointCloudT::Ptr &cloud_out, pcl::ModelCoefficients::Ptr coefficients, int modelType) {

		pcl::ProjectInliers<pcl::PointXYZ> proj;		//创建投影滤波对象
		proj.setModelType(modelType);			//设置对象对应的投影模型
		proj.setInputCloud(cloud_in);			//设置输入点云
		proj.setModelCoefficients(coefficients);		//设置模型对应的系数
		proj.filter(*cloud_out);						//执行投影滤波存储结果cloud_projected

	}

	/// <summary>
	/// 双边滤波
	/// </summary>
	/// <param name="cloud_in">输入点云</param>
	/// <param name="cloud_out">输出点云</param>
	/// <param name="sigma_s">高斯双边滤波窗口大小</param>
	/// <param name="sigma_r">高斯标准差表示强度差异</param>
	/// <param name="tree">提取搜索方法的树对象，一般用近邻的kdTree即可</param>
	void bilateralFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_out, float sigma_s, float sigma_r, pcl::search::Search<pcl::PointXYZI>::Ptr tree) {
		pcl::BilateralFilter<pcl::PointXYZI> bf;
		bf.setInputCloud(cloud_in);            //设置输入点云
		bf.setSearchMethod(tree);
		bf.setHalfSize(sigma_s);
		bf.setStdDev(sigma_r);
		bf.filter(*cloud_out);
	}

	/// <summary>
	/// 快速双边滤波
	/// </summary>
	/// <param name="cloud_in">输入点云</param>
	/// <param name="cloud_out">输出点云</param>
	/// <param name="sigma_s">高斯双边滤波窗口大小</param>
	/// <param name="sigma_r">高斯标准差表示强度差异</param>
	void fastBilateralFilter(PointCloudT::Ptr cloud_in, PointCloudT::Ptr& cloud_out, float sigma_s, float sigma_r) {
		pcl::FastBilateralFilter<pcl::PointXYZ> fbf;
		fbf.setInputCloud(cloud_in);            //设置输入点云
		fbf.setSigmaS(sigma_s);
		fbf.setSigmaR(sigma_r);
		fbf.filter(*cloud_out);
	}

	/// <summary>
	/// 索引滤波
	/// </summary>
	/// <param name="cloud_in">输入点云</param>
	/// <param name="cloud_out">输出点云</param>
	/// <param name="indices">过滤的点云索引</param>
	/// <param name="negative">默认值为 false，输出点云为在设定字段的设定范围内的点集，如果设置为 true 则刚好相反</param>
	void extractIndicesFilter(PointCloudT::Ptr cloud_in, PointCloudT::Ptr &cloud_out, pcl::PointIndices::Ptr indices, bool negative) {

		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud_in);
		extract.setIndices(indices);
		extract.setNegative(negative);
		extract.filter(*cloud_out);
	}

	/// <summary>
	/// 欧式距离分割
	/// </summary>
	/// <param name="cloud_in">输入点云</param>
	/// <param name="cluster_indices_out">输出索引</param>
	/// <param name="tolerance">近邻搜索的搜索半径 m</param>
	/// <param name="min_cluster_size">一个聚类需要的最少点数目</param>
	/// <param name="max_cluster_size">一个聚类需要的最大点数目</param>
	void euclideanClusterExtraction(PointCloudT::Ptr cloud_in, std::vector<pcl::PointIndices> &cluster_indices_out, double tolerance, int min_cluster_size, int max_cluster_size) {
		// 创建用于提取搜索方法的kdtree树对象
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud(cloud_in);

		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;		//欧式聚类对象
		ec.setClusterTolerance(tolerance);						// 设置近邻搜索的搜索半径为tolerance m
		ec.setMinClusterSize(min_cluster_size);						//设置一个聚类需要的最少的点数目为100
		ec.setMaxClusterSize(max_cluster_size);					//设置一个聚类需要的最大点数目为200000
		ec.setSearchMethod(tree);						//设置点云的搜索机制
		ec.setInputCloud(cloud_in);
		ec.extract(cluster_indices_out);					//从点云中提取聚类，并将点云索引保存在cluster_indices中
	}
	
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
	void sacSegmentationExtraction(PointCloudT::Ptr cloud_in, int methodType, pcl::SacModel modelType, double distance, int max_iterations, double probability, pcl::PointIndices &cluster_indices_out, pcl::ModelCoefficients &coefficients_out) {
		pcl::SACSegmentation<pcl::PointXYZ> sac;		//一致性分割
		sac.setInputCloud(cloud_in);
		//设置使用采样一致性方法的类型(用户给定参数),采样一致性方法的类型有
		//SAC_RANSAC（随机采样一致性）、SAC_LMEDS（最小平方中值）、 SAC_MSAC（M估计采样一致性）、SAC_RRANSAC（随机采样一致性）、SAC_RMSAC（M估计采样一致性）、SAC_MLESAC（最大似然采样一致性）、SAC_PROSAC（渐进一致性采样）
		sac.setMethodType(methodType);
		//设置随机采样一致性所构造的几何模型的类型(用户给定的参数),model为指定的模型类型参数,本章节中涉及的模型类型如下:
		/*
		SACMODEL_PLANE模型:定义为平面模型,共设置4个参数[ normal_x, normal_y, normal_z d],其中(normal_x, normal_y, normal_z)为Hessian 范式中法向量的坐标及常量d值,ax+by+cz+d=0,从点云中分割提取的内点都处在估计参数对应的平面上或与平面距离在一定范围内。

		SACMODEL_LINE模型:定义为直线模型,共设置6个参数[point_on_line.x, point_on_line.y, point_on_line.z, line_direction.x, line_direction.y, line_direction.z],其中(point_on_line.x,point_on_line.y,point_on_line.z)为直线上一点的三维坐标，(line_direction.x, line_direction.y, line_direction.z)为直线方向向量的三维坐标，从点云中分割提取的内点都处在估计参数对应直线上或与直线的距离在一定范围内。

		SACMODEL_CIRCLE2D模型:定义为二维圆的圆周模型,共设置3个参数[center.x, center.y, radius],其中(center.x, center.y)为圆周中心点的二维坐标,radius为圆周半径,从点云中分割提取的内点都处在估计参数对应的圆周上或距离圆周边线的距离在一定范围内。

		SACMODEL_NORMAL_SPHERE和SACMODEL_SPHERE模型:定义为三维球体模型,共设置4个参数[center.x, center.y, center.z radius],其中(center. x, center. y, center. z)为球体中心的三维坐标,radius为球体半径,从点云中分割提取的内点都处在估计参数对应的球体上或距离球体边线的距离在一定范围内。

		SACMODEL_CYLINDER模型:定义为圆柱体模型,共设置7个参数[point_on_axis.x, point_on_axis.y, point_on_axis.z, axis_direction.x ,axis_direction.y ,axis_diection.z, radius],其中,(point_on_axis.x, point_on_axis.y, point_on_axis.z)为轴线上点的三维坐标,(direction.x ,axis_direction.y, axis_direction.z)为轴线方向向量的三维坐标,radius为圆柱体半径，从点云中分割提取的内点都处在估计参数对应的圆柱体上或距离圆柱体表面的距离在一定范围内。

		SACMODEL_CONE模型:定义为圆锥模型，共设置7个参数[apex.x, apex.y, apex.z, axis_direction.x, axis_direction.y, axis_direction.z, opening_angle],其中(apex.x, apex.y, apex.z)圆锥顶点,(axis_direction.x, axis_direction.y, axis_direction.z)为圆锥轴方向的三维坐标,opening_angle为圆锥的开口角度

		SACMODEL_TORUS模型:定义为圆环面模型,尚未实现。

		SACMODEL_PARALLEL_LINE 模型:定义为有条件限制的直线模型,在规定的最大角度偏差限制下,直线模型与给定轴线平行,共设置6个参数[point_on_line.x, point_on_line.y, point_on_line.z, line_direction.x, line_direction.y, line_direction.z],其中(point_on_line.x, point_on_line.y, point_on_line.z)为线上一个点的三维坐标,(line_direction.x, line_direction.y, line_direction.z)为直线方向的三维坐标。

		SACMODEL_PERPENDICULAR_PLANE模型:定义为有条件限制的平面模型,在规定的最大角度偏差限制下,平面模型与给定轴线垂直,参数设置参见SAC-MODEL_PLANE模型。

		SACMODEL_NORMAL_PLANE模型:定义为有条件限制的平面模型,在规定的最大角度偏差限制下,每一个局内点的法线必须与估计的平面模型的法线平行,参数设置参见SACMODEL_PLANE模型。

		SACMODEL_PARALLEL_PLANE模型:定义为有条件限制的平面模型,在规定的最大角度偏差限制下,平面模型与给定的轴线平行,参数设置参见SACMODEL_PLANE模型。

		SACMODEL_NORMAL_PARALLEL_PLANE模型:定义为有条件限制的平面模型,在法线约束下，三维平面模型必须与用户设定的轴线平行,共4个参数[a,b,c,d],其中a是平面法线的 X 坐标（归一化）,b是平面法线的 Y 坐标（归一化）,c是平面法线的 Z 坐标（归一化）,d是平面方程的第四个 Hessian 分量。

		SACMODEL_STICK模型,定义棒是用户给定最小/最大宽度的线。共7个参数[point_on_line.x, point_on_line.y, point_on_line.z, line_direction.x, line_direction.y, line_direction.z, line_width],其中(point_on_line.x, point_on_line.y, point_on_line.z)为线上一个点的三维坐标,(line_direction.x, line_direction.y, line_direction.z)为直线方向的三维坐标,line_width为线的宽度。

		*/
		sac.setModelType(modelType);
		//设置点到模型的距离阈值,如果点到模型的距离不超过这个距离阈值,认为该点为局内点,否则认为是局外点,被剔除。
		sac.setDistanceThreshold(distance);
		sac.setMaxIterations(max_iterations);	//设置迭代次数的上限
		sac.setProbability(probability);		//设置每次从数据集中选取至少一个局内点的概率
		
		////设置是否对估计的模型参数进行优化
		//sac.setOptimizeCoefficients(optimize); 
		////该函数配合,当用户设定带有半径参数的模型类型时，设置模型半径参数的最大最小半径阈值
		//sac.setRadiusLimits(min_radius, max_radius);
		////该函数配合，当用户设定与轴线平行或垂直有关的模型类型时,设置垂直或平行于所要建立模型的轴线。
		//sac.setAxis(ax);
		////该函数配合，当用户设定有平行或垂直限定有关的模型类型时，设置判断是否平行或垂直时的角度阈值,ea是最大角度差,采用弧度制。
		//sac.setEpsAngle(ea);
		
		//输出最终估计的模型参数，以及分割得到的点集合索引
		sac.segment(cluster_indices_out, coefficients_out);
	}

	/// <summary>
	/// 凸包分割
	/// </summary>
	/// <param name="cloud_in">输入点云</param>
	/// <param name="cluster_indices_out">输出索引</param>
	/// <param name="hull_in">凸包点云</param>
	/// <param name="height_min">最大高度</param>
	/// <param name="height_max">最小高度</param>
	/// <param name="vpx">视点坐标x</param>
	/// <param name="vpy">视点坐标x</param>
	/// <param name="vpz">视点坐标z</param>
	void extractPolygonalPrismDataExtraction(PointCloudT::Ptr cloud_in, PointCloudT::Ptr hull_in, pcl::PointIndices& cluster_indices_out, double height_min, double height_max, float vpx, float vpy, float vpz) {
		pcl::ExtractPolygonalPrismData<pcl::PointXYZ> eppd;
		//设置高度范围(height_max为最大高度、height_min为最小高度),当给定点云中的点到构造棱柱体的平面模型的距离超出指定的这个高度范围时,该点视为局外点,所有的局外点都会被剔除。
		eppd.setHeightLimits(height_min, height_max);
		eppd.setViewPoint(vpx, vpy, vpz);		//设置视点,vpx,vpy,vpz分别为视点的三维坐标。
		eppd.setInputCloud(cloud_in);
		eppd.setInputPlanarHull(hull_in);		//设置平面模型上的点集,hull为指向该点集的指针
		eppd.segment(cluster_indices_out);		//从点云中提取聚类，并将点云索引保存在cluster_indices中
	}
}

#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <fstream>

namespace PCLOCTreeHelper {
	/// <summary>
	/// 点云解压缩
	/// </summary>
	/// <param name="filepath">文件路径</param>
	/// <param name="cloud_out">输出点云</param>
	void compressionDecode(std::string filepath, PointCloudT::Ptr& cloud_out) {
		pcl::io::OctreePointCloudCompression<pcl::PointXYZ>* PointCloudDecoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZ>();

		ifstream fin(filepath, ios::binary | ios::out);
		stringstream out;
		copy(istreambuf_iterator<char>(fin),
			istreambuf_iterator<char>(),
			ostreambuf_iterator<char>(out));
		fin.close();

		PointCloudDecoder->decodePointCloud(out, cloud_out);
		delete (PointCloudDecoder);
	}

	/// <summary>
	/// 点云压缩
	/// </summary>
	/// <param name="cloud_in">输入点云</param>
	/// <param name="filepath">文件路径</param>
	void compressionEncode(std::string filepath, PointCloudT::Ptr cloud_in) {

		//设置压缩选项为：分辨率1立方厘米，有颜色，快速在线编码
		pcl::io::compression_Profiles_e compressionProfile = pcl::io::LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;
		bool showStatistics = true;
		double pointResolution = 0.001;
		double octreeResolution = 0.01;
		bool doVoxelGridDownDownSampling = false;
		unsigned int iFrameRate = 30;
		bool doColorEncoding = true;
		unsigned char colorBitResolution = 6;

		//初始化压缩和解压缩对象  其中压缩对象需要设定压缩参数选项，解压缩按照数据源自行判断
		pcl::io::OctreePointCloudCompression<pcl::PointXYZ>* PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZ>(compressionProfile, showStatistics, pointResolution, octreeResolution, doVoxelGridDownDownSampling, iFrameRate, doColorEncoding, colorBitResolution);

		std::stringstream compressedData;
		PointCloudEncoder->encodePointCloud(cloud_in->makeShared(), compressedData);

		fstream file(filepath, ios::binary | ios::out);
		file << compressedData.str();
		file.close();

		//删除压缩与解压缩的实例
		delete (PointCloudEncoder);
	}

	/// <summary>
	/// 八叉树点云变化检测
	/// </summary>
	/// <param name="cloud_in1">输入点云1</param>
	/// <param name="cloud_in2">输入点云2</param>
	/// <param name="resolution">八叉树分辨率，体素边长</param>
	/// <param name="point_out">新的点的索引</param>
	void octreeChangeDetector(PointCloudT::Ptr cloud_in1, PointCloudT::Ptr cloud_in2, float resolution, std::vector<int>& indices_out) {
		// 创建八叉树点云变化检测器
		pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(resolution);
		// 添加CloudA的点到八叉树
		octree.setInputCloud(cloud_in1);
		octree.addPointsFromInputCloud();
		//切换缓冲区: 重置octree，但是保存之前内存中的树结构
		octree.switchBuffers();
		// 添加CloudB的点到octree
		octree.setInputCloud(cloud_in2);
		octree.addPointsFromInputCloud();
		// 获的新的点的索引
		octree.getPointIndicesFromNewVoxels(indices_out);
	}
}


#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/boundary.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>


namespace PCLFeatureHelper {
	/// <summary>
	/// 法线计算
	/// </summary>
	/// <param name="cloud_in">输入点云</param>
	/// <param name="radius">半径</param>
	/// <param name="normals_out">输出点云</param>
	void normalEstimationFeatures(PointCloudT::Ptr cloud_in, double radius, pcl::PointCloud<pcl::Normal>::Ptr& normals_out) {
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		ne.setInputCloud(cloud_in);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud(cloud_in);
		ne.setSearchMethod(tree);
		//设置搜索时所用的球半径,参数radius为设置搜索球体半径
		ne.setRadiusSearch(radius);
		//设置搜索时近邻的来源点云,参数cloud指向搜索时被搜索的点云对象,常常用稠密的原始点云,而不用稀疏的下采样后的点云,在特征描述和提取中查询点往往是关键点
		//ne.setSearchSurface(overlap_cloud);
		//设置搜索时所用的k近邻个数,参数k为设置搜索近邻个数
		//ne.setKSearch(k);
		ne.compute(*normals_out);
	}

	/// <summary>
	/// fpfh计算
	/// </summary>
	/// <param name="cloud_in">输入点云</param>
	/// <param name="normals_in">输入法线</param>
	/// <param name="radius">半径</param>
	/// <param name="features_out">输出特征</param>
	void fpfhEstimationFeatures(PointCloudT::Ptr cloud_in, pcl::PointCloud<pcl::Normal>::Ptr normals_in, pcl::PointCloud<pcl::FPFHSignature33>::Ptr& features_out, double radius) {
		pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfhe;
		fpfhe.setInputCloud(cloud_in);
		fpfhe.setInputNormals(normals_in);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud(cloud_in);
		fpfhe.setSearchMethod(tree);

		//设置搜索时所用的球半径,参数radius为设置搜索球体半径
		fpfhe.setRadiusSearch(radius);
		//设置搜索时近邻的来源点云,参数cloud指向搜索时被搜索的点云对象,常常用稠密的原始点云,而不用稀疏的下采样后的点云,在特征描述和提取中查询点往往是关键点
		//fpfhe.setSearchSurface(overlap_cloud);
		//设置搜索时所用的k近邻个数,参数k为设置搜索近邻个数
		//fpfhe.setKSearch(k);
		
		//设置SPFH的统计区间个数。
		//fpfhe.setNrSubdivisions(nr_bins_f1, nr_bins_f2, nr_bins_f3);
		fpfhe.compute(*features_out);
	}

	/// <summary>
	/// 轮廓计算
	/// </summary>
	/// <param name="cloud_in">输入点云</param>
	/// <param name="normals_in">输入法线</param>
	/// <param name="radius">半径</param>
	/// <param name="angle">角度</param>
	/// <param name="cloud_out">输出点云</param>
	void boundaryEstimationFeatures(PointCloudT::Ptr cloud_in, pcl::PointCloud<pcl::Normal>::Ptr normals_in, PointCloudT::Ptr& cloud_out, double radius, float angle) {
		pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> be;
		be.setInputCloud(cloud_in);
		be.setInputNormals(normals_in);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud(cloud_in);
		//设置搜索时所用的球半径,参数radius为设置搜索球体半径
		be.setRadiusSearch(radius);
		//设置搜索时近邻的来源点云,参数cloud指向搜索时被搜索的点云对象,常常用稠密的原始点云,而不用稀疏的下采样后的点云,在特征描述和提取中查询点往往是关键点
		//be.setSearchSurface(overlap_cloud);
		//设置搜索时所用的k近邻个数,参数k为设置搜索近邻个数
		//be.setKSearch(k);

		//设置将点标记为边界或规则的决定边界(角度阈值)。
		be.setAngleThreshold(angle);
		pcl::PointCloud<pcl::Boundary>::Ptr boundarys(new pcl::PointCloud<pcl::Boundary>());
		be.compute(*boundarys);
		for (size_t i = 0; i < cloud_in->size(); i++)
		{
			if ((*boundarys)[i].boundary_point > 0) {
				cloud_out->points.push_back(cloud_in->points[i]);
			}
		}
	}

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
		int k) {
		pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> scia;
		scia.setInputSource(cloud_src_in);
		scia.setInputTarget(cloud_tgt_in);
		scia.setSourceFeatures(fpfhs_src_in);
		scia.setTargetFeatures(fpfhs_tgt_in);
		scia.setMaximumIterations(maximum_iterations);// 最大迭代次数
		scia.setMinSampleDistance(distance);//设置样本间的最小距离
		scia.setNumberOfSamples(number);//设置每次迭代中使用的样本数量

		//设置计算协方差时选择附近多少个点为邻居:设置的点的数量k越高,协方差计算越准确但也会相应降低计算速度
		scia.setCorrespondenceRandomness(k);

		PointCloudT::Ptr sac_result(new PointCloudT);
		scia.align(*sac_result);
		std::cout << "sac has converged:" << scia.hasConverged() << " score: " << scia.getFitnessScore() << endl;
		sac_trans_out = scia.getFinalTransformation();
		std::cout << sac_trans_out << endl;
	}

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
		double transformation_epsilon,
		double euclidean_fitness_epsilon,
		bool use_reciprocal,
		Eigen::Matrix4f sac_trans) {
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		icp.setInputSource(cloud_src_in);
		icp.setInputTarget(cloud_tgt_in);

		//最大迭代次数
		icp.setMaximumIterations(naximum_iterations);
		//两次变化矩阵之间的差值
		icp.setTransformationEpsilon(transformation_epsilon);
		//均方误差
		icp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);
		//设置是否使用倒数对应
		icp.setUseReciprocalCorrespondences(use_reciprocal);
		PointCloudT::Ptr icp_result(new PointCloudT);
		Eigen::Matrix4f sac_trans_null = {};
		if (sac_trans == sac_trans_null) {
			icp.align(*icp_result);
		}
		else
		{
			icp.align(*icp_result, sac_trans);
		}

		std::cout << "ICP has converged:" << icp.hasConverged()
			<< " score: " << icp.getFitnessScore() << std::endl;

		icp_trans_out = icp.getFinalTransformation();
		std::cout << icp_trans_out << endl;
	}
}

