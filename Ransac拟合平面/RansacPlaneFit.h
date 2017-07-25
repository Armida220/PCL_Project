#pragma once
#include "new3s_PointXYZ.h"
#include <vector>

class RansacPlaneFit
{
public:
	RansacPlaneFit();
	~RansacPlaneFit();
	void setInputCloud(const std::vector<new3s_PointXYZ> &clouds);

	// 设置距离阈值
	void setThreshValue(const double threshvalue);

	// 设置最大迭代次数
	void setIteration(const int iters);

	// 计算平面系数
	Eigen::Matrix<double, 4, 1> computePlaneCoef();



	bool getSample(std::vector<int> &samples);

	// 三点不共线为条件判断选择点是否可行
	bool isGoodSample(const std::vector<int> &indices);

	// 计算点到直线的距离
	double computeDist(const int index, const Eigen::Matrix4Xd &coefs);

	// 计算平面的系数
	bool computeModelCoefficients(const std::vector<int> &samples, Eigen::Matrix<double , 4 , 1> &model_coefficient);

	// 统计在阈值范围内到平面的点数，并记录索引
	int countNum(const Eigen::Matrix4Xd &model_coef , std::vector<int> &indices);
	
	// 计算点云中心
	void compute3DCentroid(const std::vector<int> &indices ,  Eigen::Matrix<double, 4, 1> &centroid);

	// 计算协方差矩阵
	void computeCovarianceMatrix(const std::vector<int> &indices, const Eigen::Matrix<double, 4, 1> &centroid, Eigen::Matrix<double, 3, 3> &covariance_matrix);

	// 设置inlier和outlier的比率
	void setRatio(const double ratio);

	// 获得迭代次数
	int getIteration() const;

	// 获得距离阈值
	double getThreshValue() const;


private:
	bool isSetThreshValue() const;
private:
	// 点数
	int m_num;
	std::vector<new3s_PointXYZ> m_cloud;
	double m_threshValue;

	// 设置迭代次数
	int m_iters;

	// 记录程序迭代的次数
	int m_iteration;

	// 保存采样索引
	std::vector<int> m_samples;

	// 全局索引
	std::vector<int> m_globalindexs;

	// inlier和outlier比率
	double m_ratio;

	// 是否设置阈值标记
	bool m_threshFlag;
};

