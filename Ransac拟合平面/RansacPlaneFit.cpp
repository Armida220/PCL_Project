#include "RansacPlaneFit.h"
#include <Eigen/eigen3/Eigen/Eigen>
#include <math.h>
#include <iostream>

RansacPlaneFit::RansacPlaneFit()
	: m_threshValue(0.0)
	, m_iters(1000)
	, m_num(0)
	, m_ratio(2.0)
	, m_threshFlag(false)
	, m_iteration(0)
{
}


RansacPlaneFit::~RansacPlaneFit()
{
	if (!m_cloud.empty())
	{
		m_cloud.clear();
		std::vector<new3s_PointXYZ>().swap(m_cloud);
	}
}

void RansacPlaneFit::setInputCloud(const std::vector<new3s_PointXYZ>& clouds)
{
	if (!clouds.empty())
	{
		m_cloud = clouds;

		m_num= m_cloud.size();
		for (int i = 0 ; i < m_num; ++i)
		{
			m_globalindexs.push_back(i);
		}
	}
}

void RansacPlaneFit::setThreshValue(const double threshvalue)
{
	m_threshValue = threshvalue;
	m_threshFlag = true;
}

void RansacPlaneFit::setIteration(const int iters)
{
	m_iters = iters;
}

bool RansacPlaneFit::getSample(std::vector<int>& samples)
{
	samples.resize(3);
	srand(unsigned(time(0)));
	int id1 = rand() % m_globalindexs.size();
	int id2 = rand() % m_globalindexs.size();
	int id3 = rand() % m_globalindexs.size();
	samples[0] = id1;
	samples[1] = id2;
	samples[2] = id3;
	if (isGoodSample(samples))
	{
		return true;
	}

	return false;
}


Eigen::Matrix<double, 4, 1> RansacPlaneFit::computePlaneCoef()
{
	Eigen::Matrix<double , 4 , 1> coef ;
	coef.setZero();

	// 记录迭代次数
	int iter = 0;
	// 记录内点数
	int max_best_num = 0;

	std::vector<int> samples;

	// 统计每次迭代到拟合平面点的数量
	std::vector<int> best_nums;
	std::vector<std::vector<int> > inlier_indices;
	inlier_indices.resize(m_iters);
	double ratio = 0;
	while (iter < m_iters)
	{
		++iter;
		if (getSample(samples))
		{
			computeModelCoefficients(samples, coef);
			ratio = (countNum(coef, inlier_indices[iter - 1]) / (double)(m_num - inlier_indices[iter - 1].size()));
			std::cout << "inlier和outlier的比率：" << ratio << std::endl;
			if (ratio > m_ratio)
			{
				break;
			}
		}

		samples.clear();
		std::vector<int>().swap(samples);
	}

	m_iteration = iter;

// 	std::sort(inlier_indices.begin(), inlier_indices.end());
 	best_nums.resize(inlier_indices[iter - 1].size());
	std::copy(inlier_indices[iter - 1].begin(), inlier_indices[iter - 1].end(), best_nums.begin());

	Eigen::Matrix<double, 4, 1> xyz_centroid;
	xyz_centroid.setZero();
	compute3DCentroid(best_nums , xyz_centroid);

	Eigen::Matrix3d covariance_matrix;
	covariance_matrix.setZero();
	computeCovarianceMatrix(best_nums, xyz_centroid, covariance_matrix);

	EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
	EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
	//计算协方差矩阵最小特征值的特征向量
	Eigen::EigenSolver<Eigen::Matrix3d> CovarianceMatrix(covariance_matrix);
	Eigen::Matrix3d D = CovarianceMatrix.pseudoEigenvalueMatrix();
	Eigen::Matrix3d V = CovarianceMatrix.pseudoEigenvectors();

	std::vector<double> eval;//保存特征值
	for (int i = 0; i < 3; ++i)
	{
		eval.push_back(D(i, i));
	}
	int k_val = 0;//记录特征值最小的是哪个
	double min_val = 100000000;
	for (int i = 0; i < 3; ++i)
	{
		if (eval[i] < min_val)
		{
			min_val = eval[i];
			k_val = i;
		}
	}

	for (int i = 0; i < 3; ++i)
	{
		eigen_vector[i] = V(i, k_val);
	}

	coef[0] = eigen_vector[0];
	coef[1] = eigen_vector[1];
	coef[2] = eigen_vector[2];
	coef[3] = 0;
	coef[3] = -1 * coef.dot(xyz_centroid);

	if (coef[3] != 0)
	{
		coef[0] /= coef[3];
		coef[1] /= coef[3];
		coef[2] /= coef[3];
		coef[3] = 1.0;
	}

	return coef;
}

bool RansacPlaneFit::isGoodSample(const std::vector<int>& indices)
{
	if (indices.size() < 3)
	{
		return false;
	}

	Eigen::Vector3d p0p1 = Eigen::Vector3d(m_cloud[indices[1]].get_x() - m_cloud[indices[0]].get_x(), m_cloud[indices[1]].get_y() - m_cloud[indices[0]].get_y(), m_cloud[indices[1]].get_z() - m_cloud[indices[0]].get_z());
	Eigen::Vector3d p0p2 = Eigen::Vector3d(m_cloud[indices[2]].get_x() - m_cloud[indices[0]].get_x(), m_cloud[indices[2]].get_y() - m_cloud[indices[2]].get_y(), m_cloud[indices[1]].get_z() - m_cloud[indices[0]].get_z());

	if (p0p1(0) * p0p2(1) == p0p2(0) * p0p1(1) && p0p1(1) * p0p2(2) == p0p2(1) * p0p1(2))
	{
		return false;
	}

	return true;
}

double RansacPlaneFit::computeDist(const int index, const Eigen::Matrix4Xd & coefs)
{
	double a = coefs(0, 0);
	double b = coefs(1, 0);
	double c = coefs(2, 0);
	double d = coefs(3, 0);

	double x = m_cloud[index].get_x();
	double y = m_cloud[index].get_y();
	double z = m_cloud[index].get_z();

	return fabs((x * a + y * b + z * c + d) / (a * a + b * b + c * c));
}

bool RansacPlaneFit::computeModelCoefficients(const std::vector<int>& samples, Eigen::Matrix<double, 4, 1>& model_coefficient)
{
	if (samples.size() != 3)
	{
		std::cout << "采样点数量不足3！\n";
		return (false);
	}


	new3s_PointXYZ p0 = m_cloud[samples[0]];
	new3s_PointXYZ p1 = m_cloud[samples[1]];
	new3s_PointXYZ p2 = m_cloud[samples[2]];

	Eigen::Vector3d p0p1(p1.get_x() - p0.get_x(), p1.get_y() - p0.get_y(), p1.get_z() - p0.get_z());
	Eigen::Vector3d p0p2(p2.get_x() - p0.get_x(), p2.get_y() - p0.get_y(), p2.get_z() - p0.get_z());

	// 利用向量的叉积求法向量
	model_coefficient.resize(4);
	model_coefficient[0] = p0p1[1] * p0p2[2] - p0p2[1] * p0p1[2];
	model_coefficient[1] = p0p2[0] * p0p1[2] - p0p1[0] * p0p2[2];
	model_coefficient[2] = p0p1[0] * p0p2[1] - p0p2[0] * p0p1[1];
	model_coefficient[3] = 0;

	model_coefficient.normalize(); //模型系数单位化
	// ... + d = 0
	model_coefficient[3] = -(model_coefficient[0] * p0.get_x() + model_coefficient[1] * p0.get_y() + model_coefficient[2] * p0.get_z());

	return (true);
}


int RansacPlaneFit::countNum(const Eigen::Matrix4Xd &model_coef, std::vector<int> &indices)
{
	double d = 0;
	if (isSetThreshValue() != true) //如果没有设置阈值，自适应产生距离阈值
	{
		double adatp_threshVal = 0;
		std::vector<double> vec_d;
		double aved = 0;
		for (unsigned int i = 0; i < m_num; ++i)
		{
			d = computeDist(m_globalindexs[i], model_coef);
			aved += d;
			vec_d.push_back(d);
		}
		aved /= m_num;
		for (size_t i = 0; i < m_num; ++i)
		{
			adatp_threshVal += (vec_d[i] - aved) * (vec_d[i] - aved);
		}
		adatp_threshVal /= m_num;
		adatp_threshVal = std::sqrt(adatp_threshVal);
		m_threshValue = adatp_threshVal;
	}

	for (size_t i = 0; i < m_num; ++i)
	{
		d = computeDist(m_globalindexs[i], model_coef);
		if (d < m_threshValue)
		{
			indices.push_back(i);
		}
	}

	return indices.size();
}

void RansacPlaneFit::compute3DCentroid(const std::vector<int>& indices, Eigen::Matrix<double, 4, 1>& centroid)
{
	if (indices.empty())
	{
		std::cout << "索引文件为空!\n";
		return;
	}
	centroid.setZero();
	int num = indices.size();
	for (int i = 0 ; i < num ; ++i)
	{
		centroid[0] += m_cloud[indices[i]].get_x();
		centroid[1] += m_cloud[indices[i]].get_y();
		centroid[2] += m_cloud[indices[i]].get_z();
	}
	centroid /= static_cast<double>(indices.size());
	centroid[3] = 1;
}

void RansacPlaneFit::computeCovarianceMatrix(const std::vector<int>& indices, const Eigen::Matrix<double, 4, 1>& centroid, Eigen::Matrix<double, 3, 3>& covariance_matrix)
{
	if (indices.empty())
	{
		std::cout << "索引为空！\n";
		return ;
	}
	covariance_matrix.setZero();
	size_t num = indices.size();
	for (size_t i = 0; i < num; i++)
	{
		Eigen::Matrix<double, 4, 1> pt;
		pt[0] = m_cloud[indices[i]].get_x() - centroid[0];
		pt[1] = m_cloud[indices[i]].get_y() - centroid[1];
		pt[2] = m_cloud[indices[i]].get_z() - centroid[2];

		covariance_matrix(1, 1) += pt.y() * pt.y();
		covariance_matrix(1, 2) += pt.y() * pt.z();
		covariance_matrix(2, 2) += pt.z() * pt.z();

		pt *= pt.x();
		covariance_matrix(0, 0) += pt.x();
		covariance_matrix(0, 1) += pt.y();
		covariance_matrix(0, 2) += pt.z();
	}

	covariance_matrix(1, 0) = covariance_matrix(0, 1);
	covariance_matrix(2, 0) = covariance_matrix(0, 2);
	covariance_matrix(2, 1) = covariance_matrix(1, 2);
}


void RansacPlaneFit::setRatio(const double ratio)
{
	m_ratio = ratio;
}

int RansacPlaneFit::getIteration() const
{
	return m_iteration;
}

double RansacPlaneFit::getThreshValue() const
{
	return m_threshValue;
}

bool RansacPlaneFit::isSetThreshValue() const
{
	return m_threshFlag;
}


