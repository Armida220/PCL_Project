#pragma once
#include "new3s_PointXYZ.h"
#include <vector>

class RansacPlaneFit
{
public:
	RansacPlaneFit();
	~RansacPlaneFit();
	void setInputCloud(const std::vector<new3s_PointXYZ> &clouds);

	// ���þ�����ֵ
	void setThreshValue(const double threshvalue);

	// ��������������
	void setIteration(const int iters);

	// ����ƽ��ϵ��
	Eigen::Matrix<double, 4, 1> computePlaneCoef();



	bool getSample(std::vector<int> &samples);

	// ���㲻����Ϊ�����ж�ѡ����Ƿ����
	bool isGoodSample(const std::vector<int> &indices);

	// ����㵽ֱ�ߵľ���
	double computeDist(const int index, const Eigen::Matrix4Xd &coefs);

	// ����ƽ���ϵ��
	bool computeModelCoefficients(const std::vector<int> &samples, Eigen::Matrix<double , 4 , 1> &model_coefficient);

	// ͳ������ֵ��Χ�ڵ�ƽ��ĵ���������¼����
	int countNum(const Eigen::Matrix4Xd &model_coef , std::vector<int> &indices);
	
	// �����������
	void compute3DCentroid(const std::vector<int> &indices ,  Eigen::Matrix<double, 4, 1> &centroid);

	// ����Э�������
	void computeCovarianceMatrix(const std::vector<int> &indices, const Eigen::Matrix<double, 4, 1> &centroid, Eigen::Matrix<double, 3, 3> &covariance_matrix);

	// ����inlier��outlier�ı���
	void setRatio(const double ratio);

	// ��õ�������
	int getIteration() const;

	// ��þ�����ֵ
	double getThreshValue() const;


private:
	bool isSetThreshValue() const;
private:
	// ����
	int m_num;
	std::vector<new3s_PointXYZ> m_cloud;
	double m_threshValue;

	// ���õ�������
	int m_iters;

	// ��¼��������Ĵ���
	int m_iteration;

	// �����������
	std::vector<int> m_samples;

	// ȫ������
	std::vector<int> m_globalindexs;

	// inlier��outlier����
	double m_ratio;

	// �Ƿ�������ֵ���
	bool m_threshFlag;
};

