#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <math.h>
#include <vector>
#include "new3s_PointXYZ.h"
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <opencv.hpp>
#include "RansacPlaneFit.h"

void test_pcl(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::vector<int> &indices);

void test_opencv(cv::Mat &mat);

void test_myself(const std::vector<new3s_PointXYZ> &clouds, const std::vector<int> &indices);

void main()
{
	std::vector<new3s_PointXYZ> myself_cloud;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->width = 10000;
	cloud->height = 1;
	cloud->is_dense = FALSE;
	cloud->resize(cloud->width * cloud->height);

	// 生成平面x2 + y2 + z2 = 1的点云数据
	size_t num = cloud->size();
	for (size_t i = 0 ; i < num ; ++i)
	{
		double x = rand() / (RAND_MAX + 1.0);
		double y = rand() / (RAND_MAX + 1.0);
		double z = 1 - (x + y);
		double z_outlier = rand() / (RAND_MAX + 1.0);

		cloud->points[i].x = x;
		cloud->points[i].y = y;

		// 生成局外点
		if (i % 5 == 0)
		{
			cloud->points[i].z = z_outlier ;
			myself_cloud.push_back(new3s_PointXYZ(x , y , z_outlier));
		}
		else
		{
			cloud->points[i].z = 1 - (cloud->points[i].x + cloud->points[i].y);

			myself_cloud.push_back(new3s_PointXYZ(x, y , z));
		}	
	}
//	pcl::io::savePCDFile("1.pcd", *cloud);
	

	// 保存局内点索引
	std::vector<int> inliers;
	// 采样一致性模型对象
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
	ransac.setDistanceThreshold(0.01);
	ransac.computeModel();
	ransac.getInliers(inliers);

	std::cout << "局内点：" << inliers.size() << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
	final->resize(inliers.size());

	pcl::copyPointCloud(*cloud, inliers, *final);
	
	Eigen::VectorXf coef = Eigen::VectorXf::Zero(4 , 1);
	ransac.getModelCoefficients(coef);

	std::cout << "拟合系数：\n";
	std::cout << "a = " << coef[0] << "	b = " << coef[1] << "	c = " <<coef[2]<< "	d = " << coef[3] << "\n";

	RansacPlaneFit ransacfit;
	ransacfit.setInputCloud(myself_cloud);
	Eigen::Matrix<double, 4, 1> myself_coef = ransacfit.computePlaneCoef();
	std::cout << "迭代次数：" << ransacfit.getIteration() << std::endl;
	std::cout << "自适应距离阈值：" << ransacfit.getThreshValue() << std::endl;
	std::cout << "myself 拟合系数:" << std::endl;
	std::cout << myself_coef << std::endl;

}

int main2()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->resize(2);
	cloud->points[0].x = 1;
	cloud->points[0].y = 2;
	cloud->points[0].z = 3;
	cloud->points[1].x = 10;
	cloud->points[1].y = 20;
	cloud->points[1].z = 30;

	std::vector<int> indices;
	indices.push_back(0);
	indices.push_back(1);

	std::cout << "测试pcl:\n";
	test_pcl(cloud, indices);

	std::cout << "测试opencv:\n";
	cv::Mat data = (cv::Mat_<float>(2, 3) << 1, 2, 3, 10, 20, 30);
	test_opencv(data);

	std::cout << "测试myself:\n";
	std::vector<new3s_PointXYZ> vec;
	vec.push_back(new3s_PointXYZ(1 , 2 , 3));
	vec.push_back(new3s_PointXYZ(10 , 20 , 30));
	test_myself(vec , indices);
	return 1;
}

void test_pcl(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::vector<int>& indices)
{
	Eigen::Matrix3d covariance_matrix;
	Eigen::Vector4d xyz_centroid;
	compute3DCentroid(*cloud, indices, xyz_centroid);
//	computeCovarianceMatrix(*cloud, indices, covariance_matrix);

	computeMeanAndCovarianceMatrix(*cloud, indices, covariance_matrix, xyz_centroid);
	std::cout << "中心点坐标:\n";
	std::cout << xyz_centroid << std::endl;
	std::cout << "协方差矩阵:" << std::endl;
	std::cout << covariance_matrix << std::endl;

}

void test_opencv(cv::Mat & mat)
{
	cv::Mat covar, means;
	cv::calcCovarMatrix(mat, covar, means, CV_COVAR_NORMAL | CV_COVAR_ROWS);

	std::cout << "协方差矩阵:" << std::endl;
	std::cout << covar << std::endl;
}

void test_myself(const std::vector<new3s_PointXYZ> &clouds, const std::vector<int> &indices)
{
	RansacPlaneFit ransacfit;
	ransacfit.setInputCloud(clouds);

	Eigen::Matrix<double, 4, 1> xyz_centroid;
	xyz_centroid.setZero();

	ransacfit.compute3DCentroid(indices, xyz_centroid);
	Eigen::Matrix3d covar;
	ransacfit.computeCovarianceMatrix(indices, xyz_centroid, covar);
	std::cout << "中心点坐标:\n" << xyz_centroid << std::endl;
	std::cout << "协方差矩阵:\n"<< covar << std::endl;
}