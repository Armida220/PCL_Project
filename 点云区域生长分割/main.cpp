#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>

int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile <pcl::PointXYZ>("region_growing_tutorial.pcd", *cloud) == -1)
	{
		std::cout << "Cloud reading failed." << std::endl;
		return (-1);
	}

	pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(cloud);
	normal_estimator.setKSearch(50);
	normal_estimator.compute(*normals);

	pcl::IndicesPtr indices(new std::vector <int>);
	// ��������ĳ���ֶ��޶��ĶԵ��ƽ��м򵥵Ĺ���
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 1.0);
	pass.filter(*indices);
	
	// �����������Ʒָ���
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;

	// �趨ÿ���������ٵ���
	reg.setMinClusterSize(50);
	reg.setMaxClusterSize(1000000);
	reg.setSearchMethod(tree);
	// �������������
	reg.setNumberOfNeighbours(30);
	reg.setInputCloud(cloud);
	
	reg.setInputNormals(normals);

	// ����ƽ����ֵ��С
	reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
	// ����������ֵ
	reg.setCurvatureThreshold(1.0);

	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);

	std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
	std::cout << "First cluster has " << clusters[0].indices.size() << " points." << endl;

	// ������Ƴɹ��ָ���ز�ɫ���ƣ����򷵻�Ϊ��
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
	if (colored_cloud->empty())
	{
		std::cout << "����ָ����ʧ��" << std::endl;
		return -1;
	}
	else
	{
		pcl::io::savePCDFile("colored_cloud.pcd", *colored_cloud);
	}

	return (0);
}