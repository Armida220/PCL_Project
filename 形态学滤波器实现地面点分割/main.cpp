#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointIndicesPtr ground(new pcl::PointIndices);

	pcl::PCDReader reader;
	reader.read<pcl::PointXYZ>("samp11-utm.pcd", *cloud);

	std::cerr << "Cloud before filtering: " << std::endl;
	std::cerr << *cloud << std::endl;

	// ������̬ѧ�˲�������
	pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
	pmf.setInputCloud(cloud);

	// ���ù��˵����Ĵ��ڳߴ�
	pmf.setMaxWindowSize(20);

	// ���ü���߶���ֵ��б��ֵ
	pmf.setSlope(1.0f);

	// ���ó�ʼ�߶Ȳ�������Ϊ�ǵ����
	pmf.setInitialDistance(0.5f);

	// ���ñ���Ϊ�ǵ��������߶�
	pmf.setMaxDistance(3.0f);
	pmf.extract(ground->indices);

	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(ground);
	extract.filter(*cloud_filtered);

	std::cerr << "Ground cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_RGB(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_filtered_RGB->resize(cloud_filtered->size());
	cloud_filtered_RGB->is_dense = FALSE;

	for (size_t i = 0 ; i < cloud_filtered->points.size() ; ++i)
	{
		cloud_filtered_RGB->points[i].x = cloud_filtered->points[i].x;
		cloud_filtered_RGB->points[i].y = cloud_filtered->points[i].y;
		cloud_filtered_RGB->points[i].z = cloud_filtered->points[i].z;

		cloud_filtered_RGB->points[i].r = 0;
		cloud_filtered_RGB->points[i].g = 255;
		cloud_filtered_RGB->points[i].b = 0;
	}

	pcl::io::savePCDFileBinary("cloud_groud.pcd", *cloud_filtered_RGB);

	// ��ȡ�ǵ����
	extract.setNegative(true);
	extract.filter(*cloud_filtered);

	std::cerr << "Object cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;

	pcl::io::savePCDFileBinary("No_groud.pcd", *cloud_filtered);

	return (0);
}