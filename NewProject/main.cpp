#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <lasreader.hpp>
#include <laswriter.hpp>
#include "new3s_io.h"
#include "new3s_PointXYZRGB.h"
#include <vector>


/*********************************************
Function:       // 函数名称
Description:    // 将三维点转为pcl支持的Pointcloud格式
Input:          // 输入参数说明，vec_cloud输入的点云数据
Output:         // 输出参数说明，out_cloud 为pcl支持的点云数据
Return:         // 成功返回1，否则返回0
Others:         // 其它说明
*************************************************/
inline int convertVectoPointCloud(const std::vector<new3s_PointXYZ> &vec_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud)
{
	if (vec_cloud.empty())
	{
		std::cout << "the point cloud is empty , please check!\n";
		return 0;
	}
	int nums = vec_cloud.size();
	out_cloud->width = nums;
	out_cloud->height = 1;
	out_cloud->is_dense = FALSE;
	out_cloud->resize(nums);

	std::vector<new3s_PointXYZ>::const_iterator it;
	int i = 0;

	for (it = vec_cloud.begin(); it != vec_cloud.end(); ++it, ++i)
	{
		out_cloud->points[i].x = (*it).get_x();
		out_cloud->points[i].y = (*it).get_y();
		out_cloud->points[i].z = (*it).get_z();
	}

	return 1;
}

int convertPointCloudtoVec(const pcl::PointCloud<pcl::PointXYZ> &cloud, std::vector<new3s_PointXYZ> &vec)
{
	new3s_PointXYZ pt;
	int num = cloud.points.size();
	vec.resize(num);
	for (int i = 0 ; i < num ; ++i)
	{
		pt.set_x(cloud.points[i].x);
		pt.set_y(cloud.points[i].y);
		pt.set_z(cloud.points[i].z);

		vec[i] = pt;
	}
	return 1;
}

void main()
{
	std::vector<new3s_PointXYZ> vec_cloud;
	new3s_io io_act;
	io_act.lasFileRead("1_no6.las", vec_cloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	convertVectoPointCloud(vec_cloud, cloud);
	std::cout << "原始点云数量：" << cloud->points.size() << std::endl;

	// 创建滤波器对象
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(1, 1, 1);

	pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	sor.filter(*filter_cloud);
	std::cout << "滤波后的点云数量：" << filter_cloud->points.size() << std::endl;


	std::vector<new3s_PointXYZ> vec_filter_cloud;
	convertPointCloudtoVec(*filter_cloud, vec_filter_cloud);
	io_act.writeVectoLas(vec_filter_cloud, 255, 255, 255, "filter_cloud.las");
}