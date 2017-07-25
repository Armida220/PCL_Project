#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>

int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr convexHull(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr objects(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("table_scene_mug_stereo_textured.pcd", *cloud) != 0)
	{
		return -1;
	}

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

	// 采样一致性算法实现的分割类
	pcl::SACSegmentation<pcl::PointXYZ> segmentation;
	segmentation.setInputCloud(cloud);
	// 设置构造的几何模型类型
	segmentation.setModelType(pcl::SACMODEL_PLANE);
	// 设置采样一致性方法类型
	segmentation.setMethodType(pcl::SAC_RANSAC);
	// 设置点到模型的距离阈值
	segmentation.setDistanceThreshold(0.01);
	// 设置模型参数优化
	segmentation.setOptimizeCoefficients(true);
	pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
	segmentation.segment(*planeIndices, *coefficients);

	if (planeIndices->indices.size() == 0)
		std::cout << "Could not find a plane in the scene." << std::endl;
	else
	{
		// Copy the points of the plane to a new cloud.
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(planeIndices);
		extract.filter(*plane);

		// 保存平面点云
		pcl::io::savePCDFile("plane.pcd",*plane);

		// 生成凸包
		pcl::ConvexHull<pcl::PointXYZ> hull;
		hull.setInputCloud(plane);
		hull.setDimension(2);
		hull.reconstruct(*convexHull);

		// 冗余检查.
		if (hull.getDimension() == 2)
		{
			// 该类用于分割出棱柱模型内部的点集
			pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
			prism.setInputCloud(cloud);
			// 设置平面模型的点集
			prism.setInputPlanarHull(convexHull);

			// 设置高度范围
			prism.setHeightLimits(0.01f, 0.4f);
			pcl::PointIndices::Ptr objectIndices(new pcl::PointIndices);

			prism.segment(*objectIndices);

			// Get and show all points retrieved by the hull.
			extract.setIndices(objectIndices);
			extract.filter(*objects);
			pcl::visualization::CloudViewer viewerObjects("Objects on table");
			viewerObjects.showCloud(objects);
			pcl::io::savePCDFile("objects.pcd", *objects);
			while (!viewerObjects.wasStopped())
			{
				// Do nothing but wait.
			}
		}
		else std::cout << "The chosen hull is not planar." << std::endl;
	}
}