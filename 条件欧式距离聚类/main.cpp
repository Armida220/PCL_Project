#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <iostream>


//如果此函数返回true，则将添加候选点到种子点的簇类中。
bool
customCondition(const pcl::PointXYZ& seedPoint, const pcl::PointXYZ& candidatePoint, float squaredDistance)
{
	// 在这里你可以添加你自定义的条件
	if (candidatePoint.y < seedPoint.y)
		return false;

	return true;
}

int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0)
	{
		return -1;
	}

	// 申明一个条件聚类的对象
	pcl::ConditionalEuclideanClustering<pcl::PointXYZ> clustering;
	// 设置新类簇的容差
	clustering.setClusterTolerance(0.02);
	clustering.setMinClusterSize(100);
	clustering.setMaxClusterSize(25000);
	clustering.setInputCloud(cloud);
	// 设置要检查每对点的函数。
	clustering.setConditionFunction(&customCondition);
	std::vector<pcl::PointIndices> clusters;
	clustering.segment(clusters);

	// 对于每一个聚类结果
	int currentClusterNum = 1;
	for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
	{
		// ...add all its points to a new cloud...
		pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
			cluster->points.push_back(cloud->points[*point]);
		cluster->width = cluster->points.size();
		cluster->height = 1;
		cluster->is_dense = true;

		// ...and save it to disk.
		if (cluster->points.size() <= 0)
			break;
		std::cout << "Cluster " << currentClusterNum << " has " << cluster->points.size() << " points." << std::endl;
		std::string fileName = "cluster" + boost::to_string(currentClusterNum) + ".pcd";
		pcl::io::savePCDFileASCII(fileName, *cluster);

		currentClusterNum++;
	}
}