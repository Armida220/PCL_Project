#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <vector>
#include <ctime>

using namespace pcl;
using namespace std;
void main()
{

	srand(time(NULL));
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
	cloud->width = 1000;
	cloud->height = 1;
	cloud->resize(cloud->width*cloud->height);
	for (size_t i = 0; i < cloud->size(); ++i)
	{
		cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}
	KdTreeFLANN <PointXYZ> kdtree;//创建kdtree对象
	kdtree.setInputCloud(cloud);//设置搜索空间
	PointXYZ searchPoint;//定义查询点并设随机值
	searchPoint.x = 1024 * rand() / (RAND_MAX + 1.0f);
	searchPoint.y = 1024 * rand() / (RAND_MAX + 1.0f);
	searchPoint.z = 1024 * rand() / (RAND_MAX + 1.0f);

	cout << "K近邻搜索：\n";
	int K = 10;
	vector<int> pointIndexNKNSearch(K);//存储查询点近邻索引
	vector<float> pointNKNSquaredDistance(K);
	cout << "K nearest neighbor search at (" << searchPoint.x << " " << searchPoint.y << " " << searchPoint.z << ")with K" << K << endl;
	if (kdtree.nearestKSearch(searchPoint, K, pointIndexNKNSearch, pointNKNSquaredDistance) > 0)
	{
		for (size_t i = 0; i < pointIndexNKNSearch.size(); ++i)
		{
			cout << " " << cloud->points[pointIndexNKNSearch[i]].x << " " << cloud->points[pointIndexNKNSearch[i]].y << " " << cloud->points[pointIndexNKNSearch[i]].z
				<< "(squrared distance:" << pointNKNSquaredDistance[i] << ")" << endl;
		}
	}
	//半径r内近邻搜索
	cout << "半径r内近邻搜索：\n";
	vector<int> pointIdxRadiusSearch;
	vector<float> pointRadiusSquaredDistance;//搜索近邻对应的距离平方
	float radius = 256.0f*rand() / (RAND_MAX + 1.0f);
	if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointNKNSquaredDistance) > 0)
	{
		for (size_t i = 0; i < pointIndexNKNSearch.size(); ++i)
		{
			cout << " " << cloud->points[pointIndexNKNSearch[i]].x << " " << cloud->points[pointIndexNKNSearch[i]].y << " " << cloud->points[pointIndexNKNSearch[i]].z <<
				"(square distance:" << pointIndexNKNSearch[i] << ")" << endl;
		}
	}
	system("pause");

}