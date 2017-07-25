#include "NearestSearch.h"
#include "new3s_PointXYZ.h"
#include <iostream>
#include <vector>

void main()
{
	std::vector<new3s_PointXYZ> vec_cloud;
	new3s_PointXYZ pointxyz;
	for (int i = 0 ; i < 100 ; ++i)
	{
		pointxyz.set_x(1024.0f * rand() / (RAND_MAX + 1.0f));
		pointxyz.set_y(1024.0f * rand() / (RAND_MAX + 1.0f));
		pointxyz.set_z(1024.0f * rand() / (RAND_MAX + 1.0f));
		vec_cloud.push_back(pointxyz);
	}

	NearestSearch nearestsearch;
	nearestsearch.setInputCloud(vec_cloud);

	new3s_PointXYZ pt(50.0f, 50.0f, 50.0f);
	std::vector<int> indices;
	std::vector<float> dists;
	int num = nearestsearch.nearestKSearch(pt, 3, indices , dists);
	for (int i = 0 ; i < indices.size() ; ++i)
	{
		std::cout << vec_cloud[indices[i]].get_x() << "," << vec_cloud[indices[i]].get_y() << "," << vec_cloud[indices[i]].get_z() << "\n";
	}

	std::cout << "最近距离：\n";
	for (int i = 0 ; i < dists.size() ; ++i)
	{
		std::cout << dists[i] << ",";
	}
	std::cout << "完成!\n";
}