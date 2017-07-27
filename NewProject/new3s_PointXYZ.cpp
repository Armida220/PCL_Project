#include "new3s_PointXYZ.h"
#include <vector>
bool cmpX(const new3s_PointXYZ &pt1 , const new3s_PointXYZ &pt2)
{
	return pt1.get_x() < pt2.get_x() ;
}

bool cmpY(const new3s_PointXYZ &pt1 , const new3s_PointXYZ &pt2)
{
	return pt1.get_y() < pt2.get_y() ;
}

bool cmpZ(const new3s_PointXYZ &pt1 , const new3s_PointXYZ &pt2)
{
	return pt1.get_z() < pt2.get_z() ;
}

int judgeDirection(const std::vector<new3s_PointXYZ> &line_cloud)
{
	double xmin = 100000000 , ymin = 100000000 , xmax = 0 , ymax = 0 ;
	size_t nums = line_cloud.size() ;
	for (int i = 0 ; i < nums ; ++i)
	{
		if (line_cloud[i].get_x() > xmax)
		{
			xmax = line_cloud[i].get_x() ;
		}
		else if (line_cloud[i].get_x() < xmin)
		{
			xmin = line_cloud[i].get_x() ;
		}
		else if (line_cloud[i].get_y() > ymax)
		{
			ymax = line_cloud[i].get_y() ;
		}
		else if (line_cloud[i].get_y() < ymin)
		{
			ymin = line_cloud[i].get_y() ;
		}
	}

	double dx = xmax - xmin ;
	double dy = ymax - ymin ;

	if (dx > dy)
	{
		return 1 ;
	}
	else if (dx <= dy)
	{
		return 2 ;
	}

	return 0 ;
}