#include "new3s_PointXYZ.h"

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