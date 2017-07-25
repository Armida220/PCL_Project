/*********************************************
// Copyright:北京星闪视图科技有限公司
// Author: 韩硕
// Date:2016-08-02
// Description:本头文件主要用来添加项目所需的结构体
*************************************************/
#pragma once
#include <ostream>
#include <iomanip>
#include <Eigen/eigen3/Eigen/Eigen>
//三维点结构体
class new3s_PointXYZ
{
public:
	new3s_PointXYZ():x(0),y(0),z(0)
	{

	}
	new3s_PointXYZ(double x_ , double y_ , double z_):x(x_),y(y_),z(z_)
	{

	}

	new3s_PointXYZ(const new3s_PointXYZ &other)
	{
		*this = other ;
	}

	virtual ~new3s_PointXYZ()
	{

	}
	
	new3s_PointXYZ &operator=(const new3s_PointXYZ &other)
	{
		x = other.x ;
		y = other.y ;
		z = other.z ;
		return *this ;
	}

	//从new3s_PointXYZ中获取坐标值
	double get_x()const{return x;}
	double get_y()const{return y;}
	double get_z()const{return z;}
	void set_x(double x_)
	{
		x = x_ ;
	}
	void set_y(double y_)
	{
		y = y_ ;
	}
	void set_z(double z_)
	{
		z = z_ ;
	}
	bool operator <(const new3s_PointXYZ &pt)
	{
		return this->y < pt.get_y() ;
	}
	friend std::ostream &operator<< (std::ostream &os , const new3s_PointXYZ &pt)
	{
		os<<std::fixed<<std::setprecision(3) ;
		os<<"("<<pt.get_x()<<","<<pt.get_y()<<","<<pt.get_z()<<")" ;
		return os ;
	}
	inline Eigen::Vector4d getVector4d()
	{
		Eigen::Vector4d vec4d(x , y , z , 0) ;
		return vec4d ;
	}
private:
	double x , y , z ;
};

bool cmpZ(const new3s_PointXYZ &pt1 , const new3s_PointXYZ &pt2) ;

bool cmpX(const new3s_PointXYZ &pt1 , const new3s_PointXYZ &pt2) ;

bool cmpY(const new3s_PointXYZ &pt1 , const new3s_PointXYZ &pt2) ;
