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
#include <vector>

struct New3s_PointXY
{
	double X ;
	double Y ;
	New3s_PointXY()
	{

	}
	New3s_PointXY(double X_ , double Y_):X(X_),Y(Y_)
	{

	}
	New3s_PointXY &operator=(const New3s_PointXY &other)
	{
		X = other.X ;
		Y = other.Y ;
		return *this ;
	}

	bool operator <(const New3s_PointXY &pt)
	{
		return this->Y < pt.Y ;
	}
};

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

	new3s_PointXYZ operator -(const new3s_PointXYZ &other)
	{
		x -= other.x ;
		y -= other.y ;
		z -= other.z ;
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

/*********************************************
Function:       // 函数名称
Description:    // 判断电力线的走向
Input:          // 输入参数说明，电力线点云数据
Output:         // 输出参数说明
Return:         // 返回1，则沿着x轴走向，返回2,沿着y走向，返回0，检测失败
Others:         // 其它说明
*************************************************/
int judgeDirection(const std::vector<new3s_PointXYZ> &line_cloud) ;

bool cmpZ(const new3s_PointXYZ &pt1 , const new3s_PointXYZ &pt2) ;

bool cmpX(const new3s_PointXYZ &pt1 , const new3s_PointXYZ &pt2) ;

bool cmpY(const new3s_PointXYZ &pt1 , const new3s_PointXYZ &pt2) ;
