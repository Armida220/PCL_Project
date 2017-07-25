/*********************************************
// Copyright:北京星闪视图科技有限公司
// Author: 韩硕、彭博文
// Date:2016-08-02
// Description:最近邻搜索及特征
*************************************************/

#pragma once
#define _SCL_SECURE_NO_WARNINGS 
#include <vector>
#include <flann/flann.hpp>
#include <flann/util/matrix.h>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include "new3s_PointXYZ.h"
#include <boost/shared_ptr.hpp>


class new3s_PointXYZ;
class NearestSearch
{
public:
	typedef std::vector<int> vecInt ;
	typedef std::vector<float> vecFloat ;

	typedef ::flann::L2_Simple<float> Dist ;
	typedef ::flann::Index<Dist> FLANNIndex ;

	enum SearchMethod
	{
		KNN,
		RNN,
		CylindricalNN
	};
	NearestSearch(void);
	virtual ~NearestSearch(void);

	/*********************************************
	Function:       // 函数名称
	Description:    // 最邻近点搜索
	Input:          // 输入参数说明，cloud输入点云
	Output:         // 输出参数说明
	Return:         // 成功返回1 ，失败返回0
	Others:         // 其它说明
	*************************************************/
    int setInputCloud(std::vector<new3s_PointXYZ> &cloud) ;

	/*********************************************
	Function:       // 函数名称
	Description:    // 最邻近点搜索
	Input:          // 输入参数说明，query_point查询点
	Input:          // 输出参数说明，knn 设置近邻个数
	Output:         // 输出参数说明，indice_point 保存查到到的点坐标
	Output:         // 输出参数说明，distance 保存索引点与查询点的距离平方
	Return:         // 成功返回搜索的最近邻个数 ，失败返回0
	Others:         // 其它说明
	*************************************************/
    int nearestKSearch(new3s_PointXYZ &query_point , const int &knn , std::vector<new3s_PointXYZ> &indice_points , std::vector<float> &distance) ;

    int nearestKSearch(new3s_PointXYZ &query_point , const int &knn , std::vector<int> &indices , std::vector<float> &distance);
	/*********************************************
	Function:       // 函数名称
	Description:    // 半径最邻近点搜索
	Input:          // 输入参数说明，query_point 输入查询点
	Input:          // 输入参数说明，radius 查询半径
	Output:         // 输出参数说明，indice_points 保存查询到的近邻点坐标
	Output:         // 输出参数说明，distance 近邻点到查询点的距离平方和
	Return:         // 成功返回搜索到的最近邻个数 ，失败返回0
	Others:         // 其它说明
	*************************************************/
    int nearestRSearch(new3s_PointXYZ &query_point , const float &radius , std::vector<new3s_PointXYZ> &indice_points , std::vector<float> &distance) ;

    int nearestRSearch(new3s_PointXYZ &query_point , const float &radius , std::vector<int> &indices , std::vector<float> &distance) ;

	/*********************************************
	Function:       // 函数名称
	Description:    // 设置邻域查询方式和参数
	Input:          // 输入参数，搜索参数：个数或者距离（m）
	Input:          // 输入参数，搜索方式：KNN 或者 RNN	
	Return:         // 成功返回1 
	Others:         // 其它说明
	*************************************************/
	int setNearestSearchParameters(const float &parameter , SearchMethod method) ;
	int setNearestSearchParameters(const float &parameter1 , const float &parameter2 , SearchMethod method) ;
	

	/*********************************************
	Function:       // 函数名称
	Description:    // 计算邻域区域的重心
	Input:          // 输入参数，邻域点集的索引
	Output:         // 输出参数，保存邻域重心的坐标值
	Return:         // 成功返回1
	Others:         // 其它说明
	*************************************************/
    int computeCentroid3D(const std::vector<int> &indices , float &x , float &y , float &z);


	/*********************************************
	Function:       // 函数名称
	Description:    // 基于邻域的特征值、特征向量计算
	Input:          // 输入参数说明，当前点坐标 point
	Output:         // 输出参数说明，eigenvalue 保存该点邻域协方差矩阵特征值
	Output:         // 输出参数说明，eigenvectors 保存该点邻域协方差矩阵特征向量
	Return:         // 成功返回1 
	Others:         // 其它说明
	*************************************************/
    int computeCovarianceEigen(new3s_PointXYZ &point ,std::vector<float> &eigenvalues , std::vector<vecFloat> &eigenvectors);

    int computeCovarianceEigen(const std::vector<int> &indices , std::vector<float> &eigenvalues , std::vector<std::vector<float> > &eigenvectors);

	/*********************************************
	Function:       // 函数名称
	Description:    // 法向量估计
	Input:          // 输入参数说明，当前点坐标
	Output:         // 输出参数说明，normalVector 保存该点的法向量
	Return:         // 成功返回1 
	Others:         // 其它说明
	*************************************************/
    int normalVectorEstimation(new3s_PointXYZ &point ,std::vector<float> &normalVector);

    int VectorEstimation(const std::vector<int> &indices , std::vector<float> &vec) ;
    int VectorEstimation(const std::vector<int> &indices, Eigen::Vector3d &vec) ;


private: 
	float *m_dataset ; //保存输入点云
	int m_num_of_cloud ;//保存点云数量
	boost::shared_ptr<FLANNIndex> m_index ;	
	boost::shared_ptr<FLANNIndex> m_index_2D ;
	SearchMethod m_search_method ;
	float m_search_radius ;
	float m_search_rz ;
	int m_search_number ;
};

