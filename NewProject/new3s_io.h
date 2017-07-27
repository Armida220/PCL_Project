/*********************************************
// Copyright:北京星闪视图科技有限公司
// Author: 彭博文
// Date:2016-08-20
// Description: 2016-09-13韩硕修改添加将vector写入las
*************************************************/
#pragma once

#include <lasreader.hpp>
#include <laswriter.hpp>

#include <vector>
#include <iostream>
#include <string>
#include "new3s_PointXYZ.h"
#include "new3s_PointXYZRGB.h"

class new3s_io
{
public:
	new3s_io(void);
	~new3s_io(void);

	/*********************************************
	Function:       // 函数名称
	Description:    // 文件输入
	Input:          // 输入参数说明，输入文件名。
	Output:         // 输出参数说明。
	Return:         // 0
	Others:         // 其它说明
	*************************************************/
	int setInputFileName(char *inputname)
	{
		m_inputfilename = inputname ;
		return 0;
	}


	/*********************************************
	Function:       // 函数名称
	Description:    // 文件输出
	Input:          // 输入参数说明，输出文件名。
	Output:         // 输出参数说明。
	Return:         // 0
	Others:         // 其它说明
	*************************************************/
	int setOutputFileName(char *outputname)
	{
		m_outputfilename = outputname ;
		return 0;
	}



	/*********************************************
	Function:       // 函数名称
	Description:    // 读取las文件到vector<new3s_PointXYZ>
	Input:          // 输入参数说明，。
	Output:         // 输出参数说明。
	Return:         // 0
	Others:         // 其它说明
	*************************************************/
	int lasFileRead(const char *inputlasfile , std::vector<new3s_PointXYZ> &points) ;           // 读取全部点

	int lasFileRead(const char *inputlasfile , std::vector<new3s_PointXYZ> &points , int class_number , std::vector<int> &readIndices) ;     // 读取某一类点

	int lasFileRead(const char *inputlasfile, std::vector<new3s_PointXYZRGB> &points);


// 	/*********************************************
// 	Function:       // 函数名称
// 	Description:    // 读取H5文件到vector<new3s_PointXYZ>
// 	Input:          // 输入参数说明，。
// 	Output:         // 输出参数说明。
// 	Return:         // 0
// 	Others:         // 其它说明
// 	*************************************************/
// 	int H5FileRead(char *inputH5file , char *inputFormat , std::vector<new3s_PointXYZ> &points) ;           // 读取全部点
// 
// 	int H5FileRead(char *inputH5file , char *inputFormat , std::vector<new3s_PointXYZ> &points , int class_number , std::vector<int> &readIndices) ;        // 读取某一类点


// 	/*********************************************
// 	Function:       // 函数名称
// 	Description:    // 更新点的聚类属性
// 	Input:          // 输入参数说明，。
// 	Output:         // 输出参数说明。
// 	Return:         // 0
// 	Others:         // 其它说明
// 	*************************************************/
// 	int updateClustersLasToLas(std::vector<int> &point_clusters) ;																//  输入las输出las
// 	int updateClustersLasToH5(std::vector<int> &point_clusters , char *outputFormat) ;											//  输入las输出h5
// 	int updateClustersH5ToLas(std::vector<int> &point_clusters , char *inputFormat) ;											//  输入h5输出las
// 	int updateClustersH5ToH5(std::vector<int> &point_clusters , char *inputFormat , char *outputFormat) ;						//  输入h5输出h5
// 	int updateClusters(std::vector<int> &point_clusters , char *inputFormat , char *outputFormat) ;							    //  统一封装

	/*********************************************
	Function:       // 函数名称
	Description:    // 将容器里的点云写入las
	Input:          // 输入参数说明，cloud 保存点云的容器
	Output:         // lasname 保存las文件的名称
	Return:         // 成功返回1 ，失败返回0
	Others:         // 韩硕++ 2016-09-13
	*************************************************/
	int writeVectoLas(const std::vector<new3s_PointXYZ> &cloud , const int &R , const int &G , const int &B , char *lasname) ;


private:
	char *m_inputfilename ;
	char *m_outputfilename ;

};

