/*********************************************
// Copyright:����������ͼ�Ƽ����޹�˾
// Author: ����
// Date:2016-08-20
// Description: 2016-09-13��˶�޸���ӽ�vectorд��las
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
	Function:       // ��������
	Description:    // �ļ�����
	Input:          // �������˵���������ļ�����
	Output:         // �������˵����
	Return:         // 0
	Others:         // ����˵��
	*************************************************/
	int setInputFileName(char *inputname)
	{
		m_inputfilename = inputname ;
		return 0;
	}


	/*********************************************
	Function:       // ��������
	Description:    // �ļ����
	Input:          // �������˵��������ļ�����
	Output:         // �������˵����
	Return:         // 0
	Others:         // ����˵��
	*************************************************/
	int setOutputFileName(char *outputname)
	{
		m_outputfilename = outputname ;
		return 0;
	}



	/*********************************************
	Function:       // ��������
	Description:    // ��ȡlas�ļ���vector<new3s_PointXYZ>
	Input:          // �������˵������
	Output:         // �������˵����
	Return:         // 0
	Others:         // ����˵��
	*************************************************/
	int lasFileRead(const char *inputlasfile , std::vector<new3s_PointXYZ> &points) ;           // ��ȡȫ����

	int lasFileRead(const char *inputlasfile , std::vector<new3s_PointXYZ> &points , int class_number , std::vector<int> &readIndices) ;     // ��ȡĳһ���

	int lasFileRead(const char *inputlasfile, std::vector<new3s_PointXYZRGB> &points);


// 	/*********************************************
// 	Function:       // ��������
// 	Description:    // ��ȡH5�ļ���vector<new3s_PointXYZ>
// 	Input:          // �������˵������
// 	Output:         // �������˵����
// 	Return:         // 0
// 	Others:         // ����˵��
// 	*************************************************/
// 	int H5FileRead(char *inputH5file , char *inputFormat , std::vector<new3s_PointXYZ> &points) ;           // ��ȡȫ����
// 
// 	int H5FileRead(char *inputH5file , char *inputFormat , std::vector<new3s_PointXYZ> &points , int class_number , std::vector<int> &readIndices) ;        // ��ȡĳһ���


// 	/*********************************************
// 	Function:       // ��������
// 	Description:    // ���µ�ľ�������
// 	Input:          // �������˵������
// 	Output:         // �������˵����
// 	Return:         // 0
// 	Others:         // ����˵��
// 	*************************************************/
// 	int updateClustersLasToLas(std::vector<int> &point_clusters) ;																//  ����las���las
// 	int updateClustersLasToH5(std::vector<int> &point_clusters , char *outputFormat) ;											//  ����las���h5
// 	int updateClustersH5ToLas(std::vector<int> &point_clusters , char *inputFormat) ;											//  ����h5���las
// 	int updateClustersH5ToH5(std::vector<int> &point_clusters , char *inputFormat , char *outputFormat) ;						//  ����h5���h5
// 	int updateClusters(std::vector<int> &point_clusters , char *inputFormat , char *outputFormat) ;							    //  ͳһ��װ

	/*********************************************
	Function:       // ��������
	Description:    // ��������ĵ���д��las
	Input:          // �������˵����cloud ������Ƶ�����
	Output:         // lasname ����las�ļ�������
	Return:         // �ɹ�����1 ��ʧ�ܷ���0
	Others:         // ��˶++ 2016-09-13
	*************************************************/
	int writeVectoLas(const std::vector<new3s_PointXYZ> &cloud , const int &R , const int &G , const int &B , char *lasname) ;


private:
	char *m_inputfilename ;
	char *m_outputfilename ;

};

