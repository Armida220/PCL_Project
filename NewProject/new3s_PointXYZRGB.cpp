#include "new3s_PointXYZRGB.h"



new3s_PointXYZRGB::new3s_PointXYZRGB()
	:m_R(0)
	,m_G(0)
	,m_B(0)
{
}

new3s_PointXYZRGB::new3s_PointXYZRGB(double x_, double y_, double z_, float R_, float G_, float B_)
	:new3s_PointXYZ(x_ , y_ , z_)
	,m_R(R_)
	,m_G(G_)
	,m_B(B_)
{

}

new3s_PointXYZRGB::new3s_PointXYZRGB(const new3s_PointXYZRGB & other)
{
	*this = other;
}


new3s_PointXYZRGB::~new3s_PointXYZRGB()
{
}

void new3s_PointXYZRGB::setR(float R)
{
	m_R = R;
}

void new3s_PointXYZRGB::setG(float G)
{
	m_G = G;
}

void new3s_PointXYZRGB::setB(float B)
{
	m_B = B;
}

float new3s_PointXYZRGB::getR() const
{
	return m_R ;
}

float new3s_PointXYZRGB::getG() const
{
	return m_G ;
}

float new3s_PointXYZRGB::getB() const
{
	return m_B ;
}


