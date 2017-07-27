#pragma once
#include "new3s_PointXYZ.h"
class new3s_PointXYZRGB :
	public new3s_PointXYZ
{
public:
	new3s_PointXYZRGB();
	new3s_PointXYZRGB(double x_, double y_, double z_, float R_, float G_, float B_);
	new3s_PointXYZRGB(const new3s_PointXYZRGB &other);
	virtual ~new3s_PointXYZRGB();

	void setR(float R);
	void setG(float G);
	void setB(float B);

	float getR() const;
	float getG() const;
	float getB() const;

private:
	float m_R;
	float m_G;
	float m_B;
};

