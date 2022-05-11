#ifndef TRIANSEG_H
#define TRIANSEG_H
#include "LasOperator.h"

class Trianseg
{
public:
	Trianseg();

	// input viewpoint and targetpoint
	void Setopandvpoint(Point3D &f_viewpoint, Point3D &f_targetpoint);
	//生成三角形，angle舒适视域60，立体可见124
	// generate triangle,
	void Generatetrian(double width, double anglethr = 15.0, bool distype = false);
	//清除计算内容
	// clear compute contents
	void Clear(bool deltran = false);
	//返回三个顶点的坐标
	// return three vertex's position
	std::vector<Point3D> Getrivetrexs();
	//计算那些点落在三角形范围内
	// compute points which fall in the triangle
	std::vector<int> Getriacludepoint(std::vector<Point3D> &point3d);
	// Gets();
private:
	//点系列
	Point3D viewpoint;
	Point3D targetpoint;
	double axisldis;
	bool pointflag;
	//三角系列
	std::vector<Eigen::Vector2d> trianvertex;
};

#endif