#ifndef TRIANSEG_H
#define TRIANSEG_H
#include "LasOperator.h"

class Trianseg
{
public:
	Trianseg();

	// input viewpoint and targetpoint
	void Setopandvpoint(Point3D &f_viewpoint, Point3D &f_targetpoint);
	//���������Σ�angle��������60������ɼ�124
	// generate triangle,
	void Generatetrian(double width, double anglethr = 15.0, bool distype = false);
	//�����������
	// clear compute contents
	void Clear(bool deltran = false);
	//�����������������
	// return three vertex's position
	std::vector<Point3D> Getrivetrexs();
	//������Щ�����������η�Χ��
	// compute points which fall in the triangle
	std::vector<int> Getriacludepoint(std::vector<Point3D> &point3d);
	// Gets();
private:
	//��ϵ��
	Point3D viewpoint;
	Point3D targetpoint;
	double axisldis;
	bool pointflag;
	//����ϵ��
	std::vector<Eigen::Vector2d> trianvertex;
};

#endif