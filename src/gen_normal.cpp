///*************real-time reconstruction*********************/
// #include "HpdPointCloudDisplay.h" // just for showing
#include "LasOperator.h" // just for input
#include "SectorPartition.h"
#include "ExplicitRec.h"
// #include "SignedDistance.h"
#include "GHPR.h"
#include <iostream>
#include <cmath>

void SamplePoints(const pcl::PointCloud<pcl::PointXYZ> &vCloud,
				  pcl::PointCloud<pcl::PointXYZ> &vNewCloud,
				  int iSampleNum,
				  bool bIntervalSamp = true)
{

	vNewCloud.clear();

	// sample by interval number
	if (bIntervalSamp)
	{

		for (int i = 0; i < vCloud.points.size(); i = i + iSampleNum)
		{
			float depth = pow(vCloud.points[i].x, 2) + pow(vCloud.points[i].y, 2) + pow(vCloud.points[i].z, 2);
			if (depth > 0.1 && depth < 2500)
			{
				vNewCloud.push_back(vCloud.points[i]);
			}
		}

		// over the function and output
		return;

	} // end if

	// Sampling according to the given maximum total number
	// get the interval point number - how muny points should be skipped
	int iInterval = ceil(float(vCloud.points.size()) / float(iSampleNum));
	// sample
	for (int i = 0; i < vCloud.points.size(); i = i + iInterval)
		vNewCloud.push_back(vCloud.points[i]);

	// output
	return;
}

int main(int argc, char **argv)
{

	std::vector<Point3D> vScenePoints;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pSceneCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pRawCloud(new pcl::PointCloud<pcl::PointXYZ>);

	HPDpointclouddataread(argv[1], pRawCloud, vScenePoints, 2);

	SamplePoints(*pRawCloud, *pSceneCloud, 1);

	pcl::PointXYZ oViewPoint;
	// x 0.535947 y  0.62239 z 0.535947 bunny
	// x 0.457275 y  0.500000 z 1.814216 Cassette.las
	// x 0.0 -y 0.0 z 0.0 scene1oneframe.las
	oViewPoint.x = 0.0;
	oViewPoint.y = 0.0;
	oViewPoint.z = 0.0;

	pcl::PointCloud<pcl::PointNormal>::Ptr pFramePNormal(new pcl::PointCloud<pcl::PointNormal>);
	ExplicitRec oExplicitBuilder;
	oExplicitBuilder.HorizontalSectorSize(12);
	oExplicitBuilder.SetViewPoint(oViewPoint);
	oExplicitBuilder.FrameReconstruction(*pSceneCloud, *pFramePNormal);

	int iVerticesNum;
	int iFacesNum;
	oExplicitBuilder.CountNumber(iVerticesNum, iFacesNum);

	// pcl::io::savePLYFileASCII(argv[2], *oExplicitBuilder.m_pCenterNormal);
	pcl::io::savePLYFileASCII(argv[2], *pFramePNormal);


	std::cout << "Number of input points: " << pRawCloud->points.size() << std::endl;
	std::cout << "Number of vertices: " << iVerticesNum << std::endl;
	std::cout << "Number of reconstructed meshes: " << iFacesNum << std::endl;

	return 0;
}
