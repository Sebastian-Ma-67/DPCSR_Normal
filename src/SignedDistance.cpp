﻿#include "SignedDistance.h"

/*=======================================
SamplePoints
Input: vCloud - the raw point clouds
	vNewCloud - the sampled point clouds
	iSampleNum - interval number or the maximum number
	bIntervalSamp - interval number model (true) or the maximum number model (false)
Output: vNewCloud - the sampled point clouds
Function: Sample a point clouds. Note that tthe given maximum total number guarantes the number of sampled point is smaller than it
However, it is not guaranteed that the number of sampled point could be equal to the given maximum total number
========================================*/
void SamplePoints(const pcl::PointCloud<pcl::PointXYZ> &vCloud,
				  pcl::PointCloud<pcl::PointXYZ> &vNewCloud,
				  int iSampleNum,
				  bool bIntervalSamp)
{

	vNewCloud.clear();

	// sample by interval number
	if (bIntervalSamp)
	{

		for (int i = 0; i < vCloud.points.size(); i = i + iSampleNum)
			vNewCloud.push_back(vCloud.points[i]);

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

/*=======================================
ConvertCloud
Input: vSCloud - source point clouds
Output: vTCloud - target point clouds,also the transformed point cloud
Function: it converts points from world into local coordiante based on viewpoint
========================================*/
std::vector<float> SignedDistance::ConvexBasedGlance(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pRawCloud,
													 const pcl::PointXYZ &oViewPoint,
													 const pcl::PointCloud<pcl::PointXYZ>::Ptr &m_pVoxelCorner)
{

	GHPR oGHPRer(oViewPoint, m_GHPRParam);

	// compute the occlusion
	oGHPRer.Compute(pRawCloud);

	// get the face index
	m_vGlanceFaceIdxs = oGHPRer.GetConvexHullWorldIdx();

	// get the surface
	m_vSurfaceIdxs = oGHPRer.ConstructSurfaceIdx();

	pcl::PointCloud<pcl::PointXYZ>::Ptr pHullWorldVert(new pcl::PointCloud<pcl::PointXYZ>); //Ô­Ê¼pcdµãÔÆ
	for (int i = 0; i != pRawCloud->points.size(); ++i)
		pHullWorldVert->points.push_back(pRawCloud->points[i]);
	pHullWorldVert->points.push_back(oViewPoint);

	// compute parameters of each face
	ConvexHullOperation oConvexHullOPer;

	// compute face in two different space, respectively
	oConvexHullOPer.FaceParamInTwoSpace(*oGHPRer.m_pHullVertices, oGHPRer.m_vHullPolygonIdxs,
										*pHullWorldVert, m_vGlanceFaceIdxs);
	// oConvexHullOPer.ComputeAllFaceParams(oViewPoint, *pHullWorldVert, m_vGlanceFaceIdxs,
	//	oConvexHullOPer.m_oEuclidMatN, oConvexHullOPer.m_vEuclidD);

	std::cout << "the number of faces after GHPR: " << m_vGlanceFaceIdxs.size() << std::endl;

	// compute the face parameters
	std::vector<FacePara> vEuclidFaces;
	oConvexHullOPer.NDToFacePara(oConvexHullOPer.m_oEuclidMatN, oConvexHullOPer.m_vEuclidD, vEuclidFaces);

	// convert the voxel corners
	pcl::PointCloud<pcl::PointXYZ>::Ptr pConvertedVoxelCs(new pcl::PointCloud<pcl::PointXYZ>);
	oGHPRer.ConvertCloud(*m_pVoxelCorner, *pConvertedVoxelCs);

	// compute indicator function notation
	std::vector<SignedDis> vTransedSDis = oConvexHullOPer.ComputePointSignedDis(*pConvertedVoxelCs);

	// compute the precise signed distance
	std::vector<float> vCornerSignedDis = oConvexHullOPer.ComputePointSignedDis(vTransedSDis, *m_pVoxelCorner, vEuclidFaces);
	// std::vector<float> vCornerSignedDis = oConvexHullOPer.ComputePointSignedDis(*m_pVoxelCorner, oConvexHullOPer.m_oEuclidMatN, oConvexHullOPer.m_vEuclidD);

	// test for
	// std::vector<float> vCornerSignedDis;
	// for (int i = 0; i != vTransedSDis.size(); ++i){
	//	if (vTransedSDis[i].bInner)
	//		vCornerSignedDis.push_back(-1);
	//	else
	//		vCornerSignedDis.push_back(1);
	// }

	return vCornerSignedDis;
}

/*=======================================
ConvertCloud
Input: vSCloud - source point clouds
Output: vTCloud - target point clouds,also the transformed point cloud
Function: it converts points from world into local coordiante based on viewpoint
========================================*/
std::vector<float> SignedDistance::RaycastBasedGlance(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pRawCloud,
													  const pcl::PointXYZ &oViewPoint,
													  const pcl::PointCloud<pcl::PointXYZ>::Ptr &m_pVoxelCorner)
{

	GHPR oGHPRer(oViewPoint, m_GHPRParam);

	// compute the occlusion
	oGHPRer.Compute(pRawCloud);

	// get the face index
	m_vGlanceFaceIdxs = oGHPRer.GetConvexHullWorldIdx();

	// get the surface
	m_vSurfaceIdxs = oGHPRer.ConstructSurfaceIdx();

	//**data output
	pcl::PolygonMesh MeshModel;
	pcl::toPCLPointCloud2(*pRawCloud, MeshModel.cloud);
	MeshModel.polygons = m_vSurfaceIdxs;
	pcl::io::savePLYFileBinary("mesh_res.ply", MeshModel);

	// sample the mesh to points
	pcl::PointCloud<pcl::PointXYZ>::Ptr pZeroCrossCloud(new pcl::PointCloud<pcl::PointXYZ>);
	MeshSample oMeshSampler;
	oMeshSampler.SetSamplePointNum(100000);
	oMeshSampler.SampleMeshToPoints(MeshModel);
	oMeshSampler.OutputSampledClouds(*pZeroCrossCloud);

	WritePointCloudTxt("SampledClouds.txt", *pZeroCrossCloud);

	// compute the sectors
	Sector oSectorGrids;
	oSectorGrids.SetResolution(PI / 3600.0f, PI / 1800.0f);
	oSectorGrids.SetViewPoint(oViewPoint);
	oSectorGrids.AssignPoints(*pZeroCrossCloud);

	std::vector<int> vCloudGridLabels;
	oSectorGrids.CloudGridIdx(vCloudGridLabels);
	WritePointCloudTxt("PointSectorLabel.txt", *pZeroCrossCloud, vCloudGridLabels);

	std::vector<float> vCloudGridDis;
	oSectorGrids.CloudGridDis(vCloudGridDis);
	WritePointCloudTxt("PointtoViewDis.txt", *pZeroCrossCloud, vCloudGridDis);

	std::vector<float> vCornerSignedDis = oSectorGrids.Raycast(*pZeroCrossCloud, *m_pVoxelCorner);

	WritePointCloudTxt("NodeSectorLabel.txt", *m_pVoxelCorner, oSectorGrids.m_vNodeGridIdx);

	WritePointCloudTxt("NodeSignedDistance.txt", *m_pVoxelCorner, vCornerSignedDis);

	// output
	return vCornerSignedDis;
}

// compute point based signed distance of one glance
//"point based" means the method is based on point operation
std::vector<float> SignedDistance::PointBasedGlance(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pRawCloud,
													const pcl::PointXYZ &oViewPoint,
													Voxelization &oVoxeler)
{
	// Initialize instance oGHPRer based on parameters
	GHPR oGHPRer(oViewPoint, m_GHPRParam);

	// calculate the visibility of (a viewpoint and a given point set). 
	oGHPRer.Compute(pRawCloud);

	// output the index of each vertice on the convex hull 
	// corresponding to the input point set (with viewpoint) in the world coordinate.
	m_vGlanceFaceIdxs = oGHPRer.GetConvexHullWorldIdx();

	// output the index of each vertice(without viepoint) on the model surface 
	// corresponding to the input point set(without viewpoint) in the world coordinate.
	m_vSurfaceIdxs = oGHPRer.ConstructSurfaceIdx();

	// save visiable mesh data in ply file
	pcl::PolygonMesh MeshModel;
	pcl::toPCLPointCloud2(*pRawCloud, MeshModel.cloud);
	MeshModel.polygons = m_vSurfaceIdxs;
	pcl::io::savePLYFileBinary("visiable_mesh_res.ply", MeshModel); 
	//===========================end of GHPR recon===================

	//****samples the visiavle mesh to dense point clouds****
	// sample the mesh to points
	MeshSample oMeshSampler;
	oMeshSampler.SetSamplePointNum(100000);
	oMeshSampler.SampleMeshToPoints(MeshModel);
	// oMeshSampler.m_pSampedCloud();

	// Instantiate a object to compute the sampled point normal based on the face normal
	ConvexHullOperation oConvexHullOPer;

	// data
	pcl::PointCloud<pcl::PointXYZ>::Ptr pZeroCrossCloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<FacePara> vNormalPara;
	// compute the sampled point normal and divde it into point and normal
	oConvexHullOPer.ComputeAllFaceParams(oViewPoint, *oMeshSampler.m_pSampedCloud, *pZeroCrossCloud, vNormalPara);
	WritePointCloudTxt("SampledPointClouds.txt", *pZeroCrossCloud); // just pointxyz, no nmormal

	//****get the nodes that are near the surface****
	oVoxeler.VoxelizePoints(*pZeroCrossCloud, *pRawCloud);

	//****computes the signed distance for query nodes****
	// output mesh
	pcl::PointCloud<pcl::PointNormal>::Ptr pPNormals(new pcl::PointCloud<pcl::PointNormal>);
	oConvexHullOPer.ChangeNormalType(*pZeroCrossCloud, vNormalPara, *pPNormals);
	pcl::io::savePLYFileASCII("sampled_clouds_with_normals.ply", *pPNormals); // point with normals

	// compute distance
	std::vector<float> vCornerSignedDis = MinKDDSignedDis(pZeroCrossCloud, oVoxeler, vNormalPara);
	WritePointCloudTxt("NodeSignedDistance.txt", *oVoxeler.m_pCornerCloud, vCornerSignedDis);

	// prepare for next calculation
	oVoxeler.ClearMiddleData();

	// output
	return vCornerSignedDis;
}

std::vector<float> SignedDistance::MinKDDis(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pCloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr &pQueryCloud)
{

	std::vector<float> vMinDis;

	// get the raw index using the shortest distance approach
	pcl::KdTreeFLANN<pcl::PointXYZ> oLocalKdtree;
	oLocalKdtree.setInputCloud(pCloud);

	// searching based on kdtree
	for (size_t i = 0; i != pQueryCloud->points.size(); i++)
	{
		std::vector<int> kdindices;
		std::vector<float> kdistances;
		// find the first point as itself
		oLocalKdtree.nearestKSearch(pQueryCloud->points[i], 1, kdindices, kdistances);
		// at least one point can be found
		vMinDis.push_back(sqrt(kdistances[0]));
	}

	return vMinDis;
}

std::vector<float> SignedDistance::MinKDDSignedDis(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pCloud, Voxelization &oVoxeler,
												   const std::vector<FacePara> &vNormalPara)
{

	float fDefault = oVoxeler.m_fDefault;
	std::vector<float> vSignedDis(oVoxeler.m_pCornerCloud->points.size(), 1.0);

	// get the raw index using the shortest distance approach
	// generate a clouds for kdtree
	pcl::KdTreeFLANN<pcl::PointXYZ> oPKdtree;
	oPKdtree.setInputCloud(pCloud);

	ConvexHullOperation oNormalOper;

	// searching based on kdtree
	for (size_t i = 0; i != oVoxeler.m_pCornerCloud->points.size(); i++)
	{
		// if the node is near the potential surface
		if (oVoxeler.m_vNearStatus[i])
		{
			// define eigen type point
			Eigen::Vector3f oRefPoint(oVoxeler.m_pCornerCloud->points[i].x, oVoxeler.m_pCornerCloud->points[i].y, oVoxeler.m_pCornerCloud->points[i].z);
			// get the nearest mesh sampled point for query node point
			std::vector<int> kdindices;
			std::vector<float> kdistances;
			// find the first point as itself
			oPKdtree.nearestKSearch(oVoxeler.m_pCornerCloud->points[i], 1, kdindices, kdistances);
			// get target point id
			int iTargetId = kdindices[0];
			// get the unsigned distance
			float fMinDis = sqrt(kdistances[0]);
			// get the sign value
			float fSignValue = oNormalOper.PointToFaceDis(oRefPoint, vNormalPara[iTargetId].oNormal, vNormalPara[iTargetId].fDparam);
			// get the result
			vSignedDis[i] = fSignValue * fMinDis;
		}
	}

	return vSignedDis;
}