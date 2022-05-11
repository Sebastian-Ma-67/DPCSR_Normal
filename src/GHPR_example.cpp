#include "GHPR.h"
#include "HpdPointCloudDisplay.h"
#include <pcl/io/ply_io.h>

int main_off(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); //原始pcd点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr occloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<Point3D> point3d; // Point3D目标的特征计算点云
    //读取点云数据
    HPDpointclouddataread(argv[1], cloud, point3d, 2);
    // pcl::io::loadPCDFile(argv[1], *cloud);
    std::vector<Point3D> oripoint3d(point3d);
    //设置视点;
    pcl::PointXYZ oViewPoint;
    // {
    //     oViewPoint.x = 0.203490;
    //     oViewPoint.y = 0.62239;
    //     oViewPoint.z = 0.535947;
    // }
    {
        oViewPoint.x = 18.0;
        oViewPoint.y = 18.0;
        oViewPoint.z = 18.0;
    }
    //初始化HPR类
    GHPR hpdhpr(oViewPoint, 3.6);
    //设置遮挡索引
    std::vector<int> occindices;
    hpdhpr.Compute(cloud);
    occindices = hpdhpr.GetOccludedIdx();
    
	std::vector<pcl::Vertices> vGlanceFaceIdxs;
	vGlanceFaceIdxs = hpdhpr.GetConvexHullWorldIdx();

    std::vector<pcl::Vertices> vFaces;
    // vFaces = hpdhpr.GetConvexHullWorldIdx();
    vFaces = hpdhpr.ConstructSurfaceIdx();

    //输出
    for (size_t i = 0; i != oripoint3d.size(); i++)
        oripoint3d[i].classification = 0;
    for (size_t i = 0; i != occindices.size(); i++)
    {
        // occloud->push_back(cloud->points[occindices[i]]);
        oripoint3d[occindices[i]].classification = 1;
    }
    //窗口开启
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    HpdDisplay hpdisplay;
    // viewer=hpdisplay.Showsimplecolor(occloud,"grey");
    viewer = hpdisplay.Showclassification(oripoint3d, "assign");
    viewer->addSphere(oViewPoint, 0.002, 0.0, 0.0, 1.0, "viewpointer");
    // cloud->points.push_back(oViewPoint);
    viewer->addPolygonMesh<pcl::PointXYZ>(cloud, vFaces, "polyline");

    while (!viewer->wasStopped())
    {

        viewer->spinOnce();
    }

	pcl::PolygonMesh MeshModel;
	pcl::toPCLPointCloud2(*cloud, MeshModel.cloud);
	MeshModel.polygons = vFaces;
	pcl::io::savePLYFileBinary("mesh_res.ply", MeshModel);

    return 0;
}
