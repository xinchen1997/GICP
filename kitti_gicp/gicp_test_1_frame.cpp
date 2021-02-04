#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/gicp.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
 
int main(int argc, char** argv)
{
	// 创建点云指针
	PointCloudT::Ptr source(new PointCloudT);
	PointCloudT::Ptr target(new PointCloudT);
	PointCloudT::Ptr source_downsampled(new PointCloudT);
	PointCloudT::Ptr target_downsampled(new PointCloudT);
	PointCloudT::Ptr transformed_source(new PointCloudT);
	PointCloudT::Ptr transformed_target(new PointCloudT);
	PointCloudT::Ptr result(new PointCloudT);
	pcl::StopWatch timer;
 
	//读取pcd文件
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/chenxin/Desktop/kitti_pcl_test/kitti_raw_05/0.pcd", *source) == -1)
	{
		PCL_ERROR("Couldn't read file1 \n");
		return (-1);
	}
	std::cout << "Source: " << source->size() << " data points" << std::endl;
	
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/chenxin/Desktop/kitti_pcl_test/kitti_raw_05/1.pcd", *target) == -1)
	{
		PCL_ERROR("Couldn't read file2 \n");
		return (-1);
	}
	std::cout << "Target: " << target->size() << " data points" << std::endl;

	// Create the filtering object
 	pcl::VoxelGrid<pcl::PointXYZ> sor;
  	sor.setInputCloud(source);
  	sor.setLeafSize(0.4f, 0.4f, 0.4f);
  	sor.filter(*source_downsampled);

  	sor.setInputCloud(target);
  	sor.setLeafSize(0.4f, 0.4f, 0.4f);
  	sor.filter(*target_downsampled);

  	std::cerr << "Source downsampled: " << source_downsampled->size() << " data points " << std::endl;
    std::cerr << "Target downsampled: " << target_downsampled->size() << " data points " << std::endl;

    /*
    // Test transformation
	Eigen::Matrix4f transform_basis = Eigen::Matrix4f::Identity();
	transform_basis(0,3) = 5;
	printf ("Transformation basis:\n");
    std::cout << transform_basis << std::endl;
    */

    // Transformation matrix from kitti
    Eigen::Matrix4f transform_basis = Eigen::Matrix4f::Identity();
	transform_basis(0,0) = -1.857739385241e-03; transform_basis(0,1) = -9.999659513510e-01; transform_basis(0,2) = -8.039975204516e-03; transform_basis(0,3) = -4.784029760483e-03;
  	transform_basis(1,0) = -6.481465826011e-03; transform_basis(1,1) = 8.051860151134e-03; transform_basis(1,2) = -9.999466081774e-01; transform_basis(0,3) = -7.337429464231e-02;
  	transform_basis(2,0) = 9.999773098287e-01; transform_basis(2,1) = -1.805528627661e-03; transform_basis(2,2) = -6.496203536139e-03; transform_basis(2,3) = -3.339968064433e-01;
	printf ("Transformation basis:\n");
    std::cout << transform_basis << std::endl;
	
	/*
	// Transformation matrix from lab github
  	Eigen::Affine3f rx = Eigen::Affine3f::Identity();
    Eigen::Affine3f ry = Eigen::Affine3f(Eigen::AngleAxisf(-M_PI/2.0, Eigen::Vector3f::UnitY()));
    Eigen::Affine3f rz = Eigen::Affine3f(Eigen::AngleAxisf(M_PI/2.0, Eigen::Vector3f::UnitZ()));
    Eigen::Affine3f transform_basis = rz * ry * rx;
	printf ("Transformation basis:\n");
    std::cout << transform_basis.matrix() << std::endl;
    */

    // 执行变换，并将结果保存在新创建的 transformed_cloud 中
    pcl::transformPointCloud (*source_downsampled, *transformed_source, transform_basis);
    pcl::transformPointCloud (*target_downsampled, *transformed_target, transform_basis);

	timer.reset();

	/*
	// Test transformation
	pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
	gicp.setMaximumIterations(500);    //设置最大迭代次数iterations=true
	gicp.setInputCloud(source_downsampled); //设置输入点云
	gicp.setInputTarget(transformed_source); //设置目标点云（输入点云进行仿射变换，得到目标点云）
	gicp.align(*result);          //匹配后源点云
	*/

	//GICP
	pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
	gicp.setMaximumIterations(500);    //设置最大迭代次数iterations=true
	gicp.setInputCloud(transformed_source); //设置输入点云
	gicp.setInputTarget(transformed_target); //设置目标点云（输入点云进行仿射变换，得到目标点云）
	gicp.align(*result);          //匹配后源点云
 	
 	//ICP
	//pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp; //创建ICP对象，用于ICP配准
	//icp.setMaximumIterations(200);
	//icp.setInputCloud(source); //设置输入点云
	//icp.setInputTarget(target); //设置目标点云（输入点云进行仿射变换，得到目标点云）
	//icp.align(*result);          //匹配后源点云
	
	//icp.setMaximumIterations(1);  // 设置为1以便下次调用
	//std::cout << "Applied " << iterations << " ICP iteration(s)" << std::endl;
	if (gicp.hasConverged())//icp.hasConverged ()=1（true）输出变换矩阵的适合性评估
	{
		std::cout << "\nGICP has converged, score is: " << gicp.getFitnessScore() << std::endl;
		std::cout << "\nGICP has converged, Epsilon is: " << gicp.getEuclideanFitnessEpsilon() << std::endl;
		//std::cout << "\nGICP transformation " << iterations << " : source -> target" << std::endl;
		std::cout << "\nGICP transformation is \n " << gicp.getFinalTransformation() << std::endl;
	}
	else
	{
		PCL_ERROR("\nGICP has not converged.\n");
		return (-1);
	}
	std::cout << "GICP run time: " << timer.getTimeSeconds() << " s" << std::endl;
	
	// Visualiazation
	pcl::visualization::PCLVisualizer viewer("GICP demo");

	// Source: Red
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_source(transformed_source, 255, 0, 0 );
	viewer.addPointCloud(transformed_source, cloud_source, "source");

	//pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_source(source_downsampled, 255, 0, 0 );
	//viewer.addPointCloud(source_downsampled, cloud_source, "source before transformation");
 
	// Target: Green
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_target(transformed_source, 0, 255, 0);
	viewer.addPointCloud(transformed_target, cloud_target, "target");
 
	// Result: Blue
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_result(result, 0, 0, 255);
	viewer.addPointCloud(result, cloud_result, "result");
 
	// Add Text
	// 其中"icp_info"是添加字符串的ID标志，（10，15）为坐标, 20为字符大小, 后面分别是RGB值
	viewer.addText("Source: Red\nTarget: Green\nResult: Blue", 10, 15, 20, 1, 1, 1, "icp_info");

	// Black background
	viewer.setBackgroundColor(0, 0, 0);

	// 设置相机的坐标和方向
	viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
	viewer.setSize(1280, 1024);  // 可视化窗口的大小
 
	//显示
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
	return 0;
}