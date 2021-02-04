#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/gicp.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <string>
#include <fstream>

using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

Eigen::Matrix4f gicp(int idx, Eigen::Matrix4f seq_tf)
{	
	std::cout << "idx: " << idx << "\n" << std::endl;
	// 创建点云指针
	PointCloudT::Ptr source(new PointCloudT);
	PointCloudT::Ptr target(new PointCloudT);
	PointCloudT::Ptr source_downsampled(new PointCloudT);
	PointCloudT::Ptr target_downsampled(new PointCloudT);
	PointCloudT::Ptr transformed_source(new PointCloudT);
	PointCloudT::Ptr transformed_target(new PointCloudT);
	PointCloudT::Ptr result(new PointCloudT);
	pcl::StopWatch timer;

	//string source_file = "/home/chenxin/Desktop/kitti_pcl_test/kitti_raw_05/" + to_string(idx + 1) + ".pcd";
	//string target_file = "/home/chenxin/Desktop/kitti_pcl_test/kitti_raw_05/" + to_string(idx) + ".pcd";

	string source_file = "/media/chenxin/我不是硬盘/kitti_dataset/sequences/04/pcd/" + to_string(idx + 1) + ".pcd";
	string target_file = "/media/chenxin/我不是硬盘/kitti_dataset/sequences/04/pcd/" + to_string(idx) + ".pcd";

	//读取pcd文件
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(source_file, *source) == -1)
	{
		PCL_ERROR("Couldn't read file1 \n");
		return Eigen::Matrix4f::Identity();
	}
	std::cout << "Source: " << source->size() << " data points" << std::endl;
	
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(target_file, *target) == -1)
	{
		PCL_ERROR("Couldn't read file2 \n");
		return Eigen::Matrix4f::Identity();
	}
	std::cout << "Target: " << target->size() << " data points" << std::endl;

	// Create the filtering object
 	pcl::VoxelGrid<pcl::PointXYZ> sor;
  	sor.setInputCloud(source);
  	sor.setLeafSize(0.6f, 0.6f, 0.6f);
  	sor.filter(*source_downsampled);

  	sor.setInputCloud(target);
  	sor.setLeafSize(0.6f, 0.6f, 0.6f);
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
	//printf ("Transformation basis:\n");
    //std::cout << transform_basis << std::endl;
	
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
	
	/*
	//GICP
	pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
	gicp.setMaximumIterations(500);    //设置最大迭代次数iterations=true
	gicp.setInputCloud(transformed_source); //设置输入点云
	gicp.setInputTarget(transformed_target); //设置目标点云（输入点云进行仿射变换，得到目标点云）
	gicp.align(*result);          //匹配后源点云
 	*/

 	//ICP
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp; //创建ICP对象，用于ICP配准
	icp.setMaximumIterations(200);
	icp.setInputCloud(transformed_source); //设置输入点云
	icp.setInputTarget(transformed_target); //设置目标点云（输入点云进行仿射变换，得到目标点云）
	icp.align(*result);          //匹配后源点云
	
	Eigen::Matrix4f seq_transformation = Eigen::Matrix4f::Identity();
	//MatrixXf step_result(3, 4);
	//if (gicp.hasConverged())
	if (icp.hasConverged())
	{
		//seq_transformation = gicp.getFinalTransformation() * seq_tf;
		seq_transformation = icp.getFinalTransformation() * seq_tf;
		//std::cout << "\nGICP sequential transformation is \n " << seq_transformation << std::endl;
	}
	else
	{
		PCL_ERROR("\nGICP has not converged.\n");
		return Eigen::Matrix4f::Identity();
	}

	//std::cout << "GICP run time: " << timer.getTimeSeconds() << " s" << std::endl;
	std::cout << "ICP run time: " << timer.getTimeSeconds() << " s" << std::endl;

	Eigen::VectorXf curr_tf(12);
	Eigen::VectorXf sequantial_tf(12);

	for(int i = 0; i < 3; i++)
	{
		for(int j = 0; j < 4; j++)
		{
			//curr_tf(i*4+j) = gicp.getFinalTransformation()(i, j);
			curr_tf(i*4+j) = icp.getFinalTransformation()(i, j);
			sequantial_tf(i*4+j) = seq_transformation(i, j);
		}
	}

	//char *output_every_tf = "/home/chenxin/Desktop/kitti_gicp/gicp/gicp_every_tf.txt";
	//char *output_seq_tf = "/home/chenxin/Desktop/kitti_gicp/gicp/gicp_seq_tf.txt";
	//string output_every_tf = "gicp_every_tf.txt";
	//string output_seq_tf = "gicp_seq_tf.txt";

	// record
	FILE *outfile1 = fopen("gicp_every_tf.txt", "a");
	setbuf(outfile1, NULL); 
	fprintf(outfile1, "%8f %8f %8f %8f ", curr_tf(0), curr_tf(1), curr_tf(2), curr_tf(3));
	fflush(outfile1);
	fprintf(outfile1, "%8f %8f %8f %8f ", curr_tf(4), curr_tf(5), curr_tf(6), curr_tf(7));
	fprintf(outfile1, "%8f %8f %8f %8f\n", curr_tf(8), curr_tf(9), curr_tf(10), curr_tf(11));

	FILE *outfile2 = fopen("gicp_seq_tf.txt", "a");
	setbuf(outfile2, NULL); 
	fprintf(outfile2, "%8f %8f %8f %8f ", sequantial_tf(0), sequantial_tf(1), sequantial_tf(2), sequantial_tf(3));
	fflush(outfile2);
	fprintf(outfile2, "%8f %8f %8f %8f ", sequantial_tf(4), sequantial_tf(5), sequantial_tf(6), sequantial_tf(7));
	fprintf(outfile2, "%8f %8f %8f %8f\n", sequantial_tf(8), sequantial_tf(9), sequantial_tf(10), sequantial_tf(11));

	/*
	fstream outfile1;
    outfile1.open(output_every_tf, ios::app);
    outfile1 << curr_tf << endl;

    fstream outfile2;
    outfile2.open(output_seq_tf, ios::app);
    outfile2 << sequantial_tf << endl;
	*/
    fstream outfile3;
    outfile3.open("gicp_time.txt", ios::app);
    outfile3 << timer.getTimeSeconds() << endl;
	
	return seq_transformation;
}
 
int main(int argc, char** argv)
{
	int number = 271; //2263;
	Eigen::Matrix4f seq_tf = Eigen::Matrix4f::Identity();
	seq_tf(0,0) = 1.000000e+00; seq_tf(0,1) = 1.197625e-11; seq_tf(0,2) = 1.704638e-10; seq_tf(0,3) = -5.551115e-17;
	seq_tf(1,0) = 1.197625e-11; seq_tf(1,1) = 1.000000e+00; seq_tf(1,2) = 3.562503e-10; seq_tf(1,3) = 0.000000e+00;
	seq_tf(2,0) = 1.704638e-10; seq_tf(2,1) = 3.562503e-10; seq_tf(2,2) = 1.000000e+00; seq_tf(2,3) = 2.220446e-16;

	for(int i = 0; i < number - 1; i++)
	{
		Eigen::Matrix4f next_seq_tf = gicp(i, seq_tf);
		seq_tf = next_seq_tf;
	}
	
	return 0;
}