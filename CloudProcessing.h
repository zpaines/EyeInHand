#pragma once

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <cisstVector.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>

/**
\brief Handles the processing of point clouds. All functions are static and the class has no member variables. 

This class handles all interfacing with PCL and manages the visualization, downsampling, and segmentation of point clouds. All methods are static. 
*/
class CloudProcessing
{
public:
	CloudProcessing();
	~CloudProcessing();

	/**
	Converts a .txt file where the format is x y z on each line to a pcd file with the proper header.
	@param[in] filename the name of the file relative to the working directory of the project to convert to pcd. Must be properly formatted.
	*/
	static void TXTtoPCD(std::string filename);

	/**
	Visualizes clouds read in from files. Files must be properly formatted PCD files. There is no set limit on the number of files but the visualizer can only handle so many.
	@param[in] filenames the list of filenames relative to the working directory to read the point clouds from
	*/
	static void viewCloudsFromFiles(std::vector<std::string> filenames);

	/**
	Converts a point cloud from the PCL format to be stored as vctDynamicVector<vct3> list of points. 
	@param[in] cloud A boost managed ptr to the point cloud to be converted
	@return This function returns the list of points that the cloud contained, but stored as vctDynamicVector<vct3>
	*/
	static vctDynamicVector<vct3> PCLtoCISST(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	/**
	Does the inverse function as PCLtoCISST
	@param[in] points the list of points to be converted into a PCL cloud
	@return Returns a boost managed ptr to the cloud, which now stores the points that were passed in with the vctDynamicVector
	*/
	static pcl::PointCloud<pcl::PointXYZ>::Ptr CISSTtoPCL(vctDynamicVector<vct3> points);

	/**
	Visualizes an array of clouds. 
	@param[in] clouds A pointer to the vector of clouds to be visualized. 
	*/
	static void viewClouds(std::vector<pcl::PointCloud<pcl::PointXYZ> > *clouds);

	/**
	This function downsamples the given point cloud by constructing a KD tree where the dimensions of the bounding box of each leaf node is leaf_size. It then makes a new cloud where the points are the centroids of all the leaf nodes from the KD tree.
	@param[in] cloud The cloud to downsample
	@leaf_size[in] The size of the bounding box for the leaf_nodes. Units are millimetres
	@return A new cloud that is the "downsampled" version of the original
	*/
	static pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float leaf_size);

	/**
	Removes a plane from the given cloud using the RANSAC algorithm and PCL's SACSegmentation function. 
	@param[in] cloud The point cloud that has the plane to be removed
	@param[in] tolerance The tolerance for the plane. In other words points that lie within "tolerance" of the computed plane will be removed from the cloud. Unit is mm
	@return The new cloud with the plane removed
	*/
	static pcl::PointCloud<pcl::PointXYZ>::Ptr remove_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float tolerance);


private:
	static boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
	static boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);

};

