#include "CloudProcessing.h"


CloudProcessing::CloudProcessing()
{
}


CloudProcessing::~CloudProcessing()
{
}

void CloudProcessing::viewClouds(std::vector<pcl::PointCloud<pcl::PointXYZ> > *clouds) {
	/*
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (cloud2 != NULL) {
		cloud->resize(cloud1->size() + cloud2->size());
		*cloud += *cloud2;
	}
	else {
		cloud->resize(cloud1->size());
	}
	*cloud += *cloud1;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

	viewer = simpleVis(cloud);
	*/
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	for (int i = 0; i < clouds->size(); i++) {
		pcl::PointCloud<pcl::PointXYZ> next_cloud = (*clouds)[i];

		// pcl::io::loadPCDFile((*clouds)[i], *next_cloud);

		for (int j = 0; j < next_cloud.size(); j++) {
			pcl::PointXYZRGB point;
			point.x = next_cloud.points[j].x;
			point.y = next_cloud.points[j].y;
			point.z = next_cloud.points[j].z;
			point.r = (0 + ((i)* 255)) % 256;
			point.g = 255 - ((i)* 255);
			point.b = 255 - ((i)* 255);
			cloud->points.push_back(point);
		}
	}

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	viewer = rgbVis(cloud);

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(1000));
	}
	viewer->close();
}

void CloudProcessing::viewCloudsFromFiles(std::vector<std::string> filenames) {
	std::vector<pcl::PointCloud<pcl::PointXYZ> > clouds;
	for (int i = 0; i < filenames.size(); i++) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::io::loadPCDFile(filenames[i], *cloud);
		clouds.push_back(*cloud);
	}
	viewClouds(&clouds);
	/*
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::io::loadPCDFile(filenames[0], *cloud);
	for (int i = 1; i < filenames.size(); i++) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
		std::string temp_filename = std::string(filenames[i]);
		pcl::io::loadPCDFile(temp_filename, *cloud2);
		cloud->resize(cloud->size() + cloud2->size());
		*cloud += *cloud2;

	}

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

	viewer = simpleVis(cloud);


	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	*/
	return;
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> CloudProcessing::simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	// --------------------------------------------
	// -----Open 3D viewer and add point cloud-----
	// --------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> CloudProcessing::rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {
	// --------------------------------------------
	// -----Open 3D viewer and add point cloud-----
	// --------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return (viewer);
}

vctDynamicVector<vct3> CloudProcessing::PCLtoCISST(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> points = cloud->points;
	vctDynamicVector<vct3> cisstpoints(points.size());
	for (int i = 0; i < points.size(); i++) {
		pcl::PointXYZ p = points[i];
		vct3 cp(p.x, p.y, p.z);
		cisstpoints[i] = cp;
	}
	return cisstpoints;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CloudProcessing::CISSTtoPCL(vctDynamicVector<vct3> points) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i < points.size(); i++) {
		vct3 p = points[i];
		cloud->push_back(pcl::PointXYZ(p[0], p[1], p[2]));
	}
	return pcl::PointCloud<pcl::PointXYZ>::Ptr(cloud);
}


void CloudProcessing::TXTtoPCD(std::string filename) {
	std::string line;
	pcl::PointCloud<pcl::PointXYZ> cloud;
	std::ifstream myfile(filename);
	if (myfile.is_open())
	{
		pcl::PointCloud<pcl::PointXYZ> cloud;
		while (getline(myfile, line))
		{
			float x, y, z;
			myfile >> x >> y >> z;
			cloud.push_back(pcl::PointXYZ(x, y, z));
		}
		myfile.close();
		std::string filename_no_extension;
		for (int i = 0; i < filename.size() && filename[i] != '.'; i++) {
			filename_no_extension += filename[i];
		}
		pcl::io::savePCDFileASCII(filename_no_extension + ".pcd", cloud);
	}

	else std::cout << "Unable to open file" << std::endl;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CloudProcessing::downsample_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float leaf_size) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());

	// Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(leaf_size, leaf_size, leaf_size);
	sor.filter(*cloud_filtered);

	return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CloudProcessing::remove_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float tolerance) {
	// Initialize variables and such
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);

	// Set the tolerance level
	seg.setDistanceThreshold(tolerance);

	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);

	// Check if there is any plane
	if (inliers->indices.size() == 0)
	{
		PCL_ERROR("Could not estimate a planar model for the given dataset.");
		return NULL;
	}

	// Filter out points from plane
	pcl::ExtractIndices<pcl::PointXYZ> filter;
	filter.setInputCloud(cloud);
	filter.setIndices(inliers);
	filter.setNegative(true);
	filter.filter(*filtered_cloud);

	return filtered_cloud;
}