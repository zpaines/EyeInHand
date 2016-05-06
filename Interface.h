#pragma once

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <cisstVector.h>
#include "HECalibration.h"
#include <stdlib.h>     /* srand, rand */
#include "pxcsensemanager.h"
#include <Interface.h>
#include <Registration.h>
#include <stdio.h>
#include <vector>

#include "CloudProcessing.h"
#include "utility.h"
#include <cisstPointCloud.h>
#include <string>
#include <fstream>
#include <sstream>
#include <Camera.h>

/**
\brief Driver class for the REMS Eye in Hand Registration Package

This class handles all user interface and farms out functionality to other classes. It was designed to be fairly modular and easily extendible s.t. more menu options and functionality could be added without major code changes. 
*/
class Interface
{
public:
	/**
	ENUM defining all menu choices for the user interface. 
	*/
	enum MenuOption
	{
		GetFrame = 1,
		LoadFrame = 2,
		LoadMesh = 3,
		Register = 4,
		Calibrate = 5,
		Visualize = 6,
		CloudToFile = 7,
		TranCloudToFile = 8,
		FRegFile = 9,
		DownsampleCloud = 10,
		RemovePlane = 11,
		Quit = 12
	};
	Interface();
	~Interface();

	/**
	This gets called to display the menu to the user and handle the latest command. As well as returning the menu selection, the function also completes the user request. In other words, if the user selects to read a point cloud from a file,
	this function will read the point cloud from the file and then return the command choice (MenuOption::LoadFrame)
	*/
	MenuOption getCommand();

private:
	Camera RealSense;
	std::string getFilename();
	void handleRegistration();
	void readPoints(std::string filename);
	HandEyeRegistration registration;
	vctDynamicVector<vct3> points;
	vctFrm3 computedTransform;
	vctDynamicVector<vct3> transformedPoints;
	void testCalibration();
	void visualizeClouds();
	static void writePoints(std::string filename, vctDynamicVector<vct3> pointList);
	void downsample();
	void remove_plane();
	std::string pointCloudFilename;
	std::string meshFilename;
	void FRegToFile(std::string filename);
	static double fRand(double fMin, double fMax);
	const std::string menu =
		"1. Take Point Cloud from Camera\n2. Read Point Cloud from File\n3. Load Mesh File for Registration\n4. Register Point Cloud to Mesh\n5. Test AX = XB Calibration\n6. Visualize Point Cloud and Transformed Point Cloud\n7. Write Point Cloud to File\n8. Write Transformed Point Cloud to File\n9. Write F_Reg to file\n10. Downsample Cloud\n11. Remove Plane\n12. Quit\n";
	
};

