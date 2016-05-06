#pragma once

#include <cisstVector.h>
#include <stdlib.h>
#include <cisstPointCloud.h>
#include <pcl\visualization\cloud_viewer.h>
#include <string>

/**
\brief Class that aids in performing an Eye in Hand calibration using an Angle Bracket as the calibration object. Only static methods are implemented at the moment

This class is designed to preform a calibration using two sets of data. The first is a set of point clouds where each point cloud is from the Real Sense camera at a different robot pose. 
These point clouds should be of our calibration object, which is a large angle bracket s.t. there are three planes clearly visible in the point cloud (the two inside edges of the bracket and the table). 
However this class is not fully implemented. At the moment only the static (mathematical and file reading) methods are implemented. The processing of the point clouds to generate the poses is not yet implemented. 
*/
class HECalibration
{
public:
	HECalibration();
	~HECalibration();

	/**
	Returns the transformation between the wrist and the camera, assuming that the class is already populated with data. Not yet implemented.
	*/
	vctFrm3 GetWristCameraRegistration();

	/**
	Add a point cloud to the set of data for the calibration. Not yet implemented. 
	@param[in] cloud the point cloud to be added
	*/
	void addPointCloud(cisstPointCloud cloud);

	/**
	Add a robot pose to the set of data for calibration. Not yet implemented.
	@param[in] pose the pose to be used in the calibration. 
	*/
	void addPose(vctFrm3 pose);

	/**
	Reads in a pose from a file. The format of the file must be R P where R is a 3x3 rotation matrix and P is a column transformation matrix. The file as a whole should be 3 rows with each row containing 4 numbers.
	@param[in] filename the name of the file containing the pose. Must be properly formatted.
	@return the pose read in from the file. 
	*/
	static vctFrm3 readObjectPose(std::string filename);

	/**
	Reads in the set of robot poses from a file. Each row of this file must be P Q where P is a 1x3 space separated matrix representing the position of the robot at the time, and Q is a space separated quaternion representing the rotational pose of the robot at the time. 
	@param[in] filename the name of the file containing the poses
	@return a vector containing the poses read in from the file. 
	*/
	static std::vector<vctFrm3> readRobotPoses(std::string filename);

	/**
	Computes X that minmizes the error of AX=XB. This is done using the quaternion method as discussed in Dr. Russell Taylor's lectures. 
	@param A[in] The set of poses representing the changes in position of the robot wrist
	@param B[in] The set of poses representing the changes in position of the calibration object relative to the camera
	@return The best solution for the rotational component of the transformation from the wrist to the camera
	*/
	static vctRot3 axxb(std::vector<vctFrm3> A, std::vector<vctFrm3> B);	//Takes A and B and uses Taylor's quaternion method to compute the quaternion rotation matrix of X 

	/**
	Solves the least squares equation to solve for the translational component of F_w.
	@param Rx[in] the rotational component of F_w. This can be solved for using the axxb method
	@param A[in] The set of poses representing the changes in position of the robot wrist
	@param B[in] The set of poses representing the changes in position of the calibration object relative to the camera
	@return The full frame transformation (with param Rx as the rotational component)
	*/
	static vctFrm3 leastSquares(vctRot3 Rx, std::vector<vctFrm3> A, std::vector<vctFrm3> B);

private:
	std::vector<cisstPointCloud> g_pointClouds;
	std::vector<vctFrm3> g_poses;

	static Eigen::Matrix<double, Eigen::Dynamic, 4> genM(std::vector<vctQuatRot3> qA, std::vector<vctQuatRot3> qB);

};

