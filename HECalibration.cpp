#include "HECalibration.h"


HECalibration::HECalibration()
{
}


HECalibration::~HECalibration()
{
}

vctFrm3 HECalibration::readObjectPose(std::string filename) {
	ifstream poseFile(filename);
	double rawData[3][4];
	std::string line;
	int lineNum = 0;
	if (poseFile.is_open()) {
		while (getline(poseFile, line)) {
			std::stringstream lineStream(line);
			double value;
			int k = 0;
			while (lineStream >> value) {
				rawData[lineNum][k++] = value;
			}
			lineNum++;
		}
	}

	vct3 origin(rawData[0][3], rawData[1][3], rawData[2][3]);
	vctRot3 rotation;
	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++) {
			rotation[r][c] = rawData[r][c];
		}
	}
	vctFrm3 pose(rotation, origin);
	return pose;
}

std::vector<vctFrm3> HECalibration::readRobotPoses(std::string filename) {
	ifstream poseFile(filename);
	std::vector<vctFrm3> poses;
	std::string line;
	int lineNum = 0;
	if (poseFile.is_open()) {
		while (getline(poseFile, line)) {
			std::stringstream lineStream(line);
			double value;
			double values[7];
			int k = 0;
			while (lineStream >> value) {
				values[k++] = value;
			}
			vct3 position(values[0], values[1], values[2]);
			vctQuatRot3 quat;
			quat[3] = values[3];
			quat[0] = values[4];
			quat[1] = values[5];
			quat[3] = values[6];
			quat.NormalizedSelf();
			vctRot3 rotation;
			rotation.From(quat);
			vctFrm3 pose(rotation, position);
			poses.push_back(pose);
		}
	}

	return poses;
}


vctFrm3 HECalibration::leastSquares(vctRot3 Rx, std::vector<vctFrm3> A, std::vector<vctFrm3> B){
	Eigen::Matrix<double, Eigen::Dynamic, 3> LeastSquareA(A.size() * 3, 3);
	Eigen::Matrix<double, Eigen::Dynamic, 1> LeastSquareB(A.size() * 3, 1);

	for (int k = 0; k < A.size(); k++) {
		vctRot3 Rak = A[k].Rotation();
		vctRot3 Rbk = B[k].Rotation();
		Eigen::Matrix<double, 3, 3> leftSide;
		leftSide << Rak(0, 0) - 1, Rak(0, 1), Rak(0, 2),
			Rak(1, 0), Rak(1, 1) - 1, Rak(1, 2),
			Rak(2, 0), Rak(2, 1), Rak(2, 2) - 1;
		vct3 RxPbk = Rx * B[k].Translation();
		vct3 right = RxPbk - A[k].Translation();
		Eigen::Matrix<double, 3, 1> rightSide;
		rightSide << right[0],
			right[1],
			right[2];
		LeastSquareA.block(k * 3, 0, 3, 3) = leftSide;
		LeastSquareB.block(k * 3, 0, 3, 1) = rightSide;

	}
	Eigen::MatrixXd px = LeastSquareA.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(LeastSquareB);
	vct3 translation(px(0, 0), px(1, 0), px(2, 0));
	vctFrm3 transformation(Rx, translation);
	return transformation;
}


vctRot3 HECalibration::axxb(std::vector<vctFrm3> A, std::vector<vctFrm3> B) {
	std::vector<vctQuatRot3> qA;
	std::vector<vctQuatRot3> qB;
	for (int i = 0; i < A.size(); i++) {
		vctQuatRot3 quat;
		quat.From(A[i].Rotation());
		qA.push_back(quat);
		quat.From(B[i].Rotation());
		qB.push_back(quat);
	}
	Eigen::Matrix<double, Eigen::Dynamic, 4> M = genM(qA, qB);
	Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, 4>> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix<double, 4, 1> qx = svd.matrixV().block(0, 3, 4, 1);
	vctQuatRot3 t;
	t[0] = qx(1, 0);
	t[1] = qx(2, 0);
	t[2] = qx(3, 0);
	t[3] = qx(0, 0);	//This is out of order because CISST quaternion puts scalar last
	vctRot3 Rx;
	Rx.From(t);
	return Rx;
}

Eigen::Matrix<double, Eigen::Dynamic, 4> HECalibration::genM(std::vector<vctQuatRot3> qA, std::vector<vctQuatRot3> qB) {
	Eigen::Matrix<double, Eigen::Dynamic, 4> M(qA.size()*4, 4);

	for (int r = 0; r < qA.size(); r++) {
		Eigen::Matrix<double, 4, 1> Ai;
		Ai << qA[r].Element(3), qA[r].Element(0), qA[r].Element(1), qA[r].Element(2);
		Eigen::Matrix<double, 4, 1> Bi;
		Bi << qB[r].Element(3), qB[r].Element(0), qB[r].Element(1), qB[r].Element(2);
		Eigen::Matrix<double, 1, 4> Mtop = Ai.transpose() - Bi.transpose();
		Eigen::Matrix<double, 3, 4> Mbottom;
		Eigen::Matrix<double, 4, 1> sum = Ai + Bi;
		Eigen::Matrix<double, 3, 3> skew;
		skew << 0, -1 * sum[3], sum[2],
			sum[3], 0, -1 * sum[1],
			-1 * sum[2], sum[1], 0;
		Mbottom << Ai.block(1, 0, 3, 1) - Bi.block(1, 0, 3, 1), (Ai[0] - Bi[0])*Eigen::Matrix<double, 3, 3>::Identity() + skew;
		M.block(4*r, 0, 4 , 4) << Mtop,
			Mbottom;
	}
	return M;
}


/*
std::vector<vctFrm3> poses;
poses.push_back(HECalibration::readObjectPose("bracket5_frame.txt"));
poses.push_back(HECalibration::readObjectPose("bracket6_frame.txt"));
poses.push_back(HECalibration::readObjectPose("bracket7_frame.txt"));

std::vector<vctFrm3> robotPoses = HECalibration::readRobotPoses("test.ssv");
for (int i = 0; i < robotPoses.size(); i++) {
std::cout << robotPoses[i] << std::endl;
}

vctFrm3 Fhw0inv = robotPoses[0].Inverse();
vctFrm3 Frc0inv = poses[0].Inverse();

std::cout << poses[0] << "\n";
std::cout << Frc0inv << "\n";
std::cout << poses[0]*Frc0inv << "\n";
*/
/*/
if (robotPoses.size() != poses.size()) {
std::cout << "ERROR: Must be the same number of poses\n";
}

std::vector<vctFrm3> A;
std::vector<vctFrm3> B;

for (int k = 0; k < robotPoses.size(); k++) {
A.push_back(robotPoses[k] * Fhw0inv);
B.push_back(poses[k] * Frc0inv);
std::cout << "***********************************\n" << A[k] << "\n" << B[k] << "\n***********************************************\n";
}*/
