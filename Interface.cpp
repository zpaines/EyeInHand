#include "Interface.h"


Interface::Interface()
{
}


Interface::~Interface()
{
}

std::string Interface::getFilename() {
	std::cout << "Enter file name:";
	std::string name;
	std::cin >> name;
	return name;
}

Interface::MenuOption Interface::getCommand() {
	std::cout << menu;
	int input;
	std::cin >> input;
	MenuOption choice = static_cast<MenuOption>(input);
	switch (choice)
	{
	case Interface::GetFrame:
		RealSense.getFrame(points);
		break;
	case Interface::LoadFrame:
		pointCloudFilename = getFilename();
		readPoints(pointCloudFilename);
		break;
	case Interface::LoadMesh:
		meshFilename = getFilename();
		registration.loadMesh(meshFilename);
		break;
	case Interface::Register:
		handleRegistration();
		break;
	case Interface::Calibrate:
		testCalibration();
		break;
	case Interface::Visualize:
		visualizeClouds();
		break;
	case Interface::CloudToFile:
		Interface::writePoints(getFilename(), points);
		break;
	case Interface::TranCloudToFile:
		Interface::writePoints(getFilename(), transformedPoints);
		break;
	case Interface::DownsampleCloud:
		downsample();
		break;
	case Interface::RemovePlane:
		remove_plane();
		break;
	case Interface::FRegFile:
		FRegToFile(getFilename());
		break;
	case Interface::Quit:
		break;
	default:
		std::cout << "Not a valid choice\n";
		break;
	}
	return choice;

}

void Interface::FRegToFile(std::string filename) {
	std::ofstream outFile;
	outFile.open(filename);
	outFile << computedTransform << std::endl;
	outFile.close();
}

void Interface::remove_plane() {
	float tolerance = 0;
	std::cout << "Please enter a tolerance width: " << std::endl;
	std::cin >> tolerance;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CloudProcessing::CISSTtoPCL(points);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 = CloudProcessing::remove_plane(cloud, tolerance);
	vctDynamicVector<vct3> cisst_cloud2 = CloudProcessing::PCLtoCISST(cloud2);
	points.SetSize(cisst_cloud2.size());
	for (int i = 0; i < cisst_cloud2.size(); i++) {
		points[i] = cisst_cloud2[i];
	}
}

void Interface::downsample() {
	float leaf_size = 0;
	int original_num = points.size();
	std::cout << "What would you like to set the leaf size to be?" << std::endl;
	std::cin >> leaf_size;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 = CloudProcessing::CISSTtoPCL(points);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 = CloudProcessing::downsample_cloud(cloud1, leaf_size);
	vctDynamicVector<vct3> cisst_cloud2 = CloudProcessing::PCLtoCISST(cloud2);
	points.SetSize(cisst_cloud2.size());
	for (int i = 0; i < cisst_cloud2.size(); i++) {
		points[i] = cisst_cloud2[i];
	}
	//Points are stored in variable points
	//You can use CloudProcessing::CISSTtoPCL and CloudProcessing PCLtoCISST to go between formats
	std::cout << "Sampled " << original_num << " points to " << points.size() << "points.\n";
}

void Interface::writePoints(std::string filename, vctDynamicVector<vct3> pointList) {
	std::ofstream outFile;
	outFile.open(filename);
	for (int i = 0; i < pointList.size(); i++) {
		outFile << pointList[i] << std::endl;
	}
	outFile.close();

}

void Interface::visualizeClouds() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 = CloudProcessing::CISSTtoPCL(points);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 = NULL;
	std::vector<pcl::PointCloud<pcl::PointXYZ> > clouds;
	clouds.push_back(*cloud1);
	if (transformedPoints.size() > 0) {
		cloud2 = CloudProcessing::CISSTtoPCL(transformedPoints);
		clouds.push_back(*cloud2);
	}
	CloudProcessing::viewClouds(&clouds);
}

void Interface::handleRegistration() {
	std::cout << "Handling Registration\n";
	transformedPoints.SetSize(points.size());
	computedTransform = registration.computeTransform(points);
	for (int i = 0; i < points.size(); i++) {
		transformedPoints[i] = computedTransform.ApplyTo(points[i]);
	}
	std::cout << "You may wish to load a point cloud representation of the mesh so that you can visualize how successful the registration was\n";
}

double Interface::fRand(double fMin, double fMax)
{
	double f = (double)rand() / RAND_MAX;
	return fMin + f * (fMax - fMin);
}

void Interface::testCalibration() {
	vct3 axisWrist(fRand(-1, 1), fRand(-1, 1), fRand(-1, 1));
	axisWrist.NormalizedSelf();
	vct3 PxReal(fRand(-10, 10), fRand(-10, 10), fRand(-10, 10));
	vctRot3 rotation(vctAxAnRot3(axisWrist, fRand(-2, 2))); 
	vctQuatRot3 quat;
	quat.From(rotation);
	vctFrm3 F_w(rotation, PxReal);
	std::vector<vctFrm3> A;
	std::vector<vctFrm3> B;
	for (int k = 0; k < 3; k++) {
		double angleRot = fRand(-2, 2);
		vct3 ax(fRand(-1, 1), fRand(-1, 1), fRand(-1, 1));
		ax.NormalizedSelf();
		vctRot3 rot(vctAxAnRot3(ax, angleRot));
		vct3 Pak(fRand(-3, 3), fRand(-3, 3), fRand(-3, 3));
		vctFrm3 A_k(rot, Pak);
		vctFrm3 B_k = F_w.Inverse() * A_k * F_w;
		A.push_back(A_k);
		B.push_back(B_k);
	}

	vctRot3 result = HECalibration::axxb(A, B);
	vctFrm3 res = HECalibration::leastSquares(result, A, B);
	std::cout << "Generated F_w\n" << F_w << "\n\n";
	std::cout << "Calculated F_w\n" << res << "\n";

}


void Interface::readPoints(std::string filename) {
	std::cout << "Enter roughly how many points there are in the file\n";
	int total;
	std::cin >> total;
	std::cout << "Enter roughly what percent of those points you would like to sample\n";
	double percent;
	std::cin >> percent;
	points.SetSize(total * (percent/100.0));
	std::string line;
	ifstream cloudFile(filename);
	int i = 0;	//Total lines inspected so far
	int c = 0;	//Total number of points read into array so far
	if (cloudFile.is_open()) {
		while (getline(cloudFile, line)) {
			++i;
			std::stringstream lineStream(line);
			double value;
			std::vector<double> nums;
			if (i % (int)(100/percent) == 0) {
				++c;
				if (points.size() < c) {
					points.resize(points.size() * 2);
				}
				while (lineStream >> value) {
					nums.push_back(value*1.0000);
				}
				vct3 point(nums[0], nums[1], nums[2]);
				points[c - 1] = point;
			}
		}
	}
	points.resize(c);
	std::cout << points.size() << " points read in\n";
}
