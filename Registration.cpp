#include <Registration.h>
#include <fstream>

HandEyeRegistration::HandEyeRegistration(){}

HandEyeRegistration::~HandEyeRegistration(){}

void HandEyeRegistration::loadMesh(std::string filename){	//Loads a mesh into a pdtree
	std::string saveMeshPath = "Last_Run";
	CreateMesh(mesh, filename, &saveMeshPath);
	

}

//Taken from Seth Billing's utilities.cpp
void HandEyeRegistration::CreateMesh(cisstMesh &mesh,const std::string &meshLoadPath, std::string *SavePath_Mesh)
{
	// load mesh
	mesh.LoadMeshFile(meshLoadPath);
	if (mesh.NumVertices() == 0)
	{
		printf("ERROR: Read mesh resulted in 0 triangles\n");
		assert(0);
	}

	// save mesh
	if (SavePath_Mesh)
	{
		if (mesh.SaveMeshFile((*SavePath_Mesh).append(".mesh")) < 0)
		{
			std::cout << "ERROR: Save mesh failed" << std::endl;
			assert(0);
		}
	}
}

vctFrm3 HandEyeRegistration::computeTransform(vctDynamicVector<vct3> & points){	//Run ICP between mesh and point cloud
	
	std::vector<cisstICP::Callback> userCallbacks;
	//  callback: iteration file
	cisstICP::Callback iterCallback;
	iterCallback.cbFunc = Callback_SaveIterationsToFile;
	std::stringstream iterFile;
	iterFile << workingDir << outputDir << "SaveIterations.txt";
	std::ofstream iterFileStream(iterFile.str().c_str());
	iterCallback.userData = (void*)(&iterFileStream);
	userCallbacks.push_back(iterCallback);
	//  callback: track path file
	cisstICP::Callback xfmCallback;
	xfmCallback.cbFunc = Callback_TrackRegPath;
	std::stringstream trackPathFile;
	trackPathFile << workingDir << outputDir << "SaveTrackRegPath.txt";
	std::ofstream xfmFileStream(trackPathFile.str().c_str());
	xfmCallback.userData = (void*)(&xfmFileStream);
	userCallbacks.push_back(xfmCallback);

	algICP *pICPAlg = NULL;

	PDTreeBase*         pTree = new PDTree_Mesh(mesh, nThresh, diagThresh);
	PDTree_Mesh *pTreeMesh = dynamic_cast<PDTree_Mesh*>(pTree);
	pICPAlg = new algICP_StdICP_Mesh(pTreeMesh, points);
	cisstICP ICP;

	vctRot3 rotinit;
	rotinit[0][0] = 1;
	rotinit[0][1] = 0;
	rotinit[0][2] = 0;

	rotinit[1][0] = 0;
	rotinit[1][1] = 1;
	rotinit[1][2] = 0;

	rotinit[2][0] = 0;
	rotinit[2][1] = 0;
	rotinit[2][2] = 1;

	vct3 xinit(0, 0, 0);

	vctFrm3 FGuess(rotinit, xinit);

	// ICP Options
	cisstICP::Options opt;
	opt.auxOutputDir = workingDir + outputDir;
	opt.maxIter = 100;
	opt.termHoldIter = 2;
	opt.minE = -std::numeric_limits<double>::max();
	opt.tolE = 0.0;
	opt.dPosThresh = 0.1;
	opt.dAngThresh = 0.1*(cmnPI / 180);
	opt.dPosTerm = 0.01;
	opt.dAngTerm = 0.01*(cmnPI / 180);

	// Run ICP
	int numRuns = 1;
	vctFrm3 Freg;
	double runtime = 0.0;
	cisstICP::ReturnType rv;
	for (int i = 0; i<numRuns; i++)
	{
		rv = ICP.RunICP(pICPAlg, opt, FGuess, &userCallbacks);
		std::cout << rv.termMsg;
		runtime += rv.runTime;
		Freg = rv.Freg;
	}
	return Freg;
}

void HandEyeRegistration::Callback_TrackRegPath(cisstICP::CallbackArg &arg, void *userData)
{
	// Save to file:
	//  - error function
	//  - incremental transform
	// output format:
	//  error r00 r01 r02 r10 r11 r12 r20 r21 r22 tx ty tz
	std::ofstream *fs = (std::ofstream *)(userData);
	(*fs) << arg.E << " " << arg.dF.Rotation().Row(0) << " " << arg.dF.Rotation().Row(1) << " "
		<< " " << arg.dF.Rotation().Row(2) << " " << arg.dF.Translation() << std::endl;
}
void HandEyeRegistration::Callback_SaveIterationsToFile(cisstICP::CallbackArg &arg, void *userData)
{
	std::ofstream *fs = (std::ofstream *)(userData);

	vctRodRot3 dR(arg.dF.Rotation());
	std::stringstream ss;
	ss << cmnPrintf("iter=%u  E=%.3f  tolE=%.4f (dAng/dPos)= %.2f/%.2f  t=%.3f NNodes=%u/%u/%u NOut=%u")
		<< arg.iter
		<< arg.E
		<< arg.tolE
		<< dR.Norm() * 180 / cmnPI << arg.dF.Translation().Norm()
		<< arg.time
		//<< arg.maxNodesSearched << arg.avgNodesSearched << arg.minNodesSearched
		<< arg.nOutliers;

	(*fs) << ss.str() << std::endl;
}
