#ifndef HEREG_HEADER
#define HEREG_HEADER


#include <stdlib.h>
#include <string>
#include <cisstVector.h>
#include "cisstICP.h"
#include "cisstMesh.h"
#include "cisstPointCloud.h"
#include "PDTree_Mesh.h"
#include "PDTree_PointCloud.h"

#include "algICP_StdICP_Mesh.h"
#include "algICP_IMLP_Mesh.h"

#include "algICP_StdICP_PointCloud.h"
#include "algICP_IMLP_PointCloud.h"
#include "utility.h"

/**
\brief Class for computing registration between point cloud and mesh

This class uses Seth Billing's ICP Implementation to compute the transformation between a poin cloud and a mesh. The registration is fairly slow when run on meshes and point clouds
of the sizes that CT scans and the Real Sense camera generate, so downsampling and mesh simplification are required. The registration is quite error prone and seems to often hit local minimum
when computing the transformation. Further investigation into this is required, although it may simply be a result of attempting to register a partial section (since the camera can only see a section
of the head) and the fact that the head is a fairly spherical object. 
*/
class HandEyeRegistration {
public:
	HandEyeRegistration();
	virtual ~HandEyeRegistration();

	/**
	This function reads a mesh in from a file into a cisstMesh data structure. I would have liked to have it also create the PDTree for the mesh,
	but for some reason this caused memory issues and caused the program to crash. Not sure why, but it seemed like it had to do with how the PDTree
	gets handled by managed ptrs. 
	@param filename The file to read the mesh in from. Must be formatted correctly. 
	*/
	void loadMesh(std::string filename);

	/**
	Computes the transform using ICP between the point cloud and the mesh that it has previously loaded. 
	@param points The point cloud to compute the transformation with
	@return the computed transformation
	*/
	vctFrm3 computeTransform(vctDynamicVector<vct3> & points);	//Run ICP between mesh and point cloud

	/**
	This function is a callback that allows for ICP output
	Taken from Seth Billing's cisstICP_APP
	*/
	static void HandEyeRegistration::Callback_TrackRegPath(cisstICP::CallbackArg &arg, void *userData);

	/**
	This function allows for ICP to output some debug information to a file
	Taken from Seth Billing's cisstICP_APP
	*/
	static void HandEyeRegistration::Callback_SaveIterationsToFile(cisstICP::CallbackArg &arg, void *userData);


private:
	cisstMesh mesh;
	int    nThresh = 5;       // Cov Tree Params
	double diagThresh = 5.0;

	void CreateMesh(cisstMesh &mesh, const std::string &meshLoadPath, std::string *SavePath_Mesh);

	std::string workingDir = "../test_data";
	std::string outputDir = "LastRun";

};

#endif
