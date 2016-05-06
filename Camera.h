#pragma once
#include "pxcsensemanager.h"
#include "projection.h"
#include <cisstVector.h>

/**
 \brief Manages the interface with the Intel Real Sense Camera
 
  This class uses the windows RSSDK to configure and get points clouds from the camera. It streams in a resolution of 640x480 with a frame rate of 30. 
*/
class Camera : public PXCSenseManager::Handler {
public:
	/**
	* Enum for how the sync the streams.
	*/
	enum SYNC_OPTION {
		SYNC_OPTION_SW = 0,
		SYNC_OPTION_HW,
		SYNC_OPTION_NONE
	};

	/**
	* Basic constructor. Doesn't do anything. All initialization occurs when getFrame() is called to ensure that changes in camera in between calls don't cause problems. 
	*/
	Camera();
	virtual ~Camera();

	/**
	* Gets a point cloud from the camera and loads it into the array. This is done using the PXCProjection mapping, which maps the pixelX, pixelY, depth points to x,y,z vertices.
	* @param[out] points The vector of points to load the point cloud into
	*/
	void getFrame(vctDynamicVector<vct3> & points);

protected:
	PXCSenseManager* sm;

private:
	SYNC_OPTION synced;
	PXCCapture::DeviceInfo dinfo;
	PXCCapture::StreamType streamType = PXCCapture::STREAM_TYPE_DEPTH;
	bool adaptive = false;
	PXCCapture::Device::StreamProfileSet profiles;

};