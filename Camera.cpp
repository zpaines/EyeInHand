#include <Camera.h>
#include <iostream>
#include <fstream>


//Device # = 21000
Camera::Camera() {

}
Camera::~Camera() {

}

void Camera::getFrame(vctDynamicVector<vct3> & points) {

	sm = PXCSenseManager::CreateInstance();
	PXCCaptureManager *cm = sm->QueryCaptureManager();

	sm->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, 640, 480, 30);

	pxcStatus sts = sm->Init();
	if (sts < PXC_STATUS_NO_ERROR) {
		std::cout << "DIDN\'T WORK" << std::endl;
	}

	PXCCapture::Device *device = sm->QueryCaptureManager()->QueryDevice();
	device->ResetProperties(PXCCapture::STREAM_TYPE_ANY);

	std::cout << device->SetDepthUnit(1000) << std::endl;

	PXCProjection *projection = device->CreateProjection();


	pxcStatus sts2 = sm->AcquireFrame(false);
	if (sts2 >= PXC_STATUS_NO_ERROR) {
		PXCCapture::Sample *sample = sm->QuerySample();
		PXCImage* image = (*sample)[streamType];
		PXCImage::ImageInfo info = {};
		PXCImage::ImageData data;
		image->AcquireAccess(PXCImage::ACCESS_READ, &data);
		PXCImage::ImageInfo dinfo = sample->depth->QueryInfo();
		int dpitch = data.pitches[0] / sizeof(short);
		short *dpixels = (short*)data.planes[0];
		int i = 0;
		points.SetSize(dinfo.width * dinfo.height);

		PXCPoint3DF32 * vertices = new PXCPoint3DF32[dinfo.width * dinfo.height];
		projection->QueryVertices(image, vertices);

		int c = 0;
		for (int i = 0; i < points.size(); i++) {
			PXCPoint3DF32 point = vertices[i];
			if (point.z != 0) {
				vct3 newPoint(point.x, point.y, point.z);
				points[c++] = newPoint;
			}
		}
		points.resize(c);
		image->ReleaseAccess(&data);
	}
	projection->Release();
	std::cout << sts2 << std::endl;
}
