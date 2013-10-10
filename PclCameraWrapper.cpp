#include "PclCameraWrapper.hpp"
// STD
#include <sstream>

PclCameraWrapper::PclCameraWrapper()
{
}

PclCameraWrapper::PclCameraWrapper(const pcl::visualization::Camera &camera)
{
    updateCamera(camera);
}

double PclCameraWrapper::zoomFactor() const
{
    return (getFocal() - getPos()).length();
}

std::string PclCameraWrapper::toString()
{
    std::stringstream ss;
    ss << "[Camera] Position: " << getPos();
    ss << ", Up vector: " << getView();
    ss << ", Focal point: " << getFocal();
    ss << ", Clipping planes depth: " << getClip();
    ss << ", Fovy: " << getFovy();
    // Additional infos
    ss << "\n[Camera] Zoom factor" << zoomFactor();
    return ss.str();
}
