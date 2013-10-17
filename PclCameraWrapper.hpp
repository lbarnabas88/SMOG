#ifndef PCLCAMERAWRAPPER_HPP
#define PCLCAMERAWRAPPER_HPP

// PCL
#include <pcl/visualization/common/common.h>
// Math
#include <Vector2.hpp>
#include <Vector3.hpp>

class PclCameraWrapper
{
public:
    PclCameraWrapper();
    PclCameraWrapper(const pcl::visualization::Camera& camera);
    inline void updateCamera(const pcl::visualization::Camera& camera) { mCamera = camera; }
    // Basic informations
    inline math::Vector3d getPos() const { return math::Vector3d(mCamera.pos[0], mCamera.pos[1], mCamera.pos[2]); }
    inline math::Vector3d getView() const { return math::Vector3d(mCamera.view[0], mCamera.view[1], mCamera.view[2]); }
    inline math::Vector3d getFocal() const { return math::Vector3d(mCamera.focal[0], mCamera.focal[1], mCamera.focal[2]); }
    inline math::Vector2d getClip() const { return math::Vector2d(mCamera.clip[0], mCamera.clip[1]); }
    inline double getFovy() const { return mCamera.fovy; }
    // Wrapping informations
    double zoomFactor() const;
    std::string toString();
protected:
    pcl::visualization::Camera mCamera;
};

#endif // PCLCAMERAWRAPPER_HPP
