#include "QPointCloudVisualizer.hpp"
// VTK
#include <vtkRenderWindow.h>
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * @brief Point cloud visualizer constructor
 * @param parent the parent widget of this
 */
QPointCloudVisualizer::QPointCloudVisualizer(QWidget *parent) :
    QVTKWidget(parent), mVisualizer("Cloud", false)
{
    // Set render window
    SetRenderWindow(mVisualizer.getRenderWindow());
    // Empty cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr emptyCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // Add empty cloud
    mVisualizer.addPointCloud(emptyCloud);
    // Set background color
    mVisualizer.setBackgroundColor(0.0, 0.4, 0.6);
}
