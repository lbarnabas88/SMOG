#include "QPointCloudVisualizer.hpp"
// VTK
#include <vtkRenderWindow.h>
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// Qt
#include <QSettings>
#include <QTextStream>

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
    // Settings object
    QSettings settings;
    // Get backgorund color
    QColor bgColor = settings.value("visualizer/bgcolor").value<QColor>();
    // Set background color
    QTextStream out(stdout);
    if(bgColor.isValid())
    {
        out << "asdf\n";
        mVisualizer.setBackgroundColor(bgColor.redF(), bgColor.greenF(), bgColor.blueF());
    }
    else
    {
        out << "qwer\n";
    }
}
