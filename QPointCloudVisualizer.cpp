#include "QPointCloudVisualizer.hpp"

#include <vtkRenderWindow.h>

/**
 * @brief Point cloud visualizer constructor
 * @param parent the parent widget of this
 */
QPointCloudVisualizer::QPointCloudVisualizer(QWidget *parent) :
    QVTKWidget(parent), mVisualizer("Cloud", false)
{
    // Set render window
    SetRenderWindow(mVisualizer.getRenderWindow());
}
