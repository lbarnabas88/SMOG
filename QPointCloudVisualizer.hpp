#ifndef QPOINTCLOUDVISUALIZER_HPP
#define QPOINTCLOUDVISUALIZER_HPP

#include <QVTKWidget.h>
// PCL visualizer
#include <pcl/visualization/pcl_visualizer.h>

/**
 * @brief Point cloud visualizer Qt widget
 */
class QPointCloudVisualizer : public QVTKWidget
{
    Q_OBJECT
public:
    /**
     * @brief Point cloud visualizer constructor
     * @param parent the parent widget of this
     */
    explicit QPointCloudVisualizer(QWidget *parent = 0);
    
signals:
    
public slots:
    
private:
    // PCL visualizer object
    pcl::visualization::PCLVisualizer mVisualizer;
};

#endif // QPOINTCLOUDVISUALIZER_HPP
