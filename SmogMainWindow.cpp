#include "SmogMainWindow.hpp"
#include "ui_SmogMainWindow.h"
// QT
#include <QFileDialog>
#include <QTextStream>
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
// STD
#include <iostream>

/**
 * Constructor of the main window.
 * @brief SmogMainWindow constructor.
 * @param parent the parend widget of the window.
 */
SmogMainWindow::SmogMainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::SmogMainWindow)
{
    // Setup the ui
    ui->setupUi(this);
    // Start maximized
    showMaximized();
}

/**
 * Destructor of the main window.
 */
SmogMainWindow::~SmogMainWindow()
{
    // Delete user interface.
    delete ui;
}

/**
 * @brief Called when quit action's triggered
 */
void SmogMainWindow::on_actionQuit_triggered()
{
    // Quit app
    qApp->quit();
}

/**
 * @brief Called when the load action's triggered
 */
void SmogMainWindow::on_actionLoad_Cloud_triggered()
{
    // QString selected filter
    QString filetype;
    // Get file path to load
    QString filename = QFileDialog::getOpenFileName(this, "Load file", "", "Point cloud(*.pcd *.las);;All files(*)", &filetype);
    // If valid, log
    if(!filename.isNull())
    {
        // Logger
        QTextStream out(stdout);
        // Log selected file
        out << "[Main] Load file: " << filename << " (type: " << filetype << ")\n";
        // Cloud blob to load
        pcl::PCLPointCloud2 cloudBlob;
        // Load
        if(pcl::io::loadPCDFile(filename.toStdString(), cloudBlob) == 0)
        {
            // Log it's loaded
            out << "[Main] PCD cloud loaded (" << cloudBlob.width << 'x' << cloudBlob.height << ")\n";
            // There is RGB
            bool isRGB = false;
            // Detect rgb component
            for(auto& field : cloudBlob.fields)
                if(field.name == "rgb")
                    isRGB = true;
            // If there is color
            if(isRGB)
            {
                // Cloud
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                // Convert cloud
                pcl::fromPCLPointCloud2(cloudBlob,*cloud);
                // Update to visualizer
                ui->CloudVisualizer->visualizer().updatePointCloud(cloud);
            }
            else
            {
                // Cloud
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
                // Convert cloud
                pcl::fromPCLPointCloud2(cloudBlob,*cloud);
                // Update to visualizer
                ui->CloudVisualizer->visualizer().updatePointCloud(cloud);
            }
            // Reset camera
            ui->CloudVisualizer->visualizer().resetCamera();
        }
    }
}

void SmogMainWindow::on_actionIncrease_point_size_triggered()
{
    // Size
    double pointSize = 1.0f;
    // Get size
    ui->CloudVisualizer->visualizer().getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize);
    // Increase and set
    if(pointSize < 5.0)
        ui->CloudVisualizer->visualizer().setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize + 1);
    // Update
    ui->CloudVisualizer->update();
}

void SmogMainWindow::on_actionDecrease_point_size_triggered()
{
    // Size
    double pointSize = 1.0f;
    // Get size
    ui->CloudVisualizer->visualizer().getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize);
    // Increase and set
    if(pointSize > 1.0)
        ui->CloudVisualizer->visualizer().setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize - 1);
    // Update
    ui->CloudVisualizer->update();
}
