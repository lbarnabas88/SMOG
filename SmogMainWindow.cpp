#include "SmogMainWindow.hpp"
#include "ui_SmogMainWindow.h"
// QT
#include <QFileDialog>
#include <QTextStream>
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

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
    // Get file path to load
    QString filename = QFileDialog::getOpenFileName(this, "Load file");
    // If valid, log
    if(!filename.isNull())
    {
        // Logger
        QTextStream out(stdout);
        // Log selected file
        out << "[Main] Load file: " << filename << '\n';
        // Cloud to load
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        // Load
        if(pcl::io::loadPCDFile(filename.toStdString(), *cloud) == 0)
        {
            // Log it's loaded
            out << "[Main] PCD cloud loaded (" << cloud->width << 'x' << cloud->height << ")\n";
            // Update to visualizer
            ui->CloudVisualizer->visualizer().updatePointCloud(cloud);
            // Reset camera
            ui->CloudVisualizer->visualizer().resetCamera();
        }
    }
}
