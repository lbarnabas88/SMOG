#include "SmogMainWindow.hpp"
#include "ui_SmogMainWindow.h"
// Qt
#include <QFileDialog>
#include <QTextStream>
#include <QColorDialog>
#include <QSettings>
// Pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
// Std
#include <iostream>
// Backend
#include "CloudStore.hpp"
#include "PcdCloudData.hpp"

/**
 * Constructor of the main window.
 * @brief SmogMainWindow constructor.
 * @param parent the parend widget of the window.
 */
SmogMainWindow::SmogMainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::SmogMainWindow)
{
    // Set organization name
    QCoreApplication::setOrganizationName("PPKE-ITK");
    // Set organization domain
    QCoreApplication::setOrganizationDomain("itk.ppke.hu");
    // Set application name
    QCoreApplication::setApplicationName("Smog");
    // Set application version
    QCoreApplication::setApplicationVersion("0.0.1");
    // Setup the ui
    ui->setupUi(this);
    // Set model
    ui->CloudList->setModel(&CloudStore::getInstance());
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
 * @brief Called when the load action's triggered
 */
void SmogMainWindow::on_actionLoad_Cloud_triggered()
{
    // Settings object
    QSettings settings;
    // QString selected filter
    QString filetype;
    // Get file path to load
    QString filename = QFileDialog::getOpenFileName(this, "Load file", settings.value("main/lastdir", "").toString(), "Point cloud(*.pcd *.las);;All files(*)", &filetype);
    // If valid
    if(!filename.isNull())
    {
        // Logger
        QTextStream out(stdout);
        // Create file info object
        QFileInfo fileinfo(filename);
        // Log selected file
        out << "[Main] Load file: dir: " << fileinfo.dir().absolutePath() << ", name: " << fileinfo.fileName() << ", extension: " << fileinfo.suffix() << '\n';
        // Cloud store
        auto& cloudStore = CloudStore::getInstance();
        // Add cloud
        cloudStore.addCloud(fileinfo.baseName(), fileinfo.absoluteFilePath());
        // Update viz
        updateData(cloudStore.getCloud(cloudStore.getNumberOfClouds() - 1));
        // Set last used directory
        settings.setValue("main/lastdir", fileinfo.dir().absolutePath());
    }
}

void SmogMainWindow::on_actionIncrease_point_size_triggered()
{
    // Size
    double pointSize = 1.0f;
    // Get size
    ui->CloudVisualizer->visualizer().getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize);
    // Increase and set
    if(pointSize < 9.0)
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

void SmogMainWindow::on_actionBackground_Color_triggered()
{
    // Color picker
    QColor newBackgroundColor = QColorDialog::getColor(Qt::black, this);
    // If selected
    if(newBackgroundColor.isValid())
    {
        // Set background color
        ui->CloudVisualizer->visualizer().setBackgroundColor(newBackgroundColor.redF(), newBackgroundColor.greenF(), newBackgroundColor.blueF());
        // Settings
        QSettings settings;
        // Set to settings
        settings.setValue("visualizer/bgcolor", newBackgroundColor);

    }
}

void SmogMainWindow::updateData(CloudEntry::Ptr cloudEntry)
{
    auto raw_data = cloudEntry->getData();
    // Try as pcd
    auto pcd_data = dynamic_cast<PcdCloudData*>(raw_data.get());
    if(pcd_data)
    {
        ui->CloudVisualizer->visualizer().addPointCloud(pcd_data->getCloud<pcl::PointXYZ>(), cloudEntry->getName().toStdString());
    }
    // Try as las
}
