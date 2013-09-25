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
#include <pcl/impl/instantiate.hpp>
#include <pcl/visualization/impl/pcl_visualizer.hpp>
// Std
#include <iostream>
// Backend
#include "CloudStore.hpp"
#include "PcdCloudData.hpp"
#include "LasCloudData.hpp"

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
    // Create model
    mCloudModel.reset(new CloudModel(&CloudStore::getInstance()));
    // Set model
    ui->CloudList->setModel(mCloudModel.get());
    // Connent model to window
    connect(mCloudModel.get(), SIGNAL(dataChanged(const QModelIndex&, const QModelIndex&)), this, SLOT(cloudModelChanged(const QModelIndex&, const QModelIndex&)));
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
        mCloudModel->addCloud(fileinfo.baseName(), fileinfo.absoluteFilePath());
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

void SmogMainWindow::cloudModelChanged(const QModelIndex &from, const QModelIndex &to)
{
    Q_UNUSED(to);
    // Get corresponding cloud
    auto& cloud = CloudStore::getInstance().getCloud(from.row());
    // Log name
    QTextStream out(stdout);
    out << "Cloud name: " << cloud->getName() << " Set to " << (cloud->isVisible()) << '\n';
    // Update cloud on visualizer
    updateData(cloud);
}



void SmogMainWindow::updateData(CloudEntry::Ptr cloudEntry)
{
    if(cloudEntry->isVisible())
    {
        auto raw_data = cloudEntry->getData();
        // Try to convert
        auto pcd_data = dynamic_cast<PcdCloudData*>(raw_data.get());
        auto las_data = dynamic_cast<LasCloudData*>(raw_data.get());
        // As pcd
        if(pcd_data)
        {
            ui->CloudVisualizer->visualizer().addPointCloud(pcd_data->getCloud<pcl::PointXYZ>(), cloudEntry->getName().toStdString());
        }
        // As las
        else if(las_data)
        {
            ui->CloudVisualizer->visualizer().addPointCloud(las_data->getCloud(), cloudEntry->getName().toStdString());
        }
    }
    else
    {
        ui->CloudVisualizer->visualizer().removePointCloud(cloudEntry->getName().toStdString());
    }
    // Update visualizer
    ui->CloudVisualizer->update();
}
