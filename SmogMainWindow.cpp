#include "SmogMainWindow.hpp"
#include "ui_SmogMainWindow.h"
// Qt
#include <QFileDialog>
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
#include "AdaptiveCloudEntry.hpp"
#include "PcdCloudData.hpp"
#include "LasCloudData.hpp"
// Tools
#include "PclCameraWrapper.hpp"

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
    // Register visualizer events
    ui->CloudVisualizer->visualizer().registerMouseCallback(&SmogMainWindow::onVisualizerMouse, *this, this);
    ui->CloudVisualizer->visualizer().registerKeyboardCallback(&SmogMainWindow::onVisualizerKeyboard, *this, this);
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
        mCloudModel->addCloud(fileinfo.baseName(), fileinfo.absoluteFilePath(), ui->actionUse_adaptive_clouds->isChecked());
        // Update viz
        updateOnVisibility(cloudStore.getCloud(cloudStore.getNumberOfClouds() - 1));
        // Set last used directory
        settings.setValue("main/lastdir", fileinfo.dir().absolutePath());
    }
}

void SmogMainWindow::on_actionIncrease_point_size_triggered()
{
    // Increase with 1
    changeSelectedCloudsPointSize(1);
}

void SmogMainWindow::on_actionDecrease_point_size_triggered()
{
    // Decrease with 1
    changeSelectedCloudsPointSize(-1);
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
    if(from.column() == CloudModel::COLUMN_VISIBILITY)
        updateOnVisibility(cloud);
}



void SmogMainWindow::updateOnVisibility(CloudEntry::Ptr cloudEntry)
{
    // Call visualize
    cloudEntry->visualize(&ui->CloudVisualizer->visualizer());
    // Update visualizer
    ui->CloudVisualizer->update();
}

void SmogMainWindow::changeCloudPointSize(CloudEntry::Ptr cloud, int pointSizeDiff)
{
    // Size
    double pointSize = 1.0f;
    // Get size
    ui->CloudVisualizer->visualizer().getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloud->getName().toStdString());
    // Change
    pointSize += pointSizeDiff;
    // Increase and set
    ui->CloudVisualizer->visualizer().setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloud->getName().toStdString());
    // Update
    ui->CloudVisualizer->update();
}

void SmogMainWindow::changeSelectedCloudsPointSize(int pointSizeDiff)
{
    // For all selected row
    foreach (QModelIndex index, ui->CloudList->selectionModel()->selectedRows())
    {
        // Change cloud point size
        changeCloudPointSize(CloudStore::getInstance().getCloud(index.row()), pointSizeDiff);
    }
}

void SmogMainWindow::onVisualizerMouse(const pcl::visualization::MouseEvent &me, void *userData)
{
    QTextStream out(stdout);
    out << "[Main] Viz Mouse\n";

    if(!ui->actionLock_adaptive_view->isChecked())
    {
        for(size_t i = 0; i < CloudStore::getInstance().getNumberOfClouds(); ++i)
        {
            AdaptiveCloudEntry* cloudEntry = dynamic_cast<AdaptiveCloudEntry*>(CloudStore::getInstance().getCloud(i).get());
            if(cloudEntry)
            {
                cloudEntry->updateVisualization(&ui->CloudVisualizer->visualizer());
            }
        }
    }
}

void SmogMainWindow::onVisualizerKeyboard(const pcl::visualization::KeyboardEvent &ke, void *userData)
{
    QTextStream out(stdout);
    out << "[Main] Viz Keyboard\n";
}

void SmogMainWindow::on_actionClose_Cloud_triggered()
{
    // Deleted items
    int deleted = 0;
    // Get selected clouds
    foreach (QModelIndex index, ui->CloudList->selectionModel()->selectedRows())
    {
        // Get cloud
        auto& cloud = CloudStore::getInstance().getCloud(index.row() - deleted);
        // Remove cloud from visualizer
        cloud->setVisible(false);
        updateOnVisibility(cloud);
        // Remove the cloud
        mCloudModel->removeCloud(index.row() - deleted);
        // Inc deleted
        ++deleted;
    }
}

void SmogMainWindow::on_actionPrint_camera_details_triggered()
{
    for(size_t i = 0; i < CloudStore::getInstance().getNumberOfClouds(); ++i)
    {
        auto& cloudEntry = CloudStore::getInstance().getCloud(i);
        auto adaptiveCloudEntry = dynamic_cast<AdaptiveCloudEntry*>(cloudEntry.get());
        if(adaptiveCloudEntry)
            adaptiveCloudEntry->updateVisualization(&ui->CloudVisualizer->visualizer());
    }
    ui->CloudVisualizer->update();
}
