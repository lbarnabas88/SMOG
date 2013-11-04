#include "CloudEntry.hpp"
// Std
#include <exception>
// Backend
#include "PcdCloudData.hpp"
#include "LasCloudData.hpp"

void CloudEntry::load(const QString &filepath)
{
    // Set filepath if it's not null
    if(!filepath.isNull())
        setFilePath(filepath);
    // Call load implementation
    loadImpl();
}

void CloudEntry::save(const QString &filepath)
{
    // Set filepath if it's not null
    if(!filepath.isNull())
        setFilePath(filepath);
    // Call save implementation
    saveImpl();
}

void CloudEntry::visualize(pcl::visualization::PCLVisualizer *visualizer, QMapWidget *mapWidget)
{
    // Check the visualizer
    if(!visualizer)
        throw std::invalid_argument("[CloudEntry] Visualizer pointer is NULL!");
    // Check data
    if(!mData)
        return;
    // Try to convert
    if(isVisible())
    {
        auto pcd_data = dynamic_cast<PcdCloudData*>(mData.get());
        if(pcd_data)
        {
            auto cloud = pcd_data->getCloud<pcl::PointXYZ>();
            visualizer->addPointCloud(cloud, getName().toStdString());
        }
        auto las_data = dynamic_cast<LasCloudData*>(mData.get());
        if(las_data)
        {
            visualizer->addPointCloud<LasCloudData::PointT>(las_data->getCloud(), getName().toStdString());
        }

        // TODO add to map
    }
    // If it's not visible
    else
    {
        visualizer->removePointCloud(getName().toStdString());

        // TODO remove map
    }
}

void CloudEntry::loadImpl()
{
    // Fileinfo
    QFileInfo fileinfo(getFilePath());
    // Switch on suffix
    if(fileinfo.suffix() == "pcd")
    {
        mData.reset(new PcdCloudData());
    }
    else if(fileinfo.suffix() == "las")
    {
        mData.reset(new LasCloudData);
    }
    // Load data
    if(mData)
    {
        mData->load(getFilePath());
    }
}

void CloudEntry::saveImpl()
{
    // Call sata save
    if(mData)
        mData->save(getFilePath());
}
