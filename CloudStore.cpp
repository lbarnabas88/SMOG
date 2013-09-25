#include "CloudStore.hpp"
// Std
#include <exception>
// Qt
#include <QTextStream>
#include <QSettings>
#include <QFileInfo>
// Pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
// Backend
#include "PcdCloudData.hpp"
#include "LasCloudData.hpp"

CloudStore &CloudStore::getInstance()
{
    static CloudStore instance;
    return instance;
}

void CloudStore::addCloud(const QString &name, const QString &filepath)
{
    // Determine the file type
    QFileInfo fileinfo(filepath);
    // New cloud
    std::shared_ptr<CloudEntry> newCloud(new CloudEntry());
    // Set fields
    newCloud->setName(name);
    newCloud->setFilePath(filepath);
    // Create cloud
    if(fileinfo.suffix() == "las")
    {
        // Create las cloud
        newCloud->mData.reset(new LasCloudData());
        newCloud->mData->load(newCloud->getFilePath());
    }
    else if(fileinfo.suffix() == "pcd")
    {
        // Create pcd cloud
        newCloud->mData.reset(new PcdCloudData());
        newCloud->mData->load(newCloud->getFilePath());
    }
    // Add new cloud to the list
    mClouds.push_back(newCloud);
}

CloudEntry::Ptr& CloudStore::getCloud(const size_t &index)
{
    if(index >= mClouds.size())
        throw std::overflow_error("[CloudStore] Get cloud overflow!");
    return mClouds[index];
}

const CloudEntry::Ptr &CloudStore::getCloud(const size_t &index) const
{
    if(index >= mClouds.size())
        throw std::overflow_error("[CloudStore] Get cloud overflow!");
    return mClouds[index];
}
