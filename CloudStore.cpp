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
#include <AdaptiveCloudEntry.hpp>

CloudStore &CloudStore::getInstance()
{
    static CloudStore instance;
    return instance;
}

void CloudStore::addCloud(const QString &name, const QString &filepath, bool isAdaptive)
{
    // New cloud
    std::shared_ptr<CloudEntry> newCloud;
    // Create new cloud
    if(isAdaptive)
        newCloud.reset(new AdaptiveCloudEntry());
    else
        newCloud.reset(new CloudEntry());
    // Set fields
    newCloud->setName(name);
    newCloud->load(filepath);
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

size_t CloudStore::getCloudIndex(const QString &name)
{
    for(size_t i = 0; i < mClouds.size(); ++i)
        if(mClouds[i]->getName() == name)
            return i;
    return mClouds.size();
}

void CloudStore::removeCloud(const QString &name)
{
    removeCloud(getCloudIndex(name));
}

void CloudStore::removeCloud(const size_t &index)
{
    // If out of range
    if(index >= mClouds.size())
        return;
    // Remove from clouds
    mClouds.erase(mClouds.begin() + index);
}
