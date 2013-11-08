#include "CloudStore.hpp"
// Std
#include <exception>
#include <iostream>
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
// Liblas
#include <liblas/reader.hpp>
#include <liblas/writer.hpp>
// Math
#include "Polygon.hpp"

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

void CloudStore::filterVisibleCloudsTo(const math::Polygonf& polygon, const QString &filepath)
{
    try
    {
        // Las output
        std::ofstream ofs(filepath.toStdString().c_str(), ios::out | ios::binary);
        std::shared_ptr<liblas::Writer> writerprt;

        for(CloudEntry::Ptr& cloud : mClouds)
        {
            // Only if visible
            if(!cloud->isVisible())
                continue;
            // Only if .las
            QFileInfo fileinfo(cloud->getFilePath());
            if(fileinfo.suffix() != "las")
                continue;
            // Las input from cloud->getFilePath()
            std::ifstream ifs(cloud->getFilePath().toStdString().c_str(), ios::in | ios::binary);
            liblas::Reader reader(ifs);
            // Create writer
            if(!writerprt)
                writerprt.reset(new liblas::Writer(ofs, reader.GetHeader()));
            // Copy
            while(reader.ReadNextPoint())
            {
                const liblas::Point& point = reader.GetPoint();
                math::Vector2f filterPoint(point.GetX(), point.GetY());
                if(math::isPointInsidePolygon(polygon,filterPoint))
                    writerprt->WritePoint(point);
            }
        }
    }
    catch (std::exception const& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
    }
}
