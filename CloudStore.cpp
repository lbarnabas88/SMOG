#include "CloudStore.hpp"
// Log
#include "log.hpp"
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
#include <liblas/liblas.hpp>
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

void printLasHeader(std::ostream& out, const liblas::Header& header)
{
    out << "[Filter] Header: ";
    out << "Offset=" << math::Vector3d(header.GetOffsetX(), header.GetOffsetY(), header.GetOffsetZ());
    out << "Min=" << math::Vector3d(header.GetMinX(), header.GetMinY(), header.GetMinZ());
    out << "Max=" << math::Vector3d(header.GetMaxX(), header.GetMaxY(), header.GetMaxZ());
    out << std::endl;
}

void CloudStore::filterVisibleCloudsTo(const math::Polygonf& polygon, const QString &filepath)
{
    try
    {
        /* Old solution: copy from all file
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
            {
                writerprt.reset(new liblas::Writer(ofs, reader.GetHeader()));
                DBOUT("[Filter] Writer header: " << std::endl << writerprt->GetHeader());
            }
            // Print header
            DBOUT("[Filter] Reader Header:" << std::endl << reader.GetHeader());
            // Copy
            while(reader.ReadNextPoint())
            {
                liblas::Point point = reader.GetPoint();
                math::Vector2f filterPoint(point.GetX(), point.GetY());
                if(math::isPointInsidePolygon(polygon,filterPoint))
                {
                    // Correcting offset to the writer offset
                    point.SetX(point.GetX() + (writerprt->GetHeader().GetOffsetX() - reader.GetHeader().GetOffsetX()));
                    point.SetY(point.GetY() + (writerprt->GetHeader().GetOffsetY() - reader.GetHeader().GetOffsetY()));
                    point.SetZ(point.GetZ() + (writerprt->GetHeader().GetOffsetZ() - reader.GetHeader().GetOffsetZ()));
                    // Write point
                    writerprt->WritePoint(point);
                }
            }
        }
        */

        /// New solution: as lasmerge
        /// Step 0: collect cloud references
        std::vector<CloudEntry::Ptr> cloudsToCopy;
        for(CloudEntry::Ptr& cloud : mClouds)
        {
            QFileInfo fileinfo(cloud->getFilePath());
            if(cloud->isVisible() && fileinfo.suffix() == "las")
                cloudsToCopy.push_back(cloud);
        }
        // If no visible cloud -> return
        if(cloudsToCopy.empty())
            return;
        /// Step 1: create merged header
        // The merged header
        liblas::Header mergedHeader;
        // Read header from all the file
        for(size_t i = 0; i < cloudsToCopy.size(); ++i)
        {
            // Cloud reference
            CloudEntry::Ptr cloud = cloudsToCopy[i];
            // Input stream for the reader
            std::ifstream ifs(cloud->getFilePath().toStdString().c_str(), ios::in | ios::binary);
            // Create reader for that file
            liblas::Reader reader(ifs);
            // Get header
            const liblas::Header& header = reader.GetHeader();
            // If it's the first, set as merged
            if(i == 0)
            {
                mergedHeader = header;
            }
            // Else update the merged header
            else
            {
                // Set id
                mergedHeader.SetSystemId("MERGED");
                // Set points records count
                mergedHeader.SetPointRecordsCount(mergedHeader.GetPointRecordsCount() + header.GetPointRecordsCount());
                // Set point reccords by return
                for(size_t i = 0; i < std::min(mergedHeader.GetPointRecordsByReturnCount().size(),header.GetPointRecordsByReturnCount().size()); ++i)
                    mergedHeader.SetPointRecordsByReturnCount(i, mergedHeader.GetPointRecordsByReturnCount()[i] + header.GetPointRecordsByReturnCount()[i]);
                // Set bounds
                mergedHeader.SetMin(std::min(mergedHeader.GetMinX(), header.GetMinX()), std::min(mergedHeader.GetMinY(), header.GetMinY()), std::min(mergedHeader.GetMinZ(), header.GetMinZ()));
                mergedHeader.SetMax(std::max(mergedHeader.GetMaxX(), header.GetMaxX()), std::max(mergedHeader.GetMaxY(), header.GetMaxY()), std::max(mergedHeader.GetMaxZ(), header.GetMaxZ()));
            }
        }
        /// Step 2: Update scale
        // Limits
        int smallest_int = (1<<31)+10;
        int largest_int = smallest_int-1-20;
        // Pack infos into vectors
        math::Vector3d scale(mergedHeader.GetScaleX(), mergedHeader.GetScaleY(), mergedHeader.GetScaleZ());
        math::Vector3d offset(mergedHeader.GetOffsetX(), mergedHeader.GetOffsetY(), mergedHeader.GetOffsetZ());
        math::Vector3d min(mergedHeader.GetMinX(), mergedHeader.GetMinY(), mergedHeader.GetMinZ());
        math::Vector3d max(mergedHeader.GetMaxX(), mergedHeader.GetMaxY(), mergedHeader.GetMaxZ());
        // Correcting scales
        for(size_t i = 0; i < 3; ++i)
        {
            if((max[i] - offset[i]) / scale[i] > largest_int || (min[i] - offset[i]) / scale[i] < smallest_int)
            {
                scale[i] = 0.0000001;
                while((max[i] - offset[i]) / scale[i] > largest_int || (min[i] - offset[i]) / scale[i] < smallest_int)
                    scale[i] *= 10;
            }
        }
        // Set new scale
        mergedHeader.SetScale(scale.x, scale.y, scale.z);
        /// Step 3: write point to the output
        DBOUT("[Filter] Merged header: " << std::endl << mergedHeader);
        // Output stream
        std::ofstream ofs(filepath.toStdString().c_str(), ios::out | ios::binary);
        // Create writer
        liblas::Writer writer(ofs, mergedHeader);
        // For all input
        for(size_t i = 0; i < cloudsToCopy.size(); ++i)
        {
            // Cloud pointer
            CloudEntry::Ptr cloud = cloudsToCopy[i];
            // Input stream
            std::ifstream ifs(cloud->getFilePath().toStdString().c_str(), ios::in | ios::binary);
            // Create reader
            liblas::Reader reader(ifs);
            // Get header
            const liblas::Header& header = reader.GetHeader();
            // Log
            DBOUT("[Filter] Header: " << std::endl << header);
            // Same offset and scale
            bool same = true;
            // Chack if the same
            if(mergedHeader.GetOffsetX() != header.GetOffsetX()) same = false;
            if(mergedHeader.GetOffsetY() != header.GetOffsetY()) same = false;
            if(mergedHeader.GetOffsetZ() != header.GetOffsetZ()) same = false;
            if(mergedHeader.GetScaleX() != header.GetScaleX()) same = false;
            if(mergedHeader.GetScaleY() != header.GetScaleY()) same = false;
            if(mergedHeader.GetScaleZ() != header.GetScaleZ()) same = false;
            // All point from this cloud
            while(reader.ReadNextPoint())
            {
                // Get point
                liblas::Point point = reader.GetPoint();
                // If the point isn't inside the polygon
                if(!math::isPointInsidePolygon(polygon, math::Vector2f(point.GetX(), point.GetY())))
                    continue;
                // Correction if not the same
                if(!same)
                {
                    point.SetX(point.GetX() - (mergedHeader.GetOffsetX() - header.GetOffsetX()));
                    point.SetY(point.GetY() - (mergedHeader.GetOffsetY() - header.GetOffsetY()));
                    point.SetZ(point.GetZ() - (mergedHeader.GetOffsetZ() - header.GetOffsetZ()));

                    /*point.SetX(0.5 + (point.GetX() - mergedHeader.GetOffsetX()) / mergedHeader.GetScaleX());
                    point.SetY(0.5 + (point.GetY() - mergedHeader.GetOffsetY()) / mergedHeader.GetScaleY());
                    point.SetZ(0.5 + (point.GetZ() - mergedHeader.GetOffsetZ()) / mergedHeader.GetScaleZ());*/
                }
                // Write the point
                writer.WritePoint(point);
            }
        }
    }
    catch (std::exception const& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
    }
}
