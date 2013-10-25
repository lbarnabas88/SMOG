#include "LasCloudData.hpp"
// Std
#include <fstream>
// Pcl
#include <pcl/octree/octree.h>

liblas::PointFormatName LasCloudData::PointFormatName = liblas::ePointFormat0;

LasCloudData::LasCloudData()
{
}

bool LasCloudData::load(const QString &filepath)
{
    // Input stream
    std::ifstream ifs(filepath.toStdString().c_str(), std::ios::in | std::ios::binary);
    // Liblas reader factory
    liblas::ReaderFactory readerFactory;
    // Liblas reader
    liblas::Reader reader = readerFactory.CreateWithStream(ifs);
    // Read header
    liblas::Header const& header = reader.GetHeader();
    // Create new data
    mData.reset(new pcl::PointCloud<PointT>());
    // Resize point to las data count
    mData->resize(header.GetPointRecordsCount());
    // Get point format name
    PointFormatName = header.GetDataFormatId();
    // Copy data
    for(size_t i = 0; reader.ReadNextPoint(); ++i)
    {
        // Get points reference
        liblas::Point const& p = reader.GetPoint();
        PointT& q = (*mData)[i];
        // Copy xyz
        q.x = p.GetX();
        q.y = p.GetY();
        q.z = p.GetZ();
        // Copy returns
        q.label = p.GetNumberOfReturns();
        q.label <<= 16;
        q.label |= p.GetReturnNumber();
    }
    // Success
    return true;
}

bool LasCloudData::save(const QString &filepath)
{
    // Output stream
    std::ofstream ofs(filepath.toStdString().c_str(), std::ios::out | std::ios::binary);
    // Create header
    liblas::Header header;
    // Set format
    header.SetDataFormatId(PointFormatName);
    // Create las writer
    liblas::Writer writer(ofs, header);
    // Add points
    for(const PointT& p : (*mData))
    {
        liblas::Point q;
        q.SetX(p.x);
        q.SetY(p.y);
        q.SetZ(p.z);
        writer.WritePoint(q);
    }
    // Success
    return true;
}

pcl::PointCloud<LasCloudData::PointT>::Ptr LasCloudData::createCloud()
{
    // Resem mdata
    mData.reset(new pcl::PointCloud<PointT>());
    // Return the new data
    return mData;
}
