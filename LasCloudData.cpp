#include "LasCloudData.hpp"
// Std
#include <fstream>
// Liblas
#include <liblas/liblas.hpp>

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
    // Copy data
    for(size_t i = 0; reader.ReadNextPoint(); ++i)
    {
        // Get points reference
        liblas::Point const& p = reader.GetPoint();
        PointT& q = (*mData)[i];
        // Copy xyz
        q.x = p.GetRawX();
        q.y = p.GetRawY();
        q.z = p.GetRawZ();
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
    Q_UNUSED(filepath);
    // Isn't implemented yet
    return false;
}
