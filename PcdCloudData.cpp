#include "PcdCloudData.hpp"

PcdCloudData::PcdCloudData()
{
}

bool PcdCloudData::load(const QString &filepath)
{
    // Clear
    mCloudBlob.reset(new pcl::PCLPointCloud2);
    // Load
    if(pcl::io::loadPCDFile(filepath.toStdString(), *mCloudBlob) != 0)
    {
        // Load error
        return false;
    }
    // Done
    return true;
}

bool PcdCloudData::save(const QString &filepath)
{
    // Save cloud blob
    pcl::io::savePCDFile(filepath.toStdString(), *mCloudBlob);
    // Done
    return true;
}
