#ifndef LASCLOUDDATA_HPP
#define LASCLOUDDATA_HPP

// Backend
#include "CloudData.hpp"
// Pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class LasCloudData : public CloudData
{
public:
    // Typedef for used point type
    typedef pcl::PointXYZL PointT;
    // Constructor
    LasCloudData();
    // Load data
    bool load(const QString &filepath);
    // Save data
    bool save(const QString &filepath);
    // Get data
    inline pcl::PointCloud<PointT>::Ptr getCloud() { return mData; }
    const pcl::PointCloud<PointT>::Ptr getCloud() const { return mData; }
private:
    // Data
    pcl::PointCloud<PointT>::Ptr mData;
};

#endif // LASCLOUDDATA_HPP
