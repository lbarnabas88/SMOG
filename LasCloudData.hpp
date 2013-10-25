#ifndef LASCLOUDDATA_HPP
#define LASCLOUDDATA_HPP

// Backend
#include "CloudData.hpp"
// Pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// Liblas
#include <liblas/liblas.hpp>

class LasCloudData : public CloudData
{
public:
    // Smart pointer
    typedef std::shared_ptr<LasCloudData> Ptr;
    // Typedef for used point type
    typedef pcl::PointXYZL PointT;
    // Cloud type
    typedef pcl::PointCloud<PointT> CloudT;
    // Constructor
    LasCloudData();
    // Load data
    bool load(const QString &filepath);
    // Save data
    bool save(const QString &filepath);
    // Get data
    inline pcl::PointCloud<PointT>::Ptr getCloud() { return mData; }
    const pcl::PointCloud<PointT>::Ptr getCloud() const { return mData; }
    // Create empty cloud
    pcl::PointCloud<PointT>::Ptr createCloud();
    // Las point format id
    static liblas::PointFormatName PointFormatName;
private:
    // Data
    pcl::PointCloud<PointT>::Ptr mData;
};

#endif // LASCLOUDDATA_HPP
