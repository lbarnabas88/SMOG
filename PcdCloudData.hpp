#ifndef PCDCLOUDDATA_HPP
#define PCDCLOUDDATA_HPP

// Backend
#include "CloudData.hpp"
// Std
#include <memory>
// Pcl
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>

class PcdCloudData : public CloudData
{
public:
    // Contructor
    PcdCloudData();
    // Load cloud
    bool load(const QString& filepath);
    // Save cloud
    bool save(const QString& filepath);
    // Get cloud
    template<typename PointT>
    typename pcl::PointCloud<PointT>::Ptr getCloud();
    // Set cloud
    template<typename PointT>
    void setCloud(const pcl::PointCloud<PointT>& cloud);
    // Get blob
    inline pcl::PCLPointCloud2Ptr getCloudBlob() { return mCloudBlob; }
private:
    // Pcl cloud blob
    pcl::PCLPointCloud2Ptr mCloudBlob;
};

/**
 * Template implementation
 */

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr PcdCloudData::getCloud()
{
    // Point cloud
    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    // Convert blob to cloud
    if(mCloudBlob)
        pcl::fromPCLPointCloud2(*mCloudBlob,*cloud);
    // Return cloud
    return cloud;
}

template<typename PointT>
void PcdCloudData::setCloud(const pcl::PointCloud<PointT> &cloud)
{
    // TODO
}

#endif // PCDCLOUDDATA_HPP
