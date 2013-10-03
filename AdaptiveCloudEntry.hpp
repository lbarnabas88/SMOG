#ifndef ADAPTIVECLOUDENTRY_HPP
#define ADAPTIVECLOUDENTRY_HPP

#include "CloudEntry.hpp"
// Std
#include <vector>
// Pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
// Las type cloud data
#include "LasCloudData.hpp"

class AdaptiveCloudEntry : public CloudEntry
{
public:
    // Contructor
    AdaptiveCloudEntry();
    // Visualize
    void visualize(pcl::visualization::PCLVisualizer* visualizer);
    // Update cloud on visualizer
    void updateVisualization(pcl::visualization::PCLVisualizer* visualizer);
    // Size of a segment
    static size_t SegmentSize;
    // Thin cloud by level
    static void thinCloudBy(CloudData::Ptr cloudData, float level);
    // Thin cloud to size
    static void thinCloudTo(CloudData::Ptr cloudData, size_t size);
private:
    // Load reimplement
    void loadImpl();
    // Slice and thin
    void build(CloudData::Ptr cloudData);
    // Subclouds
    std::vector<std::vector<LasCloudData::Ptr> > mSubclouds;
    // Need to update
    bool mNeedToUpdate;
};

#endif // ADAPTIVECLOUDENTRY_HPP
