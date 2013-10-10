#ifndef ADAPTIVECLOUDENTRY_HPP
#define ADAPTIVECLOUDENTRY_HPP

#include "CloudEntry.hpp"
// Std
#include <vector>
#include <tuple>
#include <utility>
// Pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
// Las type cloud data
#include "LasCloudData.hpp"
// Math
#include "Vector3.hpp"

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

    // Type of subcloud <center,cloudptr>
    typedef std::pair<math::Vector3d,LasCloudData::Ptr> SubCloud;
    // Type of subcloud level <zoom,SubCloud>
    typedef std::pair<double,std::vector<SubCloud> > SubCloudLevel;
    // Subclouds <center,zoom,cloudptr>
    std::vector<SubCloudLevel> mSubclouds;
    // Need to update
    bool mNeedToUpdate;
};

#endif // ADAPTIVECLOUDENTRY_HPP
