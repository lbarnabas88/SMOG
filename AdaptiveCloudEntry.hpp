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
#include "Vector2.hpp"
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
    // Area struct
    struct Area
    {
        math::Vector2f min;
        math::Vector2f max;
        inline const math::Vector2f& pos() const { return min; }
        inline const math::Vector2f size() const { return max - min; }
        inline bool isPointIn(const math::Vector2f& p) { return p.x >= min.x && p.x <= max.x && p.y >= min.y && p.y <= max.y; }
    };

    // Type of subcloud
    struct SubCloud
    {
        math::Vector2f center;
        Area area;
        LasCloudData::Ptr cloud;
    };

    class IndexMap
    {
    public:
        IndexMap() : mWidth(0), mHeight(0)
        {
        }

        IndexMap(size_t width, size_t height) : mWidth(width), mHeight(height)
        {
            resize(mWidth, mHeight);
        }

        void resize(size_t width, size_t height)
        {
            mWidth = width;
            mHeight = height;
            mIndices.resize(mWidth * mHeight, -1);
        }

        void reset()
        {
            mIndices.clear();
            mIndices.resize(mWidth * mHeight, -1);
        }

        void reset(size_t width, size_t height)
        {
            resize(width,height);
            reset();
        }

        inline const size_t& getWidth() const { return mWidth; }
        inline const size_t& getHeight() const { return mHeight; }
        inline const int& index(size_t i, size_t j) const { return mIndices[j * mWidth + i]; }
        inline int& index(size_t i, size_t j) { return mIndices[j * mWidth + i]; }
        inline const int& operator()(size_t i, size_t j) const { return index(i,j); }
        inline int& operator()(size_t i, size_t j) { return index(i,j); }
    private:
        size_t mWidth, mHeight;
        std::vector<int> mIndices;
    };

    // Type of subcloud level
    struct SubCloudLevel
    {
        double zoom;
        IndexMap map;
        std::vector<SubCloud> clouds;
    };
    // Subclouds
    std::vector<SubCloudLevel> mSubclouds;
    // Middle level of cloud
    float middlelevel;
    // Need to update
    bool mNeedToUpdate;
};

#endif // ADAPTIVECLOUDENTRY_HPP
