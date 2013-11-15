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
#include "Box.hpp"

class AdaptiveCloudEntry : public CloudEntry
{
public:
    // Contructor
    AdaptiveCloudEntry();
    // Visualize
    void visualize(pcl::visualization::PCLVisualizer* visualizer, QMapWidget* mapWidget);
    // Update cloud on visualizer
    void updateVisualization(pcl::visualization::PCLVisualizer* visualizer, QMapWidget* mapWidget);
    // Size of a segment
    static size_t SegmentSize;
    // Maximum depth 0 means unlimited
    static size_t MaxDepth;
    // Use cache
    static bool UseCache;
    // Thin cloud by level
    static void thinCloudBy(CloudData::Ptr cloudData, float level);
    // Thin cloud to size
    static void thinCloudTo(CloudData::Ptr cloudData, size_t size);
private:
    // Load reimplement
    void loadImpl();
    // Slice and thin
    void build(CloudData::Ptr &cloudData);
    // Type of subcloud
    struct SubCloud
    {
        math::Areaf area;
        LasCloudData::Ptr cloud;
        QString filename;
        bool loaded;
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

        bool isIn(size_t i, size_t j) const
        {
            return i < getWidth() && j < getHeight();
        }

        bool isValid(size_t i, size_t j) const
        {
            if(!isIn(i,j))
                return false;
            return index(i,j) >= 0;
        }

        math::Vector2<size_t> indexOf(int value)
        {
            for(size_t i = 0; i < getWidth(); ++i)
                for(size_t j = 0; j < getHeight(); ++j)
                    if(index(i,j) == value)
                        return math::Vector2<size_t>(i,j);
            return math::Vector2<size_t>(getWidth(), getHeight());
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
        math::Areaf area;
        double zoom;
        IndexMap map;
        std::vector<SubCloud> clouds;
    };
    // Subclouds
    std::vector<SubCloudLevel> mSubclouds;
    // Add subcloud level, and get reference to it.
    SubCloudLevel& addLevel(size_t gridN, const std::pair<math::Vector3f,math::Vector3f>& bounds);
    // Add subcloud
    SubCloud& needSubCloud(SubCloudLevel& level, size_t depth, size_t x, size_t y, const math::Vector2f &min, const math::Vector2f &max, bool createLoaded = false);
    // Convert x,y to xi, yi
    static math::Vector2<size_t> point2indices(float x, float y, size_t gridN, const math::Vector2f& min, const math::Vector2f &max);
    // GridN from depth
    static size_t depth2gridn(const size_t& depth);
    // Segment base name
    QString mSegmentBase;
    // Middle level of cloud
    float middlelevel;
    // Generate subcloud filename
    QString genSubcloudFilename(size_t level, int xi, int yi);
    // Load subcloud
    void loadSubcloud(size_t level, int xi, int yi);
    // Unload subcloud
    void unloadSubcloud(size_t level, int xi, int yi);
    // Save subcloud
    void saveSubcloud(size_t level, int xi, int yi);
    // Need to update
    bool mNeedToUpdate;
    // Last visualized segment
    math::Vector3<size_t> mLastSegment;
};

#endif // ADAPTIVECLOUDENTRY_HPP
