#include "AdaptiveCloudEntry.hpp"
// Std
#include <algorithm>
// Pcl
#include <pcl/filters/voxel_grid.h>
// Backend
#include "LasCloudData.hpp"
#include "CacheDatabase.hpp"
// Qt
#include <QTextStream>
// Cam wrapper
#include "PclCameraWrapper.hpp"
// Log
#include "log.hpp"

size_t AdaptiveCloudEntry::SegmentSize = 100000;

AdaptiveCloudEntry::AdaptiveCloudEntry() : mNeedToUpdate(false)
{
}

void AdaptiveCloudEntry::visualize(pcl::visualization::PCLVisualizer *visualizer, QMapWidget *mapWidget)
{
    if(!isVisible())
    {
        LasCloudData::CloudT::Ptr emptyCloud(new LasCloudData::CloudT());
        visualizer->updatePointCloud<LasCloudData::PointT>(emptyCloud,getName().toStdString());
        // Remove cloud
        visualizer->removePointCloud(getName().toStdString());
        // Remove from map
        mapWidget->delCloud(getName());
    }
    else
    {
        // Call update
        updateVisualization(visualizer, mapWidget);
    }
}

void AdaptiveCloudEntry::updateVisualization(pcl::visualization::PCLVisualizer *visualizer, QMapWidget* mapWidget)
{
    // Cameras vector
    std::vector<pcl::visualization::Camera> cameras;
    // Get cameras
    visualizer->getCameras(cameras);
    // Wrap first camera
    PclCameraWrapper camWrapper(cameras[0]);
    // Camera pos and dir
    math::Vector3d p = camWrapper.getPos();
    math::Vector3d v = camWrapper.getFocal() - p;
    if(v.z == 0)
        return;
    double t = (middlelevel - p.z) / v.z;
    // Reference points
    math::Vector2f lookPoint(p.x + t * v.x, p.y + t * v.y);
    // Get level index
    size_t levelIndex = 0;
    // Search level
    for(size_t i = 0; i < mSubclouds.size(); ++i)
    {
        SubCloudLevel& level = mSubclouds[i];
        if(camWrapper.zoomFactor() < level.zoom)
            levelIndex = i;
    }
    // Return if no level
    if(mSubclouds.empty())
        return;
    // Get level
    SubCloudLevel& level = mSubclouds[levelIndex];
    // Return if no cloud on level
    if(level.clouds.empty())
        return;
    // Matrix indices
    math::Vector2<size_t> indices(0,0);

    // Convert point to indices
    math::Vector2f point = (lookPoint - level.area.min());
    math::Vector2f unitSize = level.area.size();
    unitSize.x /= level.map.getWidth();
    unitSize.y /= level.map.getHeight();
    point.x /= unitSize.x;
    point.y /= unitSize.y;
    math::Vector2<int> intIndices(point);
    // Get the closest valid
    for(size_t i = 0; i < level.map.getWidth(); ++i)
        for(size_t j = 0; j < level.map.getHeight(); ++j)
        {
            if(level.map.isValid(i,j))
                if( (math::Vector2<int>(i,j) - intIndices).length2() < (indices.toVector2<int>() - intIndices).length2() )
                    indices.set(i,j);
        }
    // Concatenate neighbourhood
    math::Vector3<size_t> segment(levelIndex, indices.x, indices.y);
    if(segment == mLastSegment)
        return;
    pcl::PointCloud<LasCloudData::PointT>::Ptr slice(new pcl::PointCloud<LasCloudData::PointT>());
    for(size_t i = std::max(size_t(1),indices.x) - 1; i < std::min(level.map.getWidth() - 1,indices.x + 1) + 1; ++i)
    {
        for(size_t j = std::max(size_t(1),indices.y) - 1; j < std::min(level.map.getHeight() - 1,indices.y + 1) + 1; ++j)
        {
            if(level.map.isValid(i,j))
            {
                SubCloud& subCloud = level.clouds[level.map(i,j)];
                if(!subCloud.loaded)
                    loadSubcloud(levelIndex, i, j);
                for(auto& p : *subCloud.cloud->getCloud().get())
                    slice->push_back(p);
                unloadSubcloud(levelIndex, i, j);
            }
        }
    }
    mLastSegment = segment;
    // Update visualizer
    if(!visualizer->updatePointCloud<LasCloudData::PointT>(slice,getName().toStdString()))
        visualizer->addPointCloud<LasCloudData::PointT>(slice,getName().toStdString());
    // Add to map
    mapWidget->fillWithPclPointCloud<LasCloudData::PointT>(*slice, getName());
}

void AdaptiveCloudEntry::thinCloudBy(CloudData::Ptr cloudData, float level)
{
    // If no data
    if(!cloudData)
        return;
    // Try get as las data
    auto cloud = ((LasCloudData*)cloudData.get())->getCloud();
    // If no cloud
    if(!cloud)
        return;
    // New size
    size_t newSize = cloud->size() / level;
    // Copy points
    for(size_t i = 0; i < newSize; ++i)
        cloud->at(i) = cloud->at(i * level);
    // Resize the cloud
    cloud->resize(newSize);
}

void AdaptiveCloudEntry::thinCloudTo(CloudData::Ptr cloudData, size_t size)
{
    // If no data
    if(!cloudData)
        return;
    // Try get as las data
    auto cloud = ((LasCloudData*)cloudData.get())->getCloud();
    // If no cloud
    if(!cloud)
        return;
    // If cloud size if smaller
    if(cloud->size() <= size)
        return;
    // Call thin by
    thinCloudBy(cloudData, (float)cloud->size() / (float)size);
}

void AdaptiveCloudEntry::loadImpl()
{
    QFileInfo fileinfo(getFilePath());
    if(fileinfo.suffix() != "las")
        return;
    // Load a las file
    mData.reset(new LasCloudData());
    mData->load(getFilePath());
    // Build the tree
    build(mData);
}

void AdaptiveCloudEntry::build(CloudData::Ptr& cloudData)
{
    // Log filepath
    DBOUT("[ACE] File to load: " << getFilePath().toStdString());
    /// USE THIS IF SMALL
    // Convert to lascloud
    auto cloud = ((LasCloudData*)cloudData.get())->getCloud();
    // If it's enough small, use this
    if(cloud->size() < SegmentSize)
    {
        mData = cloudData;
        return;
    }

    /// CLEAR SUBCLOUDS
    // Clear subclouds
    mSubclouds.clear();

    /// CALCULATE BOUNDS AND AVG POINT
    // Bounding box of the cloud
    std::pair<math::Vector3f,math::Vector3f> bounds;
    // Reset min and max value to first point
    bounds.first = bounds.second = math::Vector3f(cloud->at(0).x, cloud->at(0).y, cloud->at(0).z);
    // Avg point
    LasCloudData::PointT avgPoint;
    avgPoint.x = 0.0f;
    avgPoint.y = 0.0f;
    avgPoint.z = 0.0f;
    // Min & max search
    for(auto& point : *cloud)
    {
        if(point.x < bounds.first.x)
            bounds.first.x = point.x;
        if(point.y < bounds.first.y)
            bounds.first.y = point.y;
        if(point.z < bounds.first.z)
            bounds.first.z = point.z;
        if(point.x > bounds.second.x)
            bounds.second.x = point.x;
        if(point.y > bounds.second.y)
            bounds.second.y = point.y;
        if(point.z > bounds.second.z)
            bounds.second.z = point.z;
        avgPoint.x += point.x / cloud->size();
        avgPoint.y += point.y / cloud->size();
        avgPoint.z += point.z / cloud->size();
    }
    // 2D minmax
    std::pair<math::Vector2f,math::Vector2f> minmax = std::make_pair(bounds.first.toVector2<float>(),bounds.second.toVector2<float>());

    /// CALCULATE MIDDLE LEVEL
    // Middle level
    middlelevel = avgPoint.z;

    /// CHECK FOR VALID CACHE
    // Check if it's in the cache and exists
    mSegmentBase = CacheDatabase::getInstance().getSegmentPath(getFilePath());

    DBOUT("[ACE] Segment base: " << (mSegmentBase.isNull()?std::string("NULL"):mSegmentBase.toStdString()));

    // Need to build
    bool needBuild = true;
    /// IF There is a valid cache entry
    if(!mSegmentBase.isNull())
    {
        // Log build from cache
        DBOUT("[ACE] Using cache.");
        // Get map
        QString mapStr = CacheDatabase::getInstance().getMap(getFilePath());
        // Check if all the segments are exists
        QTextStream mapStream(&mapStr,QIODevice::ReadOnly);
        // Coords
        size_t depth, x, y;
        // Read
        while(!mapStream.atEnd())
        {
            // Read coordinates
            mapStream >> depth >> x >> y;
            // Break if no more subcloud
            if(mapStream.atEnd())
                break;
            // Log
            DBOUT("[ACE] Loaded map coords: " << math::Vector3<size_t>(depth, x, y));
            // Grid N
            size_t gridN = depth2gridn(depth);
            // Extend subclouds
            while(depth >= mSubclouds.size())
                addLevel(gridN, bounds);
            // Get level reference
            SubCloudLevel& level = mSubclouds[depth];
            // Create subcloud with need subcloud function
            needSubCloud(level, depth, x, y, minmax.first, minmax.second, false);
        }
    }
    /// ELSE need to build and save cache
    else if(needBuild)
    {
        // Log build new
        DBOUT("[ACE] Build from scatch.");
        /// GENERATE NEW SEGMENT BASE
        // Segment path
        mSegmentBase = getName();
        // While occupied, add number
        for(int i = 2; !CacheDatabase::getInstance().getCloudPath(mSegmentBase).isNull(); ++i)
        {
            mSegmentBase = getName() + QString::number(i);
        }

        /// BUILD SUBCLOUDS
        // Need to go deeper?
        bool needToGoDeeper = true;

        // Run while needed
        for(size_t depth = 0; needToGoDeeper; ++depth)
        {
            /// INIT DEPTH
            // Reset need to go deeper
            needToGoDeeper = false;
            // Grid number
            size_t gridN = depth2gridn(depth);

            /// CREATE SUBCLOUD
            SubCloudLevel& level = addLevel(gridN, bounds);

            /// SORT POINTS INTO SUBCLOUDS
            // Divide
            for(auto& point : *cloud)
            {
                math::Vector2<size_t> indices = point2indices(point.x, point.y, gridN, minmax.first, minmax.second);
                // Add point to the corresponding cloud
                needSubCloud(level, depth, indices.x, indices.y, minmax.first, minmax.second, true).cloud->getCloud()->push_back(point);
            }

            /// PROCESS SUBCLOUDS
            // Post process on subclouds of the level
            int indexOfCurrentSubcloud = 0;
            for(auto& subCloud : level.clouds)
            {
                // Thin clouds if greater
                if(subCloud.cloud->getCloud()->size() > SegmentSize)
                {
                    // Need deeper!
                    needToGoDeeper = true;
                    // Thin this
                    thinCloudTo(subCloud.cloud,SegmentSize);
                }
                // Get indices
                math::Vector2<size_t> indices = level.map.indexOf(indexOfCurrentSubcloud);
                // If valid
                if(level.map.isValid(indices.x, indices.y))
                {
                    // Save cloud
                    saveSubcloud(depth,indices.x,indices.y);
                    // Unload cloud
                    unloadSubcloud(depth,indices.x,indices.y);
                }
                // Increase index of current subcloud
                ++indexOfCurrentSubcloud;
            }
        }
        /// CONVERT MAP TO STR
        // Map string
        QString mapStr;
        // Map stream
        QTextStream mapStream(&mapStr,QIODevice::WriteOnly);
        // Save to cache database
        for(size_t level = 0; level < mSubclouds.size(); ++level)
        {
            // Get map
            const IndexMap& map = mSubclouds[level].map;
            // Write levels
            for(size_t i = 0; i < map.getWidth(); ++i)
                for(size_t j = 0; j < map.getHeight(); ++j)
                    if(map.isValid(i,j))
                        mapStream << level << ' ' << i << ' ' << j << ' ';
        }

        /// SAVE CACHE DATA
        // Save to cache
        CacheDatabase::getInstance().saveCacheEntry(getFilePath(), mSegmentBase, mapStr);
    }

    /// CLEAR BASE SOURCE
    // Delete source
    cloudData.reset();

    /// LOG
    // Print build result
    for(SubCloudLevel& level : mSubclouds)
    {
        DBOUT("[Build Result] Zoom: " << level.zoom << ", Map: W:" << level.map.getWidth() << " H:" << level.map.getHeight());
    }
}

AdaptiveCloudEntry::SubCloudLevel &AdaptiveCloudEntry::addLevel(size_t gridN, const std::pair<math::Vector3f, math::Vector3f> &bounds)
{
    // Create subcloud level
    mSubclouds.push_back(SubCloudLevel());
    // Reset zoom
    double alpha = 0.523599;
    math::Vector3f segmentDimension = (bounds.second - bounds.first) / gridN;
    mSubclouds.back().zoom = (std::max(segmentDimension.x, segmentDimension.y) / 2.0f) / tan(alpha / 2.0f);
    // Reset map
    mSubclouds.back().map.reset(gridN, gridN);
    // Calc area
    mSubclouds.back().area.setMin(bounds.first.toVector2<float>());
    mSubclouds.back().area.setMax(bounds.second.toVector2<float>());
    // Return the new subcloud level
    return mSubclouds.back();
}

AdaptiveCloudEntry::SubCloud &AdaptiveCloudEntry::needSubCloud(AdaptiveCloudEntry::SubCloudLevel &level, size_t depth, size_t x, size_t y, const math::Vector2f &min, const math::Vector2f &max, bool createLoaded)
{
    // Get the subcloud, and add. If it's doesn't exist, create
    if(level.map(x, y) == -1)
    {
        // Create new subcloud
        level.clouds.push_back(SubCloud());
        // Get reference to back
        SubCloud& cloud = level.clouds.back();
        // Create cloud data
        cloud.cloud.reset(new LasCloudData());
        // Create cloud on cloud data
        cloud.cloud->createCloud();
        // Höhö
        cloud.loaded = createLoaded;
        // Set filename
        cloud.filename = genSubcloudFilename(depth, x, y);
        // Save pointer to the matrix
        level.map(x, y) = level.clouds.size() - 1;
        /// Calc area
        // Grid size
        math::Vector2f gridSize = (max - min);
        gridSize.x /= level.map.getWidth();
        gridSize.y /= level.map.getHeight();
        // Calc area
        cloud.area.setMin(min + math::Vector2f(x * gridSize.x, y * gridSize.y));
        cloud.area.setSize(gridSize);
    }
    // Return the subcloud
    return level.clouds[level.map(x, y)];
}

math::Vector2<size_t> AdaptiveCloudEntry::point2indices(float x, float y, size_t gridN, const math::Vector2f &min, const math::Vector2f &max)
{
    // The point
    math::Vector2f p(x, y);
    // Diff with grid number
    math::Vector2f diff = (p - min) * gridN;
    // Size
    math::Vector2f size = max - min;
    // Calculated 2d indices
    math::Vector2<size_t> indices(diff.x / size.x, diff.y / size.y);
    // Correct upper bound
    for(size_t i = 0; i < indices.size; ++i)
        if(indices[i] >= gridN)
            indices[i] = gridN - 1;
    // Return result
    return indices;
}

size_t AdaptiveCloudEntry::depth2gridn(const size_t &depth)
{
    return std::pow(2, depth);
}

// Generate subcloud filename
QString AdaptiveCloudEntry::genSubcloudFilename(size_t level, int xi, int yi)
{
    return mSegmentBase + "_" + QString::number(level) + "_" + QString::number(xi) + "x" + QString::number(yi) + ".las";
}

// Load subcloud
void AdaptiveCloudEntry::loadSubcloud(size_t level, int xi, int yi)
{
    // Return if no level
    if(mSubclouds.empty() || level >= mSubclouds.size())
        return;
    // Sub level
    SubCloudLevel& subLevel = mSubclouds[level];
    // If it's not on the map
    if(!subLevel.map.isValid(xi,yi))
        return;
    // GetCloud
    SubCloud& subCloud = subLevel.clouds[subLevel.map(xi,yi)];
    // If it's loaded, do nothing
    if(subCloud.loaded)
        return;
    // Load data
    subCloud.cloud->load("tmp/" + subCloud.filename);
    // Set loaded to true
    subCloud.loaded = true;
}

// Unload subcloud
void AdaptiveCloudEntry::unloadSubcloud(size_t level, int xi, int yi)
{
    // Return if no level
    if(mSubclouds.empty() || level >= mSubclouds.size())
        return;
    // Sub level
    SubCloudLevel& subLevel = mSubclouds[level];
    // If it's not on the map
    if(!subLevel.map.isValid(xi,yi))
        return;
    // GetCloud
    SubCloud& subCloud = subLevel.clouds[subLevel.map(xi,yi)];
    // If it isn't loaded, do nothing
    if(!subCloud.loaded)
        return;
    // Clear cloud
    subCloud.cloud->createCloud();
    // Isn't loaded
    subCloud.loaded = false;
}

// Save subcloud
void AdaptiveCloudEntry::saveSubcloud(size_t level, int xi, int yi)
{
    // Return if no level
    if(mSubclouds.empty() || level >= mSubclouds.size())
        return;
    // Sub level
    SubCloudLevel& subLevel = mSubclouds[level];
    // If it's not on the map
    if(!subLevel.map.isValid(xi,yi))
        return;
    // GetCloud
    SubCloud& subCloud = subLevel.clouds[subLevel.map(xi,yi)];
    // Save data
    subCloud.cloud->save("tmp/" + subCloud.filename);
}
