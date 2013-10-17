#include "AdaptiveCloudEntry.hpp"
// Pcl
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/octree/octree_pointcloud_adjacency.h>
// Backend
#include <LasCloudData.hpp>
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

void AdaptiveCloudEntry::visualize(pcl::visualization::PCLVisualizer *visualizer)
{
    mNeedToUpdate = true;

    if(isVisible())
    {
        // Add new cloud
        pcl::PointCloud<LasCloudData::PointT>::Ptr emptyCloud(new pcl::PointCloud<LasCloudData::PointT>());
        visualizer->addPointCloud<LasCloudData::PointT>(emptyCloud, getName().toStdString());
        // Update visualizer
        updateVisualization(visualizer);
    }
    else
    {
        // Remove cloud
        visualizer->removePointCloud(getName().toStdString());
    }

    // Call update
    updateVisualization(visualizer);
}

void AdaptiveCloudEntry::updateVisualization(pcl::visualization::PCLVisualizer *visualizer)
{
    // If no need to update, return
    //if(!mNeedToUpdate)
    //    return;
    // Cameras vector
    std::vector<pcl::visualization::Camera> cameras;
    // Get cameras
    visualizer->getCameras(cameras);
    // Wrap first camera
    PclCameraWrapper camWrapper(cameras[0]);
    QTextStream out(stdout);
    //out << camWrapper.toString().c_str() << '\n';
    // Camera pos and dir
    math::Vector3d p = camWrapper.getPos();
    math::Vector3d v = camWrapper.getFocal() - p;
    if(v.z == 0)
        return;
    double t = (middlelevel - p.z) / v.z;
    // Reference points
    //math::Vector2f lookPoint = camWrapper.getFocal().toVector2<float>();
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
    // Get level
    SubCloudLevel& level = mSubclouds[levelIndex];
    // Closest
    SubCloud& closest = level.clouds.front();
    // Matrix indices
    math::Vector2<size_t> indices(0,0);

    // Search
    for(size_t i = 0; i < level.map.getWidth(); ++i)
        for(size_t j = 0; j < level.map.getHeight(); ++j)
        {
            int index = level.map(i,j);
            if(index != -1)
            {
                SubCloud& cloud = level.clouds[index];
                if(cloud.area.isPointIn(lookPoint))
                {
                    closest = cloud;
                    indices.set(i,j);
                }
            }
        }
    // Concatenate neighbourhood
    DBOUT("[APC] This segment: " << indices);
    pcl::PointCloud<LasCloudData::PointT>::Ptr slice(new pcl::PointCloud<LasCloudData::PointT>());
    for(size_t i = max(size_t(1),indices.x) - 1; i < min(level.map.getWidth() - 1,indices.x + 1) + 1; ++i)
    {
        for(size_t j = max(size_t(1),indices.y) - 1; j < min(level.map.getHeight() - 1,indices.y + 1) + 1; ++j)
        {
            if(level.map(i,j) != -1)
            {
                DBOUT("[APC] Neigbour segment: " << i << ' ' << j);
                SubCloud& subCloud = level.clouds[level.map(i,j)];
                for(auto& p : *subCloud.cloud->getCloud().get())
                    slice->push_back(p);
            }
        }
    }
    // Update visualizer
    visualizer->updatePointCloud<LasCloudData::PointT>(slice,getName().toStdString());
/*
    // Lookoutpoint visualize
    pcl::PointXYZRGB lookPointViz;
    lookPointViz.x = lookPoint.x;
    lookPointViz.y = lookPoint.y;
    lookPointViz.z = middlelevel;
    lookPointViz.r = 255;
    lookPointViz.g = 10;
    lookPointViz.b = 10;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr lookPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>(1,1,lookPointViz));

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbColorHandler(lookPointCloud);
    if(!visualizer->updatePointCloud<pcl::PointXYZRGB>(lookPointCloud, rgbColorHandler,"LookPoint"))
    {
        visualizer->addPointCloud<pcl::PointXYZRGB>(lookPointCloud, rgbColorHandler, "LookPoint");
    }
*/
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
    // Thinning the cloud
    //QTextStream out(stdout);
    // Log before
    //out << "[AdaptiveCloudEntry] Before thinning: " << cloud->size() << '\n';
    // New size
    size_t newSize = cloud->size() / level;
    // Copy points
    for(size_t i = 0; i < newSize; ++i)
        cloud->at(i) = cloud->at(i * level);
    // Resize the cloud
    cloud->resize(newSize);
    // Log after
    //out << "[AdaptiveCloudEntry] After thinning: " << cloud->size() << '\n';
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

void AdaptiveCloudEntry::build(CloudData::Ptr cloudData)
{
    // Convert to lascloud
    auto cloud = ((LasCloudData*)cloudData.get())->getCloud();
    // If it's enough small, use this
    if(cloud->size() < SegmentSize)
    {
        mData = cloudData;
        return;
    }
    // Clear subclouds
    mSubclouds.clear();
    // Bounding box of the cloud
    std::pair<math::Vector3f,math::Vector3f> bounds;
    // Reset min and max value to first point
    bounds.first = bounds.second = math::Vector3f(cloud->at(0).x, cloud->at(0).y, cloud->at(0).z);
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
    }
    // 2D minmax
    std::pair<math::Vector2f,math::Vector2f> minmax = make_pair(bounds.first.toVector2<float>(),bounds.second.toVector2<float>());
    // Middle level
    middlelevel = (bounds.first.z + bounds.second.z) / 2;
    // Size
    math::Vector2f size = minmax.second - minmax.first;
    // Need to go deeper?
    bool needToGoDeeper = true;
    // Run while needed
    for(size_t depth = 0; needToGoDeeper; ++depth)
    {
        // Reset need to go deeper
        needToGoDeeper = false;
        // Grid number
        size_t gridN = pow(2,depth);
        // Create subcloud level
        mSubclouds.push_back(SubCloudLevel());
        // Reference
        SubCloudLevel& level = mSubclouds.back();
        // Reset zoom
        level.zoom = 4000.0 / depth;
        // Reset map
        level.map.reset(gridN, gridN);
        // Divide
        for(auto& point : *cloud)
        {
            // The point
            math::Vector2f p(point.x, point.y);
            // Diff with grid number
            math::Vector2f diff = (p - minmax.first) * gridN;
            // Calculated 2d indices
            size_t xi = diff.x / size.x, yi = diff.y / size.y;
            // Correct upper bound
            if(xi >= gridN)
                xi = gridN - 1;
            if(yi >= gridN)
                yi = gridN - 1;
            // Get the subcloud, and add. If it's doesn't exist, create
            if(level.map(xi, yi) == -1)
            {
                // Create new subcloud
                level.clouds.push_back(SubCloud());
                // Create cloud data
                level.clouds.back().cloud.reset(new LasCloudData());
                // Create cloud on cloud data
                level.clouds.back().cloud->createCloud();
                // Save pointer to the matrix
                level.map(xi, yi) = level.clouds.size() - 1;
            }
            // Add to this cloud
            level.clouds[level.map(xi, yi)].cloud->getCloud()->push_back(point);
        }
        // Post process on subclouds of the level
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
        }
        // Calc center points
        for(size_t i = 0; i < gridN; ++i)
        {
            for(size_t j = 0; j < gridN; ++j)
            {
                if(level.map(i,j) == -1)
                    continue;
                SubCloud& cloud = level.clouds[level.map(i, j)];
                math::Vector2f gridSize = size / gridN;
                cloud.area.min.set(i * gridSize.x, j * gridSize.y);
                cloud.area.min += minmax.first;
                cloud.area.max = cloud.area.min + gridSize;
                cloud.center = (cloud.area.min + cloud.area.max) / 2.0f;
            }
        }
    }
}
