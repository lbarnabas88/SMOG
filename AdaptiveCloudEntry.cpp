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

size_t AdaptiveCloudEntry::SegmentSize = 500000;

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
    out << camWrapper.toString().c_str() << '\n';
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
    if(camWrapper.zoomFactor() < 2000)
        levelIndex = 4;
    else if(camWrapper.zoomFactor() < 3000)
        levelIndex = 3;
    else if(camWrapper.zoomFactor() < 4000)
        levelIndex = 2;
    else if(camWrapper.zoomFactor() < 5000)
        levelIndex = 1;
    // Get level
    auto& level = mSubclouds[levelIndex];
    // Closest
    auto& closest = level.second.front();
    // Search
    for(auto& subCloud : level.second)
        if( (lookPoint - subCloud.first).length() < (lookPoint - closest.first).length() )
            closest = subCloud;
    visualizer->updatePointCloud<LasCloudData::PointT>(closest.second->getCloud(),getName().toStdString());
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

void getLeafIndices(pcl::octree::OctreeNode* node, std::vector<int>& indices)
{
    // If branch, recursive
    if(node->getNodeType() == pcl::octree::BRANCH_NODE)
    {
        auto branchNode = static_cast<pcl::octree::OctreePointCloudPointVector<LasCloudData::PointT>::BranchNode*>(node);
        // Call for all child
        for(size_t i = 0; i < 8; ++i)
            if(branchNode->hasChild(i))
                getLeafIndices(branchNode->getChildPtr(i), indices);
    }
    // If leaf, add points
    else if(node->getNodeType() == pcl::octree::LEAF_NODE)
    {
        auto leafNode = static_cast<pcl::octree::OctreePointCloudPointVector<LasCloudData::PointT>::LeafNode*>(node);
        // Copy point indices
        std::vector<int> leafIndices;
        leafNode->getContainerPtr()->getPointIndices(leafIndices);
        for(int& index : leafIndices)
            indices.push_back(index);
    }
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
    // Divide and conquer (höhö)
    //pcl::octree::OctreePointCloudPointVector<LasCloudData::PointT> divider((double)SegmentSize);
    //divider.setInputCloud(cloud);
    //divider.addPointsFromInputCloud();

    QTextStream out(stdout);
    out << "[Divide] Start\n";

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
        // Create temorary cloud matrix
        int cloudMatrix[gridN][gridN];
        for(size_t i = 0; i < gridN; ++i)
            for(size_t j = 0; j < gridN; ++j)
                cloudMatrix[i][j] = -1;
        // Create subcloud level
        mSubclouds.push_back(SubCloudLevel());
        // Reference
        SubCloudLevel& level = mSubclouds.back();
        // Reset zoom
        level.first = 1.0;
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
            if(cloudMatrix[xi][yi] == -1)
            {
                // Create new subcloud
                level.second.push_back(SubCloud());
                // Create cloud data
                level.second.back().second.reset(new LasCloudData());
                // Create cloud on cloud data
                level.second.back().second->createCloud();
                // Save pointer to the matrix
                cloudMatrix[xi][yi] = level.second.size() - 1;
            }
            // Add to this cloud
            level.second[cloudMatrix[xi][yi]].second->getCloud()->push_back(point);
        }
        // Post process on subclouds of the level
        for(auto& subCloud : level.second)
        {
            // Thin clouds if greater
            if(subCloud.second->getCloud()->size() > SegmentSize)
            {
                // Need deeper!
                needToGoDeeper = true;
                // Thin this
                thinCloudTo(subCloud.second,SegmentSize);
            }
        }
        // Calc center points
        for(size_t i = 0; i < gridN; ++i)
            for(size_t j = 0; j < gridN; ++j)
            {
                if(cloudMatrix[i][j] == -1)
                    continue;
                auto& cloud = level.second[cloudMatrix[i][j]];
                math::Vector2f gridSize = size / gridN;
                cloud.first.set((i + 0.5) * gridSize.x,(j + 0.5) * gridSize.y);
                cloud.first += minmax.first;
            }
    }
    // Log the result
    out << "[Divide] Number of levels: " << mSubclouds.size() << '\n';
    for(auto& level : mSubclouds)
    {
        out << "[Divide] Cloud on level: " << level.second.size() << '\n';
    }
    out.flush();

    /*int n = 0;

    n = 0;
    mSubclouds.clear();
    auto end_depth_it = divider.depth_end();

    size_t sumsize = 0;

    for(auto it = divider.depth_begin();it != end_depth_it; ++it)
    {
        // Indices of this sub cloud
        std::vector<int> indices;
        // Get indices
        getLeafIndices(it.getCurrentOctreeNode(), indices);
        // Resize subcloud levels
        if(it.getCurrentOctreeDepth() + 1 > mSubclouds.size())
        {
            mSubclouds.resize(it.getCurrentOctreeDepth() + 1);
            mSubclouds.back().first = 0.0;
        }
        // The new subcloud data object
        LasCloudData::Ptr newSubCloud(new LasCloudData());
        // Create en empty cloud
        auto newCloud = newSubCloud->createCloud();
        // Copy datas to the subcloud
        pcl::copyPointCloud(*cloud, indices, *newCloud);
        // Thin cloud if branch node
        if(it.isBranchNode())
            thinCloudTo(newSubCloud, SegmentSize);
        sumsize += newSubCloud->getCloud()->size();
        // Get level vector
        auto& level = mSubclouds[it.getCurrentOctreeDepth()];
        //
        math::Vector3d center;
        // Add the new cloud
        level.second.push_back(std::pair<math::Vector3d,LasCloudData::Ptr>(center, newSubCloud));

        ++n;
    }
    // Log result
    out << "[AdaptiveCloudEntry] Original point number: " << cloud->size() << ", Segmented and thinned points: " << sumsize << '\n';
    //n = 0;
    //out << "[AdaptiveCloudEntry] Generated levels: " << mSubclouds.size() << '\n';
    //for(auto& level : mSubclouds)
    //{
    //    out << "[AdaptiveCloudEntry] Segment on level" << n++ << ": " << level.size() << '\n';
    //}
    out.flush();*/
}
