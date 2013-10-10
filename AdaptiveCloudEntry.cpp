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

size_t AdaptiveCloudEntry::SegmentSize = 6144;

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
    // Calc the needed bigyo
    if(camWrapper.zoomFactor() < 2000)
        visualizer->updatePointCloud<LasCloudData::PointT>((mSubclouds[1].second[1].second)->getCloud(),getName().toStdString());
    else
        visualizer->updatePointCloud<LasCloudData::PointT>((mSubclouds[0].second[0].second)->getCloud(),getName().toStdString());
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
    pcl::octree::OctreePointCloudPointVector<LasCloudData::PointT> divider((double)SegmentSize);
    divider.setInputCloud(cloud);
    divider.addPointsFromInputCloud();

    QTextStream out(stdout);
    // out << "[Divide] Start\n";
    int n = 0;

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
    out.flush();
}
