#include "CloudStore.hpp"
// Std
#include <exception>
// Qt
#include <QTextStream>
#include <QSettings>
#include <QFileInfo>
// Pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
// Backend
#include <PcdCloudData.hpp>

CloudStore &CloudStore::getInstance()
{
    static CloudStore instance;
    return instance;
}

void CloudStore::addCloud(const QString &name, const QString &filepath)
{
    // Determine the file type
    QFileInfo fileinfo(filepath);
    // New cloud
    std::shared_ptr<CloudEntry> newCloud(new CloudEntry());
    // Set fields
    newCloud->setName(name);
    newCloud->setFilePath(filepath);
    // Create cloud
    if(fileinfo.suffix() == "las")
    {
        // TODO create las type cloud
    }
    else if(fileinfo.suffix() == "pcd")
    {
        // Create pcd cloud
        newCloud->mData.reset(new PcdCloudData());
        newCloud->mData->load(newCloud->getFilePath());
    }
    int row = mClouds.size();
    beginInsertRows(QModelIndex(), row, row);
    // Add new cloud to the list
    mClouds.push_back(newCloud);
    endInsertRows();
}

CloudEntry::Ptr& CloudStore::getCloud(const size_t &index)
{
    if(index >= mClouds.size())
        throw std::overflow_error("[CloudStore] Get cloud overflow!");
    return mClouds[index];
}

const CloudEntry::Ptr &CloudStore::getCloud(const size_t &index) const
{
    if(index >= mClouds.size())
        throw std::overflow_error("[CloudStore] Get cloud overflow!");
    return mClouds[index];
}

int CloudStore::rowCount(const QModelIndex &parent) const
{
    Q_UNUSED(parent);
    return mClouds.size();
}

int CloudStore::columnCount(const QModelIndex &parent) const
{
    Q_UNUSED(parent);
    return 4;
}

QVariant CloudStore::data(const QModelIndex &index, int role) const
{
    // Get cloud
    auto& cloud = getCloud(index.row());
    // Switch role
    switch (role) {
    // Diplay
    case Qt::DisplayRole:
        // Switch column
        switch (index.column()) {
        case 0:
            return cloud->getName();
        }
        break;
    // Set decoration
    case Qt::DecorationRole:
        // Switch column
        switch (index.column()) {
        case 0:
            return Qt::black;
        }
        break;
    }
    // Default empty variant
    return QVariant();
}

QVariant CloudStore::headerData(int section, Qt::Orientation orientation, int role) const
{
    // Switch role
    switch (role) {
    // Display header
    case Qt::DisplayRole:
        // It's horizontal
        if(orientation == Qt::Horizontal)
        {
            // Switch section
            switch (section) {
            case 0:
                return "Color/Name";
            }
        }
        break;
    }
    return QVariant();
}
