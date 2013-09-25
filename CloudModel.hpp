#ifndef CLOUDMODEL_HPP
#define CLOUDMODEL_HPP

// Qt
#include <QtCore>
#include <QAbstractTableModel>
// Cloud store
#include "CloudStore.hpp"
// Pcl
#include <pcl/visualization/cloud_viewer.h>

class CloudModel : public QAbstractTableModel
{
    Q_OBJECT
public:
    // Constructor
    CloudModel(CloudStore* cloudStore);
    // Get number of cloumns
    int rowCount(const QModelIndex &parent) const;
    // Get number of rows
    int columnCount(const QModelIndex &parent) const;
    // Get data
    QVariant data(const QModelIndex &index, int role) const;
    // Get header data
    QVariant headerData(int section, Qt::Orientation orientation, int role) const;
    // Flag for item
    Qt::ItemFlags flags(const QModelIndex &index) const;
    // Set data
    bool setData(const QModelIndex &index, const QVariant &value, int role);
    // Add cloud to the model
    void addCloud(const QString& name, const QString& path);
private:
    // Cloud store
    CloudStore* mCloudStoreRef;
};

#endif // CLOUDMODEL_HPP