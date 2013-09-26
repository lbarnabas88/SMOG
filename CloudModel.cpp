#include "CloudModel.hpp"

#include <QTextStream>

CloudModel::CloudModel(CloudStore *cloudStore) : mCloudStoreRef(cloudStore)
{
    Q_ASSERT(cloudStore);
}

int CloudModel::rowCount(const QModelIndex &parent) const
{
    Q_UNUSED(parent);
    return mCloudStoreRef->getNumberOfClouds();
}

int CloudModel::columnCount(const QModelIndex &parent) const
{
    Q_UNUSED(parent);
    return 2;
}

QVariant CloudModel::data(const QModelIndex &index, int role) const
{
    // Get cloud
    auto& cloud = mCloudStoreRef->getCloud(index.row());
    // Switch role
    switch (role) {
    // Diplay
    case Qt::DisplayRole:
        // Switch column
        switch (index.column()) {
        case COLUMN_NAME:
            return cloud->getName() + (cloud->isDirty()?"*":"");
        }
        break;
    // Set decoration
    case Qt::DecorationRole:
        // Switch column
        switch (index.column()) {
        case COLUMN_NAME:
            return Qt::black;
        }
        break;
    // Checked state
    case Qt::CheckStateRole:
        // Switch column
        switch (index.column()) {
        case COLUMN_VISIBILITY:
            return (cloud->isVisible() ? Qt::Checked : Qt::Unchecked);
        }
        break;
    }
    // Return invalid variant
    return QVariant();
}

QVariant CloudModel::headerData(int section, Qt::Orientation orientation, int role) const
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
            case COLUMN_NAME:
                return "Color/Name";
            case COLUMN_VISIBILITY:
                return "Shown";
            }
        }
        break;
    }
    return QVariant();
}

Qt::ItemFlags CloudModel::flags(const QModelIndex &index) const
{
    // Switch column
    switch (index.column()) {
    case 1:
        // Return base + editable
        return QAbstractTableModel::flags(index) | Qt::ItemIsUserCheckable;
    }
    // Return base
    return QAbstractTableModel::flags(index);
}

bool CloudModel::setData(const QModelIndex &index, const QVariant &value, int role)
{
    // Cloud to modify
    auto& cloud = mCloudStoreRef->getCloud(index.row());
    // Switch role
    switch (role) {
    case Qt::CheckStateRole:
        // Switch column
        switch (index.column()) {
        case COLUMN_VISIBILITY:
            cloud->setVisible(value.toBool());
            emit dataChanged(index,index);
            break;
        }
    }
    // Return
    return true;
}

void CloudModel::addCloud(const QString &name, const QString &path)
{
    // Row to insert
    int row = mCloudStoreRef->getNumberOfClouds();
    // Begin insertion
    beginInsertRows(QModelIndex(), row, row);
    // Add cloud to the store
    mCloudStoreRef->addCloud(name, path);
    // End insetrion
    endInsertRows();

}

void CloudModel::removeCloud(const int &index)
{
    // Begin remove
    beginRemoveRows(QModelIndex(), index, index);
    // Remove cloud by index
    mCloudStoreRef->removeCloud(index);
    // End remove
    endRemoveRows();
}
