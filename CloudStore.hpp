#ifndef CLOUDSTORE_HPP
#define CLOUDSTORE_HPP

// Std
#include <vector>
#include <memory>
// Qt
#include <QtCore>
#include <QString>
// Backend
#include <CloudEntry.hpp>

class CloudStore
{
public:
    // Singleton
    static CloudStore& getInstance();
    // Add cloud
    void addCloud(const QString& name, const QString& filepath, bool isAdaptive = false);
    // Get number of cloud entries
    inline size_t getNumberOfClouds() const { return mClouds.size(); }
    // Get a cloud
    CloudEntry::Ptr& getCloud(const size_t& index);
    const CloudEntry::Ptr& getCloud(const size_t& index) const;
    // Get cloud index
    size_t getCloudIndex(const QString& name);
    // Remove cloud
    void removeCloud(const QString& name);
    void removeCloud(const size_t& index);
private:
    // Singleton
    CloudStore() {}
    CloudStore(const CloudStore&);
    void operator=(const CloudStore&);
    // List of clouds
    std::vector<CloudEntry::Ptr> mClouds;
};

#endif // CLOUDSTORE_HPP
