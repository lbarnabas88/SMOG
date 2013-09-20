#ifndef CLOUDDATA_HPP
#define CLOUDDATA_HPP

// Std
#include <memory>
// Qt
#include <QString>
// Pcl
#include <pcl/point_cloud.h>

class CloudData
{
public:
    // Pointer
    typedef std::shared_ptr<CloudData> Ptr;
    // Constructor
    CloudData();
    // Destructor
    virtual ~CloudData() {}
    // Load cloud
    virtual bool load(const QString& filepath) = 0;
    // Save cloud
    virtual bool save(const QString& filepath) = 0;
};

#endif // CLOUDDATA_HPP
