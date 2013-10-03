#ifndef CLOUDENTRY_HPP
#define CLOUDENTRY_HPP

// Std
#include <memory>
// Qt
#include <QtCore>
// Backend
#include "CloudData.hpp"
// Pcl
#include <pcl/visualization/pcl_visualizer.h>

/**
 * @brief The CloudEntry class
 */
class CloudEntry
{
public:
    // Pointer
    typedef std::shared_ptr<CloudEntry> Ptr;
    // Contructor/Destructor
    CloudEntry() : mVisible(true), mDirty(false) {}
    virtual ~CloudEntry() {}
    // Load and save
    void load(const QString& filepath = QString::null);
    void save(const QString& filepath = QString::null);
    // Visualize
    virtual void visualize(pcl::visualization::PCLVisualizer* visualizer);
    // Getters/Setters
    inline const QString& getName() const { return mName; }
    inline void setName(const QString& name) { mName = name; }
    inline const QString& getFilePath() const { return mFilePath; }
    inline void setFilePath(const QString& filepath) { mFilePath = filepath; }
    inline std::shared_ptr<CloudData>& getData() { return mData; }
    inline const std::shared_ptr<CloudData>& getData() const { return mData; }
    inline const bool& isVisible() const { return mVisible; }
    inline void setVisible(const bool& visible) { mVisible = visible; }
    inline const bool& isDirty() const { return mDirty; }
    inline void setDirty(const bool& dirty) { mDirty = dirty; }
protected:
    // Load & save implementation
    virtual void loadImpl();
    virtual void saveImpl();
    // Fields
    QString mName;
    QString mFilePath;
    CloudData::Ptr mData;
    bool mVisible;
    bool mDirty;
    // Friend connect
    friend class CloudStore;
};

#endif // CLOUDENTRY_HPP
