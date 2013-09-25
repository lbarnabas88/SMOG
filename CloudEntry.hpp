#ifndef CLOUDENTRY_HPP
#define CLOUDENTRY_HPP

// Std
#include <memory>
// Qt
#include <QtCore>
// Backend
#include "CloudData.hpp"

/**
 * @brief The CloudEntry class
 */
class CloudEntry
{
public:
    // Pointer
    typedef std::shared_ptr<CloudEntry> Ptr;
    // Contructor/Destructor
    CloudEntry() : mVisible(false) {}
    virtual ~CloudEntry() {}
    // Getters/Setters
    inline const QString& getName() const { return mName; }
    inline void setName(const QString& name) { mName = name; }
    inline const QString& getFilePath() const { return mFilePath; }
    inline void setFilePath(const QString& filepath) { mFilePath = filepath; }
    inline const std::shared_ptr<CloudData>& getData() { return mData; }
    inline const bool& isVisible() const { return mVisible; }
    inline void setVisible(const bool& visible) { mVisible = visible; }
private:
    // Fields
    QString mName;
    QString mFilePath;
    CloudData::Ptr mData;
    bool mVisible;
    // Friend connect
    friend class CloudStore;
};

#endif // CLOUDENTRY_HPP
