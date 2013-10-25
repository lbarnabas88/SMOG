#ifndef CACHEDATABASE_HPP
#define CACHEDATABASE_HPP

// Qt
#include <QObject>
#include <QString>
#include <QFile>
#include <QSqlDatabase>
#include <QSqlError>

class CacheDatabase : public QObject
{
public:
    // Singleton
    static CacheDatabase &getInstance();
    // Constants
    static const QString CacheTableName;
    static const QString CacheColumnCloudPath;
    static const QString CacheColumnSegmentBase;
    static const QString CacheColumnMap;

    bool openDB();
    bool prepareDB();
    int insertCacheEntry(const QString& cloud_path, const QString& segment_path, const QString& mapStr);
    bool updateCacheEntry(const QString& cloud_path, const QString& segment_path, const QString& mapStr);
    void saveCacheEntry(const QString& cloud_path, const QString& segment_path, const QString& mapStr);
    QString getSegmentPath(const QString& cloud_path);
    QString getCloudPath(const QString& segment_path);
    QString getMap(const QString& cloud_path);
    void closeDB();
    bool deleteDB();
    QSqlError lastError();
private:
    // Singleton
    CacheDatabase(QObject *parent = 0);
    CacheDatabase(const CacheDatabase&); // Non copyable!
    ~CacheDatabase();
    // Get field where...
    QString getField(const QString& what, const QString& where, const QString& value);
    // Database path
    QString mDbPath;
    // Database
    QSqlDatabase mDb;
};

#endif // CACHEDATABASE_HPP
