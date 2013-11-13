#include "CacheDatabase.hpp"
// Qt
#include <QSettings>
#include <QDir>
#include <QFileInfo>
#include <QSqlQuery>
#include <QSqlTableModel>
#include <QSqlRecord>
// Tools
#include "log.hpp"

const QString CacheDatabase::CacheTableName = "adaptive_cache";
const QString CacheDatabase::CacheColumnCloudPath = "cloud_path";
const QString CacheDatabase::CacheColumnSegmentBase = "segment_base";
const QString CacheDatabase::CacheColumnMap = "map";

CacheDatabase::CacheDatabase(QObject *parent)
{
    // Unused parent
    Q_UNUSED(parent);
}

CacheDatabase::~CacheDatabase()
{
    closeDB();
}

QString CacheDatabase::getField(const QString &what, const QString &where, const QString &value)
{
    // If same return
    if(what == where)
        return QString::null;
    // Create model
    QSqlTableModel* model = new QSqlTableModel;
    // Set table
    model->setTable(CacheTableName);
    // Edit stategy
    model->setEditStrategy(QSqlTableModel::OnManualSubmit);
    // Set filter
    model->setFilter(QString("%1='%2'").arg(where, value));
    // Run selection
    model->select();
    // Get record
    QSqlRecord record = model->record(0);
    // If not empty
    if(!record.isEmpty())
        // Get value
        return record.value(record.indexOf(what)).toString();
    // Return result
    return QString::null;
}

CacheDatabase &CacheDatabase::getInstance()
{
    static CacheDatabase instance;
    return instance;
}

bool CacheDatabase::openDB()
{
    // Find db driver
    mDb = QSqlDatabase::addDatabase("QSQLITE");
    // Set path
    mDbPath = "tmp/SmogCache.db";
    // Setup db
    mDb.setDatabaseName(mDbPath);
    // Open databse
    bool success = mDb.open();
    // Log
    DBOUT("[CacheDatabase] Open database..." << (success?"DONE":"FAIL"));
    // Return success
    return success;
}

bool CacheDatabase::prepareDB()
{
    // Return value
    bool ret = false;
    // If the database's opened
    if(mDb.isOpen())
    {
        // Sql query object
        QSqlQuery query;
        ret = query.exec(QString("CREATE TABLE %1 (%2 TEXT PRIMARY KEY,%3 TEXT,%4 TEXT)").arg(CacheTableName, CacheColumnCloudPath, CacheColumnSegmentBase, CacheColumnMap));
        DBOUT("[CacheDatabase] Create table..." << (ret?"DONE":"FAIL"));
    }
    // Return the result
    return ret;
}

int CacheDatabase::insertCacheEntry(const QString &cloud_path, const QString &segment_path, const QString &mapStr)
{
    // Return id
    int retId = -1;
    // If database's open
    if(mDb.open())
    {
        // The query
        QSqlQuery query;
        // Exec
        bool ret = query.exec(QString("INSERT INTO %1 VALUES ('%2','%3','%4')").arg(CacheTableName,cloud_path, segment_path, mapStr));

        DBOUT("[CacheDatabase] Insert sql: " << QString("INSERT INTO %1 VALUES ('%2','%3','%4')").arg(CacheTableName,cloud_path, segment_path, mapStr).toStdString());
        // If success
        if(ret)
        {
            // Get the id
            retId = query.lastInsertId().toInt();
        }
        DBOUT("[CacheDatabase] Insert cache data..." << (ret?"DONE":"FAIL:") << ": RetId=" << retId);
    }
    // Return the id
    return retId;
}

bool CacheDatabase::updateCacheEntry(const QString &cloud_path, const QString &segment_path, const QString &mapStr)
{
    // Query
    QSqlQuery query;
    // Exec update
    bool success = query.exec(QString("UPDATE %1 SET %2='%3',%4='%5' WHERE %6='%7'").arg(CacheTableName, CacheColumnSegmentBase, segment_path, CacheColumnMap, mapStr, CacheColumnCloudPath, cloud_path));

    DBOUT("[CacheDatabase] Update sql: " << QString("UPDATE %1 SET %2='%3',%4='%5' WHERE %6='%7'").arg(CacheTableName, CacheColumnSegmentBase, segment_path, CacheColumnMap, mapStr, CacheColumnCloudPath, cloud_path).toStdString());

    DBOUT("[CacheDatabase] Update..." << (success?"SUCCESS":"FAIL"));

    return success;
}

void CacheDatabase::saveCacheEntry(const QString &cloud_path, const QString &segment_path, const QString &mapStr)
{
    if(insertCacheEntry(cloud_path, segment_path, mapStr) == -1)
    {
        DBOUT("[CacheDatabase] Insert fail -> update");
        updateCacheEntry(cloud_path, segment_path, mapStr);
    }
}

QString CacheDatabase::getSegmentPath(const QString &cloud_path)
{
    return getField(CacheColumnSegmentBase, CacheColumnCloudPath, cloud_path);
}

QString CacheDatabase::getCloudPath(const QString &segment_path)
{
    return getField(CacheColumnCloudPath, CacheColumnSegmentBase, segment_path);
}

QString CacheDatabase::getMap(const QString &cloud_path)
{
    return getField(CacheColumnMap, CacheColumnCloudPath, cloud_path);
}

void CacheDatabase::closeDB()
{
    // Close databse
    mDb.close();
}

bool CacheDatabase::deleteDB()
{
    // Close database
    closeDB();
    // Remove file
    return QFile::remove(mDbPath);
}

QSqlError CacheDatabase::lastError()
{
    return mDb.lastError();
}
