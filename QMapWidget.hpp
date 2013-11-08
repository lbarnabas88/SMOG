#ifndef QMAPWIDGET_HPP
#define QMAPWIDGET_HPP

// Qt
#include <QtOpenGL/QGLWidget>
#include <QVector>
// Std
#include <vector>
#include <memory>
// Math
#include "Vector3.hpp"
#include "Polygon.hpp"
// Tools
#include "Differencer.hpp"
// Pcl
#include <pcl/point_cloud.h>

class QMapWidget : public QGLWidget
{
    Q_OBJECT
public:
    explicit QMapWidget(QWidget *parent = 0);
    /// Points
    struct PointCloud
    {
        typedef math::Vector3f PointT;
        typedef std::shared_ptr<PointCloud> Ptr;
        PointCloud(const QString &name) : name(name), visible(true) {}
        QString name;
        std::vector<PointT> points;
        bool visible;
        /// SearchHelper
        struct FilterByName
        {
            FilterByName(const QString &name) : name(name) {}
            bool operator()(const PointCloud::Ptr &pc) const { return pc && pc->name == name; }
            QString name;
        };
    };
    /// Point clouds management
    PointCloud::Ptr getCloud(const QString &name);
    bool delCloud(const QString &name);
    /// Fill with pcl cloud
    template<typename PointT>
    void fillWithPclPointCloud(const pcl::PointCloud<PointT>& cloud, const QString &name);
    /// Camera
    void cameraToClouds();
    /// Cutting polygon
    inline const math::Polygonf& getKnifePolygon() const { return mKnifePolygon; }
signals:
    
public slots:
    
protected:
    /// GL functions
    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();
    /// Input functions
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);
    void leaveEvent(QEvent *event);
    void keyPressEvent(QKeyEvent *event);
    // Input helpers
    std::shared_ptr<Differencer<QPoint> > mMouseDiff;
    /// Point clouds
    std::vector<PointCloud::Ptr> mClouds;
    /// Camera
    float mCameraScale;
    math::Vector3f mCameraPos;
    math::Vector3f mMin, mMax;
    void applyCamera();
    /// Conversion
    math::Vector2f convertPointToReal(const QPointF& point);
    QPointF convertRealToPoint(const math::Vector2f& real);
    /// Knife polygon
    math::Polygonf mKnifePolygon;
    math::Vector2f* mGrabbedKnifePoint;
    std::shared_ptr<Differencer<math::Vector2f> > mKnifePointDiff;
};

template<typename PointT>
void QMapWidget::fillWithPclPointCloud(const pcl::PointCloud<PointT>& cloud, const QString &name)
{
    PointCloud::Ptr mapCloud = getCloud(name);
    mapCloud->points.clear();
    for(const auto& p : cloud)
        mapCloud->points.push_back(PointCloud::PointT(p.x,p.y,p.z));
    cameraToClouds();
}

#endif // QMAPWIDGET_HPP
