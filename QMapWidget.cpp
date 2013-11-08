#include "QMapWidget.hpp"

#include <QtGui/QMouseEvent>

#include <GL/glu.h>
#include <QVector2D>

#include "log.hpp"
#include "Vector2.hpp"
#include "Color.hpp"

QMapWidget::QMapWidget(QWidget *parent) :
    QGLWidget(parent), mCameraScale(1.0f) ,mCameraPos(0,0,0), mGrabbedKnifePoint(NULL)
{
    setFocusPolicy(Qt::StrongFocus);
}

QMapWidget::PointCloud::Ptr QMapWidget::getCloud(const QString &name)
{
    auto it = std::find_if(mClouds.begin(), mClouds.end(), [&name] (const PointCloud::Ptr& pc) { return pc && pc->name == name; });
    // If not exist, create
    if(it == mClouds.end())
    {
        mClouds.push_back(PointCloud::Ptr(new PointCloud(name)));
        it = mClouds.end() - 1;
    }
    // Return the value
    return *it;
}

bool QMapWidget::delCloud(const QString &name)
{
    // Find the existing cloud
    auto it = std::find_if(mClouds.begin(), mClouds.end(), [&name] (const PointCloud::Ptr& pc) { return pc && pc->name == name; });
    // If not exist, create
    if(it != mClouds.end())
    {
        mClouds.erase(it);
        return true;
    }
    return false;
}

void QMapWidget::initializeGL()
{
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_COLOR_MATERIAL);
    glEnable(GL_BLEND);
    glEnable(GL_POLYGON_SMOOTH);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glClearColor(0, 0, 0, 0);
}

void QMapWidget::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, w, 0, h, -1, 1000000); // set origin to bottom left corner
    //gluPerspective(45, (float)w / (float)h, 1, 10000);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void QMapWidget::paintGL()
{
    /// Apply camera
    applyCamera();
    /// Draw clouds
    glBegin(GL_POINTS);
    for(PointCloud::Ptr& cloud : mClouds)
    {
        DBOUT("[Map] Draw cloud: " << cloud->name.toStdString() << ", number of points: " << cloud->points.size());
        for(PointCloud::PointT& point : cloud->points)
        {
            float factor = point.z / (mMax.z - mMin.z);
            glColor3fv(graphics::Colorf::fromHSV(factor * 300.0f,1,1).array());
            glVertex3fv(point.array());
        }
    }
    glEnd();
    /// Draw knife polygon
    glBegin(mKnifePolygon.size() < 2 ? GL_POINTS : GL_LINE_STRIP);
    {
        glColor3fv(graphics::Colorf::Red.array());
        for(auto& kp : mKnifePolygon)
            glVertex2fv(kp.array());
        if(!mKnifePolygon.empty())
            glVertex2fv(mKnifePolygon.front().array());
    }
    glEnd();

    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glPointSize(5);
    glBegin(GL_POINTS);
    {
        glColor3fv(graphics::Colorf::Red.array());
        for(auto& kp : mKnifePolygon)
            glVertex2fv(kp.array());
    }
    glEnd();
    glPopAttrib();
}

void QMapWidget::mousePressEvent(QMouseEvent *event)
{
    // Create differencer
    if(event->button() == Qt::MiddleButton)
        mMouseDiff.reset(new Differencer<QPoint>(QPoint(event->x(), event->y())));

    if(event->button() == Qt::RightButton)
    {
        mKnifePolygon.clear();

        updateGL();
    }

    if(event->button() == Qt::LeftButton)
    {
        // If grab point
        for(auto& kp : mKnifePolygon)
        {
            if( (convertRealToPoint(kp) - event->posF()).manhattanLength() < 5 )
            {
                mGrabbedKnifePoint = &kp;
                break;
            }
        }
        // If there is no grabbed point, create new
        if(!mGrabbedKnifePoint)
            mKnifePolygon.push_back(convertPointToReal(event->posF()));
        else
            mKnifePointDiff.reset(new Differencer<math::Vector2f>(convertPointToReal(event->posF())));

        updateGL();
    }

    DBOUT("[Map] Mouse pressed: Button=" << event->button());
}

void QMapWidget::mouseReleaseEvent(QMouseEvent *event)
{
    DBOUT("[Map] Mouse released: Button=" << event->button());
    // Ungrab point on left release
    if(event->button() == Qt::LeftButton)
        mGrabbedKnifePoint = NULL;
}

void QMapWidget::mouseMoveEvent(QMouseEvent *event)
{
    DBOUT("[Map] Mouse Moved: Pos=" << math::Vector2f(event->x(), event->y()));

    // If right button pressed
    if(event->buttons() & Qt::MiddleButton)
    {
        DBOUT("[Map] Right button is pressed");
        QPoint diff = mMouseDiff->calcDiff(event->pos());
        mCameraPos += math::Vector3f(-diff.x(), diff.y(), 0) / mCameraScale;
        updateGL();
    }

    // On grabbed point
    if(mGrabbedKnifePoint)
    {
        math::Vector2f diff = mKnifePointDiff->calcDiff(convertPointToReal(event->posF()));
        (*mGrabbedKnifePoint) += diff;
        updateGL();
    }
}

const float ZOOM_RATE = 10.f;

void QMapWidget::wheelEvent(QWheelEvent *event)
{
    DBOUT("[Map] Wheel: Diff=" << event->delta());

    // Create unified delta
    float delta = event->delta() / 120.f;

    float prevScale = mCameraScale;
    mCameraScale += mCameraScale * (delta / ZOOM_RATE);
    math::Vector2f zoomCenter = convertPointToReal(event->pos());
    float ratio = mCameraScale / prevScale - 1.0f;
    for(size_t i = 0; i < 2; ++i)
        mCameraPos[i] -= ratio * (mCameraPos[i] - zoomCenter[i]);

    updateGL();
}

void QMapWidget::leaveEvent(QEvent *event)
{
    QGLWidget::leaveEvent(event);
    Q_UNUSED(event);
    DBOUT("[Map] Leave event");
}

void QMapWidget::keyPressEvent(QKeyEvent *event)
{
    DBOUT("[Map] Pressed key: " << event->key());

    if(event->key() == Qt::Key_R)
    {
        cameraToClouds();
    }
    else
    {
        QGLWidget::keyPressEvent(event);
    }
}

void QMapWidget::applyCamera()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glScalef(mCameraScale, mCameraScale, mCameraScale);
    glTranslatef(-mCameraPos.x, -mCameraPos.y, -mCameraPos.z);
}

math::Vector2f QMapWidget::convertPointToReal(const QPointF &point)
{
    return math::Vector2f(point.x(), height()-point.y()) / mCameraScale + mCameraPos.toVector2<float>();
}

QPointF QMapWidget::convertRealToPoint(const math::Vector2f &real)
{
    math::Vector2f converted = real;
    converted -= mCameraPos.toVector2<float>();
    converted *= mCameraScale;
    return QPointF(converted.x,height() - converted.y);
}

void QMapWidget::cameraToClouds()
{
    if(mClouds.empty())
        return;
    if(mClouds[0]->points.empty())
        return;

    auto& firstPoint = mClouds[0]->points[0];
    math::Vector3f min(firstPoint.x, firstPoint.y, firstPoint.z), max = min;
    // Bounds
    for(auto& cloud : mClouds)
    {
        for(auto& point : cloud->points)
        {
            if(point.x < min.x)
                min.x = point.x;
            if(point.y < min.y)
                min.y = point.y;
            if(point.z < min.z)
                min.z = point.z;
            if(point.x > max.x)
                max.x = point.x;
            if(point.y > max.y)
                max.y = point.y;
            if(point.z > max.z)
                max.z = point.z;
        }
    }
    // Calc offset and scale
    math::Vector3f size = max - min;
    mCameraScale = std::min(width() / size.x, height() / size.y);
    mCameraPos = min;
    mCameraPos.z = max.z;

    mMin = min, mMax = max;

    DBOUT("[Map] min=" << min << ", max=" << max << ", size=" << size << ", scale=" << mCameraScale << ", width=" << width() << ", height=" << height() << ", camera=" << mCameraPos);
    updateGL();
}
