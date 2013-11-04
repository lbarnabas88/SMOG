#include "QMapWidget.hpp"

#include <QtGui/QMouseEvent>

#include <GL/glu.h>

#include "log.hpp"
#include "Vector2.hpp"

QMapWidget::QMapWidget(QWidget *parent) :
    QGLWidget(parent), mCameraScale(1.0f) ,mCameraPos(0,0,0)
{
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
    glOrtho(0, w, 0, h, 0, 1000000); // set origin to bottom left corner
    //gluPerspective(45, (float)w / (float)h, 1, 10000);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

math::Vector3f hsv2rgb(float h, float s, float v)
{
    float C = v * s;
    float X = C * (1 - fabs( (int)(h / 60) % 2 - 1) );
    float m = v - C;
    math::Vector3f rgb;
    if(h >= 0 && h < 60)
        rgb.set(C,X,0);
    if(h >= 60 && h < 120)
        rgb.set(X,C,0);
    if(h >= 120 && h < 180)
        rgb.set(0,C,X);
    if(h >= 180 && h < 240)
        rgb.set(0,X,C);
    if(h >= 240 && h < 300)
        rgb.set(X,0,C);
    if(h >= 300 && h < 360)
        rgb.set(C,0,X);
    return rgb + math::Vector3f(m,m,m);
}

void QMapWidget::paintGL()
{
    applyCamera();
    glBegin(GL_POINTS);
    for(PointCloud::Ptr& cloud : mClouds)
    {
        DBOUT("[Map] Draw cloud: " << cloud->name.toStdString() << ", number of points: " << cloud->points.size());
        for(PointCloud::PointT& point : cloud->points)
        {
            float factor = point.z / (mMax.z - mMin.z);
            math::Vector3f color = hsv2rgb(factor * 300.0f,1,1);
            glColor3f(color.x, color.y, color.z);
            glVertex3f(point.x, point.y, point.z);
        }
    }
    glEnd();
}

void QMapWidget::mousePressEvent(QMouseEvent *event)
{
    // Create differencer, if not exists
    if(event->button() == Qt::MiddleButton)
        mMouseDiff.reset(new Differencer<QPoint>(QPoint(event->x(), event->y())));

    if(event->button() == Qt::RightButton)
        cameraToClouds();

    DBOUT("[Map] Mouse pressed: Button=" << event->button());
}

void QMapWidget::mouseReleaseEvent(QMouseEvent *event)
{
    DBOUT("[Map] Mouse released: Button=" << event->button());
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
}

const float ZOOM_RATE = 10.f;

void QMapWidget::wheelEvent(QWheelEvent *event)
{
    DBOUT("[Map] Wheel: Diff=" << event->delta());

    // Create unified delta
    float delta = event->delta() / 120.f;
    // Scaling with save
    float prevScale = mCameraScale;
    mCameraScale += mCameraScale * (delta / ZOOM_RATE);
    // Modify offset
    math::Vector2f deltapos(event->x(), height() - event->y());
    // Deltapos

    updateGL();
}

void QMapWidget::leaveEvent(QEvent *event)
{
    Q_UNUSED(event);
    DBOUT("[Map] Leave event");
}

void QMapWidget::applyCamera()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glScalef(mCameraScale, mCameraScale, mCameraScale);
    glTranslatef(-mCameraPos.x, -mCameraPos.y, -mCameraPos.z);
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
