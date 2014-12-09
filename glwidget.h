//-------------------------------------------------------------------------------------------
//  University of Victoria Computer Science Department
//	FrameWork for OpenGL application under QT
//  Course title: Computer Graphics CSC305
//-------------------------------------------------------------------------------------------
//These two lines are header guiardians against multiple includes
#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QGLWidget>
#include <QProgressBar>
#include "foundation.h"
#include <QtGui>
#include <QtOpenGL>
#include <math.h>

//This is our OpenGL Component we built it on top of QGLWidget
class GLWidget : public QGLWidget
{
    Q_OBJECT

    struct Sphere {
        QVector3D center;
        double radius, ambi[3], diff[3], spec[3], specReflec;
        Sphere() {}
        Sphere(QVector3D c, double r, double sr, double a[3], double d[3], double s[3]) {
            center = c;
            radius = r;
            specReflec = sr;
            ambi[0] = a[0];
            ambi[1] = a[1];
            ambi[2] = a[2];
            diff[0] = d[0];
            diff[1] = d[1];
            diff[2] = d[2];
            spec[0] = s[0];
            spec[1] = s[1];
            spec[2] = s[2];
        }
    };
    struct PointLight{
        QVector3D center;
        double intensity[3];
        PointLight() {}
        PointLight(QVector3D c, double intens[3]) {
            center = c;
            intensity[0] = intens[0];
            intensity[1] = intens[1];
            intensity[2] = intens[2];
        }
    };
    struct AreaLight {
        QVector3D a, b, c, d;
        QVector3D normal;
        double intensity[3];
        AreaLight() {}
        AreaLight(QVector3D ta, QVector3D tb, QVector3D tc, QVector3D td, double intens[3]) {
            a = ta;
            b = tb;
            c = tc;
            d = td;
            intensity[0] = intens[0];
            intensity[1] = intens[1];
            intensity[2] = intens[2];
            normal = QVector3D::crossProduct((c - a), (b - a)).normalized();
        }
    };
    struct Triangle {
        QVector3D a, b, c, normal;
        double ambi[3], diff[3], spec[3], specReflec;
        Triangle() {}
        Triangle(QVector3D va, QVector3D vb, QVector3D vc, double sr, double la[3], double ld[3], double ls[3]) {
            a = va;
            b = vb;
            c = vc;
            specReflec = sr;
            ambi[0] = la[0];
            ambi[1] = la[1];
            ambi[2] = la[2];
            diff[0] = ld[0];
            diff[1] = ld[1];
            diff[2] = ld[2];
            spec[0] = ls[0];
            spec[1] = ls[1];
            spec[2] = ls[2];
            normal = QVector3D::crossProduct((c - a), (b - a)).normalized();
        }
    };

public:
    //Constructor for GLWidget
    GLWidget(QWidget *parent = 0);

    //Destructor for GLWidget
    ~GLWidget();

    void openImage(QString fileBuf);
    void saveImage( QString fileBuf);
    void makeImage();
    void about();
    void help();

protected:
    //Initialize the OpenGL Graphics Engine
    void initializeGL();

    //All our painting stuff are here
    void paintGL();

    //When user resizes main window, the scrollArea will be resized and it will call this function from
    //its attached GLWidget
    void resizeGL(int w, int h);

    //Handle mouse press event in scrollArea
    void mousePressEvent(QMouseEvent * );
    void mouseReleaseEvent(QMouseEvent * );
    //Handle Mouse Move Event
    void mouseMoveEvent(QMouseEvent * );
    void wheelEvent(QWheelEvent * );  // for zoom


private:
    void clear();
    int renderWidth, renderHeight;
    void displayImage();
    QProgressBar* pbar;
    void prepareImageDisplay(QImage* myimage); // converts from Qt to opengl format
    QImage glimage, qtimage;  // paintGL will display the gl formatted image
    // keep the qtimage around for saving (one is a copy of the other

    /// Additional Functions
    QVector< double > traceRay(QVector3D ray, QVector3D cameraPosition);
    QVector< double > traceRay2(QVector3D ray, QVector3D cameraPosition);

    QVector< double > intersects(QVector3D ray, QVector3D origin, double range);
    QVector< double > shadePoint(QVector3D ray, QVector3D origin, QVector<double> intersectInfo);

    QVector< double > sphereIntersection(QVector3D ray, QVector3D cameraPosition, double closestObject);
    QVector< double > lightIntersection(QVector3D ray, QVector3D cameraPosition, double closestObject);
    QVector< double > triangleIntersection(QVector3D ray, QVector3D cameraPosition, double closestObject);

    /// Additional Variables
    double sceneAmbience, lightFog, picturePlaneZ, cameraToPicturePlaneDistance, lightFallOff;
    QVector< Sphere > spheres;
    QVector< PointLight > pointLights;
    QVector< Triangle > triangles;
    QVector< AreaLight > areaLights;

};


#endif
