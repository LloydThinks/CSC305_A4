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
        QVector< double > ambi, diff, spec, reflec;
        double radius, specReflec;
        Sphere() {}
        Sphere(QVector3D c, double rad, double sr, QVector< double > la,
                QVector< double > ld, QVector< double > ls,QVector< double > r, QVector< double > col) {
            center = c;
            radius = rad;
            specReflec = sr;
            ambi.append(la[0]*col[0]);
            ambi.append(la[1]*col[1]);
            ambi.append(la[2]*col[2]);
            diff.append(ld[0]*col[0]);
            diff.append(ld[1]*col[1]);
            diff.append(ld[2]*col[2]);
            spec.append(ls[0]*col[0]);
            spec.append(ls[1]*col[1]);
            spec.append(ls[2]*col[2]);
            reflec = r;
        }
    };
    struct PointLight{
        QVector3D center;
        QVector< double > intensity;
        PointLight() {}
        PointLight(QVector3D c, QVector< double > intens) {
            center = c;
            intensity = intens;
        }
    };
    struct AreaLight {
        QVector3D a, b, c, d;
        QVector3D normal, hStep, vStep;
        QVector< double > intensity, pIntensity;
        int numHSteps, numVSteps;
        AreaLight() {}
        AreaLight(QVector3D ta, QVector3D tb, QVector3D tc, QVector3D td, QVector< double > intens) {
            a = ta;
            b = tb;
            c = tc;
            d = td;
            intensity = intens;
            pIntensity = QVector< double >(3);
            normal = QVector3D::crossProduct((c - a), (b - a)).normalized();
        }
    };
    struct Triangle {
        QVector3D a, b, c, normal;
        QVector<double> ambi, diff, spec;
        double specReflec;
        Triangle() {}
        Triangle(QVector3D va, QVector3D vb, QVector3D vc, double sr,
                 QVector<double> la, QVector<double> ld, QVector<double> ls, QVector<double> col) {
            a = va;
            b = vb;
            c = vc;
            specReflec = sr;
            ambi.append(la[0]*col[0]);
            ambi.append(la[1]*col[1]);
            ambi.append(la[2]*col[2]);
            diff.append(ld[0]*col[0]);
            diff.append(ld[1]*col[1]);
            diff.append(ld[2]*col[2]);
            spec.append(ls[0]*col[0]);
            spec.append(ls[1]*col[1]);
            spec.append(ls[2]*col[2]);
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

private:
    void clear();
    int renderWidth, renderHeight;
    void displayImage();
    QProgressBar* pbar;
    void prepareImageDisplay(QImage* myimage); // converts from Qt to opengl format
    QImage glimage, qtimage;  // paintGL will display the gl formatted image
    // keep the qtimage around for saving (one is a copy of the other

    /// Additional Functions
    void preCalculate();
    QVector< double > traceRay(QVector3D ray, QVector3D origin, int recursiveDepth);
    QVector< double > intersects(QVector3D ray, QVector3D origin, double range);
    QVector< double > shadePoint(QVector3D ray, QVector3D origin, QVector<double> intersectInfo, int recursiveDepth);


    /// Additional Variables
    double sceneAmbience, picturePlaneZ, cameraToPicturePlaneDistance, lightPersistence, unitSegs;
    QVector< Sphere > spheres;
    QVector< PointLight > pointLights;
    QVector< Triangle > triangles;
    QVector< AreaLight > areaLights;


};


#endif
