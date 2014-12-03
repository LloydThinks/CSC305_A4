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
        double radius;
        double ambi[3];
        double diff[3];
        double spec[3];
        Sphere() {}
        Sphere(QVector3D c, double r, double a[3], double d[3], double s[3]) {
            center = c;
            radius = r;
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
    struct LightSphere {
        QVector3D center;
        double radius;
        double intensity[3];
        LightSphere() {}
        LightSphere(QVector3D c, double r, double intens[3]) {
            center = c;
            radius = r;
            intensity[0] = intens[0];
            intensity[1] = intens[1];
            intensity[2] = intens[2];
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

    /// Additional Variables
    double sceneAmbience, cameraDepth, lightFog;
    QVector< Sphere > spheres;
    QVector< LightSphere > lightSpheres;

};


#endif
