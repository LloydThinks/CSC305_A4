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
    struct Sphere {
        QVector3D center;
        double radius;
        double diffR;
        double diffG;
        double diffB;
        Sphere() {}
        Sphere(QVector3D c, double r, double dR, double dG, double dB) {
            center = c;
            radius = r;
            diffR = dR;
            diffG = dG;
            diffB = dB;
        }
    };
    struct LightSphere {
        QVector3D center;
        double radius;
        LightSphere() {}
        LightSphere(QVector3D c, double r) {
            center = c;
            radius = r;
        }
    };

    QVector< Sphere > spheres;
    QVector< LightSphere > lightSpheres;

};


#endif
