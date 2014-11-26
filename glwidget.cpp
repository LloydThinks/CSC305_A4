//-------------------------------------------------------------------------------------------
//   Painting with Flowsnakes
// fsnake program modified to use open gl vertex arrays  Brian Wyvill October 2012
// added save/restore and postscript driver November 2012
// fixed memory management November 2012 delete works properly now.
// added progress bar to show memory usage.
//-------------------------------------------------------------------------------------------

#include "glwidget.h"

GLWidget::GLWidget(QWidget *parent)
    : QGLWidget(parent)
{

}

GLWidget::~GLWidget()
{    

}

void GLWidget::clear()
{
     updateGL();
}

void GLWidget::initializeGL()
{
    //Background color will be white
    glClearColor(1.0, 1.0, 1.0, 1.0);
    glShadeModel( GL_FLAT );
    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity();
    glPointSize(5);
}

void GLWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    displayImage();
}

/* 2D */
void GLWidget::resizeGL( int w, int h )
{
    glMatrixMode( GL_PROJECTION );
    glLoadIdentity();
    glOrtho(0.0,GLdouble(w),0,GLdouble(h),-10.0,10.0);
    glFlush();
    glMatrixMode(GL_MODELVIEW);
    glViewport( 0, 0, (GLint)w, (GLint)h );
    cerr << "gl new size "<< w SEP h NL;
    renderWidth = w;
    renderHeight = h;
}

// no mouse events in this demo
void GLWidget::mousePressEvent( QMouseEvent * )
{
}

void GLWidget::mouseReleaseEvent( QMouseEvent *)
{
}

void GLWidget::mouseMoveEvent ( QMouseEvent * )
{
}

// wheel event
void GLWidget::wheelEvent(QWheelEvent *)
{
}

void GLWidget::openImage(QString fileBuf)
{     
    QImage myImage;
    myImage.load(fileBuf);
    prepareImageDisplay(&myImage);
}

void GLWidget::prepareImageDisplay(QImage* myimage)
{   
    glimage = QGLWidget::convertToGLFormat( *myimage );  // flipped 32bit RGBA stored as mi
    updateGL();    
}

void GLWidget::displayImage()
{
    if (glimage.width()==0) {
        cerr << "Null Image\n";
        return;
    } else {
        glRasterPos2i(0,0);
        glDrawPixels(glimage.width(), glimage.height(), GL_RGBA, GL_UNSIGNED_BYTE, glimage.bits());
        glFlush();
    }
}

void GLWidget::saveImage( QString fileBuf )
{
    // there is no conversion  back toQImage
    // hence the need to keep qtimage as well as glimage
    qtimage.save ( fileBuf );   // note it is the qtimage in the right format for saving
}

void GLWidget::makeImage( )
{
    QImage myimage(renderWidth, renderHeight, QImage::Format_RGB32);

    //TODO: Ray trace a simple circle following the tutorial!
    QVector3D cameraPoint(renderWidth / 2, renderHeight / 2, -1);

    QVector3D circleCenter(renderWidth / 2, renderHeight / 2, 10);
    double circleRadius = 20;

    // ONLY NEED TO BE CALCULATED ONCE PER CIRCLE
        double rr = circleRadius * circleRadius;
        // Vector from the cameraPoint to the circleCenter
        QVector3D cPcCVector = (circleCenter - cameraPoint);
        // Magnitude of cPcCVector, squared
        double cc = QVector3D::dotProduct(cPcCVector, cPcCVector);


    for (int i = 0; i < renderWidth/2; i++)
    {
        for (int j = 0; j < renderHeight/2; j++)
        {
            // The current pixel we are trying to draw
            QVector3D pixelPosition(i, j, 0);
            //qDebug() << "pix: " << pixelPosition;

            // Ray to be traced through the scene
            QVector3D ray = (pixelPosition - cameraPoint).normalized();
            //qDebug() << "ray: " << ray;

            // Magnitude of ray from the cameraPoint to when it is
            // perpendicular to the normal of the circleCenter
            double v = QVector3D::dotProduct(cPcCVector, ray);
            v *= 1000;  // Temporary Fix

            // Difference between the circleRadius and distance
            // from the circleCenter to ray when they are perpendicular
            double disc = (rr - (cc - v*v));

            qDebug() << "Three Doubles";
            qDebug() << rr;
            qDebug() << cc;
            qDebug() << v*v;

            qDebug() << "disc: " << disc;  // This number is not correct
            if (disc <= 0) {  // ray does not intersect the circle
                qDebug() << "miss";
            }
            else {  // ray intersects the circle
                //qDebug() << "hit";
            }

        }
    }

    qtimage=myimage.copy(0, 0,  myimage.width(), myimage.height());

    prepareImageDisplay(&myimage);
}
/*
QVector3D Origin = CameraPoint;
QVector3D PixelPosition(i, j, 0);
QVector3D Direction = PixelPosition - Origin;
//Ray : R - O + t D;
//Our circle is at Z = 10
//so, t = 11
QVector3D Intersection = Origin + 11 * Direction;
QVector3D Distance = Intersection - circleCenter;
double length = Distance.length();
//If it's inside, set the pixel to white
//otherwise set it to black
if (length < circleRadius)
{
    //myimage.setPixel(i, j, qRgb(255, 255, 255));
}
else
{
    //myimage.setPixel(i, j, qRgb(0, 0, 0));
}

if ( disc > 0 )  // ray intersects sphere
{
    double d = sqrt(disc);
    QVector3D P = QVector3D(PixelPosition + ((v - d) * Direction));
    myimage.setPixel(i, j, qRgb(255, 255, 255));
}
else
{
    myimage.setPixel(i, j, qRgb(0, 0, 0));
    qDebug() << "BLACK";
}
*/

/* bool GLWidget::isIntersecting(QVector3D pixelPosition, QVector3D ray, QVector3D circleCenter, double radius)
 * Formal Parameters: pixelPosition - The pixel from which the ray is coming from
 *                    ray           - The direction vector that is perpendicular to the lens plane
 *                    circleCenter  - The center of the circle to check intersection against
 *                    radius        - The radius of the circle being passed in
 *
 * Returns: true  - The given ray from pixelPosition intersects with the given circle
 *          false - The given ray from pixelPosition does not intersect the given circle
 */
bool GLWidget::isIntersecting(QVector3D pixelPosition, QVector3D ray, QVector3D circleCenter, double radius)
{
    double radiusSquared = radius * radius;

    QVector3D EO = (circleCenter - pixelPosition).normalized();

    double cSquared = QVector3D::dotProduct(EO, EO);

    double v = QVector3D::dotProduct(EO, ray.normalized());

    double disc = radiusSquared - (cSquared - v*v);

    if ( disc <= 0 )
        return false;  // ray does not intersect sphere
    else
        return true;  // ray intersects sphere
}

void GLWidget::about()
{
    QString vnum;
    QString mess, notes;
    QString title="Images in Qt and Opengl ";

    vnum.setNum ( MYVERSION );
    mess="Qt OpenGl image demo Release Version: ";
    mess = mess+vnum;
    notes = "\n\n News: Every QImage is now on stack, there is no way for memory leak. -- Lucky";
    mess = mess+notes;
    QMessageBox::information( this, title, mess, QMessageBox::Ok );
}

void GLWidget::help()
{
    QString vnum;
    QString mess, notes;
    QString title="qtglimages";

    vnum.setNum ( MYVERSION);
    mess="Simple Image Handling in Qt/Opengl by Brian Wyvill Release Version: ";
    mess = mess+vnum;
    notes = "\n\n Save and Load images for display.  Also Make an image such as output from a ray tracer. ";
    mess = mess+notes;
    QMessageBox::information( this, title, mess, QMessageBox::Ok );
}

