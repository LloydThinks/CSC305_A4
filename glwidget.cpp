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

    cameraDepth = 20.0;

    // Initialize Spheres
    spheres.append(Sphere(QVector3D(2.0, 8.0, -2.0), 2.0, 0.7, 0.7, 0.7));
    spheres.append(Sphere(QVector3D(7.0, 7.0, 5.0), 2.0, 0.7, 0.7, 0.7));

    // Initialize Light Sphere
    lightSpheres.append(LightSphere(QVector3D(7.0, 2.5, 8.0), 0.25));
    lightSpheres.append(LightSphere(QVector3D(2.0, 5.0, 5.0), 0.25));
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
    glOrtho(0.0,GLdouble(w),0,GLdouble(h),-10000.0,10000.0);
    glFlush();
    glMatrixMode(GL_MODELVIEW);
    glViewport( 0, 0, (GLint)w, (GLint)h );
    //cerr << "gl new size "<< w SEP h NL;
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
        //cerr << "Null Image\n";
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
    /// Declare Variables
    QImage myimage;
    QVector3D pixelPosition, ray, cameraPosition;
    QVector<double> rayTrace;
    double widthScale, heightScale;

    /// Initilize variables
    myimage = QImage(renderWidth, renderHeight, QImage::Format_RGB32);
    widthScale = 10.0;
    heightScale = renderHeight / (renderWidth / widthScale);
    cameraPosition = QVector3D(widthScale/2.0, heightScale/2.0, cameraDepth);

    qDebug() << "RenderWidth: " << renderWidth;
    qDebug() << "RenderHeight: " << renderHeight;
    qDebug() << "CameraPosition: " << cameraPosition;

    /// Loop through the pixels on the screen, setting each one as you go
    for (int i = 0; i < renderWidth; i++) {
        for (int j = 0; j < renderHeight; j++) {
            // Initialize pixel to be black
            myimage.setPixel(i, (renderHeight - 1 - j), qRgb(0, 0, 0));

            // The current pixel we are trying to draw
            pixelPosition = QVector3D(((double(i) * widthScale) / renderWidth), ((double(j) * heightScale) / renderHeight), 10.0);

            if (i == renderWidth/2 && j == renderHeight/2)
                qDebug() << "PlaneCenter: " << pixelPosition;

            // Ray to be traced through the scene
            ray = (pixelPosition - cameraPosition).normalized();
            if (i == renderWidth/2 && j == renderHeight/2)
                qDebug() << "Ray at Center: " << ray;
            // Trace the ray and return important colour attributes
            rayTrace = traceRay(ray, cameraPosition);

            if (rayTrace[0] != 0) {  // Ray intersects something
                myimage.setPixel(i, (renderHeight - 1 - j), qRgb(rayTrace[1], rayTrace[2], rayTrace[3]));
            }
            else {  // Ray does not intersect anything
                //myimage.setPixel(i, (renderHeight - 1 - j), qRgb(0, 0, 0));
            }
        }
    }
    qtimage = myimage.copy(0, 0,  myimage.width(), myimage.height());
    prepareImageDisplay(&myimage);
}

/* QVector< double > GLWidget::intersects(QVector3D ray)
 *
 * Formal Parameters: ray - The ray to be traced in the scene
 *
 * Returns: eval[0] - 0 if intersects with nothing
 *                    1 if intersects sphere
 *                    2 if intersects light source
 *                    3 if intersects wall
 *          eval[1] - Red colour value of pixel
 *          eval[2] - Green colour value of pixel
 *          eval[3] - Blue colour value of pixel
 *
 */
QVector< double > GLWidget::traceRay(QVector3D ray, QVector3D cameraPosition)
{
    /// Variables
    QVector< double > eval = QVector< double >(4);
    QVector3D sphereCenter, lightSphereCenter, cPcCVector;
    QVector3D surfaceIntersect, surfaceNormal, lightVector, eyeVector, h;
    double cc, v, disc, d, distanceToSurface, shadingR, shadingG, shadingB, angleToLight, L, hToNormal;
    double sphereRadius, sphereRadius2, lightSphereRadius, lightSphereRadius2, closestObject, distanceToSphere;

    // Represents that we have not intersected anything yet
    eval[0] = 0;
    // Initialize the closest object to infinity, so that anything is less than that
    closestObject = INFINITY;


    /// Check ray against every sphere in world space
    for (int sIndex = 0; sIndex < spheres.size(); sIndex++) {

        // The circle to be traced
        sphereCenter = spheres[sIndex].center;
        sphereRadius = spheres[sIndex].radius;

        sphereRadius2 = sphereRadius * sphereRadius;
        // Vector from the cameraPoint to the sphereCenter
        cPcCVector = (sphereCenter - cameraPosition);
        // Magnitude of cPcCVector, squared
        cc = QVector3D::dotProduct(cPcCVector, cPcCVector);

        // Magnitude of ray from the cameraPoint to when it is
        // perpendicular to the normal of the sphereCenter
        v = QVector3D::dotProduct(cPcCVector, ray);

        // Difference between the circleRadius and distance
        // from the sphereCenter to ray when they are perpendicular
        disc = (sphereRadius2 - (cc - v*v));

        if (disc <= 0) {  // ray does not intersect the sphere
        }
        else {  // ray intersects the sphere
            eval[0] = 1;

            d = sqrt(disc);

            surfaceIntersect = cameraPosition + (v - d)*ray;
            distanceToSurface = (surfaceIntersect - cameraPosition).length();
            if (distanceToSurface < closestObject) {
                closestObject = distanceToSurface;
            }

            surfaceNormal = (surfaceIntersect - sphereCenter).normalized();

            // Before any light sources, the light is very faint
            shadingR = 0.1;
            shadingG = 0.1;
            shadingB = 0.1;

            /// See which light sources are acting on this surface intersect
            for (int lIndex = 0; lIndex < lightSpheres.size(); lIndex++) {
                distanceToSphere = (lightSpheres[lIndex].center - surfaceIntersect).length();
                //fallOff = 1 / pow(distanceToSphere /distanceToSphere*distanceToSphere, 5)

                /// Lambertian Shading
                lightVector = (lightSpheres[lIndex].center - surfaceIntersect).normalized();
                angleToLight = QVector3D::dotProduct(surfaceNormal, lightVector);
                L = max(0.0, angleToLight);
                shadingR += (0.6 * (0.7 / pow(distanceToSphere/5, 2)) * L);
                shadingG += (0.6 * (0.7 / pow(distanceToSphere/5, 2)) * L);
                shadingB += (0.6 * (0.7 / pow(distanceToSphere/5, 2)) * L);

                /// Blinn-Phong Shading
                eyeVector = (cameraPosition - surfaceIntersect).normalized();
                h = (eyeVector + lightVector).normalized();
                hToNormal = QVector3D::dotProduct(surfaceNormal, h);
                L = pow(max(0.0, hToNormal), 100);

                shadingR += 0.8 * (0.7 / pow(distanceToSphere/3, 2)) * L;
                shadingG += 0.8 * (0.7 / pow(distanceToSphere/3, 2)) * L;
                shadingB += 0.8 * (0.7 / pow(distanceToSphere/3, 2)) * L;

            }

            // If any light value is over max
            if (shadingR > 1.0 || shadingG > 1.0 || shadingB > 1.0) {
                double maxValue = max(shadingR, max(shadingG, shadingB));
                shadingR = shadingR / maxValue;
                shadingG = shadingG / maxValue;
                shadingB = shadingB / maxValue;
            }

            eval[1] = shadingR*255;
            eval[2] = shadingG*255;
            eval[3] = shadingB*255;

        }
    }

    /// Check ray against every light source in world space
    for (int lIndex = 0; lIndex < lightSpheres.size(); lIndex++) {

        // The light circle to be traced
        lightSphereCenter = lightSpheres[lIndex].center;
        lightSphereRadius = lightSpheres[lIndex].radius;

        lightSphereRadius2 = lightSphereRadius * lightSphereRadius;
        // Vector from the cameraPoint to the lightSphereCenter
        cPcCVector = (lightSphereCenter - cameraPosition);
        // Magnitude of cPcCVector, squared
        double cc = QVector3D::dotProduct(cPcCVector, cPcCVector);

        // Magnitude of ray from the cameraPoint to when it is
        // perpendicular to the normal of the lightSphereCenter
        double v = QVector3D::dotProduct(cPcCVector, ray);

        // Difference between the lightCircleRadius and distance
        // from the lightSphereCenter to ray when they are perpendicular
        double disc = (lightSphereRadius2 - (cc - v*v));

        if (disc <= 0) {  // ray does not intersect the light sphere
            // Add code
            continue;
        }
        else {  // ray intersects the light sphere
            double d = sqrt(disc);

            QVector3D surfaceIntersect = cameraPosition + (v - d)*ray;
            //qDebug() << "Surface Intersect: " << surfaceIntersect;

            double distanceToSurface = (surfaceIntersect - cameraPosition).length();
            //qDebug() << "Distance To Surface: " << distanceToSurface;
            if (distanceToSurface < closestObject) {
                closestObject = distanceToSurface;
                eval[0] = 2;
                eval[1] = 255.0;
                eval[2] = 255.0;
                eval[3] = 255.0;
            }

        }

    }

    return eval;

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

