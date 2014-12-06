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

    /// Light Settings
    sceneAmbience = 0.2;
    lightFog = 1;

    // Initialize Spheres
    double ambi[3] = {0.7, 0.7, 0.7};
    double diff[3] = {0.7, 0.7, 0.7};
    double spec[3] = {0.6, 0.6, 0.6};
    spheres.append(Sphere(QVector3D(3.0, 7.0, 5.0), 2.0, ambi, diff, spec));
    spheres.append(Sphere(QVector3D(8.0, 3.0, 3.0), 2.0, ambi, diff, spec));
    spheres.append(Sphere(QVector3D(8.0, 6.0, 5.0), 0.5, ambi, diff, spec));

    // Initialize Light Sphere
    double white[3] = {0.8, 0.8, 0.8};
    double yellow[3] = {0.6, 0.6, 0.42};
    lightSpheres.append(LightSphere(QVector3D(2.0, 3.0, 6.0), 0.25, yellow));
    lightSpheres.append(LightSphere(QVector3D(8.0, 8.0, 6.0), 0.25, white));
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

void GLWidget::makeImage()
{
    /// Declare Variables
    QImage myimage;
    QVector3D pixelPosition, ray, cameraPosition;
    QVector<double> rayTrace;
    double widthScale, heightScale;
    QVector<QVector<QVector< double > > > pixelColours = QVector<QVector<QVector< double > > >(renderWidth);

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
        pixelColours[i] = QVector<QVector< double > >(renderHeight);
        for (int j = 0; j < renderHeight; j++) {
            pixelColours[i][j] = QVector< double >(3);
            // Initialize pixel to be black
            pixelColours[i][j][0] = 0.0;
            pixelColours[i][j][1] = 0.0;
            pixelColours[i][j][2] = 0.0;

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
                pixelColours[i][j][0] = rayTrace[1];
                pixelColours[i][j][1] = rayTrace[2];
                pixelColours[i][j][2] = rayTrace[3];
            }
            else {  // Ray does not intersect anything
                ;
                //myimage.setPixel(i, (renderHeight - 1 - j), qRgb(0, 0, 0));
            }
        }
    }

    /// Find maximum colour dilution on the screen
    double maxColourValue;
    double maxDilution = 0.0;
    for (int i = 0; i < renderWidth; i++) {
        for (int j = 0; j < renderHeight; j++) {
            if (pixelColours[i][j][0] > 255.0 || pixelColours[i][j][1] > 255.0 || pixelColours[i][j][2] > 255.0) {
                maxColourValue = max(pixelColours[i][j][0], max(pixelColours[i][j][1], pixelColours[i][j][2]));
                if (maxColourValue > maxDilution)
                    maxDilution = maxColourValue;
            }
        }
    }

    /// If the max dilution in the scene is greater then 255.0
    if (maxDilution > 255.0) {
        qDebug() << maxDilution;
        double ratio = (255.0 / (maxDilution + 1.0));
        for (int i = 0; i < renderWidth; i++) {
            for (int j = 0; j < renderHeight; j++) {
                pixelColours[i][j][0] = ratio * pixelColours[i][j][0];
                pixelColours[i][j][1] = ratio * pixelColours[i][j][1];
                pixelColours[i][j][2] = ratio * pixelColours[i][j][2];
            }
        }
    }

    /// Setting the pixel colours
    for (int i = 0; i < renderWidth; i++) {
        for (int j = 0; j < renderHeight; j++) {
            myimage.setPixel(i, j, qRgb(pixelColours[i][(renderHeight - 1 - j)][0], \
                                        pixelColours[i][(renderHeight - 1 - j)][1], \
                                        pixelColours[i][(renderHeight - 1 - j)][2]));
        }
    }

    qtimage = myimage.copy(0, 0,  myimage.width(), myimage.height());
    prepareImageDisplay(&myimage);
}

/* QVector< double > GLWidget::sphereIntersection(QVector3D ray, QVector3D cameraPosition, double closestObject)
 *
 * Formal Parameters: ray - The ray to be traced in the scene
 *                    cameraPosition - The coordinates of the camera
 *                    closestObject - The distance from the camera to the closest object (so far)
 *
 * Returns: eval[0] - 0 if intersects with nothing
 *                  - 1 if intersects with sphere
 *          eval[1] - Red colour value of pixel
 *          eval[2] - Green colour value of pixel
 *          eval[3] - Blue colour value of pixel
 *          eval[4] - Distance to closest object
 */
QVector< double > GLWidget::sphereIntersection(QVector3D ray, QVector3D cameraPosition, double closestObject)
{
    /// Declarations and Initilizations
    QVector< double > eval(5);  // Vector to be returned
    QVector3D sphereCenter, cPcCVector, surfaceIntersect, surfaceNormal, sphereToLight, eyeVector, h;
    double sphereRadius, cc, v, disc, d, distanceToSurface, sphereToLightDistance;
    double shadingR, shadingG, shadingB, lightToNormal, L, hToNormal;

    eval[0] = 0;  // We have not intersected anything

    /// Check ray against every sphere in world space
    for (int sIndex = 0; sIndex < spheres.size(); sIndex++) {

        // The circle to be traced
        sphereCenter = spheres[sIndex].center;
        sphereRadius = spheres[sIndex].radius;

        // Vector from the cameraPoint to the sphereCenter
        cPcCVector = (sphereCenter - cameraPosition);
        // Magnitude of cPcCVector, squared
        cc = QVector3D::dotProduct(cPcCVector, cPcCVector);

        // Magnitude of ray from the cameraPoint to when it is
        // perpendicular to the normal of the sphereCenter
        v = QVector3D::dotProduct(cPcCVector, ray);

        // Difference between the circleRadius and distance
        // from the sphereCenter to ray when they are perpendicular
        disc = ((sphereRadius*sphereRadius) - (cc - v*v));

        if (disc <= 0) { } // ray does not intersect the sphere
        else {  // ray intersects the sphere
            eval[0] = 1;  // Ray intersects at least one thing
            d = sqrt(disc);

            surfaceIntersect = cameraPosition + (v - d)*ray;
            distanceToSurface = (surfaceIntersect - cameraPosition).length();
            if (distanceToSurface < closestObject) {
                closestObject = distanceToSurface;
            }

            surfaceNormal = (surfaceIntersect - sphereCenter).normalized();

            /// Ambient Shading
            shadingR = (spheres[sIndex].ambi[0] * sceneAmbience);
            shadingG = (spheres[sIndex].ambi[1] * sceneAmbience);
            shadingB = (spheres[sIndex].ambi[2] * sceneAmbience);

            /// See which light sources are acting on this surface intersect
            for (int lIndex = 0; lIndex < lightSpheres.size(); lIndex++) {
                sphereToLight = (lightSpheres[lIndex].center - surfaceIntersect).normalized();
                sphereToLightDistance = (lightSpheres[lIndex].center - surfaceIntersect).length();

                /// Is there any spheres in the way?  If so, shadow exists instead
                bool hitObstacle = false;
                for (int s2Index = 0; s2Index < spheres.size(); s2Index++) {
                    if (s2Index == sIndex)
                        continue;
                    // The circle to be traced
                    QVector3D sphereCenter2 = spheres[s2Index].center;
                    double sphereRadius2 = spheres[s2Index].radius;

                    // Vector from the cameraPoint to the sphereCenter
                    QVector3D cPcCVector2 = (sphereCenter2 - surfaceIntersect);
                    // Magnitude of cPcCVector, squared
                    double cc2 = QVector3D::dotProduct(cPcCVector2, cPcCVector2);

                    // Magnitude of ray from the cameraPoint to when it is
                    // perpendicular to the normal of the sphereCenter
                    double v2 = QVector3D::dotProduct(cPcCVector2, sphereToLight);

                    // Difference between the circleRadius and distance
                    // from the sphereCenter to ray when they are perpendicular
                    double disc2 = ((sphereRadius2*sphereRadius2) - (cc2 - v2*v2));
                    if (disc2 <= 0.01) {}  // Does not intersect
                    else {  // Does intersect a sphere
                        double d2 = sqrt(disc2);
                        QVector3D surfaceIntersect2 = surfaceIntersect + (v2 - d2)*sphereToLight;
                        double distanceToObstacle = (surfaceIntersect2 - surfaceIntersect).length();
                        qDebug() << "distanceToObstacle: " << distanceToObstacle;
                        qDebug() << "sphereToLightDistance: " << sphereToLightDistance;
                        if (distanceToObstacle < sphereToLightDistance) {
                            hitObstacle = true;
                            break;
                        }
                    }
                }
                if (hitObstacle == true)
                    continue;

                /// Lambertian Shading
                lightToNormal = QVector3D::dotProduct(surfaceNormal, sphereToLight);
                L = max(0.0, lightToNormal);

                shadingR += (spheres[sIndex].diff[0] * (lightSpheres[lIndex].intensity[0] / pow(sphereToLightDistance/2, 1)) * L);
                shadingG += (spheres[sIndex].diff[1] * (lightSpheres[lIndex].intensity[1] / pow(sphereToLightDistance/2, 1)) * L);
                shadingB += (spheres[sIndex].diff[2] * (lightSpheres[lIndex].intensity[2] / pow(sphereToLightDistance/2, 1)) * L);

                /// Blinn-Phong Shading
                eyeVector = (cameraPosition - surfaceIntersect).normalized();
                h = (eyeVector + sphereToLight).normalized();
                hToNormal = QVector3D::dotProduct(surfaceNormal, h);
                L = pow(max(0.0, hToNormal), 100);

                shadingR += (spheres[sIndex].spec[0] * (lightSpheres[lIndex].intensity[0] / pow(sphereToLightDistance/2, 1)) * L);
                shadingG += (spheres[sIndex].spec[1] * (lightSpheres[lIndex].intensity[1] / pow(sphereToLightDistance/2, 1)) * L);
                shadingB += (spheres[sIndex].spec[2] * (lightSpheres[lIndex].intensity[2] / pow(sphereToLightDistance/2, 1)) * L);

//                /// Constrain dilution so that it doesn't dim the scene too much
//                if (shadingR > 1.0 || shadingG > 1.0 || shadingB > 1.0) {
//                    double maxDilution = max(shadingR, max(shadingG, shadingB));
//                    shadingR = shadingR / maxDilution;
//                    shadingG = shadingG / maxDilution;
//                    shadingB = shadingB / maxDilution;
//                }
            }

            eval[1] = shadingR*255;
            eval[2] = shadingG*255;
            eval[3] = shadingB*255;
        }
    }
    eval[4] = closestObject;
    return eval;
}

/* QVector< double > GLWidget::lightIntersection(QVector3D ray, QVector3D cameraPosition, double closestObject)
 *
 * Formal Parameters: ray - The ray to be traced in the scene
 *                    cameraPosition - The coordinates of the camera
 *                    closestObject - The distance from the camera to the closest object (so far)
 *
 * Returns: eval[0] - 0 if intersects with nothing
 *                  - 1 if intersects with fog
 *                  - 2 if intersects with light source
 *          eval[1] - Red colour value of pixel
 *          eval[2] - Green colour value of pixel
 *          eval[3] - Blue colour value of pixel
 *          eval[4] - Distance to closest object
 *
 */
QVector< double > GLWidget::lightIntersection(QVector3D ray, QVector3D cameraPosition, double closestObject)
{
    /// Declarations and Initilizations
    QVector< double > eval(5);
    QVector3D lightSphereCenter, cPcCVector, surfaceIntersect, surfaceNormal, lightToCamera;
    double lightSphereRadius, cc, v, disc1, disc2, d, distanceToSurface, fogLevel, normalToCamera;

    eval[0] = 0;  // We have not intersected anything

    /// Check ray against every light source in world space
    for (int lIndex = 0; lIndex < lightSpheres.size(); lIndex++) {

        // The light circle to be traced
        lightSphereCenter = lightSpheres[lIndex].center;
        lightSphereRadius = lightSpheres[lIndex].radius;

        // Vector from the cameraPoint to the lightSphereCenter
        cPcCVector = (lightSphereCenter - cameraPosition);
        // Magnitude of cPcCVector, squared
        cc = QVector3D::dotProduct(cPcCVector, cPcCVector);

        // Magnitude of ray from the cameraPoint to when it is
        // perpendicular to the normal of the lightSphereCenter
        v = QVector3D::dotProduct(cPcCVector, ray);

        // Difference between the lightCircleRadius and distance
        // from the lightSphereCenter to ray when they are perpendicular
        disc1 = ((lightSphereRadius * lightSphereRadius) - (cc - v*v));
        disc2 = (pow((lightSphereRadius + lightFog), 2) - (cc - v*v));

        if (disc2 <= 0) {  // ray does not intersect the light sphere or surrounding light radius
            // Add code
            continue;
        }
        else if (disc1 <= 0) {  // Ray intersects the surrounding light fog, but not the light sphere
            d = sqrt(disc2);
            surfaceIntersect = cameraPosition + (v - d)*ray;
            distanceToSurface = (surfaceIntersect - cameraPosition).length();

            if (distanceToSurface < closestObject) {
                eval[0] = 1;
//                fogLevel = pow((.25 * ( d / (lightSphereRadius + lightFog))), 1.25);
                fogLevel = pow(( d / (lightSphereRadius + lightFog)), 4);
                eval[1] = (255.0 * lightSpheres[lIndex].intensity[0] * fogLevel);
                eval[2] = (255.0 * lightSpheres[lIndex].intensity[1] * fogLevel);
                eval[3] = (255.0 * lightSpheres[lIndex].intensity[2] * fogLevel);
            }
        }
        else {  // ray intersects the light sphere
            d = sqrt(disc1);
            surfaceIntersect = cameraPosition + (v - d)*ray;
            distanceToSurface = (surfaceIntersect - cameraPosition).length();

            if (distanceToSurface < closestObject) {
                closestObject = distanceToSurface;
                eval[0] = 2;

                surfaceNormal = (surfaceIntersect - lightSpheres[lIndex].center).normalized();
                lightToCamera = (cameraPosition - lightSpheres[lIndex].center).normalized();
                normalToCamera = QVector3D::dotProduct(surfaceNormal, lightToCamera);

                /// 255 is max; Each sphere has individual intensity ; falloff function for edges of light sources
                eval[1] = (255.0 * lightSpheres[lIndex].intensity[0]);
                eval[2] = (255.0 * lightSpheres[lIndex].intensity[1]);
                eval[3] = (255.0 * lightSpheres[lIndex].intensity[2]);

            }
        }
    }
    eval[4] = closestObject;
    return eval;
}

/* QVector< double > GLWidget::intersects(QVector3D ray)
 *
 * Formal Parameters: ray - The ray to be traced in the scene
 *
 * Returns: eval[0] - 0 if intersects with nothing
 *                    1 if intersects with fog
 *                    2 if intersects light source
 *                    3 if intersects sphere
 *                    4 if intersects wall
 *          eval[1] - Red colour value of pixel
 *          eval[2] - Green colour value of pixel
 *          eval[3] - Blue colour value of pixel
 */
QVector< double > GLWidget::traceRay(QVector3D ray, QVector3D cameraPosition)
{
    /// Variables
    QVector< double > eval = QVector< double >(4);
    double closestObject;

    // Represents that we have not intersected anything yet
    eval[0] = 0;
    // Initialize the closest object to infinity, so that anything is less than that
    closestObject = INFINITY;

    QVector< double > sphereIntersect = sphereIntersection(ray, cameraPosition, closestObject);
    if (sphereIntersect[0] == 1)  // A sphere was intersected
    {
        eval[0] = 3;
        eval[1] = sphereIntersect[1];
        eval[2] = sphereIntersect[2];
        eval[3] = sphereIntersect[3];
        closestObject = sphereIntersect[4];
    }

    QVector< double > lightIntersect = lightIntersection(ray, cameraPosition, closestObject);
    if (lightIntersect[0] == 1) {  // A light fog was intersected
        eval[0] = 1;
        eval[1] += lightIntersect[1];
        eval[2] += lightIntersect[2];
        eval[3] += lightIntersect[3];
        closestObject = lightIntersect[4];
    }
    else if (lightIntersect[0] == 2) {  // A light was intersected
        eval[0] = 2;
        eval[1] = lightIntersect[1];
        eval[2] = lightIntersect[2];
        eval[3] = lightIntersect[3];
        closestObject = lightIntersect[4];
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

