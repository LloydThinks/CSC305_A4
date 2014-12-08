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
    lightFog = 1.0;

    // Initialize Spheres
    double ambi[3] = {0.7, 0.7, 0.7};
    double diff[3] = {0.7, 0.7, 0.7};
    double spec[3] = {0.6, 0.6, 0.6};

    spheres = QVector< Sphere >();
    spheres.append(Sphere(QVector3D(3.0, 7.0, 5.0), 2.0, 100, ambi, diff, spec));
    spheres.append(Sphere(QVector3D(8.0, 3.0, 3.0), 2.0, 100, ambi, diff, spec));
    spheres.append(Sphere(QVector3D(8.0, 6.0, 5.0), 0.5, 100, ambi, diff, spec));

    triangles = QVector< Triangle >();
    // Draw Room - Back Wall
    triangles.append(Triangle(QVector3D(0.0, 0.0, 0.0), QVector3D(0.0, 10.0, 0.0), QVector3D(10.0, 10.0, 0.0), 100, ambi, diff, spec));
    triangles.append(Triangle(QVector3D(0.0, 0.0, 0.0), QVector3D(10.0, 10.0, 0.0), QVector3D(10.0, 0.0, 0.0), 100, ambi, diff, spec));

//    triangles.append(Triangle(QVector3D(0.0, 0.0, 5.0),
//                              QVector3D(0.0, 5.0, -1.0),
//                              QVector3D(5.0, 0.0, 5.0),
//                              100, ambi, diff, spec));

    // Initialize Light Sphere
    double white[3] = {0.8, 0.8, 0.8};
    double yellow[3] = {0.6, 0.6, 0.42};
    pointLights = QVector< PointLight >();
    pointLights.append(PointLight(QVector3D(2.0, 3.0, 4.0), yellow));
    pointLights.append(PointLight(QVector3D(8.0, 8.0, 6.0), white));

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
                pixelColours[i][j][0] = rayTrace[1] * 255.0;
                pixelColours[i][j][1] = rayTrace[2] * 255.0;
                pixelColours[i][j][2] = rayTrace[3] * 255.0;
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
        qDebug() << "Max Dilution: " << maxDilution;
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

/* QVector< double > GLWidget::intersects(QVector3D ray, QVector3D origin, double range)
 *
 * Formal Parameters: ray    - The ray to check intersections against
 *                    origin - The coordinates of where 'ray' originates from
 *                    range  - The max distance away from origin that we care about
 *
 * Returns: eval[0] - 0 if intersects with nothing
 *                  - 1 if intersects with point light
 *                  - 2 if intersects with area light
 *                  - 3 if intersects with sphere
 *                  - 4 if intersects with triangle
 *          eval[1] - Index of object in respective storage
 *          eval[2] - Surface Intersect X
 *          eval[3] - Surface Intersect Y
 *          eval[4] - Surface Intersect Z
 */
QVector< double > GLWidget::intersects(QVector3D ray, QVector3D origin, double range)
{
    /// Declarations and Initilizations
    QVector< double > eval(5);  // Vector to be returned
    QVector3D center, EO, a, b, c, d, e, myNormal;
    double radius, cc, v, disc, distanceToSurface, divisor, rho, beta;

    eval[0] = 0;

    /// Check ray against every sphere in world space
    for (int sIndex = 0; sIndex < spheres.size(); sIndex++) {

        // The circle to be traced
        center = spheres[sIndex].center;
        radius = spheres[sIndex].radius;

        // Vector from the cameraPoint to the sphereCenter
        EO = (center - origin);

        // Magnitude of cPcCVector, squared
        cc = QVector3D::dotProduct(EO, EO);

        // Magnitude of ray from the cameraPoint to when it is
        // perpendicular to the normal of the sphereCenter
        v = QVector3D::dotProduct(EO, ray);

        // Difference between the circleRadius and distance
        // from the sphereCenter to ray when they are perpendicular
        disc = ((radius*radius) - (cc - v*v));

        if (disc > 0) { // ray intersects the sphere
            distanceToSurface = (v - sqrt(disc));

            if (distanceToSurface > 0.01 && distanceToSurface < range) {
                range = distanceToSurface;

                QVector3D surfaceIntersect = origin + distanceToSurface*ray;
                eval[0] = 3;  // Ray intersects at least one sphere
                eval[1] = sIndex;
                eval[2] = surfaceIntersect.x();
                eval[3] = surfaceIntersect.y();
                eval[4] = surfaceIntersect.z();
            }
        }
    }

    /// Check ray against every light source in world space
    for (int lIndex = 0; lIndex < pointLights.size(); lIndex++) {

        // The light circle to be traced
        center = pointLights[lIndex].center;

        // Vector from the cameraPoint to the lightSphereCenter
        EO = (center - origin);
        // Magnitude of cPcCVector, squared
        cc = QVector3D::dotProduct(EO, EO);

        // Magnitude of ray from the cameraPoint to when it is
        // perpendicular to the normal of the lightSphereCenter
        v = QVector3D::dotProduct(EO, ray);

        // Difference between the lightCircleRadius and distance
        // from the lightSphereCenter to ray when they are perpendicular
        disc = ((0.0625) - (cc - v*v));

        if (disc > 0) {
            distanceToSurface = (v - sqrt(disc));

            if (distanceToSurface < range) {
                range = distanceToSurface;
                eval[0] = 1;  // Ray intersects the point light
                eval[1] = lIndex;
                eval[2] = (origin + distanceToSurface*ray).x();
                eval[3] = (origin + distanceToSurface*ray).y();
                eval[4] = (origin + distanceToSurface*ray).z();
            }
        }
    }

    /// Check ray against every triangle in world space
    d = ray;
    e = origin;
    for (int tIndex = 0; tIndex < triangles.size(); tIndex++) {
        a = triangles[tIndex].a;
        b = triangles[tIndex].b;
        c = triangles[tIndex].c;

        divisor = QMatrix4x4((a.x() - b.x()), (a.x() - c.x()), (d.x()), 0.0,\
                                    (a.y() - b.y()), (a.y() - c.y()), (d.y()), 0.0,\
                                    (a.z() - b.z()), (a.z() - c.z()), (d.z()), 0.0,\
                                    0.0, 0.0, 0.0, 1.0).determinant();
        distanceToSurface = QMatrix4x4((a.x() - b.x()), (a.x() - c.x()), (a.x() - e.x()), 0.0,\
                                    (a.y() - b.y()), (a.y() - c.y()), (a.y() - e.y()), 0.0,\
                                    (a.z() - b.z()), (a.z() - c.z()), (a.z() - e.z()), 0.0,\
                                    0.0, 0.0, 0.0, 1.0).determinant() / divisor;
        if (distanceToSurface < 0.01 || distanceToSurface > range) continue;  // Does not intersect
        rho     = QMatrix4x4((a.x() - b.x()), (a.x() - e.x()), (d.x()), 0.0,\
                                    (a.y() - b.y()), (a.y() - e.y()), (d.y()), 0.0,\
                                    (a.z() - b.z()), (a.z() - e.z()), (d.z()), 0.0,\
                                    0.0, 0.0, 0.0, 1.0).determinant() / divisor;
        if (rho < 0 || rho > 1) continue; // Does not intersect
        beta    = QMatrix4x4((a.x() - e.x()), (a.x() - c.x()), (d.x()), 0.0,\
                                    (a.y() - e.y()), (a.y() - c.y()), (d.y()), 0.0,\
                                    (a.z() - e.z()), (a.z() - c.z()), (d.z()), 0.0,\
                                    0.0, 0.0, 0.0, 1.0).determinant() / divisor;
        if (beta < 0 || beta > (1 - rho)) continue; // Does not intersect

        if (distanceToSurface < range) {
            range = distanceToSurface;
            eval[0] = 4;  // Triangle has been intersected
            eval[1] = tIndex;
            eval[2] = (origin + distanceToSurface*ray).x();
            eval[3] = (origin + distanceToSurface*ray).y();
            eval[4] = (origin + distanceToSurface*ray).z();
        }
    }

    return eval;
}

/* QVector< double > GLWidget::shadePoint(QVector3D ray, QVector3D origin, QVector<double> intersectInfo)
 *
 * Formal Parameters: ray              - The ray to check intersections against
 *                    origin           - The coordinates of where 'ray' originates from
 *                    intersectInfo[0] - 3 if surface is a sphere
 *                                     - 4 if surface is a triangle
 *                    intersectInfo[1] - Index of object in respective storage
 *                    intersectInfo[2] - Surface Intersect X
 *                    intersectInfo[3] - Surface Intersect Y
 *                    intersectInfo[4] - Surface Intersect Z
 *
 * Returns:
 */
QVector< double > GLWidget::shadePoint(QVector3D ray, QVector3D origin, QVector<double> intersectInfo)
{
    /// Declarations and Initilializations
    QVector< double > eval(3);
    QVector3D surfaceIntersect, surfaceNormal, surfaceToLight, eyeVector, h;
    double ambi[3], diff[3], spec[3], specReflec;
    double shadingR, shadingG, shadingB, surfaceToLightDistance, intensityR, intensityG, intensityB;
    double fallOff, lightMagnitude, L1, L2;

    surfaceIntersect = QVector3D(intersectInfo[2], intersectInfo[3], intersectInfo[4]);

    /// Grab values from object
    if (intersectInfo[0] == 3) {
        Sphere sphere = spheres[intersectInfo[1]];
        ambi[0] = sphere.ambi[0];
        ambi[1] = sphere.ambi[1];
        ambi[2] = sphere.ambi[2];

        diff[0] = sphere.diff[0];
        diff[1] = sphere.diff[1];
        diff[2] = sphere.diff[2];

        spec[0] = sphere.spec[0];
        spec[1] = sphere.spec[1];
        spec[2] = sphere.spec[2];

        specReflec = sphere.specReflec;

        surfaceNormal = (surfaceIntersect - sphere.center).normalized();
    }
    else {
        Triangle triangle = triangles[intersectInfo[1]];
        ambi[0] = triangle.ambi[0];
        ambi[1] = triangle.ambi[1];
        ambi[2] = triangle.ambi[2];

        diff[0] = triangle.diff[0];
        diff[1] = triangle.diff[1];
        diff[2] = triangle.diff[2];

        spec[0] = triangle.spec[0];
        spec[1] = triangle.spec[1];
        spec[2] = triangle.spec[2];

        specReflec = triangle.specReflec;

        surfaceNormal = triangle.normal;
    }

    /// Ambient Shading
    shadingR = (ambi[0] * sceneAmbience);
    shadingG = (ambi[1] * sceneAmbience);
    shadingB = (ambi[2] * sceneAmbience);

    /// See which light sources are acting on this surface intersect
    for (int lIndex = 0; lIndex < pointLights.size(); lIndex++) {
        surfaceToLight = pointLights[lIndex].center - surfaceIntersect;
        surfaceToLightDistance = surfaceToLight.length();
        surfaceToLight.normalize();

        intensityR = pointLights[lIndex].intensity[0];
        intensityG = pointLights[lIndex].intensity[1];
        intensityB = pointLights[lIndex].intensity[2];
        fallOff = pow(surfaceToLightDistance/2, 1);

        /// Lambertian Shading
        lightMagnitude = QVector3D::dotProduct(surfaceNormal, surfaceToLight);
        L1 = max(0.0, lightMagnitude);

        /// Blinn-Phong Shading
        eyeVector = (origin - surfaceIntersect).normalized();
        h = (eyeVector + surfaceToLight).normalized();
        lightMagnitude = QVector3D::dotProduct(surfaceNormal, h);
        L2 = pow(max(0.0, lightMagnitude), specReflec);

        /// If light is acting on this point, check for intermediate objects (to create shadows)
        if ((L1 + L2) != 0) {
            double check = (intersects(surfaceToLight, surfaceIntersect, surfaceToLightDistance))[0];
            if ( check == 3 || check == 4) continue;  // Intersection
        }


        /// Apply Lambertian followed by Blinn-Phong
        shadingR += (diff[0] * (intensityR / fallOff) * L1);
        shadingG += (diff[1] * (intensityG / fallOff) * L1);
        shadingB += (diff[2] * (intensityB / fallOff) * L1);

        shadingR += (spec[0] * (intensityR / fallOff) * L2);
        shadingG += (spec[1] * (intensityG / fallOff) * L2);
        shadingB += (spec[2] * (intensityB / fallOff) * L2);

    }

    eval[0] = shadingR;
    eval[1] = shadingG;
    eval[2] = shadingB;

    return eval;
}

QVector< double > GLWidget::traceRay(QVector3D ray, QVector3D cameraPosition)
{
    /// Declarations and Initilizations
    QVector< double > eval = QVector< double >(4);
    QVector< double > intersectInfo, pointColour;

    eval[0] = 0;

    intersectInfo = intersects(ray, cameraPosition, INFINITY);
    /* Returns: eval[0] - 0 if intersects with nothing
    *                  - 1 if intersects with point light
    *                  - 2 if intersects with area light
    *                  - 3 if intersects with sphere
    *                  - 4 if intersects with triangle
    */
    if (intersectInfo[0] >= 3) {
        eval[0] = 1;
        pointColour = shadePoint(ray, cameraPosition, intersectInfo);
        eval[1] = pointColour[0];
        eval[2] = pointColour[1];
        eval[3] = pointColour[2];
    }
    else if (intersectInfo[0] == 1) {
        eval[0] = 1;
        eval[1] = pointLights[intersectInfo[1]].intensity[0];
        eval[2] = pointLights[intersectInfo[1]].intensity[1];
        eval[3] = pointLights[intersectInfo[1]].intensity[2];
    }
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
 *                    4 if intersects triangle
 *          eval[1] - Red colour value of pixel
 *          eval[2] - Green colour value of pixel
 *          eval[3] - Blue colour value of pixel
 */
QVector< double > GLWidget::traceRay2(QVector3D ray, QVector3D cameraPosition)
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

    QVector< double > triangleIntersect = triangleIntersection(ray, cameraPosition, closestObject);
    if (triangleIntersect[0] == 1) {
        eval[0] = 4;
        eval[1] = triangleIntersect[1];
        eval[2] = triangleIntersect[2];
        eval[3] = triangleIntersect[3];
        closestObject = triangleIntersect[4];
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
    double shadingR, shadingG, shadingB, lightMagnitude, L;

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
            d = sqrt(disc);
            distanceToSurface = (v - d);

            if (distanceToSurface < closestObject) {
                closestObject = (v - d);
                eval[0] = 1;  // Ray intersects at least one sphere

                surfaceIntersect = cameraPosition + distanceToSurface*ray;
                surfaceNormal = (surfaceIntersect - sphereCenter).normalized();

                /// Ambient Shading
                shadingR = (spheres[sIndex].ambi[0] * sceneAmbience);
                shadingG = (spheres[sIndex].ambi[1] * sceneAmbience);
                shadingB = (spheres[sIndex].ambi[2] * sceneAmbience);

                /// See which light sources are acting on this surface intersect
                for (int lIndex = 0; lIndex < pointLights.size(); lIndex++) {
                    sphereToLight = (pointLights[lIndex].center - surfaceIntersect).normalized();
                    sphereToLightDistance = (pointLights[lIndex].center - surfaceIntersect).length();

                    /// Is there any spheres in the way?  If so, shadow exists instead
                    bool hitObstacle = false;
                    for (int s2Index = 0; s2Index < spheres.size(); s2Index++) {
//                        if (s2Index == sIndex)
//                            continue;
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
                        if (disc2 > 0.2) {  // Does intersect a sphere
                            double d2 = sqrt(disc2);
                            QVector3D surfaceIntersect2 = surfaceIntersect + (v2 - d2)*sphereToLight;
                            double distanceToObstacle = (surfaceIntersect2 - surfaceIntersect).length();
                            if (distanceToObstacle < sphereToLightDistance) {
                                hitObstacle = true;
                                break;
                            }
                        }
                    }
                    if (hitObstacle == true)
                        continue;

                    /// Lambertian Shading
                    lightMagnitude = QVector3D::dotProduct(surfaceNormal, sphereToLight);
                    L = max(0.0, lightMagnitude);

                    shadingR += (spheres[sIndex].diff[0] * (pointLights[lIndex].intensity[0] / pow(sphereToLightDistance/2, 1)) * L);
                    shadingG += (spheres[sIndex].diff[1] * (pointLights[lIndex].intensity[1] / pow(sphereToLightDistance/2, 1)) * L);
                    shadingB += (spheres[sIndex].diff[2] * (pointLights[lIndex].intensity[2] / pow(sphereToLightDistance/2, 1)) * L);

                    /// Blinn-Phong Shading
                    eyeVector = (cameraPosition - surfaceIntersect).normalized();
                    h = (eyeVector + sphereToLight).normalized();
                    lightMagnitude = QVector3D::dotProduct(surfaceNormal, h);
                    L = pow(max(0.0, lightMagnitude), 100);

                    shadingR += (spheres[sIndex].spec[0] * (pointLights[lIndex].intensity[0] / pow(sphereToLightDistance/2, 1)) * L);
                    shadingG += (spheres[sIndex].spec[1] * (pointLights[lIndex].intensity[1] / pow(sphereToLightDistance/2, 1)) * L);
                    shadingB += (spheres[sIndex].spec[2] * (pointLights[lIndex].intensity[2] / pow(sphereToLightDistance/2, 1)) * L);

    //                /// Constrain dilution so that it doesn't dim the scene too much
    //                if (shadingR > 1.0 || shadingG > 1.0 || shadingB > 1.0) {
    //                    double maxDilution = max(shadingR, max(shadingG, shadingB));
    //                    shadingR = shadingR / maxDilution;
    //                    shadingG = shadingG / maxDilution;
    //                    shadingB = shadingB / maxDilution;
    //                }
                }
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
    double cc, v, disc1, disc2, d, distanceToSurface, fogLevel, normalToCamera;

    eval[0] = 0;  // We have not intersected anything

    /// Check ray against every light source in world space
    for (int lIndex = 0; lIndex < pointLights.size(); lIndex++) {

        // The light circle to be traced
        lightSphereCenter = pointLights[lIndex].center;

        // Vector from the cameraPoint to the lightSphereCenter
        cPcCVector = (lightSphereCenter - cameraPosition);
        // Magnitude of cPcCVector, squared
        cc = QVector3D::dotProduct(cPcCVector, cPcCVector);

        // Magnitude of ray from the cameraPoint to when it is
        // perpendicular to the normal of the lightSphereCenter
        v = QVector3D::dotProduct(cPcCVector, ray);

        // Difference between the lightCircleRadius and distance
        // from the lightSphereCenter to ray when they are perpendicular
        disc1 = ((0.0625) - (cc - v*v));
        disc2 = (pow((0.25 + lightFog), 2) - (cc - v*v));

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
                fogLevel = pow(( d / (0.25 + lightFog)), 4);
                eval[1] = (255.0 * pointLights[lIndex].intensity[0] * fogLevel);
                eval[2] = (255.0 * pointLights[lIndex].intensity[1] * fogLevel);
                eval[3] = (255.0 * pointLights[lIndex].intensity[2] * fogLevel);
            }
        }
        else {  // ray intersects the light sphere
            d = sqrt(disc1);
            surfaceIntersect = cameraPosition + (v - d)*ray;
            distanceToSurface = (surfaceIntersect - cameraPosition).length();

            if (distanceToSurface < closestObject) {
                closestObject = distanceToSurface;
                eval[0] = 2;

                surfaceNormal = (surfaceIntersect - pointLights[lIndex].center).normalized();
                lightToCamera = (cameraPosition - pointLights[lIndex].center).normalized();
                normalToCamera = QVector3D::dotProduct(surfaceNormal, lightToCamera);

                /// 255 is max; Each sphere has individual intensity ; falloff function for edges of light sources
                eval[1] = (255.0 * pointLights[lIndex].intensity[0]);
                eval[2] = (255.0 * pointLights[lIndex].intensity[1]);
                eval[3] = (255.0 * pointLights[lIndex].intensity[2]);

            }
        }
    }
    eval[4] = closestObject;
    return eval;
}

/* QVector< double > GLWidget::triangleIntersection(QVector3D ray, QVector3D cameraPosition, double closestObject)
 *
 * Formal Parameters: ray - The ray to be traced in the scene
 *                    cameraPosition - The coordinates of the camera
 *                    closestObject - The distance from the camera to the closest object (so far)
 * Returns: eval[0] - 0 if intersects with nothing
 *                  - 1 if intersects with plane
 *          eval[1] - Red colour value of pixel
 *          eval[2] - Green colour value of pixel
 *          eval[3] - Blue colour value of pixel
 *          eval[4] - Distance to closest object
 */
QVector< double > GLWidget::triangleIntersection(QVector3D ray, QVector3D cameraPosition, double closestObject)
{
    /// Declarations and Initilizations
    QVector< double > eval(5);

    eval[0] = 0;  // Nothing has been intersected yet

    for (int tIndex = 0; tIndex < triangles.size(); tIndex++) {
        QVector3D a = triangles[tIndex].a;
        QVector3D b = triangles[tIndex].b;
        QVector3D c = triangles[tIndex].c;
        QVector3D d = ray;
        QVector3D e = cameraPosition;

        double divisor = QMatrix4x4((a.x() - b.x()), (a.x() - c.x()), (d.x()), 0.0,\
                                    (a.y() - b.y()), (a.y() - c.y()), (d.y()), 0.0,\
                                    (a.z() - b.z()), (a.z() - c.z()), (d.z()), 0.0,\
                                    0.0, 0.0, 0.0, 1.0).determinant();
        double rho     = QMatrix4x4((a.x() - b.x()), (a.x() - e.x()), (d.x()), 0.0,\
                                    (a.y() - b.y()), (a.y() - e.y()), (d.y()), 0.0,\
                                    (a.z() - b.z()), (a.z() - e.z()), (d.z()), 0.0,\
                                    0.0, 0.0, 0.0, 1.0).determinant() / divisor;
        if (rho < 0 || rho > 1) { continue; } // Does not intersect
        double beta    = QMatrix4x4((a.x() - e.x()), (a.x() - c.x()), (d.x()), 0.0,\
                                    (a.y() - e.y()), (a.y() - c.y()), (d.y()), 0.0,\
                                    (a.z() - e.z()), (a.z() - c.z()), (d.z()), 0.0,\
                                    0.0, 0.0, 0.0, 1.0).determinant() / divisor;
        if (beta < 0 || beta > (1 - rho)) { continue; } // Does not intersect
        double t       = QMatrix4x4((a.x() - b.x()), (a.x() - c.x()), (a.x() - e.x()), 0.0,\
                                    (a.y() - b.y()), (a.y() - c.y()), (a.y() - e.y()), 0.0,\
                                    (a.z() - b.z()), (a.z() - c.z()), (a.z() - e.z()), 0.0,\
                                    0.0, 0.0, 0.0, 1.0).determinant() / divisor;

        QVector3D surfaceIntersect = cameraPosition + t*ray;
        //QVector3D cameraToSurface = (surfaceIntersect - cameraPoint).normalized();
        double cameraToSurfaceDistance = (surfaceIntersect - cameraPosition).length();
        if (cameraToSurfaceDistance < closestObject) {
            eval[0] = 1;  // Plane has been intersected
            closestObject = cameraToSurfaceDistance;
            QVector3D surfaceNormal = QVector3D::crossProduct((c - a), (b - a)).normalized();

            /// Ambient Shading
            double shadingR = (triangles[tIndex].ambi[0] * sceneAmbience);
            double shadingG = (triangles[tIndex].ambi[1] * sceneAmbience);
            double shadingB = (triangles[tIndex].ambi[2] * sceneAmbience);

            /// See which light sources are acting on this surface intersect
            for (int lIndex = 0; lIndex < pointLights.size(); lIndex++) {
                QVector3D planeToLight = (pointLights[lIndex].center - surfaceIntersect).normalized();
                double planeToLightDistance = (pointLights[lIndex].center - surfaceIntersect).length();

                /// Lambertian Shading
                double lightToNormal = QVector3D::dotProduct(surfaceNormal, planeToLight);
                double L = max(0.0, lightToNormal);

                shadingR += (triangles[tIndex].diff[0] * (pointLights[lIndex].intensity[0] / pow(planeToLightDistance/2, 1)) * L);
                shadingG += (triangles[tIndex].diff[1] * (pointLights[lIndex].intensity[1] / pow(planeToLightDistance/2, 1)) * L);
                shadingB += (triangles[tIndex].diff[2] * (pointLights[lIndex].intensity[2] / pow(planeToLightDistance/2, 1)) * L);

                /// Blinn-Phong Shading
                QVector3D eyeVector = (cameraPosition - surfaceIntersect).normalized();
                QVector3D h = (eyeVector + planeToLight).normalized();
                double hToNormal = QVector3D::dotProduct(surfaceNormal, h);
                L = pow(max(0.0, hToNormal), 1000);

                shadingR += (triangles[tIndex].spec[0] * (pointLights[lIndex].intensity[0] / pow(planeToLightDistance/2, 1)) * L);
                shadingG += (triangles[tIndex].spec[1] * (pointLights[lIndex].intensity[1] / pow(planeToLightDistance/2, 1)) * L);
                shadingB += (triangles[tIndex].spec[2] * (pointLights[lIndex].intensity[2] / pow(planeToLightDistance/2, 1)) * L);

            }
            eval[1] = shadingR * 255;
            eval[2] = shadingG * 255;
            eval[3] = shadingB * 255;
        }
    }
    eval[4] = closestObject;
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

