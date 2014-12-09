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

    cameraToPicturePlaneDistance = 15.0;
    picturePlaneZ = 12.0;

    /// Light Settings
    sceneAmbience = 0.2;
    lightFallOff = 2.0;
    lightFog = 1.0;

    /// Spheres
    spheres = QVector< Sphere >();
    // Initialize Spheres
    double ambi1[3] = {0.0, 0.0, 0.0};
    double diff1[3] = {0.0, 0.0, 0.0};
    double spec1[3] = {0.0, 0.0, 0.0};
    double reflective1[3] = {1.0, 1.0, 1.0};
    spheres.append(Sphere(QVector3D(3.0, 3.0, 4.0), 2.0, 100, ambi1, diff1, spec1, reflective1));
    //double reflective2[3] = {0.2, 0.2, 0.2};
    //spheres.append(Sphere(QVector3D(5.0, 7.0, 5.0), 1.0, 100, ambi, diff, spec));
    //spheres.append(Sphere(QVector3D(5.0, 3.5, 5.0), 0.5, 100, ambi, diff, spec));

    /// Light Sources
    // Initialize Point Lights
    double white[3] = {1.0, 1.0, 1.0};
    double red[3] = {1.0, 0.6, 0.6};
    double yellow[3] = {0.6, 0.6, 0.42};
    pointLights = QVector< PointLight >();
    //pointLights.append(PointLight(QVector3D(7.0, 3.0, 8.0), white));
    //pointLights.append(PointLight(QVector3D(8.0, 8.0, 6.0), white));
    // Initialize Area Lights
    areaLights = QVector< AreaLight >();
    areaLights.append(AreaLight(QVector3D(4.0, 10.0, 6.0), QVector3D(6.0, 10.0, 6.0), QVector3D(6.0, 10.0, 4.0), QVector3D(4.0, 10.0, 4.0), white));

    /// Boxes
    double bambi[3] = {0.6, 0.6, 0.6};
    double bdiff[3] = {0.7, 0.7, 0.7};
    double bspec[3] = {0.6, 0.6, 0.6};
    triangles = QVector< Triangle >();
    //triangles.append(Triangle(QVector3D(0.0, 0.0, 10.0), QVector3D(0.0, 10.0, 10.0), QVector3D(0.0, 10.0, 0.0), 100, bambi, bdiff, bspec));

    /// Room Triangles
    double wambi[3] = {0.5, 0.5, 0.5};
    double wdiff[3] = {0.7, 0.7, 0.7};
    double wspec[3] = {0.3, 0.3, 0.3};

    // Draw Room - Left Wall
    triangles.append(Triangle(QVector3D(0.0, 0.0, 10.0), QVector3D(0.0, 10.0, 10.0), QVector3D(0.0, 10.0, 0.0), 100, wambi, wdiff, wspec));
    triangles.append(Triangle(QVector3D(0.0, 0.0, 10.0), QVector3D(0.0, 10.0, 0.0), QVector3D(0.0, 0.0, 0.0), 100, wambi, wdiff, wspec));

    double wdiff1[3] = {0.08235, 0.6196, 0.564};
    double wspec1[3] = {0.08235, 0.6196, 0.564};
    //Draw Room - Back Wall
    triangles.append(Triangle(QVector3D(0.0, 0.0, 0.0), QVector3D(0.0, 10.0, 0.0), QVector3D(10.0, 10.0, 0.0), 100, wambi, wdiff1, wspec1));
    triangles.append(Triangle(QVector3D(0.0, 0.0, 0.0), QVector3D(10.0, 10.0, 0.0), QVector3D(10.0, 0.0, 0.0), 100, wambi, wdiff1, wspec1));

    double wdiff2[3] = {0.6196, 0.08235, 0.135};
    double wspec2[3] = {0.6196, 0.08235, 0.135};
    // Draw Room - Right Wall
    triangles.append(Triangle(QVector3D(10.0, 0.0, 0.0), QVector3D(10.0, 10.0, 0.0), QVector3D(10.0, 10.0, 10.0), 100, wambi, wdiff2, wspec2));
    triangles.append(Triangle(QVector3D(10.0, 0.0, 0.0), QVector3D(10.0, 10.0, 10.0), QVector3D(10.0, 0.0, 10.0), 100, wambi, wdiff2, wspec2));

    // Draw Room - Floor
    triangles.append(Triangle(QVector3D(0.0, 0.0, 10.0), QVector3D(0.0, 0.0, 0.0), QVector3D(10.0, 0.0, 0.0), 100, wambi, wdiff, wspec));
    triangles.append(Triangle(QVector3D(0.0, 0.0, 10.0), QVector3D(10.0, 0.0, 0.0), QVector3D(10.0, 0.0, 10.0), 100, wambi, wdiff, wspec));
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
    cameraPosition = QVector3D(widthScale/2.0, heightScale/2.0, picturePlaneZ + cameraToPicturePlaneDistance);

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
            pixelPosition = QVector3D(((double(i) * widthScale) / renderWidth),
                                      ((double(j) * heightScale) / renderHeight), picturePlaneZ);

            if (i == renderWidth/2 && j == renderHeight/2)
                qDebug() << "PlaneCenter: " << pixelPosition;

            // Ray to be traced through the scene
            ray = (pixelPosition - cameraPosition).normalized();
            if (i == renderWidth/2 && j == renderHeight/2)
                qDebug() << "Ray at Center: " << ray;

            // Trace the ray and return important colour attributes
            rayTrace = traceRay(ray, cameraPosition, 2);
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
    QVector3D center, EO, a, b, c, d, e;
    double radius, cc, v, disc, distanceToSurface, divisor, rho, beta;

    eval[0] = 0;

    /// Check ray against every point light source in world space
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

            if (distanceToSurface > 0.01 && distanceToSurface < range) {
                range = distanceToSurface;
                eval[0] = 1;  // Ray intersects the point light
                eval[1] = lIndex;
                eval[2] = (origin + distanceToSurface*ray).x();
                eval[3] = (origin + distanceToSurface*ray).y();
                eval[4] = (origin + distanceToSurface*ray).z();
            }
        }
    }

    /// Check ray against every area light source in the world space
    for (int lIndex = 0; lIndex < areaLights.size(); lIndex++) {
        d = ray;
        e = origin;
        for (int lightHalf = 0; lightHalf < 2; lightHalf++) {
            if (lightHalf == 0) {
                a = areaLights[lIndex].a;
                b = areaLights[lIndex].b;
                c = areaLights[lIndex].c;
            }
            else {
                a = areaLights[lIndex].a;
                b = areaLights[lIndex].c;
                c = areaLights[lIndex].d;
            }

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

            range = distanceToSurface;
            eval[0] = 2;  // Area light has been intersected
            eval[1] = lIndex;
            eval[2] = (origin + distanceToSurface*ray).x();
            eval[3] = (origin + distanceToSurface*ray).y();
            eval[4] = (origin + distanceToSurface*ray).z();
        }
    }

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

        range = distanceToSurface;
        eval[0] = 4;  // Triangle has been intersected
        eval[1] = tIndex;
        eval[2] = (origin + distanceToSurface*ray).x();
        eval[3] = (origin + distanceToSurface*ray).y();
        eval[4] = (origin + distanceToSurface*ray).z();
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
QVector< double > GLWidget::shadePoint(QVector3D ray, QVector3D origin, QVector<double> intersectInfo, int recursiveDepth)
{
    /// Declarations and Initilializations
    QVector< double > eval(3), result;
    QVector3D surfaceIntersect, surfaceNormal, surfaceToLight, eyeVector, h;
    double ambi[3], diff[3], spec[3], reflec[3], specReflec;
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

        reflec[0] = sphere.reflec[0];
        reflec[1] = sphere.reflec[1];
        reflec[2] = sphere.reflec[2];

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
    // Point Lights
    for (int lIndex = 0; lIndex < pointLights.size(); lIndex++) {
        surfaceToLight = pointLights[lIndex].center - surfaceIntersect;
        surfaceToLightDistance = surfaceToLight.length();
        surfaceToLight.normalize();

        intensityR = pointLights[lIndex].intensity[0];
        intensityG = pointLights[lIndex].intensity[1];
        intensityB = pointLights[lIndex].intensity[2];
        fallOff = pow(surfaceToLightDistance/lightFallOff, 1);

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
    // Area Lights
    for (int lIndex = 0; lIndex < areaLights.size(); lIndex++) {
        // MOVE A LOT OF THIS INTO CONTSTRUCTOR / PRE-CALCULATIONS FUNCTION
        double unitSegs = 5;  // Every 1 unit of area light space gets unitSegs*(unitSegs+1) segments
        QVector3D hVector = areaLights[lIndex].b - areaLights[lIndex].a;
        QVector3D vVector = areaLights[lIndex].d - areaLights[lIndex].a;
        double numHSteps = (hVector.length() * unitSegs) + 1;
        double numVSteps = (vVector.length() * unitSegs) + 1;

        QVector3D hStep = hVector.normalized() / unitSegs;
        QVector3D vStep = vVector.normalized() / unitSegs;

        // Code needed here
        intensityR = areaLights[lIndex].intensity[0] / (numHSteps*numVSteps);
        intensityG = areaLights[lIndex].intensity[1] / (numHSteps*numVSteps);
        intensityB = areaLights[lIndex].intensity[2] / (numHSteps*numVSteps);

        QVector3D horizontalAlign = areaLights[lIndex].a;
        QVector3D currentPoint = horizontalAlign;

        for (int i = 0; i <= numVSteps; i++) {
            for (int j = 0; j <= numHSteps; j++) {

                surfaceToLight = currentPoint - surfaceIntersect;
                surfaceToLightDistance = surfaceToLight.length();
                surfaceToLight.normalize();

                fallOff = surfaceToLightDistance/lightFallOff;

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


                currentPoint += hStep;  // Move currentPoint across the area light
            }
            horizontalAlign += vStep;  // Move horizontalAlign down the area light
            currentPoint = horizontalAlign;  // Put the currentPoint back to horizontal edge
        }
    }

    // Relflect Ray if a sphere
    if (intersectInfo[0] == 3) {
        QVector3D bounce = (ray - 2*(QVector3D::dotProduct(ray, surfaceNormal)*surfaceNormal)).normalized();
        result = traceRay(bounce, surfaceIntersect, recursiveDepth-1);
        if (result[0] != 0) {  // The ray hit something
            shadingR += (reflec[0] * result[1]);
            shadingG += (reflec[1] * result[2]);
            shadingB += (reflec[2] * result[3]);
        }
    }

    /// Constrain dilution so that it doesn't dim the scene too much
    if (shadingR > 1.0 || shadingG > 1.0 || shadingB > 1.0) {
        double maxDilution = max(shadingR, max(shadingG, shadingB));
        shadingR = shadingR / maxDilution;
        shadingG = shadingG / maxDilution;
        shadingB = shadingB / maxDilution;
    }
    eval[0] = shadingR;
    eval[1] = shadingG;
    eval[2] = shadingB;

    return eval;
}

QVector< double > GLWidget::traceRay(QVector3D ray, QVector3D origin, int recursiveDepth)
{
    /// Declarations and Initilizations
    QVector< double > eval = QVector< double >(4);
    QVector< double > intersectInfo, pointColour;

    eval[0] = 0;
    if (recursiveDepth == 0)  // We have recursed enough
        return eval;

    intersectInfo = intersects(ray, origin, INFINITY);

    if (intersectInfo[0] >= 3) {
        eval[0] = 1;
        pointColour = shadePoint(ray, origin, intersectInfo, recursiveDepth);
        eval[1] = pointColour[0];
        eval[2] = pointColour[1];
        eval[3] = pointColour[2];
    }
    else if (intersectInfo[0] == 2) {  // Area Light intersected
        eval[0] = 1;
        eval[1] = areaLights[intersectInfo[1]].intensity[0];
        eval[2] = areaLights[intersectInfo[1]].intensity[1];
        eval[3] = areaLights[intersectInfo[1]].intensity[2];
    }
    else if (intersectInfo[0] == 1) {
        eval[0] = 1;
        eval[1] = pointLights[intersectInfo[1]].intensity[0];
        eval[2] = pointLights[intersectInfo[1]].intensity[1];
        eval[3] = pointLights[intersectInfo[1]].intensity[2];
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

