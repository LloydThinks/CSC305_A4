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

    /// Scene Settings
    cameraToPicturePlaneDistance = 10.0;
    picturePlaneZ = 12.0;
    antiAliasing = true;

    /// Light Settings
    sceneAmbience = 0.4;  // Overall Ambience in the scene
    lightPersistence = 5.0;  // Increase this number to allow light to travel further
    unitSegs = 5;  // How dense area lights are

    /// Colours: [0] - Red colour value
    ///          [1] - Green colour value
    ///          [2] - Blue colour value
    QVector< double > white(3); white[0] = 1.0; white[1] = 1.0; white[2] = 1.0;
    QVector< double > red(3); red[0] = 0.6196; red[1] = 0.0824; red[2] = 0.1353;
    QVector< double > blue(3); blue[0] = 0.1215; blue[1] = 0.1804; blue[2] = 0.5686;

    /// Materials: [0] - Ambient Coefficient
    ///            [1] - Diffuse Coefficient
    ///            [2] - Specular Coefficient
    ///            [3] - Reflection Coefficient
    QVector< QVector< double > > mirror(4, QVector< double >(3, 0.0));
        mirror[3][0] = 1.0; mirror[3][1] = 1.0; mirror[3][2] = 1.0;
    QVector< QVector< double > > eggShell(4, QVector< double >(3, 0.0));
        eggShell[0][0] = 0.2; eggShell[0][1] = 0.2; eggShell[0][2] = 0.2;
        eggShell[1][0] = 0.7; eggShell[1][1] = 0.7; eggShell[1][2] = 0.7;
        eggShell[2][0] = 0.8; eggShell[2][1] = 0.8; eggShell[2][2] = 0.8;
        eggShell[3][0] = 0.2; eggShell[3][1] = 0.2; eggShell[3][2] = 0.2;
    QVector< QVector< double > > wall(4, QVector< double >(3, 0.0));
        wall[0][0] = 0.2; wall[0][1] = 0.2; wall[0][2] = 0.2;
        wall[1][0] = 0.8; wall[1][1] = 0.8; wall[1][2] = 0.8;
        wall[2][0] = 0.3; wall[2][1] = 0.3; wall[2][2] = 0.3;

    /// Spheres
    spheres = QVector< Sphere >();
    spheres.append(Sphere(QVector3D(2.5, 1.6, 4.0), 1.5, 100, mirror[0], mirror[1], mirror[2], mirror[3], white));
    spheres.append(Sphere(QVector3D(8.0, 2.0, 7.0), 1.5, 10, eggShell[0], eggShell[1], eggShell[2], eggShell[3], white));

    /// Light Sources
    // Initialize Point Lights
    pointLights = QVector< PointLight >();
    //pointLights.append(PointLight(QVector3D(5.0, 10.0, 5.0), white));
    // Initialize Area Lights
    areaLights = QVector< AreaLight >();
    // Square Ceiling Light
    areaLights.append(AreaLight(QVector3D(4.0, 10.0, 6.0), QVector3D(6.0, 10.0, 6.0),
                                QVector3D(6.0, 10.0, 4.0), QVector3D(4.0, 10.0, 4.0), white));
    // Rectangle Ceiling Light
//    areaLights.append(AreaLight(QVector3D(2.0, 10.0, 6.0), QVector3D(8.0, 10.0, 6.0),
//                                    QVector3D(8.0, 10.0, 4.0), QVector3D(2.0, 10.0, 4.0), white));

    /// Boxes
    // PUT CODE HERE

    /// Square Room Triangles
    // Draw Room - Left Wall
    triangles.append(Triangle(QVector3D(0, 0, 10), QVector3D(0, 10, 10), QVector3D(0, 10, 0), 100, wall[0], wall[1], wall[2], red));
    triangles.append(Triangle(QVector3D(0, 0, 10), QVector3D(0, 10, 0), QVector3D(0, 0, 0), 100, wall[0], wall[1], wall[2], red));

    //Draw Room - Back Wall
    triangles.append(Triangle(QVector3D(0, 0, 0), QVector3D(0, 10, 0), QVector3D(10, 10, 0), 100, wall[0], wall[1], wall[2], white));
    triangles.append(Triangle(QVector3D(0, 0, 0), QVector3D(10, 10, 0), QVector3D(10, 0, 0), 100, wall[0], wall[1], wall[2], white));

    // Draw Room - Right Wall
    triangles.append(Triangle(QVector3D(10, 0, 0), QVector3D(10, 10, 0), QVector3D(10, 10, 10), 100, wall[0], wall[1], wall[2], blue));
    triangles.append(Triangle(QVector3D(10, 0, 0), QVector3D(10, 10, 10), QVector3D(10, 0, 10), 100, wall[0], wall[1], wall[2], blue));

    // Draw Room - Floor
    triangles.append(Triangle(QVector3D(0, 0, 10), QVector3D(0, 0, 0), QVector3D(10, 0, 0), 100, wall[0], wall[1], wall[2], white));
    triangles.append(Triangle(QVector3D(0, 0, 10), QVector3D(10, 0, 0), QVector3D(10, 0, 10), 100, wall[0], wall[1], wall[2], white));

    /// Rectangle Room Triangles
    /*
    // Draw Room - Left Wall
    triangles.append(Triangle(QVector3D(0, 0, 10), QVector3D(0, 10, 10), QVector3D(0, 10, -10), 100, wall[0], wall[1], wall[2], red));
    triangles.append(Triangle(QVector3D(0, 0, 10), QVector3D(0, 10, -10), QVector3D(0, 0, -10), 100, wall[0], wall[1], wall[2], red));

    //Draw Room - Back Wall
    triangles.append(Triangle(QVector3D(0, 0, -10), QVector3D(0, 10, -10), QVector3D(10, 10, -10), 100, wall[0], wall[1], wall[2], white));
    triangles.append(Triangle(QVector3D(0, 0, -10), QVector3D(10, 10, -10), QVector3D(10, 0, -10), 100, wall[0], wall[1], wall[2], white));

    // Draw Room - Right Wall
    triangles.append(Triangle(QVector3D(10, 0, -10), QVector3D(10, 10, -10), QVector3D(10, 10, 10), 100, wall[0], wall[1], wall[2], blue));
    triangles.append(Triangle(QVector3D(10, 0, -10), QVector3D(10, 10, 10), QVector3D(10, 0, 10), 100, wall[0], wall[1], wall[2], blue));

    // Draw Room - Floor
    triangles.append(Triangle(QVector3D(0, 0, 10), QVector3D(0, 0, -10), QVector3D(10, 0, -10), 100, wall[0], wall[1], wall[2], white));
    triangles.append(Triangle(QVector3D(0, 0, 10), QVector3D(10, 0, -10), QVector3D(10, 0, 10), 100, wall[0], wall[1], wall[2], white));
    */
    preCalculate();
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
    double widthScale, heightScale, shadingR, shadingG, shadingB, random;

    /// Timer
    clock_t t1,t2;
    t1 = clock();

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
        qDebug() << "Progress:" << double(i)/renderWidth*100 << "%";
        for (int j = 0; j < renderHeight; j++) {
            shadingR = 0.0;
            shadingG = 0.0;
            shadingB = 0.0;

            if (antiAliasing) {
                for (int k = 0; k < 3; k++) {
                    for (int l = 0; l < 3; l++) {
                        random = (rand() / double(RAND_MAX));

                        // The current pixel we are trying to draw
                        pixelPosition = QVector3D((((double(i) + ((rand() / double(RAND_MAX))/3.0) * double(k)) * widthScale) / renderWidth),
                                                  (((double(j) + ((rand() / double(RAND_MAX))/3.0) * double(l)) * heightScale) / renderHeight), picturePlaneZ);

                        // Ray to be traced through the scene
                        ray = (pixelPosition - cameraPosition).normalized();

                        // Trace the ray and return important colour attributes
                        rayTrace = traceRay(ray, cameraPosition, 2);
                        shadingR += (rayTrace[1]/9.0);
                        shadingG += (rayTrace[2]/9.0);
                        shadingB += (rayTrace[3]/9.0);
                    }
                }
                myimage.setPixel(i, (renderHeight - 1 - j), qRgb(shadingR*255.0, shadingG*255.0, shadingB*255.0));
            }
            else {
                // The current pixel we are trying to draw
                pixelPosition = QVector3D(((double(i) * widthScale) / renderWidth),
                                          ((double(j) * heightScale) / renderHeight), picturePlaneZ);

                // Ray to be traced through the scene
                ray = (pixelPosition - cameraPosition).normalized();

                // Trace the ray and return important colour attributes
                rayTrace = traceRay(ray, cameraPosition, 2);

                myimage.setPixel(i, (renderHeight - 1 - j), qRgb(rayTrace[1]*255.0, rayTrace[2]*255.0, rayTrace[3]*255.0));
            }
        }
    }

    t2 = clock();
    double runtimeSecs  = ((double)t2 - (double)t1) / CLOCKS_PER_SEC;
    double minutes = floor((runtimeSecs / 60));
    double seconds = runtimeSecs - (minutes * 60);
    qDebug() << "Program Execution Time:" << minutes << "Minutes;" << seconds << "Seconds";

    qtimage = myimage.copy(0, 0,  myimage.width(), myimage.height());
    prepareImageDisplay(&myimage);
}

void GLWidget::preCalculate() {
    /// Declarations and Intializations
    QVector3D hVector, vVector, hStep, vStep;
    double hVectorLength, vVectorLength;
    int numHSteps, numVSteps;

    /// Area Light data
    for (int lIndex = 0; lIndex < areaLights.size(); lIndex++) {
        // Horizontal and Vertical Vectors that run the width and height of the area light
        hVector = areaLights[lIndex].b - areaLights[lIndex].a;
        vVector = areaLights[lIndex].d - areaLights[lIndex].a;
        hVectorLength = hVector.length();
        vVectorLength = vVector.length();

        // Number of steps per pass over the width, and height
        numHSteps = (hVectorLength * unitSegs);
        numVSteps = (vVectorLength * unitSegs);

        // Distance to increment the iterating point when calculating area light intensities
        hStep = hVector / double(numHSteps);
        vStep = vVector / double(numVSteps);

        // Store this information into the area light for use later
        areaLights[lIndex].numHSteps = numHSteps;
        areaLights[lIndex].numVSteps = numVSteps;
        areaLights[lIndex].hStep = hStep;
        areaLights[lIndex].vStep = vStep;
        areaLights[lIndex].pIntensity[0] = (areaLights[lIndex].intensity[0] / ((numHSteps+1)*(numVSteps+1)));
        areaLights[lIndex].pIntensity[1] = (areaLights[lIndex].intensity[1] / ((numHSteps+1)*(numVSteps+1)));
        areaLights[lIndex].pIntensity[2] = (areaLights[lIndex].intensity[2] / ((numHSteps+1)*(numVSteps+1)));
    }
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
 * Returns: eval[0] - Red Colour value of the surface at intersectInfo
 *          eval[0] - Green Colour value of the surface at intersectInfo
 *          eval[0] - Blue Colour value of the surface at intersectInfo
 */
QVector< double > GLWidget::shadePoint(QVector3D ray, QVector3D origin, QVector<double> intersectInfo, int recursiveDepth)
{
    /// Declarations and Initilializations
    QVector< double > eval(3), result;
    QVector3D surfaceIntersect, surfaceNormal, surfaceToLight, eyeVector, h, horizontalAlign, currentPoint, hStep, vStep;
    double ambi[3], diff[3], spec[3], reflec[3], specReflec;
    double shadingR, shadingG, shadingB, surfaceToLightDistance, intensityR, intensityG, intensityB;
    double fallOff, lightMagnitude, L1, L2, numHSteps, numVSteps;

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
        fallOff = surfaceToLightDistance/lightPersistence;

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
        intensityR = areaLights[lIndex].pIntensity[0];
        intensityG = areaLights[lIndex].pIntensity[1];
        intensityB = areaLights[lIndex].pIntensity[2];

        numVSteps = areaLights[lIndex].numVSteps;
        numHSteps = areaLights[lIndex].numHSteps;
        hStep = areaLights[lIndex].hStep;
        vStep = areaLights[lIndex].vStep;

        horizontalAlign = areaLights[lIndex].a;
        currentPoint = horizontalAlign;

        for (int i = 0; i <= numVSteps; i++) {
            for (int j = 0; j <= numHSteps; j++) {
                surfaceToLight = currentPoint - surfaceIntersect;
                surfaceToLightDistance = surfaceToLight.length();
                surfaceToLight.normalize();

                fallOff = surfaceToLightDistance/lightPersistence;

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

/* QVector< double > GLWidget::traceRay(QVector3D ray, QVector3D origin, int recursiveDepth)
 *
 * Formal Parameters: ray              - The ray to check intersections against
 *                    origin           - The coordinates of where 'ray' originates from
 *                    intersectInfo[0] - 3 if surface is a sphere
 *                                     - 4 if surface is a triangle
 *                    intersectInfo[1] - Index of object in respective storage
 *                    intersectInfo[2] - Surface Intersect X
 *                    intersectInfo[3] - Surface Intersect Y
 *                    intersectInfo[4] - Surface Intersect Z
 * Returns: eval[0] - 0 if nothing was intersected
 *                  - 1 if something was intersected
 *          eval[1] - Red Colour value of the surface intersected
 *          eval[2] - Green Colour value of the surface intersected
 *          eval[3] - Blue Colour value of the surface intersected
 */
QVector< double > GLWidget::traceRay(QVector3D ray, QVector3D origin, int recursiveDepth)
{
    /// Declarations and Initilizations
    QVector< double > eval = QVector< double >(4);
    QVector< double > intersectInfo, pointColour;

    eval[0] = 0.0;
    eval[1] = 0.0;
    eval[2] = 0.0;
    eval[3] = 0.0;
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

