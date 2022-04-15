#include "gldisplaywidget.h"
#ifdef __APPLE__
    #include <glu.h>
#else
    #include <GL/glu.h>
#endif

#include "QDebug"

GLDisplayWidget::GLDisplayWidget(QWidget *parent) : QGLWidget(parent), _X(0.f), _Y(0.f),_Z(0.f), _angle(0.f)
  //UpgradeQt6: replace by : QOpenGLWidget(parent)
{
    // Update the scene
    connect( &_timer, SIGNAL(timeout()), this, SLOT(updateGL())); //UpgradeQt6: connect( &_timer, SIGNAL(timeout()), this, SLOT(update()));
    _timer.start(16); // Starts or restarts the timer with a timeout interval of 16 milliseconds.
    wireFrame = false;
    laplacian = false;
}

void GLDisplayWidget::initializeGL()
{
    // background color
    glClearColor(0.2, 0.2, 0.2, 1);

    // Shader
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);

    //** TP : To add....
    // Construction of the GeometricWorld before it is displayed
    // It can also be constructed following a signal (button)
    // _mesh.buildTetrahedron(0.5,0.6,0.8);
    // _mesh.buildPyramid(0.5,0.6,0.8);
    // _mesh.buildInput("../meshes/queen.off", 5,5,5);
    // _mesh.buildInput("../meshes/cube.off", 0.5,0.6,0.8);
    // _mesh.buildInput("../meshes/tetrahedre.off", 0.5,0.6,0.8);
    // _mesh.buildInput("../meshes/plan.off", 1,1,1);
    _mesh.buildInput("../meshes/equilateral.off", 0.7,0.7,0.7);


    // TEST ITERATOR
    // Circulator_on_vertices civ = Circulator_on_vertices(&_mesh, 2);
    // qDebug() << civ.vertex_indice << " " << civ.face_indice;
    // ++civ;
    // qDebug() << civ.vertex_indice << " " << civ.face_indice;
    // ++civ;
    // qDebug() << civ.vertex_indice << " " << civ.face_indice;
    // ++civ;
    // qDebug() << civ.vertex_indice << " " << civ.face_indice;

    Point middle = Point(0,1,0);
    _mesh.triangleSplit(middle,1);

    // _mesh.computeLaplacians();
    // qDebug() << _mesh.laplacian;
}

void GLDisplayWidget::paintGL(){

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Center the camera
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0,0,5,  0,0,0,   0,1,0); //gluLookAt(eye, center, up)  //deprecated
                                       // Returns a 4x4 matrix that transforms world coordinates to eye coordinates.
    // Translation
    glTranslated(_X, _Y, _Z);

    // Rotation
    glRotatef(_angle, 1.0f, 1.0f, 0.0f);

    // Color for your _geomWorld
    glColor3f(0, 1 ,0);

    // example with a tetraedre
    //_geomWorld.drawWireFrame();
    //_geomWorld.draw();

    // With the mesh
    if (wireFrame) {
        _mesh.drawMeshWireFrame(); // Draw vertex
    } else {
        _mesh.drawMesh(laplacian); // Full face
    };
}

void GLDisplayWidget::resizeGL(int width, int height){

    glViewport(0, 0, width, height); //Viewport in the world window
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0f, (GLfloat)width/(GLfloat)height, 0.1f, 100.0f);

    updateGL(); //UpgradeQt6: update();
}

// - - - - - - - - - - - - Mouse Management  - - - - - - - - - - - - - - - -
// When you click, the position of your mouse is saved
void GLDisplayWidget::mousePressEvent(QMouseEvent *event)
{
    if( event != NULL )
        _lastPosMouse = event->pos();
}

// Mouse movement management
void GLDisplayWidget::mouseMoveEvent(QMouseEvent *event)
{
    int dx = event->x() - _lastPosMouse.x();
    int dy = event->y() - _lastPosMouse.y();

    if( event != NULL )
    {
        _angle += dx;
        _angle += dy;
        _lastPosMouse = event->pos();

        updateGL(); //UpgradeQt6: update();
    }
}

// Mouse Management for the zoom
void GLDisplayWidget::wheelEvent(QWheelEvent *event) {
    QPoint numDegrees = event->angleDelta();
    double stepZoom = 0.25;
    if (!numDegrees.isNull())
    {
      _Z = (numDegrees.x() > 0 || numDegrees.y() > 0) ? _Z + stepZoom : _Z - stepZoom;
    }
}
