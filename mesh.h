#ifndef MESH_H
#define MESH_H

#include <QGLWidget> //UpgradeQt6: #include <QOpenGLWidget>

// TO MODIFY
class Point
{
public:
    double _x;
    double _y;
    double _z;

    Point():_x(),_y(),_z() {}
    Point(float x_, float y_, float z_):_x(x_),_y(y_),_z(z_) {}
};


//** TP : TO MODIFY

class Triangle
{
public:
    std::vector<int> point_indices;
    std::vector<int> adj_triangle;

    Triangle():point_indices(3),adj_triangle(3) {};
    Triangle(int pi, int pj, int pk, int ti, int tj, int tk);
};

class Mesh
{
  // (Q ou STL)Vector of vertices
    std::vector<Point> points;
  // (Q ou STL)Vector of faces
    std::vector<Triangle> triangles;
  // Those who do not know about STL Vectors should have a look at cplusplus.com examples
public:
    Mesh() {}; // Constructors automatically called to initialize a Mesh (default strategy)
    //~Mesh(); // Destructor automatically called before a Mesh is destroyed (default strategy)
    void drawMesh();
    void drawMeshWireFrame();
    void buildTetrahedron(double width, double depth, double height);
    void buildPyramid(double width, double depth, double height);
    void buildInput(char const *path_to_mesh, double width, double depth, double height);
};

class GeometricWorld //Here used to create a singleton instance
{
  QVector<Point> _bBox;  // Bounding box
public :
  GeometricWorld();
  void draw();
  void drawWireFrame();
  // ** TP Can be extended with further elements;
  // Mesh _mesh;
};


#endif // MESH_H
