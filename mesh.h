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
    int triangle_indice;

    Point():_x(),_y(),_z(),triangle_indice() {}
    Point(double x_, double y_, double z_):_x(x_),_y(y_),_z(z_),triangle_indice() {}

    double norm2() const;
    double norm() const;
    Point normalize();
};

Point operator+(const Point& A, const Point& B);
Point operator-(const Point& A, const Point& B);
Point operator*(const Point& A, const double b);
Point operator*(const double b, const Point& A);
Point operator/(const Point& A, const double b);
double dot(const Point& A, const Point& B);
Point cross(const Point& A, const Point& B);
double orientation(const Point& A, const Point& B, const Point& C);
bool intersectSegments(const Point& A, const Point& B, const Point& C, const Point& D);


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
    std::vector<Point> laplacians;
    Point maxLaplacian;
    Point minLaplacian;
    Mesh() {}; // Constructors automatically called to initialize a Mesh (default strategy)
    //~Mesh(); // Destructor automatically called before a Mesh is destroyed (default strategy)
    Point* getPoint(int i);
    Triangle* getTriangle(int i);
    int faceSize();
    int verticesSize();
    void drawMesh(bool laplacian);
    void drawMeshWireFrame();
    void buildTetrahedron(double width, double depth, double height);
    void buildPyramid(double width, double depth, double height);
    void buildInput(char const *path_to_mesh, double width, double depth, double height);
    Point getCrossTriangle(Triangle* ti, int j);
    double getDotTriangle(Triangle* ti, int j);

    Point computeLaplacian(int i);
    void computeLaplacians();

    void eraseFace(int i);
    void triangleSplit(Point& middlePoint, int i);
    void edgeFlip(int i, int j);

    int inTriangle(int ti, const Point& D);
    int findTriangle(const Point& M);
};

class Iterator_on_faces {
public:
    int face_indice;
    Mesh* related_mesh;
    Iterator_on_faces(Mesh* mesh);
    void operator++();
    Triangle* operator*();
    bool face_end();
};

class Iterator_on_vertices {
public:
    int vertex_indice;
    Mesh* related_mesh;
    Iterator_on_vertices(Mesh* mesh);
    void operator++();
    Point* operator*();
    bool vertex_end();
};

class Circulator_on_faces {
public:
    int face_indice;
    int related_vertex_indice;
    Mesh* related_mesh;
    Circulator_on_faces(Mesh* mesh, int related_vertex_indice);
    void operator++();
    void operator--();
    Triangle* operator*();

    Circulator_on_faces previous();
    Circulator_on_faces next();
};

class Circulator_on_vertices {
public:
    int vertex_indice;
    int right_face_indice;
    int left_face_indice;
    Mesh* related_mesh;
    int related_vertex_indice;
    Circulator_on_vertices(Mesh* mesh, int related_vertex_indice);
    void operator++();
    void operator--();
    Point* operator*();

    Circulator_on_vertices previous();
    Circulator_on_vertices next();
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
