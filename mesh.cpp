#include <cstdio>
#include <map>
#include <QDebug>

#include "mesh.h"

GeometricWorld::GeometricWorld()
{
    double width=0.5, depth=0.6, height=0.8;
    _bBox.push_back(Point(-0.5*width,-0.5*depth,-0.5*height)); //0
    _bBox.push_back(Point(-0.5*width,0.5*depth,-0.5*height)); // 1
    _bBox.push_back(Point(0.5*width,-0.5*depth,-0.5*height)); // 2
    _bBox.push_back(Point(-0.5*width,-0.5*depth,0.5*height)); // 3
}

// The following functions could be displaced into a module OpenGLDisplayGeometricWorld that would include mesh.h

// Draw a Point
void glPointDraw(const Point & p) {
    glVertex3f(p._x, p._y, p._z);
}

Triangle::Triangle(int pi,int pj,int pk,  int ti,int tj,int tk) : Triangle() {
    point_indices[0] = pi;
    point_indices[1] = pj;
    point_indices[2] = pk;

    adj_triangle[0] = ti;
    adj_triangle[1] = tj;
    adj_triangle[2] = tk;
};

void Mesh::buildTetrahedron(double width, double depth, double height) {
    // Build hardcoded tetrahedron (triangle based pyramid)
    points.push_back(Point(-0.5*width,-0.5*depth,-0.5*height)); //0
    points.push_back(Point(-0.5*width,0.5*depth,-0.5*height)); // 1
    points.push_back(Point(0.5*width,-0.5*depth,-0.5*height)); // 2
    points.push_back(Point(-0.5*width,-0.5*depth,0.5*height)); // 3

    triangles.push_back(Triangle(0,1,2,  2,3,1));
    triangles.push_back(Triangle(0,3,1,  2,0,3));
    triangles.push_back(Triangle(1,3,2,  3,0,1));
    triangles.push_back(Triangle(0,3,2,  2,0,1));
};

void Mesh::buildPyramid(double width, double depth, double height) {
    // Build hardcoded squared base pyramid
    points.push_back(Point(-0.5*width, -0.5*depth, -0.5*height)); // 0
    points.push_back(Point(-0.5*width, 0.5*depth, -0.5*height)); // 1
    points.push_back(Point(0.5*width, -0.5*depth, -0.5*height)); // 2
    points.push_back(Point(0.5*width, 0.5*depth, -0.5*height)); // 3
    points.push_back(Point(0,0,0.5*height)); // 4

    triangles.push_back(Triangle(0,2,1, 4,1,5));
    triangles.push_back(Triangle(0,1,4, 2,5,0));
    triangles.push_back(Triangle(1,3,4, 3,1,4));
    triangles.push_back(Triangle(3,2,4, 5,2,4));
    triangles.push_back(Triangle(1,2,3, 3,2,0));
    triangles.push_back(Triangle(0,4,2, 3,0,1));
};

class indice_tuple {
    // Class to map a tuple of vertex indices (edges) to a couple of triangle
    // i<j for unicity
public:
    indice_tuple(int a, int b) {
        if (a<b) {
            i = a;
            j = b;
        } else {
            j = a;
            i = b;
        }
    };

    bool operator<(const indice_tuple other) const {
        if (i < other.i) return true;
        if (i > other.i) return false;
        return j < other.j;
    };

    int i;
    int j;
};

void Mesh::buildInput(char const *path_to_mesh, double width, double depth, double height) {
    // Build mesh from a .off mesh input
    // Start by reading the mesh file and create non linked triangles, then link the vertices with their opposed triangle
    FILE* f;
    f = fopen(path_to_mesh, "r");
    if (f == NULL) {
        qDebug() << "Can't read the file";
    }
    char line[255];
    if (!fgets(line, 255, f)) return;
    int number_vertices;
    int number_faces;
    sscanf(line, "%d %d 0", &number_vertices, &number_faces);
    for (int i = 0; i < number_vertices; ++i) {
        char line[255];
        if (!fgets(line, 255, f)) break;
        double x,y,z;
        sscanf(line, "%lf %lf %lf", &x,&y,&z);
        points.push_back(Point(x*width, y*depth, z*height));
    }

    std::map<indice_tuple, std::vector<int>> edge_to_triangle;
    for (int i = 0; i < number_faces; ++i) {
        char line[255];
        if (!fgets(line, 255, f)) break;
        int pi,pj,pk;
        sscanf(line, "3 %d %d %d", &pi, &pj, &pk);
        triangles.push_back(Triangle(pi,pj,pk, 0,0,0));
        edge_to_triangle[indice_tuple(pi,pj)].push_back(i);
        edge_to_triangle[indice_tuple(pi,pk)].push_back(i);
        edge_to_triangle[indice_tuple(pk,pj)].push_back(i);
    }
    fclose(f);

    // Linking loop
    for (int indice = 0; indice < number_faces; ++indice) {
        int pi = triangles[indice].point_indices[0];
        int pj = triangles[indice].point_indices[1];
        int pk = triangles[indice].point_indices[2];

        std::vector<int> triangle_i = edge_to_triangle[indice_tuple(pj, pk)];
        std::vector<int> triangle_j = edge_to_triangle[indice_tuple(pi, pk)];
        std::vector<int> triangle_k = edge_to_triangle[indice_tuple(pi, pj)];
        if (triangle_i[0] == indice) {
            triangles[indice].adj_triangle[0] = triangle_i[1];
        } else {
            triangles[indice].adj_triangle[0] = triangle_i[0];
        }
        if (triangle_j[0] == indice) {
            triangles[indice].adj_triangle[1] = triangle_j[1];
        } else {
            triangles[indice].adj_triangle[1] = triangle_j[0];
        }
        if (triangle_k[0] == indice) {
            triangles[indice].adj_triangle[2] = triangle_k[1];
        } else {
            triangles[indice].adj_triangle[2] = triangle_k[0];
        }
    }
};

void Mesh::drawMesh() {
    for (auto& triangle : triangles) {
        glColor3d(0.5,0.5,0.5);
        glBegin(GL_TRIANGLES);
        glPointDraw(points[triangle.point_indices[0]]);
        glPointDraw(points[triangle.point_indices[1]]);
        glPointDraw(points[triangle.point_indices[2]]);
        glEnd();
    };
    drawMeshWireFrame();
};

void Mesh::drawMeshWireFrame() {
    for (auto& triangle : triangles) {
        glColor3d(0.3,0.3,0.3);
        glBegin(GL_LINE_STRIP);
        glPointDraw(points[triangle.point_indices[0]]);
        glPointDraw(points[triangle.point_indices[1]]);
        glEnd();

        glColor3d(0.4,0.4,0.4);
        glBegin(GL_LINE_STRIP);
        glPointDraw(points[triangle.point_indices[1]]);
        glPointDraw(points[triangle.point_indices[2]]);
        glEnd();

        glColor3d(0.4,0.4,0.4);
        glBegin(GL_LINE_STRIP);
        glPointDraw(points[triangle.point_indices[2]]);
        glPointDraw(points[triangle.point_indices[0]]);
        glEnd();
    }
};

//Example with a bBox
void GeometricWorld::draw() {
    glColor3d(1,0,0);
    glBegin(GL_TRIANGLES);
    glPointDraw(_bBox[0]);
    glPointDraw(_bBox[1]);
    glPointDraw(_bBox[2]);
    glEnd();

    glColor3d(0,1,0);
    glBegin(GL_TRIANGLES);
    glPointDraw(_bBox[0]);
    glPointDraw(_bBox[2]);
    glPointDraw(_bBox[3]);
    glEnd();

    glColor3d(0,0,1);
    glBegin(GL_TRIANGLES);
    glPointDraw(_bBox[0]);
    glPointDraw(_bBox[3]);
    glPointDraw(_bBox[1]);
    glEnd();

    //glColor3d(1,1,0);
}

//Example with a wireframe bBox
void GeometricWorld::drawWireFrame() {
    glColor3d(0,1,0);
    glBegin(GL_LINE_STRIP);
    glPointDraw(_bBox[0]);
    glPointDraw(_bBox[1]);
    glEnd();
    glColor3d(0,0,1);
    glBegin(GL_LINE_STRIP);
    glPointDraw(_bBox[0]);
    glPointDraw(_bBox[2]);
    glEnd();
    glColor3d(1,0,0);
    glBegin(GL_LINE_STRIP);
    glPointDraw(_bBox[0]);
    glPointDraw(_bBox[3]);
    glEnd();
}

