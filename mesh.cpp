#include <cstdio>
#include <map>
#include <QDebug>
#include <math.h>

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

double Point::norm2() const {
    return _x*_x + _y*_y + _z*_z;
};

double Point::norm() const {
    return sqrt(norm2());
}

Point Point::normalize() {
    double n = norm();
    return Point(_x/n,_y/n,_z/n);
};

Point operator+(const Point& A, const Point& B) {
    return Point(A._x+B._x,A._y+B._y,A._z+B._z);
};

Point operator-(const Point& A, const Point& B) {
    return Point(A._x-B._x,A._y-B._y,A._z-B._z);
};

Point operator*(const Point& A, const double b) {
    return Point(A._x*b,A._y*b,A._z*b);
};

Point operator*(const double b, const Point& A) {
    return Point(A._x*b,A._y*b,A._z*b);
};

Point operator/(const Point& A, const double b) {
    return Point(A._x/b,A._y/b,A._z/b);
};

double dot(const Point& A, const Point& B) {
    return A._x*B._x+A._y*B._y+A._z*B._z;
};

Point cross(const Point& A, const Point& B) {
    return Point(
                A._y*B._z - A._z*B._y,
                A._z*B._x - A._x*B._z,
                A._x*B._y - A._y*B._x
                );
};

double orientation(const Point& A, const Point& B, const Point& C) {
    Point cross_product = cross(B-A, C-A);
    return dot(cross_product, Point(0,0,1));
};

int Mesh::inTriangle(int ti, const Point &D) {
    Triangle triangle_on = *getTriangle(ti);
    std::vector<double> orientations;
    for (int k = 0; k<3; ++k) {
        orientations[k] = orientation(*getPoint(triangle_on.point_indices[k]), *getPoint(triangle_on.point_indices[(k+1)%3]), D);
    }
    if (orientations[0] > 0 && orientations[1] > 0 && orientations[2] > 0) {
        return 1;
    } else if (orientations[0] == 0 || orientations[1] == 0 || orientations[2] == 0) {
        return 0;
    } else {
        return -1;
    }
};

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

Point* Mesh::getPoint(int i) {
    return &points[i];
};

Triangle* Mesh::getTriangle(int i) {
    return &triangles[i];
}

int Mesh::faceSize() {
    return triangles.size();
};

int Mesh::verticesSize() {
    return points.size();
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
    // int number_vertices;
    // int number_faces;
    // sscanf(line, "%d %d 0", &number_vertices, &number_faces);
    QString qline = QString(line);
    QStringList metadata = qline.split(' ');
    int number_vertices = metadata[0].toInt();
    int number_faces = metadata[1].toInt();
    for (int i = 0; i < number_vertices; ++i) {
        char line[255];
        if (!fgets(line, 255, f)) break;
        // double x,y,z;
        // sscanf(line, "%lf %lf %lf", &x,&y,&z);
        QString qline = QString(line);
        QStringList coords = qline.split(' ');
        double x = coords[0].toDouble();
        double y = coords[1].toDouble();
        double z = coords[2].toDouble();
        points.push_back(Point(x*width, y*depth, z*height));
    }

    std::map<indice_tuple, std::vector<int>> edge_to_triangle;
    for (int i = 0; i < number_faces; ++i) {
        char line[255];
        if (!fgets(line, 255, f)) break;
        // int pi,pj,pk;
        // sscanf(line, "3 %d %d %d", &pi, &pj, &pk);
        QString qline = QString(line);
        QStringList indices = qline.split(' ');
        int pi = indices[1].toInt();
        int pj = indices[2].toInt();
        int pk = indices[3].toInt();
        triangles.push_back(Triangle(pi,pj,pk, -1,-1,-1));
        edge_to_triangle[indice_tuple(pi,pj)].push_back(i);
        edge_to_triangle[indice_tuple(pi,pk)].push_back(i);
        edge_to_triangle[indice_tuple(pk,pj)].push_back(i);
        points[pi].triangle_indice = i;
        points[pj].triangle_indice = i;
        points[pk].triangle_indice = i;
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
            if (triangle_i.size() == 2) {triangles[indice].adj_triangle[0] = triangle_i[1];}
        } else {
            triangles[indice].adj_triangle[0] = triangle_i[0];
        }
        if (triangle_j[0] == indice) {
            if (triangle_j.size() == 2) {triangles[indice].adj_triangle[1] = triangle_j[1];}
        } else {
            triangles[indice].adj_triangle[1] = triangle_j[0];
        }
        if (triangle_k[0] == indice) {
            if (triangle_k.size() == 2) {triangles[indice].adj_triangle[2] = triangle_k[1];}
        } else {
            triangles[indice].adj_triangle[2] = triangle_k[0];
        }
    }
};

Point Mesh::getCrossTriangle(Triangle* ti, int j) {
    for (int k = 0; k<3; ++k) {
        if (j == k) {
            return cross(points[ti->point_indices[(k+1)%3]]-points[ti->point_indices[k]],
                    points[ti->point_indices[(k+2)%3]]-points[ti->point_indices[k]]);
        }
    }
    return Point(0,0,0);
};

double Mesh::getDotTriangle(Triangle* ti, int j) {
    for (int k = 0; k<3; ++k) {
        if (j == k) {
            return dot(points[ti->point_indices[(k+1)%3]]-points[ti->point_indices[k]],
                    points[ti->point_indices[(k+2)%3]]-points[ti->point_indices[k]]);
        }
    }
    return 0;
};

Iterator_on_faces::Iterator_on_faces(Mesh* mesh) {
    related_mesh = mesh;
    face_indice = 0;
};

void Iterator_on_faces::operator++() {
    ++face_indice;
};

Triangle* Iterator_on_faces::operator*() {
    return related_mesh->getTriangle(face_indice);
};

bool Iterator_on_faces::face_end() {
    return face_indice == related_mesh->faceSize();
};

Iterator_on_vertices::Iterator_on_vertices(Mesh* mesh) {
    related_mesh = mesh;
    vertex_indice = 0;
};

void Iterator_on_vertices::operator++() {
    ++vertex_indice;
};

Point* Iterator_on_vertices::operator*() {
    return related_mesh->getPoint(vertex_indice);
};

bool Iterator_on_vertices::vertex_end() {
    return vertex_indice == related_mesh->verticesSize();
};

Circulator_on_faces::Circulator_on_faces(Mesh* mesh, int related_vertex_indice) {
    related_mesh = mesh;
    this->related_vertex_indice = related_vertex_indice;
    face_indice = mesh->getPoint(related_vertex_indice)->triangle_indice;
};

Triangle* Circulator_on_faces::operator*() {
    return related_mesh->getTriangle(face_indice);
};

Circulator_on_faces Circulator_on_faces::next() {
    Triangle* triangle_on = **this;
    Circulator_on_faces new_circ = *this;
    for (int k = 0; k<3; ++k) {
        if (related_vertex_indice == triangle_on->point_indices[k]) {
            new_circ.face_indice = triangle_on->adj_triangle[(k+1)%3];
        }
    }
    return new_circ;
};

Circulator_on_faces Circulator_on_faces::previous() {
    Triangle* triangle_on = **this;
    Circulator_on_faces new_circ = *this;
    for (int k = 0; k<3; ++k) {
        if (related_vertex_indice == triangle_on->point_indices[k]) {
            new_circ.face_indice = triangle_on->adj_triangle[(k+2)%3];
        }
    }
    return new_circ;
};

void Circulator_on_faces::operator++() {
    Triangle* triangle_on = **this;
    for (int k = 0; k<3; ++k) {
        if (related_vertex_indice == triangle_on->point_indices[k]) {
            int new_indice = triangle_on->adj_triangle[(k+1)%3];
            if (new_indice != -1) {
                face_indice = new_indice;
            } else {
                Circulator_on_faces prev = previous();
                while (prev.face_indice != -1) {
                    face_indice = prev.face_indice;
                    prev = prev.previous();
                }
            }

        }
    }
};

void Circulator_on_faces::operator--() {
    Triangle* triangle_on = **this;
    for (int k = 0; k<3; ++k) {
        if (related_vertex_indice == triangle_on->point_indices[k]) {
            int new_indice = triangle_on->adj_triangle[(k+2)%3];
            if (new_indice != -1) {
                face_indice = new_indice;
            } else {
                Circulator_on_faces nex = next();
                while (nex.face_indice != -1) {
                    face_indice = nex.face_indice;
                    nex = nex.next();
                }
            }

        }
    }
};

Circulator_on_vertices::Circulator_on_vertices(Mesh* mesh, int related_vertex_indice) {
    related_mesh = mesh;
    this->related_vertex_indice = related_vertex_indice;
    right_face_indice = mesh->getPoint(related_vertex_indice)->triangle_indice;
    left_face_indice = -1;
    vertex_indice = -1;
    Triangle* triangle_on = mesh->getTriangle(right_face_indice);
    for (int k = 0; k<3; ++k) {
        if (related_vertex_indice == triangle_on->point_indices[k]) {
            vertex_indice = triangle_on->point_indices[(k+1)%3];
            left_face_indice = triangle_on->adj_triangle[(k+2)%3];
        }
    }
};

Point* Circulator_on_vertices::operator*() {
    return related_mesh->getPoint(vertex_indice);
};

Circulator_on_vertices Circulator_on_vertices::next() {
    Triangle* triangle_on = related_mesh->getTriangle(right_face_indice);
    Circulator_on_vertices new_circ = *this;
    for (int k = 0; k<3; ++k) {
        if (related_vertex_indice == triangle_on->point_indices[k]) {
            new_circ.left_face_indice = right_face_indice;
            new_circ.right_face_indice = triangle_on->adj_triangle[(k+1)%3];
            new_circ.vertex_indice = triangle_on->point_indices[(k+2)%3];
        }
    }
    return new_circ;
};


Circulator_on_vertices Circulator_on_vertices::previous() {
    Triangle* triangle_on = related_mesh->getTriangle(left_face_indice);
    Circulator_on_vertices new_circ = *this;
    for (int k = 0; k<3; ++k) {
        if (related_vertex_indice == triangle_on->point_indices[k]) {
            new_circ.right_face_indice = left_face_indice;
            new_circ.left_face_indice = triangle_on->adj_triangle[(k+2)%3];
            new_circ.vertex_indice = triangle_on->point_indices[(k+1)%3];
        }
    }
    return new_circ;
};

void Circulator_on_vertices::operator++() {
    if (right_face_indice != -1) {
        Triangle* triangle_on = related_mesh->getTriangle(right_face_indice);
        for (int k = 0; k<3; ++k) {
            if (related_vertex_indice == triangle_on->point_indices[k]) {
                left_face_indice = right_face_indice;
                right_face_indice = triangle_on->adj_triangle[(k+1)%3];
                vertex_indice = triangle_on->point_indices[(k+2)%3];
            }
        }
    } else {
        Circulator_on_vertices prev = *this;
        do {
            prev = prev.previous();
            left_face_indice = prev.left_face_indice;
            right_face_indice = prev.right_face_indice;
            vertex_indice = prev.vertex_indice;
        } while (prev.left_face_indice != -1);
    }
}

void Circulator_on_vertices::operator--() {
    if (left_face_indice != -1) {
        Triangle* triangle_on = related_mesh->getTriangle(left_face_indice);
        for (int k = 0; k<3; ++k) {
            if (related_vertex_indice == triangle_on->point_indices[k]) {
                right_face_indice = left_face_indice;
                left_face_indice = triangle_on->adj_triangle[(k+2)%3];
                vertex_indice = triangle_on->point_indices[(k+1)%3];
            }
        }
    } else {
        Circulator_on_vertices nex = *this;
        do {
            nex = nex.previous();
            left_face_indice = nex.left_face_indice;
            right_face_indice = nex.right_face_indice;
            vertex_indice = nex.vertex_indice;
        } while (nex.right_face_indice != -1);
    }
}

Point Mesh::computeLaplacian(int i) {
    Circulator_on_vertices circu = Circulator_on_vertices(this, i);
    int starting_vertex = circu.vertex_indice;
    double aera = 0;
    double laplacian_x = 0;
    double laplacian_y = 0;
    double laplacian_z = 0;
    do {
        double cotan_right = 0.;
        double cotan_left = 0.;
        if (circu.right_face_indice == -1) {
            Triangle* left_triangle = getTriangle(circu.left_face_indice);
            int left_triangle_angle_indice = -1;
            for (int k = 0; k<3; ++k) {
                if (circu.vertex_indice == left_triangle->point_indices[k]) {
                    left_triangle_angle_indice = (k+2)%3;
                }
            }
            Point cross_left = getCrossTriangle(left_triangle, left_triangle_angle_indice);
            double cross_left_norm = cross_left.norm();
            double dot_left = getDotTriangle(left_triangle, left_triangle_angle_indice);
            cotan_left = dot_left/cross_left_norm;
            aera += cross_left_norm;
        } else if (circu.left_face_indice == -1) {
            Triangle* right_triangle = getTriangle(circu.right_face_indice);
            int right_triangle_angle_indice = -1;
            for (int k = 0; k<3; ++k) {
                if (circu.vertex_indice == right_triangle->point_indices[k]) {
                    right_triangle_angle_indice = (k+1)%3;
                }
            }
            Point cross_right = getCrossTriangle(right_triangle, right_triangle_angle_indice);
            double cross_right_norm = cross_right.norm();
            double dot_right = getDotTriangle(right_triangle, right_triangle_angle_indice);
            cotan_right = dot_right/cross_right_norm;
            aera += cross_right_norm;
        } else {
            Triangle* right_triangle = getTriangle(circu.right_face_indice);
            Triangle* left_triangle = getTriangle(circu.left_face_indice);
            int right_triangle_angle_indice = -1;
            int left_triangle_angle_indice = -1;
            for (int k = 0; k<3; ++k) {
                if (circu.vertex_indice == right_triangle->point_indices[k]) {
                    right_triangle_angle_indice = (k+1)%3;
                }
            }

            for (int k = 0; k<3; ++k) {
                if (circu.vertex_indice == left_triangle->point_indices[k]) {
                    left_triangle_angle_indice = (k+2)%3;
                }
            }

            Point cross_right = getCrossTriangle(right_triangle, right_triangle_angle_indice);
            Point cross_left = getCrossTriangle(left_triangle, left_triangle_angle_indice);
            double cross_right_norm = cross_right.norm();
            double cross_left_norm = cross_left.norm();
            double dot_right = getDotTriangle(right_triangle, right_triangle_angle_indice);
            double dot_left = getDotTriangle(left_triangle, left_triangle_angle_indice);

            cotan_right = dot_right/cross_right_norm;
            cotan_left = dot_left/cross_left_norm;
            aera += cross_right_norm;
        }
        Point* rotating_point = getPoint(circu.vertex_indice);
        Point* center_point = getPoint(i);
        laplacian_x += (cotan_left+cotan_right)*(rotating_point->_x-center_point->_x);
        laplacian_y += (cotan_left+cotan_right)*(rotating_point->_y-center_point->_y);
        laplacian_z += (cotan_left+cotan_right)*(rotating_point->_z-center_point->_z);
        ++circu;
    } while (circu.vertex_indice != starting_vertex);

    double invert_aera = 3/aera;
    return Point(invert_aera*laplacian_x, invert_aera*laplacian_y,invert_aera*laplacian_z);
};

void Mesh::computeLaplacians() {
    for (Iterator_on_vertices itv = Iterator_on_vertices(this); !itv.vertex_end(); ++itv) {
        Point laplacian_i = computeLaplacian(itv.vertex_indice);
        if (abs(laplacian_i._x) > maxLaplacian._x) maxLaplacian._x = abs(laplacian_i._x);
        if (abs(laplacian_i._y) > maxLaplacian._y) maxLaplacian._y = abs(laplacian_i._y);
        if (abs(laplacian_i._z) > maxLaplacian._z) maxLaplacian._z = abs(laplacian_i._z);

        if (abs(laplacian_i._x) < minLaplacian._x) minLaplacian._x = abs(laplacian_i._x);
        if (abs(laplacian_i._y) < minLaplacian._y) minLaplacian._y = abs(laplacian_i._y);
        if (abs(laplacian_i._z) < minLaplacian._z) minLaplacian._z = abs(laplacian_i._z);

        laplacians.push_back(laplacian_i);
    }

};

void Mesh::eraseFace(int i) {
    triangles.erase(triangles.begin()+i);
    for (Triangle& triangle : triangles) {
        for (int k = 0; k < 3; ++k) {
            if (triangle.adj_triangle[k] >= i) {--triangle.adj_triangle[k];}
        }
    }
}

void Mesh::triangleSplit(Point& middlePoint, int i) {
    Triangle triangleSplitted = *getTriangle(i);
    int nb_vertices = verticesSize();
    int nb_triangles = faceSize();
    points.push_back(middlePoint);
    for (int k =0; k<3; ++k) {
        triangles.push_back(
            Triangle(
                triangleSplitted.point_indices[k%3],
                triangleSplitted.point_indices[(k+1)%3],
                nb_vertices,
                nb_triangles+(k+1)%3,
                nb_triangles+(k+2)%3,
                triangleSplitted.adj_triangle[(k+2)%3]
            )
        );
    }
    for (int k = 0; k < 3; ++k) {
        if (triangleSplitted.adj_triangle[k] != -1) {
            Triangle* neighbour = getTriangle(triangleSplitted.adj_triangle[k]);
            for (int l = 0;  l<3; ++l) {
                if (i == neighbour->adj_triangle[l]) {
                    neighbour->adj_triangle[0] = nb_triangles+(k+1)%3;
                }
            }
        }
    }
    eraseFace(i);
};

int find(std::vector<int> liste, int a) {
    for (int k = 0; k<(int)liste.size(); ++k ) {
        if (liste[k] == a) {return k;}
    }
    return -1;
};

void Mesh::edgeFlip(int i, int j) {
    Triangle triangle1 = *getTriangle(i);
    Triangle triangle2 = *getTriangle(j);
    int indice1 = -1;
    int indice2 = -1;
    for (int k = 0; k<3; ++k) {
        int indice_find1 = find(triangle2.point_indices,triangle1.point_indices[k]);
        if (indice_find1 != -1) {
            for (int l = k+1; l<3; ++l) {
                int indice_find2 = find(triangle2.point_indices, triangle1.point_indices[l]);
                if (indice_find2 != 1) {
                    indice1 = (2*(k+l))%3;
                    indice2 = (2*(indice_find1+indice_find2))%3;
                }
            }
        }
    }
    int nb_triangles = faceSize();
    if (indice1 != -1) {
        triangles.push_back(
            Triangle(
                triangle1.point_indices[indice1],
                triangle1.point_indices[(indice1+1)%3],
                triangle2.point_indices[indice2],
                triangle2.adj_triangle[(indice2+1)%3],
                nb_triangles+1,
                triangle1.adj_triangle[(indice1+2)%3]
            )
        );
        triangles.push_back(
            Triangle(
                triangle2.point_indices[indice2],
                triangle2.point_indices[(indice2+1)%3],
                triangle1.point_indices[indice1],
                triangle1.adj_triangle[(indice1+1)%3],
                nb_triangles,
                triangle2.adj_triangle[(indice2+2)%3]
            )
        );
    }
    for (int k = 1; k<3; ++k) {
        if (triangle1.adj_triangle[(indice1+k)%3] != -1) {
            Triangle* tr1 = getTriangle(triangle1.adj_triangle[(indice1+k)%3]);
            for (int l = 0; l<3; ++l) {
                if (tr1->adj_triangle[l] == i) {tr1->adj_triangle[l] = nb_triangles+(k%2);}
            }
        }
        if (triangle2.adj_triangle[(indice2+k)%3] != -1) {
            Triangle* tr2 = getTriangle(triangle2.adj_triangle[(indice2+k)%3]);
            for (int l = 0; l<3; ++l) {
                if (tr2->adj_triangle[l] == j) {tr2->adj_triangle[l] = nb_triangles+((k+1)%2);}
            }
        }
    }
    if (i<j) {
        eraseFace(j);
        eraseFace(i);
    } else {
        eraseFace(i);
        eraseFace(j);
    }
};

// DRAWING FEATURES

void Mesh::drawMesh(bool laplacian) {
    for (auto& triangle : triangles) {
        if (laplacian) {
            Point medLaplacian = (laplacians[triangle.point_indices[0]]+laplacians[triangle.point_indices[1]]+laplacians[triangle.point_indices[2]])/3;
            glColor3d((abs(medLaplacian._x)-minLaplacian._x)/(maxLaplacian._x-minLaplacian._x),
                      (abs(medLaplacian._y)-minLaplacian._y)/(maxLaplacian._y-minLaplacian._y),
                      (abs(medLaplacian._z)-minLaplacian._z)/(maxLaplacian._z-minLaplacian._z));
        } else {
            glColor3d(0.3,0.3,0.6);
        };
        glBegin(GL_TRIANGLES);
        glPointDraw(points[triangle.point_indices[0]]);
        glPointDraw(points[triangle.point_indices[1]]);
        glPointDraw(points[triangle.point_indices[2]]);
        glEnd();
    };
    if (not laplacian) drawMeshWireFrame();
};

void Mesh::drawMeshWireFrame() {
    for (auto& triangle : triangles) {
        glColor3d(0.5,0.5,0.5);
        glBegin(GL_LINE_STRIP);
        glPointDraw(points[triangle.point_indices[0]]);
        glPointDraw(points[triangle.point_indices[1]]);
        glEnd();

        glColor3d(0.5,0.5,0.5);
        glBegin(GL_LINE_STRIP);
        glPointDraw(points[triangle.point_indices[1]]);
        glPointDraw(points[triangle.point_indices[2]]);
        glEnd();

        glColor3d(0.5,0.5,0.5);
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

