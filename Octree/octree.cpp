#include "octree.hpp"
#include <fstream>  // CORREGIDO: Inclusión necesaria para resolver std::ofstream y el operador <<
#include <iostream>

//###########################################################################
//###########################################################################
//                                  PRIVATE
//###########################################################################
//###########################################################################

double Octree::square_distance(const Point &a, const Point &b) const {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = a.z - b.z;
    return dx * dx + dy * dy + dz * dz;
}

double Octree::minimum_square_box_distance(const Point &p) const {
    double closestX = std::max(bottomLeft.x, std::min(p.x, bottomLeft.x + h));
    double closestY = std::max(bottomLeft.y, std::min(p.y, bottomLeft.y + h));
    double closestZ = std::max(bottomLeft.z, std::min(p.z, bottomLeft.z + h));

    return square_distance(p, Point(closestX, closestY, closestZ));
}

void Octree::find_closest_aux(const Point &cPoint, double &bestDistSq, Point &bestPoint, bool &found) const {
    if (minimum_square_box_distance(cPoint) > bestDistSq) return;

    if (isLeaf) {
        for (int i = 0; i < nPoints; i++) {
            double tmpDistSq = square_distance(cPoint, points[i]);
            if (tmpDistSq <= bestDistSq) {
                bestDistSq = tmpDistSq;
                bestPoint = points[i];
                found = true;
            }
        }
    } else {
        for (int i = 0; i < 8; i++) {
            if (children[i] != nullptr) {
                children[i]->find_closest_aux(cPoint, bestDistSq, bestPoint, found);
            }
        }
    }
}

// NUEVO: Método privado auxiliar para exportar los cubos de los nodos hoja
void Octree::export_obj_aux(std::ofstream &file, int &vertexOffset) const {
    if (isLeaf) {
        // Solo exportamos si la hoja contiene al menos un punto
        if (nPoints == 0) return;

        double x = bottomLeft.x;
        double y = bottomLeft.y;
        double z = bottomLeft.z;

        // NUEVO: Definición de los 8 vértices del cubo que abarca el nodo hoja
        file << "v " << x << " " << y << " " << z << "\n";
        file << "v " << x + h << " " << y << " " << z << "\n";
        file << "v " << x + h << " " << y + h << " " << z << "\n";
        file << "v " << x << " " << y + h << " " << z << "\n";
        file << "v " << x << " " << y << " " << z + h << "\n";
        file << "v " << x + h << " " << y << " " << z + h << "\n";
        file << "v " << x + h << " " << y + h << " " << z + h << "\n";
        file << "v " << x << " " << y + h << " " << z + h << "\n";

        int v = vertexOffset;

        // NUEVO: Definición de las 12 aristas usando líneas ('l') en formato OBJ
        file << "l " << v << " " << v + 1 << "\n";
        file << "l " << v + 1 << " " << v + 2 << "\n";
        file << "l " << v + 2 << " " << v + 3 << "\n";
        file << "l " << v + 3 << " " << v << "\n";

        file << "l " << v + 4 << " " << v + 5 << "\n";
        file << "l " << v + 5 << " " << v + 6 << "\n";
        file << "l " << v + 6 << " " << v + 7 << "\n";
        file << "l " << v + 7 << " " << v + 4 << "\n";

        file << "l " << v << " " << v + 4 << "\n";
        file << "l " << v + 1 << " " << v + 5 << "\n";
        file << "l " << v + 2 << " " << v + 6 << "\n";
        file << "l " << v + 3 << " " << v + 7 << "\n";

        // Incrementar offset para el siguiente cubo
        vertexOffset += 8;
    } else {
        // Llamada recursiva a los octantes hijos
        for (int i = 0; i < 8; i++) {
            if (children[i] != nullptr) {
                children[i]->export_obj_aux(file, vertexOffset);
            }
        }
    }
}

//###########################################################################
//###########################################################################
//                                  PUBLIC
//###########################################################################
//###########################################################################

// CORREGIDO: Coincide con la firma const Point &bottomLeft de la cabecera
Octree::Octree(const Point &bottomLeft, double h, int capacity) {
    for (int i = 0; i < 8; i++) children[i] = nullptr;
    points = nullptr;

    this->capacity = capacity;
    isLeaf = true;
    this->bottomLeft = bottomLeft;
    this->h = h;

    nPoints = 0;
}

Octree::~Octree() {
    if (points != nullptr) {
        delete[] points;
    }
    for (int i = 0; i < 8; i++) {
        if (children[i] != nullptr) {
            delete children[i];
        }
    }
}

bool Octree::exist(const Point &cPoint) const {
    if (cPoint.x < bottomLeft.x || cPoint.x > bottomLeft.x + h ||
        cPoint.y < bottomLeft.y || cPoint.y > bottomLeft.y + h ||
        cPoint.z < bottomLeft.z || cPoint.z > bottomLeft.z + h)
        return false;

    if (isLeaf) {
        for (int i = 0; i < nPoints; i++) {
            if (cPoint.x == points[i].x && 
                cPoint.y == points[i].y && 
                cPoint.z == points[i].z)
                return true;
        }
        return false; 
    }

    double midX = bottomLeft.x + h / 2.0;
    double midY = bottomLeft.y + h / 2.0;
    double midZ = bottomLeft.z + h / 2.0;

    int nextOctant = -1;

    if (cPoint.x < midX) {
        if (cPoint.y < midY) {
            nextOctant = (cPoint.z < midZ) ? FLB : FLT;
        } else {
            nextOctant = (cPoint.z < midZ) ? FRB : FRT;
        }
    } else {
        if (cPoint.y < midY) {
            nextOctant = (cPoint.z < midZ) ? BLB : BLT;
        } else {
            nextOctant = (cPoint.z < midZ) ? BRB : BRT;
        }
    }

    if (children[nextOctant] == nullptr) return false;
    return children[nextOctant]->exist(cPoint);
}

void Octree::insert(const Point &cPoint) {
    if (exist(cPoint)) return;

    if (cPoint.x < bottomLeft.x || cPoint.x > bottomLeft.x + h ||
        cPoint.y < bottomLeft.y || cPoint.y > bottomLeft.y + h ||
        cPoint.z < bottomLeft.z || cPoint.z > bottomLeft.z + h)
        return;

    double midX = bottomLeft.x + h / 2.0;
    double midY = bottomLeft.y + h / 2.0;
    double midZ = bottomLeft.z + h / 2.0;

    int nextOctant = -1;

    if (isLeaf) {
        if (nPoints < capacity) {
            if (points == nullptr) {
                points = new Point[capacity + 1];
            }
            points[nPoints] = cPoint;
            ++nPoints;
            return;
        }

        points[capacity] = cPoint;
        double halfH = h / 2.0;

        double varX[] = {0, 0, 0, 0, halfH, halfH, halfH, halfH},
               varY[] = {0, halfH, halfH, 0, 0, halfH, halfH, 0},
               varZ[] = {halfH, halfH, 0, 0, halfH, halfH, 0, 0};

        for (int i = 0; i < 8; i++) {
            children[i] = new Octree(Point(bottomLeft.x + varX[i],
                                           bottomLeft.y + varY[i],
                                           bottomLeft.z + varZ[i]), halfH, capacity);
        }

        for (int i = 0; i <= capacity; i++) {
            if (points[i].x < midX) {
                if (points[i].y < midY) {
                    nextOctant = (points[i].z < midZ) ? FLB : FLT;
                } else {
                    nextOctant = (points[i].z < midZ) ? FRB : FRT;
                }
            } else {
                if (points[i].y < midY) {
                    nextOctant = (points[i].z < midZ) ? BLB : BLT;
                } else {
                    nextOctant = (points[i].z < midZ) ? BRB : BRT;
                }
            }
            children[nextOctant]->insert(points[i]);
        }

        nPoints = 0;
        isLeaf = false;
        delete[] points;
        points = nullptr;
    } else {
        if (cPoint.x < midX) {
            if (cPoint.y < midY) {
                nextOctant = (cPoint.z < midZ) ? FLB : FLT;
            } else {
                nextOctant = (cPoint.z < midZ) ? FRB : FRT;
            }
        } else {
            if (cPoint.y < midY) {
                nextOctant = (cPoint.z < midZ) ? BLB : BLT;
            } else {
                nextOctant = (cPoint.z < midZ) ? BRB : BRT;
            }
        }

        if (children[nextOctant] != nullptr) {
            children[nextOctant]->insert(cPoint);
        }
    }
}

Point Octree::find_closest(const Point &cPoint, double radius, bool &found) const{
    found = false;
    Point bestPoint;
    double bestDistSq = radius * radius;

    find_closest_aux(cPoint, bestDistSq, bestPoint, found);

    return bestPoint;
}

void Octree::print(int depth) const {
    std::string indent(depth * 2, ' ');
    std::cout << indent << "[Nodo] bottomLeft: (" << bottomLeft.x << ", " 
              << bottomLeft.y << ", " << bottomLeft.z << ") | h: " << h 
              << " | Puntos: " << nPoints << " | Hoja: " << (isLeaf ? "Si" : "No") << "\n";

    if (isLeaf) {
        for (int i = 0; i < nPoints; i++) {
            std::cout << indent << "   -> Point: (" << points[i].x << ", " 
                      << points[i].y << ", " << points[i].z << ")\n";
        }
    } else {
        for (int i = 0; i < 8; i++) {
            if (children[i] != nullptr) {
                children[i]->print(depth + 1);
            }
        }
    }
}

double Octree::get_node_h_for_point(const Point &p) const {
    if (p.x < bottomLeft.x || p.x > bottomLeft.x + h ||
        p.y < bottomLeft.y || p.y > bottomLeft.y + h ||
        p.z < bottomLeft.z || p.z > bottomLeft.z + h)
        return -1.0;

    if (isLeaf) {
        for (int i = 0; i < nPoints; i++) {
            if (p.x == points[i].x && p.y == points[i].y && p.z == points[i].z)
                return h;
        }
        return -1.0;
    }

    double midX = bottomLeft.x + h / 2.0;
    double midY = bottomLeft.y + h / 2.0;
    double midZ = bottomLeft.z + h / 2.0;

    int nextOctant = -1;
    if (p.x < midX) {
        if (p.y < midY) nextOctant = (p.z < midZ) ? FLB : FLT;
        else nextOctant = (p.z < midZ) ? FRB : FRT;
    } else {
        if (p.y < midY) nextOctant = (p.z < midZ) ? BLB : BLT;
        else nextOctant = (p.z < midZ) ? BRB : BRT;
    }

    if (children[nextOctant] == nullptr) return -1.0;
    return children[nextOctant]->get_node_h_for_point(p);
}

bool Octree::get_node_bottom_left_for_point(const Point &p, Point &outBL) const {
    if (p.x < bottomLeft.x || p.x > bottomLeft.x + h ||
        p.y < bottomLeft.y || p.y > bottomLeft.y + h ||
        p.z < bottomLeft.z || p.z > bottomLeft.z + h)
        return false;

    if (isLeaf) {
        for (int i = 0; i < nPoints; i++) {
            if (p.x == points[i].x && p.y == points[i].y && p.z == points[i].z) {
                outBL = bottomLeft;
                return true;
            }
        }
        return false;
    }

    double midX = bottomLeft.x + h / 2.0;
    double midY = bottomLeft.y + h / 2.0;
    double midZ = bottomLeft.z + h / 2.0;

    int nextOctant = -1;
    if (p.x < midX) {
        if (p.y < midY) nextOctant = (p.z < midZ) ? FLB : FLT;
        else nextOctant = (p.z < midZ) ? FRB : FRT;
    } else {
        if (p.y < midY) nextOctant = (p.z < midZ) ? BLB : BLT;
        else nextOctant = (p.z < midZ) ? BRB : BRT;
    }

    if (children[nextOctant] == nullptr) return false;
    return children[nextOctant]->get_node_bottom_left_for_point(p, outBL);
}

// NUEVO: Método público principal para la generación del archivo OBJ
void Octree::export_obj(const std::string &filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error al abrir el archivo " << filename << std::endl;
        return;
    }

    int vertexOffset = 1;
    export_obj_aux(file, vertexOffset);
    file.close();
}