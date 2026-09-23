#include "octree.hpp"

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
    // Poda: si la distancia mínima a la caja supera el récord actual, descartar rama
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

//###########################################################################
//###########################################################################
//                                  PUBLIC
//###########################################################################
//###########################################################################



Octree::Octree(Point bottomLeft, double h){

    for(int i = 0; i < 8; i++) children[i] = nullptr;
    points = nullptr;

    capacity = 4;
    isLeaf = true;
    this->bottomLeft = bottomLeft;
    this->h = h;

    nPoints = 0;
}

Octree::~Octree(){
    if (points != nullptr) {
        delete[] points;
    }
    for (int i = 0; i < 8; i++) {
        if (children[i] != nullptr) {
            delete children[i];
        }
    }
}


bool Octree::exist(const Point &cPoint){

    if(cPoint.x < bottomLeft.x || cPoint.x > bottomLeft.x + h ||
        cPoint.y < bottomLeft.y || cPoint.y > bottomLeft.y + h ||
        cPoint.z < bottomLeft.z || cPoint.z > bottomLeft.z + h)
        return false;

    if(isLeaf){
        for(int i = 0; i < nPoints; i++){
            if(cPoint.x == points[i].x && 
                cPoint.y == points[i].y && 
                cPoint.z == points[i].z)

                return true;
        }

        return false; 
    }

    double midX, midY, midZ;
    midX = bottomLeft.x + h/2.0;
    midY = bottomLeft.y + h/2.0;
    midZ = bottomLeft.z + h/2.0;

    int nextOctant = -1;

    if(cPoint.x < midX){
        if(cPoint.y < midY){
            if(cPoint.z < midZ){
                nextOctant = FLB; 
            }
            else{
                nextOctant = FLT;
            }
        }
        else{
            if(cPoint.z < midZ){
                nextOctant = FRB;
            }
            else{
                nextOctant = FRT;
            }
        }
    }
    else{
        if(cPoint.y < midY){
            if(cPoint.z < midZ){
                nextOctant = BLB;
            }
            else{
                nextOctant = BLT;
            }
        }
        else{
            if(cPoint.z < midZ){
                nextOctant = BRB;
            }
            else{
                nextOctant = BRT;
            }
        }
    }

    return children[nextOctant]->exist(cPoint);
}


void Octree::insert(const Point &cPoint){
    
    if(exist(cPoint)) return;

    if(cPoint.x < bottomLeft.x || cPoint.x > bottomLeft.x + h ||
        cPoint.y < bottomLeft.y || cPoint.y > bottomLeft.y + h ||
        cPoint.z < bottomLeft.z || cPoint.z > bottomLeft.z + h)
        return;

    double midX, midY, midZ;
    midX = bottomLeft.x + h/2.0;
    midY = bottomLeft.y + h/2.0;
    midZ = bottomLeft.z + h/2.0;

    int nextOctant = -1;

    if(isLeaf){
        if(nPoints < capacity){
            if(points == nullptr){
                points = new Point[capacity+1];
            }

            points[nPoints] = cPoint;
            ++nPoints;
            return;
        }


        points[capacity] = cPoint;

        double halfH = h/2;

        double varX[] = {0,0,0,0,halfH,halfH,halfH,halfH},
                varY[] = {0,halfH,halfH,0,0,halfH,halfH,0},
                varZ[] = {halfH,halfH,0,0,halfH,halfH,0,0};


        for(int i = 0; i < 8; i++){
            children[i] = new Octree(Point(bottomLeft.x + varX[i],
                                        bottomLeft.y + varY[i],
                                        bottomLeft.z + varZ[i]), halfH);
        }

        for(int i = 0; i <= capacity; i++){
            if(points[i].x < midX){
                if(points[i].y < midY){
                    if(points[i].z < midZ){
                        nextOctant = FLB; 
                    }
                    else{
                        nextOctant = FLT;
                    }
                }
                else{
                    if(points[i].z < midZ){
                        nextOctant = FRB;
                    }
                    else{
                        nextOctant = FRT;
                    }
                }
            }
            else{
                if(points[i].y < midY){
                    if(points[i].z < midZ){
                        nextOctant = BLB;
                    }
                    else{
                        nextOctant = BLT;
                    }
                }
                else{
                    if(points[i].z < midZ){
                        nextOctant = BRB;
                    }
                    else{
                        nextOctant = BRT;
                    }
                }
            }

            children[nextOctant]->insert(points[i]);
        }

        nPoints = 0;
        isLeaf = false;
        delete[] points;
        points = nullptr;
    }
    else{
        if(cPoint.x < midX){
            if(cPoint.y < midY){
                if(cPoint.z < midZ){
                    nextOctant = FLB; 
                }
                else{
                    nextOctant = FLT;
                }
            }
            else{
                if(cPoint.z < midZ){
                    nextOctant = FRB;
                }
                else{
                    nextOctant = FRT;
                }
            }
        }
        else{
            if(cPoint.y < midY){
                if(cPoint.z < midZ){
                    nextOctant = BLB;
                }
                else{
                    nextOctant = BLT;
                }
            }
            else{
                if(cPoint.z < midZ){
                    nextOctant = BRB;
                }
                else{
                    nextOctant = BRT;
                }
            }
        }

        return children[nextOctant]->insert(cPoint);
    }


    
}

Point Octree::find_closest(const Point &cPoint, double radius, bool &found){
    
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

