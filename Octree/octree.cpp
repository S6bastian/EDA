#include "octree.hpp"

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
