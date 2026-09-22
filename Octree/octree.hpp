#ifndef OCTREE_H
#define OCTREE_H


//   z x
//   |/
//   ----y


struct Point{
   double x;
   double y;
   double z;

   Point() : x(-1), y(-1), z(-1) {};
   Point(double a, double b, double c) : x(a), y(b), z(c) {}

   Point& operator=(const Point &other){
      if(this != &other){
         this->x = other.x;
         this->y = other.y;
         this->z = other.z;
      }
      return *this;
   }
};


class Octree {
private:
   Octree *children[8];
   Point *points; // std::vector<Point> points
   
   // bottomLeft y h definen el espacio(cubo más grande)
   int capacity;
   bool isLeaf;
   Point bottomLeft;
   double h;

   int nPoints; // puntos ingresados.


public:
   Octree(Point, double);
   ~Octree();
   bool exist(const Point &);
   void insert(const Point &);
   Point find_closest(const Point &, int radius);

   enum Octant{
      FLT = 0, // FrontLeftTop 0
      FRT,     // FrontRightTop 1
      FRB,     // FrontRightBottom 2
      FLB,     // FrontLeftBottom 3
      BLT,     // BackLeftTop 4
      BRT,     // BackRightTop 5
      BRB,     // BackRightBottom 6
      BLB      // BackLeftBottom 7
   };
};

#endif
