#include <iostream>
#include <vector>
#include "octree.hpp"

int main() {
    Point rootBottomLeft(0, 0, 0);
    double N = 100.0;
    Octree tree(rootBottomLeft, N);

    std::cout << "--- DATOS DEL NODO RAIZ ---" << std::endl;
    std::cout << "Medida del lado h de la raiz: " << tree.get_h() << std::endl;
    std::cout << "Coordenada bottomLeft de la raiz: (" 
              << tree.get_bottom_left().x << ", " 
              << tree.get_bottom_left().y << ", " 
              << tree.get_bottom_left().z << ")\n" << std::endl;

    
    
    std::vector<Point> testPoints = {
        Point(5, 5, 5),       
        Point(12, 15, 10),    
        Point(15, 10, 8),     
        Point(20, 20, 20),    
        Point(45, 45, 45),    
        Point(80, 80, 80),    
        Point(90, 90, 90),    
        Point(5, 80, 5)       
    };

    std::cout << "--- INSERTANDO PUNTOS ---" << std::endl;
    for (const auto &p : testPoints) {
        std::cout << "Insertando: (" << p.x << ", " << p.y << ", " << p.z << ")... ";
        tree.insert(p);
        std::cout << "OK" << std::endl;
    }
    std::cout << std::endl;

    
    std::cout << "--- ESTRUCTURA DEL OCTREE ---" << std::endl;
    tree.print();
    std::cout << std::endl;

    
    Point A(10, 10, 10); 
    double radius = 25.0;
    bool found = false;

    std::cout << "Buscando punto mas cercano a A(10, 10, 10) con radio " << radius << "..." << std::endl;
    Point X = tree.find_closest(A, radius, found);

    std::cout << "\n--- MAS CERCANO ---" << std::endl;
    if (found) {
        std::cout << "Punto X: (" << X.x << ", " << X.y << ", " << X.z << ")" << std::endl;
        std::cout << "Lado h del nodo correspondiente a X: " << tree.get_node_h_for_point(X) << std::endl;
    } else {
        std::cout << "Punto X: NULL" << std::endl;
        std::cout << "Lado h del nodo correspondiente a X: N/A" << std::endl;
    }

    return 0;
}