#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <limits>
#include <iomanip>
#include "octree.hpp"

std::string formatDoubleWithComma(double value) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(3) << value;
    std::string str = ss.str();
    size_t decimalPointPos = str.find('.');
    if (decimalPointPos != std::string::npos) {
        str[decimalPointPos] = ',';
    }
    return str;
}

std::vector<Point> loadPointsFromFile(const std::string& filePath, Point& outBottomLeft, double& outH) {
    std::ifstream file(filePath);
    std::vector<Point> points;
    
    if (!file.is_open()) {
        std::cerr << "Error: No se pudo abrir el archivo " << filePath << std::endl;
        return points;
    }
    
    std::string line;
    double minX = std::numeric_limits<double>::max();
    double minY = std::numeric_limits<double>::max();
    double minZ = std::numeric_limits<double>::max();
    
    double maxX = std::numeric_limits<double>::lowest();
    double maxY = std::numeric_limits<double>::lowest();
    double maxZ = std::numeric_limits<double>::lowest();
    
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        
        std::replace(line.begin(), line.end(), ',', ' ');
        std::stringstream ss(line);
        double x, y, z;
        
        if (ss >> x >> y >> z) {
            points.emplace_back(x, y, z);
            
            minX = std::min(minX, x);
            minY = std::min(minY, y);
            minZ = std::min(minZ, z);
            
            maxX = std::max(maxX, x);
            maxY = std::max(maxY, y);
            maxZ = std::max(maxZ, z);
        }
    }
    
    file.close();
    
    if (!points.empty()) {
        outBottomLeft = Point(minX, minY, minZ);
        
        double rangeX = maxX - minX;
        double rangeY = maxY - minY;
        double rangeZ = maxZ - minZ;
        
        outH = std::max({rangeX, rangeY, rangeZ}) + 0.001;
    }
    
    return points;
}

// NUEVO: Exporta la nube de puntos a un archivo .obj separado
void exportPointsToOBJ(const std::vector<Point>& points, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error al crear archivo de puntos OBJ: " << filename << std::endl;
        return;
    }
    for (const auto& p : points) {
        file << "v " << p.x << " " << p.y << " " << p.z << "\n";
    }
    file.close();
}

int main() {
    Point rootBottomLeft;
    double h = 0.0;
    
    std::cout << "--- CARGANDO PUNTOS DESDE ARCHIVO ---" << std::endl;
    std::vector<Point> points = loadPointsFromFile("aguila.xyz", rootBottomLeft, h);
    
    if (points.empty()) {
        std::cerr << "Error: No se encontraron puntos validos." << std::endl;
        return 1;
    }
    
    // NUEVO: Guardar la nube de puntos del águila
    exportPointsToOBJ(points, "aguila_puntos.obj");
    std::cout << "Se exportaron los puntos a 'aguila_puntos.obj'." << std::endl;

    // NUEVO: Exportar archivos .obj para distintas capacidades por nodo
    std::vector<int> capacidades = {1, 5, 10, 20, 50, 100};
    for (int cap : capacidades) {
        Octree tree(rootBottomLeft, h, cap);
        for (const auto& p : points) {
            tree.insert(p);
        }
        
        std::string filename = "octree_cap" + std::to_string(cap) + ".obj";
        tree.export_obj(filename);
        std::cout << "Generado: " << filename << " (Capacidad por nodo: " << cap << ")" << std::endl;
    }

    return 0;
}