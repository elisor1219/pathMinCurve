#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <array>
#include <nlopt.hpp>

struct Point
{
    double x;
    double y;
};


std::vector<std::vector<Point>> get_path(std::string fileName) 
{

    // The rows are the different paths and the columns are the two cones
    // that make up the path so a nx2 matrix
    std::vector<std::vector<Point>> coneMatrix;
    
    std::ifstream csvfile(fileName);
    
    if (!csvfile.is_open()) {
        std::cerr << "Error opening file " << fileName << std::endl;
        exit(1);
    }

    std::string line;
    while (std::getline(csvfile, line)) {
        if (line[0] == '#') {
            continue;
        }

        std::istringstream iss(line);
        std::string value;
        std::vector<Point> conePair;
        conePair.resize(2);

        for (int i = 0; i < 4; i += 2) {
            if (!std::getline(iss, value, ',')) {
                std::cerr << "Error reading data from line: " << line << std::endl;
                break;
            }

            std::istringstream converter(value);
            double x, y;
            if (!(converter >> x)) {
                std::cerr << "Error converting x-coordinate from line: " << line << std::endl;
                break;
            }

            if (!std::getline(iss, value, ',')) {
                std::cerr << "Error reading data from line: " << line << std::endl;
                break;
            }

            converter.clear();
            converter.str(value);
            if (!(converter >> y)) {
                std::cerr << "Error converting y-coordinate from line: " << line << std::endl;
                break;
            }

            Point cone = {x, y};
            conePair[i / 2] = cone;
        }

        coneMatrix.push_back(conePair);
    }

    csvfile.close();

    return coneMatrix;

}



int main() {

    auto coneMatrix = get_path("test.csv");


    return 0;
}


