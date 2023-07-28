#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <array>
#include <nlopt.hpp>
#include <cmath>
#include <cmath>
#include <iomanip>

struct Point
{
    double x;
    double y;

    // + operator
    Point operator+(const Point & rhs) const
    {
        return {x + rhs.x, y + rhs.y};
    }

    // - operator
    Point operator-(const Point & rhs) const
    {
        return {x - rhs.x, y - rhs.y};
    }

    // * operator
    Point operator*(const double & rhs) const
    {
        return {x * rhs, y * rhs};
    }

    // / operator
    Point operator/(const double & rhs) const
    {
        return {x / rhs, y / rhs};
    }
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

typedef struct
{
    double a, b;
} my_constraint_data;

struct OptimizationData
{
    const std::vector<std::vector<Point>> & coneMatrix;
    double safetyMargin;
};

/*
HELPERS ------------------------
*/


inline double lengthOfVector(const std::vector<double> & vector)
{
    return sqrt(vector[0] * vector[0] + vector[1] * vector[1]);
}

inline double lengthOfVector(const Point & point)
{
    return sqrt(point.x * point.x + point.y * point.y);
}

/**
 * @brief Calculates the area of a triangle given three points
 * @param P1 The first point
 * @param P2 The second point
 * @param P3 The third point
 * @return The area of the triangle
*/
double areaOfPoints(const Point & P1, const Point & P2, const Point & P3)
{
    return abs((P1.x * (P2.y - P3.y)) + (P2.x * (P3.y - P1.y)) + (P3.x * (P1.y - P2.y))) / 2;
}

/**
 * @brief Calculates the curvature of the three points, or in other words the
 *        curvature of the second point.
 * @param M1 The first point
 * @param M2 The second point
 * @param M3 The third point
 * @return The curvature of the second point
*/
double getCurvature(const Point & M1, const Point & M2, const Point & M3)
{
    return 4 *
           areaOfPoints(
        M1, M2,
        M3) / (lengthOfVector(M1 - M2) * lengthOfVector(M2 - M3) * lengthOfVector(M3 - M1));
}


/**
 * @brief Calculates the 'midpoint' of two cones where the point can be shifted
 *        to the left or right of the line connecting the two cones.
 * @param P1 The first cone
 * @param P2 The second cone
 * @param v The shift of the point from the line connecting the two cones.
 *          Between -1 and 1
 * @param d The safety margin one of the cones to the point
 * @return The 'midpoint' of the two cones
*/
Point getMidpoint(const Point P1, const Point P2, double v, double d)
{
    // Calculate the left term
    Point nominator = P1 - P2;
    double denominator = lengthOfVector(P1 - P2);

    Point leftTerm = (nominator / denominator) * d * v;

    // Calculate the right term
    Point nominatorTwo = P1 * (1 - v) + P2 * (v + 1);
    double denominatorTwo = 2;

    Point rightTerm = (nominatorTwo / denominatorTwo);

    return leftTerm + rightTerm;
}

double totalCurvatureObjective(
    const std::vector<double> & theta,
    const std::vector<std::vector<Point>> & coneMatrix,
    double safetyMargin)
{
    // Calculate the midpoint of each pair of cones
    std::vector<Point> midPointMatrix(coneMatrix.size());

    for (size_t i = 0; i < theta.size(); ++i) {
        midPointMatrix[i] = getMidpoint(coneMatrix[i][0], coneMatrix[i][1], theta[i], safetyMargin);
    }

    // Calculate the curvature
    std::vector<double> curvature(theta.size() - 2);
    for (size_t i = 1; i < theta.size() - 1; ++i) {
        curvature[i] =
          getCurvature(midPointMatrix[i - 1], midPointMatrix[i], midPointMatrix[i + 1]);
    }

    // Calculate the cost function
    double costFun = 0.0;
    for (double curv : curvature) {
        // Take the square of the curvature
        costFun += curv * curv;
    }

    return costFun;
}

/*
OPTIMIZATION ------------------------
*/

double myvfunc(const std::vector<double> & theta, std::vector<double> & grad, void * my_func_data)
{
    if (!grad.empty()) {
        std::cout << "ERROR: Gradient not possible" << std::endl;
    }

    // Cast my_func_data to the OptimizationData pointer
    OptimizationData * data = static_cast<OptimizationData *>(my_func_data);

    // Access the coneMatrix and safetyMargin from the data struct
    const std::vector<std::vector<Point>> & coneMatrix = data->coneMatrix;
    double safetyMargin = data->safetyMargin;

    return totalCurvatureObjective(theta, coneMatrix, safetyMargin);
}

double myvconstraint(const std::vector<double> & x, std::vector<double> & grad, void * data)
{
    my_constraint_data * d = reinterpret_cast<my_constraint_data *>(data);
    double a = d->a, b = d->b;

    if (!grad.empty()) {
        std::cout << "ERROR: Gradient not possible" << std::endl;
    }

    return (a * x[0] + b) * (a * x[0] + b) * (a * x[0] + b) - x[1];
}

int main()
{
    // Load the cones from the csv file
    std::vector<std::vector<Point>> coneMatrix = get_path("test.csv");
    int numberOfOptimizationVariables = coneMatrix.size();
    std::cout << "Number of cones: " << coneMatrix.size() << std::endl;

    std::cout << areaOfPoints({0, 0}, {1, 1}, {1, 0}) << std::endl;
    exit(1);


    // Initialize the optimization data
    OptimizationData optiData = {coneMatrix, 0.5};
    std::cout << "Optimization data initialized" << std::endl;
    std::cout << "Safety margin: " << optiData.safetyMargin << std::endl;

    std::cout << "Cones: " << std::endl;
    for (auto cones : optiData.coneMatrix) {
        std::cout << "  (" << cones[0].x << ", " << cones[0].y << ") \t";
        std::cout << "  (" << cones[1].x << ", " << cones[1].y << ") ";
        std::cout << std::endl;
    }

    // Initialize the optimization object
    nlopt::opt opt(nlopt::LN_COBYLA, numberOfOptimizationVariables);
    std::cout << "Optimization object initialized" << std::endl;

    // Set the lower bounds, to -1
    std::vector<double> lowerBound(numberOfOptimizationVariables);
    for (size_t i = 0; i < lowerBound.size(); ++i) {
        lowerBound[i] = -1;
    }
    opt.set_lower_bounds(lowerBound);

    // Set the upper bounds, to 1
    std::vector<double> upperBound(numberOfOptimizationVariables);
    for (size_t i = 0; i < upperBound.size(); ++i) {
        upperBound[i] = 1;
    }
    opt.set_upper_bounds(upperBound);

    // Set the objective function
    opt.set_min_objective(myvfunc, &optiData);

    //my_constraint_data data[2] = {{2, 0}, {-1, 1}};

    //opt.add_inequality_constraint(myvconstraint, &data[0], 1e-8);
    //opt.add_inequality_constraint(myvconstraint, &data[1], 1e-8);

    opt.set_xtol_rel(1e-4);

    // Initialize the optimization variables (Guess)
    std::vector<double> theta(numberOfOptimizationVariables);
    for (size_t i = 0; i < theta.size(); ++i) {
        theta[i] = 0;
    }

    double minf;

    try {
        nlopt::result result = opt.optimize(theta, minf);
        std::cout << "Optimization finished" << std::endl;
        std::cout << "Found minimum at : ";
        for (double t : theta) {
            std::cout << t << " ";
        }
        std::cout << std::endl;

        std::cout << std::setprecision(10) << minf << std::endl;
    } catch (std::exception & e) {
        std::cout << "nlopt failed: " << e.what() << std::endl;
    }

    // Calculate the curvature of the path to compare with the optimization
    double calculatedCurvature = totalCurvatureObjective(
        theta, optiData.coneMatrix,
        optiData.safetyMargin);
    std::cout << "Calculated curvature: " << calculatedCurvature << std::endl;

    return 0;
}
