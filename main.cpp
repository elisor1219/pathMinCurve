#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <array>
#include <nlopt.hpp>
#include <cmath>
#include <cmath>
#include <iomanip>
#include <chrono>

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
    return std::abs((P1.x * (P2.y - P3.y)) + (P2.x * (P3.y - P1.y)) + (P3.x * (P1.y - P2.y))) / 2;
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
        curvature[i - 1] =
          getCurvature(midPointMatrix[i - 1], midPointMatrix[i], midPointMatrix[i + 1]);
    }

    // Calculate the cost function
    double costFun = 0.0;
    for (size_t i = 0; i < curvature.size(); ++i) {
        costFun += curvature[i] * curvature[i];
    }

    return costFun;
}

double totalCurvature(
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
        curvature[i - 1] =
          getCurvature(midPointMatrix[i - 1], midPointMatrix[i], midPointMatrix[i + 1]);
    }

    // Calculate the cost function
    double costFun = 0.0;
    for (size_t i = 0; i < curvature.size(); ++i) {
        costFun += curvature[i];
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

int main()
{
    // Load the cones from the csv file
    std::vector<std::vector<Point>> coneMatrix = get_path("test3.csv");
    int numberOfOptimizationVariables = coneMatrix.size();
    std::cout << "Number of cones: " << coneMatrix.size() << std::endl;


    // Initialize the optimization data
    OptimizationData optiData = {coneMatrix, 1.60};
    std::cout << "Optimization data initialized" << std::endl;
    std::cout << "Safety margin: " << optiData.safetyMargin << std::endl;

    std::cout << "Cones: " << std::endl;
    for (auto cones : optiData.coneMatrix) {
        std::cout << "  (" << cones[0].x << ", " << cones[0].y << ") \t";
        std::cout << "  (" << cones[1].x << ", " << cones[1].y << ") ";
        std::cout << std::endl;
    }

////////////////////////////////////////////////////////////////////////
    /* Tested algorithms 1
    Seting the stopval to 0.099357 and testing on test.csv
    Local derivative free algorithms
    LN_COBYLA - 44.1387 s
    LN_BOBYQA - 0.45278 s
    LN_NEWUOA - roundoff error
    LN_NEWUOA_BOUND - To long...
    LN_PRAXIS - did not converge to the right value, but extremely fast
    LN_NELDERMEAD - 0.145483 s
    LN_SBPLX - 1.02695 s

    Global derivative free algorithms
    GN_CRS2_LM - 0.945763 s
    GN_DIRECT (all varaitions) - To long...
    GN_ORIG_DIRECT_L - did not converge to the right value
    GN_ESCH - To long...
    GN_ISRES - To long...
    GN_MLSL - Core dumped
    GN_MLSL_LDS - Core dumped
    */
////////////////////////////////////////////////////////////////////////
    /* Summary
    - LN_PRAXIS is lightning fast, and could work, but will seldom converge to the "right value".
      It does not listen to the stopval. This is more useful to smooth the path, but keep it as
      close to the original as possible.
    - GN_CRS2_LM is a global derivative free algorithm that is very fast if you use opt.set_ftol_rel(1e-7)
        and opt.set_ftol_abs(1e-7). It is also very accurate.
    - LN_BOBYQA is a fast and accurate local derivative free algorithm. It also works very well with
        opt.set_ftol_rel(1e-7).
    */

    // Initialize the optimization object
    nlopt::opt opt(nlopt::LN_BOBYQA, numberOfOptimizationVariables);
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

    // Set the tolerance
    //opt.set_xtol_rel(1e-10);
    //opt.set_ftol_abs(1e-10);
    opt.set_ftol_rel(1e-7);

    //opt.set_stopval(0.099357);

    // Initialize the optimization variables (Guess)
    std::vector<double> theta(numberOfOptimizationVariables);
    for (size_t i = 0; i < theta.size(); ++i) {
        theta[i] = 0;
    }

    double minf;

    double curvatureBefore = totalCurvature(theta, coneMatrix, optiData.safetyMargin);

    try {
        std::cout << "Starting optimization" << std::endl;
        auto start = std::chrono::high_resolution_clock::now();

        nlopt::result result = opt.optimize(theta, minf);

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;

        std::cout << "Optimization finished" << std::endl;
        std::cout << "Found minimum at : ";
        for (double t : theta) {
            std::cout << t << ", ";
        }
        std::cout << std::endl;

        std::cout << "##Elapsed time: " << elapsed.count() << " s" << std::endl;
        std::cout << "Number of evaluations: " << opt.get_numevals() << std::endl;

        std::cout << "Obj func value " << std::setprecision(5) << minf << std::endl;
    } catch (std::exception & e) {
        std::cout << "nlopt failed: " << e.what() << std::endl;
    }

    double curvatureAfter = totalCurvature(theta, coneMatrix, optiData.safetyMargin);

    std::cout << "Curvature before: " << curvatureBefore << std::endl;
    std::cout << "Curvature after: " << curvatureAfter << std::endl;

    return 0;
}
