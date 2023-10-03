#include <iostream>
#include <chrono>
#include <iomanip>

#include "include/curvature_minimizer.hpp"

/*
d8888b. db    db d8888b. db      d888888b  .o88b.
88  `8D 88    88 88  `8D 88        `88'   d8P  Y8
88oodD' 88    88 88oooY' 88         88    8P
88~~~   88    88 88~~~b. 88         88    8b
88      88b  d88 88   8D 88booo.   .88.   Y8b  d8
88      ~Y8888P' Y8888P' Y88888P Y888888P  `Y88P'
*/

CurvatureMinimizer::CurvatureMinimizer(const CurvatureMinimizerParams params)
  : params_(params)
{
}

std::vector<double> CurvatureMinimizer::optimize_path(const std::vector<PathPoint> & path)
const
{
    // Reformat the path into a cone matrix
    std::vector<std::vector<Point>> coneMatrix(path.size());
    for (size_t i = 0; i < path.size(); i++) {
        coneMatrix[i].push_back(path[i].coneOne);
        coneMatrix[i].push_back(path[i].coneTwo);
    }
    int numberOfOptimizationVariables = coneMatrix.size();

    // TODO: Check the distance between the cones

    // Initialize the optimization object
    nlopt::opt opt(params_.optAlgorithm, numberOfOptimizationVariables);
    //TODO: Make so we can remove optimization variables

    // Initialize the optimization data
    OptimizationData optiData = {coneMatrix, params_.safetyMargin, params_.degree};

    if (params_.verbose) {
        std::cout << "Cones: " << std::endl;
        for (auto cones : optiData.coneMatrix) {
            std::cout << "  (" << cones[0].x << ", " << cones[0].y << ") \t";
            std::cout << "  (" << cones[1].x << ", " << cones[1].y << ") ";
            std::cout << std::endl;
        }
    }

    // Set the lower bounds, to -1
    std::vector<double> lowerBound(numberOfOptimizationVariables, -1);
    opt.set_lower_bounds(lowerBound);

    // Set the upper bounds, to 1
    std::vector<double> upperBound(numberOfOptimizationVariables, 1);
    opt.set_upper_bounds(upperBound);

    // Set the objective function
    opt.set_min_objective(objective_function, &optiData);

    // Set the relative tolerance
    opt.set_ftol_rel(params_.relativeTolerance);

    // Initialize the optimization variables (Guess)
    std::vector<double> theta(numberOfOptimizationVariables, 0);
    //TODO: Make a function where you can set the guess

    std::cout << "\nStarting optimization" << std::endl;

    double minf;
    double curvatureBefore = sum_of_curvature(theta, optiData.coneMatrix, optiData.safetyMargin);

    try {
        auto start = std::chrono::high_resolution_clock::now();

        nlopt::result result = opt.optimize(theta, minf);

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;

        std::cout << "Optimization finished!!\n" << std::endl;

        if (params_.verbose) {
            std::cout << "Found minimum at : ";
            for (double t : theta) {
                std::cout << t << ", ";
            }
            std::cout << "\n" << std::endl;
        }

        std::cout << "Elapsed time: " << elapsed.count() << " s" << std::endl;
        std::cout << "Number of evaluations: " << opt.get_numevals() << std::endl;

        std::cout << "Obj func value " << std::setprecision(5) << minf << "\n" << std::endl;

        double curvatureAfter = sum_of_curvature(theta, coneMatrix, optiData.safetyMargin);

        std::cout << "Curvature before: " << curvatureBefore << std::endl;
        std::cout << "Curvature after: " << curvatureAfter << std::endl;
        std::cout << "Curvature difference: " << curvatureBefore - curvatureAfter << std::endl;
    } catch (std::exception & e) {
        std::cout << "nlopt failed: " << e.what() << std::endl;
        std::cout << "Exiting..." << std::endl;
        exit(1);
    }


    return theta;
}

/*
d8888b. d8888b. d888888b db    db  .d8b.  d888888b d88888b
88  `8D 88  `8D   `88'   88    88 d8' `8b `~~88~~' 88'
88oodD' 88oobY'    88    Y8    8P 88ooo88    88    88ooooo
88~~~   88`8b      88    `8b  d8' 88~~~88    88    88~~~~~
88      88 `88.   .88.    `8bd8'  88   88    88    88.
88      88   YD Y888888P    YP    YP   YP    YP    Y88888P
*/

double CurvatureMinimizer::objective_function(
    const std::vector<double> & theta, std::vector<double> & grad, void * optiData)
{
    if (!grad.empty()) {
        std::cout << "ERROR: Gradient not possible" << std::endl;
    }

    // Cast optiData to the OptimizationData pointer
    OptimizationData * data = static_cast<OptimizationData *>(optiData);

    // Access the coneMatrix and safetyMargin from the data struct
    const std::vector<std::vector<Point>> & coneMatrix = data->coneMatrix;
    double safetyMargin = data->safetyMargin;
    double degree = data->degree;

    return cost_function(theta, coneMatrix, safetyMargin, degree);
}

double CurvatureMinimizer::cost_function(
    const std::vector<double> & theta,
    const std::vector<std::vector<Point>> & coneMatrix,
    const double safetyMargin,
    const double degree)
{
    // Calculate the curvature over the path
    std::vector<double> curvature = calculate_curvature_over_path(theta, coneMatrix, safetyMargin);

    // Calculate the cost as the sum of the squared curvature
    double cost = 0.0;
    for (size_t i = 0; i < curvature.size(); i++) {
        cost += pow(curvature[i], degree);
    }

    return cost;
}

double CurvatureMinimizer::sum_of_curvature(
    const std::vector<double> & theta,
    const std::vector<std::vector<Point>> & coneMatrix,
    const double safetyMargin)
{
    // Calculate the curvature over the path
    std::vector<double> curvature = calculate_curvature_over_path(theta, coneMatrix, safetyMargin);

    // Sum the curvature
    double sum_curve = 0.0;
    for (size_t i = 0; i < curvature.size(); i++) {
        sum_curve += curvature[i];
    }

    return sum_curve;
}

std::vector<double> CurvatureMinimizer::calculate_curvature_over_path(
    const std::vector<double> & theta,
    const std::vector<std::vector<Point>> & coneMatrix,
    const double safetyMargin)
{


    //TODO: This can be parallelized
    // Instead of calculating and saving the midpoints we can calculate the
    // curvature directly and save that

    // Calculate the midpoint of each pair of cones
    std::vector<Point> midPointMatrix(coneMatrix.size());

    for (size_t i = 0; i < theta.size(); ++i) {
        midPointMatrix[i] = calculate_middle_point(
            coneMatrix[i][0], coneMatrix[i][1], theta[i],
            safetyMargin);
    }

    // Calculate the curvature
    std::vector<double> curvature(theta.size() - 2);

    for (size_t i = 1; i < theta.size() - 1; ++i) {
        curvature[i - 1] =
          calculate_curvature(midPointMatrix[i - 1], midPointMatrix[i], midPointMatrix[i + 1]);
    }

    return curvature;

}

/**
 * @brief Calculates the 'midpoint' of two cones where the point can be shifted
 *        to the left or right of the line connecting the two cones.
 * @param coneOne The first cone
 * @param coneTwo The second cone
 * @param theta The shift of the point from the line connecting the two cones.
 *          Between -1 and 1
 * @param safetyMargin The safety margin one of the cones to the point
 * @return The 'midpoint' of the two cones
*/
Point CurvatureMinimizer::calculate_middle_point(
    const Point & coneOne, const Point & coneTwo, const double theta,
    const double safetyMargin)
{
    // Calculate the left term
    Point nominator = coneOne - coneTwo;
    double denominator = length_of_vector(coneOne - coneTwo);

    Point leftTerm = (nominator / denominator) * safetyMargin * theta;

    // Calculate the right term
    Point nominatorTwo = coneOne * (1 - theta) + coneTwo * (theta + 1);
    double denominatorTwo = 2;

    Point rightTerm = (nominatorTwo / denominatorTwo);

    return leftTerm + rightTerm;
}

double CurvatureMinimizer::calculate_curvature(
    const Point & midPoint1, const Point & midPoint2,
    const Point & midPoint3)
{
    return 4 *
           area_of_triangle(
        midPoint1, midPoint2,
        midPoint3) /
           (length_of_vector(midPoint1 - midPoint2) * length_of_vector(midPoint2 - midPoint3) *
           length_of_vector(midPoint3 - midPoint1));
}


inline double CurvatureMinimizer::length_of_vector(const Point & point)
{
    return std::sqrt(point.x * point.x + point.y * point.y);
}

inline double CurvatureMinimizer::area_of_triangle(
    const Point & point1, const Point & point2,
    const Point & point3)
{
    return 0.5 * std::abs(
        point1.x * (point2.y - point3.y) +
        point2.x * (point3.y - point1.y) +
        point3.x * (point1.y - point2.y));
}
