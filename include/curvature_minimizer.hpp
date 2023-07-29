#ifndef CURVATURE_MINIMIZER_HPP
#define CURVATURE_MINIMIZER_HPP

#include <nlopt.hpp>
#include <vector>

#include "include/point.hpp"


struct CurvatureMinimizerParams
{
    // Parameters
    bool verbose; // Print debug messages
    double safetyMargin; // Safety margin to the cones
    double relativeTolerance; // Relative tolerance for the optimization
    nlopt::algorithm optAlgorithm; // The algorithm to use


    CurvatureMinimizerParams()
      : verbose{false},
        safetyMargin{1.6},
        relativeTolerance{1e-7},
        optAlgorithm{nlopt::LN_BOBYQA}
    {
    }
};

// The additional data to pass to the optimization function
struct OptimizationData
{
    const std::vector<std::vector<Point>> & coneMatrix;
    double safetyMargin;
};

class CurvatureMinimizer
{
public:
    // Constructor
    explicit CurvatureMinimizer(
        const CurvatureMinimizerParams params = CurvatureMinimizerParams());

    std::vector<double> optimize_path(const std::vector<PathPoint> & path) const;

private:
    // Functions

    static double objective_function(
        const std::vector<double> & theta, std::vector<double> & grad,
        void * my_func_data);

    static double cost_function(
        const std::vector<double> & theta,
        const std::vector<std::vector<Point>> & coneMatrix,
        const double safetyMargin);

    static double sum_of_curvature(
        const std::vector<double> & theta,
        const std::vector<std::vector<Point>> & coneMatrix,
        const double safetyMargin);

    static std::vector<double> calculate_curvature_over_path(
        const std::vector<double> & theta,
        const std::vector<std::vector<Point>> & coneMatrix,
        const double safetyMargin);

    static Point calculate_middle_point(
        const Point & coneOne, const Point & coneTwo, const double theta,
        const double safetyMargin);

    static double calculate_curvature(
        const Point & midPoint1, const Point & midPoint2, const Point & midPoint3);

    // Helpers ish

    static inline double length_of_vector(const Point & point);

    static inline double area_of_triangle(
        const Point & point1, const Point & point2, const Point & point3);

    // Variables
    CurvatureMinimizerParams params_;


};


#endif // CURVATURE_MINIMIZER_HPP
