#include <iostream>

#include "include/curvature_minimizer.hpp"
#include "include/get_path.hpp"
#include "include/save_theta_to_file.hpp"

int main()
{
    std::string trackName = "track2"; // The name of the track to use

    // Load the cones from the csv file
    std::string trackPath = "tracks/" + trackName + ".csv";
    std::vector<std::vector<Point>> coneMatrix = get_path(trackPath);

    // An ugly reformating of the cone matrix to the pathPoint format
    std::vector<PathPoint> pathToOptimize(coneMatrix.size());
    for (size_t i = 0; i < coneMatrix.size(); i++) {
        pathToOptimize[i].coneOne = coneMatrix[i][0];
        pathToOptimize[i].coneTwo = coneMatrix[i][1];
    }


    CurvatureMinimizerParams params;
    params.verbose = false;
    params.safetyMargin = 1.6;
    params.relativeTolerance = 1e-8;
    params.degree = 2;
    params.optAlgorithm = nlopt::GN_CRS2_LM;

    CurvatureMinimizer curvatureMinimizer(params);
    auto optimizedTheta = curvatureMinimizer.optimize_path(pathToOptimize);

    save_theta_to_file(optimizedTheta, trackName, params.safetyMargin);


    return 0;
}
