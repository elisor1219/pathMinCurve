import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

from pythonUtilities.importMap import importMap
from pythonUtilities.distanceCalc import checkDistance
from pythonUtilities.costFunction import costFunction
from pythonUtilities.costFunction import curvatureOverPath
from pythonUtilities.costFunction import calculateMidpoints


def main():
    # Parameters
    safetyMargin = 1.60  # Safety margin (Current car track width is 1.25 m)
    fileName = "test"  # Name of the track in the 'tracks' folder
    optimize = True  # Optimize the curvature

    # Import the track
    pathToFile = "tracks/" + fileName + ".csv"
    coneMatrix = importMap(pathToFile)
    numberOfMiddlePoints = len(coneMatrix)
    print("Cones loaded")  # Status update

    # Check that all the distances between cones are greater than 2*safetyMargin
    conesToClose = checkDistance(coneMatrix, safetyMargin)
    if conesToClose.size != 0:
        # TODO: This is easier to fix in the c++ code
        print("Cones are too close to each other! TODO: Fix this")
    else:
        print("All cones are at a safe distance from each other")

    # Initialize the variables to be optimized over
    # Initial guess is all zeros
    theta = np.zeros(numberOfMiddlePoints)

    # Define the bounds
    bounds = [(-1, 1)] * numberOfMiddlePoints

    # Optimize the curvature
    if optimize:
        print("Beginning optimization")

        result = minimize(
            costFunction, theta, args=(coneMatrix, safetyMargin), bounds=bounds
        )

        if result.success:
            print("Optimization successful")
            oldTheta = theta
            theta = result.x
        else:
            print("Optimization failed")
            print(result.message)

    # Calculate the optimized curvature over the path
    curvature = np.sum(curvatureOverPath(theta, coneMatrix, safetyMargin))
    print("Total curvature: ", curvature)

    # Calculate the middle points
    midPointMatrix = calculateMidpoints(theta, coneMatrix, safetyMargin)

    # Plot the left cones
    plt.plot(coneMatrix[:, 0], coneMatrix[:, 1], "bo")
    # Plot the right cones
    plt.plot(coneMatrix[:, 2], coneMatrix[:, 3], "yo")
    # Plot the optimized path
    plt.plot(midPointMatrix[:, 0], midPointMatrix[:, 1], "r")
    plt.show()


if __name__ == "__main__":
    main()
