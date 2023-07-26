import numpy as np
import matplotlib.pyplot as plt
import csv
from scipy.optimize import minimize


def main():
    safetyMargin = 1.60  # Safety margin (Current car track width is 1.25 m)
    importMap = True  # Import a map of cones or build one
    fileName = "test2.csv"
    optimize = True  # Optimize the curvature

    if importMap:
        # Import a map of cones or build one
        plt.grid()
        ax = plt.gca()
        ax.set_aspect("equal", adjustable="box")

        trueConesPos = np.zeros((0, 2))

        with open(fileName, newline="") as csvfile:
            coneReader = csv.reader(csvfile, delimiter=",", quotechar="|")
            for row in coneReader:
                if row[0].startswith("#"):
                    continue
                xCone1 = float(row[0])
                yCone1 = float(row[1])
                xCone2 = float(row[2])
                yCone2 = float(row[3])
                trueConesPos = np.append(
                    trueConesPos, np.array([[xCone1, yCone1], [xCone2, yCone2]]), axis=0
                )

        conesPos = trueConesPos

        # Convert to numpy array
        conesPos = np.array(conesPos)
        coneMatrix = conesPos
        n = len(coneMatrix)

        # Plot the cones
        plt.plot([x[0] for x in conesPos], [x[1] for x in conesPos], "bo")
    else:
        # Set axes to -10, 10
        plt.axis([-10, 10, -10, 10])
        coneMatrix = plt.ginput(-1)
        n = len(coneMatrix)
        if (n % 2) != 0:
            print("Number of cones must be even, removing last cone")
            coneMatrix = np.delete(coneMatrix, n - 1, 0)
            n = len(coneMatrix)
        # Rezise the matrix to nx2
        coneMatrix = np.resize(coneMatrix, (n, 2))

    print("Cones loaded")

    # Check that all the distances between cones are greater than 2*safetyMargin
    j = 0
    for i in range(0, int(n / 2)):
        if lengthOfVector(coneMatrix[j] - coneMatrix[j + 1]) < 2 * safetyMargin:
            print("Distance between cones too small, exiting")
            # exit()
        j += 2

    print("Cones checked")

    # theta vector (optimized later)
    theta = np.zeros(int(n / 2))

    # Define the bounds for theta (between -1 and 1)
    bounds = [(-1, 1)] * len(theta)

    # Optimization to minimize the curvature
    if optimize:
        result = minimize(
            totalCurvatureObjective,
            theta,
            args=(coneMatrix, safetyMargin),
            bounds=bounds,
        )
        if result.success:
            optimized_theta = result.x
            print("Optimized theta:", optimized_theta)
            theta = optimized_theta
        else:
            print("Optimization failed")
            print(result.message)

    # Calculate the midpoint of each pair of cones
    midPointMatrix = np.zeros((int(n / 2), 2))
    j = 0
    for i in range(int(n / 2)):
        midPointMatrix[i] = getMidpoint(
            coneMatrix[j], coneMatrix[j + 1], theta[i], safetyMargin
        )
        j += 2

    # Plot the cones
    plt.plot(coneMatrix[:, 0], coneMatrix[:, 1], "ro")
    plt.plot(midPointMatrix[:, 0], midPointMatrix[:, 1])

    # Calculate the curvature
    curvature = np.zeros(int(n / 2))
    for i in range(1, int(n / 2) - 1):
        curvature[i] = getCurvature(
            midPointMatrix[i - 1], midPointMatrix[i], midPointMatrix[i + 1]
        )

    # Calculate the total curvature
    totalCurvature = np.sum(curvature)

    print("Total curvature: ", totalCurvature)
    plt.show()


def totalCurvatureObjective(theta, coneMatrix, safetyMargin):
    # Calculate the midpoint of each pair of cones
    midPointMatrix = np.zeros((len(theta), 2))
    j = 0
    for i in range(len(theta)):
        midPointMatrix[i] = getMidpoint(
            coneMatrix[j], coneMatrix[j + 1], theta[i], safetyMargin
        )
        j += 2

    # Calculate the curvature
    curvature = np.zeros(len(theta))
    for i in range(1, len(theta) - 1):
        curvature[i] = getCurvature(
            midPointMatrix[i - 1], midPointMatrix[i], midPointMatrix[i + 1]
        )

    # Calculate the total curvature
    costFun = np.sum(curvature**2)

    return costFun


# P1 and P2 are the two points
# v is a number between -1 and 1
# d is the safety margin
def getMidpoint(P1, P2, v, d):
    firstTerm = ((P1 - P2) / lengthOfVector(P1 - P2)) * d * v
    secondTerm = (P1 * (1 - v) + P2 * (v + 1)) / 2
    return firstTerm + secondTerm


def lengthOfVector(vector):
    return np.sqrt(vector[0] ** 2 + vector[1] ** 2)


def getCurvature(M1, M2, M3):
    return (
        4
        * areaOfPoints(M1, M2, M3)
        / (lengthOfVector(M1 - M2) * lengthOfVector(M2 - M3) * lengthOfVector(M3 - M1))
    )


def areaOfPoints(P1, P2, P3):
    return (
        abs(
            (P1[0] * (P2[1] - P3[1]))
            + (P2[0] * (P3[1] - P1[1]))
            + (P3[0] * (P1[1] - P2[1]))
        )
        / 2
    )


if __name__ == "__main__":
    main()
