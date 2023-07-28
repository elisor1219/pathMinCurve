import numpy as np

from pythonUtilities.distanceCalc import lengthOfVector


def costFunction(
    theta: np.ndarray, coneMatrix: np.ndarray, safetyMargin: float
) -> float:
    """
    This is the cost function that is minimized in the optimization.
    """

    # Calculate the total curvature in squared form
    curvature = curvatureOverPath(theta, coneMatrix, safetyMargin)
    cost = np.sum(curvature**2)

    return cost


def curvatureOverPath(
    theta: np.ndarray, coneMatrix: np.ndarray, safetyMargin: float
) -> np.ndarray:
    """
    Calculates the curvature of the track at each point.
    """

    # Calculate the midpoint of each pair of cones
    midPointMatrix = calculateMidpoints(theta, coneMatrix, safetyMargin)

    # Calculate the curvature
    curvature = calculateCurvatures(midPointMatrix)

    return curvature


def calculateMidpoints(
    theta: np.ndarray, coneMatrix: np.ndarray, safetyMargin: float
) -> np.ndarray:
    """
    Calculates the midpoint of the track at each point.
    """
    numberOfMiddlePoints = len(coneMatrix)

    # Calculate the midpoint of each pair of cones
    midPointMatrix = np.zeros((numberOfMiddlePoints, 2))
    for i in range(numberOfMiddlePoints):
        midPointMatrix[i] = calculateMidpoint(
            coneMatrix[i, 0:2], coneMatrix[i, 2:4], theta[i], safetyMargin
        )

    return midPointMatrix


def calculateMidpoint(
    P1: np.ndarray, P2: np.ndarray, theta: float, d: float
) -> np.ndarray:
    """
    P1 and P2 are the two points, theta is a number between -1 and 1, d is the safety distance.
    Return the midpoint between P1 and P2 shifted by theta but not closer than d
    """
    firstTerm = ((P1 - P2) / lengthOfVector(P1 - P2)) * d * theta
    secondTerm = (P1 * (1 - theta) + P2 * (theta + 1)) / 2
    return firstTerm + secondTerm


def calculateCurvatures(midPointMatrix: np.ndarray) -> np.ndarray:
    """
    Calculates the curvature of the track at each point, similar to curvatureOverPath,
    but takes the midPointMatrix as input instead of calculating it.
    """
    numberOfMiddlePoints = len(midPointMatrix)
    curvature = np.zeros(numberOfMiddlePoints)
    for i in range(1, numberOfMiddlePoints - 1):
        curvature[i] = calculateCurvature(
            midPointMatrix[i - 1], midPointMatrix[i], midPointMatrix[i + 1]
        )

    return curvature


def calculateCurvature(P1: np.ndarray, P2: np.ndarray, P3: np.ndarray) -> float:
    """
    Calculate the curvature at P2
    """

    nominator = 4 * areaOfPoints(P1, P2, P3)
    denominator = (
        lengthOfVector(P1 - P2) * lengthOfVector(P2 - P3) * lengthOfVector(P3 - P1)
    )
    return nominator / denominator


def areaOfPoints(P1: np.ndarray, P2: np.ndarray, P3: np.ndarray) -> float:
    """
    Calculate the area of the triangle with the corners P1, P2 and P3
    """
    return (
        abs(
            (P1[0] * (P2[1] - P3[1]))
            + (P2[0] * (P3[1] - P1[1]))
            + (P3[0] * (P1[1] - P2[1]))
        )
        / 2
    )
