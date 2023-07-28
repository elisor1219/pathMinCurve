import numpy as np


def lengthOfVector(vector: np.ndarray) -> float:
    """
    Calculates the length of a vector
    """
    return np.sqrt(vector[0] ** 2 + vector[1] ** 2)


def checkDistance(conePos: np.ndarray, safetyDistance: float) -> np.ndarray:
    """
    This function checks so that all cones in the track are at least 2*safetyDistance apart
    Returns the index of the pairs of cones that are too close. If the return value is an empty,
    then all cones are at least 2*safetyDistance apart.
    """

    twoSafetyDistance = 2 * safetyDistance
    conesTooClose = np.zeros(0, dtype=int)

    # Go over all the middle points
    for index, cones in enumerate(conePos):
        cone1 = cones[0:2]
        cone2 = cones[2:4]
        if lengthOfVector(cone1 - cone2) > 2 * twoSafetyDistance:
            conesTooClose = np.append(conesTooClose, index)

    return conesTooClose
