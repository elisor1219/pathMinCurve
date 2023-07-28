import csv
import numpy as np
import string


def importMap(fileName: string) -> np.ndarray:
    """
    Imports a map of cones from a csv file.
    The format of the return value is a numpy array of shape (n, 4) where n is the number of
    middle points and 4 is the number of coordinates (x1, y1, x2, y2).
    """
    trueConesPos = np.zeros((0, 4))

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
                trueConesPos, np.array([[xCone1, yCone1, xCone2, yCone2]]), axis=0
            )

    # Convert to numpy array
    coneMatrix = np.array(trueConesPos)

    return coneMatrix
