import csv
import numpy as np
import string
from typing import Union


def importTheta(fileName: string) -> Union[np.ndarray, float]:
    """
    Imports the optimized theta from a csv file and the safety margin.
    """
    theta = np.zeros(0)
    safetyMargin = 0

    with open(fileName, newline="") as csvfile:
        coneReader = csv.reader(csvfile, delimiter=",", quotechar="|")
        # Skip the first line
        next(coneReader)
        for row in coneReader:
            if row[0].startswith("#"):
                continue
            theta = np.append(theta, float(row[0]))
            safetyMargin = float(row[1])

    return theta, safetyMargin
