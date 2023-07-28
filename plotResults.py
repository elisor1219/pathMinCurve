import argparse
import matplotlib.pyplot as plt
import os

from pythonUtilities.importMap import importMap
from pythonUtilities.importTheta import importTheta
from pythonUtilities.costFunction import calculateMidpoints
from pythonUtilities.costFunction import calculateCurvatures


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("file", help="Path to csv-file to plot", type=str)
    args = parser.parse_args()

    # Import optimized theta
    pathToFile = args.file
    optimizedTheta, safetyMargin = importTheta(pathToFile)

    # Import the track
    # Remove the folder name
    nameOfFile = pathToFile.split("/")[-1]
    # Remove the date, time and file extension
    nameOfTrack = nameOfFile.split("_")[0]

    pathToTrack = "tracks/" + nameOfTrack + ".csv"

    # Import the track
    coneMatrix = importMap(pathToTrack)

    # Calculate the optimized middle points
    optimizedMidPointMatrix = calculateMidpoints(
        optimizedTheta, coneMatrix, safetyMargin
    )

    # Calculate the non-optimized middle points
    zeroTheta = [0] * len(optimizedTheta)
    midPointMatrix = calculateMidpoints(zeroTheta, coneMatrix, safetyMargin)

    # Plot 4 different subplots
    # 1. Non-optimized middle points
    # 2. Optimized middle points
    # 3. Non-optimized and optimized middle points
    # 4. Curvature over the path

    # - - - Plot the non-optimized middle points - - -
    # Plot the left cones
    plt.subplot(2, 2, 1)
    plt.plot(coneMatrix[:, 0], coneMatrix[:, 1], "bo")

    # Plot the right cones
    plt.plot(coneMatrix[:, 2], coneMatrix[:, 3], "yo")

    # Plot the non-optimized middle points
    plt.plot(midPointMatrix[:, 0], midPointMatrix[:, 1], "g", label="Non-optimized")
    plt.legend()

    # - - - Plot the optimized middle points - - -
    plt.subplot(2, 2, 2)

    # Plot the left cones
    plt.plot(coneMatrix[:, 0], coneMatrix[:, 1], "bo")

    # Plot the right cones
    plt.plot(coneMatrix[:, 2], coneMatrix[:, 3], "yo")

    # Plot the optimized middle points
    plt.plot(
        optimizedMidPointMatrix[:, 0],
        optimizedMidPointMatrix[:, 1],
        "r",
        label="Optimized",
    )
    plt.legend()

    # - - - Plot the middle points together - - -
    plt.subplot(2, 2, 3)

    # Plot the left cones
    plt.plot(coneMatrix[:, 0], coneMatrix[:, 1], "bo")

    # Plot the right cones
    plt.plot(coneMatrix[:, 2], coneMatrix[:, 3], "yo")

    # Plot the non-optimized middle points
    plt.plot(midPointMatrix[:, 0], midPointMatrix[:, 1], "g", label="Non-optimized")

    # Plot the optimized middle points
    plt.plot(
        optimizedMidPointMatrix[:, 0],
        optimizedMidPointMatrix[:, 1],
        "r",
        label="Optimized",
    )
    plt.legend()

    # - - - Plot the curvature over the path - - -
    plt.subplot(2, 2, 4)

    # Calculate the curvature
    optimizedCurvature = calculateCurvatures(optimizedMidPointMatrix)
    curvature = calculateCurvatures(midPointMatrix)

    # Plot the curvature as two violinplots
    plt.violinplot(curvature, [1], showmeans=True)
    plt.violinplot(optimizedCurvature, [2], showmeans=True)

    # Set the labels
    plt.xticks([1, 2], ["Non-optimized", "Optimized"])
    plt.ylabel("Curvature")

    # Save the plot
    # Create a folder for the plots if it does not exist
    try:
        os.mkdir("plots")
    except FileExistsError:
        pass

    plt.tight_layout()

    # Before saving, make the plot larger
    plt.gcf().set_size_inches(18.5, 10.5)

    nameOfFile = nameOfFile.split(".")[0]

    plt.savefig("plots/" + nameOfFile + ".svg")

    # Show the plot
    plt.show()


if __name__ == "__main__":
    main()
