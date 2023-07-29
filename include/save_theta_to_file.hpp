#ifndef SAVE_THETA_TO_FILE_HPP
#define SAVE_THETA_TO_FILE_HPP

#include <vector>
#include <iostream>
#include <fstream>
#include <iomanip>

void save_theta_to_file(
    const std::vector<double> & theta, std::string trackName,
    double safetyMargin)
{
    // Create a folder named "results" if it does not exist
    if (system("mkdir -p results") != 0) {
        std::cerr << "Error creating folder results" << std::endl;
        exit(1);
    }
    std::string folderName = "results/";

    // Get the current time as YYYYMMDD_HHMMSS
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::stringstream ss;
    ss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    std::string time = ss.str();

    // Create the file name
    std::string fileName = folderName + trackName + "_" + time + ".csv";
    std::ofstream file(fileName);

    if (!file.is_open()) {
        std::cerr << "Error opening file " << fileName << std::endl;
        exit(1);
    }

    // Create the headers
    file << "optimized_theta" << "," << "safety_margin" << std::endl;

    for (double t : theta) {
        file << t << "," << safetyMargin << std::endl;
    }

    file.close();

    std::cout << "\nSaved optimized theta to file " << fileName << std::endl;
    std::cout << "To plot the result run the following command:";
    std::cout << "\n\tpython plotResults.py " << fileName << std::endl;
}

#endif // SAVE_THETA_TO_FILE_HPP
