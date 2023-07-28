# Curvature Minimization

This is a test on how one can minimize the curvature of a given path. 
By doing so the path becomes more smooth and becomes more similar to a racing line.

## Installation
There are two different ways to use this code.
Either with python or c++ and python.

### Python
This is the easiest way to use this code and you can see this as a proof of concept.
#### Requirements (Or what I used)
- matplotlib 3.6.3
- numpy 1.24.1
- scipy 1.11.1

#### How to use
1. Clone this repository
2. Run `python minCurvOpti.py`

After that a plot should appear with the cones and an optimized path.

### C++ and Python
This is the way I would recommend to use this code.
The optimization is done in c++ and the plotting is done in python.
#### Requirements Python (Or what I used)
- matplotlib 3.6.3
- numpy 1.24.1
#### Requirements C++ (Or what I used)
- nlopt 2.7.1

#### How to use
1. Clone this repository
2. Run `cmake .` in the root directory
3. Run `make` in the root directory
4. Run the executable `./main.out` (A new folder called `results` should appear)
5. Run `python plotResults.py [path to csv file]` in the root directory, where `[path to csv file]` is the path to the csv file that was created in the `results` folder. (A new folder called `plots` should appear)

