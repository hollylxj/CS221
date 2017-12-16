# A Survey of Motion Planning for Robotic Arms 

## Getting Started

For this project, we deliver a survey of motion planning for robotic arms in a complexed environment setup based on PyBullet. We start by inspecting various path planning algorithm such as A*, RRT and PRM in 2D and 3D space with arbitrary obstacles. Then, we apply these methods as well as a reinforcement learning method in robotic arm motion control with PyBullet interface. Lastly, we compare the performance of traditional algorithm based motion planning and reinforcement learning based motion planning to see whether reinforcement learning can reach the same speed, smoothness and accuracy as traditional algorithms.

Since all the data we need in this project is generated during the experiment in PyBullet, we do not have any external dataset.

### Prerequisites

* To run PyBullet you need:
  - Python 3
  - [PyBullet](https://pybullet.org/wordpress/), refer to the build and installation section of the document for installation instructions(https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA).

* To run our state sampling algorithm (RRT and PRM), you need:
  - C++
  - We provide the executables for this part so you don't have to install the dependencies. While if you want to try different start/goal/obstacles, you need the full library to compile:
   - [OMPL](http://ompl.kavrakilab.org/download.html)
```
git clone https://github.com/ompl/omplapp.git
cd omplapp
git clone https://github.com/ompl/ompl.git
```
   - MATLAB, as a mean to visualize the state sampling algorithm



You should be able to run our project code after installing the dependencies.

## Running the tests

### A*
This algorithm shows how A* runs under pybullet. After a second of planning, the arm directly moves to the target location. The target is set to (0.7,-0.7,0.1), which can be changed in astar_run.py

```
python astar_run.py
```


### Path planning using sampling-based algorithm (C++/MATLAB)

First, we show how to calculate and visualize a path in the 2D space. We provide the compiled executable "demo_GeomPlanningInfo" to run RRT algorithm and output the path, edges, vertices to a standard file. The start, goal, obstacles and planning algorithm can also be changed in the code "GeomPlanningSE2_Info.cpp" and recompiled using the modified "CMakeLists.txt" provided.

```
cd path_planning_cpp
./demo_GeomPlanningInfo
```

This will create 4 files: path0.dat(raw path), path.dat(smoothed path), vertices.dat, edges.dat. We have provided these files for PRM and RRT, and to visualize them, you can simply run "plotSolution2D.m".

Then we move to 3D space path planning. Similarly, we provide the executable "demo_3DSampling" but you can also change the parameters in "testSampling.cpp".

```
cd path_planning_cpp
./demo_3DSampling
```

This will create plain text files which have the full planning path in it. We also provide a variety of the experiment results in our folder. To visualize the path, you can simply run "plotSolution3D.m"

### RL-based path planning (Python3)
This algorithm shows how Q-learning lambd runs under pybullet. After a 2000 trial loops, the arm stop, and once user presses "enter", robot will move to the target location in one test loop with exploration factor set to zero. The target is set to (0.7,-0.7,0.1), which can be changed in sawyer.py

```
python3 test.py
```


## Deployment

There might be version problems with OMPL dependencies. If you are having trouble compiling the code, you might need to use an older version of the library.

## Authors
* **Tony Chen** - *A* algorithm in 2D and in PyBullet, related documentation*
* **Zhihan Jiang** - *RRT and PRM algorithm 2D and 3D visualization, related documentation*
* **Holly Liang** - *Implementation of RL-based path planning, PyBullet setup, related documentation*


## Acknowledgments

* We used some of the skeleton or demo code from the official site (OMPL and PyBullet).
* CS221 course staff, especially our TA Andrey.

