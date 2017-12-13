# A Survey of Motion Planning for Robotic Arms 

## Getting Started

For this project, we deliver a survey of motion planning for robotic arms in a complexed environment setup based on PyBullet. We start by inspecting various path planning algorithm such as Astar, RRT and PRM in 2D and 3D space with arbitrary obstacles. Then we move to implementing robotic arm control in PyBullet interface. We also implement reinforcement learning based path planning. Then we compare the performance of traditional algorithm based motion planning and reinforcement learning based motion planning.

### Prerequisites

* To run PyBullet and Astar algorithm, you need:
  - Python 3
  - [PyBullet](https://pybullet.org/wordpress/)

* To run our state sampling algorithm (RRT and PRM), you need:
  - C++
  - [OMPL](http://ompl.kavrakilab.org/download.html)
  - MATLAB, as a mean to visualize the state sampling algorithm
  - We provide the executables for this part so you don't have to install the dependencies. While if you want to try different start/goal/obstacles, you need the full library to compile.

### Installing

Please refer to the link above on how to install the dependencies.

[Add more here]

```
git clone https://github.com/ompl/omplapp.git
cd omplapp
git clone https://github.com/ompl/ompl.git
```

You should be able to run our project code after installing the dependencies.

## Running the tests

Our overall topology is: 1) get environment data (start/goal/obstacles) from simulation environment and feed into C++ code base, 2) Use various planning algorithm to calculate a smoothed path, 3) visualize and direct the path back to simulation interface, 4) Utilize robotic control algorithm to direct the robotic arm to the destination. We will start by introducing how the path is calculated and visualized.

### Path planning using sampling-based algorithm (C++/MATLAB)

First, we show how to calculate and visualize a path in the 2D space. We provide the compiled executable "demo_GeomPlanningInfo" to run RRT algorithm and output the path, edges, vertices to a standard file. The start, goal, obstacles and planning algorithm can also be changed in the code "GeomPlanningSE2_Info.cpp" and recompiled using the modified "CMakeLists.txt" provided.

```
cd path_planning_cpp
./demo_GeomPlanningInfo
```

This will create 4 files: path0.dat(raw path), path.dat(smoothed path), vertices.dat, edges.dat. We have provided these files for PRM and RRT, and to visualize them, you can simply run "plotSolution2D.m".

Then we move to 3D space path planning. Similarly, we provide the executable "demo_3DSampling" but you can also change the parameters in "testSampling.cpp".

```
./demo_3DSampling
```

This will create plain text files which have the full planning path in it. We also provide a variety of the experiment results in our folder. To visualize the path, you can simply run "plotSolution3D.m"

### RL-based path planning (Python3)

[Add here]

```

```

### PyBullet (Python3)

[Add here]

```

``` 

## Deployment

There might be version problems with OMPL dependencies. If you are having trouble compiling the code, you might need to use an older version of the library.

## Authors

* **Zhihan Jiang** - *Path planning, visualization, documentation*
* **Holly Liang** - *RL-based path planning, PyBullet interface*
* **Tony Chen** - *Path planning algorithm, integration*

## Acknowledgments

* We used some of the skeleton or demo code from the official site (OMPL and PyBullet).
* CS221 best class ever

