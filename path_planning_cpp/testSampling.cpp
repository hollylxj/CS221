/*3D Path Planning and Visualization
Author: Zhihan Jiang
*/

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>
#include <fstream>
#include <thread>

using namespace std;
namespace ob = ompl::base;
namespace og = ompl::geometric;


class MyValidStateSampler : public ob::ValidStateSampler
{
public:
    MyValidStateSampler(const ob::SpaceInformation *si) : ValidStateSampler(si)
    {
        name_ = "my sampler";
    }
    // Generate a sample in the valid part of the R^3 state space
    // Valid states satisfy the following constraints:
    // -1<= x,y,z <=1
    // if .25 <= z <= .5, then |x|>.8 and |y|>.8
    bool sample(ob::State *state) override
    {
        double* val = static_cast<ob::RealVectorStateSpace::StateType*>(state)->values;
        double z = rng_.uniformReal(-1,1);

        if (z>.25 && z<.5)
        {
            double x = rng_.uniformReal(0,1.8), y = rng_.uniformReal(0,.2);
            switch(rng_.uniformInt(0,3))
            {
                case 0: val[0]=x-1;  val[1]=y-1;
                case 1: val[0]=x-.8; val[1]=y+.8;
                case 2: val[0]=y-1;  val[1]=x-1;
                case 3: val[0]=y+.8; val[1]=x-.8;
            }
        }
        else
        {
            val[0] = rng_.uniformReal(-1,1);
            val[1] = rng_.uniformReal(-1,1);
        }
        val[2] = z;
        assert(si_->isValid(state));
        return true;
    }
    // We don't need this in the example below.
    bool sampleNear(ob::State* /*state*/, const ob::State* /*near*/, const double /*distance*/) override
    {
        throw ompl::Exception("MyValidStateSampler::sampleNear", "not implemented");
        return false;
    }
protected:
    ompl::RNG rng_;
};

bool isStateValid(const ob::State *state)
{
    const ob::RealVectorStateSpace::StateType& pos = *state->as<ob::RealVectorStateSpace::StateType>();
    // Let's pretend that the validity check is computationally relatively
    // expensive to emphasize the benefit of explicitly generating valid
    // samples
    std::this_thread::sleep_for(ompl::time::seconds(.0005));
    // Valid states satisfy the following constraints:
    // -1<= x,y,z <=1
    // if .25 <= z <= .5, then |x|>.8 and |y|>.8
    return !((fabs(pos[0])<.8 || fabs(pos[1])<.8) && pos[2]>.25 && pos[2]<.5);
}

// return an obstacle-based sampler
ob::ValidStateSamplerPtr allocOBValidStateSampler(const ob::SpaceInformation *si)
{
    // we can perform any additional setup / configuration of a sampler here,
    // but there is nothing to tweak in case of the ObstacleBasedValidStateSampler.
    return std::make_shared<ob::ObstacleBasedValidStateSampler>(si);
}

// return an instance of my sampler
ob::ValidStateSamplerPtr allocMyValidStateSampler(const ob::SpaceInformation *si)
{
    return std::make_shared<MyValidStateSampler>(si);
}


void plan(int samplerIndex, ofstream &infile, ofstream &simple)
{
    // construct the state space we are planning in
    auto space(std::make_shared<ob::RealVectorStateSpace>(3));

    // set the bounds
    ob::RealVectorBounds bounds(3);
    bounds.setLow(-1);
    bounds.setHigh(1);
    space->setBounds(bounds);

    // define a simple setup class
    og::SimpleSetup ss(space);

    // set state validity checking for this space
    ss.setStateValidityChecker(isStateValid);

    // create a start state
    ob::ScopedState<> start(space);
    start[0] = start[1] = start[2] = 0;

    // create a goal state
    ob::ScopedState<> goal(space);
    goal[0] = goal[1] = 0.;
    goal[2] = 1;

    // set the start and goal states
    ss.setStartAndGoalStates(start, goal);

    // set sampler (optional; the default is uniform sampling)
    if (samplerIndex==1)
        // use obstacle-based sampling
        ss.getSpaceInformation()->setValidStateSamplerAllocator(allocOBValidStateSampler);
    else if (samplerIndex==2)
        // use my sampler
        ss.getSpaceInformation()->setValidStateSamplerAllocator(allocMyValidStateSampler);

    // set motion validator
    ss.getSpaceInformation()->setMotionValidator(
        std::make_shared<ob::DiscreteMotionValidator>(
            ss.getSpaceInformation()));

    // create a planner for the defined space
    auto planner(std::make_shared<og::PRM>(ss.getSpaceInformation()));
    ss.setPlanner(planner);

    // attempt to solve the problem within ten seconds of planning time
    ob::PlannerStatus solved = ss.solve(10.0);
    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        ss.getSolutionPath().print(std::cout);
        ss.getSolutionPath().printAsMatrix(infile);

        ss.simplifySolution();
        ss.getSolutionPath().printAsMatrix(simple);
    }
    else
        std::cout << "No solution found" << std::endl;
}

int main(int /*argc*/, char ** /*argv*/)
{
    ofstream uniform, obstacle, diy;
    // uniform.open("uniform.txt");
    obstacle.open("obstacle.txt");
    diy.open("diy.txt");
    ofstream simple1, simple2, simple3;
    simple1.open("simple1.txt");
    // simple2.open("simple2.txt");
    simple3.open("simple3.txt");
    std::cout << "Using default uniform sampler:" << std::endl;
    plan(0, uniform, simple1);
    // std::cout << "\nUsing obstacle-based sampler:" << std::endl;
    // plan(1, obstacle, simple2);
    std::cout << "\nUsing my sampler:" << std::endl;
    plan(2, diy, simple3);
    return 0;
}
