///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: Swaha Roy and Peyton Elebash
//////////////////////////////////////

#include <iostream>

// Your random tree planner
#include "RTP.h"

//trying to include correct headers:
#include <omplapp/apps/SE3RigidBodyPlanning.h>
#include <omplapp/config.h>
#include "ompl/tools/benchmark/Benchmark.h"
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/est/EST.h>

using namespace ompl;

void benchmarkApartment()
{
    // Plan in SE3
    app::SE3RigidBodyPlanning setup;

    // Load robot and environment
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Apartment_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Apartment_env.dae";
    //not sure if needed?
    //geometric::SimpleSetup ss(space);
    setup.setRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);

    // Define start state
    base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(241.81);
    start->setY(106.15);
    start->setZ(36.46);
    start->rotation().setIdentity();

    // define goal state
    base::ScopedState<base::SE3StateSpace> goal(start);
    goal->setX(-31.19);
    goal->setY(-99.85);
    goal->setZ(36.46);
    goal->rotation().setIdentity();
  
    // set the start & goal states
    setup.setStartAndGoalStates(start, goal);
  
    // setting collision checking resolution to 1% of the space extent
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);
  
    // we call setup just so print() can show more information
    setup.setup();
    setup.print();
    
    // Create benchmark class
    tools::Benchmark b(setup, "my experiment");

    // Add planners
    b.addPlanner(base::PlannerPtr(new geometric::RTP(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new geometric::PRM(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new geometric::RRT(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new geometric::EST(setup.getSpaceInformation())));

    // Create benchmark request
    tools::Benchmark::Request req;
    req.maxTime = 30.0;
    req.maxMem = 100.0;
    req.runCount = 50;
    req.displayProgress = true;
    b.benchmark(req);

    // Generate file
    b.saveResultsToFile();
}

void benchmarkHome()
{
    //same as apartment but change path to mesh
    // Plan in SE3
    app::SE3RigidBodyPlanning setup;

    // Load robot and environment
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Home_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Home_env.dae";
    //not sure if needed?
    //geometric::SimpleSetup ss(space);
    setup.setRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);

    // Define start state
    //TO DO: start and goal states may be too simple/why it isnt not benchmarking correctly(too fast)
    base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(252.95);
    start->setY(-214.95);
    start->setZ(46.19);
    start->rotation().setIdentity();

    // define goal state
    base::ScopedState<base::SE3StateSpace> goal(start);
    goal->setX(262.95);
    goal->setY(75.05);
    goal->setZ(46.19);
    goal->rotation().setIdentity();
  
    // set the start & goal states
    setup.setStartAndGoalStates(start, goal);
  
    // setting collision checking resolution to 1% of the space extent
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);
  
    // we call setup just so print() can show more information
    setup.setup();
    setup.print();
    
    // Create benchmark class
    tools::Benchmark b(setup, "my experiment");

    // Add planners
    b.addPlanner(base::PlannerPtr(new geometric::RTP(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new geometric::PRM(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new geometric::RRT(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new geometric::EST(setup.getSpaceInformation())));

    // Create benchmark request
    tools::Benchmark::Request req;
    req.maxTime = 30.0;
    req.maxMem = 100.0;
    req.runCount = 50;
    req.displayProgress = true;
    b.benchmark(req);

    // Generate file
    b.saveResultsToFile();
}

int main(int /* argc */, char ** /* argv */)
{
    int environment;

    do
    {
        std::cout << "Benchmark for: " << std::endl;
        std::cout << " (1) Apartment" << std::endl;
        std::cout << " (2) Home" << std::endl;

        std::cin >> environment;
    } while (environment < 1 || environment > 2);

    switch (environment)
    {
        case 1:
            benchmarkApartment();
            break;
        case 2:
            benchmarkHome();
            break;
        default:
            std::cerr << "Invalid Environment!" << std::endl;
            break;
    }

    return 0;
}
