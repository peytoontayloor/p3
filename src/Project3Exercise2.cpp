///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: FILL ME OUT!!
//////////////////////////////////////

#include <iostream>

// The collision checker routines
#include "CollisionChecking.h"

// Your random tree planner
#include "RTP.h"

void planPoint(const std::vector<Rectangle> &obstacles)
{
    base::RTP planner;
    // TODO: Use your implementation of RTP to plan for a point robot.

    //got all of this from the demo on OMPL (rigid body planning in 2D...)
    //set the planner:
    
    //need to set up state information for the planner (this is all a messy skeleton, not sure on syntax)

    base::SpaceInformation si;
    si.setStateValidityChecker(isValidStatePoint);
    //instructions imply we need to manually construct the state space? but thinking it is just SE2
     base::ScopedState<base::SE2StateSpace> start(setup.getSpaceInformation());
     start->setX(0.0);
     start->setY(0.0);

     //goal:
     // define goal state
     base::ScopedState<base::SE2StateSpace> goal(start);
     goal->setX(6.0);
     goal->setY(3.0);

     planner.setStartAndGoalStates(start, goal);

     //how to set up obstacles? --> not sure, couldnt find on ompl

     if (planner.solve())
     {
        //this should theoretically work? --> demo said it would? 
        planner.getSolutionPath().print(std::cout);
     }

}

void planBox(const std::vector<Rectangle> &obstacles)
{
    // TODO: Use your implementation of RTP to plan for a rotating square robot.
    //same as point?
    base::ScopedState<base::SE2StateSpace> start(setup.getSpaceInformation());
     start->setX(0.0);
     start->setY(0.0);
}

void makeEnvironment1(std::vector<Rectangle> &obstacles)
{
    // TODO: Fill in the vector of rectangles with your first environment.
    //obstacle 1: 
    Rectangle r1, r2, r3;

    r1.x = 1;
    r1.y = 1;
    r1.width = 2;
    r1.height = 4;

    r2.x = 4;
    r2.y = 4;
    r2.width = 2;
    r2.height = 1;

    r3.x = 4;
    r3.y = 1;
    r3.width = 2;
    r3.height = 1;

    obstacles.push_back(r1);
    obstacles.push_back(r2);
    obstacles.push_back(r3);

}

void makeEnvironment2(std::vector<Rectangle> &obstacles)
{
    // TODO: Fill in the vector of rectangles with your second environment.
}

int main(int /* argc */, char ** /* argv */)
{
    int robot, choice;
    std::vector<Rectangle> obstacles;

    do
    {
        std::cout << "Plan for: " << std::endl;
        std::cout << " (1) A point in 2D" << std::endl;
        std::cout << " (2) A rigid box in 2D" << std::endl;

        std::cin >> robot;
    } while (robot < 1 || robot > 2);

    do
    {
        std::cout << "In Environment: " << std::endl;
        std::cout << " (1) TODO" << std::endl;
        std::cout << " (2) TODO" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    switch (choice)
    {
        case 1:
            makeEnvironment1(obstacles);
            break;
        case 2:
            makeEnvironment2(obstacles);
            break;
        default:
            std::cerr << "Invalid Environment Number!" << std::endl;
            break;
    }

    switch (robot)
    {
        case 1:
            planPoint(obstacles);
            break;
        case 2:
            planBox(obstacles);
            break;
        default:
            std::cerr << "Invalid Robot Type!" << std::endl;
            break;
    }

    return 0;
}
