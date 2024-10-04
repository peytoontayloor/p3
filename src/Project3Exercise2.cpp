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
    //most implementation from slide examples, need to double check if applied them correctly

    // TODO: Use your implementation of RTP to plan for a point robot.

    //need to set up state space:
    ompl::base::StateSpacePtr pointrobot;
    //in slides, says point robot is this space, but doing R2 instead of just the state space, not
    //sure if correct
    auto rv = std::make_shared<ompl::base::RealVectorStateSpace>(2);
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-20);
    bounds.setHigh(20);
    rv->setBounds(bounds);

    pointrobot = rv;

    //from slides, not sure about "simple set up" --> changed to rtp but still iffy
    //not sure if calling planner from RTP.h right?
    ompl::geometric::RTP rtp(pointrobot);

    //need to set state validator:
    rtp.setStateValidityChecker(std::bind(isValidStatePoint, std::placeholders::_1, obstacles));

    //need to set goal and start states: 
    ompl::base::ScopedState<> start(pointrobot);
    //not sure if need to set specific x and y values? 
    //start[0] = 0;

    //above was from slides, following demo doing:
    start->setX(0.0);
    start->setY(0.0);

    ompl::base:ScopedState<> goal(pointrobot);
    goal->setX(6.0);
    goal->setY(3.0);

    rtp.setStartAndGoalStates(start, goal);

    //call the planner:
    ompl::base::PlannerStatus solved = rtp.solve(1.0);
    //1.0 is ptc, should adjust and see what happens
    if (solved)
    {
        //print path to screen (from slides like most of this section)
        std::cout << "Found Solution: " << std::endl;
        ompl::geometric::PathGeometric &path = rtp.getSolutionPath();
        //not sure why example interpolates, commenting out for now
        //path.interpolate(50);
        path.printAsMatrix(std::cout);
    }
    else
    {
        std::cout << "No Solution Found" << std::endl;
    }

}

void planBox(const std::vector<Rectangle> &obstacles)
{
    // TODO: Use your implementation of RTP to plan for a rotating square robot.
    //same as point? --> minus state space?
    ompl::base::StateSpacePtr boxrobot;
    //since using SE2 state space, less involved? not sure exactly
    auto se = std::make_shared<ompl::base::SE2StateSpace>;
    //based on doc still takes real vector bounds as bounds
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-20);
    bounds.setHigh(20);
    se->setBounds(bounds);

    boxrobot = se;

    //again, not sure if calling on RTP correctly:
    ompl::geometric::RTP rtp(boxrobot);
    rtp.setStateValidityChecker(std::bind(isValidStateSquare, std::placeholders::_1, obstacles));
    
    //also iffy on if declaring start and goal states right?
    ompl::base::ScopedState<> start(boxrobot);
    start->setX(0.0);
    start->setY(0.0);

    ompl::base::ScopedState<> goal(boxrobot);
    goal->setX(6.0);
    goal->setY(3.0);

    rtp.setStartAndGoalStates(start, goal);

    ompl::base::PlannerStatus solved = rtp.solve(1.0);
    if (solved)
     {
        //print path to screen (from slides like most of this section)
        std::cout << "Found Solution: " << std::endl;
        ompl::geometric::PathGeometric &path = rtp.getSolutionPath();
        //not sure why example interpolates, commenting out for now
        //path.interpolate(50);
        path.printAsMatrix(std::cout);
    }
    else
    {
        std::cout << "No Solution Found" << std::endl;
    }

}

void makeEnvironment1(std::vector<Rectangle> &obstacles)
{
    // TODO: Fill in the vector of rectangles with your first environment.
    //obstacles:
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
    //this is the one where a larger rectangle might not fit in narrow paths but point robot will
    Rectangle r1, r2, r3, r4;

    r1.x = 2;
    r1.y = 0;
    r1.width = 2.5;
    r1.height = 1;

    r2.x = 4.5;
    r2.y = 1.5;
    r2.width = 1;
    r2.height = 5;

    r3.x = 3;
    r3.y = 2;
    r3.width = 2;
    r3.height = 2;

    r4.x = 1;
    r3.y = 3;
    r3.width = 1;
    r3.height = 3;

    obstacles.push_back(r1);
    obstacles.push_back(r2);
    obstacles.push_back(r3);
    obstacles.push_back(r4);
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
