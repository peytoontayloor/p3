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

/* Use RTP to plan for a point robot */
void planPoint(const std::vector<Rectangle> &obstacles)
{
    // Create R^2 state (configuration) space of point robot
    ompl::base::StateSpacePtr r2(new ompl::base::RealVectorStateSpace(2));

    // Set lower an upper bounds
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-20);
    bounds.setHigh(20);
    r2->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);

    // Construct an instance of space information from this state space
    //TODO: understand why this didn't work -> ompl::base::SpaceInformationPtr si(r2);
    ompl::base::SpaceInformationPtr si = std::make_shared<ompl::base::SpaceInformation>(r2);

    // Set the StateValidityChecker for a point robot
    si->setStateValidityChecker(std::bind(isValidStatePoint, std::placeholders::_1, obstacles));
    si->setup();

    // Create a problem instance
    ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));

    // TODO: maybe set random star t and goal states?
    // Specify the start and goal states
    ompl::base::ScopedState<> start(r2);
    start[0] = 0.0;
    start[1] = 0.0;

    ompl::base::ScopedState<> goal(r2);
    goal[0] = 6.0;
    goal[1] = 3.0;

    pdef->setStartAndGoalStates(start, goal);

    // Instantiate the RTP planner
    ompl::base::PlannerPtr planner(new ompl::geometric::RTP(si));
    planner->setProblemDefinition(pdef);
    planner->setup();
    
    // Call planner
    // 1.0 is ptc, should adjust and see what happens
    ompl::base::PlannerStatus solved = planner->solve(1.0);

    if (solved)
    {
        // Print path to screen
        std::cout << "Found Solution: " << std::endl;
        ompl::base::PathPtr path = pdef->getSolutionPath();
        //not sure why example interpolates, commenting out for now
        //path.interpolate(50);
        path->as<ompl::geometric::PathGeometric>()->printAsMatrix(std::cout);
    }
    else
    {
        std::cout << "No Solution Found" << std::endl;
    }

    planner->clear();

}

void planBox(const std::vector<Rectangle> &obstacles)
{
    // // TODO: Use your implementation of RTP to plan for a rotating square robot.
    // //same as point? --> minus state space?
    // ompl::base::StateSpacePtr boxrobot;
    // //since using SE2 state space, less involved? not sure exactly
    // auto se = std::make_shared<ompl::base::SE2StateSpace>;
    // //based on doc still takes real vector bounds as bounds
    // ompl::base::RealVectorBounds bounds(2);
    // bounds.setLow(-20);
    // bounds.setHigh(20);
    // se->setBounds(bounds);

    // boxrobot = se;

    // //again, not sure if calling on RTP correctly:
    // ompl::geometric::RTP rtp(boxrobot);
    // rtp.setStateValidityChecker(std::bind(isValidStateSquare, std::placeholders::_1, obstacles));
    
    // //also iffy on if declaring start and goal states right?
    // ompl::base::ScopedState<> start(boxrobot);
    // //assuming pos 0 is x and pos 1 is y
    // start[0] = 0.0;
    // start[1] = 0.0;

    // ompl::base::ScopedState<> goal(boxrobot);
    // goal[0] = 6.0;
    // goal[1] = 3.0;

    // rtp.setStartAndGoalStates(start, goal);

    // ompl::base::PlannerStatus solved = rtp.solve(1.0);
    // if (solved)
    //  {
    //     //print path to screen (from slides like most of this section)
    //     std::cout << "Found Solution: " << std::endl;
    //     ompl::geometric::PathGeometric &path = rtp.getSolutionPath();
    //     //not sure why example interpolates, commenting out for now
    //     //path.interpolate(50);
    //     path.printAsMatrix(std::cout);
    // }
    // else
    // {
    //     std::cout << "No Solution Found" << std::endl;
    // }

    //new implementation following point robot implementation:
    ompl::base::StateSpacePtr se2(new ompl::base::SE2StateSpace);

    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-20);
    bounds.setHigh(20);
    se2->as<ompl::base::SE2StateSpace>()->setBounds(bounds);

    ompl::base::SpaceInformationPtr si = std::make_shared<ompl::base::SpaceInformation>(se2);

    si->setStateValidityChecker(std::bind(isValidStateSquare, std::placeholders::_1, obstacles));
    si->setup();

    ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));

    ompl::base::ScopedState<> start(se2);
    start[0] = 0.0;
    start[1] = 1.0;

    ompl::base::ScopedState<> goal(se2);
    goal[0] = 6.0;
    goal[1] = 3.0;

    pdef->setStartAndGoalStates(start, goal);

    ompl::base::PlannerPtr planner(new ompl::geometric::RTP(si));
    planner->setProblemDefinition(pdef);
    planner->setup();

    ompl::base::PlannerStatus solved = planner->solve(1.0);

    if (solved)
    {
        std::cout << "Found SOlution: " << std::endl;
        ompl::base::PathPtr path = pdef->getSolutionPath();
        path->as<ompl::geometric::PathGeometric>()->printAsMatrix(std::cout);
    }
    else
    {
        std::cout << "No Solution Found" << std::endl;
    }

    planner->clear();



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
