///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: Swaha Roy and Peyton Elebash
//////////////////////////////////////

#include <iostream>
#include <fstream>

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
    bounds.setLow(0);
    bounds.setHigh(9);
    r2->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);

    // Construct an instance of space information from this state space
    ompl::base::SpaceInformationPtr si = std::make_shared<ompl::base::SpaceInformation>(r2);

    // Set the StateValidityChecker for a point robot
    si->setStateValidityChecker(std::bind(isValidStatePoint, std::placeholders::_1, obstacles));
    si->setup();

    // Create a problem instance
    ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));

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
    ompl::base::PlannerStatus solved = planner->solve(5.0);

    if (solved)
    {
        // Print path in terminal
        std::cout << "Found Solution: " << std::endl;
        ompl::base::PathPtr path = pdef->getSolutionPath();
        path->as<ompl::geometric::PathGeometric>()->printAsMatrix(std::cout);

        // Capture output in path.txt file
        std::ofstream pathOutput("path.txt");
        path->as<ompl::geometric::PathGeometric>()->printAsMatrix(pathOutput);
        pathOutput.close();
    }
    else
    {
        std::cout << "No Solution Found" << std::endl;
        
    }

    planner->clear();

}

/* Use RTP to plan for a rectangular robot */
void planBox(const std::vector<Rectangle> &obstacles)
{
    ompl::base::StateSpacePtr se2(new ompl::base::SE2StateSpace);

    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.setHigh(9);
    se2->as<ompl::base::SE2StateSpace>()->setBounds(bounds);

    ompl::base::SpaceInformationPtr si = std::make_shared<ompl::base::SpaceInformation>(se2);

    // Choose length of robot
    si->setStateValidityChecker(std::bind(isValidStateSquare, std::placeholders::_1, 1, obstacles));
    si->setup();

    ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));

    //also need orientation? -> no, assumes orientation 0 if nothing is put
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
        std::cout << "Found Solution: " << std::endl;
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

/* Environment with axis-aligned rectangular obstacles with narrow gaps s.t. a larger rectangual
    robot may be obstructed.
*/
void makeEnvironment2(std::vector<Rectangle> &obstacles)
{
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
    r4.y = 3;
    r4.width = 1;
    r4.height = 3;

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
        std::cout << " (1) Three Rectangles " << std::endl;
        std::cout << " (2) Overlapping Rectangles " << std::endl;

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
