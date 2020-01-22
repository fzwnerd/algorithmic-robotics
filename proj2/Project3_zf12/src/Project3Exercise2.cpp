///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: Zhenwei Feng
//////////////////////////////////////
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>
#include <iostream>

// The collision checker produced in project 2
#include "CollisionChecking.h"

// Your random tree planner

#include "RTP.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
const double sqrSide = 0.3;

bool isPointStateValid(const ob::State *state, const std::vector<Rectangle> obstacles)
{
    const double* st = static_cast<const ob::RealVectorStateSpace::StateType*>(state)->values;
    return isValidPoint(st[0], st[1], obstacles);
}

bool isSquareStateValid(const ob::State *state, const std::vector<Rectangle> obstacles)
{
    const auto* se2State = state->as<ob::SE2StateSpace::StateType>();
    double x = se2State->getX();
    double y = se2State->getY();
    double theta = se2State->getYaw();
    return isValidSquare(x, y, theta, sqrSide, obstacles);
}

void planPoint(const std::vector<Rectangle> & obstacles)
{
    // TODO: Use your implementation of RTP to plan for a point robot.
    auto space(std::make_shared<ob::RealVectorStateSpace>(2));
    space->setBounds(-5, 5);
    og::SimpleSetup ss(space);
    ss.setStateValidityChecker(std::bind(isPointStateValid, std::placeholders::_1, obstacles));

    // set RTP planner
    auto planner(std::make_shared<og::RTP>(ss.getSpaceInformation()));
    //auto planner(std::make_shared<og::RRTConnect>(ss.getSpaceInformation()));
    ss.setPlanner(planner);

    ob::ScopedState<> start(space);
    //start.random();
    start[0] = start[1] = -2.75;
 
    ob::ScopedState<> goal(space);
    //goal.random();
    goal[0] = 4.9;
    goal[1] = 0;
 
    ss.setStartAndGoalStates(start, goal);

    ob::PlannerStatus solved = ss.solve(1.0);

    if (solved)
     {
         // get the goal representation from the problem definition (not the same as the goal state)
         // and inquire about the found path
         std::cout << "Found solution:" << std::endl;
 
         // print the path to screen
         //path->print(std::cout);
         
         ss.getSolutionPath().printAsMatrix(std::cout);
     }
     else
         std::cout << "No solution found" << std::endl;
}

void planBox(const std::vector<Rectangle> & obstacles)
{
    // TODO: Use your implementation of RTP to plan for a rotating square robot.
    auto space(std::make_shared<ob::SE2StateSpace>());

    ob::RealVectorBounds bounds(2);
    bounds.setLow(-5);
    bounds.setHigh(5);
    space->setBounds(bounds);

    og::SimpleSetup ss(space);
    ss.setStateValidityChecker(std::bind(isSquareStateValid, std::placeholders::_1, obstacles));

    // set RTP planner
    auto planner(std::make_shared<og::RTP>(ss.getSpaceInformation()));
    //auto planner(std::make_shared<og::RRTConnect>(ss.getSpaceInformation()));
    ss.setPlanner(planner);

    ob::ScopedState<ob::SE2StateSpace> start(space);
    //start.random();
    start->setX(-3.5);
    start->setY(-3.5);
    start->setYaw(0.0);
 
    ob::ScopedState<ob::SE2StateSpace> goal(space);
    //goal.random();
    goal->setX(4.7);
    goal->setY(4.7);
    goal->setYaw(3.1415926/4);
 
    ss.setStartAndGoalStates(start, goal);

    ob::PlannerStatus solved = ss.solve(5.0);

    if (solved)
     {
         // get the goal representation from the problem definition (not the same as the goal state)
         // and inquire about the found path
         std::cout << "Found solution:" << std::endl;
 
         // print the path to screen
         //path->print(std::cout);
         //ss.getSolutionPath().printAsMatrix(std::cout);
         og::PathGeometric &path = ss.getSolutionPath();
         path.interpolate(25);
         path.printAsMatrix(std::cout);
     }
     else
         std::cout << "No solution found" << std::endl;

}

void makeEnvironment1(std::vector<Rectangle> & obstacles )
{
    // TODO: Fill in the vector of rectangles with your first environment.
    Rectangle rect;

    // middle
    rect.x = 0.5;
    rect.y = 0.0;
    rect.height = 1.0;
    rect.width = 2.0;
    obstacles.push_back(rect);

    // upper middle
    rect.x = -1.0;
    rect.y = 2.0;
    rect.height = 1.0;
    rect.width = 2.0;
    obstacles.push_back(rect);

    // lower middle
    rect.x = -1.0;
    rect.y = -2.0;
    rect.height = 1.0;
    rect.width = 2.0;
    obstacles.push_back(rect);

    // left
    rect.x = -3.0;
    rect.y = -2.5;
    rect.height = 5.0;
    rect.width = 1.0;
    obstacles.push_back(rect);

    // right
    rect.x = 3.35;
    rect.y = -1.25;
    rect.height = 3.75;
    rect.width = 0.5;
    obstacles.push_back(rect);
}

void makeEnvironment2(std::vector<Rectangle> & obstacles)
{
    // TODO: Fill in the vector of rectangles with your second environment.
    /*
    Rectangle rect;

    // middle
    rect.x = -1.0;
    rect.y = -1.5;
    rect.height = 1.0;
    rect.width = 0.5;
    obstacles.push_back(rect);

    rect.x = -1.0;
    rect.y = 0.5;
    rect.height = 1.0;
    rect.width = 0.5;
    obstacles.push_back(rect);

    rect.x = 0.5;
    rect.y = -1.5;
    rect.height = 1.0;
    rect.width = 0.5;
    obstacles.push_back(rect);

    rect.x = 0.5;
    rect.y = 0.5;
    rect.height = 1.0;
    rect.width = 0.5;
    obstacles.push_back(rect);
    */
   Rectangle rect;

    // middle
    rect.x = 0.5;
    rect.y = 0.0;
    rect.height = 1.0;
    rect.width = 2.0;
    obstacles.push_back(rect);

    // upper middle
    rect.x = -1.0;
    rect.y = 2.0;
    rect.height = 1.0;
    rect.width = 2.0;
    obstacles.push_back(rect);

    // lower middle
    rect.x = -1.0;
    rect.y = -2.0;
    rect.height = 1.0;
    rect.width = 2.0;
    obstacles.push_back(rect);

    // left
    rect.x = -4.3;
    rect.y = -2.5;
    rect.height = 1.0;
    rect.width = 3.0;
    obstacles.push_back(rect);

    // right
    rect.x = 3.35;
    rect.y = -2.75;
    rect.height = 1.0;
    rect.width = 1.0;
    obstacles.push_back(rect);
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
