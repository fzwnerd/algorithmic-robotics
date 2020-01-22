///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 5
// Authors: Zhenwei Feng 
//////////////////////////////////////

#include <iostream>
#include <cmath>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>
#include <boost/tuple/tuple.hpp>
#include <ompl/tools/benchmark/Benchmark.h>
// The collision checker produced in project 2
#include "CollisionChecking.h"

// Your random tree planner
#include "DRRT.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;


const double radius = 0.1; // radius of circle robot

// create obstacles when there are four robots
void makeObstacles1(std::vector<Rectangle> & obstacles)
{
    Rectangle rect;

    // left 
    rect.x = 0.0;
    rect.y = 0.3;
    rect.height = 0.7;
    rect.width = 0.5;
    obstacles.push_back(rect);

    // right
    rect.x = 0.8;
    rect.y = 0.3;
    rect.height = 0.7;
    rect.width = 0.5;
    obstacles.push_back(rect);
}

void makeObstacles2(std::vector<Rectangle> &  obstacles)
{
    Rectangle rect;

    // lower left 
    rect.x = 0.0;
    rect.y = 0.3;
    rect.height = 0.7;
    rect.width = 0.8;
    obstacles.push_back(rect);

    // lower right
    rect.x = 1.1;
    rect.y = 0.3;
    rect.height = 0.7;
    rect.width = 0.8;
    obstacles.push_back(rect);

    // upper left
    rect.x = 0.0;
    rect.y = 1.3;
    rect.height = 0.7;
    rect.width = 0.8;
    obstacles.push_back(rect);

    // upper right
    rect.x = 1.1;
    rect.y = 1.3;
    rect.height = 0.7;
    rect.width = 0.8;
    obstacles.push_back(rect);
}

bool isMultRobStateValid(const ob::State *state, int numRob, const std::vector<Rectangle> & obstacles)
{
    auto cstate = state->as<ob::CompoundState>(); 

    std::vector<std::vector<double>> states;

    for (int i = 0; i < numRob; i++)
    {
        auto r2state = cstate->as<ob::RealVectorStateSpace::StateType>(i);
        double x = r2state->values[0];
        double y = r2state->values[1];
        if (!isValidCircle(x, y, radius, obstacles))
            return false;
        std::vector<double> temp{x, y};
        states.push_back(temp);
    }

    // check collision between circle robots
    for (int i = 0; i < numRob; i++)
    {
        for (int j = i + 1; j < numRob; j++)
        {
            auto state1 = states[i];
            auto state2 = states[j];
            double x1 = state1[0];
            double y1 = state1[1];
            double x2 = state2[0];
            double y2 = state2[1];
            double dist = std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
            if (dist <= 2 * radius) return false;
        }
    }
    return true;
    
}

bool isSingleRobStateValid(const ob::State *state, const std::vector<Rectangle> & obstacles)
{
    auto r2state = state->as<ob::RealVectorStateSpace::StateType>();
    double x = r2state->values[0];
    double y = r2state->values[1];
    return isValidCircle(x, y, radius, obstacles);
}

std::pair<double**, double**> setStartGoal(int numRob)
{
    double** starts = new double*[numRob];
    double** goals = new double*[numRob];
    std::pair<double**, double**> res;
    if (numRob == 4)
    {
        starts[0] = new double[2]{0.15, 1.15};
        starts[1] = new double[2]{1.15, 1.15};
        starts[2] = new double[2]{0.15, 0.15};
        starts[3] = new double[2]{1.15, 0.15};
        goals[0] = new double[2]{1.15, 0.15};
        goals[1] = new double[2]{0.15, 0.15};
        goals[2] = new double[2]{1.15, 1.15};
        goals[3] = new double[2]{0.15, 1.15};
    }
    else
    {
        starts[0] = new double[2]{0.15, 1.15};   // middle left
        starts[1] = new double[2]{1.75, 1.15};   // middle right
        starts[2] = new double[2]{0.15, 0.15};   // lower left
        starts[3] = new double[2]{1.75, 0.15};   // lower right
        starts[4] = new double[2]{0.15, 2.15};   // upper left
        //starts[5] = new double[2]{1.75, 2.15};   // upper right
        goals[0] = new double[2]{0.15, 0.15};
        goals[1] = new double[2]{0.15, 2.15};
        goals[2] = new double[2]{1.75, 0.15};
        goals[3] = new double[2]{0.15, 1.15};
        goals[4] = new double[2]{1.75, 1.15};
        //goals[5] = new double[2]{0.15, 1.15};
    }
    res.first = starts;
    res.second = goals;
    //std::cout << res.first[0][0];
    return res;
}

og::SimpleSetupPtr createMultRob(int numRob, std::vector<Rectangle> & obstacles, 
        double** starts, double** goals)
{
    ob::StateSpacePtr compSS;

    for (int i = 0; i < numRob; i++)
    {
        auto r2 = std::make_shared<ob::RealVectorStateSpace>(2);
        ob::RealVectorBounds bounds(2);
        if (numRob == 4)
        {
            bounds.setLow(0.1);  
            bounds.setHigh(1.2);
        }
        else
        {
            bounds.setLow(0, 0.1);  
            bounds.setHigh(0, 1.8);
            bounds.setLow(1, 0.1);  
            bounds.setHigh(1, 2.2);
        }
        
        r2->setBounds(bounds);
        compSS = compSS + r2;
    }

    auto ssptr(std::make_shared<og::SimpleSetup>(compSS));

    ssptr->setStateValidityChecker(
        [numRob, obstacles](const ob::State *state) { return isMultRobStateValid(state, numRob, obstacles); });

    ob::ScopedState<> start(compSS);

    ob::ScopedState<> goal(compSS);
    
    //std::cout << starts->at(0).at(0);
    for (int i = 0; i < 2 * numRob; i++)
    {
        start[i] = starts[i / 2][i % 2];
        goal[i] = goals[i / 2][i % 2];
    }

    // 0.28 finish in 95 seconds
    // 0.20 finish in 300 seconds
    if (numRob == 4)
        ssptr->setStartAndGoalStates(start, goal, 0.20);
    else
        ssptr->setStartAndGoalStates(start, goal, 0.25);
    
    //std::cout << start[6] << std::endl << start[7] << std::endl
    //        << goal[6] << std::endl << goal[7] << std::endl;
    
    return ssptr;
}

PRMptr createPRM(int numRob, std::vector<Rectangle> & obstacles, double** starts, double** goals)
{
    auto space(std::make_shared<ob::RealVectorStateSpace>(2));
    ob::RealVectorBounds bounds(2);
    if (numRob == 4)
    {
        bounds.setLow(0.1);  
        bounds.setHigh(1.2);
    }
    else
    {
        bounds.setLow(0, 0.1);  
        bounds.setHigh(0, 1.8);
        bounds.setLow(1, 0.1);  
        bounds.setHigh(1, 2.2);
    }
    
    space->setBounds(bounds);
    auto ssptr(std::make_shared<og::SimpleSetup>(space));
    ssptr->setStateValidityChecker(
        [obstacles](const ob::State *state) { return isSingleRobStateValid(state, obstacles); });
    ompl::base::ScopedState<> start(space);
    ompl::base::ScopedState<> goal(space);
    start[0] = starts[0][0];
    start[1] = starts[0][1];
    goal[0] = goals[0][0];
    goal[1] = goals[0][1];
    ssptr->setStartAndGoalStates(start, goal);
    //ssptr->print();
    PRMptr PRMplanner(std::make_shared<og::PRM>(ssptr->getSpaceInformation()));
    //ompl::geometric::PRM* PRMplanner = new ompl::geometric::PRM(ssptr->getSpaceInformation());
    ssptr->setPlanner(PRMplanner);
    //ssptr->setPlanner(std::make_shared<og::PRM>(*PRMplanner));
    ssptr->setup();
    ob::PlannerStatus solved = ssptr->solve(10.0);

    // test PRM for single robot
    if (!solved)
    {
        //std::cout << "Found solution:" << std::endl;
        /// print the path to screen
        //ssptr->getSolutionPath().printAsMatrix(std::cout);
        std::cerr << "No valid PRM roadmap found" << std::endl;
        
    }
    //else
        //std::cout << "No solution found" << std::endl;

    og::PRM::Graph roadmap = PRMplanner->getRoadmap();
    
    return PRMplanner;
}

void planMultRob(og::SimpleSetupPtr & ss, int numRob, const PRMptr& PRMplanner)
{
    //std::cout << ss->getSpaceInformation()->getStateDimension() << std::endl;
    auto planner = std::make_shared<og::DRRT>(ss->getSpaceInformation(), numRob, PRMplanner);
    ss->setPlanner(planner);
    
    ss->setup();
    //std::cout << "aa" << std::endl;
    //
    /// attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved;
    if (numRob == 4)
        solved = ss->solve(1200.0);
    else
        solved = ss->solve(4000.0);
    
    if (solved)
    {
        //std::cout << "planner data:" << std::endl;
        //ompl::control::PlannerData pd(ss->getSpaceInformation());
        //planner->getPlannerData(pd);
        //pd.printGraphML();
        
        std::cout << "Found solution:" << std::endl;
        /// print the path to screen
        ss->getSolutionPath().printAsMatrix(std::cout);       
    }
    else
        std::cout << "No solution found" << std::endl;
}

void benchmarkMultRob(og::SimpleSetupPtr & ss, int numRob, const PRMptr& PRMplanner)
{
    //ompl::tools::Benchmark b;
    if (numRob == 4)
    {
        ompl::tools::Benchmark b(*ss, "Rob4 experiment with states");
        //ompl::tools::Benchmark b(*ss, "test");
        ompl::tools::Benchmark::Request req;
        req.maxTime = 1500.0;
        //req.maxTime = 2.0;
        req.runCount = 11;
        b.addPlanner(ob::PlannerPtr(new og::DRRT(ss->getSpaceInformation(), numRob, PRMplanner)));
        req.displayProgress = true;
        b.benchmark(req);
        // This will generate a file of the form ompl_host_time.log
        b.saveResultsToFile();
    }
    else
    {
        ompl::tools::Benchmark b(*ss, "Rob5 experiment");
        //b.addPlanner(ob::PlannerPtr(new og::DRRT(ss->getSpaceInformation(), numRob, PRMplanner)));
        b.addPlanner(ob::PlannerPtr(new og::DRRT(ss->getSpaceInformation(), numRob, PRMplanner)));
        ompl::tools::Benchmark::Request req;
        req.maxTime = 4200.0;
        req.runCount = 16;
        req.displayProgress = true;
        b.benchmark(req);
        // This will generate a file of the form ompl_host_time.log
        b.saveResultsToFile();
    }
}

int main(int /* argc */, char ** /* argv */)
{
    int choice;
    std::vector<Rectangle> obstacles;

    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);
    //std::cout << choice << "aaa" << std::endl;
    int numRob; // number of robots
    do
    {
        std::cout << "Number of robots: " << std::endl;
        std::cout << " (1) 4" << std::endl;
        std::cout << " (2) 5" << std::endl;

        std::cin >> numRob;
    } while (numRob < 1 || numRob > 2);
    if (numRob == 1) numRob = 4;
    else numRob = 5;
    auto startGoal = setStartGoal(numRob);
    og::SimpleSetupPtr ss = nullptr;
    
    if (numRob == 4)
        makeObstacles1(obstacles);
    else
        makeObstacles2(obstacles);
    PRMptr PRMplanner = createPRM(numRob, obstacles, startGoal.first, startGoal.second);
    /* visualize PRM
    boost::graph_traits<og::PRM::Graph>::vertex_iterator i, end;
    boost::property_map<og::PRM::Graph, og::PRM::vertex_state_t>::const_type states_map = 
                                    boost::get(og::PRM::vertex_state_t(), PRMplanner->getRoadmap());
    for (boost::tie(i, end) = boost::vertices(PRMplanner->getRoadmap()); i != end; ++i)
    {
        //std::cout << i->values[0] << " " << i->values[1] << 
        //std::cout << boost::get(index_map, *i) << std::endl;
        ob::State* st = boost::get(states_map, *i);
        auto r2state = st->as<ob::RealVectorStateSpace::StateType>();
        std::cout << r2state->values[0] << " " << r2state->values[1] << std::endl;
        //std::cout << boost::out_degree(*i, PRMplanner->getRoadmap()) << std::endl;
    }
    */
    ss = createMultRob(numRob, obstacles, startGoal.first, startGoal.second);
    // Planning
    //
    if (choice == 1)
    {
        //;
        planMultRob(ss, numRob, PRMplanner);
        //std::cout << "aaa\n";
    }
    else if (choice == 2)
        benchmarkMultRob(ss, numRob, PRMplanner);
    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;
    for (int i = 0; i < numRob; i++)
    {
        delete[] startGoal.first[i];
        delete[] startGoal.second[i];
    }
    delete[] startGoal.first;
    delete[] startGoal.second;
    //delete PRMplanner;
    return 0;
}
