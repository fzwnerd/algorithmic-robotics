///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Zhenwei Feng
//////////////////////////////////////

#include <iostream>
#include <cmath>
#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/config.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/tools/benchmark/Benchmark.h>
// The collision checker produced in project 2
#include "CollisionChecking.h"

// Your implementation of RG-RRT
#include "RG-RRT.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

const double PI = 3.1415926;

// Your projection for the car
class CarProjection : public ompl::base::ProjectionEvaluator
{
public:
    CarProjection(const ompl::base::StateSpacePtr& space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // TODO: The dimension of your projection for the car
        return 2;
    }

    virtual void defaultCellSizes(void)
    {
        cellSizes_.resize(2);
        cellSizes_[0] = 0.1;
        cellSizes_[1] = 0.1;
    }

    void project(const ompl::base::State * state, Eigen::Ref<Eigen::VectorXd>  projection) const override
    {
        // TODO: Your projection for the car
        auto cstate = state->as<ob::CompoundState>(); 

        // R3 
        auto r3state = cstate->as<ob::RealVectorStateSpace::StateType>(0);
        double x = r3state->values[0];
        double y = r3state->values[1];
        //double vel = r3state->values[2];

        // SO2
        //auto SO2state = cstate->as<ob::SO2StateSpace::StateType>(1);
        //double theta = SO2state->value;

        projection[0] = x;
        projection[1] = y;
        //projection[2] = theta;
    }
};

void carODE(const ompl::control::ODESolver::StateType &  q , const ompl::control::Control *  control ,
            ompl::control::ODESolver::StateType &  qdot )
{
    // TODO: Fill in the ODE for the car's dynamics
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;

    double omega = u[0];
    double acc = u[1]; // acceleration

    //double x = q[0];
    //double y = q[1];
    double v = q[2];
    double theta = q[3];

    qdot.resize(q.size(), 0);

    qdot[0] = v * std::cos(theta);
    qdot[1] = v * std::sin(theta);
    qdot[2] = acc;
    qdot[3] = omega;
}

void makeStreet(std::vector<Rectangle> &  obstacles )
{
    // TODO: Fill in the vector of rectangles with your street environment.
    Rectangle rect;

    // up
    rect.x = -5.0;
    rect.y = 6.0;
    rect.height = 2.0;
    rect.width = 10.0;
    obstacles.push_back(rect);

    // left
    rect.x = -5.0;
    rect.y = -4.0;
    rect.height = 7.0;
    rect.width = 3.0;
    obstacles.push_back(rect);
   
    // right
    rect.x = 2.5;
    rect.y = -4.0;
    rect.height = 7.0;
    rect.width = 2.5;
    obstacles.push_back(rect);
    
    // bottome
    rect.x = -5.0;
    rect.y = -10.0;
    rect.height = 4.0;
    rect.width = 10.0;
    obstacles.push_back(rect);
}

bool isCarStateValid(const oc::SpaceInformation *si, const ob::State *state, const std::vector<Rectangle> & obstacles)
{
    auto cstate = state->as<ompl::base::CompoundState>(); 
    
    auto r3state = cstate->as<ompl::base::RealVectorStateSpace::StateType>(0);

    double x = r3state->values[0];
    double y = r3state->values[1];

    return si->satisfiesBounds(state) && isValidPoint(x, y, obstacles);
}

void postCarPropagate(const ob::State* , const oc::Control* , const double , ob::State* result)
{
    ompl::base::SO2StateSpace SO2;
    // Ensure that the car's resulting orientation lies between 0 and 2*pi.
    auto s = result->as<ob::CompoundStateSpace::StateType>()->as<ob::SO2StateSpace::StateType>(1);
    //ompl::base::SE2StateSpace::StateType& s = *result->as<ompl::base::SE2StateSpace::StateType>();
    SO2.enforceBounds(s);
}

ompl::control::SimpleSetupPtr createCar(std::vector<Rectangle> & obstacles)
{
    // TODO: Create and setup the car's state space, control space, validity checker, everything you need for planning.
    // R3*s1(x, y, v, theta)
    ob::StateSpacePtr compSS;

    auto r3 = std::make_shared<ob::RealVectorStateSpace>(3);

    auto so2 = std::make_shared<ob::SO2StateSpace>();

    ob::RealVectorBounds bounds(3);

    // bounds on x
    bounds.setLow(0, -5);  
    bounds.setHigh(0, 5);

    // bounds on y
    bounds.setLow(1, -10);  
    bounds.setHigh(1, 10);

    // bounds on v
    bounds.setLow(2, -5);  
    bounds.setHigh(2, 5);

    r3->setBounds(bounds);

    // compound state space
    compSS = r3 + so2;

    // control space: angular velocity * acceleration
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(compSS, 2));

    ob::RealVectorBounds cbounds(2);

    // the range of both angular velocity and acceleration is [-5, 5]
    cbounds.setLow(-10);
    cbounds.setHigh(10);

    cspace->setBounds(cbounds);

    //oc::SimpleSetup ss(cspace);
    auto ssptr(std::make_shared<oc::SimpleSetup>(cspace));

    oc::SpaceInformation *si = ssptr->getSpaceInformation().get();
    ssptr->setStateValidityChecker(
        [si, obstacles](const ob::State *state) { return isCarStateValid(si, state, obstacles); });

    auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ssptr->getSpaceInformation(), &carODE));
    ssptr->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &postCarPropagate));
    //ssptr->getSpaceInformation()->setPropagationStepSize(0.2);
    ompl::base::ScopedState<> start(compSS);
    start[0] = -4.0;
    start[1] = -5.0;
    start[2] = 0.0;
    start[3] = 0.0;

    ompl::base::ScopedState<> goal(compSS);
    goal[0] = 3.5;
    goal[1] = 4.5;
    goal[2] = 0.0;
    goal[3] = 0.0;

    // set the start and goal states
    ssptr->setStartAndGoalStates(start, goal, 0.5);

    return ssptr;
}

void planCar(ompl::control::SimpleSetupPtr & ss , int  choice )
{
    // TODO: Do some motion planning for the car
    // choice is what planner to use.
    ob::PlannerPtr planner;
    auto space = ss->getStateSpace();
    switch (choice)
    {
    case 1:
        planner = std::make_shared<oc::RRT>(ss->getSpaceInformation());
        break;
    case 2:
        space->registerProjection("myProjection", ob::ProjectionEvaluatorPtr(new CarProjection(space)));
        planner = std::make_shared<oc::KPIECE1>(ss->getSpaceInformation());
        planner->as<oc::KPIECE1>()->setProjectionEvaluator("myProjection");
        break;
    default:
        planner = std::make_shared<oc::RGRRT>(ss->getSpaceInformation());
        break;
    }
    ss->setPlanner(planner);
    ss->setup();

    /// attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = ss->solve(120.0);

    if (solved)
    {
        //std::cout << "planner data:" << std::endl;
        //ompl::control::PlannerData pd(ss->getSpaceInformation());
        //planner->getPlannerData(pd);
        //pd.printGraphML();
        
        std::cout << "Found solution:" << std::endl;
        /// print the path to screen
        ss->getSolutionPath().asGeometric().printAsMatrix(std::cout);
        
    }
    else
        std::cout << "No solution found" << std::endl;

}

void benchmarkCar(ompl::control::SimpleSetupPtr & ss )
{
    // TODO: Do some benchmarking for the car
    ompl::tools::Benchmark b(*ss, "car experiment");

    b.addPlanner(ob::PlannerPtr(new oc::RRT(ss->getSpaceInformation())));
    ss->getStateSpace()->registerProjection("myProjection", ob::ProjectionEvaluatorPtr(new CarProjection(ss->getStateSpace())));
    auto KPplanner = std::make_shared<oc::KPIECE1>(ss->getSpaceInformation());
    KPplanner->as<oc::KPIECE1>()->setProjectionEvaluator("myProjection");
    b.addPlanner(KPplanner);
    b.addPlanner(ob::PlannerPtr(new oc::RGRRT(ss->getSpaceInformation())));

    ompl::tools::Benchmark::Request req;
    req.maxTime = 120.0;
    //req.maxMem = 1000.0;
    req.runCount = 50;
    req.displayProgress = true;
    b.benchmark(req);
    // This will generate a file of the form ompl_host_time.log
    b.saveResultsToFile();
}

int main(int /* argc */, char ** /* argv */)
{
    std::vector<Rectangle> obstacles;
    makeStreet(obstacles);

    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    ompl::control::SimpleSetupPtr ss = createCar(obstacles);

    // Planning
    if (choice == 1)
    {
        int planner;
        do
        {
            std::cout << "What Planner? " << std::endl;
            std::cout << " (1) RRT" << std::endl;
            std::cout << " (2) KPIECE1" << std::endl;
            std::cout << " (3) RG-RRT" << std::endl;

            std::cin >> planner;
        } while (planner < 1 || planner > 3);

        planCar(ss, planner);
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkCar(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}
