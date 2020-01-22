///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Zhenwei Feng
//////////////////////////////////////

#include <iostream>
#include <cmath>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/config.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/tools/benchmark/Benchmark.h>
// Your implementation of RG-RRT
#include "RG-RRT.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

const double PI = 3.1415926;

// Your projection for the pendulum
class PendulumProjection : public ompl::base::ProjectionEvaluator
{
public:
    PendulumProjection(const ompl::base::StateSpacePtr&  space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // TODO: The dimension of your projection for the pendulum
        return 1;
    }
    /*
    virtual void defaultCellSizes(void)
    {
        cellSizes_.resize(1);
        cellSizes_[0] = 0.1;
        //cellSizes_[1] = 0.25;
    }
    */
    void project(const ompl::base::State * state , Eigen::Ref<Eigen::VectorXd>  projection ) const override
    {
        // TODO: Your projection for the pendulum
        auto cstate = state->as<ompl::base::CompoundState>();       

        // Extract theta (SO(2))
        auto so2State = cstate->as<ompl::base::SO2StateSpace::StateType>(0);
        double theta = so2State->value;

        //auto r1state = cstate->as<ompl::base::RealVectorStateSpace::StateType>(1);
        //double rotVel = r1state->values[0];

        projection(0) = theta;
        //projection(1) = rotVel;
    }
};

void pendulumODE(const ompl::control::ODESolver::StateType & q , const ompl::control::Control * control ,
                 ompl::control::ODESolver::StateType & qdot )
{
    // TODO: Fill in the ODE for the pendulum's dynamics
    // control space only has torque
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    double torque = u[0];

    // state of pendulum from S1*R1
    double theta = q[0];
    double rotVel = q[1];

    qdot.resize(q.size(), 0);

    qdot[0] = rotVel;            
    qdot[1] = -9.81 * cos(theta) + torque;   
}

bool isPendulumStateValid(const oc::SpaceInformation *si, const ob::State *state)
{
    //auto cstate = state->as<ompl::base::CompoundState>();
    return si->satisfiesBounds(state);
}

void postPendulumPropagate(const ob::State* , const oc::Control* , const double , ob::State* result)
{
    ompl::base::SO2StateSpace SO2;
    // Ensure that the car's resulting orientation lies between 0 and 2*pi.
    auto s = result->as<ob::CompoundStateSpace::StateType>()->as<ob::SO2StateSpace::StateType>(0);
    //ompl::base::SE2StateSpace::StateType& s = *result->as<ompl::base::SE2StateSpace::StateType>();
    SO2.enforceBounds(s);
}

ompl::control::SimpleSetupPtr createPendulum(double  torque)
{
    // TODO: Create and setup the pendulum's state space, control space, validity checker, everything you need for
    // planning.

    // S1*R1
    ob::StateSpacePtr compSS;

    auto so2 = std::make_shared<ob::SO2StateSpace>();

    auto r1 = std::make_shared<ob::RealVectorStateSpace>(1);

    // bounds on rotational velocity
    ob::RealVectorBounds bounds(1);
    bounds.setLow(-10);  
    bounds.setHigh(10);  

    // Set the bounds on R2
    r1->setBounds(bounds);

    compSS = so2 + r1;

    // control space
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(compSS, 1));
    ob::RealVectorBounds cbounds(1);
    cbounds.setLow(-torque);
    cbounds.setHigh(torque);

    cspace->setBounds(cbounds);

    //oc::SimpleSetup ss(cspace);
    auto ssptr(std::make_shared<oc::SimpleSetup>(cspace));

    oc::SpaceInformation *si = ssptr->getSpaceInformation().get();
    ssptr->setStateValidityChecker(
        [si](const ob::State *state) { return isPendulumStateValid(si, state); });

    auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ssptr->getSpaceInformation(), &pendulumODE));
    //ssptr->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &postPendulumPropagate));
    ssptr->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver));
    ompl::base::ScopedState<> start(compSS);
    start[0] = -PI / 2;
    start[1] = 0.0;

    ompl::base::ScopedState<> goal(compSS);
    goal[0] = PI / 2;
    goal[1] = 0.0;

    // set the start and goal states
    ssptr->setStartAndGoalStates(start, goal, 0.05);

    return ssptr;

    
}

void planPendulum(ompl::control::SimpleSetupPtr &  ss , int  choice )
{
    // TODO: Do some motion planning for the pendulum
    // choice is what planner to use.
    ob::PlannerPtr planner;
    auto space = ss->getStateSpace();
    switch (choice)
    {
    case 1:
        planner = std::make_shared<oc::RRT>(ss->getSpaceInformation());
        break;
    case 2:
        space->registerProjection("myProjection", ob::ProjectionEvaluatorPtr(new PendulumProjection(space)));
        planner = std::make_shared<oc::KPIECE1>(ss->getSpaceInformation());
        planner->as<oc::KPIECE1>()->setProjectionEvaluator("myProjection");
        //space->registerDefaultProjection(ob::ProjectionEvaluatorPtr(new PendulumProjection(space)));
        break;
    default:
        planner = std::make_shared<oc::RGRRT>(ss->getSpaceInformation());
        break;
    }
    ss->setPlanner(planner);
    ss->setup();

    /// attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = ss->solve(20.0);

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


    //return nullptr;
}

void benchmarkPendulum(ompl::control::SimpleSetupPtr & ss )
{
    // TODO: Do some benchmarking for the pendulum
    ompl::tools::Benchmark b(*ss, "pendulum experiment");

    b.addPlanner(ob::PlannerPtr(new oc::RRT(ss->getSpaceInformation())));
    ss->getStateSpace()->registerProjection("myProjection", ob::ProjectionEvaluatorPtr(new PendulumProjection(ss->getStateSpace())));
    auto KPplanner = std::make_shared<oc::KPIECE1>(ss->getSpaceInformation());
    KPplanner->as<oc::KPIECE1>()->setProjectionEvaluator("myProjection");
    b.addPlanner(KPplanner);
    b.addPlanner(ob::PlannerPtr(new oc::RGRRT(ss->getSpaceInformation())));

    ompl::tools::Benchmark::Request req;
    req.maxTime = 20.0;
    //req.maxMem = 1000.0;
    req.runCount = 50;
    req.displayProgress = true;
    b.benchmark(req);
    // This will generate a file of the form ompl_host_time.log
    b.saveResultsToFile();
}

int main(int /* argc */, char ** /* argv */)
{
    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    int which;
    do
    {
        std::cout << "Torque? " << std::endl;
        std::cout << " (1)  3" << std::endl;
        std::cout << " (2)  5" << std::endl;
        std::cout << " (3) 10" << std::endl;

        std::cin >> which;
    } while (which < 1 || which > 3);

    double torques[] = {3., 5., 10.};
    double torque = torques[which - 1];

    ompl::control::SimpleSetupPtr ss = createPendulum(torque);

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

        planPendulum(ss, planner);
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkPendulum(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}
