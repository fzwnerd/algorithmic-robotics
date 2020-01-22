///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: Zhenwei Feng
//////////////////////////////////////

#include <iostream>
#include "ompl/tools/benchmark/Benchmark.h"
#include "omplapp/apps/SE3RigidBodyPlanning.h"
#include "omplapp/config.h"
#include "ompl/geometric/planners/rrt/RRT.h"
#include "ompl/geometric/planners/prm/PRM.h"
#include "ompl/geometric/planners/est/EST.h"
// Your random tree planner
#include "RTP.h"

using namespace ompl;

void benchmarkCubicles()
{
    app::SE3RigidBodyPlanning setup;

    // load the robot and the environment
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/cubicles_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/cubicles_env.dae";
    setup.setRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);

    // define start state
    base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(-4.96);
    start->setY(-40.62);
    start->setZ(70.57);
    start->rotation().setIdentity();

    // define goal state
    base::ScopedState<base::SE3StateSpace> goal(start);
    goal->setX(200.49);
    goal->setY(-40.62);
    goal->setZ(70.57);
    goal->rotation().setIdentity();

    setup.setStartAndGoalStates(start, goal);

    // setting collision checking resolution to 1% of the space extent
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

    setup.setup();

    ompl::tools::Benchmark b(setup, "Cubicle experiment");

    b.addPlanner(base::PlannerPtr(new geometric::RTP(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new geometric::RRT(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new geometric::PRM(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new geometric::EST(setup.getSpaceInformation())));

    ompl::tools::Benchmark::Request req;
    req.maxTime = 60.0;
    req.maxMem = 1000.0;
    req.runCount = 50;
    req.displayProgress = true;
    b.benchmark(req);

    b.saveResultsToFile();
}

void benchmarkTwistycool()
{
    // TODO
    app::SE3RigidBodyPlanning setup;

    // load the robot and the environment
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Twistycool_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Twistycool_env.dae";
    setup.setRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);

    // define start state
    base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(270.0);
    start->setY(160.0);
    start->setZ(-200.0);
    start->rotation().setIdentity();

    // define goal state
    base::ScopedState<base::SE3StateSpace> goal(start);
    goal->setX(270.0);
    goal->setY(160.0);
    goal->setZ(-400.0);
    goal->rotation().setIdentity();

    setup.setStartAndGoalStates(start, goal);

    // setting collision checking resolution to 1% of the space extent
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

    setup.setup();

    ompl::tools::Benchmark b(setup, "Twistycool experiment");

    b.addPlanner(base::PlannerPtr(new geometric::RTP(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new geometric::RRT(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new geometric::PRM(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new geometric::EST(setup.getSpaceInformation())));

    ompl::tools::Benchmark::Request req;
    req.maxTime = 125.0;
    req.maxMem = 1000.0;
    req.runCount = 50;
    req.displayProgress = true;
    b.benchmark(req);

    b.saveResultsToFile();
}

int main(int /* argc */, char ** /* argv */)
{
    int environment;

    do
    {
        std::cout << "Benchmark for: " << std::endl;
        std::cout << " (1) Cubicles" << std::endl;
        std::cout << " (2) Twistycool" << std::endl;

        std::cin >> environment;
    } while (environment < 1 || environment > 2);

    switch (environment)
    {
        case 1:
            benchmarkCubicles();
            break;
        case 2:
            benchmarkTwistycool();
            break;
        default:
            std::cerr << "Invalid Environment!" << std::endl;
            break;
    }

    return 0;
}
