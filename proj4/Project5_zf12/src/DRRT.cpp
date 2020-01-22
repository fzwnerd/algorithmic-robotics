///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 5
// Authors: Zhenwei Feng zf12
//////////////////////////////////////

#include "DRRT.h"
#include <limits>
#include <cmath>
#include <queue>
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"


ompl::geometric::DRRT::DRRT(const base::SpaceInformationPtr &si, int numRob, const PRMptr& PRMplanner):
    base::Planner(si, "DRRT"), numRob_(numRob), PRMplanner_(std::move(PRMplanner))
{
    specs_.directed = true;
    //PRMplanner_ = std::make_shared<ompl::geometric::PRM>(*PRMplanner);
    //numRob_ = numRob;
    //PRMplanner_ = PRMplanner;
    //std::cout << this->getProblemDefinition()->getStartStateCount() << std::endl;
    //std::cout << numRob_ << std::endl;
}
/*
ompl::geometric::DRRT::DRRT(const base::SpaceInformationPtr &si, int numRob, const ompl::geometric::PRM* PRMplanner):
    base::Planner(si, "DRRT"), numRob_(numRob), PRMplanner_(PRMplanner)
{
    specs_.directed = true;
    //PRMplanner_ = std::make_shared<ompl::geometric::PRM>(*PRMplanner);
    //numRob_ = numRob;
    //PRMplanner_ = PRMplanner;
    //std::cout << this->getProblemDefinition()->getStartStateCount() << std::endl;
    //std::cout << numRob_ << std::endl;
}
*/
void ompl::geometric::DRRT::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
        for (auto &motion : addition)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }

    }
}

ompl::geometric::DRRT::~DRRT()
{
    freeMemory();
}

void ompl::geometric::DRRT::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
    addition.clear();
}

void ompl::geometric::DRRT::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    //sc.configurePlannerRange(maxDistance_);
 
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}

ompl::base::PlannerStatus ompl::geometric::DRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    //std::cout << approxdif << std::endl;
    //std::cout << "aa" << std::endl;
    checkValidity();
    const ompl::geometric::PRM::Graph& g = PRMplanner_->getRoadmap();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);
    //std::cout << pis_.haveMoreStartStates() << std::endl;
    // not all starting positions of robots are in roadmap, move to nearest point
    //const base::State *st = pdef_->getStartState(0);
    const base::State *st = pis_.nextStart();
    auto cst = st->as<ompl::base::CompoundState>();
    for (int i = 0; i < numRob_; i++)
    {
        auto r2state = cst->as<ompl::base::RealVectorStateSpace::StateType>(i);
        boost::graph_traits<ompl::geometric::PRM::Graph>::vertex_iterator j, end;
    
        boost::property_map<ompl::geometric::PRM::Graph, ompl::geometric::PRM::vertex_state_t>::const_type states_map = 
                                    boost::get(ompl::geometric::PRM::vertex_state_t(), g);

        double minDist = std::numeric_limits<double>::infinity();
        double tempX = 0;
        double tempY = 0;
        for (boost::tie(j, end) = boost::vertices(g); j != end; ++j)
        {
            ompl::base::State* prmst = boost::get(states_map, *j);
            auto prmr2state = prmst->as<ompl::base::RealVectorStateSpace::StateType>();
            double xdiff = r2state->values[0] - prmr2state->values[0];
            double ydiff = r2state->values[1] - prmr2state->values[1];
            double dist = std::pow(xdiff, 2) + std::pow(ydiff, 2);
            if (dist < minDist)
            {
                minDist = dist;
                tempX = prmr2state->values[0];
                tempY = prmr2state->values[1];
            }
        }
        r2state->values[0] = tempX;
        r2state->values[1] = tempY;
        //std::cout << r2state->values[0] << " " << r2state->values[1] << std::endl;
    }
    auto *motion = new Motion(si_);
    si_->copyState(motion->state, st);
    nn_->add(motion);
    /*
    
    while (const base::State *st = pis_.nextStart())
    {
        //std::cout << "bb" << std::endl;
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        nn_->add(motion);
    }
    */
    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();
    auto cxstate = xstate->as<ompl::base::CompoundState>();
    double dist = 0.0;

    while (!ptc)
    {
        if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);
        
        Motion *nmotion = nn_->nearest(rmotion);
        auto nstate = nmotion->state->as<ompl::base::CompoundState>();

        for (int i = 0; i < numRob_; i++)
        {
            auto rr2state = rstate->as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(i);
            auto nr2state = nstate->as<ompl::base::RealVectorStateSpace::StateType>(i);
            auto xr2state = cxstate->as<ompl::base::RealVectorStateSpace::StateType>(i);
            oracle(rr2state, nr2state, xr2state, g);
            //std::cout<< xr2state->values[0] << " " << xr2state->values[1] << std::endl;
        }
        //std::cerr <<"stop\n";
        //segmentatin error ?
        if (!si_->isValid(xstate))
            continue;
        /*
        auto cstate = xstate->as<ompl::base::CompoundState>(); 
        for (int i = 0; i < numRob_; i++)
        {
            auto r2state = cstate->as<ompl::base::RealVectorStateSpace::StateType>(i);
            double x = r2state->values[0];
            double y = r2state->values[1];
            std::cout << x << " " << y << std::endl; 
        }
        */
        /*
        std::vector<std::vector<double>> states;
        bool invalid = false;
        for (int i = 0; i < numRob_; i++)
        {
            auto r2state = cxstate->as<ompl::base::RealVectorStateSpace::StateType>(i);
            double x = r2state->values[0];
            double y = r2state->values[1];
            //if (!isValidCircle(x, y, radius, obstacles))
            //    return false;
            //std::cout << x << " " << y << std::endl;
            std::vector<double> temp{x, y};
            states.push_back(temp);
        }

        std::cout << states[0][0] << states[0][1] << std::endl 
                << states[1][0] << states[1][1] << std::endl
                << states[2][0] << states[2][1] << std::endl
                << states[3][0] << states[3][1] << std::endl;
        
        for (int i = 0; i < numRob_; i++)
        {
            for (int j = i + 1; j < numRob_; j++)
            {   
                //auto state1 = states[i];
                //auto state2 = states[j];
                double x1 = states[i][0];
                double y1 = states[i][1];
                double x2 = states[j][0];
                double y2 = states[j][1];
                //std::cout << x1 << " " << x2 << " " << y1 << " " << y2 << std::endl;
                double dist = std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
                if (dist <= 0.1) 
                {
                    invalid = true;
                    break;
                }
            }
            if (invalid)
                break;
        }
        //std::cout << "aaa" << std::endl;
        if (invalid) continue;
        */
        //if (!checkMotion(nmotion->state, xstate))
        //    continue;
        std::vector<int> order = localPlan(nmotion->state, xstate);
        // if no planning is found, do sampling again
        if (order.empty())
            continue;
        
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, xstate);
        motion->parent = nmotion;
        nn_->add(motion);

        nmotion = motion;

        //double dist = 0.0;
        bool sat = goal->isSatisfied(nmotion->state, &dist);
        if (sat)
        {
            approxdif = dist;
            solution = nmotion;
            break;
        }
        if (dist < approxdif)
        {
            approxdif = dist;
            approxsol = nmotion;
        }
        
    }
    //std::cout << dist << std::endl;
    bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;

        /* construct the solution path */
        std::vector<Motion *> mpath;

        base::State *tempSt = si_->allocState();

        while (solution != nullptr)
        {
            mpath.push_back(solution);
            //std::cout << "aa" << std::endl;
            if (solution->parent != nullptr)
            {
                si_->copyState(tempSt, solution->state);
                std::vector<int> order = localPlan(solution->parent->state, solution->state);
                for (int i = order.size() - 1; i >= 1; --i)
                {
                    PRMplanner_->getSpaceInformation()->copyState(
                        tempSt->as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(order[i]),
                        solution->parent->state->as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(order[i])                    
                    );
                    auto tempMt = new Motion(si_);
                    si_->copyState(tempMt->state, tempSt);
                    mpath.push_back(tempMt);
                    addition.push_back(tempMt);               
                }
            }
            solution = solution->parent;
        }

        si_->freeState(tempSt);

        /* set the solution path */
        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
        solved = true;
    }

    si_->freeState(xstate);
    if (rmotion->state != nullptr)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return {solved, approximate};

}

void ompl::geometric::DRRT::oracle(ompl::base::RealVectorStateSpace::StateType*& qrand, ompl::base::RealVectorStateSpace::StateType*& qnear, ompl::base::RealVectorStateSpace::StateType*& qnew,
                        const ompl::geometric::PRM::Graph& g)
{
    boost::graph_traits<ompl::geometric::PRM::Graph>::vertex_iterator i, end;
    
    boost::property_map<ompl::geometric::PRM::Graph, ompl::geometric::PRM::vertex_state_t>::const_type states_map = 
                                    boost::get(ompl::geometric::PRM::vertex_state_t(), g);
    //bool found = false;
    for (boost::tie(i, end) = boost::vertices(g); i != end; ++i)
    {
        //std::cout << i->values[0] << " " << i->values[1] << 
        //std::cout << boost::get(index_map, *i) << std::endl;
        ompl::base::State* st = boost::get(states_map, *i);
        auto r2state = st->as<ompl::base::RealVectorStateSpace::StateType>();
        //std::cout << r2state->values[0] << " " << r2state->values[1] << std::endl
        
        if (r2state->values[0] == qnear->values[0] && r2state->values[1] == qnear->values[1])
        {           
            //found = true;
            double minAngle = std::numeric_limits<double>::infinity();
            boost::graph_traits<ompl::geometric::PRM::Graph>::adjacency_iterator ai, aend;
            boost::tie(ai, aend) = boost::adjacent_vertices( *i, g);
            //if (ai == aend) std::cout << "aaa" <<std::endl;
            for (boost::tie(ai, aend) = boost::adjacent_vertices( *i, g); ai != aend; ai++)
            {
                auto adjr2state = boost::get(states_map, *ai)->as<ompl::base::RealVectorStateSpace::StateType>();
                double x1 = qrand->values[0] - qnear->values[0];
                double y1 = qrand->values[1] - qnear->values[1];
                double x2 = adjr2state->values[0] - qnear->values[0];
                double y2 = adjr2state->values[1] - qnear->values[1];
                //double angle = (r2state->values[0] * adjr2state->values[0] + r2state->values[1] * adjr2state->values[1])
                //               / (std::sqrt(std::pow(r2state->values[0], 2) + std::pow(r2state->values[1], 2))  *
                //                  std::sqrt(std::pow(adjr2state->values[0], 2) + std::pow(adjr2state->values[1], 2)));
                //std::cout << adjr2state->values[0] << std::endl;
                //std::cout << angle << std::endl;
                double angle = (x1 * x2 + y1 * y2) / std::sqrt(std::pow(x1, 2) + std::sqrt(std::pow(y1, 2))) /
                                                    std::sqrt(std::pow(x2, 2) + std::sqrt(std::pow(y2, 2)));
                if (angle < minAngle)
                {
                    minAngle = angle;
                    qnew->values[0] = adjr2state->values[0];
                    qnew->values[1] = adjr2state->values[1];
                    
                }
                
                //double angle = ()
            }
            break;
            
        }
        //std::cout << qnew->values[0] << " " << qnew->values[1] << std::endl;
        ///for (auto vd : boost::make_iterator_range(boost::adjacent_vertices( *i, g)))
        
    }
    //if (!found) std::cout << qnear->values[0] << " " << qnear->values[1] << std::endl;
}

bool ompl::geometric::DRRT::checkSingleRobPath(const ompl::base::State* start, const ompl::base::State* end, 
                            const ompl::base::State* st)
{
    //bool result = true;
    int nd = PRMplanner_->getSpaceInformation()->getStateSpace()->validSegmentCount(start, end);

    std::queue<std::pair<int, int>> pos;

    if (nd >= 2)
    {
        pos.push(std::make_pair(1, nd - 1));

        ompl::base::State *test = PRMplanner_->getSpaceInformation()->allocState();

        while (!pos.empty())
        {
            std::pair<int, int> x = pos.front();
 
            int mid = (x.first + x.second) / 2;

            PRMplanner_->getSpaceInformation()->getStateSpace()->interpolate(start, end, (double)mid / (double)nd, test);

            auto testr2 = test->as<ompl::base::RealVectorStateSpace::StateType>();
            auto xtr2 = st->as<ompl::base::RealVectorStateSpace::StateType>();

            double x1 = testr2->values[0];
            double y1 = testr2->values[1];
            double x2 = xtr2->values[0];
            double y2 = xtr2->values[1];
            double dist = std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
            
            // circle radius is 0.1
            if (dist <= 2 * 0.1) 
            {
                return false;
            }

            pos.pop();
 
            if (x.first < mid)
                pos.push(std::make_pair(x.first, mid - 1));
            if (x.second > mid)
                pos.push(std::make_pair(mid + 1, x.second));
        }
        PRMplanner_->getSpaceInformation()->freeState(test);
    }

    return true;
}

/*
bool ompl::geometric::DRRT::checkMotion(const ompl::base::State* start, const ompl::base::State* end)
{
    auto cstart = start->as<ompl::base::CompoundState>();
    auto cend = end->as<ompl::base::CompoundState>();
    //std::cout << "aaa" <<std::endl;
    for (int i = 0; i < numRob_; i++)
    {
        for (int j = 0; j < numRob_; j++)
        {
            if (i == j) continue;
            auto s1 = cstart->as<ompl::base::RealVectorStateSpace::StateType>(i);
            auto s2 = cend->as<ompl::base::RealVectorStateSpace::StateType>(i);
            auto s3 = cstart->as<ompl::base::RealVectorStateSpace::StateType>(j);
            auto s4 = cend->as<ompl::base::RealVectorStateSpace::StateType>(j);

            if ((!checkSingleRobPath(s1, s2, s3)) && (!checkSingleRobPath(s1, s2, s4)))
                return false;
        }
    }
    return true;
}
*/

std::vector<int> ompl::geometric::DRRT::localPlan(const ompl::base::State* start, const ompl::base::State* end)
{
    std::vector<std::vector<int>> graph(numRob_);
    auto cstart = start->as<ompl::base::CompoundState>();
    auto cend = end->as<ompl::base::CompoundState>();
    for (int i = 0; i < numRob_; i++)
    {
        for (int j = 0; j < numRob_; j++)
        {
            if (i == j) continue;
            auto s1 = cstart->as<ompl::base::RealVectorStateSpace::StateType>(i);
            auto s2 = cend->as<ompl::base::RealVectorStateSpace::StateType>(i);
            auto s3 = cstart->as<ompl::base::RealVectorStateSpace::StateType>(j);
            auto s4 = cend->as<ompl::base::RealVectorStateSpace::StateType>(j);

            //if ((!checkSingleRobPath(s1, s2, s3)) && (!checkSingleRobPath(s1, s2, s4)))
            //    return false;
            if (!checkSingleRobPath(s1, s2, s3))
                graph[j].push_back(i);   // robot j should move before i
            if (!checkSingleRobPath(s1, s2, s4))
                graph[i].push_back(j);   // robot i move before j
        }
    }

    std::vector<int> order;
    std::vector<bool> todo(numRob_, false), done(numRob_, false);

    for (int i = 0; i < numRob_; i++) 
    {
        if (!done[i] && !dfs(graph, todo, done, i, order)) 
        {
            return {};
        }
    }
    reverse(order.begin(), order.end());
    return order;
}

bool ompl::geometric::DRRT::dfs(const std::vector<std::vector<int>>& graph, std::vector<bool>& todo, 
            std::vector<bool>& done, int node, std::vector<int>& order)
{
    if (todo[node]) 
    {
        return false;
    }
    if (done[node]) 
    {
        return true;
    }
    todo[node] = done[node] = true;
    for (int neigh : graph[node]) {
        if (!dfs(graph, todo, done, neigh, order)) 
        {
            return false;
        }
    }
    order.push_back(node);
    todo[node] = false;
    return true;
}

void ompl::geometric::DRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    if (lastGoalMotion_ != nullptr)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
    }
}