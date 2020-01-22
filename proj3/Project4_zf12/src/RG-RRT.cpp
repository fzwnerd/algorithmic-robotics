///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Zhenwei Feng 
//////////////////////////////////////

#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>
#include <cmath>
#include <iostream>
#include <ompl/control/PlannerData.h>
#include "RG-RRT.h"

// TODO: Implement RGRRT as described
ompl::control::RGRRT::RGRRT(const SpaceInformationPtr &si) : base::Planner(si, "RGRRT")
{
    specs_.approximateSolutions = true;
    siC_ = si.get();

    Planner::declareParam<double>("goal_bias", this, &RGRRT::setGoalBias, &RGRRT::getGoalBias, "0.:.05:1.");
    //Planner::declareParam<bool>("intermediate_states", this, &RRT::setIntermediateStates, &RRT::getIntermediateStates);
}

void ompl::control::RGRRT::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            // free reachability set
            if (motion->reachSet.size() > 0)
                motion->freeReachSet();
            if (motion->state)
                si_->freeState(motion->state);
            if (motion->control)
                siC_->freeControl(motion->control);
            delete motion;
        }
    }
}

double ompl::control::RGRRT::distanceFunction(const ompl::control::RGRRT::Motion *a, const ompl::control::RGRRT::Motion *b) const
{
    /*
    double aTob = siC_->distance(a->state, b->state);
    //double aRSTob = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < a->reachSet.size(); i++)
    {
        ompl::base::State* st = a->reachSet[i];
        if (siC_->distance(st, b->state) < aTob)
            return aTob;
    }
    return std::numeric_limits<double>::infinity();
    */
   return si_->distance(a->state, b->state);
}

ompl::control::RGRRT::~RGRRT()
{
    freeMemory();
}

void ompl::control::RGRRT::clear()
{
    Planner::clear();
    sampler_.reset();
    controlSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
}

void ompl::control::RGRRT::setup()
{
    base::Planner::setup();
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
        //nn_.reset(new NearestNeighborsLinear<Motion*>());
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b)
                             {
                                 return distanceFunction(a, b);
                             });
}

ompl::base::PlannerStatus ompl::control::RGRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);
 
    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(siC_);
        si_->copyState(motion->state, st);
        
        siC_->nullControl(motion->control);
        motion->getReachSet();
        nn_->add(motion);   
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }
 
    if (!sampler_)
        sampler_ = si_->allocStateSampler();
    if (!controlSampler_)
        controlSampler_ = siC_->allocDirectedControlSampler();
 
    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());
 
    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();

    auto *rmotion = new Motion(siC_);
    base::State *rstate = rmotion->state;
    //Control *rctrl = rmotion->control;
    base::State *xstate = si_->allocState();

    while (ptc == false)
    {
        /* sample random state (with goal biasing) */
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
        //if (goal_s && rng_.uniform01() < goalBias_)
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        Motion *nmotion = nn_->nearest(rmotion);

        //if (std::isinf(distanceFunction(nmotion, rmotion)))
        //    continue;

        // if nmotion not satisfy reachability set condition, get a new rstate
        base::State* closestReachInNmotion = getClosestInRS(nmotion, rmotion);
        if (closestReachInNmotion == nullptr)
            continue;

        auto *motion = new Motion(siC_);
        si_->copyState(motion->state, closestReachInNmotion);
        motion->getReachSet();
        siC_->copyControl(motion->control, nmotion->scMap[closestReachInNmotion]);
        motion->steps = Motion::reachStep;
        motion->parent = nmotion;

        nn_->add(motion);
        double dist = 0.0;
        bool solv = goal->isSatisfied(motion->state, &dist);

        if (solv)
        {
            approxdif = dist;
            solution = motion;
            break;
        }
        if (dist < approxdif)
        {
            approxdif = dist;
            approxsol = motion;
        }
    }

    bool solved = false;
    bool approximate = false;
    //std::cout << solved << " " << approximate << std::endl;
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
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }
 
         /* set the solution path */
        auto path(std::make_shared<PathControl>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            if (mpath[i]->parent)
                path->append(mpath[i]->state, mpath[i]->control, mpath[i]->steps * siC_->getPropagationStepSize());
                //path->append(mpath[i]->state, mpath[i]->control, 1);
            else
                path->append(mpath[i]->state);
        solved = true;
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
    }

    if (rmotion->reachSet.size() > 0)
        rmotion->freeReachSet(); 
    if (rmotion->state)
        si_->freeState(rmotion->state);
    if (rmotion->control)
        siC_->freeControl(rmotion->control);
    delete rmotion;
    si_->freeState(xstate);

    //std::cout << "planer data:" << std::endl;
    //auto pd = ompl::control::PlannerData(std::shared_ptr<ompl::control::SpaceInformation>(siC_));
    //pd.printGraphviz(std::cout);
 
    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());
 
    return base::PlannerStatus(solved, approximate);
}


void ompl::control::RGRRT::Motion::getReachSet()
{
    if (reachSet.size() > 0)
        return;
    
    const unsigned int nums = 41;
                
    // get control space
    ControlSpacePtr csp = siC_->getControlSpace();

    // cast to real vector control space
    auto vecSp = csp->as<RealVectorControlSpace>();

    // dimension of control space
    unsigned int dimSp = vecSp->getDimension();

    // bounds of control space
    const base::RealVectorBounds& bounds = vecSp->getBounds();

    //double low1 = bounds.low[0];
    //double high1 = bounds.high[0];

    // each vector represents possible values of each dimension of control space
    std::vector<std::vector<double>> vals;

    for (unsigned int i = 0; i < dimSp; i++)
    {
        std::vector<double> res;
        double low = bounds.low[i];
        double high = bounds.high[i];
        for (unsigned int j = 1; j <= nums; j++)
        { 
            double val = low + (high - low) / (nums - 1) * (nums - j);
            res.push_back(val);
        }
        vals.push_back(res);
    }
    //}

    if (dimSp == 1)
    {
        for (auto i : vals[0])
        {
            auto vecContr = vecSp->allocControl()->as<RealVectorControlSpace::ControlType>();
            vecContr->values[0] = i;
            base::State* reachSt = siC_->allocState();

            siC_->propagate(state, vecContr, Motion::reachStep, reachSt);

            if (siC_->isValid(reachSt))
            {
                reachSet.push_back(reachSt);
                scMap[reachSt] = vecContr;
            }
            else
            {
                siC_->freeState(reachSt);
                siC_->freeControl(vecContr);
            }
        }
    }
    else if (dimSp == 2)
    {
        for (auto i : vals[0])
        {
            for (auto j : vals[1])
            {
                auto vecContr = vecSp->allocControl()->as<RealVectorControlSpace::ControlType>();
                vecContr->values[0] = i;
                vecContr->values[1] = j;
                base::State* reachSt = siC_->allocState();
                siC_->propagate(state, vecContr, Motion::reachStep, reachSt);

                if (siC_->isValid(reachSt))
                {
                    reachSet.push_back(reachSt);
                    scMap[reachSt] = vecContr;
                }
                else
                {
                    siC_->freeState(reachSt);
                    siC_->freeControl(vecContr);
                }
            }
        }
    }
    /*
    for (unsigned int i = 1; i <= nums; i++)
    {
        auto vecContr = vecSp->allocControl()->as<RealVectorControlSpace::ControlType>();

        for (unsigned int j = 0; j < dimSp; j++)
        {
            double low = bounds.low[j];
            double high = bounds.high[j];
            vecContr->values[j] = low + (high - low) / (nums - 1) * (nums - j);
        }

        base::State* reachSt = siC_->allocState();

        siC_->propagate(state, vecContr, Motion::reachStep, reachSt);

        if (siC_->isValid(reachSt))
        {
            reachSet.push_back(reachSt);
            scMap[reachSt] = vecContr;
        }
        else
        {
            siC_->freeState(reachSt);
            siC_->freeControl(vecContr);
        }
    }
    */
    //std::cout << reachSet.size() << endl;
}

void ompl::control::RGRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);
 
    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);
 
    double delta = siC_->getPropagationStepSize();
 
    if (lastGoalMotion_)
         data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));
 
    for (auto m : motions)
    {
        if (m->parent)
        {
            if (data.hasControls())
                data.addEdge(base::PlannerDataVertex(m->parent->state), base::PlannerDataVertex(m->state),
                              control::PlannerDataEdgeControl(m->control, m->steps * delta));
            else
                data.addEdge(base::PlannerDataVertex(m->parent->state), base::PlannerDataVertex(m->state));
        }
        else
            data.addStartVertex(base::PlannerDataVertex(m->state));
     }
 }

/*
ompl::base::State* ompl::control::RGRRT::Motion::getClosestInRS(ompl::control::RGRRT::Motion* mt)
{
    ompl::base::State* st = mt->state;
    double distToSelfSt = 
    ompl::base::State* closestSt = reachSet[0];
    double minDist = siC_->distance(closestSt, st);
    for (std::size_t i = 1; i < reachSet.size(); i++)
    {
        double dist = siC_->distance(reachSet[i], st);
        if (dist < minDist)
        {
            minDist = dist;
            closestSt = reachSet[i];
        }
    }
    return closestSt;
}
*/

ompl::base::State* ompl::control::RGRRT::getClosestInRS(const Motion* a, const Motion *b) const
{
    ompl::base::State* as = a->state;
    ompl::base::State* bs = b->state;
    double distAtoB = si_->distance(as, bs);
    ompl::base::State* res = nullptr;
    double minDist = distAtoB;
    for (const auto st : a->reachSet)
    {
        if (si_->distance(st, b->state) < minDist)
        {
            minDist = si_->distance(st, b->state);
            res = st;
        }
    }
    return res;
}

void ompl::control::RGRRT::Motion::freeReachSet()
{
    for (auto& p : scMap)
    {
        siC_->freeState(p.first);
        siC_->freeControl(p.second);
    }
    scMap.clear();
    reachSet.clear();  

    // exmpty reachability set and map

}


