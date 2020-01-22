///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: Zhenwei Feng 
//////////////////////////////////////

#include "RTP.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include <limits>
// TODO: Implement RTP as described
ompl::geometric::RTP::RTP(const base::SpaceInformationPtr &si) : base::Planner(si, "RTP")
{
    specs_.approximateSolutions = true;
    //specs_.
}

ompl::geometric::RTP::~RTP()
{
    freeMemory();
}

void ompl::geometric::RTP::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    motions.clear();
    lastGoalMotion_ = nullptr;
}

void ompl::geometric::RTP::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
}

void ompl::geometric::RTP::freeMemory()
{
    for (auto& motion : motions)
        delete motion;
}

ompl::base::PlannerStatus ompl::geometric::RTP::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motions.push_back(motion);
    }

    if (motions.size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states", getName().c_str(), motions.size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();

    while (!ptc)
    {
        if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        Motion *nmotion = motions[rng_.uniformInt(0, motions.size() - 1)];
        base::State *dstate = rstate;

        if (si_->checkMotion(nmotion->state, dstate))
        {
            auto *motion = new Motion(si_);
            si_->copyState(motion->state, dstate);
            motion->parent = nmotion;
            motions.push_back(motion);
            nmotion = motion;
        }

        double dist = 0.0;
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
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

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

    OMPL_INFORM("%s: Created %u states", getName().c_str(), motions.size());

    return {solved, approximate};
}

void ompl::geometric::RTP::getPlannerData(ompl::base::PlannerData &data) const
{
    Planner::getPlannerData(data);

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

