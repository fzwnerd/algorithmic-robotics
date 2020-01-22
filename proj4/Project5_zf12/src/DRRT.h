///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 5
// Authors: Zhenwei Feng
//////////////////////////////////////

#ifndef DISCRETE_TREE_H
#define DISCRETE_TREE_H
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include <vector>

using PRMptr = std::shared_ptr<ompl::geometric::PRM>;

namespace ompl
{
    namespace geometric
    {
        class DRRT : public base::Planner
        {
        public:
            DRRT(const base::SpaceInformationPtr &si, int numRob, const PRMptr& PRMplanner);
            //DRRT(const base::SpaceInformationPtr &si, int numRob, const ompl::geometric::PRM* PRMplanner);

            ~DRRT() override;

            void clear() override;

            void setup() override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void getPlannerData(base::PlannerData &data) const override;

        protected:
            class Motion
            {
            public:
                Motion() = default;
 
                Motion(const base::SpaceInformationPtr &si) : state(si->allocState())
                {
                }
 
                ~Motion() = default;
 
                base::State *state{nullptr};
 
                Motion *parent{nullptr};
            };

            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            void freeMemory();

            base::StateSamplerPtr sampler_;

            int numRob_;

            const PRMptr PRMplanner_;
            //const ompl::geometric::PRM* PRMplanner_;

            double goalBias_{.05};

            //double maxDistance_{0.};

            RNG rng_;

            //ompl::geometric::PRM::Graph g_;
            Motion *lastGoalMotion_{nullptr};

            void oracle(ompl::base::RealVectorStateSpace::StateType*& qrand, ompl::base::RealVectorStateSpace::StateType*& qnear, 
                        ompl::base::RealVectorStateSpace::StateType*& qnew, const ompl::geometric::PRM::Graph& g);

            // check if path strt->end would collide with st, if collide return false
            bool checkSingleRobPath(const ompl::base::State* start, const ompl::base::State* end, 
                            const ompl::base::State* st);

            // check if there is cycle in coorporate motions among robots, if infeasible return false
            //bool checkMotion(const ompl::base::State* start, const ompl::base::State* end);
            std::vector<int> localPlan(const ompl::base::State* start, const ompl::base::State* end);
            
            bool dfs(const std::vector<std::vector<int>>& g, std::vector<bool>& todo, std::vector<bool>& done, 
                        int node, std::vector<int>& order);

            std::vector<Motion*> addition;
                        
        };

    }  // namespace geometric
}  // namespace ompl

#endif
