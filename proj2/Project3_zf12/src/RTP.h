///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: Zhenwei Feng 
//////////////////////////////////////

#ifndef RANDOM_TREE_H
#define RANDOM_TREE_H

#include <vector>
#include "ompl/geometric/planners/PlannerIncludes.h"

namespace ompl
{
    namespace geometric
    {
        // TODO: Implement RTP as described

        class RTP : public base::Planner
        {
        public:
            RTP(const base::SpaceInformationPtr &si);

            ~RTP() override;

            void getPlannerData(base::PlannerData &data) const override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void clear() override;
            
            // need this?
            void setup() override;

        protected:
            /** \brief Representation of a motion

                This only contains pointers to parent motions as we
                only need to go backwards in the tree. */
            class Motion
            {
            public:
                Motion() = default;

                /** \brief Constructor that allocates memory for the state */
                Motion(const base::SpaceInformationPtr &si) : state(si->allocState())
                {
                }

                ~Motion() = default;

                /** \brief The state contained by the motion */
                base::State *state{nullptr};

                /** \brief The parent motion in the exploration tree */
                Motion *parent{nullptr};
            };

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief State sampler */
            base::StateSamplerPtr sampler_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is
             * available) */
            double goalBias_{.05};

            /** \brief The random number generator */
            RNG rng_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion *lastGoalMotion_{nullptr};

            std::vector<Motion*> motions;

        };

    }  // namespace geometric
}  // namespace ompl

#endif
