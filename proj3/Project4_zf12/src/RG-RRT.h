///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Zhenwei Feng
//////////////////////////////////////

#ifndef RGRRT_H
#define RGRRT_H
#include <ompl/base/SpaceInformation.h>
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighborsLinear.h"
#include "ompl/control/ControlSpace.h"
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <vector>
#include <map>

namespace ompl
{
    namespace control
    {
        // TODO: Implement RGRRT as described

        class RGRRT : public base::Planner
        {
        public:
            RGRRT(const SpaceInformationPtr &si);

            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            /** \brief Get the goal bias the planner is using */
            double getGoalBias() const
            {
                return goalBias_;
            }

            void clear() override;

            void setup() override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void getPlannerData(base::PlannerData &data) const override;

            ~RGRRT() override;

        protected:
            class Motion
            {
            public:
                Motion() = default;

                /** \brief Constructor that allocates memory for the state and the control */
                Motion(const SpaceInformation *si)
                    :siC_{si}, state(si->allocState()), control(si->allocControl()), reachSet(std::vector<base::State*>())
                {
                }  

                ~Motion() = default;

                static const int reachStep = 1;

                const SpaceInformation *siC_;

                /** \brief The state contained by the motion */
                base::State *state{nullptr};

                /** \brief The control contained by the motion */
                Control *control{nullptr};

                /** \brief The number of steps the control is applied for */
                unsigned int steps{0};

                /** \brief The parent motion in the exploration tree */
                Motion *parent{nullptr};

                // reachability set of each motion
                std::vector<base::State*> reachSet;

                // map of reachability to control
                std::map<base::State*, control::RealVectorControlSpace::ControlType*> scMap;

                // get reachability set after state is known                
                void getReachSet();

                // get closest state in reachability set with known motion
                //base::State* getClosestInRS(Motion*);

                void freeReachSet();
            };

            

            void freeMemory();

            // a is in nearest data structure, b is q rand
            double distanceFunction(const Motion *a, const Motion *b) const;

            // get closest state in reachability set with known motion
            // a is in nearest data structure, b is q rand
            base::State* getClosestInRS(const Motion* a, const Motion *b) const;

            base::StateSamplerPtr sampler_;
 
            DirectedControlSamplerPtr controlSampler_;

            const SpaceInformation *siC_;

            //std::shared_ptr<NearestNeighborsLinear<Motion *>> nn_;
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            // increase the goal region sampling frequency to drag the tree to goal
            double goalBias_{0.05};

            /** \brief Flag indicating whether intermediate states are added to the built tree of motions */
            bool addIntermediateStates_{false};

            /** \brief The random number generator */
            RNG rng_;

            Motion *lastGoalMotion_{nullptr};
        };

    }  // namespace control 
}  // namespace ompl

#endif
