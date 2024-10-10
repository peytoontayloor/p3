///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: Swaha Roy and Peyton ElebashE
//////////////////////////////////////

#include "RTP.h"
#include "ompl/base/ScopedState.h"
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/geometric/PathGeometric.h>

#include <limits>
#define NO_PARENT std::numeric_limits<size_t>::max()

using namespace ompl::base;
using namespace ompl::geometric;

// Calculate exact solution path following RTP algorithm. If timeout, calculates approximate path.
PlannerStatus RTP::solve(const PlannerTerminationCondition &ptc)
{
    checkValidity();

    // Sets goal state from problem definition
    auto goal_r = std::dynamic_pointer_cast<GoalSampleableRegion>(pdef_->getGoal());

    if(goal_r == nullptr)
    {
        // Goal state undefined
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }
    
    // State sampler for the state space
    StateSamplerPtr sampler = si_->allocStateSampler();

    // Populate with starts
    while (const State *s = pis_.nextStart())
    {
        if(si_->isValid(s))
        {
            // Inserts valid state space
            starts.push_back(ScopedState<>(si_->getStateSpace(), s));

            // No parent for start state, indicated by -1
            parents.push_back(NO_PARENT);
        }
    }

    if (starts.size() == 0)
    {
        // No valid initial states
        return PlannerStatus::INVALID_START;
    }

   
    ScopedState<> qb(si_);
    size_t approxidx;
    double approxdif = std::numeric_limits<double>::infinity();

    // Runs until planner termination condition is met
    while (!ptc)
    {
        // Select random state
        size_t state_id = rng_.uniformInt(0, starts.size() - 1);

        ScopedState<> &qa = starts[state_id];

        // Sample a random state, with small chance of qb being the goal
        double goalBias_ = 0.05;
        if (rng_.uniform01() < goalBias_ && goal_r->canSample())
            goal_r->sampleGoal(qb.get());
        else
            sampler->sampleUniform(qb.get());

        // If valid path between qa and qb:
        if(si_->checkMotion(qa.get(), qb.get()))
        {
            // Add new state to starts, future samples can now sample the new path
            starts.push_back(ScopedState<>(si_->getStateSpace(), qb.get()));

            // Assign index of qa in starts to be parent of qb
            parents.push_back(state_id);
            
            // Instantiate variable to store path
            auto path = std::make_shared<PathGeometric>(si_);

            // Add qb (the goal) to path
            path->append(qb.get());

            // Index of qb's parent
            size_t i = parents.back();

            // Construct path until root is found
            while(i != NO_PARENT)
            {
                path->append(starts[i].get());

                // Update index to parent corresponding to next
                i = parents[i];
            }
            // Reverse path from goal-to-start to start-to-goal
            path->reverse();

            // Check if qb satisfies goal
            double dist = 0.0;
            bool sat = goal_r->isSatisfied(qb.get(), &dist);
            
            if (sat) 
            {
                approxdif = dist;
                pdef_ ->addSolutionPath(path, false, 0.0, "RTP");
                return PlannerStatus::EXACT_SOLUTION;
            }

            if (dist < approxdif)
            {
                approxdif = dist;       // distance between qb and goal
                approxidx = state_id;   // qb's start index
            }
           
        }
    }

    // Since timeout, construct approx solution from start to starts[approxidx]
    auto path = std::make_shared<PathGeometric>(si_);
    path->append(starts[approxidx].get());
    size_t i = parents[approxidx];
    while(i != NO_PARENT)
    {
        path->append(starts[i].get());
        i = parents[i];
    }
    path->reverse();
    pdef_ ->addSolutionPath(path, true, approxdif, "RTP");
    return PlannerStatus::APPROXIMATE_SOLUTION;
}

void RTP::clear() {

    // Clear internal planner datastructures
    Planner::clear();

    // Clear global vectors storing tree structure
    starts.clear();
    parents.clear();
}

// Store all the vertexes statically, include all of the states and create edges and vertices with them
void RTP::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    // Add vertices and edge from starts and parents vectors
    // Not just adding the path, adding all states and their edges
    for(size_t i = 0; i < starts.size(); i++)
    {
        // Add start vertices
        if(parents[i] == NO_PARENT)
        {
            data.addStartVertex(base::PlannerDataVertex(starts[i].get()));
        }
        else
        {
            // Add an edge from parent of i to i
            data.addEdge(base::PlannerDataVertex(starts[parents[i]].get()), base::PlannerDataVertex(starts[i].get()));
        }
    }
}
