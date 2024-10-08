///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: FILL ME OUT!!
//////////////////////////////////////

#include "RTP.h"
#include "ompl/base/ScopedState.h"
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/geometric/PathGeometric.h>

using namespace ompl::base;
using namespace ompl::geometric;


/* Calculate exact solution path following RTP algorithm.
    If timeout, calculates approximate path. */
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

    // populate with starts
    while (const State *s = pis_.nextStart())
    {
        if(si_->isValid(s))
        {
            // Inserts valid state space
            starts.push_back(ScopedState<>(si_->getStateSpace(), s));

            // No parent for start state, indicated by -1
            parents.push_back(-1);
        }
    }


    if (starts.size() == 0)
    {
        // No valid initial states
        return PlannerStatus::INVALID_START;
    }

    //ScopedState<> sample(si_), goal(si_);
    ScopedState<> qb(si_);

    // Runs until planner termination condition is met
    while (!ptc)
    {
        //uses rng to select random state
        size_t state_id = rng_.uniformInt(0, starts.size() - 1);

        ScopedState<> &qa = starts[state_id];

        //from rrt documentation, sample a random state, with small chance of qb being the goal
        /* sample random state (with goal biasing) */
        //if random number less than goal bias and goal state can be sampled, qb is goal state, else qb is sample
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
            
            // TODO: "can i plus in qb.get()?"
            //if goal state was just adfed to the tree, find path and return:
            //not sure parameters of isSatisfied, got from RRT but am not sure? can i plus in qb.get()
            // Check if qb satisfies goal
            if(goal_r->isSatisfied(qb.get()))
            {
                // Instantiate variable to store path
                auto path = std::make_shared<PathGeometric>(si_);

                // Add qb (the goal) to path
                path->append(qb.get());

                //set index i to the index of qb in starts
                //hoping that .back() correctly accesses last element in parents which should correspond to last element in starts, qb
                
                // Index of qb's parent
                size_t i = parents.back();

                // Construct path until root (parent idx = -1) is found
                while(i != -1)
                {
                    // Add state corresponding to start[i] to path
                    path->append(starts[i].get());

                    // Update index to parent corresponding to next
                    i = parents[i];
                }

                // Reverse path from goal-to-start to start-to-goal
                path->reverse();

                // TODO: double check "addSolutionPath" is necessary
                pdef_ ->addSolutionPath(path);

                return PlannerStatus::EXACT_SOLUTION;
            }
        }
    }

    // Since timeout, find closest state to goal in tree to approx sol path
    //double distance = si_->distance(starts[0].get(), goal_r.get());
    //need goal to be of type State not sampleable type
    //either pdef->getGoal() or goal_r?
    double distance = si_->distance(starts[0].get(), pdef->getGoal());
    size_t index = 0;
    for(size_t i = 1; i < starts.size(); i++)
    {
        double temp = si_->distance(starts[i].get(),  pdef->getGoal());
        if(temp < distance)
        {
            distance = temp;
            index = i;
        }
    }

    // Construct approx sol path from start to starts[closest to goal]
    auto path = std::make_shared<PathGeometric>(si_);
    path->append(starts[index].get());
    size_t i = parents[index];
    while(i != -1)
    {
        path->append(starts[i].get());
        i = parents[i];
    }

    path->reverse();
    // TODO: double check "addSolutionPath" is necessary
    //TODO: need to mark as approx vs exact?
    pdef_ ->addSolutionPath(path);
    return PlannerStatus::APPROXIMATE_SOLUTION;
}


// TODO: not sure if can leave as default or if need to add more layers (if so, adjust the header file too)
void RTP::clear() {

    //according to rrt, this is called if the input data to the planner has changed and we don't want to continue planning
    Planner::clear();

    //TO DO: one of the errors is bc sampler declared in solve() not the RTP class, not sure if needed to be declared in class, for now commenting it out
    //sampler.reset();
    //TO DO: one of the errors same as above, not sure if freememory() necessary
    //freeMemory();

    // Clear global vectors storing tree structure
    starts.clear();
    parents.clear();
}

// TODO: determine return, currently doing void
//pdf says it returns a planner data object, but rrt documentation doesnt return anything? 
//store all the vertexes statically
//include all of the states and create edges and vertices with them
//if can find solution from exercise 2 should be okay
//shared pointer should be automatically released 
void RTP::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    //now add vertices and edges! (from starts and parents vectors)
    //not just adding the path, adding all states and their edges

    //if returning, return data? 

    for(size_t i = 0; i < starts.size(); i++)
    {
        // Add start vertices
        if(parents[i] == -1)
        {
            data.addStartVertex(base::PlannerDataVertex(starts[i].get()));
        }
        else
        {
            // Add an edge from parent of i to i
            data.addEdge(base::PlannerDataVertex(starts[parents[i]].get()), base::PlannerDataVertex(starts[i].get()));
        }
    }

    //not sure if should be returning still?--> pdf says to so should for now
    // return data
}
