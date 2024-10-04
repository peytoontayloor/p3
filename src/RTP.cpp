///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: FILL ME OUT!!
//////////////////////////////////////

#include "RTP.h"
#include "ompl/base/ScopedState.h"
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/geometric/PathGeometric.h>

// TODO: Implement RTP as described
using namespace ompl::base;
using namespace ompl::geometric;

//solve is supposed to do approximate path!! --> need to implement this 

PlannerStatus RTP::solve(const PlannerTerminationCondition &ptc)
{
    //checks to make sure goal and start state defined correctly (i think)
    checkValidity();

    //grabs goal state from pdef
    auto goal_r = std::dynamic_pointer_cast<GoalSampleableRegion>(pdef_->getGoal());

    //do check like in rrt
    if(goal_r == nullptr)
    {
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }
    
    //state sampler for the state space:
    StateSamplerPtr sampler = si_->allocStateSampler();

    //create a vector to store the valid states: 
    //std::vector<ScopedState<>> starts;
    //create parents vector that essentially does parent[i] = k or starts[i]'s parent = k
    //parents stores the index of the parent of i in starts vector
    //std::vector<size_t> parents;

    //starts = [start, sample1, sample2]
    //parents = [nullptr, 0, 1]
    //indicates parent of start is none (root), parents of sample1 is start, and parent of sample2 is sample1.

    //populate with starts
    while (const State *s = pis_.nextStart())
    {
        if(si_->isValid(s))
        {
            //inserts state space into vector if it is valid
            starts.push_back(ScopedState<>(si_->getStateSpace(), s));
            //no parent for start state so doing -1
            parents.push_back(-1);
        }
    }

    if (starts.size() == 0)
    {
        //if no valid start states, return invalid
        return PlannerStatus::INVALID_START;
    }

    //ScopedState<> sample(si_), goal(si_);
    ScopedState<> qb(si_);

    //runs until planner termination condition met
    while (!ptc)
    {
        //uses rng to select random state
        size_t state_id = rng_.uniformInt(0, starts.size() - 1);

        ScopedState<> &qa = starts[state_id];

        //from rrt documentation, sample a random state, with small chance of qb being the goal
        /* sample random state (with goal biasing) */
        //if random number less than goal bias and goal state can be sampled, qb is goal state, else qb is sample
        //setting goal bias to 0.05 but not sure if that's a valid bias
        double goalBias_ = 0.05;
        if (rng_.uniform01() < goalBias_ && goal_r->canSample())
            goal_r->sampleGoal(qb.get());
        else
            sampler->sampleUniform(qb.get());

        //if valid path between qa and qb:
        if(si_->checkMotion(qa.get(), qb.get()))
        {
            //add new state to list of starts
            //so that will now sample from all start states including new path
            starts.push_back(ScopedState<>(si_->getStateSpace(), qb.get()));
            //the parent of new state qb, is the index of qa in starts
            //find index of qa and starts with loop (might be easier way?)
            for(size_t index = 0; index < starts.size(); index++)
            {
                if(starts[index] == qa)
                {
                    break;
                }
            }
            parents.push_back(index);

            //if goal state was just added to the tree, find path and return:
            //not sure parameters of isSatisfied, got from RRT but am not sure? can i plus in qb.get()
           if(goal_r->isSatisfied(qb.get()))
           {
               //need to get path and return it 
               auto path = std::make_shared<PathGeometric>(si_);
               //set index i to the index of qb in starts
               //hoping that .back() correctl accesses last element in parents which should correspond to last element in starts, qb
               size_t i = parents.back();
               //add qb aka goal to the path:
               path->append(qb.get());

               //now loop until at start, appending to the path: (-1 corresponds to no parent, aka root)
               while(i != -1)
               {
                   path->append(starts[i].get());
                   //update index to parent corresponding to next
                   i = parents[i];
               }
               //since did path from goal to start, need to reverse to get correct path
               path->reverse();
               return PlannerStatus::EXACT_SOLUTION;
           }

        }
    }

    //if timeout, return approximate path, which ends at the closest state to the goal in the tree
    //how to find closest state? -->spaceInfo::dist()
    double distance = si_->distance(starts[0].get(), goal_r.get());
    size_t index = 0;
    for(size_t i = 1; i < starts.size(); i++)
    {
        double temp = si_->distance(starts[i].get(), goal_r.get());
        if(temp < distance)
        {
            distance = temp;
            index = i;
        }
    }

    //now append path from start to starts[closest to goal]
    auto path = std::make_shared<PathGeometric>(si_);
    size_t i = parents[index];
    path->append(starts[index]);
    while(i != -1)
    {
        path->append(starts[i].get());
        i = parents[i]
    }

    path->reverse();
    return PlannerStatus::APPROXIMATE_SOLUTION;
    //return PlannerStatus::TIMEOUT;
}

//not sure if can leave as default or if need to add more layers (if so, adjust the header file too)
void RTP::clear() {

    //according to rrt, this is called if the input data to the planner has changed and we don't want to continue planning
    Planner::clear();
    sampler.reset();
    freeMemory();
    //need to clear and free my two global vectors!!
    starts.clear();
    parents.clear();
}

//pdf says it returns a planner data object, but rrt documentation doesnt return anything? 
//store all the vertexes statically
//include all of the states and create edges and vertices with them
//if can find solution from exercise 2 should be okay
//shared pointer should be automatically released 
PlannerData RTP::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    //now add vertices and edges! (from starts and parents vectors)
    //not just adding the path, adding all states and their edges

    //if returning, return data? 

    for(size_t i = 0; i < starts.size(); i++)
    {
        //add start vertex if no parents
        if(parents[i] == -1)
        {
            data.addStartVertex(base::PlannerDataVertex(starts[i]));
        }
        else
        {
            //add an edge with the starting with the parent of i and ending with i
            data.addEdge(base::PlannerDataVertex(starts[parents[i]]), base::PlannerDataVertex(starts[i]));
        }
    }

    //not sure if should be returning still?--> pdf says to so should for now
    return data
}
