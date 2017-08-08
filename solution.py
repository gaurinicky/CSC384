#Look for #IMPLEMENT tags in this file. These tags indicate what has
#to be implemented to complete the Snowman Puzzle domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

# import os for time functions
import os
from search import * #for search engines
from snowman import SnowmanState, Direction, snowman_goal_state #for snowball specific classes and problems
from test_problems import PROBLEMS #20 test problems

##my imports
import math
from timeit import default_timer as timer
import sys

#snowball HEURISTICS
def heur_simple(state):
  '''trivial admissible snowball heuristic'''
  '''INPUT: a snowball state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state (# of moves required to get) to the goal.'''
  return len(state.snowballs)

def heur_zero(state):
  return 0

def heur_manhattan_distance(state):
#IMPLEMENT
    '''admissible snowball puzzle heuristic: manhattan distance'''
    '''INPUT: a snowball state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #We want an admissible heuristic, which is an optimistic heuristic.
    #It must always underestimate the cost to get from the current state to the goal.
    #The sum of the Manhattan distances between the snowballs and the destination for the Snowman is such a heuristic.
    #When calculating distances, assume there are no obstacles on the grid.
    #You should implement this heuristic function exactly, even if it is tempting to improve it.
    #Your function should return a numeric value; this is the estimate of the distance to the goal.

    #Thought process-  give distance of the state specified
    #Manhattan Distance = Math.abs(x1-x0) + Math.abs (y1-y0)
    #get goal co-ordinates (x0, y0) and get location of the state coordinates
    # (x1, y1)
    #goal is state.destination -- gives us the coordinates

    #initialize distance
    distance = 0
    for snow in state.snowballs:
        distance += abs(snow[0] - state.destination[0]) + abs(snow[1] - state.destination[1])
            #print ("Goal is", state.destination)

    return distance


def heur_alternate(state):
#IMPLEMENT
    '''a better heuristic'''
    '''INPUT: a snowball state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #heur_manhattan_distance has flaws.
    #Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    #Your function should return a numeric value for the estimate of the distance to the goal.


    #initialize variables
    distance = 0
    man = 0
    min_distance = 0
    height = state.height - 1
    width = state.width - 1
    snow = state.snowballs

    for index in state.snowballs:
      #excluding corner cases when the snowball is not at the destination
      if index != state.destination:
        #snowball is at the corner of the board
        top = (0, height)
        bottom = (0, 0)
        top_right = (width, height)
        bottom_right = (width, 0)
        if index == top and state.destination != top:
            return float("inf")
        elif index == bottom and state.destination != bottom:
            return float("inf")
        elif index == top_right and state.destination != top_right:
            return float("inf")
        elif index == bottom_right and state.destination != bottom_right:
            return float("inf")
        elif index == top_right:
            return float("inf")
        elif index == top:
            return float("inf")
        elif index == bottom_right:
            return float("inf")
        elif index == bottom:
            return float("inf")

        #located on the wall

        if(state.destination[0] == 0 and index[0] == width):
            return float("inf")
        elif(state.destination[0] == width) and index[0] == 0:
            return float("inf")
        elif(state.destination[1] == 0) and index[1] == height:
            return float("inf")
        elif(state.destination[1] == height) and index[1] == 0:
            return float("inf")
        elif(state.destination[0] != 0) and (index[0] == 0):
            return float("inf")
        elif(state.destination[1] != 0) and (index[1] == 0):
            return float("inf")
        elif (state.destination[0] != width) and (index[0] == width):
            return float("inf")
        elif (state.destination[1] != height) and (index[1] == height):
            return float("inf")

        #if 3 snowballs are together stack two and push them to a goal then take third to goal
        #bms, bsm, sbm, smb, msb, mbs
        #0, 3, 6
        if (snow == state.destination):
            if(snow[index] != 0):
                return float("inf")
            elif(snow[index] != 3):
                return float("inf")
            elif(snow[index] != 6):
                return float("inf")

        #snowball between obstacles
        if(index[0]+1, index[1]) in state.obstacles and (index[0], index[1]+1) in state.obstacles:
            return float("inf")
        elif(index[0]-1, index[1]) in state.obstacles and (index[0], index[1]-1) in state.obstacles:
            return float("inf")
        elif(index[0]+1, index[1]) in state.obstacles and (index[0], index[1]-1) in state.obstacles:
            return float("inf")
        elif(index[0]-1, index[1]) in state.obstacles and (index[0], index[1]+1) in state.obstacles:
            return float("inf")

        #corner and obstacles
        if (index[0] == 0):
            if (0, index[1]+1) in state.obstacles or (0, index[1]-1) in state.obstacles:
                return float('inf')
    #    if (index[0] == height):
    #        if(height, index[1]+1) in state.obstacles or (0, index[1]-1) in state.obstacles:
    #            return float("inf")
        if (index[1] == 0):
            if(index[0]+1, 0) in state.obstacles or (index[0]-1, 0) in state.obstacles:
                return float("inf")
        if (index[1] == width):
            if(index[0]+1, width) in state.obstacles or (index[0]-1, width) in state.obstacles:
                return float("inf")



      # for snowballs that are on top one another we have to increase the distance - since it requires two pushes

      if snow[index] >= 3:
    #B C times 2, A not on goal times 3, A on goal 0


        #if it is A and on the goal, we don't need to move it
        #distance += ((abs(index[0] - state.destination[0])) * (abs(index[0] - state.destination[0])) + (abs(index[1] - state.destination[1])) * (abs(index[1] - state.destination[1])))
        #distance += ((abs(index[0] - state.robot[0])) * (abs(index[0] - state.robot[0])) + (abs(index[1] - state.robot[1])) * (abs(index[1] - state.robot[1])))

        if snow[index] == 3 and index == state.destination or snow[index] == 6 and index == state.destination:
            distance += 0
            man += 0
        elif (snow[index] == 4 or snow[index] == 5) and index != state.destination:
            distance += ((abs(index[0] - state.destination[0])) * (abs(index[0] - state.destination[0])) + (abs(index[1] - state.destination[1])) * (abs(index[1] - state.destination[1])))
            distance += ((abs(index[0] - state.robot[0])) * (abs(index[0] - state.robot[0])) + (abs(index[1] - state.robot[1])) * (abs(index[1] - state.robot[1])))

            man += 2 * (abs(index[0] - state.destination[0]) + abs(index[1] - state.destination[1]))
            man += 2 * (abs(index[0] - state.robot[0]) + abs(index[1] - state.robot[1]))

            min_distance = min(man, distance)
        elif snow[index] == 6 and index!= state.destination or snow[index]==3 and index!=state.destination:
            man += 3 * (abs(index[0] - state.destination[0]) + abs(index[1] - state.destination[1]))
            man += 3 * (abs(index[0] - state.robot[0]) + abs(index[1] - state.robot[1]))
            distance += 3*((abs(index[0] - state.destination[0])) * (abs(index[0] - state.destination[0])) + (abs(index[1] - state.destination[1])) * (abs(index[1] - state.destination[1])))
            distance += 3 *((abs(index[0] - state.robot[0])) * (abs(index[0] - state.robot[0])) + (abs(index[1] - state.robot[1])) * (abs(index[1] - state.robot[1])))
            min_distance = min(man, distance)



      else:

        distance += math.sqrt((abs(index[0] - state.destination[0])) * (abs(index[0] - state.destination[0])) + (abs(index[1] - state.destination[1])) * (abs(index[1] - state.destination[1])))
        distance += math.sqrt((abs(index[0] - state.robot[0])) * (abs(index[0] - state.robot[0])) + (abs(index[1] - state.robot[1])) * (abs(index[1] - state.robot[1])))
        man += (abs(index[0] - state.destination[0]) + abs(index[1] - state.destination[1]))
        man += (abs(index[0] - state.robot[0]) + abs(index[1] - state.robot[1]))

        min_distance = min(man, distance)


    return min_distance




def fval_function(sN, weight):
#IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SnowballState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """

    #Many searches will explore nodes (or states) that are ordered by their f-value.
    #For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    #You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    #The function must return a numeric f-value.
    #The value will determine your state's position on the Frontier list during a 'custom' search.
    #You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.

    value = 0
    value += sN.gval + (weight * sN.hval)
    return value

def anytime_gbfs(initial_state, heur_fn, timebound = 10):
#IMPLEMENT
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a snowball state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''


    #using best first search we will use COST to constantly prune the result till we find the best one
    #if none can be found we will return false
    time = os.times()[0]
    totalTime = timebound + time

    #initialize SearchEngine
    #turn on cycle checking
    wrapped_fval_function = (lambda sN : fval_function(sN, weight))
    wrapped_heur_function = (lambda state :  heur_alternate(state))
    search = SearchEngine(strategy='best_first', cc_level='full')
    search.init_search(initial_state, snowman_goal_state, wrapped_heur_function, wrapped_fval_function)

    found = False
    goal_node = search.search(timebound)

    cost_bound_pruned = (float("inf"), float("inf"), float("inf"))
    short_goal_node = False

    while (time < totalTime):
        start = timer()

        if goal_node:
            found = True
            found = goal_node
            #this is when we prune it using costbound
            if(found.gval < cost_bound_pruned[0]):
                #we will now change the cost to the gval since it is shorter
                #for best first search the fval is the hval so
                cost_bound_pruned = (goal_node.gval, goal_node.gval, goal_node.gval*2) #this will be cost(gbound, hbound, g+hbound)
                short_goal_node = goal_node
                #but since we are doing greedy first search it will keep pruning it
            goal_node = search.search(timebound)
        if (goal_node == False):
            return found
        #timebound -= time
        stop = timer()
        time -= start-stop
        #time -= totalTime

    return short_goal_node



    #the cost will be calculated with the path's gval

def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound = 10):
#IMPLEMENT
    '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
    '''INPUT: a snowball state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''

    time = os.times()[0]
    totalTime = timebound + time

    #initialize SearchEngine
    #weight = (initial_state.height - 1) * (initial_state.width - 1)
    if(initial_state.height == initial_state.width):
        weight = (initial_state.height * initial_state.width)-initial_state.width
        #weight = initial_state.height*3
        #weight = 11
    elif(initial_state.height < initial_state.width):
        #weight = initial_state.width-1#20#(initial_state.height-1) * (initial_state.width-1)
        weight = initial_state.width-1
    elif(initial_state.height > initial_state.width):
        weight = initial_state.height-1#7 #(initial_state.height * initial_state.width)-1

    wrapped_fval_function = (lambda sN : fval_function(sN, weight))
    wrapped_heur_function = (lambda state :  heur_alternate(state))
    search = SearchEngine(strategy='custom', cc_level='full')
    search.init_search(initial_state, snowman_goal_state, heur_fn, wrapped_fval_function)

    found = False
    goal_node = search.search(timebound)
    cost = (float("inf"), float("inf"), float("inf"))


    short_goal_node = False

    while (time < totalTime):
        start = timer()

        if goal_node:
            found = True
            found = goal_node
            #this is when we prune it using costbound
            if(found.gval < cost[0]):
                #we will now change the cost to the gval since it is shorter
                cost = (goal_node.gval, goal_node.gval, goal_node.gval*2) #this will be cost(gbound, hbound, g+hbound)
                short_goal_node = goal_node
                #but since we are doing greedy first search it will keep priuning it
            goal_node = search.search(timebound)
        if (goal_node == False):
            short_goal_node = found


        stop = timer()
        time -= start-stop
        #time -= totalTime

    return short_goal_node


if __name__ == "__main__":
  #TEST CODE
  solved = 0; unsolved = []; counter = 0; percent = 0; timebound = 2; #2 second time limit for each problem
  print("*************************************")
  print("Running A-star")

  for i in range(0, 10): #note that there are 20 problems in the set that has been provided.  We just run through 10 here for illustration.

    print("*************************************")
    print("PROBLEM {}".format(i))

    s0 = PROBLEMS[i] #Problems will get harder as i gets bigger

    se = SearchEngine('astar', 'full')
    se.init_search(s0, goal_fn=snowman_goal_state, heur_fn=heur_simple)
    final = se.search(timebound)

    if final:
      final.print_path()
      solved += 1
    else:
      unsolved.append(i)
    counter += 1

  if counter > 0:
    percent = (solved/counter)*100

  print("*************************************")
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))
  print("*************************************")

  solved = 0; unsolved = []; counter = 0; percent = 0; timebound = 8; #8 second time limit
  print("Running Anytime Weighted A-star")

  for i in range(0, 10):
    print("*************************************")
    print("PROBLEM {}".format(i))

    s0 = PROBLEMS[i] #Problems get harder as i gets bigger
    weight = 10
    final = anytime_weighted_astar(s0, heur_fn=heur_simple, weight=weight, timebound=timebound)

    if final:
      final.print_path()
      solved += 1
    else:
      unsolved.append(i)
    counter += 1

  if counter > 0:
    percent = (solved/counter)*100

  print("*************************************")
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))
  print("*************************************")
