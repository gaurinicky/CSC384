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

        for index in state.snowballs:
          #excluding corner cases when the snowball is not at the destination
          if index != state.destination:
            #snowball is at the corner of the board
            top = (0, height)
            bottom = (0, 0)
            top_right = (width, height)
            bottom_right = (width, 0)
            if index == top or index == bottom or index == bottom_right or index == top_right:
                return float("inf")
            #if destination is on a wall and the snowball is on another wall
            if(state.destination[0] != 0) and (index[0] == 0):
                return float("inf")
            elif(state.destination[1] != 0) and (index[1] == 0):
                return float("inf")
            elif (state.destination[0] != width) and (index[0] == width):
                return float("inf")
            elif (state.destination[1] != height) and (index[1] == height):
                return float("inf")

            #snowball is in between two obstacles

            if(index[0]+1, index[1]) in state.obstacles and (index[0], index[1]+1) in state.obstacles:
                return float("inf")
            elif(index[0]-1, index[1]) in state.obstacles and (index[0], index[1]-1) in state.obstacles:
                return float("inf")
            elif(index[0]+1, index[1]) in state.obstacles and (index[0], index[1]-1) in state.obstacles:
                return float("inf")
            elif(index[0]-1, index[1]) in state.obstacles and (index[0], index[1]+1) in state.obstacles:
                return float("inf")


            #obstalce + snowball + corner
            if (index[0] == 0 and (0, index[1]+1) in state.obstacles) or (index[0] == 0 and (0, index[1]-1) in state.obstacles):
                return float("inf")
            elif (index[1] == 0 and (index[0] + 1, 0) in state.obstacles) or (index[1] == 0 and (index[0]-1, 0) in state.obstacles):
                return float("inf")
            elif ((index[0] == height) and ((height, index[1] + 1) in state.obstacles)) or ((index[0] == height) and ((height, index[1] - 1) in state.obstacles)):
              return float("inf")
            elif ((index[1] == width) and ((index[0] + 1, width) in state.obstacles)) or ((index[1] == width) and ((index[0] - 1, width) in state.obstacles)):
              return float("inf")



          # for snowballs that are on top one another we have to increase the distance
          snow = state.snowballs
          if snow[index] >= 3:

            distance += ((abs(index[0] - state.destination[0])) * (abs(index[0] - state.destination[0])) + (abs(index[1] - state.destination[1])) * (abs(index[1] - state.destination[1])))
            distance += ((abs(index[0] - state.robot[0])) * (abs(index[0] - state.robot[0])) + (abs(index[1] - state.robot[1])) * (abs(index[1] - state.robot[1])))

            #man += 2 * abs(index[0] - state.destination[0]) + abs(index[1] - state.destination[1])
            #man += 2 * abs(index[0] - state.robot[0]) + abs(index[1] - state.robot[1])


          else:
            #man += abs(index[0] - state.destination[0]) + abs(index[1] - state.destination[1])
            #man += abs(index[0] - state.robot[0]) + abs(index[1] - state.robot[1])

            distance += math.sqrt((abs(index[0] - state.destination[0])) * (abs(index[0] - state.destination[0])) + (abs(index[1] - state.destination[1])) * (abs(index[1] - state.destination[1])))
            distance += math.sqrt((abs(index[0] - state.robot[0])) * (abs(index[0] - state.robot[0])) + (abs(index[1] - state.robot[1])) * (abs(index[1] - state.robot[1])))


        return distance
