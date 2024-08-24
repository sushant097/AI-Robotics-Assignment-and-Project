#!/usr/bin/env python

from PIL import Image, ImageDraw
import math
import time
try:
    import queue
except ImportError:
    import Queue as queue
import sys
import os.path
import pickle
from discretemap import *

###################################
# BEGIN Global Variable Definitions
dmap = None  # This is our discrete map
# END Global Variable Definitions
###################################

#This class will be used in our search
class SearchNode:
    def __init__(self,state,parent,cost):
        self.parent = parent # Pointer to parent node (will be None for root node)
        self.state = state # The state (grid cell) this node points to
        self.cost = cost # The cumulative cost of this search node (sum of all actions to get here)
        self.h = 0 # This node's heuristic value (must be set separately)
    def __lt__(self, other):
        return self.h < other.h
        
# This function returns the euclidean distance between two grid cells 
def euclidean_distance_grid(a,b):
    d = math.hypot(b[0]-a[0],b[1]-a[1])
    return d

# This function will set up everything we need for the search and run it
def run_a_star(start,goal, search_type):
    #start and goal are in grid coordinates
    # search_type: str ; which algorithm use to search

    #Create the start node
    start_node = SearchNode(start, None, 0)
    #Compute it's heuristic value
    start_node.h = euclidean_distance_grid(start,goal)

    #Create Fringe
    fringe = queue.PriorityQueue()

    #Insert start node into fringe
    priority = start_node.cost + start_node.h # Compute it's f-value
    fringe.put((priority, start_node)) # Add it to the fringe

    print('Starting A* search from grid cell ', start_node.state, ' to goal cell ', goal)
    print('Starting f-value is ', start_node.h + start_node.cost)

    #Run the A* search
    goal_node = a_star(fringe,goal, search_type)

    #Extract path from the goal_node and return it    
    path = []

    if goal_node is not None:
        # print('Found a solution!')
        cur_node = goal_node
        while cur_node is not None:
            path.append(cur_node.state)
            cur_node = cur_node.parent
        path.reverse()

    return path
    
# This is the main A* function.  It performs the search
def a_star(fringe,goal,search_type="ASTAR"):
    closed = [] # This keeps track of which grid cells we have already visited
    expansions = 0 # This keeps track of the number of expansions
    
    # Stay in loop as long as we have unexpanded nodes
    while not fringe.empty():
    
        # Get the node out of the fringe.  Format of items in the fringe is (priority, searchNode)
        current_node = fringe.get()[1]  # This gets the node to expand
 
        # Make sure that we didn't already expand a node pointing to this state
        if current_node.state not in closed:
            expansions = expansions + 1 # Increment our expansions counter
 
            #Test for the goal
            if current_node.state == goal:
                # We found it!  Return the goal node.
                #  It stores the entire path through the parent pointers
                # print('Found the goal after ', expansions, ' node expansions')
                if search_type=="ASTAR":
                    print(f"A* algorithm: {expansions} node expansions")
                elif search_type == "GBFS":
                    print(f"Greedy best-first search algorithm: {expansions} node expansions")
                elif search_type == "UCS":
                    print(f"Uniform cost search algorithm: {expansions} node expansions")

                return current_node
        
            # Add expanded node's id to the closed list
            closed.append(current_node.state)

            # Add all this nodes neighbors that aren't already in the closed list
            neighbors = get_neighbors(current_node.state, closed)

            #For each neighbor (a grid cell tuple(gx,gy) )
            for neighbor_cell in neighbors:
                #########################
                # Part B Code START
                #########################
                #Remove next line once you have code in the loop.  This is just so that python will run file as is
                print('Exploring neighbor', neighbor_cell)
                
                # 1. Compute the cost to get to neighbor_cell from the current cell (use euclidean_distance_grid())
                # 2. Compute the cumulative cost to get to neighbor_cell (add the cost for current_cell)
                # 3. Create a new SearchNode object for it (constructor => SearchNode(neighbor cell, current node, cumulative cost)
                # 4. Compute the new node's heuristic value (use euclidean_distance_grid())
                # 5. Compute the new node's priority value
                # 6. Add it to the fringe, with proper priority (use fringe.put((priority, new SearchNode))

                # 1. Compute the cost to get to neighbor_cell from the current cell (use euclidean_distance_grid())
                cost_to_neighbor = euclidean_distance_grid(current_node.state, neighbor_cell)

                # 2. Compute the cumulative cost to get to neighbor_cell (add the cost for current_cell)
                cumulative_cost = current_node.cost + cost_to_neighbor

                # 3. Create a new SearchNode object for it (constructor => SearchNode(neighbor cell, current node, cumulative cost)
                new_node = SearchNode(neighbor_cell, current_node, cumulative_cost)

                # 4. Compute the new node's heuristic value (use euclidean_distance_grid())
                new_node.h = euclidean_distance_grid(neighbor_cell, goal)

                # 5. Compute the new node's priority value
                # priority = new_node.cost + new_node.h
                if search_type=="ASTAR":
                    # Default to A* search (sum of cumulative cost and heuristic)
                    priority = new_node.cost + new_node.h
                elif search_type == "GBFS":
                    # For Greedy Best-First Search, set the priority to be only the heuristic value
                    priority = new_node.h
                elif search_type == "UCS":
                    # For Uniform Cost Search, set the priority to be the cumulative cost
                    priority = new_node.cost

                # 6. Add it to the fringe, with proper priority (use fringe.put((priority, new SearchNode))
                fringe.put((priority, new_node))

                #########################
                # Part B Code END
                #########################

    # We have exhausted all possible paths.  No solution found
    print('Didn\'t find a path after ', expansions, ' expansions')

# This function should return a list of the grid cells adjacent to
# g.  g = (x,y), so g[0] gives g's x-coordinate on grid, and g[1] gives g's y-coordinate on grid
# closed is a list of grid cells that have already been expanded
# dmap.occupied is a list of grid cells that have obstacles in them
# dmap.grid_width gives the width of the grid
# dmap.grid_height gives the height of the grid
def get_neighbors(g, closed):
    global dmap # Our discrete map
    neighbors = [] # The list of successors/neighbors

    #########################
    # Part A Code START
    #########################
    
    # Determine all of the neighbors/successors of grid cell g = (x,y)
    # Each should be a tuple of x and y grid coordinates 
    # Append each valid neighbor to g
    # To be valid, neighbor must:
    #  1. Not already be in closed (list of already explored states)
    #  2. Be on the map (check dmap.grid_width and dmap.grid_height)
    #  3. Not be occupied by an obstacle (dmap.occupied is the list of occupied cells)
    # Get the x and y coordinates of the current grid cell
    x, y = g

    # Define possible neighbor offsets (up, down, left, right, diagonal)
    offsets = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]

    for dx, dy in offsets:
        # Calculate the potential neighbor's coordinates
        neighbor_x = x + dx
        neighbor_y = y + dy

        # Check if the potential neighbor is within the map boundaries
        if (0 <= neighbor_x < dmap.grid_width) and (0 <= neighbor_y < dmap.grid_height):
            neighbor_cell = (neighbor_x, neighbor_y)

            # Check if the potential neighbor is not in the closed list
            if neighbor_cell not in closed:
                # Check if the potential neighbor is not occupied by an obstacle
                if neighbor_cell not in dmap.occupied:
                    # Append the valid neighbor to the list
                    neighbors.append(neighbor_cell)

    #########################
    # Part A Code END
    #########################

    # Return the list of neighbors
    return neighbors  

if __name__ == '__main__':
    # Create the Discrete Map.  This will load start and goal locations from the world file
    dmap = DiscreteMap(sys.argv[1], 5)
    
    # get second argument for search_type
    search_type = sys.argv[2]
    # Expand the obstacles by half the obstacle's radius (radius = 0.35)
    dmap.expand_obstacles(0.175)

    #Use A* to get the path from start to goal
    start = time.time()
    path = run_a_star(dmap.map_to_grid(dmap.start), dmap.map_to_grid(dmap.goal), search_type)
    end = time.time()
    duration = end - start

    #Uncomment to print out the path (in grid cell coordinates)
    #print('PATH = ', path)

    #Uncomment to print out the path (in map coordinates)
    #print('MAP PATH = [',)
    
    solution_quality = 0.0
    map_path = []
    for p in path:
        map_path.append(dmap.grid_to_map(p))
        if len(map_path) > 1:
            solution_quality += math.hypot((map_path[-1][0] - map_path[-2][0]),(map_path[-1][1] - map_path[-2][1]))
    #Uncomment to print out path in map coordinates
    #    print(map_path[-1], ',')
    #Uncomment to print out path in map coordinates
    #print(']')

    #Print out statistics about this search 
    print('Quality/Cost of solution: ', solution_quality)
    print('Search took: ', duration, 'seconds')

    #Save an image of the path
    display_image_file = sys.argv[1][0:-6] + f"_astarpath_{search_type}.png"
    dmap.display_path(path, display_image_file)
    print('Saved image of path to file: ', display_image_file)