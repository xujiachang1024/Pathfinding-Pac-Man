# partialAgent.py
# parsons/15-oct-2017
#
# Version 1
#
# The starting point for CW1.
#
# Intended to work with the PacMan AI projects from:
#
# http://ai.berkeley.edu/
#
# These use a simple API that allow us to control Pacman's interaction with
# the environment adding a layer on top of the AI Berkeley code.
#
# As required by the licensing agreement for the PacMan AI we have:
#
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
#
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).

# The agent here is was written by Simon Parsons, based on the code in
# pacmanAgents.py

from pacman import Directions
from game import Agent
import api
import random
import game
import util
import sys

"""
SearchAgent class: A type of agent that inherits from the base class of Agent,
and that implements the search/pathfinding algorithms (i.e. BFS, A*). This class
doesn't define the getActino(), so it cannot play in the maze.

@author: Jiachang (Ernest) Xu
"""
class SearchAgent(Agent):

    """
    Backtrace the path, used in A* and BFS

    @param self: the class itself
    @param location: the target location to initiate the backtrace
    @param backtrace: the dictionary that guide the backtracing among locations
    @return: a size-2 tuple that contains [0] the path from the agent location
            to the target location, and [1] the length of the path
    """
    def construct_path(self, location, backtrace):
		path_stack = util.Stack()
		path_length = 0
		while backtrace[location] != (None, None):
			path_stack.push(location)
			path_length += 1
			location = backtrace[location]
		return (path_stack, path_length)

    """
    Declaration: I've learnt the Weighted A* search algorithm
    during my undergraduate study at University of Southern California,
    in pursuit of the B.S. Computer Science/Business Administration degree,
    from the class CSCI 360: Introduction to Artificial Intelligence.

    Weighted A* search algorithm to find a suboptimal path from a start location
    (i.e. the agent) to a goal location (i.e. a ghost, a corner)

    @param self: the class itself
    @param start: the start location (i.e. the agent usually)
    @param goal: the goal location (i.e. a ghost, a corner)
    @param walls: a list of locations of walls within the maze
    @param floors: a list of locations of navigatable floors within the maze
    @param w: the weight assigned to the heuristic function of the A* algorithm;
            the default value is 1
    @param survival_mode: whether the agent is calling this function to
            calculate the path to a ghost (need to handle the decimals); the
            default value is False
    @return: a size-2 tuple that contains [0] the path from the start location
            to the goal location, and [1] the length of the path, if there
            exists a path; otherwise, None
    """
    # A* requires a SINGLE target
    def ASS_for_closest_target(self, start, goal, walls, floors, w = 1, survival_mode = False):
		# A* data structures
        discovered = set()
        evaluated = set()
        backtrace = dict()
        g_scores = dict()
        f_scores = dict()
        displacements = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        ghost_shadows = [(0.0, 0.0), (0.5, 0.0), (-0.5, 0.0), (0.0, 0.5), (0.0, -0.5)]
		# A* initialization
        discovered.add(start)
        backtrace[start] = (None, None)
        for floor in floors: # floor = location (x, y)
			if floor not in walls:
				g_scores[floor] = sys.maxint
				f_scores[floor] = sys.maxint
        g_scores[start] = 0
        f_scores[start] = g_scores[start] + w * util.manhattanDistance(start, goal)
		# A* in work
        while len(discovered) > 0:
			# grab the location from the discovered set with the lowest f score
            curr_location = None
            for location in discovered:
                if curr_location == None:
                    curr_location = location
                if f_scores[location] < f_scores[curr_location]:
					curr_location = location
			# goal reached: backtrace the path and return
            if survival_mode:
                for ghost_shadow in ghost_shadows:
                    if (curr_location[0] + ghost_shadow[0], curr_location[1] + ghost_shadow[1]) == goal:
                        return self.construct_path(curr_location, backtrace)
            else:
                if curr_location == goal:
                    return self.construct_path(curr_location, backtrace)
            # curr_location evaluated
            discovered.remove(curr_location)
            evaluated.add(curr_location)
            # iterate through the neighbors of curr_location
            for displacement in displacements:
				next_location = (curr_location[0] + displacement[0], curr_location[1] + displacement[1])
				# next_location is a wall: ignore
				if next_location in walls:
					continue
				# next_location already evaluated: ignore
				if next_location in evaluated:
					continue
				# distance from start to next_location
				g_score = g_scores[curr_location] + 1
				# next_location discovered: add to the discovered set
				if next_location not in discovered:
					discovered.add(next_location)
				# not a better path: skip this
				elif g_score >= g_scores[next_location]:
					continue
                # bookkeeping for next_location
				backtrace[next_location] = curr_location
				g_scores[next_location] = g_score
				f_scores[next_location] = g_scores[next_location] + w * util.manhattanDistance(next_location, goal)

    """
    Declaration: I've learnt the Breath-First Search (BFS) algorithm
    during my undergraduate study at University of Southern California,
    in pursuit of the B.S. Computer Science/Business Administration degree,
    from the class CSCI 104: Data Structures anf repudiating d Object Oriented Design.

    Breath-First Search (BFS) algrithm to find the shortest path from the agent
    location to the nearest target (i.e. food)

    @param self: the class itself
    @param agent_location: the location of the agent
    @param targets: a list of locations of targets
    @param walls: a list of locations of walls within the maze
    @param ghostbuster_mode: whether the agent is hunting for ghosts; the
            default value is False (this is a functionality for my own side
            project: GhostbusterAgent)
    @return: a size-2 tuple that contains [0] the shortest path from the agent
            location to the nearest target, and [1] the length of the path, if
            there exists a path; otherwise, None
    """
    def BFS_for_closest_target(self, agent_location, targets, walls, ghostbuster_mode=False):
		# BFS data structures
		queue = util.Queue()
		visited = set()
		backtrace = dict()
		displacements = [(1, 0), (-1, 0), (0, 1), (0, -1)]
		ghost_shadows = [(0.0, 0.0), (0.5, 0.0), (-0.5, 0.0), (0.0, 0.5), (0.0, -0.5)]
		# BFS initialization
		queue.push(agent_location)
		visited.add(agent_location)
		backtrace[agent_location] = (None, None)
		# BFS in work
		while not queue.isEmpty():
            # pop the first location from the queue
			curr_location = queue.pop()
            # check if curr_location is one of the targets
			if ghostbuster_mode:
			    for ghost_shadow in ghost_shadows:
			        if (curr_location[0] + ghost_shadow[0], curr_location[1] + ghost_shadow[1]) in targets:
			            return self.construct_path(curr_location, backtrace)
			else:
			    if curr_location in targets:
				    return self.construct_path(curr_location, backtrace)
            # find the neighbors of curr_location
			for displacement in displacements:
				next_location = (curr_location[0] + displacement[0], curr_location[1] + displacement[1])
                # next_location is a wall: ignore
				if next_location in walls:
					continue
                # next_location is already visited: ignore
				if next_location in visited:
					continue
                # bookkeeping for next_location
				queue.push(next_location)
				visited.add(next_location)
				backtrace[next_location] = curr_location

    """
    Declaration: I've learnt the Breath-First Search (BFS) algorithm
    during my undergraduate study at University of Southern California,
    in pursuit of the B.S. Computer Science/Business Administration degree,
    from the class CSCI 104: Data Structures and Object Oriented Design.

    Breath-First Search algorithm to find directions that lead to death traps

    @param self: the class itself've learnt the Breath-
    @param agent_location: the location of the agent
    @param walls: a list of locations of walls
    @param threshold: the threshold to classified whether a direction leads to
            a death trap
    @return: a set of directions which lead to possible death traps
    """
    def BFS_for_death_traps(self, agent_location, walls, threshold=5):
        # BFS data structures
        queue = util.Queue()
        visited = dict()
        death_trap_directions = set()
        displacements = {(1, 0): Directions.EAST, (-1, 0): Directions.WEST, (0, 1): Directions.NORTH, (0, -1): Directions.SOUTH}
        # BFS initialization: start with the immediate neighbors of the agent location
        for displacement, direction in displacements.items():
            immediate_next_step = (agent_location[0] + displacement[0], agent_location[1] + displacement[1])
            if immediate_next_step in walls:
                continue
            queue.push((immediate_next_step, direction, 1))
            visited[immediate_next_step] = 1
            death_trap_directions.add(direction)
        # BFS in work
        while not queue.isEmpty():
            # pop the first location from the queue
            curr_location, direction, distance = queue.pop()
            # if this direction exceeds the threshold: remove this it from the return set
            if distance >= threshold:
                if direction in death_trap_directions:
                    death_trap_directions.remove(direction)
                    continue
            # find the neighbors of curr_location
            for displacement in displacements.keys():
                next_location = (curr_location[0] + displacement[0], curr_location[1] + displacement[1])
                # next_location is a wall: ignore
                if next_location in walls:
                    continue
                # next_location is already visited: ignore
                if next_location in visited and distance > visited[next_location]:
                    continue
                # bookkeeping for next_location
                queue.push((next_location, direction, distance + 1))
                visited[next_location] = distance + 1
        return death_trap_directions



class PartialAgent(SearchAgent):

    """
    Contrustor: initialize internal memories as empty containers or None

    @param self: the class itself
    @return None
    """
    def __init__(self):
        self.states = []
        self.foods = set()
        self.walls = None
        self.corners = None
        self.floors = None
        self.counter = 0
        self.path_to_food = util.Stack()

    """
    This method will be called between multiple games to reset internal memories

    @param self: the class itself
    @param state: the current game state
    @return None
    """
    def final(self, state):
        self.states = []
        self.foods = set()
        self.walls = None
        self.corners = None
        self.floors = None
        self.counter = 0
        self.path_to_food = util.Stack()

    """
    Decide which direction to go based on the current game state

    @param self: the class itself
    @param state: the current game state
    @return the direction the agent decides to go
    """
    def getAction(self, state):
        """map-building operations"""
        # log game state history
        self.states.append(state)
        # first step of each round: populate internal memories
        if self.walls == None:
			self.walls = api.walls(state)
        if self.corners == None:
            # locations or corners on the exterior walls
			self.corners = api.corners(state)
            # calculate the real corners that the agent can move to
			for i in range(len(self.corners)):
				x = self.corners[i][0]
				y = self.corners[i][1]
				if x == 0:
					x += 1
				else:
					x -= 1
				if y == 0:
					y += 1
				else:
					y -= 1
				self.corners[i] = (x, y)
        if self.floors == None:
			self.floors = []
			x_coordinates = []
			y_coordinates = []
			for corner in self.corners:
			    x_coordinates.append(corner[0])
			    y_coordinates.append(corner[1])
			x_minimum, x_maximum = min(x_coordinates), max(x_coordinates)
			y_minimum, y_maximum = min(y_coordinates), max(y_coordinates)
			for x in range(x_minimum, x_maximum + 1):
				for y in range(y_minimum, y_maximum + 1):
					self.floors.append((x, y))
        agent_location = api.whereAmI(state)
        # print("agent_location:" + str(agent_location))
        # this location won't have food any more in the future
        if agent_location in self.foods:
            self.foods.remove(agent_location)
        # if at a corner: set the target corner to another one
        if agent_location in self.corners:
			self.counter = self.corners.index(agent_location)
			self.counter += random.choice([1, 2, 3])
        # discover the legal actions
        legal = api.legalActions(state)
        # remove STOP to increase mobility
        legal.remove(Directions.STOP)
        # discover any death trap directions
        death_trap_directions = self.BFS_for_death_traps(agent_location, self.walls, threshold=5)
        for death_trap_direction in death_trap_directions:
            if death_trap_direction in legal:
                legal.remove(death_trap_direction)
        if len(legal) == 0:
            legal.append(Directions.STOP)
        # add all the nearby foods into internal memories
        foods = api.food(state)
        for food in foods:
            self.foods.add(food)
        capsules = api.capsules(state)
        # add all the nearby capsules into internal memories
        for capsule in capsules:
            self.foods.add(capsule)
        """survival mode"""
        survival_mode = False
        ghosts = api.ghosts(state)
        if len(ghosts) > 0:
            for ghost in ghosts:
                # use A* to calculate the actual path from the agent to the ghost
                path_to_ghost, path_length = self.ASS_for_closest_target(agent_location, ghost, self.walls, self.floors, survival_mode=True)
                # if path_to_ghost is too long, ignore this ghost
                if path_length > 4:
                    continue
                survival_mode = True
                # calculate the neighboring location that poses the imminent threat
                proximity = None
                if path_to_ghost == None or path_to_ghost.isEmpty():
                    proximity = (ghost[0] - agent_location[0], ghost[1] - agent_location[1])
                else:
                    location_to_avoid = path_to_ghost.pop()
                    proximity = (location_to_avoid[0] - agent_location[0], location_to_avoid[1] - agent_location[1])
                # if the imminent threat comes from EAST
                if proximity[0] > 0:
                    if Directions.EAST in legal:
                        legal.remove(Directions.EAST)
                # if the imminent threat comes from WEST
                if proximity[0] < 0:
                    if Directions.WEST in legal:
                        legal.remove(Directions.WEST)
                # if the imminent threat comes from NORTH
                if proximity[1] > 0:
                    if Directions.NORTH in legal:
                        legal.remove(Directions.NORTH)
                # if the imminent threat comes from SOUTH
                if proximity[1] < 0:
                    if Directions.SOUTH in legal:
                        legal.remove(Directions.SOUTH)
            if len(legal) == 0:
				legal.append(Directions.STOP)
            # if there exist imminent threats
            if survival_mode:
                # need to recalculate the path to fod later
                self.path_to_food = util.Stack()
                # return a random legal action that is still available
                return random.choice(legal)
        """hungry mode"""
        if len(self.foods) > 0:
            # if the path_to_food internal memory is empty: recalculate
            if self.path_to_food == None or self.path_to_food.isEmpty():
                self.path_to_food, path_length = self.BFS_for_closest_target(agent_location, self.foods, self.walls)
            if self.path_to_food != None and not self.path_to_food.isEmpty():
                # find the next location to enter, and calculate displacement
                location_to_enter = self.path_to_food.pop()
                displacement = (location_to_enter[0] - agent_location[0], location_to_enter[1] - agent_location[1])
                # use displacement to decide which direction to go
                if displacement == (-1, 0):
                    return Directions.WEST
                if displacement == (1, 0):
                    return Directions.EAST
                if displacement == (0, -1):
                    return Directions.SOUTH
                if displacement == (0, 1):
                    return Directions.NORTH
        """corner-seeking mode"""
        # locate the corner that the agent needs to seek
        corner = self.corners[self.counter % len(self.corners)]
        # calculate the path to the target corner
        path_to_corner, path_length = self.ASS_for_closest_target(agent_location, corner, self.walls, self.floors, 1)
        if path_to_corner != None and not path_to_corner.isEmpty():
            # find the next location to enter, and calculate displacement
            location_to_enter = path_to_corner.pop()
            displacement = (location_to_enter[0] - agent_location[0], location_to_enter[1] - agent_location[1])
            # use displacement to decide which direction to go
            if displacement == (-1, 0):
                return Directions.WEST
            if displacement == (1, 0):
                return Directions.EAST
            if displacement == (0, -1):
                return Directions.SOUTH
            if displacement == (0, 1):
                return Directions.NORTH
        return random.choice(legal)
