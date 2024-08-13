from __future__ import annotations

import sys
from constants import *
from environment import *
from state import State
import heapq
from typing import List, Tuple, Optional, Any

"""
solution.py

This file is a template you should use to implement your solution.

You should implement 

COMP3702 2024 Assignment 1 Support Code
"""


def filter_widget_dict(widget_dict, center_dict):
    center_types = set(center_dict.values())

    # Create a copy of widget_dict for iteration, to avoid modifying a dictionary while iterating it.
    widget_dict_copy = widget_dict.copy()

    # Iterate through widget_dict_copy
    for widget_location, widget_type in widget_dict_copy.items():
        # If the type of widget is not in center_dict, remove it from widget_dict
        if widget_type not in center_types:
            del widget_dict[widget_location]


class Solver:

    def __init__(self, environment, loop_counter):
        self.environment = environment
        self.loop_counter = loop_counter
        #
        # TODO: Define any class instance variables you require here.
        # NOTE: avoid performing any computationally expensive heuristic preprocessing operations here - use the preprocess_heuristic method below for this purpose
        #

    def solve_ucs(self):
        """
        Find a path which solves the environment using Uniform Cost Search (UCS).
        :return: path (list of actions, where each action is an element of BEE_ACTIONS)
        """
        initial_state = self.environment.get_init_state()
        visited = {initial_state: 0}
        frontier = [StateNode(0, initial_state, None, None)]
        heapq.heapify(frontier)

        while frontier:
            self.loop_counter.inc()
            node = heapq.heappop(frontier)
            state = node.state
            if self.environment.is_solved(state):
                return node.get_path()

            successors = node.get_successors()
            for s in successors:
                if s.state not in visited.keys() or s.path_cost < visited[s.state]:
                    visited[s.state] = s.path_cost
                    heapq.heappush(frontier, s)

        return []

    # === A* Search =========================================================================
    def preprocess_heuristic(self):
        """
        Perform pre-processing (e.g. pre-computing repeatedly used values) necessary for your heuristic,
        """

        #
        #
        # TODO: (Optional) Implement code for any preprocessing required by your heuristic here (if your heuristic
        #  requires preprocessing).
        #
        # If you choose to implement code here, you should call this method from your solve_a_star method (e.g. once at
        # the beginning of your search).
        #
        #

        pass

    def compute_heuristic(self, state):
        total_distance = 0

        if state.is_on_edge():
            total_distance += 10

        if state.is_next_to_obstacle():
            total_distance += 10

        center_dict = state.get_target_centers()
        widget_dict = {}

        for i in range(len(state.widget_centres)):
            widget_dict[state.widget_centres[i]] = state.environment.widget_types[i]

        for widget_location, widget_type in widget_dict.items():
            min_distance = float('inf')
            optimal_center = None

            for center, center_type in center_dict.items():
                if center_type == widget_type:
                    distance = abs(center[0] - widget_location[0]) + abs(center[1] - widget_location[1])
                    if distance < min_distance:
                        min_distance = distance
                        optimal_center = center

            if optimal_center and min_distance == 0:
                del center_dict[optimal_center]

            total_distance += min_distance


        return total_distance/5



    def solve_a_star(self):
        initial_state = self.environment.get_init_state()
        visited = {initial_state: 0}
        frontier = [StateNode(0, initial_state, None, None)]

        while frontier:
            self.loop_counter.inc()
            node = heapq.heappop(frontier)
            state = node.state
            if self.environment.is_solved(state):
                return node.get_path()

            successors = node.get_successors()
            for s in successors:
                heuristic_cost = self.compute_heuristic(s.state)
                full_cost = s.path_cost + heuristic_cost
                if s.state not in visited.keys() or full_cost < visited[s.state]:
                    visited[s.state] = full_cost
                    heapq.heappush(frontier, StateNode(full_cost, s.state, node, s.action_from_parent))

        return []




class StateNode:
    def __init__(self, path_cost: float, state: Any, parent: Optional[StateNode],
                 action_from_parent: Optional[str] = None):
        self.path_cost = path_cost
        self.state = state
        self.parent = parent
        self.action_from_parent = action_from_parent

    def __lt__(self, other: StateNode) -> bool:
        return self.path_cost < other.path_cost

    def get_path(self):
        path = []
        cur = self
        while cur.action_from_parent is not None:
            path.append(cur.action_from_parent)
            cur = cur.parent
        path.reverse()
        return path

    def get_successors(self):
        successors = []
        for a in BEE_ACTIONS:
            success, cost, next_state = self.state.environment.perform_action(self.state, a)
            if success:
                successors.append(StateNode(self.path_cost + cost, next_state, self, a))
        return successors


