#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os  # for time functions
from search import *  # for search engines
from sokoban import SokobanState, Direction, PROBLEMS  # for Sokoban specific classes and problems
from sokoban import UP, DOWN, LEFT, RIGHT
import heapq


# ================================================= Utilities ======================================================
def sokoban_goal_state(state):
    """
  @return: Whether all boxes are stored.
  """
    for box in state.boxes:
        if box not in state.storage:
            return False
    return True


def calc_manhattan(p1, p2):
    """calculates manhattan distance between two points (x1, y1) and (x2, y2)"""
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

def calc_manhattan_tup(p1, p2):
    """returns tuple representing signed x and y distance from p1 to p2"""
    return p2[0] - p1[0], p2[1] - p1[1]


def is_dead_state(state: SokobanState):
    """returns whether the given state is dead, i.e a solution is never possible from the given state"""
    # a state is dead if any box is in an unmovable position and not on a storage position
    for box in state.boxes:
        if box not in state.storage and dead(box, state, set()):
            return True
    return False

def is_oob(pos, state: SokobanState):
    """returns whether a given position is out of bounds"""
    return not (0 <= pos[0] < state.width and 0 <= pos[1] < state.height)

def dead(pos, state: SokobanState, visited: set):
    """
        returns whether a given (x,y) position is dead (unmovable)
        a position is dead if it is an obsticle (or boundry), or a box cornered by two dead positions
    """
    # if the position is an obstacle or boundry it is dead
    # if this pos is already in visited then there is a box at this position that is deadlocked with another box
    if pos in state.obstacles or is_oob(pos, state) or pos in visited:
        return True
    elif pos not in state.boxes:  # not an obsticle or box, then position is not dead
        return False

    corners = [(DOWN, LEFT), (LEFT, UP), (UP, RIGHT), (RIGHT, DOWN)]
    visited.add(pos)

    for d1, d2 in corners:
        if dead(d1.move(pos), state, visited) and dead(d2.move(pos), state, visited):
            return True
    return False


# ================================================= Heuristics =====================================================
def heur_manhattan_distance(state: SokobanState):
    """admissible sokoban puzzle heuristic: manhattan distance
    INPUT: a sokoban state
    OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal."""

    total = 0
    for box in state.boxes:
        target = min(state.storage, key=lambda x: calc_manhattan(box, x))
        total += calc_manhattan(box, target)

    return total


def heur_smart_manhattan(state: SokobanState):
    ls = [[]] * len(state.boxes)

    for s, stor in enumerate(state.storage):
        for b, box in enumerate(state.boxes):
            heapq.heappush(ls[b], (s, calc_manhattan(box, stor)))

    total = 0
    filled = set()
    for heap in ls:

        while True:
            closest = heapq.heappop(heap)
            if closest[0] not in filled:
                filled.add(closest[0])
                total += closest[1]
                break

    return total


# each robot looks at nearest box and calculate
def heur_smart_robots(state: SokobanState):
    total = 0

    for box in state.boxes:
        # if box is in storage already, assume it adds no cost and go to next box
        if box in state.storage:
            continue

        # calculate distance and direction box needs to move, and find closest robot to move box
        box_signed_dst = calc_manhattan_tup(box, min(state.storage, key=lambda x: calc_manhattan(box, x)))
        closest_robot = min(state.robots, key=lambda x: calc_manhattan(box, x))

        # find positions robot would need to move to to move box in correct directions
        target_pos = [None, None]
        if box_signed_dst[0] < 0:  # if box wants to go left, robot needs to move to right of box
            target_pos[0] = RIGHT.move(box)
        elif box_signed_dst[0] > 0:
            target_pos[0] = LEFT.move(box)
        if box_signed_dst[1] < 0:  # if box wants to go up, robot needs to move to below box
            target_pos[1] = DOWN.move(box)
        elif box_signed_dst[1] > 0:
            target_pos[1] = UP.move(box)

        # box needs to move to storage position, so add movement cost to total
        total += abs(box_signed_dst[0]) + abs(box_signed_dst[1])

        if target_pos[0] is not None and target_pos[1] is not None:
            total += min(calc_manhattan(closest_robot, target_pos[0]), calc_manhattan(closest_robot, target_pos[1])) + 2

        # box is in same y pos as storage, so just x needs to change (target has same y pos as box)
        elif target_pos[0] is not None:

            total += calc_manhattan(closest_robot, target_pos[0])

            # if robot is on same y pos as target (and box), and the box is between the robot and the target,
            # need to move around the box so add 2
            if closest_robot[1] == target_pos[0][1] and \
                    (target_pos[0][0] < box[0] < closest_robot[0] or
                     closest_robot[0] < box[0] < target_pos[0][0]):
                total += 2

        # box is in same x pos as storage, so just y needs to change (target has same x pos as box
        elif target_pos[1] is not None:

            total += calc_manhattan(closest_robot, target_pos[1])

            # if robot is on same x pos as target (and box), and the box is between the robot and the target,
            # need to move around the box so add 2
            if closest_robot[1] == target_pos[1][0] and \
                    (target_pos[1][1] < box[1] < closest_robot[1] or
                     closest_robot[1] < box[1] < target_pos[1][1]):
                total += 2
        else:
            # at least one of target_pos [0] and [1] is not None otherwise box in storage and would have been skipped
            assert False

    return total


times_called = [0]
def heur_alternate(state: SokobanState):
    """a better heuristic
    INPUT: a sokoban state
    OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal."""
    global times_called
    times_called[0] += 1
    # if times_called[0] > 100000:
    #     state.print_state()

    if is_dead_state(state):
        return 99999999999

    return heur_smart_robots(state)


def trivial_heuristic(state):
    """trivial admissible sokoban heuristic
    INPUT: a sokoban state
    OUTPUT: a numeric value that serves as an estimate of the distance of the state
            (# of moves required to get) to the goal. """
    count = 0
    for box in state.boxes:
        if box not in state.storage:
            count += 1
    return count


def heur_zero(state):
    """Zero Heuristic can be used to make A* search perform uniform cost search"""
    return 0


# ================================================ Anytime Algorithms ================================================

def fval_function(sN, weight):
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """

    # IMPLEMENT

    # Many searches will explore nodes (or states) that are ordered by their f-value. For UCS, the fvalue is the same
    # as the gval of the state. For best-first search, the fvalue is the hval of the state. You can use this function
    # to create an alternate f-value for states; this must be a function of the state and the weight. The function
    # must return a numeric f-value. The value will determine your state's position on the Frontier list during a
    # 'custom' search. You must initialize your search engine object as a 'custom' search engine if you supply a
    # custom fval function.
    return 0


def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound=10):
    """Provides an implementation of anytime weighted a-star, as described in the HW1 handout
    INPUT: a sokoban state that represents the start state and a timebound (number of seconds)
    OUTPUT: A goal state (if a goal is found), else False
    implementation of weighted astar algorithm"""

    # IMPLEMENT
    return False


def anytime_gbfs(initial_state, heur_fn, timebound=10):
    """Provides an implementation of anytime greedy best-first search, as described in the HW1 handout
    INPUT: a sokoban state that represents the start state and a timebound (number of seconds)
    OUTPUT: A goal state (if a goal is found), else False
    implementation of weighted astar algorithm"""

    # IMPLEMENT
    return False


if __name__ == "__main__":
    s0 = PROBLEMS[5]
    print(s0.state_string())

    se = SearchEngine('best_first', 'full')
    se.trace_on()

    se.init_search(s0, goal_fn=sokoban_goal_state, heur_fn=heur_alternate)

    final = se.search()
