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

def calc_manhattan_ls(p1s, p2s):
    total = 0
    for p1 in p1s:
        total += calc_manhattan(p1,min(p2s, key=lambda x: calc_manhattan(p1, x)))
    return total

def obsticle_in_dir(pos, dir: Direction, state: SokobanState):
    return dir.move(pos) in state.obstacles

def box_in_dir(pos, dir: Direction, state: SokobanState):
    return dir.move(pos) in state.boxes

def opp_dir(dir: Direction):
    return Direction("opposite " + dir.name, (-dir.delta[0], -dir.delta[1]))


def movable(box, state: SokobanState):
    ob_left = obsticle_in_dir(box, LEFT, state)
    ob_right = obsticle_in_dir(box, RIGHT, state)
    ob_up = obsticle_in_dir(box, UP, state)
    ob_down = obsticle_in_dir(box, DOWN, state)

    return not (
            (ob_down and ob_right) or
            (ob_down and ob_left) or
            (ob_left and ob_up) or
            (ob_up and ob_right)
    )


def pushable_dirs(box, state: SokobanState):
    ob_left = obsticle_in_dir(box, LEFT, state) or box_in_dir(box, LEFT, state)
    ob_right = obsticle_in_dir(box, RIGHT, state) or box_in_dir(box, RIGHT, state)
    ob_up = obsticle_in_dir(box, UP, state) or box_in_dir(box, UP, state)
    ob_down = obsticle_in_dir(box, DOWN, state) or box_in_dir(box, DOWN, state)
    dirs = []

    if not ob_left and not ob_right:
        dirs += LEFT
        dirs += RIGHT
    if not ob_up and not ob_down:
        dirs += LEFT
        dirs += RIGHT
    return dirs

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
    ls = [[]]*len(state.boxes)

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

def heur_spaghetti(state: SokobanState):
    total = 0

    moved_robot = None
    # prioritise states that move the same robot as their parent
    if state.parent is not None and state.parent.parent is not None:
        # get which robot moved from previous state to this one
        for i in range(len(state.robots)):
            if state.robots[i] is not state.parent.robots[i]:
                moved_robot = i
                break

        # don't use current state if current moved robot is different from the robot previous state moved
        if state.parent.robots[moved_robot] is not state.parent.parent.robots[moved_robot]:
            total += 100

    # if box is not in storage, calculate distance from current robot to each box and add to estimation
    # if current robot is none calculate distance to nearest robot for each box
    for box in state.boxes:
        if not movable(box, state):
            total += 999
        elif box not in state.storage:
            target = state.robots[moved_robot] if moved_robot is not None \
                                               else min(state.robots, key=lambda x: calc_manhattan(box, x))
            total += calc_manhattan(box, target)

    return total + heur_manhattan_distance(state)

def heur_idk(state: SokobanState):
    return calc_manhattan_ls(state.storage, state.boxes) + calc_manhattan_ls(state.robots, state.boxes) - len(state.boxes)

def heur_alternate(state: SokobanState):
    """a better heuristic
    INPUT: a sokoban state
    OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal."""

    return heur_idk(state)


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
