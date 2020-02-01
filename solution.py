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
        if box not in state.storage and (is_immovable(box, state) or is_stuck(box, state)):
            return True
    return False


def is_oob(pos, state: SokobanState):
    """returns whether a given position is out of bounds"""
    return not (0 <= pos[0] < state.width and 0 <= pos[1] < state.height)


def is_immovable(pos, state):
    """
            returns whether a given (x,y) position is immovable
            a position is immovable if it is an obsticle (or boundry), or a box cornered by two immovable positions
    """
    return _is_immov(pos, state, set())


def _is_immov(pos, state: SokobanState, visited: set):
    _pos = tuple(pos)
    # if the position is an obstacle or boundry it is immovable
    # if this pos is already in visited then there is a box at this position that is deadlocked with another box
    if _pos in state.obstacles or is_oob(_pos, state) or _pos in visited:
        return True
    elif _pos not in state.boxes:  # not an obsticle or box, then position is not dead
        return False

    visited.add(_pos)

    up = _is_immov(UP.move(_pos), state, visited)
    down = _is_immov(DOWN.move(_pos), state, visited)
    if not up and not down:
        return False

    left = _is_immov(LEFT.move(_pos), state, visited)
    if (up and left) or (down and left):
        return True

    right = _is_immov(RIGHT.move(_pos), state, visited)
    if (down and right) or (up and right):
        return True
    return False


def is_stuck(box, state: SokobanState):
    """
        returns whether a box is stuck against a wall with no storage position
        this is the case when a box is in a situation like this:

        #    $    #
         #########

        where $ is the box and # is *any* immovable position
        (can be oriented in any way and the bottom wall can be any length)

        **** ASSUMES BOX IS MOVABLE ****
    """

    # wall dir is direction of wall from box
    for wall_dir in (UP, DOWN, LEFT, RIGHT):

        found_plus_edge, found_minus_edge = False, False
        plus_pos, minus_pos = [box[0], box[1]], [box[0], box[1]]
        plus_dir, minus_dir = cw_dir(wall_dir), ccw_dir(wall_dir)

        while not found_plus_edge:

            if tuple(plus_pos) in state.storage or not is_immovable(wall_dir.move(plus_pos), state):
                return False
            else:
                if is_immovable(plus_pos, state):
                    found_plus_edge = True

            ladd2(plus_pos, plus_dir.delta)

        while not found_minus_edge:

            if tuple(minus_pos) in state.storage or not is_immovable(wall_dir.move(minus_pos), state):
                return False
            else:
                if is_immovable(minus_pos, state):
                    found_minus_edge = True

            ladd2(minus_pos, minus_dir.delta)

        return found_plus_edge and found_minus_edge


def cw_dir(dir: Direction):
    """returns the next direction in the sequence UP RIGHT DOWN LEFT UP ..."""
    if dir is UP:
        return RIGHT
    elif dir is RIGHT:
        return DOWN
    elif dir is DOWN:
        return LEFT
    elif dir is LEFT:
        return UP
    else:
        assert False


def ccw_dir(dir: Direction):
    """returns the next direction in the sequence UP LEFT DOWN RIGHT UP ..."""
    if dir is UP:
        return LEFT
    elif dir is LEFT:
        return DOWN
    elif dir is DOWN:
        return RIGHT
    elif dir is RIGHT:
        return UP
    else:
        assert False


def ladd2(iter1, iter2):
    iter1[0] += iter2[0]
    iter1[1] += iter2[1]


def tadd(tup1, tup2):
    """for some two tuples, returns a new tuple where each index i is tup1[i] + tup2[i]"""
    return tuple([i + j for i, j in zip(tup1, tup2)])


def tmul(i, tup):
    """for some tuple, returns a tuple where each index is multiplied by i"""
    return tuple([j * i for j in tup])


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

        if is_immovable(box, state) or is_stuck(box, state):
            return float('inf')

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


def heur_manhattan_with_pruning(state: SokobanState):
    remaining_stor = set(state.storage)
    remaining_box = set(state.boxes)
    total = 0

    # don't consider any boxes that are already in a storage position
    for box in state.boxes:
        if box in remaining_stor:
            remaining_box.remove(box)
            remaining_stor.remove(box)

    for box in remaining_box:

        # check for dead state and don't look at any successors
        if is_immovable(box, state) or is_stuck(box, state):
            return float('inf')

        # check distance between box and closest storage point that hasn't been "claimed" so far
        min_dst = float('inf')
        target = None
        for stor in remaining_stor:
            d = calc_manhattan(box, stor)
            if d < min_dst:
                min_dst = d
                target = stor

        total += min_dst
        remaining_stor.remove(target)

    # looking at tests with and without checking for distance between robots and boxes, it seems like it is worth it to
    # check iff there are any obsticles, or there are 2 or fewer robots and 3 or fewer boxes
    if len(state.obstacles) > 0 or len(state.robots) <= 2 and len(state.boxes) <= 3:
        for robot in state.robots:
            if len(remaining_box) is 0:
                break
            min_dst = float('inf')
            target = None
            for box in remaining_box:
                d = calc_manhattan(robot, box)
                if d < min_dst:
                    min_dst = d
                    target = box

            total += min_dst
            remaining_box.remove(target)

    return total


times_called = [0]


def heur_alternate(state: SokobanState):
    """a better heuristic
    INPUT: a sokoban state
    OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal."""
    global times_called
    times_called[0] += 1

    # return heur_manhattan_with_pruning(state)
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


test_state = SokobanState("START", 0, None, 5, 5,  # dimensions
                          ((2, 1), (2, 3)),  # robots
                          frozenset(((1, 1), (4, 0))),  # (1, 3), (3, 1), (3, 3))), #boxes
                          frozenset(((0, 0), (0, 4), (4, 0), (4, 4))),  # storage
                          frozenset(((1, 0), (2, 0), (3, 0), (1, 4), (2, 4), (3, 4)))  # obstacles
                          )

if __name__ == "__main__":
    s0 = test_state  # PROBLEMS[0]
    print(s0.state_string())
    se = SearchEngine('best_first', 'full')
    se.init_search(s0, goal_fn=sokoban_goal_state, heur_fn=heur_alternate)
    final = se.search()
