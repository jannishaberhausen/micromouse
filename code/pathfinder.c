#include <stdio.h>
#include <stdlib.h>
#include "pathfinder.h"
#include "pathfinder_tests.h" //REMOVE AFTER TESTING

/** internal representation of the grid as a matrix of cells */
cell maze[SIZE][SIZE];

typedef struct coord {unsigned int x; unsigned int y;} coord;

///////////////////////////////////////////////////////////
//                       DEBUG                           //
///////////////////////////////////////////////////////////


cell** DEBUG_get_maze() {
    return (cell **) maze;
}


/**
 * Initialize the pathfinder
 *
 * @param x Starting position in x-direction
 * @param y Starting position in y-direction
 * @param dir Orientation at the start wrt. the maze
 */
void initMaze(unsigned int x, unsigned int y, orientation dir) {
    // set all cells to `uninitialized`
    for (int i = 0; i < SIZE; i++) {
        for (int j = 0; j < SIZE; j++) {
            // all sides unexplored
            for (int k = 0; k < 4; k++) {
                maze[i][j].walls[k] = UNKNOWN;
            }
            // cell not yet explored
            maze[i][j].flag = 0;
        }
    }

    // entry to the maze is not drivable.
    maze[x][y].walls[(dir+2)%4] = WALL;
}

/**
 * per-cell routine for the explore phase.
 *
 * Performs DFS on the maze, writing the obtained information
 * to the maze matrix.
 *
 * This has to happen while driving - we don't want to stop
 * for a few ms in every cell.
 * Suggestion: Call this from the motor control once the next
 * cell is reached. This means that we have to implement this
 * iteratively (not recursive) and store the complete state
 * globally.
 * Here, the complete state is stored in the cell.flag and
 * the walls of the cell.
 *
 * @param x Current x position
 * @param y Current y position
 * @param dir current orientation wrt. the maze
 *
 * @return the direction in which to turn wrt. the mouse
 */
direction explore(unsigned int x, unsigned int y, orientation dir) {

    // current cell
    cell* pos = &maze[x][y];

    // walls wrt. the current orientation
    wall left = UNKNOWN;
    wall front = UNKNOWN;
    wall right = UNKNOWN;

    // global direction which cell to explore next
    orientation next;

    // next actions depend on the state of the cell
    switch (pos->flag) {
        case 0:
            // first time visiting this cell
            // retrieve and fill in new info
            //TODO interpret sensor values to define
            DEBUG_get_walls(&left, &front, &right);

            pos->walls[(dir-1)%4] = left;
            pos->walls[dir] = front;
            pos->walls[(dir+1)%4] = right;

            // Set entrance. Special case in the start cell
            if(pos->walls[(dir+2)%4] != WALL)
                pos->walls[(dir+2)%4] = ENTRY;

            pos->flag = 1;
            // continue exploring (no break!)
        case 1:
            // cell not yet fully explored, walls known
            // select next open wall to explore
            //TODO: prefer going straight
            for (next = 0; next < 4; next ++) {
                if (pos->walls[next] == WAY) {
                    // will be fully explored when we come back
                    pos->walls[next] = EXPLORED;
                    //go on to explore this path
                    return (next - dir)%4;
                }
            }

            // didn't find a wall to explore: completed cell
            pos->flag = 2;
            // walk back (no break!)
        case 2:
            // cell fully explored, walk back to where we came from
            for (next = 0; next < 4; next++) {
                if (pos->walls[next] == ENTRY) {
                    //drive that way
                    return (next - dir)%4;
                }
            }

            // don't come from anywhere: must be the start cell
            return STOP;
    }

}


/**
 * Find the shortest path to the goal.
 *
 * This function executes BFS on the maze,
 * using the data from the exploration phase, and returns
 * an array containing all necessary directions.
 * This offline-approach saves computation time during the race.
 * Directions are given for every cell.
 * The array has to be freed by the caller.
 *
 * @param x Start position in x direction
 * @param y Start position in y direction
 * @param dir Orientation at the start
 *
 * @param x_dest Goal position in x direction
 * @param y_dest Goal position in y direction
 *
 * @return Array of directions leading to the goal
 */
direction* exploit(unsigned int x, unsigned int y, orientation dir,
                   unsigned int x_dest, unsigned int y_dest) {

    // prepare the maze: use flag for distance
    for (int i = 0; i < SIZE; i++) {
        for (int j = 0; j < SIZE; j++) {
            maze[i][j].flag = -1;
            for (int k = 0; k < 4; k++) {
                maze[i][j].walls[k] = (maze[i][j].walls[k] > 0);
            }
        }
    }
    maze[x][y].flag = 0;

    cell *current;
    cell *next;

    //... create frontier, perform BFS,

    //we need max. SIZE elements in the frontier:
    //max number of elements with equal distance to the start point
    coord frontier[SIZE*SIZE];
    int head = 0;
    int tail = 0;
    frontier[tail].x = x;
    frontier[tail].y = y;
    tail ++;

    //until goal found, iterate over frontier
    while(1) {
        coord pos = frontier[head];
        current = &maze[pos.x][pos.y];

        if(pos.x == x_dest && pos.y == y_dest) {
            break;
        }

        if(current->walls[NORTH] == WAY) {
            next = &maze[pos.x][pos.y+1];
            if(next->flag < 0) {
                next->walls[SOUTH] = ENTRY;
                next->flag = current->flag + 1;
                frontier[tail].x = pos.x;
                frontier[tail].y = pos.y + 1;
                tail = (tail + 1) % (SIZE*SIZE);
            }
        }

        if(current->walls[EAST] == WAY) {
            next = &maze[pos.x+1][pos.y];
            if(next->flag < 0) {
                next->walls[WEST] = ENTRY;
                next->flag = current->flag + 1;
                frontier[tail].x = pos.x + 1;
                frontier[tail].y = pos.y;
                tail = (tail + 1) % (SIZE*SIZE);
            }
        }

        if(current->walls[SOUTH] == WAY) {
            next = &maze[pos.x][pos.y-1];
            if(next->flag < 0) {
                next->walls[NORTH] = ENTRY;
                next->flag = current->flag + 1;
                frontier[tail].x = pos.x;
                frontier[tail].y = pos.y - 1;
                tail = (tail + 1) % (SIZE*SIZE);
            }
        }

        if(current->walls[WEST] == WAY) {
            next = &maze[pos.x-1][pos.y];
            if(next->flag < 0) {
                next->walls[EAST] = ENTRY;
                next->flag = current->flag + 1;
                frontier[tail].x = pos.x - 1;
                frontier[tail].y = pos.y;
                tail = (tail + 1) % (SIZE*SIZE);
            }
        }

        head = (head+1)%(SIZE*SIZE);
    }

    // calc and save directions for optimal path ...
    int path_len = maze[x_dest][y_dest].flag;
    direction *path = malloc((path_len+1) * sizeof(direction));

    coord pos = {x_dest, y_dest};
    path[path_len] = STOP;

    for(int i = path_len-1; i >= 0; i--) {
        for(int w = 0; w < 4; w++) {
            if(maze[pos.x][pos.y].walls[w] == ENTRY) {
                // save global direction first,
                // convert to driving directions afterwards
                path[i] = (w+2)%4;

                switch (w) {
                    case NORTH:
                        pos.y ++;
                        break;
                    case EAST:
                        pos.x ++;
                        break;
                    case SOUTH:
                        pos.y --;
                        break;
                    case WEST:
                        pos.x --;
                        break;
                    default:
                        //start reached
                        break;
                }
                break;
            }
        }
    }

    if(pos.x != x || pos.y != y) {
        printf("ERROR start position mismatch\n");
    }

    //convert orientation to direction
    for(int i = 0; i < path_len; i++) {
        orientation tmp = (orientation) path[i];
        path[i] = (path[i] - dir)%4;
        dir = tmp;
    }
    printf("\n");

    return path;
}
