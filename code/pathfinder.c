#include <stdio.h>
#include "pathfinder.h"

/** internal representation of the grid as a matrix of cells */
cell maze[SIZE][SIZE];

///////////////////////////////////////////////////////////
//                       DEBUG                           //
///////////////////////////////////////////////////////////

cell env[8][8];
int x;
int y;
orientation dir;

void DEBUG_get_walls(int *left, int *front, int *right) {
    *left = env[x][y].walls[(dir-1)%4];
    *front = env[x][y].walls[dir];
    *right = env[x][y].walls[(dir+1)%4];
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
    cell pos = maze[x][y];

    // walls wrt. the current orientation
    wall left = UNKNOWN;
    wall front = UNKNOWN;
    wall right = UNKNOWN;

    // global direction which cell to explore next
    orientation next;

    // next actions depend on the state of the cell
    switch (pos.flag) {
        case 0:
            // first time visiting this cell
            // retrieve and fill in new info
            //TODO interpret sensor values to define
            DEBUG_get_walls(&left, &front, &right);

            pos.walls[(dir-1)%4] = left;
            pos.walls[dir] = front;
            pos.walls[(dir+1)%4] = right;

            // Set entrance. Special case in the start cell
            if(pos.walls[(dir+2)%4] != WALL)
                pos.walls[(dir+2)%4] = ENTRY;

            pos.flag = 1;
            // continue exploring (no break!)
        case 1:
            // cell not yet fully explored, walls known
            // select next open wall to explore
            //TODO: prefer going straight
            for (next = 0; next < 4; next ++) {
                if (pos.walls[next] == WAY) {
                    // will be fully explored when we come back
                    pos.walls[next] = EXPLORED;
                    //go on to explore this path
                    return (next - dir)%4;
                }
            }

            // didn't find a wall to explore: completed cell
            pos.flag = 2;
            // walk back (no break!)
        case 2:
            // cell fully explored, walk back to where we came from
            for (next = 0; next < 4; next++) {
                if (pos.walls[next] == ENTRY) {
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
 * This function executes A* search on the maze,
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
direction* exploit(unsigned int x, unsigned int y, direction dir,
                   unsigned int x_dest, unsigned int y_dest) {

    // prepare the maze: use flag for distance
    for (int i = 0; i < SIZE; i++) {
        for (int j = 0; j < SIZE; j++) {
            maze[i][j].flag = INF;
            for (int k = 0; k < 4; k++) {
                maze[i][j].walls[k] = (maze[i][j].walls[k] > 0);
            }
        }
    }
    maze[x_dest][y_dest].flag = 0;

    cell *current = &maze[x][y];

    //... create frontier, perform A*,
    // calc and save directions for optimal path ...

    return NULL;
}

int main() {

    x = 0;
    y = 0;
    dir = NORTH;
    direction move = STOP;

    //TODO initialize env for different test cases

    initMaze(x, y, dir);

    do {
        move = explore(x, y, dir);
        dir = (dir+move)%4;

        switch (dir) {
            case NORTH:
                y ++;
                break;
            case EAST:
                x++;
                break;
            case SOUTH:
                y--;
                break;
            case WEST:
                x--;
                break;
            default:
                break;
        }
        printf("%d, %d", x, y);
    } while (move != STOP);

    return 0;
}
