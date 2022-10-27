#include "xc.h"
#include <stdio.h>
#include <stdlib.h>
#include "pathfinder.h"
#include "mouse_motion.h"
#include "IOconfig.h"
#include "sharp.h"
//#include "pathfinder_tests.h" //REMOVE AFTER TESTING

/** internal representation of the grid as a matrix of cells */
cell maze[SIZE][SIZE];

typedef struct coord {unsigned int x; unsigned int y;} coord;

// internal state
coord position;
orientation dir;
plannerState current_state_planner;

int race_path_index;

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
 * Find the coordinates of the goal region in an explored maze.
 * 
 * @return the coordinates of the south-west corner goal region
 */
coord findGoal() {
    for (int i = 0; i < SIZE-1; i++) {
        for (int j = 0; j < SIZE-1; j++) {
            // check for four connected cells
            if (maze[i][j].walls[NORTH] == WALL 
                    || maze[i][j].walls[EAST] == WALL
                    || maze[i+1][j+1].walls[SOUTH] == WALL
                    || maze[i+1][j+1].walls[WEST] == WALL) {
                continue;
            }
            // check for only one entry
            int entries = 0;
            if (maze[i][j].walls[WEST] != WALL)
                entries++;
            if (maze[i][j].walls[SOUTH] != WALL)
                entries++;
            if (maze[i+1][j].walls[SOUTH] != WALL)
                entries++;
            if (maze[i+1][j].walls[EAST] != WALL)
                entries++;
            if (maze[i+1][j+1].walls[EAST] != WALL)
                entries++;
            if (maze[i+1][j+1].walls[NORTH] != WALL)
                entries++;
            if (maze[i][j+1].walls[NORTH] != WALL)
                entries++;
            if (maze[i][j+1].walls[WEST] != WALL)
                entries++;
            
            if(entries != 1)
                continue;
            
            return (coord) {i, j};
        }
    }
    return (coord) {0, 0};
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
            get_walls(&left, &front, &right);

            // only update unknown walls
            if(pos->walls[(dir-1)%4] == UNKNOWN)
                pos->walls[(dir-1)%4] = left;
            if(pos->walls[dir] == UNKNOWN)
                pos->walls[dir] = front;
            if(pos->walls[(dir+1)%4] == UNKNOWN)
                pos->walls[(dir+1)%4] = right;
            
            // in case there are unexplored walls in neighboring cells
            // that have already been visited, update them as well.
            // See the test maze, west wall of cell (1, 1) when exploring (0, 2)
            if (x > 0 && (&maze[x-1][y])->walls[EAST] == WAY) {
                (&maze[x-1][y])->walls[EAST] = EXPLORED;
                pos->walls[WEST] = EXPLORED;
            }
            if (x < SIZE-1 && (&maze[x+1][y])->walls[WEST] == WAY) {
                (&maze[x+1][y])->walls[WEST] = EXPLORED;
                pos->walls[EAST] = EXPLORED;
            }
            if (y > 0 && (&maze[x][y-1])->walls[NORTH] == WAY) {
                (&maze[x][y-1])->walls[NORTH] = EXPLORED;
                pos->walls[SOUTH] = EXPLORED;
            }
            if (y < SIZE-1 && (&maze[x][y+1])->walls[SOUTH] == WAY) {
                (&maze[x][y+1])->walls[SOUTH] = EXPLORED;
                pos->walls[NORTH] = EXPLORED;
            }

            // Set entrance. Special case in the start cell
            if(pos->walls[(dir+2)%4] != WALL)
                pos->walls[(dir+2)%4] = ENTRY;

            pos->flag = 1;
            // continue exploring (no break!)
        case 1:
            // cell not yet fully explored, walls known
            // select next open wall to explore
            for (next = 0; next < 4; next ++) {
                if (pos->walls[(dir+next)%4] == WAY) {
                    // will be fully explored when we come back
                    pos->walls[(dir+next)%4] = EXPLORED;
                    //go on to explore this path
                    return next;
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

    // should never be reached
    return STOP;
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
void exploit(unsigned int x, unsigned int y, orientation dir,
                   unsigned int x_dest, unsigned int y_dest) {

    // prepare the maze: use flag for distance
    for (int i = 0; i < SIZE; i++) {
        for (int j = 0; j < SIZE; j++) {
            maze[i][j].flag = -1;
            for (int k = 0; k < 4; k++) {
                maze[i][j].walls[k] = (maze[i][j].walls[k] == 1 || maze[i][j].walls[k] == -1) ? WALL : WAY;
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
        
        orientation entry_orientation = dir;
        for(int i = 0; i < 4; i++) {
            if(current->walls[i] == ENTRY)
                entry_orientation = (i+2)%4;
        }

        for (int i = 0; i < 4; i++) {
            orientation side = (entry_orientation+i)%4;
            
            if(current->walls[side] == WAY) {
                // move that way
                coord next_pos;
                switch(side) {
                    case NORTH:
                        next_pos.x = pos.x;
                        next_pos.y = pos.y + 1;
                        break;
                    case EAST:
                        next_pos.x = pos.x + 1;
                        next_pos.y = pos.y;
                        break;
                    case SOUTH:
                        next_pos.x = pos.x;
                        next_pos.y = pos.y - 1;
                        break;
                    case WEST:
                        next_pos.x = pos.x - 1;
                        next_pos.y = pos.y;
                        break;
                }
                next = &maze[next_pos.x][next_pos.y];

                // if not yet visited, add to frontier
                if(next->flag < 0) {
                    next->walls[(side+2)%4] = ENTRY;
                    next->flag = current->flag + 1;
                    frontier[tail].x = next_pos.x;
                    frontier[tail].y = next_pos.y;
                    tail = (tail + 1) % (SIZE*SIZE);
                }
            }
        }
        
        head = (head+1)%(SIZE*SIZE);
    }

    // calc and save directions for optimal path ...
    int path_len = maze[x_dest][y_dest].flag;
    direction path[path_len+1];
    direction race_path[path_len+2];

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
        //printf("ERROR start position mismatch\n");
    }

    //convert orientation to direction
    for(int i = 0; i < path_len; i++) {
        orientation tmp = (orientation) path[i];
        path[i] = (path[i] - dir)%4;
        dir = tmp;
    }

    
    /* convert the path[] for the exploit phase and save the transformed path
     *  in a new variable race_path[]
     */
    race_path_index = 0;
    if (path[0] == RIGHT) {
        race_path[0] = ROTATE_RIGHT;
        race_path[1] = HALF_FRONT;
        race_path_index = 2;
    } else if (path[0] == LEFT) {
        race_path[0] = ROTATE_LEFT;
        race_path[1] = HALF_FRONT;
        race_path_index = 2;
    } else if (path[0] == FRONT) {
        race_path[0] = HALF_FRONT;
        race_path_index = 1;
    }
    for(int i = 1; path[i] != STOP; i++) {
        race_path[race_path_index] = path[i];
        race_path_index++;
    }
    race_path[race_path_index] = HALF_FRONT;
    race_path[race_path_index+1] = STOP;
    
    // replay path (experimental raceing version)
    while(1) {
    
        // we need the button to start
        setupSwitch();

        // value will be changed by the button ISR
        current_state_planner = WAIT_EXPLOIT;
        while(current_state_planner == WAIT_EXPLOIT);
        current_state_planner = EXPLOIT;
    
        // we need the motors to drive
        setMotionState(STOP);
        setupMotors();
        resetController();
        
        
        /*#############################################################################
         ##### DRIVE THE SHORTEST PATH WITH REGULAR CONTROLLER (WORKING VERSION) ######
         ############################################################################*/
        /*
        // replay path (working version)
        for(int i = 0; path[i] != STOP; i++) {
            setMotionState(path[i]);
            // wait for completion
            while (!getMotionCompleted());
        }
        setMotionState(STOP);
         */
        
        
        /*#############################################################################
         ### DRIVE THE SHORTEST PATH WITH RACING CONTROLLER (EXPERIMENTAL VERSION) ####
         ############################################################################*/
        BASE_SPEED = 30;
        for(int i = 0; race_path[i] != STOP; i++) {
    //        if(race_path[i] == FRONT) {
    //            if(race_path[i+1] == FRONT) {
    //                BASE_SPEED = 40;
    //            } else {
    //                BASE_SPEED = 30;
    //            }
    //        } else {
    //            BASE_SPEED = 20;
    //        }

            setRaceMotionState(race_path[i]);
            // wait for completion
            while (!getRaceMotionCompleted());
        }
        setRaceMotionState(STOP);
    }
}


/**
 * Main function of the motion planner.
 * 
 * Implements the (inlined...) finite state machine for the motion planner
 * and path finder. 
 */
void plannerFSM() {
    
    LED2 = LEDON;
    LED4 = LEDON;
    
    ////////////////////////////////////////////////////////////
    //              1. Wait for Explore Phase                 //
    ////////////////////////////////////////////////////////////
    
    // we need the button to start
    setMotionState(EMPTY);
    setMotorDirections_Disable();
    setupSwitch();
    
    // value will be changed by the button ISR
    current_state_planner = WAIT_EXPLORE;
    while(current_state_planner == WAIT_EXPLORE);
    current_state_planner = EXPLORE;
    
    
    LED2 = LEDON;
    LED4 = LEDON;
    
    
    ////////////////////////////////////////////////////////////
    //                  2. Explore Phase                      //
    ////////////////////////////////////////////////////////////
    
    // we need motors, encoders, and sensors to drive
    setMotionState(STOP);
    setupMotors();
    setupEncoders();
    setupSensors();
    resetController();
    
    // initialize internal state
    direction move = STOP;
    position.x = 0;
    position.y = 0;
    dir = NORTH;

    initMaze(position.x, position.y, dir);
    
    // loop until explore phase completed
    do {
        
        // plan next step. Includes collecting sensor information
        move = explore(position.x, position.y, dir);
        
        // start the execution of the motion
        setMotionState(move);
        
        // update internal state representation
        dir = (dir+move)%4;

        if(move != STOP) {
            switch (dir) {
                case NORTH:
                    position.y++;
                    break;
                case EAST:
                    position.x++;
                    break;
                case SOUTH:
                    position.y--;
                    break;
                case WEST:
                    position.x--;
                    break;
                default:
                    break;
            }
        }
        
        // wait for completion of the motion
        while (!getMotionCompleted());
        
    } while(move != STOP);
    
    
    LED2 = LEDOFF;
    LED4 = LEDOFF;
    
    setMotionState(EMPTY);
    
    // turn back to start orientation.
    // Uses rotation functions directly to avoid driving forward afterwards
    switch(dir) {
        case EAST:
            driveLeftTurn(90);
            break;
        case SOUTH:
            driveLeftTurn(180);
            break;
        case WEST:
            driveRightTurn(90);
            break;
        default:
            break;
    }
    
    dir = NORTH;
    
    brake();
    
    setMotorDirections_Disable();
    
    ////////////////////////////////////////////////////////////
    //              3. Wait for Exploit Phase                 //
    ////////////////////////////////////////////////////////////
    
    
    
    ////////////////////////////////////////////////////////////
    //                  4. Exploit Phase                      //
    ////////////////////////////////////////////////////////////
    
    
    LED2 = LEDON;
    LED4 = LEDON;
    
    // find goal location
    coord goal = findGoal();
    exploit(position.x, position.y, dir, goal.x, goal.y);
    
    LED2 = LEDOFF;
    LED4 = LEDON;
    /*

    // replay path
    for(int i = 0; path[i] != STOP; i++) {
        setMotionState(path[i]);
        // wait for completion
        while (!getMotionCompleted());
    }
    setMotionState(STOP);

    // COMPLETE!
    */
}