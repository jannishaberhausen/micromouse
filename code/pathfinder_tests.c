#include "pathfinder.h"
//#include "hw_tests.h"
#include "IOconfig.h"
#include "pathfinder_tests.h"
#include "mouse_motion.h"
#include <stdio.h>

cell env[SIZE][SIZE];
int x;
int y;
orientation test_dir;

plannerState test_current_state_planner;


// part of motors.h in the real environment

// simulates the time required to execute a motion
int debug_delay = 100;

/**
 * Print the maze in its internal representation.
 *
 * For debugging purposes. Shows the exact status of all walls
 * and the value of the flag for all cells, as used in the pathfinder.
 *
 * Encoding:
 *      `?`                 :   UNKNOWN
 *      `---`, `|`          :   WALL
 *      ` `                 :   WAY
 *      `O`                 :   EXPLORED
 *      `^`, `>`, `v`, `<`  :   ENTRY
 *
 * @param maze the maze to be printed
 */
void printMaze(cell maze[SIZE][SIZE]) {


    for(int y = SIZE-1; y >= 0; y--) {
        //north
        for(int x = 0; x < SIZE; x++) {
            printf(" +");
            switch (maze[x][y].walls[NORTH]) {
                case UNKNOWN:
                    printf(" ? ");
                    break;
                case ENTRY:
                    printf(" v ");
                    break;
                case WAY:
                    printf("   ");
                    break;
                case EXPLORED:
                    printf(" O ");
                    break;
                case WALL:
                    printf("---");
                    break;
            }
            printf("+ ");
        }
        printf("\n");

        //center
        for(int x = 0; x < SIZE; x++) {
            switch (maze[x][y].walls[WEST]) {
                case UNKNOWN:
                    printf(" ?");
                    break;
                case ENTRY:
                    printf(" >");
                    break;
                case WAY:
                    printf("  ");
                    break;
                case EXPLORED:
                    printf(" O");
                    break;
                case WALL:
                    printf(" |");
                    break;
            }

            if(maze[x][y].flag < 0)
                printf(" - ");
            else
                printf(" %d ", maze[x][y].flag);

            switch (maze[x][y].walls[EAST]) {
                case UNKNOWN:
                    printf("? ");
                    break;
                case ENTRY:
                    printf("< ");
                    break;
                case WAY:
                    printf("  ");
                    break;
                case EXPLORED:
                    printf("O ");
                    break;
                case WALL:
                    printf("| ");
                    break;
            }
        }
        printf("\n");

        //south
        for(int x = 0; x < SIZE; x++) {
            printf(" +");
            switch (maze[x][y].walls[SOUTH]) {
                case UNKNOWN:
                    printf(" ? ");
                    break;
                case ENTRY:
                    printf(" ^ ");
                    break;
                case WAY:
                    printf("   ");
                    break;
                case EXPLORED:
                    printf(" O ");
                    break;
                case WALL:
                    printf("---");
                    break;
            }
            printf("+ ");
        }
        printf("\n");
    }
}

/**
 * Print the maze for fast and nice display.
 *
 * Shows only the layout of the maze, not its internal state.
 * Avoids showing the same wall twice as seen from different cells.
 *
 * Encoding:
 *      `?`                 :   UNKNOWN
 *      `---`, `|`          :   WALL
 *      ` `                 :   WAY, EXPLORED, ENTRY
 *
 * @param maze the maze to be printed
 */
void prettyPrintMaze(cell maze[SIZE][SIZE]) {


    for(int y = SIZE-1; y >= 0; y--) {
        //north
        for(int x = 0; x < SIZE; x++) {
            printf("+");
            switch (maze[x][y].walls[NORTH]) {
                case UNKNOWN:
                    printf(" ? ");
                    break;
                case WALL:
                    printf("---");
                    break;
                default:
                    printf("   ");
                    break;
            }
        }
        printf("+\n");

        //center
        for(int x = 0; x < SIZE; x++) {
            switch (maze[x][y].walls[WEST]) {
                case UNKNOWN:
                    printf("?");
                    break;
                case WALL:
                    printf("|");
                    break;
                default:
                    printf(" ");
                    break;
            }

            printf(" %d ", maze[x][y].flag);
        }

        switch (maze[SIZE-1][y].walls[EAST]) {
            case UNKNOWN:
                printf("?");
                break;
            case WALL:
                printf("|");
                break;
            default:
                printf(" ");
                break;
        }
        printf("\n");
    }

    //south
    for(int x = 0; x < SIZE; x++) {
        printf("+");
        switch (maze[x][y].walls[SOUTH]) {
            case UNKNOWN:
                printf(" ? ");
                break;
            case WALL:
                printf("---");
                break;
            default:
                printf("   ");
                break;
        }
    }
    printf("+\n");
}


/**
 * Mock for reading and evaluating sensor values
 * to determine wall positions.
 *
 * @param [OUT] left is there a wall on the left
 * @param [OUT] front is there a wall in the front
 * @param [OUT] right is there a wall on the right
 */
void DEBUG_get_walls(int *left, int *front, int *right) {
    *left = env[x][y].walls[(test_dir-1)%4];
    *front = env[x][y].walls[test_dir];
    *right = env[x][y].walls[(test_dir+1)%4];
}

/**
 * Mock for querying whether the desired motion was completed by the MC.
 * 
 * To be implemented using an additional mouse state.
 * 
 * @return 1 if completed, 0 otherwise
 */
int DEBUG_getMotionCompleted() {
    debug_delay --;
    if(debug_delay == 0) {
        debug_delay = 100;
        return 1;
    }
    return 0;
}

/**
 * Mock for setting the state of the MC.
 * 
 * To be implemented by setting the mouseState.
 */
void DEBUG_setMotionState(direction newState) {
    debug_delay = 100;
    printf("%d\n", newState);
    //mouseState = newState;
}

void DEBUG_setup_maze() {

    /*
     * Example maze:
     *
     *   +---+---+---+---+
     * 3 |               |
     *   +---+---+---+---+
     * 2 |               |
     *   +   +---+   +   +
     * 1 |           |   |
     *   +---+   +---+   +
     * 0 | s     | g     |
     *   +---+---+---+---+
     *     0   1   2   3
     */


    env[0][0].walls[0]=WALL;
    env[0][0].walls[1]=WAY;
    env[0][0].walls[2]=WALL;
    env[0][0].walls[3]=WALL;

    env[1][0].walls[0]=WAY;
    env[1][0].walls[1]=WALL;
    env[1][0].walls[2]=WALL;
    env[1][0].walls[3]=WAY;

    env[2][0].walls[0]=WALL;
    env[2][0].walls[1]=WAY;
    env[2][0].walls[2]=WALL;
    env[2][0].walls[3]=WALL;

    env[3][0].walls[0]=WAY;
    env[3][0].walls[1]=WALL;
    env[3][0].walls[2]=WALL;
    env[3][0].walls[3]=WAY;

    env[0][1].walls[0]=WAY;
    env[0][1].walls[1]=WAY;
    env[0][1].walls[2]=WALL;
    env[0][1].walls[3]=WALL;

    env[1][1].walls[0]=WALL;
    env[1][1].walls[1]=WAY;
    env[1][1].walls[2]=WAY;
    env[1][1].walls[3]=WAY;

    env[2][1].walls[0]=WAY;
    env[2][1].walls[1]=WALL;
    env[2][1].walls[2]=WALL;
    env[2][1].walls[3]=WAY;

    env[3][1].walls[0]=WAY;
    env[3][1].walls[1]=WALL;
    env[3][1].walls[2]=WAY;
    env[3][1].walls[3]=WALL;

    env[0][2].walls[0]=WALL;
    env[0][2].walls[1]=WAY;
    env[0][2].walls[2]=WAY;
    env[0][2].walls[3]=WALL;

    env[1][2].walls[0]=WALL;
    env[1][2].walls[1]=WAY;
    env[1][2].walls[2]=WALL;
    env[1][2].walls[3]=WAY;

    env[2][2].walls[0]=WALL;
    env[2][2].walls[1]=WAY;
    env[2][2].walls[2]=WAY;
    env[2][2].walls[3]=WAY;

    env[3][2].walls[0]=WALL;
    env[3][2].walls[1]=WALL;
    env[3][2].walls[2]=WAY;
    env[3][2].walls[3]=WAY;

    env[0][3].walls[0] = WALL;
    env[0][3].walls[1] = WAY;
    env[0][3].walls[2] = WALL;
    env[0][3].walls[3] = WALL;

    env[1][3].walls[0] = WALL;
    env[1][3].walls[1] = WAY;
    env[1][3].walls[2] = WALL;
    env[1][3].walls[3] = WAY;

    env[2][3].walls[0] = WALL;
    env[2][3].walls[1] = WAY;
    env[2][3].walls[2] = WALL;
    env[2][3].walls[3] = WAY;

    env[3][3].walls[0] = WALL;
    env[3][3].walls[1] = WALL;
    env[3][3].walls[2] = WALL;
    env[3][3].walls[3] = WAY;

}


/**
 * Concept of the DFA for the motion plnner.
 */
int test_main() {

    // setup test environment
    DEBUG_setup_maze();
    // print the test environment
    prettyPrintMaze(env);
    
    ////////////////////////////////////////////////////////////
    //              1. Wait for Explore Phase                 //
    ////////////////////////////////////////////////////////////
    
    // busy wait until started from switch ISR
    //while(test_current_state_planner == WAIT_EXPLORE);
    
    ////////////////////////////////////////////////////////////
    //                  2. Explore Phase                      //
    ////////////////////////////////////////////////////////////
    
    direction move;
    x = 0;
    y = 0;
    test_dir = NORTH;

    initMaze(x, y, test_dir);
    
    // loop until explore phase completed
    do {

        // plan next step. Includes collecting sensor information
        move = explore(x, y, test_dir);
        
        // execute the movement
        /// 1. turn if necessary
        if(move != FRONT) {
            DEBUG_setMotionState(move);
            // wait for completion
            while (DEBUG_getMotionCompleted() == 0);
        }
        /// 2. move forward
        DEBUG_setMotionState(FRONT);
        // wait for completion
        while (DEBUG_getMotionCompleted() == 0);
        
        
        // update internal state representation
        test_dir = (test_dir+move)%4;

        if(move != STOP) {
            switch (test_dir) {
                case NORTH:
                    y++;
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
        }
        
        
        // print the updated map, and the next action
        printMaze(DEBUG_get_maze());
        printf("Moving in direction %d, to cell %d, %d, facing %d\n", move, x, y, test_dir);
        
    } while(move != STOP);
    
    //TODO: turn back to start orientation
    test_dir = NORTH;
    
    
    ////////////////////////////////////////////////////////////
    //              3. Wait for Exploit Phase                 //
    ////////////////////////////////////////////////////////////
    
    test_current_state_planner = WAIT_EXPLOIT;
    
    // busy wait until started from switch ISR
    //while(test_current_state_planner == WAIT_EXPLOIT);
    
    
    ////////////////////////////////////////////////////////////
    //                  4. Exploit Phase                      //
    ////////////////////////////////////////////////////////////
    
    // plan path
    exploit(x, y, test_dir, 2, 0);

    // COMPLETE!
    
    return 0;
}


void testMotionSequence() {
    
    direction path[] = {FRONT, STOP};
    for(int i = 0; path[i] != STOP; i++) {
        setMotionState(path[i]);
        // wait for completion
        while (!getMotionCompleted());
    }
     
    //setMotionState(RIGHT);
    //while (getMotionCompleted() == 0);
    setMotionState(STOP);
    
    
    //driveRightTurn(90);
    //driveForward();
}