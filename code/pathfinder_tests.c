#include "pathfinder.h"
//#include "hw_tests.h"
#include "pathfinder_tests.h"
#include <stdio.h>

cell env[SIZE][SIZE];
int x;
int y;
orientation dir;

plannerState current_state_planner;


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
    *left = env[x][y].walls[(dir-1)%4];
    *front = env[x][y].walls[dir];
    *right = env[x][y].walls[(dir+1)%4];
}

/**
 * Mock for querying whether the desired motion was completed by the MC.
 * 
 * To be implemented using an additional mouse state.
 * 
 * @return 1 if completed, 0 otherwise
 */
int DEBUG_get_completed() {
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
void DEBUG_set_state(direction newState) {
    debug_delay = 100;
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

int test_main() {

    DEBUG_setup_maze();

    x = 0;
    y = 0;
    dir = EAST;
    direction move;


    initMaze(x, y, dir);
    prettyPrintMaze(env);
    printf("\n");

    do {
        move = explore(x, y, dir);
        dir = (dir+move)%4;
        printMaze(DEBUG_get_maze());

        if(move != STOP) {
            switch (dir) {
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
        printf("go to %d, %d\n\n", x, y);
    } while (move != STOP);

    prettyPrintMaze(DEBUG_get_maze());

    direction *path = exploit(x, y, EAST, 2, 0);

    printf("Final path: ");
    for(int i = 0;; i++) {
        printf("%d, ", path[i]);
        if(path[i] == STOP) {
            printf("\n");
            break;
        }
    }

    return 0;
}


/**
 * Concept of the DFA for the motion plnner.
 */
int testAutomaton_main() {

    // setup test environment
    DEBUG_setup_maze();
    
    ////////////////////////////////////////////////////////////
    //              1. Wait for Explore Phase                 //
    ////////////////////////////////////////////////////////////
    
    // busy wait until started from switch ISR
    //TODO: change state from switch ISR
    //while(current_state_planner == WAIT_EXPLORE);
    
    ////////////////////////////////////////////////////////////
    //                  2. Explore Phase                      //
    ////////////////////////////////////////////////////////////
    
    direction move = STOP;
    x = 0;
    y = 0;
    dir = NORTH;

    initMaze(x, y, dir);
    
    // loop until explore phase completed
    do {

        // plan next step. Includes collecting sensor information
        move = explore(x, y, dir);
        
        
        // execute the movement
        /// 1. turn if necessary
        if(move != FRONT) {
            DEBUG_set_state(move);
            // wait for completion
            while (DEBUG_get_completed() == 0);
        }
        /// 2. move forward
        DEBUG_set_state(FRONT);
        // wait for completion
        while (DEBUG_get_completed() == 0);
        
        
        // update internal state representation
        dir = (dir+move)%4;

        if(move != STOP) {
            switch (dir) {
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
    } while(move != STOP);
    
    //TODO: turn back to start orientation
    
    
    ////////////////////////////////////////////////////////////
    //              3. Wait for Exploit Phase                 //
    ////////////////////////////////////////////////////////////
    
    current_state_planner = WAIT_EXPLOIT;
    
    // busy wait until started from switch ISR
    //TODO: change state from switch ISR
    //while(current_state_planner == WAIT_EXPLOIT);
    
    
    ////////////////////////////////////////////////////////////
    //                  4. Exploit Phase                      //
    ////////////////////////////////////////////////////////////
    
    // plan path
    direction *path = exploit(x, y, EAST, 2, 0);

    // replay path
    for(int i = 0; path[i] != STOP; i++) {
        DEBUG_set_state(path[i]);
        // wait for completion
        while (DEBUG_get_completed() == 0);
        printf("%d", path[i]);

    }

    // COMPLETE!
    
    return 0;
}
