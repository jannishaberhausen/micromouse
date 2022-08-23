#include "pathfinder.h"
#include <stdio.h>

cell env[4][4];
int x;
int y;
orientation dir;

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

int test_main() {

    x = 0;
    y = 0;
    dir = EAST;
    direction move;

    /*
     * Example maze:
     *
     *   +---+---+---+---+
     * 3 |               |
     *   +---+---+---+---+
     * 2 |               |
     *   +   +---+   +   +
     * 1 |   |       |   |
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
    env[0][1].walls[1]=WALL;
    env[0][1].walls[2]=WALL;
    env[0][1].walls[3]=WALL;

    env[1][1].walls[0]=WALL;
    env[1][1].walls[1]=WAY;
    env[1][1].walls[2]=WAY;
    env[1][1].walls[3]=WALL;

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
