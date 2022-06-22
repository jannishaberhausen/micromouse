#include "pathfinder.h"
#include <stdio.h>

cell env[8][8];
int x;
int y;
orientation dir;

void printMaze(cell** maze) {


    for(int y = 0; y < SIZE; y++) {
        //north
        for(int x = 0; x < SIZE; x++) {
            printf("+");
            switch (maze[x][y].walls[NORTH]) {
                case UNKNOWN:
                    printf("?");
                    break;
                case ENTRY:
                    printf("v");
                    break;
                case WAY:
                    printf(" ");
                    break;
                case EXPLORED:
                    printf("O");
                    break;
                case WALL:
                    printf("-");
                    break;
            }
            printf("+");
        }
        printf("\n");

        //center
        for(int x = 0; x < SIZE; x++) {
            switch (maze[x][y].walls[WEST]) {
                case UNKNOWN:
                    printf("?");
                    break;
                case ENTRY:
                    printf(">");
                    break;
                case WAY:
                    printf(" ");
                    break;
                case EXPLORED:
                    printf("O");
                    break;
                case WALL:
                    printf("|");
                    break;
            }
            printf("%d", maze[x][y].flag);
            switch (maze[x][y].walls[EAST]) {
                case UNKNOWN:
                    printf("?");
                    break;
                case ENTRY:
                    printf("<");
                    break;
                case WAY:
                    printf(" ");
                    break;
                case EXPLORED:
                    printf("O");
                    break;
                case WALL:
                    printf("|");
                    break;
            }
        }
        printf("\n");

        //south
        for(int x = 0; x < SIZE; x++) {
            printf("+");
            switch (maze[x][y].walls[SOUTH]) {
                case UNKNOWN:
                    printf("?");
                    break;
                case ENTRY:
                    printf("^");
                    break;
                case WAY:
                    printf(" ");
                    break;
                case EXPLORED:
                    printf("O");
                    break;
                case WALL:
                    printf("-");
                    break;
            }
            printf("+");
        }
        printf("\n");
    }
}


void DEBUG_initMaze(char* input, cell**maze) {
    for(int y = 0; y < SIZE; y++) {
        for(int x = 0; x < SIZE; x++) {
            
        }
    }
}

void DEBUG_get_walls(int *left, int *front, int *right) {
    *left = env[x][y].walls[(dir-1)%4];
    *front = env[x][y].walls[dir];
    *right = env[x][y].walls[(dir+1)%4];
}

int main() {

    x = 0;
    y = 0;
    dir = NORTH;
    direction move;

    //TODO initialize env for different test cases

    initMaze(x, y, dir);

    do {
        move = explore(x, y, dir);
        dir = (dir+move)%4;
        printMaze(DEBUG_get_maze());

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

    exploit(x, y, NORTH, SIZE/2, SIZE/2);

    printMaze(DEBUG_get_maze());

    return 0;
}
