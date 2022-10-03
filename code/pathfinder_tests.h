#ifndef PATHFINDER_TESTS_H
#define PATHFINDER_TESTS_H

#ifdef	__cplusplus
extern "C" {
#endif
    void DEBUG_get_walls(int *left, int *front, int *right);

    void printMaze(cell maze[SIZE][SIZE]);
    void prettyPrintMaze(cell maze[SIZE][SIZE]);

#ifdef	__cplusplus
}
#endif

#endif /*PATHFINDER_TESTS_H*/