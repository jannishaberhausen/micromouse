#ifndef CODE_PATHFINDER_H
#define CODE_PATHFINDER_H


// Size of the maze
#define SIZE 4

/**
 * internal representation of the state of a wall
 */
enum wall {
    UNKNOWN=-1, WALL=0, WAY=1, EXPLORED=2, ENTRY=3
};

typedef enum wall wall;


/**
 * Information stored for every cell of the grid
 */
struct cell {
    // -1 if the corresponding side is not yet explored,
    // 0 if it is not drivable,
    // 1 if it is drivable,
    // 2 if it is drivable and explored,
    // 3 if we entered through this wall.
    wall walls[4];

    // Further info, used by the pathfinder
    int flag;
};

typedef struct cell cell;


/**
 * Possible orientations of the mouse wrt. the maze
 */
enum orientation {
    NORTH=0, EAST=1, SOUTH=2, WEST=3
};
typedef enum orientation orientation;

/**
 * Possible orientations of the mouse wrt. the mouse
 */
enum direction {
    FRONT=0, RIGHT=1, BACK=2, LEFT=3, STOP=4
};
typedef enum direction direction;


enum plannerState {
    WAIT_EXPLORE, EXPLORE, WAIT_EXPLOIT, EXPLOIT
};
typedef enum plannerState plannerState;

extern plannerState current_state_planner;

void initMaze(unsigned int x, unsigned int y, orientation dir);
direction explore(unsigned int x, unsigned int y, orientation dir);
direction* exploit(unsigned int x, unsigned int y, orientation dir,
                   unsigned int x_dest, unsigned int y_dest);

void plannerFSM();

cell** DEBUG_get_maze();

#endif //CODE_PATHFINDER_H
