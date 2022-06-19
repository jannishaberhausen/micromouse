#ifndef MAZE_H
#define MAZE_H

// int_max on 16bit
#define INF 65535

/** struct of a node. A node is a unique position in the maze.
 *  Attr:
 *      id (int): unique identifier of a node
 *      parent (Node*): pointer to the parent node
 *      north, east, south, west (Node*): pointer to a neighboring node
 */ 
typedef struct Node Node;
struct Node {
    int id;
    Node *parent;
    Node *north;
    Node *east;
    Node *south;
    Node *west;
};

#endif // MAZE_H
