#include <stdio.h>
#include <stdlib.h>

#include "maze.h"

void printMaze(struct Node *node)
{
    while (node != NULL) {
        printf(" %d ", node->id);
        node = node->north;
    }
}

int main()
{
    struct Node *head = NULL;
    struct Node *second = NULL;
    struct Node *third = NULL;

    head = (struct Node*)malloc(sizeof(struct Node));
    second = (struct Node*)malloc(sizeof(struct Node));
    third = (struct Node*)malloc(sizeof(struct Node));

    /* Three blocks have been allocated dynamically.
     We have pointers to these three blocks as head,
     second and third    
       head           second           third
        |                |               |
        |                |               |
    +---+-----+     +----+----+     +----+----+
    | #  | #  |     | #  | #  |     |  # |  # |
    +---+-----+     +----+----+     +----+----+
    
    # represents any random value.
    Data is random because we haven’t assigned
    anything yet  */
   
    head->id = 1;
    head->north = second;
    head->east = NULL;
    head->south = NULL;
    head->west = NULL;

    /* data has been assigned to the id part of the first
     block (block pointed by the head). And north
     pointer of first block points to second node. 
     So they both are linked.
 
       head          second         third
        |              |              |
        |              |              |
    +---+---+     +----+----+     +-----+----+
    | 1  | o----->| #  | #  |     |  #  | #  |
    +---+---+     +----+----+     +-----+----+   
    */

    second->id = 2;
    second->north = NULL;
    second->east = third;
    second->south = NULL;
    second->west = NULL;

    printMaze(head);
    
    return 0;
}