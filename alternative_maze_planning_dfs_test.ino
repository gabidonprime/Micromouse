
#include "Stack.hpp"
#include "Tuple.hpp"
#include "Graph.hpp"
#include "graph2ascii.hpp"
#include "ascii2graph.hpp"

// Assumptions:
// 1. Starting at cell 1 facing south
// 2. Cells are ordered as in previous milestons

// Alternate maze mapping variables.


// Setting up pose and cardinal directions
enum CardinalDirections {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3,
};
int* row = 0;
int* col = 0;
int head = SOUTH;

// Initialise maze with nodes.
mtrn3100::Graph<int, bool> maze(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 
21, 22, 23, 24, 25, 26, 27, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45);


// Global variables
int cellsConnected = 0;
int frontConnected = false;
int leftConnected = false;
int rightConnected = false;
int currentCell = 0;
int unvisitedCells[45];
Stack<int> junctions;

// Test maze
char testMazeString[] = 
" --- --- --- --- --- --- --- --- --- \n"
"|               |       |       |   |\n"
" --- ---     ---                     \n"
"|           |       |           |   |\n"
"         ---     --- ---     ---     \n"
"|   |               |               |\n"
" --- ---     --- ---                 \n"
"|           |           |       |   |\n"
"     --- ---             --- ---     \n"
"|               |   |   |           |\n"
" --- --- --- --- --- --- --- --- --- \0";
 
mtrn3100::Graph<int, bool> testMaze = mtrn3100::ascii2graph(testMazeString);

// Mains
void setup() {
    Serial.begin(115200);
    delay(100);

    // Initialise unvisited cells array
    for (int i = 0; i < 45; i++) {
        unvisitedCells[i] = i + 1;
    }
}

void loop() {
    // Add data of current cell to graph
    mapCell();

    // If multiple cells connected, explore junction
    if (cellsConnected > 1) {
        junctions.push(currentCell);
        exploreJunction();
    } else { // Otherwise, continue
        wallFollowStep();
    }

    // If all cells visited and junctions stack is empty
    if (sizeof(unvisitedCells) == 0) {
        // Send maze data to bot
        sendMazeData();

        // Finish
        while(1) {
            // Do nothing.
        }
    }
}

// Helper functions
void exploreJunction() {
    if (rightConnected) {
            turnRight();
            moveForward();
            mapCell();
            while (isInArray(currentCell, unvisitedCells)) { // Continue until reached a visited cell
                if (cellsConnected > 1) {
                    junctions.push(currentCell);
                    // Nested explore junctions allows for backtracking
                    exploreJunction();
                } else {
                    wallFollowStep();
                    mapCell();
                }
                wallFollowStep();
                mapCell();
            }
            goToCell(junctions.top(), row, col);
        }
        if (frontConnected) {
            moveForward();
            mapCell();
            while (isInArray(currentCell, unvisitedCells)) {
                if (cellsConnected > 1) {
                    junctions.push(currentCell);
                    exploreJunction();
                } else {
                    wallFollowStep();
                    mapCell();
                }
            }
            goToCell(junctions.top(), row, col);
        } 
        if (leftConnected) {
            turnLeft();
            moveForward();
            mapCell();
            while (isInArray(currentCell, unvisitedCells)) {
                if (cellsConnected > 1) {
                    junctions.push(currentCell);
                    exploreJunction();
                } else {
                    wallFollowStep();
                    mapCell();
                }
            }
            goToCell(junctions.top(), row, col);
        }

        // Once junction is explored, pop  
        junctions.pop();
        if (junctions.isEmpty()) {
            return;
        } else {
            goToCell(junctions.top(), row, col);
        }
}

void mapCell() {
    // Determine which cells are connected by index
    currentCell = pos2index(row, col);
    int leftCell = 0;
    int rightCell = 0;
    int frontCell = 0;

    if (head == NORTH) {
        frontCell = currentCell - 8;
        leftCell = currentCell - 1;
        rightCell = currentCell + 1;
    } else if (head == EAST) {
        frontCell = currentCell + 1;
        leftCell = currentCell - 8;
        rightCell = currentCell + 8;
    } else if (head == SOUTH) {
        frontCell = currentCell + 8;
        leftCell = currentCell + 1;
        rightCell = currentCell - 1;
    } else {
        frontCell = currentCell - 1;
        leftCell = currentCell + 8;
        rightCell = currentCell - 8;
    }

    // Detect walls
    if (testMaze.is_connected(currentCell, leftCell)) {
        leftConnected = true;
    } 
    if (testMaze.is_connected(currentCell, rightCell)) {
        rightConnected = true;
    } 
    if (testMaze.is_connected(currentCell, frontCell)) {
        frontConnected = true;
    }

    // Remove current cell from list of unvisited cells
    removeElementFromArray(unvisitedCells, currentCell);

    // Add data from cell to graph
    if (frontConnected) {
        insertEdge(maze, currentCell, frontCell);
        cellsConnected++;
    }
    if (leftConnected) {
        insertEdge(maze, currentCell, leftCell);
        cellsConnected++;
    }
    if (rightConnected) {
        insertEdge(maze, currentCell, rightCell);
        cellsConnected++;
    }
}

void wallFollowStep() {
    if (rightConnected){
        turnRight();
        moveForward();
    } else if (frontConnected) {
        moveForward();
    } else {
        turnLeft();
    }
}

void moveForward() {

    // Update position
    switch(head) {
        case NORTH:
            row--;
            break;
        case EAST:
            col++;
            break;
        case SOUTH:
            row++;
            break;
        case WEST:
            col--;
            break;
    }
}

void turnLeft() {
    // Update heading
    head -= 1;
    if (head < 0) {
        head = 3;
    }
}

void turnRight() {
    // Update heading
    head += 1;
    if (head > 3) {
        head = 0;
    }
}

// Convert node positions to node indexes
int pos2index(int* row, int* col) {
    return (*row * 9 + *col + 1);
}

int* index2pos(int index) {
    int rowCol[2];
    rowCol[0] = floor((index - 1) / 5);
    rowCol[1] = (index - 1) % 9;
    return rowCol;
}

// Remove element from an array
void removeElementFromArray(const int* originalArray, int elementToRemove) {
    int size = sizeof(originalArray);
    int newArray[size];
    int newArrayIndex = 0;

    for (int i = 0; i < size; ++i) {
        if (originalArray[i] != elementToRemove) {
            newArray[newArrayIndex] = originalArray[i];
            ++newArrayIndex;
        }
    }

    while (newArrayIndex < size) {
        newArray[newArrayIndex] = 0;
        ++newArrayIndex;
    }

    originalArray = newArray;
}

bool isInArray(int number, int arr[]) {
    for (int i = 0; i < sizeof(arr); ++i) {
        if (arr[i] == number) {
            return true;  // Number is found in the array
        }
    }
    return false;  // Number is not found in the array
}

// Insert edges into maze
void insertEdge(mtrn3100::Graph<int, bool>& maze, int cell1, int cell2) {
    maze.insert_edge(cell1, cell2, 0);
    maze.insert_edge(cell2, cell1, 0);
}

void goToCell(int cell, int* row, int* col) {
    *row = index2pos(cell)[0];
    *col = index2pos(cell)[1];
}

void sendMazeData() {
    // Harry to complete
}
