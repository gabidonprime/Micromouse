#include "Tuple.hpp"
#include <Arduino.h>
// #include <string>
// #include <iostream>

#include "Graph.hpp"
#include "LinkedList.hpp"

namespace mtrn3100 {

const int NORTH = 0;
const int EAST = 1;
const int SOUTH = 2;
const int WEST = 3;

const int MAZE_CELLS_WIDTH = 9;
const int MAZE_CELLS_HEIGHT = 5;


/**
 * Check if a path exists between two nodes in a graph.
 *
 * @param g The graph to search for the path.
 * @param src The source node to start the search from.
 * @param dst The destination node to search for.
 * @return True if a path exists between the source and destination nodes, false otherwise.
 */
template <typename N, typename E>
bool path_exists(Graph<N, E> const& g, N const& src, N const& dst) {
    LinkedList<N> queue;
    queue.push_front(src);

    while (!queue.empty()) {
        auto node = queue.pop_front();

        if (node == dst) {
            return true;
        }
        queue = g.nodes(node);
    }
    return false;
}

/**
 * Find the shortest path between two nodes in a graph using breadth-first search.
 *
 * @param g The graph to search for the path.
 * @param src The source node to start the search from.
 * @param dst The destination node to search for.
 * @return A linked list containing the nodes in the shortest path from the source to the destination,
 *         or an empty list if no path exists.
 */
template <typename N, typename E>
LinkedList<N> bfs_single(Graph<N, E> const& g, N const& src, N const& dst) {
    LinkedList<LinkedList<N>> queue;
    queue.push_front({src});

    while (!queue.empty()) {
        auto path = queue.pop_front();
        auto node = path.back();

        if (node == dst) {
            return path;
        }

        auto neighbours = g.nodes(node);
        while (!neighbours.empty()) {
            auto neighbour = neighbours.pop_front();
            if (path.contains(neighbour)) {
                continue;
            }
            auto new_path = path;
            new_path.push_back(neighbour);
            queue.push_back(new_path);
        }
    }
    

    return {};
}

/**
 * Find all shortest paths between two nodes in a graph using breadth-first search.
 *
 * @param g The graph to search for the paths.
 * @param src The source node to start the search from.
 * @param dst The destination node to search for.
 * @return A linked list of linked lists, where each inner list contains the nodes in a shortest path
 *         from the source to the destination. If no path exists, the list will be empty.
 */
template <typename N, typename E>
LinkedList<LinkedList<N>> bfs_multiple(Graph<N, E> const& g, N const& src, N const& dst) {
    LinkedList<LinkedList<N>> paths;
    LinkedList<LinkedList<N>> queue;

    size_t shortestPath = 0;

    queue.push_front({src});

     while (!queue.empty()) {
        auto path = queue.pop_front();
        auto node = path.back();

        if (node == dst) {
            if (shortestPath == 0) {
                shortestPath = path.size();
            } else if (path.size() > shortestPath) {
                break;
            }
            
            paths.push_back(path);
        }

        auto neighbours = g.nodes(node);
        while (!neighbours.empty()) {
            auto neighbour = neighbours.pop_front();
            if (path.contains(neighbour)) {
                continue;
            }
            auto new_path = path;
            new_path.push_back(neighbour);
            queue.push_back(new_path);
        }
    }
    
    

    return paths;
}

//TODO: Fix this function
int get_node_index(mtrn3100::Tuple<int, int> const& node) {
    return mtrn3100::get<0>(node) + mtrn3100::get<1>(node) * MAZE_CELLS_WIDTH + 1;
} 

//TODO: Implement this function
int get_relative_heading(int currentNode, int nextNode) {
    int currentX = currentNode % MAZE_CELLS_WIDTH - 1;
    int currentY = currentNode / MAZE_CELLS_WIDTH;
    int nextX = nextNode % MAZE_CELLS_WIDTH - 1;
    int nextY = nextNode / MAZE_CELLS_WIDTH;
    // std::cout << "currentNode: " << currentNode << std::endl;
    // std::cout << "nextNode: " << nextNode << std::endl;
    // std::cout << "currentX: " << currentX << std::endl;
    // std::cout << "currentY: " << currentY << std::endl;
    // std::cout << "nextX: " << nextX << std::endl;
    // std::cout << "nextY: " << nextY << std::endl;


    if (currentX == nextX) {
        if (currentY < nextY) {
            return NORTH;
        } else {
            return SOUTH;
        }
    } else {
        if (currentX < nextX) {
            return WEST;
        } else {
            return EAST;
        }
    }
}

String solve_maze(Graph<int, int> const& g, mtrn3100::Tuple<int, int> const& start, mtrn3100::Tuple<int, int> const& end, int initial_heading) {
    auto src = get_node_index(start);
    auto dst = get_node_index(end);

    auto path = bfs_single(g, src, dst);
    String result = "";
    auto current_pos = src;
    auto current_heading = initial_heading;

    path.pop_front(); // Remove the first node, which is the starting position
    while (!path.empty()) {
        auto next_pos = path.pop_front();
        auto heading_to_next = get_relative_heading(current_pos, next_pos);

        if (heading_to_next != current_heading) {
            if (heading_to_next == (current_heading + 1) % 4) {
                result += "R";
            } else {
                result += "L";
            }

            current_heading = heading_to_next;
        }

        result += "F";
        current_pos = next_pos;
    }

    return result;
}

}// namespace mtrn3100
