/**
 * File: trailblazer.cpp
 * --------------
 * Elizabeth Culbertson
 * Section Leader: Rasmus Rygaard
 *
 * This file implements various pathfinding algorithms.
 * Note: I implemented one extension (disjoint set forest)
 * which simply replaced my ADT solution for the maze
 * generator (so, no need to look anywhere else for it)
 *
 */

#include "costs.h"
#include "trailblazer.h"
#include "pqueue.h"
#include "queue.h"
#include "set.h"
#include "vector.h"

using namespace std;

//Prototypes
bool depthFirstSearchRec(BasicGraph &graph, Vertex *curr, Vertex *end, Vector<Vertex *> &path);
Vector<Vertex*> makePath(Vertex *vertex);
Vector<Vertex*> dijkstraPlusAStar(BasicGraph &graph, Vertex *start, Vertex *end, bool useAStar);
void processEdge(Edge *edge, Vertex *curr, PriorityQueue<Vertex *> &searchQueue, Vertex *end, bool useAStar);
double priority(Vertex *vertex, Vertex *end, bool useAStar);
Vertex* makeSet(Vertex *vert);
Vertex* findSet(Vertex *vert);
void unionSet(Vertex *first, Vertex *second);

/* Conducts a depth first search based on a graph representing a map or terrain.
 * Wrapper function.
 *
 * graph: the graph that encodes the maze or terrain
 * start: the Vertex* that points to the starting location on the graph
 * end: the desired end location on the graph
 *
 * return: a path from the start to end vertex
 */
Vector<Vertex*> depthFirstSearch(BasicGraph& graph, Vertex* start, Vertex* end) {
    graph.resetData();
    Vector<Vertex*> path;
    depthFirstSearchRec(graph, start, end, path);
    return path;
}

/* Conducts a depth first search based on a graph representing a map or terrain.
 * Recursive function.
 *
 * graph: the graph that encodes the maze or terrain
 * start: the Vertex* that points to the starting location on the graph
 * end: the desired end location on the graph
 * path: the list of vertices between the start and end that encodes the path
 *
 * return: a path from the start to end vertex
 */
bool depthFirstSearchRec(BasicGraph &graph, Vertex* curr, Vertex* end, Vector<Vertex*> &path) {
    if (curr == end) {
        curr->setColor(GREEN);
        path.add(curr);
        return true;
    }
    if (curr->getColor() == GRAY || curr->getColor() == GREEN) return false;
    path.add(curr);
    curr->setColor(GREEN);
    for (Edge* edge : curr->edges) {
        Vertex* next = ((edge->start == curr) ? edge->finish:edge->start);
        if (depthFirstSearchRec(graph, next, end, path)) return true;
    }
    path.remove(path.size()-1);
    curr->setColor(GRAY);
    return false;
}

/* Conducts a breadth first search based on a graph representing a map or terrain.
 *
 * graph: the graph that encodes the maze or terrain
 * start: the Vertex* that points to the starting location on the graph
 * end: the desired end location on the graph
 *
 * return: a path from the start to end vertex
 */
Vector<Vertex*> breadthFirstSearch(BasicGraph& graph, Vertex* start, Vertex* end) {
    graph.resetData();
    Queue<Vertex*> searchQueue;
    searchQueue.enqueue(start);
    start->setColor(YELLOW);
    while (!searchQueue.isEmpty()) {
        Vertex* curr = searchQueue.dequeue();
        curr->setColor(GREEN);
        if (curr == end) break;
        for (Edge* edge : curr->edges) {
            Vertex* next = edge->finish;
            if (next->getColor() != GREEN && next->getColor() != YELLOW) {
                next->previous = curr;
                next->setColor(YELLOW);
                searchQueue.enqueue(next);
            }
        }
    }
    return makePath(end);
}

/*Given an end vertex, constructs a path back to the start by following
 * parent pointers and placing them in a vector in reverse order
 *
 * vertex: the end of the path
 *
 * return: a path from the start to end vertex
 */
Vector<Vertex*> makePath(Vertex* vertex) {
    Vector<Vertex*> result;
    Stack<Vertex*> temp;
    while (vertex->previous != NULL) {
        temp.push(vertex);
        vertex = vertex->previous;
    }
    temp.push(vertex);
    while (!temp.isEmpty()) {
        result.add(temp.pop());
    }
    return result;
}

/* Conducts a breadth first search based on a graph representing a map or terrain
 * using Dijkstra's Algorithm.
 *
 * graph: the graph that encodes the maze or terrain
 * start: the Vertex* that points to the starting location on the graph
 * end: the desired end location on the graph
 *
 * return: a path from the start to end vertex
 */
Vector<Vertex*> dijkstrasAlgorithm(BasicGraph& graph, Vertex* start, Vertex* end) {
    return dijkstraPlusAStar(graph, start, end, false);
}

/* Conducts a breadth first search based on a graph representing a map or terrain
 * using the A* Algorithm.
 *
 * graph: the graph that encodes the maze or terrain
 * start: the Vertex* that points to the starting location on the graph
 * end: the desired end location on the graph
 *
 * return: a path from the start to end vertex
 */
Vector<Vertex*> aStar(BasicGraph& graph, Vertex* start, Vertex* end) {
    return dijkstraPlusAStar(graph, start, end, true);
}

/* Conducts a breadth first search based on a graph representing a map or terrain
 * using Dijkstra's Algorithm or the A* Algorithm depending on the value passed
 * for useAStar.
 *
 * useAStar will be passed to priority() within the call to determine whether the
 * algorithm used is Dijkstra (priority doesn't take into account heuristic distance)
 * or A* (priority does take into account heuristic distance)
 *
 * graph: the graph that encodes the maze or terrain
 * start: the Vertex* that points to the starting location on the graph
 * end: the desired end location on the graph
 * useAStar: indicates which algorith is being implemented
 *
 * return: a path from the start to end vertex
 */
Vector<Vertex*> dijkstraPlusAStar(BasicGraph& graph, Vertex* start, Vertex* end, bool useAStar) {
    graph.resetData();
    PriorityQueue<Vertex*> searchQueue;
    start->cost = 0;
    start->setColor(YELLOW);
    searchQueue.enqueue(start, priority(start, end, useAStar));
    while (!searchQueue.isEmpty()) {
        Vertex* curr = searchQueue.dequeue();
        curr->setColor(GREEN);
        if (curr == end) break;
        for (Edge* edge : curr->edges) {
            processEdge(edge, curr, searchQueue, end, useAStar);
        }
    }
    return makePath(end);
}

/* Processes an edge coming from a vertex in accordance with Dijkstra's and the A*
 * algorithm.
 *
 * edge: the edge being processed
 * curr: the current "green" node in the algorithm
 * searchQueue: the queue of "yellow" nodes
 * end: the desired end point
 * useAStar: indicates which algorithm is being implemented
 */
void processEdge(Edge* edge, Vertex *curr, PriorityQueue<Vertex*>& searchQueue, Vertex* end, bool useAStar) {
    Vertex* next = edge->finish;
    double addedCost = curr->cost + edge->cost;
    if (next->getColor() == UNCOLORED) {
        next->cost = addedCost;
        next->previous = curr;
        searchQueue.enqueue(next, priority(next, end, useAStar));
        next->setColor(YELLOW);
    } else if (next->getColor() == YELLOW && next->cost > addedCost) {
        next->cost = addedCost;
        next->previous = curr;
        searchQueue.changePriority(next, priority(next, end, useAStar));
    }
}

/* Calculates the priority of a node about to be enqueued.
 * Priority will always be at least the cost of the current vertex, and will be
 * cost plus the heuristic distance from the vertex to the end if the algorithm
 * is A*.
 *
 * vertex: the current vertex whose priority is being determined
 * end: the desired end vertex
 * useAStar: indicates which algorithm is being implemented i.e. indicates
 * whether the priority should take the heuristic distance to the end into account
 *
 * return: the priority of the vertex for enqueueing
 */
double priority(Vertex* vertex, Vertex* end, bool useAStar) {
    double result = vertex->cost;
    if (useAStar) result += heuristicFunction(vertex, end);
    return result;
}

/* Implements Kruskal's algorithm for generating a minimum spanning tree
 * and thus returns a set of edges that can be turned into a maze.
 *
 * graph: the basicGraph denoting all possible vertices and edges on the board.
 *
 * return: a set of edges representing a minimum spanning tree for the graph.
 *
 * Note that this is implemented using a disjoint set forest. Resources used to research the disjoint
 * set forest (outside of our class materials) are:
 * the wikipedia page for kruskal's algorithm (always a good start):
 *      http://en.wikipedia.org/wiki/Kruskal%27s_algorithm
 * a powerpoint from UTSA that describes the disjoint set forest and includes pseudocode:
 *      http://www.cs.utsa.edu/~bylander/cs5633/disjoint-sets.pdf
 * a powerpoint from the University of Edinburgh that also describes and includes pseudocode:
 *      http://www.inf.ed.ac.uk/teaching/courses/ads/Lects/lecture14.pdf
 * Actual translation from pseudocode to C++ was 100% original.
 */
Set<Edge*> kruskal(BasicGraph& graph) {
    Set<Edge*> result;
    int numClusters = graph.getVertexSet().size();
    for (Vertex* vert : graph.getVertexSet()) {
        makeSet(vert);
    }
    PriorityQueue<Edge*> queue;
    for (Edge* edge : graph.getEdgeSet()) {
        queue.enqueue(edge, edge->cost);
    }
    while (numClusters > 1) {
        Edge* edge = queue.dequeue();
        if (findSet(edge->start) != findSet(edge->finish)) {
            unionSet(edge->start, edge->finish);
            numClusters--;
            result.add(edge);
        }
    }
    return result;
}

/* Makes a set from a vertex in a disjoing set forest
 *
 * vert: the vertex to be placed in its own set
 *
 * return: a pointer to the representative for that new set
 */
Vertex* makeSet(Vertex* vert) {
    vert->previous = vert;
    return vert;
}

/* Finds the set a vertex belongs to by returning the
 * representative for that set. Compresses the set by
 * reassigning parent pointers to the representative
 * as the representative is found.
 *
 * vert: the vertex whose set we are trying to find
 *
 * return: the representative for the set vert is in
 */
Vertex* findSet(Vertex* vert) {
    if (vert == vert->previous) return vert;
    vert->previous = findSet(vert->previous);
    return vert->previous;
}

/*Unions two sets by setting the parent pointer of one
 * representative to the other representative.
 * Keeps track of tree heights to keep trees as low as possible.
 *
 * first: the first set of vertices in the union
 * second: the second set of vertices in the union
 */
void unionSet(Vertex* first, Vertex* second) {
    first = findSet(first);
    second = findSet(second);
    if (first->cost < second->cost) {
        first->previous = second;
        second->cost += first->cost;
    } else {
        second->previous = first;
        first->cost += second->cost;
    }
}
