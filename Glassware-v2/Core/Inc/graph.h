#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <float.h>

#define MAX_NODES 100
#define MAX_EDGES 100

typedef struct Node {
    double x;
    double y;
} Node;

typedef struct Edge {
    int elemL;
    int elemR;
} Edge;

typedef struct Graph {
    Node* nodes[MAX_NODES];
    Edge* edges[MAX_EDGES];
    int numNodes;
    int numEdges;
} Graph;

Graph* createGraph();

bool nodeExists(Graph* graph, double x, double y);

bool edgeExists(Graph* graph, int elemL, int elemR);

void addNode(Graph* graph, double x, double y);

void addEdge(Graph* graph, int elemL, int elemR);

int findNode(Graph* graph, double x, double y);

void buildGraphFromMap(Graph* graph, double ** map, int map_size);
