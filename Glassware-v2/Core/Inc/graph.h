#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#define MAX_NODES 100
#define MAX_EDGES 100

typedef struct Node {
    int x;
    int y;
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

bool nodeExists(Graph* graph, int x, int y);

bool edgeExists(Graph* graph, int elemL, int elemR);

void addNode(Graph* graph, int x, int y);

void addEdge(Graph* graph, int elemL, int elemR);

int findNode(Graph* graph, int x, int y);

void buildGraphFromMap(Graph* graph, uint16_t ** map, size_t map_size);
