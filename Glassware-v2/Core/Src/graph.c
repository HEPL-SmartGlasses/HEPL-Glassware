#include "graph.h"

Graph* createGraph() {
    Graph* graph = (Graph*) malloc(sizeof(Graph));
    graph->numNodes = 0;
    graph->numEdges = 0;

    for (int i = 0; i < MAX_NODES; i++) {
        graph->nodes[i] = NULL;
    }

    for (int i = 0; i < MAX_EDGES; i++) {
        graph->edges[i] = NULL;
    }
    return graph;
}

bool nodeExists(Graph* graph, double x, double y) {
    for (int i = 0; i < graph->numNodes; i++) {
        if (graph->nodes[i]->x == x && graph->nodes[i]->y == y) {
            return true;
        }
    }
    return false;
}

bool edgeExists(Graph* graph, int elemL, int elemR) {
    for (int i = 0; i < graph->numEdges; i++) {
        if (graph->edges[i]->elemL == elemL && graph->edges[i]->elemR == elemR) {
            return true;
        }
    }
    return false;
}

void addNode(Graph* graph, double x, double y) {
    if (nodeExists(graph, x, y)) {
        // printf("Node already exists in graph.\n");
        return;
    }

    Node* newNode = (Node*) malloc(sizeof(Node));
    newNode->x = x;
    newNode->y = y;

    graph->nodes[graph->numNodes] = newNode;
    graph->numNodes++;
}

void addEdge(Graph* graph, int elemL, int elemR) {
    if (edgeExists(graph, elemL, elemR)) {
        // printf("Node already exists in graph.\n");
        return;
    }

    Edge* newEdge = (Edge*) malloc(sizeof(Edge));
    newEdge->elemL = elemL;
    newEdge->elemR = elemR;

    graph->edges[graph->numEdges] = newEdge;
    graph->numEdges++;
}

int findNode(Graph* graph, double x, double y) {
    if (!nodeExists(graph, x, y)) {
        // printf("Node already exists in graph.\n");
        return -1;
    }

    for (int i = 0; i < graph->numNodes; i++) {
        if (graph->nodes[i]->x == x && graph->nodes[i]->y == y) {
            return i;
        }
    }

    return -1;
}

void buildGraphFromMap(Graph* graph, double ** map, int map_size){

	for (int i = 0; i < map_size; i++){
		 double x1 = map[i][1];
		 double y1 = map[i][2];
		 double x2 = map[i][3];
		 double y2 = map[i][4];

		 // add nodes
		 addNode(graph, x1, y1);
		 addNode(graph, x2, y2);

		 // add edges
		 int idx1 = findNode(graph, x1, y1);
		 int idx2 = findNode(graph, x2, y2);

		 addEdge(graph, idx1, idx2);
	}

	return;
}
