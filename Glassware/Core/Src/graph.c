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

bool nodeExists(Graph* graph, int x, int y) {
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

void addNode(Graph* graph, int x, int y) {
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
    if (nodeExists(graph, x, y)) {
        // printf("Node already exists in graph.\n");
        return;
    }

    Edge* newEdge = (Edge*) malloc(sizeof(Edge));
    newEdge->elemL = elemL;
    newEdge->elemR = elemR;

    graph->edges[graph->numEdges] = newEdge;
    graph->numEdges++;
}

int findNode(Graph* graph, int x, int y) {
    if (!nodeExists(graph, x, y)) {
        // printf("Node already exists in graph.\n");
        return -1;
    }

    for (int i = 0; i < graph->numNodes; i++) {
        if (graph->nodes[i]->x == x && graph->nodes[i]->y == y) {
            return i;
        }
    }
}

void buildGraphFromMap(Graph* graph, uint16_t ** map, size_t map_size){

	for (int i = 0; i < map_size; i++){
		 int x1 = map[i][1];
		 int y1 = map[i][2];
		 int x2 = map[i][3];
		 int y2 = map[i][4];

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
