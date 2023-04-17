#include "graph.h"
#include "list.h"

Entry findLeastF(List *l);

List* getSucc(Entry q, Graph* graph);

double distance(int idx1, int idx2, Graph * graph);

bool findFList(List * open, int idx, double f);

double heading(Graph* graph, int * path);

int * flipList2array(List * path);

int* backtrack(List * closed);

int* findShortestPath(Graph * graph, int startIdx, int destinationIdx);
