#include "graph.h"
#include "list.h"

Entry* findLeastF(List *l);

List* getSucc(Entry* q, Graph* graph);

double distance(int idx1, int idx2, Graph * graph);

bool findFList(List * open, int idx, double f);

Entry * getLastElem(List * l);

int getListSize(List * l);

int * flipList2array(List * path, List * pathEnd);

int* backtrack(List * closed, List* closedEnd);

int* findShortestPath(Graph * graph, int startIdx, int destinationIdx);
