#include "path.h"
#include <limits.h>
#include <math.h>

Entry* findLeastF(List *l){
	int f = INT_MAX;
	int i = 0;
	int idx;
	Entry* q;

	NodeList* l_copy = l->head;

	while(l_copy != NULL) { // traverse through list
		int f_new = l_copy->val->f;
		if (f_new < f) {
			f = f_new;
			q = l_copy->val;
			idx = i;
		}
		l_copy = l_copy->next;
		i++;
	}

	// recreate q so it is not deleted
	Entry *retq = createEntry(q->index, q->f, q->parent);
	// remove entry from list
	removeList(l, idx);

	return retq;
}

List* getSucc(Entry* q, Graph* graph){
// return list of successors to q
		List * succ = createList();
		for (int i = 0; i < graph->numEdges; i++){
			int idx1 = graph->edges[i]->elemL, idx2 = graph->edges[i]->elemR;

			if (q->index == idx1){
				addList(succ, createEntry(idx2, 0, q->index));
			} else if  (q->index == idx2){
				addList(succ, createEntry(idx1, 0, q->index));
			}
		}

		return succ;
}

double distance(int idx1, int idx2, Graph * graph){
// compute Euclidean distance between point in graph
     double x1 = graph->nodes[idx1]->x;
     double y1 = graph->nodes[idx1]->y;

     double x2 = graph->nodes[idx2]->x;
     double y2 = graph->nodes[idx2]->y;

     double d = sqrt( pow(x1 - x2, 2) + pow(y1 - y2, 2));

     return d;
}

bool findFList(List * open, int idx, double f){
// find if open list has any entry with lower f
    NodeList * l_copy = open->head;
    while(l_copy != NULL){
    	if (l_copy->val->index == idx && l_copy->val->f < f) {
    		return true;
    	}
    	l_copy = l_copy->next;
    }

    return false;
}

int * flipList2array(List * path){
	int size = getListSize(path);

	int* arr = malloc(size * sizeof(int));

	NodeList* end = path->tail;

	int i = 0;
	while (end != NULL){
		arr[i] = end->val->index;
		end = end->prev;
		i++;
	}

	return arr;
}

double heading(Graph* graph, int * path){
	// TODO: edge cases for safe array access
	int cur = path[0];
	int next = path[1];

	double curX = graph->nodes[cur]->x;
	double curY = graph->nodes[cur]->y;

	double nextX = graph->nodes[next]->x;
	double nextY = graph->nodes[next]->y;

	double dot = nextX - curX;
	double norm = sqrt( pow(nextX - curX, 2) + pow(nextY - curY, 2));
	double theta = acos(dot/norm);

	if ( (nextY - curY) >= 0) {
		return theta;
	} else {
		return 2 * M_PI - theta;
	}

}

int* backtrack(List * closed){
    List * path = createList();

    NodeList * end_copy = closed->tail;
    Entry * dest = getLastElem(closed);

    addList(path, dest);
    int parent = dest->parent;

    while (end_copy != NULL) {
    	Entry* entry = end_copy->val;
    	int nodeIdx = entry->index;

    	if (parent == nodeIdx) {
    		addList(path, createEntry(nodeIdx, 0, 0));
    		parent = entry->parent;
    	} else {
    		end_copy = end_copy->prev;
    		continue;
    	}
    	end_copy = end_copy->prev;
    }

    return flipList2array(path);
}

int* findShortestPath(Graph * graph, int startIdx, int destinationIdx){
// A* Search Algorithm

//	1.  Initialize the open list
	List * open = createList();;

//	2.  Initialize the closed list
//	    put the starting node on the open
//	    list (you can leave its f at zero)
	List * closed = createList();

	Entry* entry = createEntry(startIdx, 0.0, 0);
	addList(open, entry);

//	3.  while the open list is not empty
	while (!isEmptyList(open)) {
	//	    a) find the node with the least f on
	//	       the open list, call it "q"
	//	    b) pop q off the open list
			Entry* q = findLeastF(open);
	//
	//	    c) generate q's 8 successors and set their
	//	       parents to q
			List * succ = getSucc(q, graph);
	//	    d) for each successor
			NodeList * succ_copy = succ->head;
			while (succ_copy != NULL){

				int succIdx = succ_copy->val->index;

				if (succIdx == destinationIdx){
					// if successor is the goal, stop search
					addList(closed, q);
					addList(closed, createEntry(succIdx, q->f, q->index));
					return backtrack(closed);
				} else {
					// else, compute both g and h for successor
					double g = q->f + distance(q->index, succIdx, graph);
					double h = distance(destinationIdx, succIdx, graph);
					double f = g + h;

					succ_copy->val->f = f;

					// iii) if a node with the same position as
					// successor is in the OPEN list which has a
					// lower f than successor, skip this successor

		            if(findFList(open, succIdx, f)){
		            	succ_copy = succ_copy->next;
		            	continue;
		            }
					// iV) if a node with the same position as
					// successor  is in the CLOSED list which has
					// a lower f than successor, skip this successor
					// otherwise, add  the node to the open list
		            if(findFList(closed, succIdx, f)){
		            	succ_copy = succ_copy->next;
		            	continue;
		            }
		            addList(open, createEntry(succIdx, f, q->index));

				}
				// push q on the closed list
				addList(closed, q);
				// iterate
				succ_copy = succ_copy->next;
			}
	}
	return backtrack(closed);
}
