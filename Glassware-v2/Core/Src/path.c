Entry* findLeastF(List *l){
	int f = INT_MAX;
	int i = 0;
	int idx;
	Entry* q;

	List* l_copy = l;

	while(l_copy != NULL) { // traverse through list
		int f_new = l_copy->val->q;
		if (f_new < f) {
			f = f_new;
			q = l_copy->val;
			idx = i;
		}
		l_copy = l_copy->next;
		i++;
	}

	// remove entry from list
	removeList(l, idx);

	return q;
}

List* getSucc(Entry* q, Graph* graph){
// return list of successors to q
		List * succ = createList();
		for (int i = 0; i < graph->numEdges; i++){
			int idx1 = graph->edges[1]->elemL, idx2 = graph->edges[1]->elemR;

			if (q->index == idx1){
				addList(succ, createEntry(idx2, q->index, 0));
			} else if  (q->index == idx2){
				addList(succ, createEntry(idx1, q->index, 0));
			}
		}
}

double distance(int idx1, int idx2, Graph * graph){
// compute Euclidean distance between point in graph
     int x1 = graph->nodes[idx1]->x;
     int y1 = graph->nodes[idx1]->y;

     int x2 = graph->nodes[idx2]->x;
     int y2 = graph->nodes[idx2]->y;

     double d = sqrt( pow(x1 - x2, 2) + pow(y1 - y2, 2));

     return d;
}

bool findFList(List * open, int idx, double f){
// find if open list has any entry with lower f
    List * l_copy = open;
    while(l_copy != NULL){
    	if (l_copy->val->index == idx && l_copy->val->q < f) {
    		return true;
    	}
    	l_copy = l_copy->next;
    }

    return false;
}

Entry * getLastElem(List * l){
	List * l_copy = l;
	while (l_copy != NULL){
		List * next = l_copy->next;

		if (next == NULL){
			return l_copy->val;
		} else {
			l_copy = next;
		}
	}

	return NULL;
}

int getListSize(List * l){
	int count = 0;

	while (l != NULL){
		l = l->next;
		count++;
	}

	return count;
}

int * flipList2array(List * path, List * pathEnd){
	int size = getListSze(path);

	int* arr = malloc(size * sizeof(int));

	int i = 0;
	while (pathEnd != NULL){
		arr[i] = pathEnd->val->index;
		pathEnd = pathEnd->prev;
		i++;
	}

	return arr;
}

int*  backtrack(List * closed, List* closedEnd){
    List * path = createList();
    List * pathEnd = path;

    List * end_copy = closedEnd;
    Entry * dest = getLastElem(closed);

    pathEnd = addList(path, dest);
    int parent = dest->parent;

    while (end_copy != NULL) {
    	Entry* entry = end_copy->val;
    	int nodeIdx = entry->index;

    	if (parent == nodeIdx) {
    		pathEnd = addList(path, createNetry(nodeIdx, 0, 0));
    		parent = entry->parent;
    	} else {
    		continue;
    	}
    }

    return flipList2array(path, pathEnd);
}

int* findShortestPath(Graph * graph, int startIdx, int destinationIdx){
// A* Search Algorithm

//	1.  Initialize the open list
	List * open;

//	2.  Initialize the closed list
//	    put the starting node on the open
//	    list (you can leave its f at zero)
	List * closed;
	Entry* entry = createEntry(startIdx, 0, 0);
	addList(closed, entry);

//	3.  while the open list is not empty
	while (isEmptyList(open)) {
	//	    a) find the node with the least f on
	//	       the open list, call it "q"
	//	    b) pop q off the open list
			Entry* q = findLeastF(open);
	//
	//	    c) generate q's 8 successors and set their
	//	       parents to q
			List * succ = getSucc(q, graph);
	//	    d) for each successor
			List * succ_copy = succ;
			while (succ_copy != NULL){

				if (succ_copy->val->index == destinationIdx){
					// if successor is the goal, stop search
					addList(closed, q);
					addList(closed, createEntry(succ_copy->val->index, q->q, q->index))
					return backtrack(closed);
				} else {
					// else, compute both g and h for successor
					double g = q->q + distance(q->index, succ_copy->val->index, graph);
					double h = distance(destinationIdx, succ_copy->val->index, graph);
					double f = g + h;

					succ_copy->val->g = f;

					// iii) if a node with the same position as
					// successor is in the OPEN list which has a
					// lower f than successor, skip this successor

		            if(findFList(open, succIdx, f)){
		            	continue;
		            }
					// iV) if a node with the same position as
					// successor  is in the CLOSED list which has
					// a lower f than successor, skip this successor
					// otherwise, add  the node to the open list
		            if(findFList(closed, succIdx, f)){
		            	continue;
		            }
		            addList(open, createEntry(succ_copy->val->index, f, q->index));

				}
				// push q on the closed list
				addList(closed, q);
				// iterate
				succ_copy = succ_copy->next;
			}
			return backtrack(closed);
	}
}
