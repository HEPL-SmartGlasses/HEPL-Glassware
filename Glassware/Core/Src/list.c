List* createList(){
    List* list = (List*) malloc(sizeof(List));

    List->val = NULL;
    List->next = NULL;
    List->prev = NULL;

    return list;
}

// List destroyer

Entry* createEntry(int index, int q, int parent){
	Entry* entry = (Entry *) malloc(sizeof(Entry));

	entry->index = index;
	entry->q = q;
	entry->parent = parent;

	return entry;
}

// entry destroyer

List* addList(List * l, Entry* elem){
	// if it is empty
	if (isEmptyList(l->val)){
		l->val = elem;
		return;
	}

	// otherwise traverse
	List* prev = l;
	List*next = l->next;

	while(!isEmptyList(next)){
		List* prev = next;
		next = next->next;
	}

	List * newList = malloc(sizeof(List));
	newList->next = NULL;
	newList->prev = prev;
	newList->val = elem;

	prev->next = newList;

	return newList;
}

List* removeList(List * l, int index){
	// if it is empty
	if (isEmptyList(l->val)){
		return NULL;
	}

	// else index = 0
	if (index == 0) {
		List * temp = l;
		l = l->next;

		free(temp);
		return NULL;
	}

	// otherwise traverse
	List* prev = l;
	List*next = l->next;

	for(int i = 0; i < index - 1; i++){
		if (!isEmptyList(next)) {
			List* prev = next;
			next = next->next;
		} else {
			return NULL;
		}
	}

	prev->next = next->next;
	next->prev = prev;
	// free memory
	free(next);

	if (next->next != NULL){
		return NULL;
	} else {
		return prev;
	}
}

bool isEmptyList(List* l){
	if (l == NULL){
		return true;
	} else {
		return false;
	}
}
