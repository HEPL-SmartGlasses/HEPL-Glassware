#include "list.h"

List* createList(){
    List* list = (List*) malloc(sizeof(List));

    list->head = NULL;
    list->tail = NULL;

    return list;
}

// List destroyer

Entry* createEntry(int index, double f, int parent){
	Entry* entry = (Entry *) malloc(sizeof(Entry));

	entry->index = index;
	entry->f = f;
	entry->parent = parent;

	return entry;
}

// entry destroyer

void addList(List * list, Entry* elem){
    NodeList* new_node = (NodeList*) malloc(sizeof(NodeList));
    new_node->val = elem;
    new_node->next = NULL;

    if (list->head == NULL) {
        // The list is empty, so make the new node both the head and tail.
        new_node->prev = NULL;
        list->head = new_node;
        list->tail = new_node;
    } else {
        // Add the new node to the end of the list.
        new_node->prev = list->tail;
        list->tail->next = new_node;
        list->tail = new_node;
    }
    return;
}

void removeList(List * list, int index){
	// if it is empty
	if (list->head == NULL || index < 0){
		return;
	}

	if (index == 0) {
		NodeList * old_head = list->head;
		list->head = old_head->next;
		if (list->head != NULL){
			list->head->prev = NULL;
		} else {
			list->tail = NULL;
		}

		free(old_head->val);
		free(old_head);

		return;
	}

	// otherwise traverse
	NodeList * current = list->head;
	int i = 0;
	while (i < index && current != NULL){
		current = current->next;
		i++;
	}

    // If the index is out of bounds, return without doing anything.
    if (current == NULL) {
        return;
    }

	current->prev->next = current->next;
	if (current->next != NULL){
		current->next->prev = current->prev;
	} else {
		list->tail = current->prev;
	}

	// free memory
	free(current->val);
	free(current);

	return;
}

bool isEmptyList(List* list){
	return list->head == NULL;
}

int getListSize(List * list)
{
	int count = 0;
	NodeList* current = list->head;

	while (current != NULL)
	{
		current = current->next;
		count++;
	}

	return count;
}

Entry * getLastElem(List * list)
{
	if (list->tail == NULL){
		return NULL;
	} else {
		return list->tail->val;
	}
}
