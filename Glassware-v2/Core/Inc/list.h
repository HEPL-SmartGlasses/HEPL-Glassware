#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

typedef struct Entry {
	int index;
	double f;
	int parent;
} Entry;

typedef struct NodeList {
	Entry* val;
	struct NodeList* next;
	struct NodeList* prev;
} NodeList;

typedef struct List {
	NodeList *head;
	NodeList *tail;
} List;

List* createList();

Entry* createEntry();

void addList(List * l, Entry* elem);

void removeList(List * l, int index);

bool isEmptyList(List* l);

int getListSize(List * l);

Entry * getLastElem(List * list);
