#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

typedef struct Entry {
	int index;
	int q;
	int parent;
} Entry;

typedef struct List {
	Entry* val;
	struct List* next;
	struct List* prev;
} List;

List* createList();

Entry* createEntry();

List* addList(List * l, Entry* elem);

List* removeList(List * l, int index);

bool isEmptyList(List* l);
