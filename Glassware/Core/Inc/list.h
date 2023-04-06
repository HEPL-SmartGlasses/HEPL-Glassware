typedef struct Entry {
	int index;
	int q;
	int parent;
} Entry;

typedef struct List {
	Entry* val;
	List* next;
	List* prev;
} List;

List* createList();

Entry* createEntry();

void addList(List * l, Entry elem);

void removeList(List * l, int index);

bool isEmptyList(List* l);
