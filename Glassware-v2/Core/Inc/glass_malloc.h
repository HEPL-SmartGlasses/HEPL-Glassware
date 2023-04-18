

int num_allocated = 0;
int num_freed = 0;

void* glass_malloc(int size)
{
	num_allocated++;
	return malloc(size);
}

void glass_free(void *ptr)
{
	num_freed++;
	free(ptr);
}
