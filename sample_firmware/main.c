unsigned int a = 5;
unsigned int b = 5;
char res[4] = {0};
//char bb = 'a';
//extern unsigned int __data_start;
void printf(char * s)
{
	while(*s!=0)
	{
	  *((int *) 0x0200) = *s;
	  s++;
	}
}

int main() {
	
	
	printf("Hello");

	return 0;
	//return fib(20); // fib(20) should be 6765
}
