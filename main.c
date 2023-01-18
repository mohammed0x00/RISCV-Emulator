#include <stdio.h>
#include <stdint.h>
#include <pthread.h>
#include <unistd.h>
#include <sched.h>
#include "core.h"
#define RAM_SIZE    64*1024*1024
#define RAM_BASE    0x80000000
uint8_t ram[RAM_SIZE];
riscv_core mycore;

uint8_t memory_write(uint32_t address, uint32_t data, uint8_t type, uint8_t bytes_num);
uint8_t memory_read(uint32_t address, uint8_t type, uint32_t * dest, uint8_t bytes_num);
void *timer_signal_thread(void *arg);


int main(void)
{
    pthread_t thread;
    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    pthread_attr_setschedparam(&attr, &param);
    
    //pthread_create(&thread, &attr, timer_signal_thread, NULL);

    FILE *in=fopen("Image2","r");
    fseek( in, 0, SEEK_END );
	long flen = ftell( in );
	fseek( in, 0, SEEK_SET );
	if(fread( ram, flen, 1, in )!=1)
    {
        printf("Cannot load Kernel\n");
        return 0;
    }
	fclose(in);

    FILE *in2=fopen("./minimal.dtb","r");
    fseek( in2, 0, SEEK_END );
	flen = ftell( in2 );
	fseek( in2, 0, SEEK_SET );
	if(fread(ram + RAM_SIZE - flen, flen, 1, in2 )!=1)
    {
        printf("Cannot load dtb\n");
        return 0;
    }
	fclose(in2);

    mycore.mvendorid = 0;
    mycore.operations.mem_read = memory_read;
    mycore.operations.mem_write = memory_write;
    Core_Init(&mycore);
    mycore.x[10] = 0x00;
    mycore.x[11] = RAM_BASE + RAM_SIZE - flen;
    mycore.pc = RAM_BASE;
    for(int i=0;;i++)
    {
        if(i>= 100){mycore.time++;i=0;}
        Core_SingleCycle(&mycore);
    }
    pthread_join(thread, NULL);
}

uint8_t memory_write(uint32_t address, uint32_t data, uint8_t type, uint8_t bytes_num)
{
    if (address >= 0x10000000 && address < 0x12000000)
    {
        if(address==0x10000000)
        {
            printf("%c", (uint8_t)data);
            fflush(stdout);
        }
        else if(address == 0x11004004)
        {
            *((uint32_t *)((&mycore.timercmp)+4)) = data;
        }
        else if(address == 0x11004000)
        {
            *((uint32_t *)(&mycore.timercmp)) = data;
        }
        return ACCESS_GRANTED;
    }
    else if(address >= RAM_BASE && address < RAM_BASE + RAM_SIZE)
    {
        address -= RAM_BASE;
        switch (bytes_num)
        {
        case 1:
            ram[address] = (uint8_t) data;
            break;
        case 2:
            *(uint16_t *)(ram + address) = (uint16_t) data;
            break;
        case 4:
            *(uint32_t *)(ram + address) = data;
            break;
        }
        return ACCESS_GRANTED;
    }
    return ACCESS_REVOKED;
}

uint8_t memory_read(uint32_t address, uint8_t type, uint32_t * dest, uint8_t bytes_num)
{
    if (address >= 0x1000000 && address < 0x12000000)
    {
        if(address == 0x10000000)
        {
            *dest = 0;
        }
        else if( address == 0x10000005 )
		    *dest = 0x60;
        else if(address == 0x1100bff8)
            *dest = (uint32_t)mycore.time;
        else if(address == 0x1100bffc)
            *dest = (uint32_t)(mycore.time >> 32);
        else
            *dest = 0;
        return ACCESS_GRANTED;
    }
    if(address >= RAM_BASE && address < RAM_BASE + RAM_SIZE)
    {
        address -= RAM_BASE;
        *dest = *(uint32_t *)(ram + address);
        return ACCESS_GRANTED;
    }
    return ACCESS_REVOKED;
}

void *timer_signal_thread(void *arg) {
    while (1) {
        mycore.time+=20;
        usleep(20);
    }
    return NULL;
}