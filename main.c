#include <stdio.h>
#include <stdint.h>
#include <pthread.h>
#include <unistd.h>
#include <stdlib.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sched.h>
#include <spawn.h>
#include <sys/wait.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <signal.h>

#include "core.h"
#define RAM_SIZE    128*1024*1024
#define RAM_BASE    0x80000000
uint8_t ram[RAM_SIZE];
riscv_core mycore;
extern char **environ;
char* shared_memory;
int shmid;
pid_t receiver_pid;

uint8_t memory_write(uint32_t address, uint32_t data, uint8_t type, uint8_t bytes_num);
uint8_t memory_read(uint32_t address, uint8_t type, uint32_t * dest, uint8_t bytes_num);
void *interface_thread(void *arg);
void *timer_signal_thread(void *arg);
void run_interface_application(void);
void terminal_init(void);
int stdin_has_char(void);
char stdin_read_char(void);
void sigterm_handler(int signum);


int main(void)
{
    signal(SIGINT, sigterm_handler);
    signal(SIGTERM, sigterm_handler);

    key_t key = 0xDEADBEEF;
    size_t size = 4096;
    int shmflg = IPC_CREAT | 0666;

    // create the shared memory segment
    shmid = shmget(key, size, shmflg);
    if (shmid == -1) {
        perror("Error creating shared memory segment");
        return 1;
    }

    // attach the shared memory segment to the process's address space
    shared_memory = shmat(shmid, NULL, 0);
    if (shared_memory == (char *) -1) {
        perror("Error attaching shared memory segment");
        shmctl(shmid, IPC_RMID, NULL);
        return 1;
    }

    pthread_t interface;
    //pthread_t timer;
    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    pthread_attr_setschedparam(&attr, &param);
    
    //pthread_create(&timer, &attr, timer_signal_thread, NULL);
    pthread_create(&interface, NULL, interface_thread, NULL);
    
    terminal_init();

    FILE *in=fopen("./linux/Image","r");
    fseek( in, 0, SEEK_END );
	long flen = ftell( in );
	fseek( in, 0, SEEK_SET );
	if(fread( ram, flen, 1, in )!=1)
    {
        printf("Cannot load Kernel\n");
        return 0;
    }
	fclose(in);

    FILE *in2=fopen("./linux/devicetree.dtb","r");
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
        if(i>= 300){mycore.time++;i=0;}
        Core_SingleCycle(&mycore);
    }
    return 0;
}

uint8_t memory_write(uint32_t address, uint32_t data, uint8_t type, uint8_t bytes_num)
{
    if (address >= 0x30000000 && address < 0x32000000)
    {
        if(address==0x30000000)
        {
            printf("%c", (uint8_t)data);
            fflush(stdout);
        }
        else if(address == 0x31004004)
        {
            *((uint32_t *)((&mycore.timercmp)+4)) = data;
        }
        else if(address == 0x31004000)
        {
            *((uint32_t *)(&mycore.timercmp)) = data;
        }
        return ACCESS_GRANTED;
    }
    else if((address >= 0x40000000) && (address < 0x4000000F))
    {
        address -= 0x40000000;
        shared_memory[address] = (uint8_t) data;
        kill(receiver_pid, SIGUSR1);
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
    if (address >= 0x3000000 && address < 0x32000000)
    {
        if(address == 0x30000000)
        {
            *dest = stdin_read_char();
        }
        else if(address == 0x30000005)
		{
            *dest = 0x60 | stdin_has_char();
        }
        else if(address == 0x3100bff8)
            *dest = (uint32_t)mycore.time;
        else if(address == 0x3100bffc)
            *dest = (uint32_t)(mycore.time >> 32);
        else
            *dest = 0;
        return ACCESS_GRANTED;
    }
    else if((address >= 0x40000000) && (address < 0x4000000F))
    {
        address -= 0x40000000;
        *dest = shared_memory[address];
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

void *interface_thread(void *arg) {
    run_interface_application();
    sigterm_handler(0);
    return NULL;
}

void *timer_signal_thread(void *arg) {
    while (1) {
        mycore.time+=20;
        usleep(20);
    }
    return NULL;
}

void terminal_init(void)
{
    struct termios tty;
    tcgetattr(STDIN_FILENO, &tty);
    tty.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &tty);
}

int stdin_has_char(void) {
    int bytes_waiting;
    ioctl(STDIN_FILENO, FIONREAD, &bytes_waiting);
    if(bytes_waiting > 0)
        return 1;
    else
        return 0;
}

char stdin_read_char(void)
{
    char c;
    if(read(STDIN_FILENO, &c, 1) > 0)
    {
        return c;  
    }
    else
    {
        return -1;
    }
}

void run_interface_application(void) {
    // create a new process and execute the application
    const char* path = "./Interface/Interface";
    char* const argv[8] = {};
    pid_t pid;
    int status;

    /* Create a new process */
    if (posix_spawn(&pid, path, NULL, NULL, argv, environ) != 0) {
        perror("posix_spawn");
        return;
    }

    receiver_pid = pid;
    /* Wait for the child process to complete */
    if (waitpid(pid, &status, 0) == -1) {
        perror("waitpid");
        return;
    }
}

void sigterm_handler(int signum)
{
    shmctl(shmid, IPC_RMID, 0);
    exit(0);
}

