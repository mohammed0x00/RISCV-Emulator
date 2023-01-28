#include "interface.h"

#include <QApplication>
#include <sys/shm.h>

#include <signal.h>

char buffer;
Interface * main_interface;
int shmid;
char *shared_mem;

void receive_data(int signal);

int main(int argc, char *argv[])
{
    // key for the shared memory segment
    key_t key = 0xDEADBEEF;

    // attach the shared memory segment to the process's address space
    shmid = shmget(key, 0, 0666);
    if (shmid == -1) {
        return 1;
    }

    shared_mem = (char *)shmat(shmid, NULL, 0);
    if (shared_mem == (char *) -1) {
        return 1;
    }

    signal(SIGUSR1, receive_data);
    QApplication a(argc, argv);
    Interface w;
    main_interface = &w;
    w.set_sharedmem(shared_mem);
    w.show();
    return a.exec();
}


void receive_data(int signal) {
    if (signal == SIGUSR1) {
        for(int i=0; i< 8;i++)
        {
            if(shared_mem[0] & (1<<i)) main_interface->set_led(i);
            else main_interface->clear_led(i);
        }
    }
}
