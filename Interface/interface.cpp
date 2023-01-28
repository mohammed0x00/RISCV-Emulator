#include <QFile>
#include "interface.h"
#include "ui_interface.h"

#define LED_ON_IMAGE ":/Leds/led_on.png"
#define LED_OFF_IMAGE ":/Leds/led_off.png"

Interface::Interface(QWidget *parent): QMainWindow(parent), ui(new Ui::Interface)
{
    ui->setupUi(this);
    led_on_img = new QPixmap(LED_ON_IMAGE);
    led_off_img = new QPixmap(LED_OFF_IMAGE);
    ui_leds = new QLabel *[]{
        ui->led_0,
        ui->led_1,
        ui->led_2,
        ui->led_3,
        ui->led_4,
        ui->led_5,
        ui->led_6,
        ui->led_7,
    };
}

void Interface::set_led(int index)
{
    if(index < 8)
    ui_leds[index]->setPixmap(*led_on_img);
}

void Interface::clear_led(int index)
{
    if(index < 8)
    ui_leds[index]->setPixmap(*led_off_img);
}

Interface::~Interface()
{
    delete ui;
}


void Interface::set_sharedmem(void * address)
{
    shared_mem = (char *)address;
}


void Interface::on_horizontalSlider_0_valueChanged(int value)
{
    shared_mem[2] &= 0x0F;
    if(value == 1) shared_mem[2] |= (1<<0);
    else shared_mem[2] &= ~(1<<0);
}


void Interface::on_horizontalSlider_1_valueChanged(int value)
{
    shared_mem[2] &= 0x0F;
    if(value == 1) shared_mem[2] |= (1<<1);
    else shared_mem[2] &= ~(1<<1);
}


void Interface::on_horizontalSlider_2_valueChanged(int value)
{
    shared_mem[2] &= 0x0F;
    if(value == 1) shared_mem[2] |= (1<<2);
    else shared_mem[2] &= ~(1<<2);
}


void Interface::on_horizontalSlider_3_valueChanged(int value)
{
    shared_mem[2] &= 0x0F;
    if(value == 1) shared_mem[2] |= (1<<3);
    else shared_mem[2] &= ~(1<<3);
}


void Interface::on_pushButton_0_pressed()
{
    shared_mem[1] |= (1<<0);
}


void Interface::on_pushButton_0_released()
{
    shared_mem[1] &= ~(1<<0);
}


void Interface::on_pushButton_1_pressed()
{
    shared_mem[1] |= (1<<1);
}


void Interface::on_pushButton_1_released()
{
    shared_mem[1] &= ~(1<<1);
}


void Interface::on_pushButton_2_pressed()
{
    shared_mem[1] |= (1<<2);
}


void Interface::on_pushButton_2_released()
{
    shared_mem[1] &= ~(1<<2);
}


void Interface::on_pushButton_3_pressed()
{
    shared_mem[1] |= (1<<3);
}


void Interface::on_pushButton_3_released()
{
    shared_mem[1] &= ~(1<<3);
}

