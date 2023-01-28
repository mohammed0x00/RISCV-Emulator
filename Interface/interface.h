#ifndef INTERFACE_H
#define INTERFACE_H

#include <QMainWindow>
#include <QLabel>

QT_BEGIN_NAMESPACE
namespace Ui { class Interface; }
QT_END_NAMESPACE

class Interface : public QMainWindow
{
    Q_OBJECT

public:
    Interface(QWidget *parent = nullptr);
    ~Interface();
    void set_led(int index);
    void clear_led(int index);
    void set_sharedmem(void * address);

private slots:

    void on_horizontalSlider_0_valueChanged(int value);

    void on_horizontalSlider_1_valueChanged(int value);

    void on_horizontalSlider_2_valueChanged(int value);

    void on_horizontalSlider_3_valueChanged(int value);

    void on_pushButton_0_pressed();

    void on_pushButton_0_released();

    void on_pushButton_1_pressed();

    void on_pushButton_1_released();

    void on_pushButton_2_pressed();

    void on_pushButton_2_released();

    void on_pushButton_3_pressed();

    void on_pushButton_3_released();

private:
    Ui::Interface *ui;
    QPixmap *led_on_img;
    QPixmap *led_off_img;
    QLabel **ui_leds;
    char * shared_mem;
};
#endif // INTERFACE_H
