#pragma once

#include <RTOS.h>


namespace rtos
{
    //сигнатура функции для отдельного потока. Аргументы прорабатываются по ходу разработки
    using taskHundler = void(*)();

    void createTask(taskHundler&);
}