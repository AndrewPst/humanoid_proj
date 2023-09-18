#include <Arduino.h>

#include "src/kinematics/kinematics.h"
#include "sout.h"
#include <RTOS.h>

osThreadId thread_id_loop;
osMutexId _s_mutex;
osMutexDef_t _s_mutex_def;

static void thread_loop(void const *arg)
{
    (void)arg;
    for (;;)
    {
        loop();
        if (serialEventRun)
            serialEventRun();
    }
}

void setup()
{
    //init mutexes
    osMutexDef(_s_mutex);
    _s_mutex = osMutexCreate(osMutex(_s_mutex));

    //SERIAL_INIT
    SERIAL_BEGIN
    SERIAL_INIT
    SERIAL_END
    // Register tasks
    osThreadDef(THREAD_NAME_LOOP, thread_loop, osPriorityNormal, 0, 8192);

    // Serial.println("Started");
    //  create tasks
    thread_id_loop = osThreadCreate(osThread(THREAD_NAME_LOOP), NULL);

    // start kernel
    osKernelStart();
}

double buf{0};
kinematics::pos_t tpos;
kinematics::leg_t _out;

void loop()
{
    if (Serial.available())
    {
        buf = Serial.readString().toFloat();
        tpos.z = buf;
        Serial.println(tpos.z);
        Serial.flush();
        kinematics::legIK(tpos, &_out);
        Serial.print(_out.values[0]);
        Serial.print('\t');
        Serial.print(_out.values[1]);
        Serial.print('\t');
        Serial.println(_out.values[2]);
        Serial.flush();
        // dxl_wb.goalPosition(indexes[0], (float)(_out._servos[0] + M_PI_2));
        // dxl_wb.goalPosition(indexes[1], (float)(_out._servos[1] + M_PI_2));
        // dxl_wb.goalPosition(indexes[2], (float)(M_PI_2 - _out._servos[2]));
    }
    osDelay(5);
}
