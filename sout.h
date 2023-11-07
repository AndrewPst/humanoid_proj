#pragma once

#include "config.h"
#include <RTOS.h>

#if DEBUG_LOG == 1
#define SERIAL_BEGIN osMutexWait(_s_mutex, 0);
#define SERIAL_END osMutexRelease(_s_mutex);

extern osMutexId _s_mutex;

// Thread safe realization of serial out
#define SERIAL_INIT           \
    Serial.setTimeout(5);    \
    Serial.begin(DEBUG_BOUD); \
    Serial.flush();
#define SERIAL_OUT(text) \
    Serial.print(text);  \
    Serial.flush();
#define SERIAL_OUT_L(text) \
    Serial.println(text);  \
    Serial.flush();
#define SERIAL_OUT_THRSAFE(text) \
    SERIAL_BEGIN                 \
    SERIAL_OUT(text)             \
    SERIAL_END
#define SERIAL_OUT_L_THRSAFE(text) \
    SERIAL_BEGIN                   \
    SERIAL_OUT_L(text)             \
    SERIAL_END

#define SERIAL_READ_STRING_THREAD_SAVE(out) \
    SERIAL_BEGIN;                           \
    (out) = Serial.readString();            \
    SERIAL_END;

#define SERIAL_READ_STRING_UNTIL_THREAD_SAVE(out, sep) \
    SERIAL_BEGIN;                                      \
    (out) = Serial.readStringUntil(sep);               \
    SERIAL_END;

#define SERIAL_AVAIABLE_THRSAFE(out) \
    SERIAL_BEGIN;                    \
    out = Serial.available();        \
    SERIAL_END;

#else
#define SERIAL_INIT
#define SERIAL_OUT(text)
#define SERIAL_OUT_L(text)
#define SERIAL_BEGIN
#define SERIAL_END
#define SERIAL_OUT_THRSAFE(text)
#define SERIAL_OUT_L_THRSAFE(text)
#define SERIAL_READ_STRING_THREAD_SAVE(out)
#define SERIAL_READ_STRING_UNTIL_THREAD_SAVE(out, sep)
#define SERIAL_AVAIABLE_THRSAFE(out)
#endif
