#ifndef __LOG_H
#define __LOG_H

#include <stdio.h>

/*
 * printf serialization.
 *
 * printf() is a blocking character loop in debug.c, so two tasks writing at
 * once would interleave their lines. LOG_Lock()/LOG_Unlock() wrap a mutex
 * around the call; use LOGF() for a whole formatted line. Before the mutex
 * exists (early boot) the calls degrade to plain printf().
 */

void LOG_Init(void);
void LOG_Lock(void);
void LOG_Unlock(void);

#define LOGF(...)               \
    do {                        \
        LOG_Lock();             \
        printf(__VA_ARGS__);    \
        LOG_Unlock();           \
    } while(0)

#endif /* __LOG_H */
