#ifndef __kinect_timestamp_h__
#define __kinect_timestamp_h__

#include <stdint.h>
#include <sys/time.h>
#include <time.h>

typedef struct timestamp_sync_state timestamp_sync_state_t;

struct timestamp_sync_state {
    double  dev_ticks_per_second; // how fast does device clock count? (nominal)
    int64_t dev_ticks_wraparound; // device clock counts modulo what?
    double  max_rate_error;       // how fast do we need to count to ensure we're counting faster than device?

    int64_t sync_host_time;       // when we last synced, what time was it for the host?
    int64_t dev_ticks_since_sync; // how many device ticks have elapsed since the last sync?

    int64_t last_dev_ticks;        // what device time was it when we were last called?

    uint8_t is_valid;             // have we ever synced?
};

int64_t timestamp_now(void);
int64_t timestamp_seconds(int64_t v);
int64_t timestamp_useconds(int64_t v);
void timestamp_to_timeval(int64_t v, struct timeval *tv);
void timestamp_to_timespec(int64_t v, struct timespec *ts);

/** Create a new time synchronizer.
    @param dev_ticks_per_second  The nominal rate at which the device time increments
    @param dev_ticks_wraparound  Assume that dev_ticks wraps around every wraparound ticks
    @param rate                  An upper bound on the rate error
**/
timestamp_sync_state_t *
timestamp_sync_init (double dev_ticks_per_second, int64_t dev_ticks_wraparound,
        double rate);
void
timestamp_sync_free (timestamp_sync_state_t * s);
int64_t
timestamp_sync (timestamp_sync_state_t * s, int64_t dev_ticks,
        int64_t host_utime);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif
