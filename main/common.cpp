// vim: ts=4:sw=4:sts=4:et

#include "common.h"

static const char *TAG = "common";

long fast_int_sqrt(long n) {
    if (n < 2) return n;
    long x = n >> 1;
    long y = ( x + n / x ) >> 1;

    while (y < x) {
        x = y;
        y = (x + n / x) >> 1;
    }
    return x;
}

void sleepMicroseconds(long sleepMicros) {
    if(sleepMicros <= 0) {
        return;
    }
    int64_t time = esp_timer_get_time();
    int64_t end = time + sleepMicros;
    int64_t delta = sleepMicros;
    int numTicks = delta / 1000 / portTICK_PERIOD_MS;
    if(numTicks > 0) {
        vTaskDelay(numTicks);
    }
    while(end > esp_timer_get_time()) ;
}
