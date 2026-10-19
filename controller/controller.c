/* RP2040 hand-controller placeholder. Radio/display/joystick pins are unset. */
#include <stdio.h>
#include "pico/stdlib.h"

int main(void) {
    stdio_init_all();
    sleep_ms(1500);
    puts("QCopter controller safe scaffold: inputs and radio are not configured.");
    for (;;) {
        /* No commands are transmitted until hardware and packet protocol are set. */
        tight_loop_contents();
    }
}
