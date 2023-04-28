// SPDX-License-Identifier: MIT
// Copyright (c) 2018-2023 The Pybricks Authors

#include <pbio/battery.h>
#include <pbio/control.h>
#include <pbio/drivebase.h>
#include <pbio/servo.h>

#include <contiki.h>

#if PBIO_CONFIG_MOTOR_PROCESS != 0

PROCESS(pbio_motor_process, "servo");

PROCESS_THREAD(pbio_motor_process, ev, data) {
    static struct etimer timer;

    PROCESS_BEGIN();

    // Initialize battery voltage.
    pbio_battery_init();

    // Initialize motors in stopped state.
    pbio_dcmotor_stop_all(true);

    etimer_set(&timer, PBIO_CONFIG_CONTROL_LOOP_TIME_MS);

    for (;;) {
        PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER && etimer_expired(&timer));

        // Update battery voltage.
        pbio_battery_update();

        // Update drivebase
        pbio_drivebase_update_all();

        // Update servos
        pbio_servo_update_all();

        // Reset timer to wait for next update. Using etimer_reset() instead
        // of etimer_restart() makes average update period closer to the expected
        // PBIO_CONFIG_CONTROL_LOOP_TIME_MS when occasional delays occur.
        etimer_reset(&timer);

        // If we have fallen too far behind though, jump ahead to catch up.
        // Otherwise, this process will run in a tight loop and not yield
        // until the timer catches up.
        if (timer_expired(&timer.timer)) {
            etimer_restart(&timer);
        }
    }

    PROCESS_END();
}

void pbio_motor_process_start(void) {
    process_start(&pbio_motor_process);
}

#endif // PBIO_CONFIG_MOTOR_PROCESS
