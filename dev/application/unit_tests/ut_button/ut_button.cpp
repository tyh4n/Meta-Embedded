#include "ch.hpp"
#include "hal.h"

#include "led.h"

using namespace chibios_rt;

int main(void) {
    halInit();
    System::init();

    LED::all_off();
    LED::red_on();

    // RoboMaster Board C user button: PA0, external pull-up, active-low.
    palSetPadMode(GPIOA, GPIOA_USER_BUTTON, PAL_MODE_INPUT_PULLUP);

    bool stable_pressed = false;
    uint8_t debounce_count = 0;
    bool green_selected = false;

    while (true) {
        const bool raw_pressed =
                palReadPad(GPIOA, GPIOA_USER_BUTTON) == PAL_LOW;

        if (raw_pressed == stable_pressed) {
            debounce_count = 0;
        } else if (++debounce_count >= 3U) {
            stable_pressed = raw_pressed;
            debounce_count = 0;

            if (stable_pressed) {
                green_selected = !green_selected;
                if (green_selected) {
                    LED::red_off();
                    LED::green_on();
                } else {
                    LED::green_off();
                    LED::red_on();
                }
            }
        }

        chThdSleepMilliseconds(10);
    }
}
