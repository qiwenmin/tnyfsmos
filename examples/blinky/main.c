#include "../../tnyfsmos.h"

enum {
    TASK_BLINKY = 0,

    TASK_COUNT
};

TFO_INIT(TASK_COUNT)

enum {
    BLINKY_STATE_TOGGLE = TFO_STATE_USER
};

#ifdef __SDCC_mcs51

#define LED_PIN P3_5

static void init_led(void) {
    P3M0 |= (1 << 5);
    LED_PIN = 1;
}

static void toggle_led(void) {
    LED_PIN = !LED_PIN;
}

#endif // __SDCC_mcs51

#ifdef __SDCC_stm8

#include "../../stm8util.h"

static void init_led(void) {
    set_bit(PB_DDR, 5);
    set_bit(PB_CR1, 5);

    clear_bit(PB_ODR, 5);
}

static void toggle_led(void) {
    toggle_bit(PB_ODR, 5);
}

#endif // __SDCC_stm8

static void blinky_state_machine(void) {
    tfo_task_state state = tfo_get_task_state(TASK_BLINKY);

    if (TFO_STATE_FLAGS(state)) return;

    switch (state) {
        case TFO_STATE_INIT:
            init_led();
            tfo_delay(TASK_BLINKY, 500, BLINKY_STATE_TOGGLE);
            break;
        case BLINKY_STATE_TOGGLE:
            toggle_led();
            tfo_delay(TASK_BLINKY, 500, BLINKY_STATE_TOGGLE);
            break;
    }
}

void main(void) {
    tfo_init_os();

    while (1) {
        blinky_state_machine();
    }
}
