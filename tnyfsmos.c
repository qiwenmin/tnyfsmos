/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "tnyfsmos.h"

#if defined(__AVR__)

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#endif

extern __tfo_task_cb __tfo_tasks[];
extern const unsigned char __tfo_task_count;

void init_timer();

static volatile unsigned int __tfo_ms = 0;

void process_delay_waiting() {
    for (unsigned char task_id = 0; task_id < __tfo_task_count; task_id ++) {
        if (TFO_STATE_DELAYING(__tfo_tasks[task_id].state)) {
            if (__tfo_tasks[task_id].delay_until == __tfo_ms) {
                __tfo_tasks[task_id].delay_until = 0;
                __tfo_tasks[task_id].state &= (~__TFO_STATE_FLAGS_MASK);
            }
        }
    }
}

#if defined(__SDCC)

unsigned int tfo_millis() __critical {
    return __tfo_ms;
}

#elif defined(__AVR__)

unsigned int tfo_millis() {
    unsigned int m;
    uint8_t oldSREG = SREG;

    cli();
    m = __tfo_ms;
    SREG = oldSREG;

    return m;
}

#endif

tfo_task_state tfo_get_task_state(unsigned char task_id) {
    return __tfo_tasks[(task_id)].state;
}

void tfo_in_state(unsigned char task_id) {
    if (TFO_STATE_DELAYING(__tfo_tasks[task_id].state)) return;

    __tfo_tasks[task_id].state |= __TFO_STATE_INSTATE_MASK;
}

void tfo_goto_state(unsigned char task_id, tfo_task_state state) {
    if (TFO_STATE_DELAYING(__tfo_tasks[task_id].state)) return;

    tfo_goto_state_force(task_id, state);
}

void tfo_goto_state_force(unsigned char task_id, tfo_task_state state) {
    __tfo_tasks[task_id].state = state;
}

void tfo_delay(unsigned char task_id, unsigned int delay_ms, tfo_task_state state) {
    if (TFO_STATE_DELAYING(__tfo_tasks[task_id].state)) return;

    tfo_delay_force(task_id, delay_ms, state);
}

void tfo_delay_force(unsigned char task_id, unsigned int delay_ms, tfo_task_state state) {
    __tfo_tasks[task_id].delay_until = delay_ms + tfo_millis();

    __tfo_tasks[task_id].state = state | __TFO_STATE_DELAYING_MASK;
}

static void init_task_cb(unsigned char task_id) {
    __tfo_task_cb *p = &(__tfo_tasks[task_id]);
    p->state = 0;
    p->delay_until = 0;
}

void tfo_init_os() {
    // Initialize all task control blocks.
    for (unsigned char i = 0; i < __tfo_task_count; i ++) {
        init_task_cb(i);
    }

    // Initialize timer
    init_timer();
}

/*********************************************************************
 * Arduino MiniCore ATMega8
 */
#ifdef ARDUINO_AVR_ATmega8

#if (F_CPU == 1000000L)

// for 125 ticks
#define TCNT0_INIT (130)

// for clock freq / 8
#define TCCR0_INIT (0x02)

#endif

void init_timer() {
    TCCR0 = TCCR0_INIT; // clock frequency / 1024
    TIMSK |= (1 << TOIE0); // Enable overflow interrupt
    TCNT0 = TCNT0_INIT; // Start to count from zero

    sei();
}

ISR(TIMER0_OVF_vect) // Timer 0 overflow vector - this run every time timer0 overflows
{
    TIMSK &= ~(1 << TOIE0);

    process_delay_waiting();

    __tfo_ms ++;

    TIMSK |= (1 << TOIE0);
    TCNT0 = TCNT0_INIT;
}
#endif

/*********************************************************************
 * MCS51
 *********************************************************************/
#ifdef __SDCC_mcs51

#define FOSC 6000000

void timer2_isr() __interrupt 12 {
    process_delay_waiting();

    __tfo_ms ++;
}

void init_timer() {
    AUXR &= ~0x04; //Timer2 clock is 12T mode

    // 1ms
    T2L = (0x10000 - FOSC / 12 / 1000) & 0xFF;
    T2H = (0x10000 - FOSC / 12 / 1000) >> 8;

    AUXR |= 0x10; // Start Timer2
    IE2 |= 0x04; // Enable Timer2 interrupt

    EA = 1; // Open global interrupt switch
}

#endif // __SDCC_mcs51

/*********************************************************************
 * STM8
 *********************************************************************/
#ifdef __SDCC_stm8

#include "stm8util.h"

void timer4_millis_isr() __interrupt(TIM4_ISR) {
    clear_bit(TIM4_SR, TIM4_SR_UIF);

    process_delay_waiting();

    __tfo_ms ++;
}

void init_timer() {
    enable_interrupts();
    TIM4_PSCR = 3;
    TIM4_ARR = 249;
    set_bit(TIM4_IER, TIM4_IER_UIE);
    set_bit(TIM4_CR1, TIM4_CR1_CEN);
}

#endif // __SDCC_stm8
