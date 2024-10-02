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

#ifndef __TNYFSMOS_H__
#define __TNYFSMOS_H__

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__SDCC_mcs51)
    #include "stc15.h"
#elif defined(__SDCC_stm8)
    #include "stm8s.h"
#endif

typedef unsigned char tfo_task_state;

typedef struct {
    tfo_task_state state;
    unsigned int delay_until;
} __tfo_task_cb;

#if defined(__SDCC_mcs51)
#ifdef STC8G1K08
    #define __TFO_TIMER_DECL void timer1_isr(void) __interrupt (3)
#else
    #define __TFO_TIMER_DECL void timer2_isr(void) __interrupt (12)
#endif // STC8G1K08
#elif defined(__SDCC_stm8)
    #define __TFO_TIMER_DECL void timer4_millis_isr(void) __interrupt(TIM4_ISR)
#endif

#if defined(__AVR__)
#define TFO_INIT(n) \
    __tfo_task_cb __tfo_tasks[(n)]; \
    const unsigned char __tfo_task_count = (n);
#else
#define TFO_INIT(n) \
    __tfo_task_cb __tfo_tasks[(n)]; \
    const unsigned char __tfo_task_count = (n); \
    __TFO_TIMER_DECL;
#endif

#define TFO_STATE_INIT (0)
#define TFO_STATE_USER (16)

#define __TFO_STATE_VALUE_MASK (0x3F)
#define __TFO_STATE_INSTATE_MASK (0x40)
#define __TFO_STATE_DELAYING_MASK (0x80)
#define __TFO_STATE_FLAGS_MASK (__TFO_STATE_INSTATE_MASK | __TFO_STATE_DELAYING_MASK)

#define TFO_STATE_VALUE(v) ((v) & __TFO_STATE_VALUE_MASK)
#define TFO_STATE_INSTATE(v) ((v) & __TFO_STATE_INSTATE_MASK)
#define TFO_STATE_DELAYING(v) ((v) & __TFO_STATE_DELAYING_MASK)
#define TFO_STATE_FLAGS(v) ((v) & (__TFO_STATE_FLAGS_MASK))
#define TFO_STATE_IS_WATING_FOR(v, s) (v == (s | __TFO_STATE_DELAYING_MASK))

unsigned int tfo_millis(void);

void tfo_init_os(void);

tfo_task_state tfo_get_task_state(unsigned char task_id);

void tfo_in_state(unsigned char task_id);
void tfo_goto_state(unsigned char task_id, tfo_task_state state);
void tfo_goto_state_force(unsigned char task_id, tfo_task_state state);
void tfo_delay(unsigned char task_id, unsigned int delay_ms, tfo_task_state state);
void tfo_delay_force(unsigned char task_id, unsigned int delay_ms, tfo_task_state state);

#ifdef __cplusplus
}
#endif

#endif // __TNYFSMOS_H__
