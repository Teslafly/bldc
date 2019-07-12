/*
	Copyright 2019 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "timer.h"
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"

// Settings
#define TIMER_HZ					1e7 //10mhz
#define FAST_TIMER_HW     TIM5
#define FAST_TIMER_Periph RCC_APB1Periph_TIM5

// todo, move function timers to a 16 bit timer (free up timer 5)
// move tachometer/rpm timer use to systick clock if accurate enough.

void timer_init(void) {
	RCC_APB1PeriphClockCmd(FAST_TIMER_Periph, ENABLE);
	uint16_t PrescalerValue = (uint16_t) ((SYSTEM_CORE_CLOCK / 2) / TIMER_HZ) - 1;

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(FAST_TIMER_HW, &TIM_TimeBaseStructure);

	FAST_TIMER_HW->CNT = 0;
	TIM_Cmd(FAST_TIMER_HW, ENABLE);
}

uint32_t timer_time_now(void) {
	return FAST_TIMER_HW->CNT;
}

float timer_seconds_elapsed_since(uint32_t time) {
	uint32_t diff = FAST_TIMER_HW->CNT - time;
	return (float)diff / (float)TIMER_HZ;
}

/**
 * Blocking sleep based on timer.
 *
 * @param seconds
 * Seconds to sleep.
 */
void timer_sleep(float seconds) {
	uint32_t start_t = FAST_TIMER_HW->CNT;

	for (;;) {
		if (timer_seconds_elapsed_since(start_t) >= seconds) {
			return;
		}
	}
}
