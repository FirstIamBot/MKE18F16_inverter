/*
 * app_buttons_led.h
 * Обработка кнопок и FAULT LED для инвертора 115 В / 400 Гц.
 *
 * Кнопки:
 *   STAR_STOP (PTB0, pin 34) — запуск/останов инвертора.
 *   TAB_CAL   (PTB1, pin 33) — переключение режима синусоиды TABLE ↔ CALC.
 *
 * Обе кнопки обрабатываются программным дребезгом (20 мс) в задаче FreeRTOS
 * с периодом 5 мс.
 *
 * FAULT LED:
 *   FAULT_LED (PTC9, pin 35) — мигает при аварии (500 мс вкл / 500 мс выкл).
 *   Управляется из задачи вместе с кнопками.
 */

#ifndef APP_BUTTONS_LED_H_
#define APP_BUTTONS_LED_H_

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "ftm_spwm_dma.h"

/* -----------------------------------------------------------------------
 * Параметры
 * ----------------------------------------------------------------------- */
/** Период опроса кнопок (мс) */
#define BTN_POLL_PERIOD_MS     5U
/** Порог дребезга: число подряд совпадающих отсчётов */
#define BTN_DEBOUNCE_COUNT     4U   /* 4 × 5 мс = 20 мс */
/** Период мигания FAULT LED (мс в одном состоянии) */
#define FAULT_LED_BLINK_MS     500U

/* -----------------------------------------------------------------------
 * API
 * ----------------------------------------------------------------------- */
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Создать задачу FreeRTOS для обработки кнопок и LED.
 *        Вызывать один раз после SPWM_Init().
 * @param spwm Указатель на SPWM-дескриптор.
 */
void AppButtonsLed_CreateTask(spwm_t *spwm);

#ifdef __cplusplus
}
#endif

#endif /* APP_BUTTONS_LED_H_ */
