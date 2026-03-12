/*
 * sine_gen.h
 * Генератор синусоидального опорного сигнала на базе NXP RTCESL GFLIB.
 *
 * Синус вычисляется через GFLIB_Sin_FLT_C(fltAngle) из libGFLIB.a.
 * Функция принимает угол в радианах (float_t) и возвращает sin(angle)
 * в диапазоне [-1.0, +1.0].
 *
 * Параметры:
 *   f_out  = 400 Гц (авиационный стандарт)
 *   f_sw   = 20 кГц (FTM1, peripherals.c)
 *   N      = f_sw / f_out = 50 — целое, нет накопления фазовой ошибки
 *   V_rms  = 115 В, полумост: duty = 0.5*(1 + ma*sin(ωt)), ma = 0.95
 *
 * Таблица CnV строится однократно в SineGen_Init() вызовом GFLIB_Sin_FLT_C()
 * для каждого из N=50 углов. В ISR SineGen_Next() читает из таблицы — без
 * float-вычислений в реальном времени.
 *
 * Второй режим SINE_MODE_CALC вычисляет GFLIB_Sin_FLT_C() непосредственно
 * в ISR (для отладки и прецизионных измерений).
 */

#ifndef SINE_GEN_H_
#define SINE_GEN_H_

#include <stdint.h>
#include <stdbool.h>

/* NXP RTCESL GFLIB: GFLIB_Sin_FLT_C, GFLIB_SIN_T_FLT, gfltSinCoef */
#include "gflib_FP.h"

/* -----------------------------------------------------------------------
 * Параметры
 * ----------------------------------------------------------------------- */
#define SINE_VOUT_RMS_V        115.0f
#define SINE_VDC_V             28.0f
#define SINE_FREQ_HZ           400.0f
#define SINE_FSW_HZ            20000.0f
/** Число точек на период: f_sw / f_out = 20000 / 400 = 50 (целое) */
#define N_SINE_SAMPLES         50U
#define SINE_MODULATION_INDEX  0.95f
/** 2π / N — шаг фазы за один ШИМ-период */
#define SINE_PHASE_INC         (6.28318530f / (float)(N_SINE_SAMPLES))

/* -----------------------------------------------------------------------
 * Типы
 * ----------------------------------------------------------------------- */
typedef enum
{
    SINE_MODE_TABLE = 0,   /**< ROM-таблица 50 точек (built via GFLIB_Sin, fast) */
    SINE_MODE_CALC         /**< GFLIB_Sin_FLT_C() в ISR (точнее, медленнее)     */
} sine_mode_t;

typedef struct
{
    sine_mode_t  mode;
    uint32_t     step;         /**< Текущий индекс 0…N-1 (TABLE mode)    */
    float_t      phase_acc;    /**< Фазовый аккумулятор (рад, CALC mode) */
    uint32_t     pwm_modulo;   /**< Значение FTM1_TIMER_MODULO_VALUE      */
    /** ROM-таблица, заполняется в SineGen_Init() через GFLIB_Sin_FLT_C() */
    uint32_t     cnv_table[N_SINE_SAMPLES];
} sine_gen_t;

/* -----------------------------------------------------------------------
 * API
 * ----------------------------------------------------------------------- */
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Инициализация генератора.
 *        Заполняет cnv_table[N] значениями CnV через GFLIB_Sin_FLT_C().
 * @param gen         Дескриптор.
 * @param pwm_modulo  Значение FTM1_TIMER_MODULO_VALUE.
 * @param mode        Начальный режим TABLE или CALC.
 */
void SineGen_Init(sine_gen_t *gen, uint32_t pwm_modulo, sine_mode_t mode);

/**
 * @brief Следующее значение CnV — вызывать каждый ШИМ-период.
 *        TABLE: читает cnv_table[step], step++.
 *        CALC:  GFLIB_Sin_FLT_C(phase_acc), phase_acc += SINE_PHASE_INC.
 * @return CnV для FTM1->CONTROLS[0].CnV (0 … pwm_modulo).
 */
uint32_t SineGen_Next(sine_gen_t *gen);

/** Переключение режима без потери фазы. */
void SineGen_SetMode(sine_gen_t *gen, sine_mode_t mode);

/** Сброс в начало периода (phase = 0, step = 0). */
void SineGen_Reset(sine_gen_t *gen);

#ifdef __cplusplus
}
#endif

#endif /* SINE_GEN_H_ */
