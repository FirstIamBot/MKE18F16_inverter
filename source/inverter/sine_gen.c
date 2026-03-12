/*
 * sine_gen.c
 * Генератор синусоидального опорного сигнала.
 *
 * Таблица CnV строится в SineGen_Init() однократно через GFLIB_Sin_FLT_C()
 * из NXP RTCESL libGFLIB.a. В ISR используется только чтение из таблицы
 * (SINE_MODE_TABLE) или прямой вызов GFLIB_Sin_FLT_C() (SINE_MODE_CALC).
 *
 * Формула для полумостовой топологии (CnV смещено от 0 до MOD):
 *
 *   duty(i) = 0.5 * (1 + SINE_MODULATION_INDEX * sin(2π*i/N))
 *   CnV(i)  = round(duty(i) * MOD)
 *
 * GFLIB_Sin_FLT_C(fltAngle):
 *   - Принимает угол в радианах (float_t)
 *   - Возвращает sin(angle) ∈ [-1.0, +1.0]
 *   - Реализован через полиномиальную аппроксимацию, оптимизированную
 *     для Cortex-M4F FPU (libGFLIB.a)
 */

#include "sine_gen.h"
#include <string.h>

/* -----------------------------------------------------------------------
 * Инициализация: заполнение таблицы через GFLIB_Sin_FLT_C()
 * ----------------------------------------------------------------------- */
void SineGen_Init(sine_gen_t *gen, uint32_t pwm_modulo, sine_mode_t mode)
{
    uint32_t i;
    float_t  angle;
    float_t  duty;
    float_t  mod_f = (float_t)pwm_modulo;

    memset(gen, 0, sizeof(*gen));
    gen->pwm_modulo = pwm_modulo;
    gen->mode       = mode;
    gen->step       = 0U;
    gen->phase_acc  = 0.0f;

    /*
     * Заполняем таблицу N=50 значений CnV.
     * GFLIB_Sin_FLT_C(angle) — макрос, вызывающий GFLIB_Sin_FLT_FC()
     * из libGFLIB.a с предвычисленными коэффициентами gfltSinCoef.
     */
    for (i = 0U; i < N_SINE_SAMPLES; i++)
    {
        angle = SINE_PHASE_INC * (float_t)i;              /* 0 … 2π*(N-1)/N */
        duty  = 0.5f * (1.0f + SINE_MODULATION_INDEX
                              * GFLIB_Sin_FLT_C(angle));  /* [0.025 … 0.975] */
        /* Округление: +0.5f для round-to-nearest */
        gen->cnv_table[i] = (uint32_t)(duty * mod_f + 0.5f);
    }
}

/* -----------------------------------------------------------------------
 * Следующее значение CnV (вызывается каждые 50 мкс в DMA0_IRQHandler)
 * ----------------------------------------------------------------------- */
uint32_t SineGen_Next(sine_gen_t *gen)
{
    uint32_t cnv;
    float_t  duty;

    if (gen->mode == SINE_MODE_TABLE)
    {
        /*
         * Быстрое чтение из ROM-таблицы: ~2-5 тактов CPU.
         * Нет float-вычислений в ISR.
         */
        cnv = gen->cnv_table[gen->step];
        gen->step++;
        if (gen->step >= N_SINE_SAMPLES)
        {
            gen->step = 0U;
        }
    }
    else
    {
        /*
         * SINE_MODE_CALC: GFLIB_Sin_FLT_C() непосредственно в ISR.
         * Точнее, но занимает ~50-60 тактов FPU Cortex-M4F.
         */
        duty = 0.5f * (1.0f + SINE_MODULATION_INDEX
                             * GFLIB_Sin_FLT_C(gen->phase_acc));
        cnv  = (uint32_t)(duty * (float_t)gen->pwm_modulo + 0.5f);

        gen->phase_acc += SINE_PHASE_INC;
        /* Нормализация: держим phase_acc в [0, 2π) */
        if (gen->phase_acc >= 6.28318530f)
        {
            gen->phase_acc -= 6.28318530f;
        }
    }

    return cnv;
}

/* -----------------------------------------------------------------------
 * Смена режима без потери фазы
 * ----------------------------------------------------------------------- */
void SineGen_SetMode(sine_gen_t *gen, sine_mode_t mode)
{
    if (gen->mode == mode)
    {
        return;
    }

    if (mode == SINE_MODE_CALC)
    {
        /* TABLE → CALC: пересчитать phase_acc из текущего step */
        gen->phase_acc = SINE_PHASE_INC * (float_t)gen->step;
    }
    else
    {
        /* CALC → TABLE: найти ближайший step по phase_acc */
        gen->step = (uint32_t)(gen->phase_acc / SINE_PHASE_INC);
        if (gen->step >= N_SINE_SAMPLES)
        {
            gen->step = 0U;
        }
    }

    gen->mode = mode;
}

/* -----------------------------------------------------------------------
 * Сброс в начало периода
 * ----------------------------------------------------------------------- */
void SineGen_Reset(sine_gen_t *gen)
{
    gen->step      = 0U;
    gen->phase_acc = 0.0f;
}
