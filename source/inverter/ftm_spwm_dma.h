/*
 * ftm_spwm_dma.h
 * SPWM-движок: FTM1 Combined PWM + PDB0 DMA-триггер + eDMA ping-pong + ADC0.
 *
 * ─── Периферия (board/peripherals.c, board/pin_mux.c) ──────────────────────
 *  FTM1      — Combined PWM CH0/CH1, PTB2(pin32)/PTB3(pin31), f_sw=20кГц
 *              deadTime=12 такта≈100нс. Инициализирован в peripherals.c.
 *              MOD = SPWM_MOD_VALUE = 5999 при 120МГц (edge-aligned).
 *  PDB0      — источник DMA-запроса kDmaRequestMux0PDB0 (IDLY=5 тактов).
 *              Запускается от FTM1 CH0 trigger через TRGMUX (input0).
 *  DMA0 CH0  — ping-pong, INTMAJOR → DMA0_IRQHandler каждые 50 мкс.
 *              SADDR: cnv_buf[buf_idx] → DADDR: FTM1->CONTROLS[1].CnV (C1V).
 *              C0V=0 фиксирован, ширина имп. = C1V/MOD. soff=0, doff=0, NBYTES=4.
 *  ADC0 SE1  — PTA1 (pin49), 12 бит, kADC12_ClockSourceAlt0=PLLDIV2=15МГц.
 *              Измеряет ток транзисторов полумоста (шунт + усилитель).
 *              HW-trigger от PDB0 pre-trigger в середине периода.
 *  FAULT_LED — PTC9 (pin35), активно-высокий.
 *  STAR_STOP — PTB0 (pin34), активно-низкий, pull-up.
 *  TAB_CAL   — PTB1 (pin33), активно-низкий, pull-up.
 *
 * ─── NXP RTCESL (libs/GFLIB/libGFLIB.a) ────────────────────────────────────
 *  GFLIB_Sin_FLT_C(angle)           — синус, вход [рад], выход [-1..+1]
 *  GFLIB_CtrlPIpAW_FLT_C()          — параллельный ПИ с anti-windup
 *  GFLIB_CtrlPIpAWInit_FLT_Ci()     — сброс состояния ПИ
 *  GFLIB_Ramp_FLT_Ci(target, *p)    — плавный пуск: fltState 0.0→1.0
 *  GFLIB_RampInit_FLT_Ci(val, *p)   — установка начального fltState
 *
 * ─── Ping-pong связи DMA ────────────────────────────────────────────────────
 *  srcAddr0[0] = (uint32_t)&g_spwm.cnv_buf[0]    — ping-буфер
 *  srcAddr1[0] = (uint32_t)&g_spwm.cnv_buf[1]    — pong-буфер
 *  dstAddr0[0] = dstAddr1[0] = &FTM1->CONTROLS[1].CnV  (C1V, второй фронт)
 *  peripherals.c использует эти символы через extern.
 *  SADDR переключается в DMA0_IRQHandler под buf_idx.
 */

#ifndef FTM_SPWM_DMA_H_
#define FTM_SPWM_DMA_H_

#include <stdint.h>
#include <stdbool.h>
#include "fsl_ftm.h"
#include "fsl_edma.h"
#include "fsl_clock.h"
#include "peripherals.h"
#include "gflib_FP.h"
#include "sine_gen.h"
#include "pi_ctrl.h"

// #define FTM1_PWM_FREQ_HZ (FTM1_CLOCK_SOURCE / (2U * ((FTM1_TIMER_MODULO_VALUE) + 1U))) // → 120000000 / (2 × 3000) = 20000 Гц
/* ───────────────────────────────────────────────────────────────────────────
 * FTM1 — ШИМ-таймер (из peripherals.h)
 *
 * Center-Aligned PWM: период = 2 × MOD тактов
 *   MOD = F_clk / (2 × F_sw) − 1 = 120 000 000 / (2 × 20 000) − 1 = 2999
 *
 * ВНИМАНИЕ: peripherals.h (генерируется Config Tools) содержит
 *   FTM1_TIMER_MODULO_VALUE = 5999  ← значение для EdgeAligned, инструмент
 *   не обновил после смены режима на CenterAligned.
 *   Используйте SPWM_MOD_VALUE и SPWM_FSW_HZ из этого файла.
 * ─────────────────────────────────────────────────────────────────────────── */
#define SPWM_FTM_BASE           FTM1_PERIPHERAL         /* = FTM1           */
#define SPWM_FTM_CHNL           kFTM_Chnl_0

/* MOD для edge-aligned Combined PWM 20 кГц при 120 МГц: F_clk/F_sw − 1 = 5999
 * Combined Mode совместим только с EdgeAligned (CPWMS=0), по RM KE18.        */
#define SPWM_MOD_VALUE          5999U

/* Частота ШИМ [Гц]: F_clk / (MOD + 1) = 120M / 6000 = 20 000              */
#define SPWM_FSW_HZ             (FTM1_CLOCK_SOURCE / (SPWM_MOD_VALUE + 1U))


/* ───────────────────────────────────────────────────────────────────────────
 * ADC0 SE1 — измерение тока транзисторов полумоста
 *
 * Цепь: шунт R_shunt (мОм) + инструментальный усилитель (gain) → PTA1.
 * Формула пересчёта:
 *   I [А] = adc_raw × SPWM_ADC_CURRENT_SCALE
 *   SPWM_ADC_CURRENT_SCALE = Vref / FullScale / R_shunt_Ohm / gain
 *
 * Для текущего железа: R_shunt=22 мОм, gain=10:
 *   SCALE = 3.3 / 4095 / 0.022 / 10 = 0.003663 А/ЕАЦ
 *
 * ВАЖНО: откалибровать под реальный шунт и усилитель!
 * ─────────────────────────────────────────────────────────────────────────── */
#define SPWM_ADC_VREF_V         3.3f
#define SPWM_ADC_FULL_SCALE     4095.0f
#define SPWM_SHUNT_OHM          0.022f
#define SPWM_AMP_GAIN           10.0f
#define SPWM_ADC_CURRENT_SCALE  (SPWM_ADC_VREF_V / SPWM_ADC_FULL_SCALE / SPWM_SHUNT_OHM / SPWM_AMP_GAIN)

/* ───────────────────────────────────────────────────────────────────────────
 * Защитные пороги
 * ─────────────────────────────────────────────────────────────────────────── */
/** Порог токовой защиты (А). Подобрать под I_max транзисторов Q1/Q2. */
#define SPWM_OC_THRESHOLD_A     15.0f

/* ───────────────────────────────────────────────────────────────────────────
 * Плавный пуск
 * 500 шагов × 50 мкс = 25 мс (0.0 → 1.0).
 * float-шаг вычисляется в SPWM_Init(): fltRampUp = 1.0 / SOFTSTART_STEPS.
 * ─────────────────────────────────────────────────────────────────────────── */
#define SPWM_SOFTSTART_STEPS    500U

/* ───────────────────────────────────────────────────────────────────────────
 * Коды аварий
 * ─────────────────────────────────────────────────────────────────────────── */
#define SPWM_FAULT_OVERCURRENT  (1U << 0U)
#define SPWM_FAULT_DMA_ERROR    (1U << 2U)

/* ───────────────────────────────────────────────────────────────────────────
 * КА инвертора
 * ─────────────────────────────────────────────────────────────────────────── */
typedef enum
{
    SPWM_STATE_IDLE      = 0,  /**< Останов: CnV=MOD/2, ток=0, ожидание START */
    SPWM_STATE_SOFTSTART,      /**< Плавный пуск: рампа амплитуды 0→1 (25мс)  */
    SPWM_STATE_RUN,            /**< Рабочий режим: ПИ-регулятор тока активен  */
    SPWM_STATE_FAULT,          /**< Авария: CnV=MOD/2, LED мигает, ожидание   */
} spwm_state_t;

/* ───────────────────────────────────────────────────────────────────────────
 * Главный дескриптор инвертора
 * ─────────────────────────────────────────────────────────────────────────── */
typedef struct
{
    spwm_state_t           state;
    sine_gen_t             sine;
    pi_ctrl_t              pi;

    /** Плавный пуск: GFLIB_RAMP_T_FLT, fltState: 0.0→1.0 за 25 мс */
    GFLIB_RAMP_T_FLT       ramp;

    /** Ping-pong буферы CnV (DMA читает; ISR пишет теневой) */
    uint32_t               cnv_buf[2];
    uint8_t                buf_idx;     /**< Индекс активного буфера (0 или 1) */

    uint16_t               adc_raw_last;/**< Последний валидный raw ADC код */
    float_t                i_meas;      /**< Измеренный ток (А) от ADC0 SE1    */
    float_t                i_ref;       /**< Задание тока RMS (А)              */
    uint32_t               fault_flags; /**< Маска активных аварий             */
    uint32_t               isr_count;   /**< Счётчик вызовов ISR (сбрасывается Monitor) */
    uint8_t                mon_partial_window; /**< 1: окно Monitor содержит переход состояния */
} spwm_t;

/* ───────────────────────────────────────────────────────────────────────────
 * Публичный API
 * ─────────────────────────────────────────────────────────────────────────── */
#ifdef __cplusplus
extern "C" {
#endif

/** Инициализация. Вызывать ПОСЛЕ BOARD_InitBootPeripherals(). Без старта PWM/DMA. */
void    SPWM_Init(spwm_t *spwm, sine_mode_t mode);

/** Запуск инвертора (arm DMA/PDB/FTM + переход IDLE/FAULT→SOFTSTART). */
void    SPWM_Start(spwm_t *spwm);

/** Останов (переход в IDLE, CnV=50%). */
void    SPWM_Stop(spwm_t *spwm);

/** Управляющий алгоритм. Вызывается из DMA0_IRQHandler каждые 50 мкс. */
void    SPWM_DMA_ISR(spwm_t *spwm, uint16_t adc_raw);

/** Аварийное отключение с установкой fault_flags. */
void    SPWM_FaultHandler(spwm_t *spwm, uint32_t fault_mask);

/** Возвращает true если state == RUN. */
bool    SPWM_IsRunning(const spwm_t *spwm);   /* true только в STATE_RUN          */
bool    SPWM_IsActive(const spwm_t *spwm);    /* true в SOFTSTART и RUN           */

/** Переключение режима синусоиды TABLE/CALC без потери фазы. */
void    SPWM_SetSineMode(spwm_t *spwm, sine_mode_t mode);

/** Возвращает указатель на синглтон g_spwm. */
spwm_t *SPWM_GetInstance(void);

#ifdef __cplusplus
}
#endif

#endif /* FTM_SPWM_DMA_H_ */
