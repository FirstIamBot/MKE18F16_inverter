/*
 * MKE18F16_inverter.c
 * Точка входа приложения: инициализация, FreeRTOS задачи.
 *
 * Инвертор 115 В / 400 Гц (MIL-STD-704F)
 * МК: NXP MKE18F512VLH16, Cortex-M4F @ 120 МГц, FPU.
 *
 * Последовательность инициализации:
 *   1. BOARD_InitBootPins()        — pin_mux.c
 *   2. BOARD_InitBootClocks()      — clock_config.c: SPLL 120МГц, PLLDIV2 15МГц
 *   3. BOARD_InitBootPeripherals() — peripherals.c: FTM1, ADC0, DMA0 CH0
 *   4. SPWM_Init()                 — TCD, sine table, PI, ramp
 *   5. FreeRTOS задачи + vTaskStartScheduler()
 */

#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "FreeRTOS.h"
#include "task.h"

#include "ftm_spwm_dma.h"
#include "app_buttons_led.h"
#include "board_init_ext.h"

/* ═══════════════════════════════════════════════════════════════════════
 * Задача мониторинга (1 раз в секунду)
 * Выводит состояние КА, ток, счётчик ISR и маску аварий.
 * ═══════════════════════════════════════════════════════════════════════ */
static void Monitor_Task(void *pvParameters)
{
    spwm_t *spwm = (spwm_t *)pvParameters;

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000U));

        const char *state_str;
        switch (spwm->state)
        {
            case SPWM_STATE_IDLE:       state_str = "IDLE";      break;
            case SPWM_STATE_SOFTSTART:  state_str = "SOFTSTART"; break;
            case SPWM_STATE_RUN:        state_str = "RUN";       break;
            case SPWM_STATE_FAULT:      state_str = "FAULT";     break;
            default:                    state_str = "UNKNOWN";   break;
        }

        PRINTF("[MON] State=%-10s I=%.3f A  ISR/s=%-6lu  Faults=0x%02lX  Mode=%s\r\n",
               state_str,
               (double)spwm->i_meas,
               (unsigned long)spwm->isr_count,
               (unsigned long)spwm->fault_flags,
               (spwm->sine.mode == SINE_MODE_TABLE) ? "TABLE" : "CALC");

        spwm->isr_count = 0U;
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 * main
 * ═══════════════════════════════════════════════════════════════════════ */
int main(void)
{
    /* ── 1. Инициализация платы ── */
    BOARD_InitBootPins();
    /*
     * PTB0 (pin34) → GPIO INPUT pull-up  (STAR_STOP, активно-низкий)
     * PTB1 (pin33) → GPIO INPUT pull-up  (TAB_CAL,   активно-низкий)
     * PTB2 (pin32) → FTM1_CH0            (Hi-side Q1 gate)
     * PTB3 (pin31) → FTM1_CH1            (Lo-side Q2 gate)
     * PTA1 (pin49) → ADC0_SE1            (ток полумоста, аналог)
     * PTC9 (pin35) → GPIO OUTPUT         (FAULT_LED, активно-высокий)
     */

    BOARD_InitBootClocks();
    /*
     * SPLL: FIRC(48МГц)/4 × 20 / 2 = 120 МГц → CoreSysClk
     * PLLDIV2 = 15 МГц → ADC0 тактирование
     */

    BOARD_HardwareInit();
    /*
     * Вместо BOARD_InitBootPeripherals() — source/inverter/board_init_ext.c:
     *   1. CLOCK_EnableClock: DMA0/DMAMUX/FTM1/ADC0/PDB0
     *   2. BOARD_InitPeripherals() — generated код
     *   3. Патч: FTM1 MOD=2999, TRGMUX FTM1→PDB0, PDB MOD=750,
     *            IDLY=5, ADC pre-trigger delay=375
     */

#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    BOARD_InitDebugConsole();
#endif

    PRINTF("\r\n+==================================================+\r\n");
    PRINTF(  "|  Inverter 115 V / 400 Hz  (MIL-STD-704F)         |\r\n");
    PRINTF(  "|  MCU: MKE18F512VLH16 @ 120 MHz (Cortex-M4F FPU ) |\r\n");
    PRINTF(  "|  PWM: FTM1 Combined PWM 20 kHz (PTB2/PTB3)       |\r\n");
    PRINTF(  "|  DMA: -> DMA0 -> FTM1 CnV (ping-pong)   |\r\n");
    PRINTF(  "|  ADC: ADC0 SE1 (PTA1) - current sense @ 15 MHz   |\r\n");
    PRINTF(  "+==================================================+\r\n\r\n");
    PRINTF("Controls:\r\n");
    PRINTF("  PTB0 (pin34) - START / STOP / FAULT RESET\r\n");
    PRINTF("  PTB1 (pin33) - TABLE / CALC sine mode\r\n");
    PRINTF("  PTC9 (pin35) - FAULT LED (active high)\r\n\r\n");

    /* ── 2. SPWM-движок (вызывать ПОСЛЕ BOARD_InitBootPeripherals) ── */
    spwm_t *spwm = SPWM_GetInstance();
    SPWM_Init(spwm, SINE_MODE_TABLE);
    /*
     * SPWM_Init():
     *   - SineGen_Init(): 50 × GFLIB_Sin_FLT_C() → cnv_table[50]
     *   - PI_Init(): GFLIB_CtrlPIpAWInit_FLT_Ci(), Kp=2.0, IGain=800.0
     *   - Ramp: fltRampUp=0.002, fltState=0.0
        *   - Runtime DMA/PDB/FTM пока не запускается
        *     (запуск только в SPWM_Start() по кнопке PTB0)
     */

    PRINTF("SPWM OK. MOD=%lu  fsw=%u Hz  I_ref=%.1f A\r\n",
           (unsigned long)SPWM_MOD_VALUE,
           (unsigned)SPWM_FSW_HZ,
           (double)spwm->i_ref);
    PRINTF("Нажми PTB0 (pin34 синий) для стартаt...\r\n\r\n");

    /* ── 3. FreeRTOS задачи ── */
    AppButtonsLed_CreateTask(spwm);
    /*
     * BtnLed task: prio=2, 512 слов стека, период 5 мс.
     * Опрос PTB0/PTB1 с дребезгозащитой 20 мс.
     * Мигание FAULT LED при state=FAULT (500/500 мс).
     */

    xTaskCreate(Monitor_Task, "Monitor", 512U, spwm, 1U, NULL);

    /* ── 4. Запуск планировщика ── */
    vTaskStartScheduler();

    /* Сюда не попадём — планировщик не вернётся */
    for (;;)
    {
        __WFI();
    }

    return 0;
}
