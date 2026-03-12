/*
 * app_buttons_led.c
 * Задача FreeRTOS: опрос кнопок с дребезгозащитой и управление FAULT LED.
 */

#include "app_buttons_led.h"
#include "fsl_gpio.h"
#include "pin_mux.h"
#include "fsl_debug_console.h"

/* -----------------------------------------------------------------------
 * Вспомогательная структура дребезгозащиты
 * ----------------------------------------------------------------------- */
typedef struct
{
    uint8_t  cnt;      /**< Счётчик подряд идущих совпадений */
    bool     state;    /**< Текущее стабильное состояние (true = нажата) */
    bool     prev_raw; /**< Предыдущее сырое состояние */
    bool     pressed;  /**< Флаг нажатия (автосброс после чтения) */
} btn_debounce_t;

/* -----------------------------------------------------------------------
 * Контекст задачи
 * ----------------------------------------------------------------------- */
typedef struct
{
    spwm_t         *spwm;
    btn_debounce_t  btn_start_stop;
    uint32_t        led_blink_cnt;
    bool            led_state;
} app_ctx_t;

static app_ctx_t g_ctx;

/* -----------------------------------------------------------------------
 * Вспомогательные функции
 * ----------------------------------------------------------------------- */

/**
 * Обновить состояние одной кнопки (вызывать каждые BTN_POLL_PERIOD_MS).
 * @param btn      Дескриптор дребезгозащиты.
 * @param raw_val  Сырое считанное значение GPIO (1 = кнопка не нажата
 *                 для активно-низкой логики, 0 = нажата).
 */
static void btn_update(btn_debounce_t *btn, uint32_t raw_val)
{
    /* Активно-низкая кнопка: 0 = нажата */
    bool raw = (raw_val == 0U);

    if (raw == btn->prev_raw)
    {
        btn->cnt++;
        if (btn->cnt >= BTN_DEBOUNCE_COUNT)
        {
            /* Стабильное состояние изменилось */
            if (raw && !btn->state)
            {
                btn->pressed = true;  /* Фронт нажатия */
            }
            btn->state = raw;
            btn->cnt   = BTN_DEBOUNCE_COUNT; /* Предотвратить переполнение */
        }
    }
    else
    {
        btn->cnt = 0U;
    }
    btn->prev_raw = raw;
}

/** Считать и сбросить флаг нажатия */
static bool btn_is_pressed(btn_debounce_t *btn)
{
    if (btn->pressed)
    {
        btn->pressed = false;
        return true;
    }
    return false;
}

/* -----------------------------------------------------------------------
 * Задача FreeRTOS
 * ----------------------------------------------------------------------- */

static void AppButtonsLed_Task(void *pvParameters)
{
    app_ctx_t *ctx = (app_ctx_t *)pvParameters;
    TickType_t  xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod  = pdMS_TO_TICKS(BTN_POLL_PERIOD_MS);

    /* Счётчик для мигания LED: тикает каждые BTN_POLL_PERIOD_MS мс */
    uint32_t blink_ticks = FAULT_LED_BLINK_MS / BTN_POLL_PERIOD_MS;

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        /* ---- Опрос кнопки START/STOP ---- */
        uint32_t ss_raw = GPIO_PinRead(BOARD_INITPINS_STAR_STOP_GPIO,
                                        BOARD_INITPINS_STAR_STOP_PIN);
        btn_update(&ctx->btn_start_stop, ss_raw);

        if (btn_is_pressed(&ctx->btn_start_stop))
        {
            if (SPWM_IsActive(ctx->spwm))          /* SOFTSTART или RUN → стоп */
            {
                SPWM_Stop(ctx->spwm);
                PRINTF("STOPPED\r\n");
            }
            else if (ctx->spwm->state != SPWM_STATE_FAULT)
            {
                SPWM_Start(ctx->spwm);
                PRINTF("STARTED\r\n");
            }
            else
            {
                /* Сброс аварии: очищаем флаги и возвращаем в IDLE */
                ctx->spwm->fault_flags = 0U;
                ctx->spwm->state       = SPWM_STATE_IDLE;
                GPIO_PinWrite(BOARD_INITPINS_FAULT_LED_GPIO,
                              BOARD_INITPINS_FAULT_LED_PIN, 0U);
                PRINTF("FAULT CLEARED\r\n");
            }
        }

        /* ---- Мигание FAULT LED при аварии ---- */
        if (ctx->spwm->state == SPWM_STATE_FAULT)
        {
            ctx->led_blink_cnt++;
            if (ctx->led_blink_cnt >= blink_ticks)
            {
                ctx->led_blink_cnt = 0U;
                ctx->led_state = !ctx->led_state;
                GPIO_PinWrite(BOARD_INITPINS_FAULT_LED_GPIO,
                              BOARD_INITPINS_FAULT_LED_PIN,
                              ctx->led_state ? 1U : 0U);
            }
        }
        else
        {
            ctx->led_blink_cnt = 0U;
        }
    }
}

/* -----------------------------------------------------------------------
 * Публичный API
 * ----------------------------------------------------------------------- */

void AppButtonsLed_CreateTask(spwm_t *spwm)
{
    g_ctx.spwm         = spwm;
    g_ctx.led_blink_cnt = 0U;
    g_ctx.led_state     = false;

    xTaskCreate(AppButtonsLed_Task,
                "BtnLed",
                1024U,           /* Размер стека (слова) */
                &g_ctx,
                2U,             /* Приоритет */
                NULL);
}
