/*
 * ftm_spwm_dma.c
 * SPWM-движок: eDMA ping-pong, КА инвертора, ISR.
 *
 * ─── Изменения: FTM0 удалён полностью ──────────────────────────────────────
 *
 *  БЫЛО:  SPWM_FTM0_TriggerInit() инициализировал FTM0 как генератор
 *         DMA-запроса kDmaRequestMux0FTM0Channel0.
 *
 *  СТАЛО: PDB0 overflow (PDBIF, IDLY=5 тактов) генерирует kDmaRequestMux0PDB0.
 *         SPWM_FTM0_TriggerInit() удалена.
 *         kCLOCK_Ftm0, FTM_Init(FTM0,...), FTM_StartTimer(FTM0,...) — удалены.
 *
 *  SPWM_Init():
 *    БЫЛО:  SPWM_FTM0_TriggerInit(mod) перед SPWM_DMA_ReconfTCD()
 *    СТАЛО: вызов удалён, всё остальное без изменений
 *
 * ─── Полная временна́я диаграмма (один период 50 мкс) ──────────────────────
 *
 *   t = 0        FTM1: CNT = 0 (переполнение MOD→0, MOD=2999)
 *                → kFTM_InitTrigger → TRGMUX → PDB0 trigger (reset счётчика)
 *
 *   t ≈ 333 нс   PDB0: CNT достигает IDLY=5 тактов bus (15 МГц)
 *                → kDmaRequestMux0PDB0 → eDMA CH0
 *                → cnv_buf[buf_idx] скопирован в FTM1->CONTROLS[0].CnV
 *                → DMA0_IRQHandler:
 *                     читает ADC0->R[0]  (ток из ПРЕДЫДУЩЕГО периода)
 *                     вычисляет новый CnV (SPWM_DMA_ISR)
 *                     FTM_SetSoftwareTrigger (LDOK) — FTM1 загрузит CnV
 *
 *   t = 25 мкс   PDB0: CNT достигает pretrigger delay=375 тактов (15 МГц)
 *                → ADC0 SE1 старт преобразования
 *
 *   t ≈ 25.9 мкс ADC0: COCO=1, R[0] содержит ток этого периода
 *
 *   t = 50 мкс   FTM1: следующее переполнение — цикл повторяется
 */

#include "ftm_spwm_dma.h"
#include "fsl_adc12.h"
#include "fsl_gpio.h"
#include "fsl_pdb.h"
#include "pin_mux.h"
#include <string.h>

/* ═══════════════════════════════════════════════════════════════════════
 * Синглтон и ping-pong адреса
 * ═══════════════════════════════════════════════════════════════════════ */
static spwm_t g_spwm;
static bool   g_spwm_runtime_armed;

uint32_t srcAddr0[4];
uint32_t srcAddr1[4];
uint32_t dstAddr0[4];
uint32_t dstAddr1[4];

/* ═══════════════════════════════════════════════════════════════════════
 * Утилиты
 * ═══════════════════════════════════════════════════════════════════════ */
static inline int32_t i32_clamp(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline float_t adc_raw_to_amps(uint16_t raw)
{
    return (float_t)raw * SPWM_ADC_CURRENT_SCALE;
}

/* ═══════════════════════════════════════════════════════════════════════
 * Реконфигурация TCD eDMA CH0
 * ═══════════════════════════════════════════════════════════════════════ */
static void SPWM_DMA_ReconfTCD(spwm_t *spwm)
{
    /*
     * Destination = CONTROLS[1].CnV (C1V = второй фронт).
     * CONTROLS[0].CnV (C0V = первый фронт) = 0 — зафиксирован FTM_SetupPwmMode.
     * DMA изменяет только C1V → ширина импульса = C1V − 0 = sine_value.
        * В текущей конфигурации CPWMS=0 (edge-aligned combined mode KE18):
        * выход HIGH от CNT=0 до CNT=C1V → скважность = C1V/(MOD+1).
     */
    uint32_t cnv_reg = (uint32_t)&(FTM1_PERIPHERAL->CONTROLS[1].CnV);

    srcAddr0[0] = (uint32_t)&spwm->cnv_buf[0];
    srcAddr1[0] = (uint32_t)&spwm->cnv_buf[1];
    dstAddr0[0] = cnv_reg;
    dstAddr1[0] = cnv_reg;

    EDMA_AbortTransfer(&DMA_CH0_Handle);

    DMA0->TCD[DMA_CH0_DMA_CHANNEL].SADDR         = (uint32_t)&spwm->cnv_buf[0];
    DMA0->TCD[DMA_CH0_DMA_CHANNEL].SOFF          = 0;
    DMA0->TCD[DMA_CH0_DMA_CHANNEL].ATTR          = DMA_ATTR_SSIZE(2U) | DMA_ATTR_DSIZE(2U);
    DMA0->TCD[DMA_CH0_DMA_CHANNEL].NBYTES_MLNO   = sizeof(uint32_t);
    DMA0->TCD[DMA_CH0_DMA_CHANNEL].SLAST         = 0;
    DMA0->TCD[DMA_CH0_DMA_CHANNEL].DADDR         = cnv_reg;
    DMA0->TCD[DMA_CH0_DMA_CHANNEL].DOFF          = 0;
    DMA0->TCD[DMA_CH0_DMA_CHANNEL].CITER_ELINKNO = 1U;
    DMA0->TCD[DMA_CH0_DMA_CHANNEL].DLAST_SGA     = 0;
    DMA0->TCD[DMA_CH0_DMA_CHANNEL].BITER_ELINKNO = 1U;
    DMA0->TCD[DMA_CH0_DMA_CHANNEL].CSR           = DMA_CSR_INTMAJOR_MASK;

    EDMA_StartTransfer(&DMA_CH0_Handle);
}

static void SPWM_RuntimeArm(spwm_t *spwm)
{
    if (g_spwm_runtime_armed)
    {
        return;
    }

    SPWM_DMA_ReconfTCD(spwm);

    /* Ensure DMA uses the currently active ping-pong buffer index. */
    DMA0->TCD[DMA_CH0_DMA_CHANNEL].SADDR = (uint32_t)&spwm->cnv_buf[spwm->buf_idx];

    EDMA_ClearChannelStatusFlags(DMA0,
                                 DMA_CH0_DMA_CHANNEL,
                                 kEDMA_DoneFlag | kEDMA_InterruptFlag | kEDMA_ErrorFlag);
    PDB_ClearStatusFlags(PDB0, kPDB_DelayEventFlag);

    NVIC_ClearPendingIRQ(DMA0_IRQn);
    NVIC_ClearPendingIRQ(DMA_Error_IRQn);
    EnableIRQ(DMA0_IRQn);
    EnableIRQ(DMA_Error_IRQn);

    /* Надёжный путь измерения: software-trigger ADC из DMA ISR (20 кГц).
     * Так измерение не зависит от PDB/HW-trigger цепочки и всегда привязано к ISR. */
    ADC12_EnableHardwareTrigger(ADC0_PERIPHERAL, false);
    g_spwm.adc_raw_last = 0U;
    ADC12_SetChannelConfig(ADC0_PERIPHERAL, 0U, &ADC0_channelsConfig[0]); /* Start first conversion. */

    /* FTM1_CH0 match at C0V=0 generates one DMA request at the start of each PWM period.
     * KE18 requires both CHIE and DMA bits set in C0SC for channel DMA requests. */
    SPWM_FTM_BASE->CONTROLS[0].CnSC |= FTM_CnSC_CHIE_MASK;
    FTM_EnableDmaTransfer(SPWM_FTM_BASE, SPWM_FTM_CHNL, true);

    EDMA_EnableChannelRequest(DMA0, DMA_CH0_DMA_CHANNEL);
    PDB_Enable(PDB0, true);
    PDB_DoLoadValues(PDB0);   /* LDOK: загружает MOD=750, IDLY=5, DLY0=375 — только при PDBEN=1 */

    /* Предзагрузить C1V актуальным значением перед запуском FTM.
     * FTM_SetupPwmMode записал в буфер C1V=dutyValue=0. Если FTM стартует
     * с C1V=0 и C0V=0, на первом такте SET и CLEAR происходят одновременно
     * → всплеск на выходе. Пишем начальное значение (MOD/2) в буфер CnV,
     * которое LDOK (ниже) загрузит в рабочий регистр до старта таймера. */
    FTM1_PERIPHERAL->CONTROLS[1].CnV = spwm->cnv_buf[spwm->buf_idx];

    FTM_SetSoftwareTrigger(SPWM_FTM_BASE, true);  /* LDOK: C1V_working = cnv_buf[0] = MOD/2 */
    FTM_StartTimer(SPWM_FTM_BASE, kFTM_SystemClock);

    g_spwm_runtime_armed = true;
}

static void SPWM_RuntimeDisarm(void)
{
    EDMA_DisableChannelRequest(DMA0, DMA_CH0_DMA_CHANNEL);
    FTM_EnableDmaTransfer(SPWM_FTM_BASE, SPWM_FTM_CHNL, false);
    SPWM_FTM_BASE->CONTROLS[0].CnSC &= ~FTM_CnSC_CHIE_MASK;
    DisableIRQ(DMA0_IRQn);
    DisableIRQ(DMA_Error_IRQn);
    NVIC_ClearPendingIRQ(DMA0_IRQn);
    NVIC_ClearPendingIRQ(DMA_Error_IRQn);

    FTM_StopTimer(SPWM_FTM_BASE);
    PDB_Enable(PDB0, false);

    EDMA_ClearChannelStatusFlags(DMA0,
                                 DMA_CH0_DMA_CHANNEL,
                                 kEDMA_DoneFlag | kEDMA_InterruptFlag | kEDMA_ErrorFlag);
    PDB_ClearStatusFlags(PDB0, kPDB_DelayEventFlag);

    g_spwm_runtime_armed = false;
}

/* ═══════════════════════════════════════════════════════════════════════
 * SPWM_Init
 * Изменение: SPWM_FTM0_TriggerInit() удалён
 * ═══════════════════════════════════════════════════════════════════════ */
void SPWM_Init(spwm_t *spwm, sine_mode_t mode)
{
    uint32_t mod;

    memset(spwm, 0, sizeof(*spwm));
    spwm->state = SPWM_STATE_IDLE;
    spwm->i_ref = 5.0f;

    mod = SPWM_MOD_VALUE;

    spwm->cnv_buf[0] = mod / 2U;
    spwm->cnv_buf[1] = mod / 2U;
    spwm->buf_idx    = 0U;

    SineGen_Init(&spwm->sine, mod, mode);
    PI_Init(&spwm->pi);

    spwm->ramp.fltRampUp   = 1.0f / (float_t)SPWM_SOFTSTART_STEPS;
    spwm->ramp.fltRampDown = 1.0f / (float_t)SPWM_SOFTSTART_STEPS;
    GFLIB_RampInit_FLT_Ci(0.0f, &spwm->ramp);

    /* Движок не запускаем здесь: старт только по кнопке в SPWM_Start(). */
    SPWM_RuntimeDisarm();
    spwm->i_meas = 0.0f;
    spwm->isr_count = 0U;
    spwm->mon_partial_window = 0U;
}

/* ═══════════════════════════════════════════════════════════════════════
 * SPWM_Start / Stop / FaultHandler — без изменений
 * ═══════════════════════════════════════════════════════════════════════ */
void SPWM_Start(spwm_t *spwm)
{
    if (spwm->state == SPWM_STATE_FAULT) return;

    if (!g_spwm_runtime_armed)
    {
        SPWM_RuntimeArm(spwm);
    }

    SineGen_Reset(&spwm->sine);
    PI_SetEnabled(&spwm->pi, false);
    PI_Reset(&spwm->pi);
    GFLIB_RampInit_FLT_Ci(0.0f, &spwm->ramp);
    spwm->isr_count = 0U;
    spwm->mon_partial_window = 1U;
    spwm->state = SPWM_STATE_SOFTSTART;
    GPIO_PinWrite(BOARD_INITPINS_FAULT_LED_GPIO, BOARD_INITPINS_FAULT_LED_PIN, 0U);
}

void SPWM_Stop(spwm_t *spwm)
{
    uint32_t mod = SPWM_MOD_VALUE;

    PI_SetEnabled(&spwm->pi, false);
    PI_Reset(&spwm->pi);
    GFLIB_RampInit_FLT_Ci(0.0f, &spwm->ramp);
    spwm->cnv_buf[0] = mod / 2U;
    spwm->cnv_buf[1] = mod / 2U;
    FTM_SetSoftwareTrigger(SPWM_FTM_BASE, true);

    if (g_spwm_runtime_armed)
    {
        SPWM_RuntimeDisarm();
    }

    spwm->i_meas = 0.0f;
    spwm->isr_count = 0U;
    spwm->mon_partial_window = 1U;
    spwm->state = SPWM_STATE_IDLE;
    GPIO_PinWrite(BOARD_INITPINS_FAULT_LED_GPIO, BOARD_INITPINS_FAULT_LED_PIN, 0U);
}

void SPWM_FaultHandler(spwm_t *spwm, uint32_t fault_mask)
{
    uint32_t mod = SPWM_MOD_VALUE;

    spwm->fault_flags |= fault_mask;
    spwm->state        = SPWM_STATE_FAULT;
    spwm->cnv_buf[0]   = mod / 2U;
    spwm->cnv_buf[1]   = mod / 2U;
    FTM_SetSoftwareTrigger(SPWM_FTM_BASE, true);

    if (g_spwm_runtime_armed)
    {
        SPWM_RuntimeDisarm();
    }

    PI_SetEnabled(&spwm->pi, false);
    GFLIB_RampInit_FLT_Ci(0.0f, &spwm->ramp);
    spwm->isr_count = 0U;
    spwm->mon_partial_window = 1U;
    GPIO_PinWrite(BOARD_INITPINS_FAULT_LED_GPIO, BOARD_INITPINS_FAULT_LED_PIN, 1U);
}

/* ═══════════════════════════════════════════════════════════════════════
 * SPWM_DMA_ISR — управляющий алгоритм
 * ═══════════════════════════════════════════════════════════════════════ */
void SPWM_DMA_ISR(spwm_t *spwm, uint16_t adc_raw)
{
    uint32_t mod      = SPWM_MOD_VALUE;
    uint8_t  next_idx = (spwm->buf_idx == 0U) ? 1U : 0U;
    uint32_t sine_cnv;
    int32_t  cnv_next;

    spwm->isr_count++;
    spwm->i_meas = adc_raw_to_amps(adc_raw);

    switch (spwm->state)
    {
        case SPWM_STATE_SOFTSTART:
        {
            float_t ramp_val = GFLIB_Ramp_FLT_Ci(1.0f, &spwm->ramp);
            if (spwm->ramp.fltState >= 1.0f)
            {
                spwm->state = SPWM_STATE_RUN;
                PI_SetEnabled(&spwm->pi, true);
            }
            sine_cnv = SineGen_Next(&spwm->sine);
            {
                uint32_t centre = mod / 2U;
                cnv_next = (int32_t)centre
                         + (int32_t)(ramp_val * (float_t)((int32_t)sine_cnv
                                                          - (int32_t)centre));
            }
            cnv_next = i32_clamp(cnv_next, 0, (int32_t)mod);
            break;
        }

        case SPWM_STATE_RUN:
        {
            if (spwm->i_meas > SPWM_OC_THRESHOLD_A)
            {
                SPWM_FaultHandler(spwm, SPWM_FAULT_OVERCURRENT);
                return;
            }
            sine_cnv = SineGen_Next(&spwm->sine);
            {
                float_t i_peak     = spwm->i_ref * 1.41421356f;
                float_t prev_phase = spwm->sine.phase_acc - SINE_PHASE_INC;
                float_t i_ref_inst = i_peak * GFLIB_Sin_FLT_C(prev_phase);
                float_t error      = i_ref_inst - spwm->i_meas;
                float_t pi_out     = PI_Step(&spwm->pi, error);
                cnv_next = (int32_t)sine_cnv + (int32_t)pi_out;
            }
            cnv_next = i32_clamp(cnv_next, 0, (int32_t)mod);
            break;
        }

        case SPWM_STATE_IDLE:
        case SPWM_STATE_FAULT:
        default:
            cnv_next = (int32_t)(mod / 2U);
            break;
    }

    spwm->cnv_buf[next_idx] = (uint32_t)cnv_next;
    spwm->buf_idx            = next_idx;
    FTM_SetSoftwareTrigger(SPWM_FTM_BASE, true);
}

/* Вспомогательные API */
bool    SPWM_IsRunning(const spwm_t *spwm) { return (spwm->state == SPWM_STATE_RUN); }
bool    SPWM_IsActive(const spwm_t *spwm)  { return (spwm->state == SPWM_STATE_RUN ||
                                                      spwm->state == SPWM_STATE_SOFTSTART); }
void    SPWM_SetSineMode(spwm_t *spwm, sine_mode_t mode) { SineGen_SetMode(&spwm->sine, mode); }
spwm_t *SPWM_GetInstance(void) { return &g_spwm; }

/* ═══════════════════════════════════════════════════════════════════════
 * DMA0_IRQHandler
 *
 * Вызывается через ~333 нс после FTM1 init-trigger (PDB0 IDLY=5 тактов bus 15 МГц).
 * PDBIF сбрасывается вручную — иначе DMA-запрос не повторится.
 * ═══════════════════════════════════════════════════════════════════════ */
void DMA0_IRQHandler(void)
{
    /* 1. Сбросить флаги eDMA */
    EDMA_ClearChannelStatusFlags(DMA0, DMA_CH0_DMA_CHANNEL,
                                 kEDMA_DoneFlag | kEDMA_InterruptFlag);

    /*
     * 2. Сбросить PDBIF в PDB0->SC.
     *    Без сброса PDB0 не сгенерирует следующий DMA-запрос.
     *    PDB_ClearStatusFlags(PDB0, kPDB_DelayEventFlag) эквивалентно
     *    записи SC &= ~PDBIF — SDK inline функция.
     */
    PDB_ClearStatusFlags(PDB0, kPDB_DelayEventFlag);

    /* 3. Переключить SADDR на активный ping-pong буфер */
    DMA0->TCD[DMA_CH0_DMA_CHANNEL].SADDR =
        (uint32_t)&g_spwm.cnv_buf[g_spwm.buf_idx];

    /*
     * 4. Читаем ADC только когда COCO=1; иначе используем последний валидный код.
     *    В конце ISR запускаем следующую software-conversion (SC1A write).
     */
    uint16_t adc_raw = g_spwm.adc_raw_last;
    if ((ADC0_PERIPHERAL->SC1[0] & ADC_SC1_COCO_MASK) != 0U)
    {
        adc_raw = (uint16_t)(ADC0_PERIPHERAL->R[0] & 0x0FFFU);
        g_spwm.adc_raw_last = adc_raw;
    }

    /* Start next conversion for the next ISR period. */
    ADC12_SetChannelConfig(ADC0_PERIPHERAL, 0U, &ADC0_channelsConfig[0]);

    /* 5. Управляющий алгоритм */
    SPWM_DMA_ISR(&g_spwm, adc_raw);
}

void DMA_Error_IRQHandler(void)
{
    (void)EDMA_GetErrorStatusFlags(DMA0);
    EDMA_DisableChannelRequest(DMA0, DMA_CH0_DMA_CHANNEL);
    EDMA_ClearChannelStatusFlags(DMA0,
                                 DMA_CH0_DMA_CHANNEL,
                                 kEDMA_DoneFlag | kEDMA_InterruptFlag | kEDMA_ErrorFlag);
    PDB_ClearStatusFlags(PDB0, kPDB_DelayEventFlag);
    SPWM_FaultHandler(&g_spwm, SPWM_FAULT_DMA_ERROR);
}

void DMA_callback(edma_handle_t *handle, void *userData,
                  bool transferDone, uint32_t tcds)
{
    (void)handle; (void)userData; (void)transferDone; (void)tcds;
}
