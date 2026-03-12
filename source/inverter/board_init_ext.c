/*
 * board_init_ext.c
 *
 * Расширение инициализации. НЕ генерируется Config Tools.
 *
 * ─── Архитектура ────────────────────────────────────────────────────────────
 *
 *   Config Tools генерирует board/peripherals.c для FTM1, ADC0, PDB0.
 *   DMA instance в .mex ОТКЛЮЧЁН (enabled=false) — чтобы Config Tools
 *   не генерировал DMA_init() с malloc/PrepareTransfer/SubmitTransfer.
 *
 *   Вся инициализация eDMA0 выполняется здесь, в prv_DMA_Init():
 *     - DMAMUX_Init / DMAMUX_SetSource / DMAMUX_EnableChannel
 *     - EDMA_Init / EDMA_CreateHandle / EDMA_SetCallback
 *     - EDMA_EnableChannelInterrupts + NVIC приоритеты
 *   TCD не настраивается здесь — SPWM_DMA_ReconfTCD() запишет его
 *   напрямую через DMA0->TCD[] после SPWM_Init().
 *
 * ─── Почему prv_ClockEnable() отсутствует ───────────────────────────────────
 *
 *   SDK-драйверы (EDMA_Init, DMAMUX_Init, FTM_Init, ADC12_Init, PDB_Init)
 *   сами вызывают CLOCK_EnableClock() внутри Init.
 *   Ручной CLOCK_EnableClock(kCLOCK_Ftm1) до FTM_Init() вызывает:
 *     CGC=1 при PCS=0 → INUSE=1 → FTM_Init() → CLOCK_SetIpSrc()
 *     → assert(INUSE==0) → HardFault.
 *
 * ─── Тактирование ───────────────────────────────────────────────────────────
 *   Core / FTM1   120 МГц   SPLL PLLDIV1  (clock_config.c)
 *   Bus / PDB0     15 МГц   SPLL / DIVBUS=8
 *   ADC0           15 МГц   SPLL PLLDIV2  (clock_config.c)
 *   DMA0 / DMAMUX  bus clock
 *
 * ─── Временна́я диаграмма (50 мкс @ bus=15 МГц) ─────────────────────────────
 *   t = 0       FTM1 CNT=0 → kFTM_InitTrigger → TRGMUX → PDB0 reset
 *   t ≈ 333 нс  PDB0 IDLY=5 → DMA0 CH0 → FTM1->CnV обновлён
 *   t = 25 мкс  PDB0 pretrigger delay=375 → ADC0 SE1 старт
 *   t = 50 мкс  FTM1 следующее переполнение
 */

#include "board_init_ext.h"
#include "peripherals.h"
#include "fsl_edma.h"
#include "fsl_dmamux.h"
#include "fsl_trgmux.h"
#include "fsl_pdb.h"
#include "fsl_ftm.h"

/* ── Константы ──────────────────────────────────────────────────────────── */
#define BOARD_FTM1_MOD      2999U  /* CenterAligned 20кГц@120МГц: 120M/(2×20k)−1  */
#define BOARD_PDB_MOD        750U  /* 50мкс × 15МГц = 750 тактов bus              */
#define BOARD_PDB_IDLY         5U  /* 5 × 66.67нс = 333нс                         */
#define BOARD_PDB_PRETRIG    375U  /* 25мкс × 15МГц = 375 тактов (центр периода)  */

/* DMA0_IRQn: обязан быть >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY=2
 * Приоритет 0 не маскируется FreeRTOS BASEPRI → разрушает планировщик */
#define BOARD_DMA0_IRQ_PRIORITY       2U
/* DMA_Error_IRQn: минимальный приоритет, не использует FreeRTOS API */
#define BOARD_DMA_ERROR_IRQ_PRIORITY  15U

/* ── eDMA конфигурация ──────────────────────────────────────────────────── */
static const edma_config_t s_edmaConfig = {
    .enableContinuousLinkMode    = false,
    .enableHaltOnError           = true,
    .enableRoundRobinArbitration = false,
    .enableDebugMode             = false,
};

/* Глобальное определение — extern объявлен в board_init_ext.h и ftm_spwm_dma.h */
edma_handle_t DMA_CH0_Handle;

/* ── Шаг 1: инициализация eDMA / DMAMUX ────────────────────────────────── */
static void prv_DMA_Init(void)
{
    /* EDMA_Init включает kCLOCK_Dma0 самостоятельно */
    EDMA_Init(DMA0, &s_edmaConfig);

    /* DMAMUX_Init включает kCLOCK_Dmamux0 самостоятельно */
    DMAMUX_Init(DMAMUX);
    DMAMUX_SetSource(DMAMUX, 0U, (uint8_t)(kDmaRequestMux0PDB0 & 0xFFU));
    DMAMUX_EnableChannel(DMAMUX, 0U);

    /* Handle — нужен для EDMA_AbortTransfer/StartTransfer в SPWM_DMA_ReconfTCD */
    EDMA_CreateHandle(&DMA_CH0_Handle, DMA0, 0U);
    EDMA_SetCallback(&DMA_CH0_Handle, DMA_callback, NULL);

    /* Major Complete + Error interrupts на уровне eDMA-канала */
    EDMA_EnableChannelInterrupts(DMA0, 0U,
        (uint32_t)kEDMA_MajorInterruptEnable |
        (uint32_t)kEDMA_ErrorInterruptEnable);

    /* NVIC: DMA0_IRQn — рабочий ISR (20кГц), должен маскироваться FreeRTOS */
    NVIC_SetPriority(DMA0_IRQn, BOARD_DMA0_IRQ_PRIORITY);
    /* DMA_Error_IRQn включается в SPWM_RuntimeArm() вместе с DMA0_IRQn */
    NVIC_SetPriority(DMA_Error_IRQn, BOARD_DMA_ERROR_IRQ_PRIORITY);
    EnableIRQ(DMA_Error_IRQn);

    /* TCD не настраивается здесь:
     * srcAddr/dstAddr известны только после SPWM_Init().
     * SPWM_DMA_ReconfTCD() запишет TCD напрямую через DMA0->TCD[0]. */
}

/* ── Шаг 2: патч значений после BOARD_InitPeripherals() ────────────────── */
static void prv_PatchPeripherals(void)
{
    status_t st;

    /* FTM1 MOD: Config Tools генерирует для EdgeAligned (5999).
     * CenterAligned 20кГц @ 120МГц: MOD = 120M/(2×20k) − 1 = 2999 */
    FTM_SetTimerPeriod(FTM1_PERIPHERAL, BOARD_FTM1_MOD);

    /* CPWMS: kFTM_CombinedPwm не устанавливает CPWMS автоматически.
     * Без CPWMS=1 FTM1 считает только вверх (EdgeAligned, 40кГц).
     * Устанавливаем вручную для up-down счёта (CenterAligned, 20кГц). */
    FTM1_PERIPHERAL->SC |= FTM_SC_CPWMS_MASK;

    /* TRGMUX: FTM1 InitTrigger → PDB0 trigger input 0
     * Config Tools не настраивает TRGMUX вообще */
    st = TRGMUX_SetTriggerSource(TRGMUX0,
                                 (uint32_t)kTRGMUX_Pdb0,
                                 kTRGMUX_TriggerInput0,
                                 (uint32_t)kTRGMUX_SourceFtm1);
    assert(st == kStatus_Success);

    /* PDB0 MOD: 50мкс × 15МГц = 750 тактов
     * Config Tools генерирует 5000 (по умолчанию) */
    PDB_SetModulusValue(PDB0_PERIPHERAL, BOARD_PDB_MOD);

    /* DMAEN: при срабатывании IDLY генерировать DMA request (а не прерывание).
     * pdb_config_t не имеет поля enableDMA — устанавливается отдельно. */
    PDB_EnableDMA(PDB0_PERIPHERAL, true);

    /* PDB0 IDLY: DMA-запрос через 5 тактов bus = 333нс */
    PDB0->IDLY = BOARD_PDB_IDLY;

    /* PDB0 ADC0 pre-trigger 0: HW trigger в центре периода (25мкс)
     * enablePreTriggerMask bit0=1 : pretrigger 0 enable
     * enableOutputMask     bit0=1 : HW trigger через PDB delay
     * enableBackToBackOperationMask=0: независимый режим            */
    pdb_adc_pretrigger_config_t pretrig = {
        .enablePreTriggerMask          = (1U << 0U),
        .enableOutputMask              = (1U << 0U),
        .enableBackToBackOperationMask = 0U,
    };
    PDB_SetADCPreTriggerConfig(PDB0_PERIPHERAL, kPDB_ADCTriggerChannel0, &pretrig);
    PDB_SetADCPreTriggerDelayValue(PDB0_PERIPHERAL, kPDB_ADCTriggerChannel0,
                                   kPDB_ADCPreTrigger0, BOARD_PDB_PRETRIG);

    /* PDB_DoLoadValues() НЕ вызывается здесь — PDB ещё выключен (PDBEN=0).
     * На MKE18F16 LDOK защёлкивает значения только при PDBEN=1.
     * PDB_Enable(true) + PDB_DoLoadValues() вызываются в SPWM_RuntimeArm(). */
}

/* ── Публичный API ──────────────────────────────────────────────────────── */
/*
 * Вызывать из main() ВМЕСТО BOARD_InitBootPeripherals():
 *
 *   BOARD_InitBootPins();
 *   BOARD_InitBootClocks();
 *   BOARD_HardwareInit();
 */
void BOARD_HardwareInit(void)
{
    prv_DMA_Init();            /* eDMA0/DMAMUX — вне досягаемости Config Tools  */
    BOARD_InitPeripherals();   /* generated: FTM1, ADC0, PDB0 (DMA отключён)    */
    prv_PatchPeripherals();    /* патч: FTM1 MOD, TRGMUX, PDB MOD/IDLY/pretrig */
}
