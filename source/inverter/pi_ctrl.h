/*
 * pi_ctrl.h
 * Обёртка над GFLIB_CtrlPIpAW_FLT_C() из NXP RTCESL libGFLIB.a.
 *
 * Используемая функция:
 *   GFLIB_CtrlPIpAW_FLT_C(fltInErr, pbStopIntegFlag, psParam)
 *     — Параллельный ПИ-регулятор с anti-windup (stop-integrator flag).
 *     — Тип: float_t → float_t (FPU Cortex-M4F).
 *     — Структура GFLIB_CTRL_PI_P_AW_T_FLT:
 *         fltPGain    — пропорциональный коэффициент Kp
 *         fltIGain    — интегральный коэффициент Ki (уже умножен на Ts внутри!)
 *         fltIAccK_1  — состояние интегратора (шаг k-1)
 *         fltInErrK_1 — ошибка на шаге k-1
 *         fltUpperLim — верхний предел выхода
 *         fltLowerLim — нижний предел выхода
 *         bLimFlag    — флаг насыщения (out → anti-windup)
 *
 *   Anti-windup: при насыщении (bLimFlag=1) интегрирование останавливается
 *   через pbStopIntegFlag = &psParam->bLimFlag. Это стандартная схема
 *   RTCESL для силовых преобразователей.
 *
 * Примечание о Ki:
 *   В GFLIB_CtrlPIpAW fltIGain = Ki_дискретный = Ki_непрерывный * Ts.
 *   При f_sw = 20 кГц, Ts = 50 мкс:
 *     fltIGain = Ki_cont * 50e-6
 *   Для Ki_cont = 16 000 000: fltIGain = 16e6 * 50e-6 = 800.0
 */

#ifndef PI_CTRL_H_
#define PI_CTRL_H_

#include <stdbool.h>

/* NXP RTCESL GFLIB: GFLIB_CtrlPIpAW_FLT_C, GFLIB_CTRL_PI_P_AW_T_FLT */
#include "gflib_FP.h"

/* -----------------------------------------------------------------------
 * Параметры по умолчанию
 * ----------------------------------------------------------------------- */
#define PI_DEFAULT_KP           2.0f
/**
 * fltIGain = Ki_cont * Ts = 16 000 000 * 50e-6 = 800.0
 * Настройте под реальный LC-фильтр (L ≈ 100 мкГн, C ≈ 10 мкФ, f_res ≈ 5 кГц).
 */
#define PI_DEFAULT_IGAIN        800.0f
/** Максимальная коррекция скважности: ±500 тиков ≈ ±8.3% от MOD=5999 */
#define PI_DEFAULT_OUT_LIM      500.0f

/* -----------------------------------------------------------------------
 * Дескриптор: тонкая обёртка над GFLIB_CTRL_PI_P_AW_T_FLT
 * ----------------------------------------------------------------------- */
typedef struct
{
    GFLIB_CTRL_PI_P_AW_T_FLT  gflib;     /**< Нативная структура RTCESL   */
    bool                       enabled;   /**< Флаг активности регулятора  */
} pi_ctrl_t;

/* -----------------------------------------------------------------------
 * API
 * ----------------------------------------------------------------------- */
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Инициализация со значениями по умолчанию.
 * @param ctrl  Дескриптор.
 */
void PI_Init(pi_ctrl_t *ctrl);

/**
 * @brief Установить коэффициенты.
 * @param ctrl    Дескриптор.
 * @param kp      Пропорциональный коэффициент (Kp).
 * @param igain   Дискретный интегральный коэффициент (Ki_cont * Ts).
 */
void PI_SetGains(pi_ctrl_t *ctrl, float_t kp, float_t igain);

/**
 * @brief Установить пределы выхода.
 * @param ctrl    Дескриптор.
 * @param lo      Нижний предел (отрицательное число тиков CnV).
 * @param hi      Верхний предел (положительное число тиков CnV).
 */
void PI_SetLimits(pi_ctrl_t *ctrl, float_t lo, float_t hi);

/**
 * @brief Шаг регулятора — вызывать каждые 50 мкс в DMA0_IRQHandler.
 *        Если ctrl->enabled == false, возвращает 0.0.
 * @param ctrl   Дескриптор.
 * @param error  Ошибка напряжения: V_ref_inst − V_meas_inst (В).
 * @return       Коррекция CnV (тики FTM1, может быть отрицательной).
 */
float_t PI_Step(pi_ctrl_t *ctrl, float_t error);

/**
 * @brief Сброс состояния интегратора в 0.
 * @param ctrl  Дескриптор.
 */
void PI_Reset(pi_ctrl_t *ctrl);

/**
 * @brief Включить/выключить регулятор.
 *        При выключении: выход=0, интегратор не обновляется.
 *        При включении: сброс состояния (PI_Reset) для плавного старта.
 */
void PI_SetEnabled(pi_ctrl_t *ctrl, bool enabled);

#ifdef __cplusplus
}
#endif

#endif /* PI_CTRL_H_ */
