/*
 * pi_ctrl.c
 * Обёртка над GFLIB_CtrlPIpAW_FLT_C() (NXP RTCESL libGFLIB.a).
 *
 * GFLIB_CtrlPIpAW_FLT_C(fltInErr, pbStopIntegFlag, psParam):
 *   - Параллельный ПИ с anti-windup типа "stop integrator":
 *       u[k] = Kp * e[k] + IAccK_1
 *       IAccK_1 = IAccK_1 + IGain * e[k]   (если !bLimFlag)
 *   - bLimFlag устанавливается библиотекой при достижении Upper/LowerLim.
 *   - pbStopIntegFlag указывает на bLimFlag — стандартная схема RTCESL.
 *   - Вся работа с FPU выполняется внутри libGFLIB.a.
 */

#include "pi_ctrl.h"
#include <string.h>

void PI_Init(pi_ctrl_t *ctrl)
{
    memset(ctrl, 0, sizeof(*ctrl));

    ctrl->gflib.fltPGain    = PI_DEFAULT_KP;
    ctrl->gflib.fltIGain    = PI_DEFAULT_IGAIN;
    ctrl->gflib.fltUpperLim = PI_DEFAULT_OUT_LIM;
    ctrl->gflib.fltLowerLim = -PI_DEFAULT_OUT_LIM;
    ctrl->enabled           = false;

    /* Сброс состояния GFLIB: интегратор и ошибка в 0 */
    GFLIB_CtrlPIpAWInit_FLT_Ci(0.0f, &ctrl->gflib);
}

void PI_SetGains(pi_ctrl_t *ctrl, float_t kp, float_t igain)
{
    ctrl->gflib.fltPGain = kp;
    ctrl->gflib.fltIGain = igain;
}

void PI_SetLimits(pi_ctrl_t *ctrl, float_t lo, float_t hi)
{
    ctrl->gflib.fltLowerLim = lo;
    ctrl->gflib.fltUpperLim = hi;
}

float_t PI_Step(pi_ctrl_t *ctrl, float_t error)
{
    if (!ctrl->enabled)
    {
        return 0.0f;
    }

    /*
     * GFLIB_CtrlPIpAW_FLT_C — макрос из gflib_FP.h:
     *   → GFLIB_CtrlPIpAW_FLT_FC(error, &gflib.bLimFlag, &gflib)
     *
     * Передаём &gflib.bLimFlag как pbStopIntegFlag:
     * библиотека сама устанавливает флаг при насыщении и
     * останавливает интегратор — стандартная anti-windup схема RTCESL.
     */
    return GFLIB_CtrlPIpAW_FLT_C(error,
                                  &ctrl->gflib.bLimFlag,
                                  &ctrl->gflib);
}

void PI_Reset(pi_ctrl_t *ctrl)
{
    /* GFLIB_CtrlPIpAWInit_FLT_Ci: обнуляет IAccK_1 и InErrK_1 */
    GFLIB_CtrlPIpAWInit_FLT_Ci(0.0f, &ctrl->gflib);
    ctrl->gflib.bLimFlag = (bool_t)0U;
}

void PI_SetEnabled(pi_ctrl_t *ctrl, bool enabled)
{
    if (enabled && !ctrl->enabled)
    {
        /* При включении сбрасываем состояние — плавный старт без скачка */
        PI_Reset(ctrl);
    }
    ctrl->enabled = enabled;
}
