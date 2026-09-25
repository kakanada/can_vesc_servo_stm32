/**
 ******************************************************************************
 * @file    vesc_servo.c
 * @brief   Реализация сервослоя поверх motor_vesc. См. vesc_servo.h.
 *
 * @author  Mechanic
 * @date    25.09.2026
 * @version 1.5
 * @copyright Copyright (c) 2026 Mechanic.
 *            Свободное некоммерческое использование и модификация. Условия
 *            распространения - см. LICENSE / README.md в составе проекта.
 ******************************************************************************
 */

#include "vesc_servo.h"
#include <string.h>
#include <math.h>

/* ========================================================================
 *  Опциональная интеграция с stm32_logger (см. @warning в шапке .h) -
 *  подключается ТОЛЬКО если проект сам определил LOGGER_ENABLE_VESC_SERVO
 *  (и подключаемый logger_codes.h реально резервирует LOG_CODE_VESC_SERVO_*)
 *  - без этого define модуль полностью самодостаточен, как и раньше.
 * ====================================================================== */
#ifdef LOGGER_ENABLE_VESC_SERVO
#include "logger.h"
#include "logger_codes.h"
#define VESC_SERVO_LOG(code, source_id, value) LOGGER_Log((code), (source_id), (value))
#else
#define VESC_SERVO_LOG(code, source_id, value) ((void)0)
#endif

/* ========================================================================
 *  Внутреннее состояние модуля
 * ====================================================================== */

/* VESC_Servo_Handle_t объявлен целиком в vesc_servo.h (см. пояснение там же) -
 * здесь просто статический пул хэндлов, на которые модуль отдаёт указатели. */
static VESC_Servo_Handle_t s_servo_pool[VESC_SERVO_MAX_SERVOS];

/* ========================================================================
 *  Общие вспомогательные функции
 * ====================================================================== */

/**
 * @brief   Ищет первый свободный (ещё не занятый) слот в статическом пуле серв.
 * @return  Указатель на свободный слот, либо NULL если пул исчерпан.
 */
static VESC_Servo_Handle_t *vesc_servo_find_free_slot(void)
{
    for (uint32_t i = 0U; i < VESC_SERVO_MAX_SERVOS; i++)
    {
        if (!s_servo_pool[i].used)
        {
            return &s_servo_pool[i];
        }
    }
    return NULL;
}

/**
 * @brief   Ищет уже зарегистрированную серву по хэндлу вески, которую она оборачивает.
 *
 *          Используется ТОЛЬКО обработчиком телеметрии (vesc_servo_telemetry_handler),
 *          чтобы по пришедшему в колбэк VESC_Handle_t* найти "свою" VESC_Servo_Handle_t*,
 *          так как сам колбэк motor_vesc не передаёт пользовательский контекст. Пул серв
 *          небольшой (VESC_SERVO_MAX_SERVOS), линейный поиск в ISR обходится в единицы
 *          сравнений указателей и не создаёт заметной задержки.
 * @param   vesc  хэндл вески, для которой ищется обёртывающая её серва
 * @return  Указатель на серву, либо NULL если эта веска не обёрнута.
 */
static VESC_Servo_Handle_t *vesc_servo_find_by_vesc(const VESC_Handle_t *vesc)
{
    for (uint32_t i = 0U; i < VESC_SERVO_MAX_SERVOS; i++)
    {
        if (s_servo_pool[i].used && (s_servo_pool[i].vesc == vesc))
        {
            return &s_servo_pool[i];
        }
    }
    return NULL;
}

/**
 * @brief   Читает мгновенное состояние концевика сервы по последней телеметрии.
 *
 *          Сравнивает telemetry.custom_sensor_state (кастомный статус №7 - см.
 *          motor_vesc.h) с cfg.limit_switch_pressed_state. Общая внутренняя
 *          реализация для публичной VESC_Servo_ReadLimitSwitch() и для процедуры
 *          хоуминга.
 * @param   s  серва
 * @return  1, если концевик нажат, иначе 0.
 */
static uint8_t vesc_servo_read_limit(const VESC_Servo_Handle_t *s)
{
    return (s->vesc->telemetry.custom_sensor_state == s->cfg.limit_switch_pressed_state) ? 1U : 0U;
}

/**
 * @brief   Переводит накопленный угол мотора в угол ВЫХОДНОГО вала.
 *
 *          Делит на передаточное число и добавляет офсет, выставленный при
 *          хоуминге/ручной калибровке. Единая точка, где применяется gear_ratio,
 *          чтобы не разойтись между разными местами кода.
 * @param   s  серва
 * @return  Угол выходного вала в градусах.
 */
static float vesc_servo_output_deg(const VESC_Servo_Handle_t *s)
{
    return (s->motor_unwrapped_deg * s->inv_gear_ratio) + s->output_offset_deg;
}

/**
 * @brief   Отправляет на веску механическую скорость мотора, эквивалентную
 *          заданной скорости выходного вала.
 *
 *          Единая точка выхода команды скорости - и для хоуминга, и для контура
 *          позиции. 360°/об, 60 с/мин -> RPM = (deg_s / 6) - смотри вывод в шапке файла.
 * @param   s              серва
 * @param   output_deg_s   желаемая скорость ВЫХОДНОГО вала, град/с
 */
static void vesc_servo_send_motor_speed(const VESC_Servo_Handle_t *s, float output_deg_s)
{
    float motor_rpm = output_deg_s * s->motor_rpm_per_deg_s;
    VESC_CAN_SendMechanicalSpeed(s->vesc, motor_rpm);
}

/**
 * @brief   Немедленная остановка мотора (тормозной ток 0 А, свободное вращение).
 *
 *          Общая точка для аварийных остановок: вход в DISABLED, провал хоуминга,
 *          потеря телеметрии (VESC_Servo_CheckAlive). Не путать с "тормозом в цели"
 *          (см. brake_at_target_fraction) - там используется VESC_CAN_SendCurrentBrakeRel
 *          с заданной ненулевой силой, а не эта функция.
 * @param   s  серва
 */
static void vesc_servo_stop_motor(const VESC_Servo_Handle_t *s)
{
    VESC_CAN_SendCurrentBrake(s->vesc, 0.0f);
}

/**
 * @brief   Обрезает угол выходного вала по зоне лимитов конфига.
 *
 *          Единая точка, применяется к любому значению, которое собирается стать
 *          s->target_deg, чтобы гарантированно никогда не задать цель за пределами
 *          лимитов, независимо от того, что попросил пользователь библиотеки. См.
 *          подробности в vesc_servo.h.
 * @param   s    серва
 * @param   deg  угол выходного вала, градусы
 * @return  Угол, обрезанный по [limit_min_deg, limit_max_deg].
 */
static float vesc_servo_clamp_to_limits(const VESC_Servo_Handle_t *s, float deg)
{
    if (deg < s->cfg.limit_min_deg) { return s->cfg.limit_min_deg; }
    if (deg > s->cfg.limit_max_deg) { return s->cfg.limit_max_deg; }
    return deg;
}

/**
 * @brief   Проверяет, лежит ли угол снаружи зоны лимитов.
 *
 *          Единая точка для этого условия, используется и контуром позиции
 *          (vesc_servo_position_step - принудительная коррекция вне гистерезиса), и
 *          местами, которые переустанавливают факт. позицию (VESC_Servo_Enable(),
 *          VESC_Servo_SetCurrentPosition()) и должны знать, нужно ли сразу считать
 *          серву "едущей", а не "на месте". См. подробности про зону лимитов в
 *          vesc_servo.h.
 * @param   s    серва
 * @param   deg  угол выходного вала, градусы
 * @return  1, если deg вне [limit_min_deg, limit_max_deg], иначе 0.
 */
static uint8_t vesc_servo_beyond_limits(const VESC_Servo_Handle_t *s, float deg)
{
    return ((deg < s->cfg.limit_min_deg) || (deg > s->cfg.limit_max_deg)) ? 1U : 0U;
}

/**
 * @brief   Сбрасывает состояние трапецеидального профиля и ПИД слежения на позицию pos.
 *
 *          Единая точка для всех мест, где контур позиции (пере)стартует от заданной
 *          фактической позиции (начало новой коррекции в гистерезисе, завершение
 *          хоуминга, ручная калибровка, VESC_Servo_Enable()), чтобы не тянуть за собой
 *          старую скорость/интеграл предыдущего движения и не создавать рывок. НЕ
 *          трогает target_deg/moving - это остаётся на усмотрение вызывающего.
 * @param   s    серва
 * @param   pos  позиция (градусы выходного вала), на которую сбрасывается профиль/ПИД
 */
static void vesc_servo_reset_tracking(VESC_Servo_Handle_t *s, float pos)
{
    s->profile_pos_deg   = pos;
    s->profile_vel_deg_s = 0.0f;
    s->pid_integral       = 0.0f;
    s->has_last_error     = 0U;
}

/**
 * @brief   Проверяет свежесть телеметрии STATUS_4 ("тика" контура).
 *
 *          В отличие от VESC_CAN_IsAlive() из motor_vesc, который считает "живым"
 *          приход ЛЮБОГО из 7 статусных пакетов - эта проверка специфична именно для
 *          STATUS_4, от которого зависит весь контур позиции/хоуминга (см. шапку файла).
 * @param   s  серва
 * @return  1, если STATUS_4 не старше telemetry_timeout_ms, иначе 0.
 */
static uint8_t vesc_servo_telemetry_fresh(const VESC_Servo_Handle_t *s)
{
    return (s->tick_initialized && ((HAL_GetTick() - s->last_tick) <= s->cfg.telemetry_timeout_ms)) ? 1U : 0U;
}

/**
 * @brief   Проверяет свежесть телеметрии STATUS_7 (кастомный статус с концевиком).
 *
 *          В отличие от telemetry.rx_mask из motor_vesc, который лишь фиксирует
 *          "приходил хотя бы раз" и никогда не сбрасывается - эта проверка ловит
 *          случай, когда STATUS_7 перестал приходить (например кастомный статус
 *          отключили в VESC Tool уже после первого прихода), а rx_mask бы этого не
 *          заметил.
 * @param   s  серва
 * @return  1, если STATUS_7 не старше telemetry_timeout_ms, иначе 0.
 */
static uint8_t vesc_servo_limit_switch_fresh(const VESC_Servo_Handle_t *s)
{
    return (s->status7_initialized && ((HAL_GetTick() - s->last_status7_tick) <= s->cfg.telemetry_timeout_ms)) ? 1U : 0U;
}

/**
 * @brief   Пересчитывает и обновляет s->telemetry по текущему внутреннему состоянию.
 *
 *          Единая точка, чтобы "снимок" в s->telemetry никогда не расходился с
 *          реальным state/target/moving. Вызывается после любого изменения этих
 *          полей: изнутри обработчика телеметрии на каждый принятый статусный пакет
 *          (перед вызовом пользовательского VESC_Servo_TelemetryCallback_t), а также
 *          сразу после каждого публичного вызова API, меняющего цель или состояние.
 * @param   s  серва
 */
static void vesc_servo_refresh_telemetry(VESC_Servo_Handle_t *s)
{
    s->telemetry.position_deg = vesc_servo_output_deg(s);
    s->telemetry.target_deg   = s->target_deg;
    s->telemetry.error_deg    = s->target_deg - s->telemetry.position_deg;
    s->telemetry.moving       = s->moving;
    s->telemetry.state        = s->state;
    s->telemetry.homing_state = s->homing_state;
}

/**
 * @brief   Общая внутренняя реализация задания цели контура позиции.
 *
 *          Используется и VESC_Servo_SetPosition(), и VESC_Servo_SetPositionNormalized().
 *          Обрезает target_deg по зоне лимитов (см. vesc_servo_clamp_to_limits) перед
 *          сохранением - см. подробности в vesc_servo.h.
 * @param   s           серва
 * @param   target_deg  новая цель, градусы выходного вала
 * @return  HAL_OK при успехе, HAL_ERROR если серва не READY или target_deg не конечен.
 */
static HAL_StatusTypeDef vesc_servo_set_target(VESC_Servo_Handle_t *s, float target_deg)
{
    if (!isfinite(target_deg))
    {
        return HAL_ERROR; /* NaN/Inf: сравнения с NaN всегда ложны, vesc_servo_clamp_to_limits()
                            * пропустил бы такое значение НЕобрезанным - лучше явно отклонить на
                            * входе, чем задать серве недостижимую/неопределённую цель */
    }
    if (s->state != VESC_SERVO_STATE_READY)
    {
        return HAL_ERROR; /* не хоумлена, отключена, либо в фолте - см. VESC_Servo_GetState() */
    }

    s->target_deg = vesc_servo_clamp_to_limits(s, target_deg);
    vesc_servo_refresh_telemetry(s);
    return HAL_OK;
}

/* ========================================================================
 *  Разворачивание (unwrap) угла мотора
 * ====================================================================== */

/**
 * @brief   Обновляет накопленный (развёрнутый) угол мотора по свежему сырому значению.
 *
 *          Считает кратчайшую дельту между предыдущим и новым сырым (0..360,
 *          заворачивающимся) значением telemetry.pid_pos (то есть предполагает, что
 *          между приходами STATUS_4 мотор провернулся МЕНЬШЕ чем на пол-оборота - см.
 *          предупреждение в vesc_servo.h) и прибавляет её к s->motor_unwrapped_deg. При
 *          первом вызове для этой сервы только запоминает точку отсчёта.
 * @param   s  серва
 */
static void vesc_servo_update_wrap_tracking(VESC_Servo_Handle_t *s)
{
    float raw = s->vesc->telemetry.pid_pos;

    if (!s->wrap_initialized)
    {
        s->last_raw_pid_pos_deg = raw;
        s->wrap_initialized = 1U;
        return;
    }

    float delta = raw - s->last_raw_pid_pos_deg;
    if (delta >  180.0f) { delta -= 360.0f; }
    if (delta < -180.0f) { delta += 360.0f; }

    s->motor_unwrapped_deg += delta;
    s->last_raw_pid_pos_deg = raw;
}

/* ========================================================================
 *  Процедура поиска нуля (хоуминг)
 * ====================================================================== */

/**
 * @brief   Успешное завершение хоуминга (концевик отпустился на этапе BACKOFF).
 *
 *          Останавливает мотор, выставляет output_offset_deg так, чтобы ПРЯМО СЕЙЧАС
 *          угол выходного вала стал равен home_position_deg, сбрасывает профиль/ПИД/
 *          гистерезис на эту же точку (цель = текущая позиция, коррекция не активна)
 *          и переводит серву в READY.
 * @param   s  серва
 */
static void vesc_servo_homing_finish(VESC_Servo_Handle_t *s)
{
    vesc_servo_stop_motor(s);

    s->output_offset_deg = s->cfg.home_position_deg - (s->motor_unwrapped_deg * s->inv_gear_ratio);

    s->target_deg = vesc_servo_clamp_to_limits(s, s->cfg.home_position_deg);
    s->moving      = 0U;
    vesc_servo_reset_tracking(s, s->cfg.home_position_deg);

    s->homing_state = VESC_SERVO_HOMING_DONE;
    s->state        = VESC_SERVO_STATE_READY;
    VESC_SERVO_LOG(LOG_CODE_VESC_SERVO_HOMING_DONE, s->cfg.vesc_id, 0);
}

/**
 * @brief   Останавливает мотор и переводит серву в FAULT.
 *
 *          Единая точка для всех причин входа в этот фолт (провал хоуминга по
 *          таймауту, потеря телеметрии по VESC_Servo_CheckAlive(), аномальный разрыв
 *          STATUS_4 - см. max_step_dt_ms в vesc_servo.h).
 * @param   s               серва
 * @param   homing_failed   пометить ли ещё и homing_state как VESC_SERVO_HOMING_FAILED
 *                          (актуально, только если хоуминг был активен)
 */
static void vesc_servo_enter_fault(VESC_Servo_Handle_t *s, uint8_t homing_failed)
{
    vesc_servo_stop_motor(s);
    if (homing_failed)
    {
        s->homing_state = VESC_SERVO_HOMING_FAILED;
    }
    s->moving = 0U;
    s->state  = VESC_SERVO_STATE_FAULT;
    VESC_SERVO_LOG(LOG_CODE_VESC_SERVO_FAULT_ENTERED, s->cfg.vesc_id, homing_failed ? 1 : 0);
}

/**
 * @brief   Один шаг процедуры хоуминга.
 *
 *          Вызывается изнутри обработчика телеметрии на каждый приход STATUS_4, пока
 *          s->state == VESC_SERVO_STATE_HOMING. Проверяет концевик и таймаут текущего
 *          этапа, при необходимости переключает этап (SEEK -> BACKOFF) либо завершает
 *          хоуминг (успешно или по таймауту), и в конце (если процедура ещё не
 *          завершилась) повторно отправляет команду скорости, соответствующую
 *          текущему этапу.
 * @param   s  серва
 */
static void vesc_servo_homing_step(VESC_Servo_Handle_t *s)
{
    uint8_t  pressed = vesc_servo_read_limit(s);
    uint32_t now      = HAL_GetTick();
    uint32_t elapsed   = now - s->homing_phase_start_tick;

    if (s->homing_state == VESC_SERVO_HOMING_SEEK)
    {
        if (pressed)
        {
            s->homing_state = VESC_SERVO_HOMING_BACKOFF;
            s->homing_phase_start_tick = now;
        }
        else if (elapsed > s->cfg.homing_timeout_ms)
        {
            vesc_servo_enter_fault(s, 1U);
            return;
        }
    }
    else if (s->homing_state == VESC_SERVO_HOMING_BACKOFF)
    {
        if (!pressed)
        {
            vesc_servo_homing_finish(s);
            return;
        }
        else if (elapsed > s->cfg.homing_timeout_ms)
        {
            vesc_servo_enter_fault(s, 1U);
            return;
        }
    }

    float v = (s->homing_state == VESC_SERVO_HOMING_SEEK) ? s->cfg.homing_seek_speed_deg_s
                                                            : s->cfg.homing_backoff_speed_deg_s;
    vesc_servo_send_motor_speed(s, v);
}

/* ========================================================================
 *  Контур позиции: гистерезис + трапецеидальный профиль + ПИД слежения
 * ====================================================================== */

/**
 * @brief   Один шаг контура позиции (гистерезис + трапецеидальный профиль + ПИД).
 *
 *          Вызывается изнутри обработчика телеметрии на каждый приход STATUS_4, пока
 *          s->state == VESC_SERVO_STATE_READY.
 *
 *  Сначала гистерезис по КОНЕЧНОЙ ошибке (target_deg минус факт) решает,
 *  активна ли сейчас коррекция (s->moving):
 *    - если серва СТОИТ и |ошибка| выросла до error_start_correcting_deg
 *      и выше - коррекция запускается (профиль/ПИД сбрасываются на
 *      текущую факт. позицию, чтобы не было рывка);
 *    - если серва ЕДЕТ и |ошибка| упала до error_stop_deg и ниже -
 *      коррекция останавливается.
 *  ИСКЛЮЧЕНИЕ из этого гистерезиса - выход факт. угла за ЗОНУ ЛИМИТОВ
 *  (limit_min_deg/limit_max_deg): в этом случае коррекция запускается
 *  немедленно и не останавливается, пока угол не вернётся внутрь зоны,
 *  независимо от порогов гистерезиса - см. подробности в vesc_servo.h.
 *  Пока коррекция не активна - на веску шлётся либо просто 0 (по
 *  умолчанию), либо тормоз заданной силы (см. VESC_Servo_SetBrakeAtTarget),
 *  а трапецеидальный профиль/ПИД вообще не считаются - без этого именно
 *  такое непрерывное подруливание около цели и создаёт "дрожание".
 *
 *  Пока коррекция активна - трапецеидальный профиль (s->profile_pos_deg/
 *  profile_vel_deg_s) продвигается на dt_s секунд (время с прошлого
 *  прихода STATUS_4) в сторону s->target_deg с ограничением
 *  max_speed_deg_s/max_accel_deg_s2, а ПИД считает коррекцию ОШИБКИ
 *  СЛЕЖЕНИЯ (позиция профиля минус факт, а не конечная цель минус факт), и
 *  на веску уходит сумма feedforward-скорости профиля и этой коррекции.
 *
 *  @note  dt_s приходит сюда уже проверенным вызывающей стороной
 *         (vesc_servo_telemetry_handler) - гарантированно 0 < dt_s <=
 *         max_step_dt_ms/1000; более длинный разрыв обрабатывается ДО
 *         вызова этой функции (см. max_step_dt_ms в vesc_servo.h).
 * @param   s     серва
 * @param   dt_s  время с прошлого прихода STATUS_4, секунды
 */
static void vesc_servo_position_step(VESC_Servo_Handle_t *s, float dt_s)
{
    float actual  = vesc_servo_output_deg(s);
    float abs_err = fabsf(s->target_deg - actual);

    /* Фактический угол вышел за ЗОНУ ЛИМИТОВ (внешнее воздействие и т.п.) -
     * гистерезис в этом случае не действует: коррекция обязана начаться
     * немедленно (даже если ошибка меньше error_start_correcting_deg) и не
     * должна останавливаться, пока угол не вернётся внутрь зоны - см.
     * подробности в vesc_servo.h. target_deg уже гарантированно внутри
     * зоны лимитов (см. vesc_servo_clamp_to_limits), так что обычный
     * профиль/ПИД ниже сам довезёт вал обратно. */
    uint8_t beyond_limits = vesc_servo_beyond_limits(s, actual);

    /* --- 0. Гистерезис: решаем, едем мы сейчас или стоим --- */
    if (s->moving)
    {
        if (!beyond_limits && (abs_err <= s->cfg.error_stop_deg))
        {
            s->moving = 0U;
        }
    }
    else
    {
        if (beyond_limits || (abs_err >= s->cfg.error_start_correcting_deg))
        {
            s->moving = 1U;
            vesc_servo_reset_tracking(s, actual); /* без рывка - профиль стартует от факта */
        }
    }

    if (!s->moving)
    {
        if (s->brake_at_target_enabled)
        {
            VESC_CAN_SendCurrentBrakeRel(s->vesc, s->cfg.brake_at_target_fraction);
        }
        else
        {
            vesc_servo_send_motor_speed(s, 0.0f);
        }
        return; /* коррекция не активна - профиль/ПИД не считаем вовсе */
    }

    /* --- 1. Трапецеидальный профиль --- */
    float max_v = s->cfg.max_speed_deg_s;
    float max_a = s->cfg.max_accel_deg_s2;
    float to_go = s->target_deg - s->profile_pos_deg;

    float stopping_dist = (s->profile_vel_deg_s * s->profile_vel_deg_s) * s->inv_2_max_accel;
    float desired_v;
    if (fabsf(to_go) <= stopping_dist)
    {
        desired_v = 0.0f; /* пора тормозить, чтобы не проскочить цель */
    }
    else
    {
        desired_v = (to_go >= 0.0f) ? max_v : -max_v;
    }

    float dv     = desired_v - s->profile_vel_deg_s;
    float max_dv = max_a * dt_s;
    if (dv >  max_dv) { dv =  max_dv; }
    if (dv < -max_dv) { dv = -max_dv; }
    s->profile_vel_deg_s += dv;

    float step = s->profile_vel_deg_s * dt_s;
    if (((to_go >= 0.0f) && (step > to_go)) || ((to_go < 0.0f) && (step < to_go)))
    {
        /* не даём профилю перепрыгнуть цель за один шаг на малой дистанции */
        step = to_go;
        s->profile_vel_deg_s = 0.0f;
    }
    s->profile_pos_deg += step;

    /* --- 2. ПИД коррекции ошибки слежения за профилем --- */
    float error = s->profile_pos_deg - actual;

    /* Анти-виндап клэмпит не сам интеграл (град*с), а его ВКЛАД В КОМАНДУ
     * (pid_ki * pid_integral, град/с) - именно так задокументирован
     * pid_i_max в vesc_servo.h. s->pid_integral_max = pid_i_max/|pid_ki|
     * посчитан один раз в VESC_Servo_Init() (см. кэш производных величин в
     * .h) - при pid_ki == 0.0f там же корректно получился +inf (деление
     * положительного pid_i_max на 0.0f), клэмп превращается в "не
     * ограничивать интеграл вовсе", т.к. при ki == 0 сам интеграл на
     * команду и так не влияет. */
    s->pid_integral += error * dt_s;
    if (s->pid_integral >  s->pid_integral_max) { s->pid_integral =  s->pid_integral_max; }
    if (s->pid_integral < -s->pid_integral_max) { s->pid_integral = -s->pid_integral_max; }

    float derr = s->has_last_error ? ((error - s->last_error_deg) / dt_s) : 0.0f;
    s->last_error_deg = error;
    s->has_last_error  = 1U;

    float correction = (s->cfg.pid_kp * error) + (s->cfg.pid_ki * s->pid_integral) + (s->cfg.pid_kd * derr);

    /* --- 3. Итоговая команда: feedforward профиля + коррекция, клэмп по потолку --- */
    float command_v = s->profile_vel_deg_s + correction;
    if (command_v >  max_v) { command_v =  max_v; }
    if (command_v < -max_v) { command_v = -max_v; }

    vesc_servo_send_motor_speed(s, command_v);
}

/* ========================================================================
 *  Обработчик телеметрии - "сердце" событийной модели (см. motor_vesc.h,
 *  VESC_TelemetryCallback_t / VESC_CAN_SetTelemetryCallback), и точка
 *  ретрансляции наружу через VESC_Servo_TelemetryCallback_t (см. vesc_servo.h)
 * ====================================================================== */

/**
 * @brief  Единственный обработчик телеметрии вески.
 *
 *         Регистрируется на
 *         веску сервы внутри VESC_Servo_Init() через
 *         VESC_CAN_SetTelemetryCallback(). Вызывается motor_vesc из
 *         прерывания приёма CAN на КАЖДЫЙ распознанный статусный пакет.
 *
 *         Делает три вещи:
 *           1. Если пакет - VESC_CAN_PACKET_STATUS_7 (кастомный, несёт
 *              концевик): запоминает момент прихода - см.
 *              vesc_servo_limit_switch_fresh().
 *           2. Если пакет - VESC_CAN_PACKET_STATUS_4 (несёт pid_pos, "тик"
 *              контура - см. обоснование выбора именно этого статуса в
 *              шапке vesc_servo.h): считает dt с прошлого прихода STATUS_4.
 *              Если разрыв не длиннее max_step_dt_ms (штатный тик, либо
 *              первый приход вообще) - обновляет разворачивание угла мотора
 *              и, в зависимости от состояния сервы, продвигает либо
 *              процедуру хоуминга, либо контур позиции. Если разрыв длиннее
 *              (см. max_step_dt_ms в vesc_servo.h) - НЕ пытается протащить
 *              через него дельту разворачивания угла (переанкерует точку
 *              отсчёта вместо этого) и, если серва была READY/HOMING,
 *              переводит её в VESC_SERVO_STATE_FAULT - накопленная до
 *              разрыва позиция более не достоверна. На остальные типы
 *              статусов эта часть логики не реагирует.
 *           3. Вне зависимости от типа пакета - обновляет s->telemetry (см.
 *              vesc_servo_refresh_telemetry), и если пользователь задал свой
 *              обработчик через VESC_Servo_SetTelemetryCallback() - вызывает
 *              его - см. подробности в vesc_servo.h.
 *
 * @param  h          хэндл вески, от которой пришёл статус (см.
 *                    VESC_TelemetryCallback_t в motor_vesc.h)
 * @param  status_id  какой именно статусный пакет только что разобран
 */
static void vesc_servo_telemetry_handler(VESC_Handle_t *h, VESC_CAN_PacketId_t status_id)
{
    VESC_Servo_Handle_t *s = vesc_servo_find_by_vesc(h);
    if (s == NULL)
    {
        return; /* веска не обёрнута ни одной сервой этого модуля */
    }

    if (status_id == VESC_CAN_PACKET_STATUS_7)
    {
        s->last_status7_tick   = HAL_GetTick();
        s->status7_initialized = 1U;
    }

    if (status_id == VESC_CAN_PACKET_STATUS_4)
    {
        uint32_t now      = HAL_GetTick();
        uint8_t  had_tick = s->tick_initialized;
        float    dt_s     = had_tick ? ((float)(now - s->last_tick) / 1000.0f) : 0.0f;

        s->last_tick        = now;
        s->tick_initialized = 1U;

        if (had_tick && (dt_s > s->max_step_dt_s))
        {
            /* Разрыв между приходами STATUS_4 больше max_step_dt_ms - см.
             * подробное обоснование обоих следствий в комментарии над
             * max_step_dt_ms в vesc_servo.h. */
            s->wrap_initialized = 0U; /* переанкеруемся на новой сырой точке на следующем валидном тике */
            if ((s->state == VESC_SERVO_STATE_READY) || (s->state == VESC_SERVO_STATE_HOMING))
            {
                vesc_servo_enter_fault(s, s->state == VESC_SERVO_STATE_HOMING);
            }
        }
        else
        {
            vesc_servo_update_wrap_tracking(s);

            if (had_tick && (dt_s > 0.0f)) /* не первый тик и не дублирующий приход в тот же HAL_GetTick() */
            {
                switch (s->state)
                {
                    case VESC_SERVO_STATE_HOMING:
                        vesc_servo_homing_step(s);
                        break;

                    case VESC_SERVO_STATE_READY:
                        vesc_servo_position_step(s, dt_s);
                        break;

                    case VESC_SERVO_STATE_DISABLED:
                    case VESC_SERVO_STATE_FAULT:
                    default:
                        break; /* контур не активен - команды не шлём, но unwrap выше уже обновлён */
                }
            }
        }
    }

    /* Снимок телеметрии сервы обновляется на КАЖДЫЙ статусный пакет
     * безусловно (не только когда задан колбэк) - см. контракт в
     * vesc_servo.h: s->telemetry можно опрашивать напрямую в любой момент. */
    vesc_servo_refresh_telemetry(s);
    if (s->telemetry_callback != NULL)
    {
        s->telemetry_callback(s, status_id);
    }
}

/* ========================================================================
 *  Публичный API - регистрация и опрос состояния
 * ====================================================================== */

/**
 * @brief   Регистрирует веску и серву поверх неё по заданному конфигу.
 *
 *          Регистрирует веску (шина + CAN ID + число полюсов из конфига) через
 *          VESC_CAN_Init() нижележащей библиотеки, затем регистрирует серву поверх
 *          неё, подставляя разумные значения по умолчанию для необязательных нулевых
 *          полей (telemetry_timeout_ms, homing_timeout_ms), строго валидируя
 *          остальные, и подписывается на телеметрию вески (см. предупреждение о
 *          единственном слоте под колбэк в vesc_servo.h). Подробности - см. vesc_servo.h.
 * @param   config  конфигурация сервы (копируется внутрь, указатель не сохраняется)
 * @return  Хэндл зарегистрированной сервы, либо NULL при ошибке конфигурации,
 *          исчерпанном пуле серв или ошибке нижележащей motor_vesc.
 */
VESC_Servo_Handle_t *VESC_Servo_Init(const VESC_Servo_Config_t *config)
{
    if (config == NULL)
    {
        VESC_SERVO_LOG(LOG_CODE_VESC_SERVO_INIT_BAD_CONFIG, 0U, 0);
        return NULL;
    }
    /* NaN/Inf где угодно в конфиге нужно ловить ЗДЕСЬ, отдельно и в первую
     * очередь - сравнения с NaN всегда ложны, так что ни одна из проверок
     * "<= 0"/"вне диапазона" ниже сама по себе NaN не поймает (например
     * NaN <= 0.0f тоже false, т.е. "обязательно > 0" пропустит NaN). Без
     * этой проверки NaN/Inf из конфига тихо просочился бы в кэш производных
     * величин (inv_gear_ratio и т.п. - см. ниже) и во весь контур. */
    if (!isfinite(config->gear_ratio) || !isfinite(config->max_speed_deg_s)
        || !isfinite(config->max_accel_deg_s2) || !isfinite(config->pid_kp)
        || !isfinite(config->pid_ki) || !isfinite(config->pid_kd) || !isfinite(config->pid_i_max)
        || !isfinite(config->error_start_correcting_deg) || !isfinite(config->error_stop_deg)
        || !isfinite(config->brake_at_target_fraction) || !isfinite(config->limit_min_deg)
        || !isfinite(config->limit_max_deg) || !isfinite(config->working_min_deg)
        || !isfinite(config->working_max_deg) || !isfinite(config->home_position_deg)
        || !isfinite(config->homing_seek_speed_deg_s) || !isfinite(config->homing_backoff_speed_deg_s))
    {
        VESC_SERVO_LOG(LOG_CODE_VESC_SERVO_INIT_BAD_CONFIG, config->vesc_id, 0);
        return NULL;
    }
    if ((config->gear_ratio <= 0.0f) || (config->max_speed_deg_s <= 0.0f) || (config->max_accel_deg_s2 <= 0.0f))
    {
        VESC_SERVO_LOG(LOG_CODE_VESC_SERVO_INIT_BAD_CONFIG, config->vesc_id, 1);
        return NULL;
    }
    if (config->pid_i_max <= 0.0f)
    {
        VESC_SERVO_LOG(LOG_CODE_VESC_SERVO_INIT_BAD_CONFIG, config->vesc_id, 2);
        return NULL; /* нулевой/отрицательный анти-виндап - интегратор либо не работает, либо не ограничен */
    }
    if (config->limit_switch_pressed_state == VESC_CUSTOM_SENSOR_NONE)
    {
        VESC_SERVO_LOG(LOG_CODE_VESC_SERVO_INIT_BAD_CONFIG, config->vesc_id, 3);
        return NULL; /* NONE - это "данных ещё не было", не физическое состояние концевика */
    }
    if (config->error_start_correcting_deg <= 0.0f)
    {
        VESC_SERVO_LOG(LOG_CODE_VESC_SERVO_INIT_BAD_CONFIG, config->vesc_id, 4);
        return NULL; /* иначе коррекция будет запускаться даже при нулевой ошибке - постоянное дрожание */
    }
    if ((config->error_stop_deg < 0.0f) || (config->error_stop_deg > config->error_start_correcting_deg))
    {
        VESC_SERVO_LOG(LOG_CODE_VESC_SERVO_INIT_BAD_CONFIG, config->vesc_id, 5);
        return NULL; /* порог остановки должен быть в [0, порог_запуска] - иначе гистерезис не работает */
    }
    if ((config->brake_at_target_fraction < 0.0f) || (config->brake_at_target_fraction > 1.0f))
    {
        VESC_SERVO_LOG(LOG_CODE_VESC_SERVO_INIT_BAD_CONFIG, config->vesc_id, 6);
        return NULL; /* доля тока в VESC_CAN_SendCurrentBrakeRel - только 0.0..1.0 */
    }
    if (config->limit_max_deg <= config->limit_min_deg)
    {
        VESC_SERVO_LOG(LOG_CODE_VESC_SERVO_INIT_BAD_CONFIG, config->vesc_id, 7);
        return NULL; /* зона лимитов должна быть непустой */
    }
    if ((config->working_min_deg < config->limit_min_deg) || (config->working_max_deg > config->limit_max_deg)
        || (config->working_max_deg <= config->working_min_deg))
    {
        VESC_SERVO_LOG(LOG_CODE_VESC_SERVO_INIT_BAD_CONFIG, config->vesc_id, 8);
        return NULL; /* рабочий диапазон должен быть непустым и целиком помещаться в зону лимитов */
    }
    if ((config->home_position_deg < config->limit_min_deg) || (config->home_position_deg > config->limit_max_deg))
    {
        VESC_SERVO_LOG(LOG_CODE_VESC_SERVO_INIT_BAD_CONFIG, config->vesc_id, 9);
        return NULL; /* точка хоуминга обязана лежать внутри зоны лимитов */
    }

    /* Слот пула серв резервируем ДО регистрации вески в motor_vesc -
     * VESC_CAN_Init() ниже необратимо занимает слот в СОБСТВЕННОМ пуле
     * motor_vesc (деинициализации у него нет), поэтому если бы пул серв
     * оказался исчерпан ПОСЛЕ уже успешной регистрации вески, эта веска
     * осталась бы зарегистрирована в motor_vesc навсегда без обёртывающей её
     * сервы - утечка слота motor_vesc на каждый неудачный из-за
     * VESC_SERVO_MAX_SERVOS вызов. Слот сервы на этом этапе ещё не помечен
     * used - при любом более позднем return NULL ниже он просто останется
     * свободным, ничего откатывать не нужно. */
    VESC_Servo_Handle_t *s = vesc_servo_find_free_slot();
    if (s == NULL)
    {
        VESC_SERVO_LOG(LOG_CODE_VESC_SERVO_INIT_POOL_FULL, config->vesc_id, 0);
        return NULL; /* исчерпан VESC_SERVO_MAX_SERVOS */
    }

    /* Веску регистрирует сама серва - пользователю motor_vesc напрямую
     * трогать больше не нужно (см. шапку vesc_servo.h). Поля памяти
     * положения RTC сознательно не заполняются (остаются нулевыми/NULL) -
     * эта функция нижнего уровня этим слоем не используется и не должна
     * включаться для вески, обёрнутой сервой (см. предупреждение в .h). */
    VESC_Config_t vesc_cfg = {0};
    vesc_cfg.bus        = config->bus;
    vesc_cfg.vesc_id    = config->vesc_id;
    vesc_cfg.pole_count = config->pole_count;

    VESC_Handle_t *vesc = VESC_CAN_Init(&vesc_cfg);
    if (vesc == NULL)
    {
        VESC_SERVO_LOG(LOG_CODE_VESC_SERVO_INIT_VESC_FAIL, config->vesc_id, 0);
        return NULL; /* ошибка конфигурации/периферии CAN - см. VESC_CAN_Init() в motor_vesc.h */
    }
    if (vesc_servo_find_by_vesc(vesc) != NULL)
    {
        VESC_SERVO_LOG(LOG_CODE_VESC_SERVO_INIT_DUPLICATE, config->vesc_id, 0);
        return NULL; /* эта веска уже обёрнута другой сервой - см. предупреждение в vesc_servo.h:
                       * колбэк телеметрии на веску только один, вторая серва его бы просто отобрала
                       * и молча перестала получать события. VESC_CAN_Init() выше идемпотентен для
                       * уже зарегистрированной (bus, vesc_id) - новый слот motor_vesc не расходуется. */
    }

    memset(s, 0, sizeof(*s));
    s->cfg  = *config;
    s->vesc = vesc;

    if (s->cfg.telemetry_timeout_ms == 0U) { s->cfg.telemetry_timeout_ms = 200U;   }
    if (s->cfg.homing_timeout_ms == 0U)     { s->cfg.homing_timeout_ms = 15000U;  }
    if (s->cfg.max_step_dt_ms == 0U)        { s->cfg.max_step_dt_ms = 100U;       }

    /* cfg отсюда и до конца жизни сервы больше не меняется (сеттера нет) -
     * считаем производные величины для горячего пути один раз здесь, а не
     * на каждом тике (см. поля-кэши в vesc_servo.h). */
    s->inv_gear_ratio      = 1.0f / s->cfg.gear_ratio;
    s->motor_rpm_per_deg_s = s->cfg.gear_ratio / 6.0f;
    s->inv_2_max_accel     = 1.0f / (2.0f * s->cfg.max_accel_deg_s2);
    s->pid_integral_max    = s->cfg.pid_i_max / fabsf(s->cfg.pid_ki);
    s->max_step_dt_s       = (float)s->cfg.max_step_dt_ms / 1000.0f;

    s->used         = 1U;
    s->state        = VESC_SERVO_STATE_DISABLED;
    s->homing_state = VESC_SERVO_HOMING_IDLE;

    vesc_servo_refresh_telemetry(s);

    /* Подписка на телеметрию вески - с этого момента контур сам крутится
     * по приходу STATUS_4 (см. vesc_servo_telemetry_handler выше). */
    VESC_CAN_SetTelemetryCallback(s->vesc, vesc_servo_telemetry_handler);

    VESC_SERVO_LOG(LOG_CODE_VESC_SERVO_INIT_OK, config->vesc_id, 0);
    return s;
}

/**
 * @brief   Возвращает текущий угол выходного вала (см. в .h про достоверность до хоуминга).
 * @param   s  серва
 * @return  Угол выходного вала, градусы; 0.0f если s == NULL.
 */
float VESC_Servo_GetPositionDeg(VESC_Servo_Handle_t *s)
{
    if (s == NULL)
    {
        return 0.0f;
    }
    return vesc_servo_output_deg(s);
}

/**
 * @brief   Возвращает текущую заданную конечную цель.
 * @param   s  серва
 * @return  Цель, градусы выходного вала; 0.0f если s == NULL.
 */
float VESC_Servo_GetTargetDeg(VESC_Servo_Handle_t *s)
{
    return (s != NULL) ? s->target_deg : 0.0f;
}

/**
 * @brief   Возвращает указатель на снимок телеметрии сервы (см. vesc_servo.h).
 * @param   s  серва
 * @return  &s->telemetry, либо NULL если s == NULL.
 */
const VESC_Servo_Telemetry_t *VESC_Servo_GetTelemetry(VESC_Servo_Handle_t *s)
{
    return (s != NULL) ? &s->telemetry : NULL;
}

/**
 * @brief   Проверяет, завершались ли когда-либо успешно хоуминг/калибровка.
 * @param   s  серва
 * @return  1, если да, иначе 0.
 */
uint8_t VESC_Servo_IsHomed(VESC_Servo_Handle_t *s)
{
    return ((s != NULL) && (s->homing_state == VESC_SERVO_HOMING_DONE)) ? 1U : 0U;
}

/**
 * @brief   Проверяет, что серва READY и коррекция позиции сейчас не активна
 *          (см. s->moving/error_stop_deg).
 * @param   s  серва
 * @return  1, если да, иначе 0.
 */
uint8_t VESC_Servo_IsAtTarget(VESC_Servo_Handle_t *s)
{
    if ((s == NULL) || (s->state != VESC_SERVO_STATE_READY))
    {
        return 0U;
    }
    return (uint8_t)(s->moving == 0U);
}

/**
 * @brief   Возвращает общее состояние сервы.
 * @param   s  серва
 * @return  Текущее состояние; VESC_SERVO_STATE_DISABLED если s == NULL.
 */
VESC_Servo_State_t VESC_Servo_GetState(VESC_Servo_Handle_t *s)
{
    return (s != NULL) ? s->state : VESC_SERVO_STATE_DISABLED;
}

/**
 * @brief   Возвращает текущий/последний этап хоуминга.
 * @param   s  серва
 * @return  Этап хоуминга; VESC_SERVO_HOMING_IDLE если s == NULL.
 */
VESC_Servo_HomingState_t VESC_Servo_GetHomingState(VESC_Servo_Handle_t *s)
{
    return (s != NULL) ? s->homing_state : VESC_SERVO_HOMING_IDLE;
}

/**
 * @brief   Публичная обёртка над vesc_servo_read_limit() для внешней диагностики.
 * @param   s  серва
 * @return  1, если концевик нажат, иначе 0.
 */
uint8_t VESC_Servo_ReadLimitSwitch(VESC_Servo_Handle_t *s)
{
    if (s == NULL)
    {
        return 0U;
    }
    return vesc_servo_read_limit(s);
}

/**
 * @brief   Возвращает нижнюю границу зоны лимитов из конфига.
 * @param   s  серва
 * @return  limit_min_deg; 0.0f если s == NULL.
 */
float VESC_Servo_GetLimitMinDeg(VESC_Servo_Handle_t *s)
{
    return (s != NULL) ? s->cfg.limit_min_deg : 0.0f;
}

/**
 * @brief   Возвращает верхнюю границу зоны лимитов из конфига.
 * @param   s  серва
 * @return  limit_max_deg; 0.0f если s == NULL.
 */
float VESC_Servo_GetLimitMaxDeg(VESC_Servo_Handle_t *s)
{
    return (s != NULL) ? s->cfg.limit_max_deg : 0.0f;
}

/**
 * @brief   Возвращает нижнюю границу рабочего диапазона из конфига.
 * @param   s  серва
 * @return  working_min_deg; 0.0f если s == NULL.
 */
float VESC_Servo_GetWorkingMinDeg(VESC_Servo_Handle_t *s)
{
    return (s != NULL) ? s->cfg.working_min_deg : 0.0f;
}

/**
 * @brief   Возвращает верхнюю границу рабочего диапазона из конфига.
 * @param   s  серва
 * @return  working_max_deg; 0.0f если s == NULL.
 */
float VESC_Servo_GetWorkingMaxDeg(VESC_Servo_Handle_t *s)
{
    return (s != NULL) ? s->cfg.working_max_deg : 0.0f;
}

/* ========================================================================
 *  Публичный API - управление
 * ====================================================================== */

/**
 * @brief   Запускает процедуру хоуминга.
 *
 *          Проверяет концевик прямо сейчас и, в зависимости от того, нажат ли он уже,
 *          стартует сразу с этапа BACKOFF (концевик изначально нажат - едем прочь от
 *          него) либо с этапа SEEK (концевик отпущен - сперва едем к нему); дальше
 *          процедура продвигается сама по приходу STATUS_4 (см.
 *          vesc_servo_telemetry_handler). Подробности - см. vesc_servo.h.
 * @param   s  серва
 * @return  HAL_OK при успешном запуске, HAL_ERROR если s == NULL или телеметрия
 *          (STATUS_4/STATUS_7) не свежая.
 */
HAL_StatusTypeDef VESC_Servo_StartHoming(VESC_Servo_Handle_t *s)
{
    if (s == NULL)
    {
        return HAL_ERROR;
    }
    if (!vesc_servo_telemetry_fresh(s))
    {
        return HAL_ERROR; /* без свежего STATUS_4 не видно, куда едем - запускать хоуминг вслепую небезопасно */
    }
    if (!vesc_servo_limit_switch_fresh(s))
    {
        return HAL_ERROR; /* от вески ещё ни разу не приходил (или давно не обновлялся) кастомный
                            * статус STATUS_7 с концевиком - см. vesc_servo_limit_switch_fresh() */
    }

    /* Если серва до этого что-то активно делала (едет к цели, либо уже была
     * в HOMING) - останавливаем мотор перед переходом на новый этап, а не
     * оставляем предыдущую команду скорости исполняться до первого тика
     * vesc_servo_homing_step(). */
    vesc_servo_stop_motor(s);

    uint8_t pressed = vesc_servo_read_limit(s);

    s->state                   = VESC_SERVO_STATE_HOMING;
    s->moving                  = 0U;
    s->homing_phase_start_tick = HAL_GetTick();
    s->homing_state            = pressed ? VESC_SERVO_HOMING_BACKOFF : VESC_SERVO_HOMING_SEEK;

    vesc_servo_refresh_telemetry(s);
    VESC_SERVO_LOG(LOG_CODE_VESC_SERVO_HOMING_START, s->cfg.vesc_id, 0);
    return HAL_OK;
}

/**
 * @brief   Задаёт новую конечную цель контура позиции.
 *
 *          Обрезает цель по зоне лимитов (см. vesc_servo_clamp_to_limits). Профиль и
 *          гистерезис НЕ сбрасываются принудительно - если серва уже едет
 *          (s->moving == 1), она продолжает движение от текущей позиции профиля к
 *          новой цели без рывка; если стоит, запуск коррекции решится гистерезисом на
 *          следующем приходе STATUS_4.
 * @param   s           серва
 * @param   target_deg  новая цель, градусы выходного вала
 * @return  HAL_OK при успехе, HAL_ERROR если s == NULL, серва не READY или target_deg
 *          не конечен.
 */
HAL_StatusTypeDef VESC_Servo_SetPosition(VESC_Servo_Handle_t *s, float target_deg)
{
    if (s == NULL)
    {
        return HAL_ERROR;
    }
    return vesc_servo_set_target(s, target_deg);
}

/**
 * @brief   Задаёт цель в нормализованном виде относительно рабочего диапазона.
 *
 *          Переводит normalized (-1.0..1.0 относительно working_min_deg/
 *          working_max_deg) в градусы и делегирует в vesc_servo_set_target() (та же
 *          обрезка по зоне лимитов, что и у VESC_Servo_SetPosition()). Значения вне
 *          -1.0..1.0 линейно экстраполируются - см. подробности в vesc_servo.h.
 * @param   s           серва
 * @param   normalized  нормализованная цель, обычно -1.0..1.0
 * @return  HAL_OK при успехе, HAL_ERROR если s == NULL, серва не READY или итоговый
 *          угол не конечен.
 */
HAL_StatusTypeDef VESC_Servo_SetPositionNormalized(VESC_Servo_Handle_t *s, float normalized)
{
    if (s == NULL)
    {
        return HAL_ERROR;
    }

    float mid  = (s->cfg.working_min_deg + s->cfg.working_max_deg) * 0.5f;
    float half = (s->cfg.working_max_deg - s->cfg.working_min_deg) * 0.5f;

    return vesc_servo_set_target(s, mid + (normalized * half));
}

/**
 * @brief   Ручная калибровка "на лету" без прохождения хоуминга.
 *
 *          Пересчитывает output_offset_deg так, чтобы текущий угол выходного вала
 *          стал равен actual_position_deg, сбрасывает профиль/ПИД/гистерезис на эту
 *          же точку и переводит серву в READY с хоумингом, засчитанным как пройденный.
 *          Подробности - см. vesc_servo.h.
 * @param   s                     серва
 * @param   actual_position_deg   фактический угол выходного вала прямо сейчас, градусы
 * @return  HAL_OK при успехе, HAL_ERROR если s == NULL, значение не конечно или ещё
 *          нет ни одного отсчёта телеметрии.
 */
HAL_StatusTypeDef VESC_Servo_SetCurrentPosition(VESC_Servo_Handle_t *s, float actual_position_deg)
{
    if (s == NULL)
    {
        return HAL_ERROR;
    }
    if (!isfinite(actual_position_deg))
    {
        return HAL_ERROR; /* NaN/Inf - см. ту же причину в vesc_servo_set_target() */
    }
    if (!s->wrap_initialized)
    {
        return HAL_ERROR; /* ещё нет ни одного отсчёта телеметрии - не от чего считать офсет */
    }

    /* Если серва до этого что-то активно делала (HOMING, либо READY с
     * активной коррекцией) - останавливаем мотор, а не оставляем предыдущую
     * команду исполняться до следующего тика. */
    vesc_servo_stop_motor(s);

    s->output_offset_deg = actual_position_deg - (s->motor_unwrapped_deg * s->inv_gear_ratio);

    s->target_deg = vesc_servo_clamp_to_limits(s, actual_position_deg);
    s->moving      = vesc_servo_beyond_limits(s, actual_position_deg); /* см. beyond_limits в vesc_servo_position_step */
    vesc_servo_reset_tracking(s, actual_position_deg);

    s->homing_state = VESC_SERVO_HOMING_DONE;
    s->state         = VESC_SERVO_STATE_READY;

    vesc_servo_refresh_telemetry(s);
    return HAL_OK;
}

/**
 * @brief   Пропускает физический хоуминг - текущий угол становится home_position_deg.
 *
 *          Тонкая обёртка над VESC_Servo_SetCurrentPosition(s, cfg.home_position_deg) -
 *          подробности и ограничения (в т.ч. почему это безопасно при перезапуске
 *          STM32, но не заменяет физическую повторяемость концевика) - см. vesc_servo.h.
 * @param   s  серва
 * @return  HAL_OK при успехе, HAL_ERROR если s == NULL или ещё нет ни одного отсчёта
 *          телеметрии.
 */
HAL_StatusTypeDef VESC_Servo_SkipHoming(VESC_Servo_Handle_t *s)
{
    if (s == NULL)
    {
        return HAL_ERROR;
    }
    return VESC_Servo_SetCurrentPosition(s, s->cfg.home_position_deg);
}

/**
 * @brief   Включает контур позиции без повторного хоуминга.
 *
 *          Продолжает от текущей фактической позиции (профиль/цель сбрасываются на
 *          неё же - рывка не будет). Если за время VESC_SERVO_STATE_DISABLED вал
 *          свободно провернулся и фактический угол оказался за зоной лимитов - цель
 *          обрезается по зоне лимитов, и коррекция активируется немедленно (см.
 *          beyond_limits в vesc_servo_position_step); иначе коррекция изначально не
 *          активна.
 * @param   s  серва
 * @return  HAL_OK при успехе (в т.ч. если уже была READY), HAL_ERROR если s == NULL,
 *          хоуминг/калибровка ни разу не выполнялись или телеметрия не свежая.
 */
HAL_StatusTypeDef VESC_Servo_Enable(VESC_Servo_Handle_t *s)
{
    if (s == NULL)
    {
        return HAL_ERROR;
    }
    if (s->homing_state != VESC_SERVO_HOMING_DONE)
    {
        return HAL_ERROR; /* хоуминг/калибровка ни разу не выполнялись */
    }
    if (s->state == VESC_SERVO_STATE_READY)
    {
        return HAL_OK; /* уже включена */
    }
    if (!vesc_servo_telemetry_fresh(s))
    {
        return HAL_ERROR; /* без свежего STATUS_4 неизвестна текущая факт. позиция -
                            * включать контур по ней вслепую небезопасно (см. ту же
                            * проверку в VESC_Servo_StartHoming()); актуально в первую
                            * очередь для выхода из VESC_SERVO_STATE_FAULT, вызванного
                            * VESC_Servo_CheckAlive() - без этой проверки Enable() мог
                            * бы молча вернуть серву в READY при всё ещё мёртвой связи */
    }

    float actual = vesc_servo_output_deg(s);
    s->target_deg = vesc_servo_clamp_to_limits(s, actual);
    s->moving      = vesc_servo_beyond_limits(s, actual); /* см. doc-комментарий выше */
    vesc_servo_reset_tracking(s, actual);

    s->state = VESC_SERVO_STATE_READY;
    vesc_servo_refresh_telemetry(s);
    return HAL_OK;
}

/**
 * @brief   Немедленно останавливает мотор и переводит серву в DISABLED.
 *
 *          Прошедший ранее хоуминг не сбрасывается. Разворачивание угла мотора
 *          продолжает работать (обработчик телеметрии по-прежнему подписан). Если
 *          вызвана посреди HOMING - процедура прерывается и засчитывается как
 *          неудавшаяся (см. VESC_SERVO_HOMING_FAILED).
 * @param   s  серва
 * @return  HAL_OK при успехе, HAL_ERROR если s == NULL.
 */
HAL_StatusTypeDef VESC_Servo_Disable(VESC_Servo_Handle_t *s)
{
    if (s == NULL)
    {
        return HAL_ERROR;
    }
    vesc_servo_stop_motor(s);
    if (s->state == VESC_SERVO_STATE_HOMING)
    {
        s->homing_state = VESC_SERVO_HOMING_FAILED;
    }
    s->moving = 0U;
    s->state  = VESC_SERVO_STATE_DISABLED;
    vesc_servo_refresh_telemetry(s);
    return HAL_OK;
}

/**
 * @brief   Переключатель "тормозить ли в цели" (см. подробности в vesc_servo.h).
 *
 *          Само значение силы тормоза (brake_at_target_fraction) берётся из конфига
 *          сервы и здесь не меняется.
 * @param   s        серва
 * @param   enabled  0 - выключить тормоз в цели, иначе - включить
 * @return  HAL_OK при успехе, HAL_ERROR если s == NULL.
 */
HAL_StatusTypeDef VESC_Servo_SetBrakeAtTarget(VESC_Servo_Handle_t *s, uint8_t enabled)
{
    if (s == NULL)
    {
        return HAL_ERROR;
    }
    s->brake_at_target_enabled = enabled ? 1U : 0U;
    return HAL_OK;
}

/**
 * @brief   Задаёт (или снимает, если callback == NULL) пользовательский колбэк
 *          телеметрии сервы (см. подробности в vesc_servo.h).
 * @param   s         серва
 * @param   callback  функция, вызываемая на каждый принятый статусный пакет,
 *                     либо NULL чтобы снять колбэк
 * @return  HAL_OK при успехе, HAL_ERROR если s == NULL.
 */
HAL_StatusTypeDef VESC_Servo_SetTelemetryCallback(VESC_Servo_Handle_t *s, VESC_Servo_TelemetryCallback_t callback)
{
    if (s == NULL)
    {
        return HAL_ERROR;
    }
    s->telemetry_callback = callback;
    return HAL_OK;
}

/**
 * @brief   Необязательная диагностика полной потери связи (см. предупреждение о
 *          пределах событийной модели в vesc_servo.h).
 *
 *          На сам контур позиции не влияет (он и так продвигается по приходу
 *          телеметрии), нужна только для того, чтобы VESC_Servo_GetState() отражал
 *          полную тишину на шине, которую сам обработчик телеметрии заметить не
 *          может по определению.
 * @param   s  серва
 */
void VESC_Servo_CheckAlive(VESC_Servo_Handle_t *s)
{
    if (s == NULL)
    {
        return;
    }
    if (vesc_servo_telemetry_fresh(s))
    {
        return; /* STATUS_4 свежий - всё в порядке */
    }
    if ((s->state != VESC_SERVO_STATE_READY) && (s->state != VESC_SERVO_STATE_HOMING))
    {
        return; /* уже DISABLED/FAULT - команды и так не шлём, повторно останавливать нечего */
    }

    vesc_servo_enter_fault(s, s->state == VESC_SERVO_STATE_HOMING);
    vesc_servo_refresh_telemetry(s);
}
