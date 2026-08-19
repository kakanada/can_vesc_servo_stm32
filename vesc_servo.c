/**
 ******************************************************************************
 * @file    vesc_servo.c
 * @brief   Реализация сервослоя поверх motor_vesc. См. vesc_servo.h.
 *
 * @author  Mechanic
 * @date    12.08.2026
 * @version 1.2
 * @copyright Свободное некоммерческое использование и модификация -
 *          PolyForm Noncommercial License 1.0.0, полный текст см. LICENSE
 *          в корне библиотеки либо https://polyformproject.org/licenses/noncommercial/1.0.0
 ******************************************************************************
 */

#include "vesc_servo.h"
#include <string.h>
#include <math.h>

/* ========================================================================
 *  Внутреннее состояние модуля
 * ====================================================================== */

/* VESC_Servo_Handle_t объявлен целиком в vesc_servo.h (см. пояснение там же) -
 * здесь просто статический пул хэндлов, на которые модуль отдаёт указатели. */
static VESC_Servo_Handle_t s_servo_pool[VESC_SERVO_MAX_SERVOS];

/* ========================================================================
 *  Общие вспомогательные функции
 * ====================================================================== */

/** Ищет первый свободный (ещё не занятый) слот в статическом пуле серв. */
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

/** Ищет уже зарегистрированную серву по хэндлу вески, которую она
 *  оборачивает - используется ТОЛЬКО обработчиком телеметрии
 *  (vesc_servo_telemetry_handler), чтобы по пришедшему в колбэк
 *  VESC_Handle_t* найти "свою" VESC_Servo_Handle_t*, так как сам колбэк
 *  motor_vesc не передаёт пользовательский контекст. Пул серв небольшой
 *  (VESC_SERVO_MAX_SERVOS), линейный поиск в ISR обходится в единицы
 *  сравнений указателей и не создаёт заметной задержки. */
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

/** Читает мгновенное состояние концевика сервы s по последней принятой
 *  телеметрии вески (telemetry.custom_sensor_state, кастомный статус №7 -
 *  см. motor_vesc.h), сравнивая её с cfg.limit_switch_pressed_state. 1 -
 *  концевик нажат. Общая внутренняя реализация для публичной
 *  VESC_Servo_ReadLimitSwitch() и для процедуры хоуминга. */
static uint8_t vesc_servo_read_limit(const VESC_Servo_Handle_t *s)
{
    return (s->vesc->telemetry.custom_sensor_state == s->cfg.limit_switch_pressed_state) ? 1U : 0U;
}

/** Переводит накопленный (развёрнутый) угол мотора в угол ВЫХОДНОГО вала:
 *  делит на передаточное число и добавляет офсет, выставленный при
 *  хоуминге/ручной калибровке. Единая точка, где применяется gear_ratio,
 *  чтобы не разойтись между разными местами кода. */
static float vesc_servo_output_deg(const VESC_Servo_Handle_t *s)
{
    return (s->motor_unwrapped_deg / s->cfg.gear_ratio) + s->output_offset_deg;
}

/** Отправляет на веску целевую МЕХАНИЧЕСКУЮ скорость мотора, эквивалентную
 *  заданной скорости ВЫХОДНОГО вала (с учётом gear_ratio). Единая точка
 *  выхода команды скорости - и для хоуминга, и для контура позиции.
 *  360°/об, 60 с/мин -> RPM = (deg_s / 6) - смотри вывод в шапке файла. */
static void vesc_servo_send_motor_speed(const VESC_Servo_Handle_t *s, float output_deg_s)
{
    float motor_rpm = (output_deg_s * s->cfg.gear_ratio) / 6.0f;
    VESC_CAN_SendMechanicalSpeed(s->vesc, motor_rpm);
}

/** Немедленная остановка мотора (тормозной ток 0 А - свободное вращение
 *  вала, без активного удержания). Общая точка для аварийных остановок:
 *  вход в DISABLED, провал хоуминга, потеря телеметрии (VESC_Servo_CheckAlive).
 *  Не путать с "тормозом в цели" (см. brake_at_target_fraction) - там
 *  используется VESC_CAN_SendCurrentBrakeRel с заданной ненулевой силой,
 *  а не эта функция. */
static void vesc_servo_stop_motor(const VESC_Servo_Handle_t *s)
{
    VESC_CAN_SendCurrentBrake(s->vesc, 0.0f);
}

/** Обрезает угол выходного вала по ЗОНЕ ЛИМИТОВ (limit_min_deg/limit_max_deg
 *  из конфига) - единая точка, применяется к любому значению, которое
 *  собирается стать s->target_deg, чтобы гарантированно никогда не задать
 *  цель за пределами лимитов, независимо от того, что попросил пользователь
 *  библиотеки. См. подробности в vesc_servo.h. */
static float vesc_servo_clamp_to_limits(const VESC_Servo_Handle_t *s, float deg)
{
    if (deg < s->cfg.limit_min_deg) { return s->cfg.limit_min_deg; }
    if (deg > s->cfg.limit_max_deg) { return s->cfg.limit_max_deg; }
    return deg;
}

/** Пересчитывает и обновляет s->telemetry (см. VESC_Servo_Telemetry_t) по
 *  текущему внутреннему состоянию сервы - единая точка, чтобы "снимок" в
 *  s->telemetry никогда не расходился с реальным state/target/moving.
 *  Вызывается после любого изменения этих полей: изнутри обработчика
 *  телеметрии на каждый принятый статусный пакет (перед вызовом
 *  пользовательского VESC_Servo_TelemetryCallback_t - чтобы снимок был
 *  гарантированно свежим на момент вызова), а также сразу после каждого
 *  публичного вызова API, меняющего цель или состояние сервы. */
static void vesc_servo_refresh_telemetry(VESC_Servo_Handle_t *s)
{
    s->telemetry.position_deg = vesc_servo_output_deg(s);
    s->telemetry.target_deg   = s->target_deg;
    s->telemetry.error_deg    = s->target_deg - s->telemetry.position_deg;
    s->telemetry.moving       = s->moving;
    s->telemetry.state        = s->state;
    s->telemetry.homing_state = s->homing_state;
}

/** Общая внутренняя реализация задания цели - используется и
 *  VESC_Servo_SetPosition(), и VESC_Servo_SetPositionNormalized(). Обрезает
 *  target_deg по зоне лимитов (см. vesc_servo_clamp_to_limits) перед
 *  сохранением - см. подробности в vesc_servo.h. */
static HAL_StatusTypeDef vesc_servo_set_target(VESC_Servo_Handle_t *s, float target_deg)
{
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

/** Обновляет накопленный угол мотора (s->motor_unwrapped_deg) по свежему
 *  "сырому" (0..360, заворачивающемуся) значению telemetry.pid_pos. Считает
 *  кратчайшую дельту между предыдущим и новым сырым значением (то есть
 *  предполагает, что между приходами STATUS_4 мотор провернулся МЕНЬШЕ чем
 *  на пол-оборота - см. предупреждение в vesc_servo.h) и прибавляет её к
 *  накопителю. При первом вызове для этой сервы только запоминает точку
 *  отсчёта, ничего не накапливая. */
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

/** Успешное завершение хоуминга (концевик отпустился на этапе BACKOFF):
 *  останавливает мотор, выставляет output_offset_deg так, чтобы ПРЯМО
 *  СЕЙЧАС угол выходного вала стал равен home_position_deg, сбрасывает
 *  профиль/ПИД/гистерезис на эту же точку (цель = текущая позиция,
 *  коррекция не активна) и переводит серву в READY. */
static void vesc_servo_homing_finish(VESC_Servo_Handle_t *s)
{
    vesc_servo_stop_motor(s);

    s->output_offset_deg = s->cfg.home_position_deg - (s->motor_unwrapped_deg / s->cfg.gear_ratio);

    s->target_deg        = vesc_servo_clamp_to_limits(s, s->cfg.home_position_deg);
    s->moving             = 0U;
    s->profile_pos_deg   = s->cfg.home_position_deg;
    s->profile_vel_deg_s = 0.0f;
    s->pid_integral       = 0.0f;
    s->has_last_error     = 0U;

    s->homing_state = VESC_SERVO_HOMING_DONE;
    s->state        = VESC_SERVO_STATE_READY;
}

/** Провал хоуминга (таймаут на этапе SEEK или BACKOFF): останавливает
 *  мотор, помечает хоуминг как FAILED и переводит серву в FAULT. */
static void vesc_servo_homing_abort(VESC_Servo_Handle_t *s)
{
    vesc_servo_stop_motor(s);
    s->homing_state = VESC_SERVO_HOMING_FAILED;
    s->state        = VESC_SERVO_STATE_FAULT;
}

/** Один шаг процедуры хоуминга, вызывается изнутри обработчика телеметрии
 *  на каждый приход STATUS_4, пока s->state == VESC_SERVO_STATE_HOMING.
 *  Проверяет концевик (по телеметрии кастомного статуса вески) и таймаут
 *  текущего этапа, при необходимости переключает этап (SEEK -> BACKOFF)
 *  либо завершает хоуминг (успешно или по таймауту), и в конце (если
 *  процедура ещё не завершилась) на каждом шаге повторно отправляет
 *  команду скорости, соответствующую текущему этапу - это недорого и
 *  подстраховывает от единичной потери CAN-кадра во время длинного
 *  хоуминга. */
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
            vesc_servo_homing_abort(s);
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
            vesc_servo_homing_abort(s);
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

/** Один шаг контура позиции, вызывается изнутри обработчика телеметрии на
 *  каждый приход STATUS_4, пока s->state == VESC_SERVO_STATE_READY.
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
 *  на веску уходит сумма feedforward-скорости профиля и этой коррекции. */
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
    uint8_t beyond_limits = (actual < s->cfg.limit_min_deg) || (actual > s->cfg.limit_max_deg);

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
            /* профиль стартует заново от текущей факт. позиции - без рывка */
            s->profile_pos_deg   = actual;
            s->profile_vel_deg_s = 0.0f;
            s->pid_integral        = 0.0f;
            s->has_last_error      = 0U;
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

    float stopping_dist = (s->profile_vel_deg_s * s->profile_vel_deg_s) / (2.0f * max_a);
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

    s->pid_integral += error * dt_s;
    if (s->pid_integral >  s->cfg.pid_i_max) { s->pid_integral =  s->cfg.pid_i_max; }
    if (s->pid_integral < -s->cfg.pid_i_max) { s->pid_integral = -s->cfg.pid_i_max; }

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
 * @brief  Единственный обработчик телеметрии вески, регистрируется на
 *         веску сервы внутри VESC_Servo_Init() через
 *         VESC_CAN_SetTelemetryCallback(). Вызывается motor_vesc из
 *         прерывания приёма CAN на КАЖДЫЙ распознанный статусный пакет.
 *
 *         Делает две вещи:
 *           1. Если пакет - VESC_CAN_PACKET_STATUS_4 (несёт pid_pos, "тик"
 *              контура - см. обоснование выбора именно этого статуса в
 *              шапке vesc_servo.h): находит "свою" серву (см.
 *              vesc_servo_find_by_vesc), обновляет разворачивание угла
 *              мотора, считает dt с прошлого прихода STATUS_4 и, в
 *              зависимости от состояния сервы, продвигает либо процедуру
 *              хоуминга, либо контур позиции. На остальные типы статусов
 *              внутренняя логика контура не реагирует.
 *           2. Вне зависимости от типа пакета - если пользователь задал
 *              свой обработчик через VESC_Servo_SetTelemetryCallback(),
 *              обновляет s->telemetry (см. vesc_servo_refresh_telemetry) и
 *              вызывает этот обработчик - см. подробности в vesc_servo.h.
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

    if (status_id == VESC_CAN_PACKET_STATUS_4)
    {
        vesc_servo_update_wrap_tracking(s);

        uint32_t now = HAL_GetTick();
        if (!s->tick_initialized)
        {
            /* первый приход STATUS_4 после регистрации/включения - только
             * запоминаем момент времени, dt ещё не известен, шаг контура
             * пропускаем */
            s->last_tick        = now;
            s->tick_initialized = 1U;
        }
        else
        {
            float dt_s = (float)(now - s->last_tick) / 1000.0f;
            s->last_tick = now;

            if (dt_s > 0.0f) /* иначе - два прихода в один и тот же тик HAL_GetTick(), нечего интегрировать */
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

    /* Ретрансляция наружу - на КАЖДЫЙ статусный пакет, не только STATUS_4 */
    if (s->telemetry_callback != NULL)
    {
        vesc_servo_refresh_telemetry(s);
        s->telemetry_callback(s, status_id);
    }
}

/* ========================================================================
 *  Публичный API - регистрация и опрос состояния
 * ====================================================================== */

/** Регистрирует веску (шина + CAN ID + число полюсов из конфига) через
 *  VESC_CAN_Init() нижележащей библиотеки, затем регистрирует серву поверх
 *  неё, подставляя разумные значения по умолчанию для необязательных
 *  нулевых полей (telemetry_timeout_ms, homing_timeout_ms), строго
 *  валидируя остальные, и подписывается на телеметрию вески (см.
 *  предупреждение о единственном слоте под колбэк в vesc_servo.h).
 *  Подробности - см. vesc_servo.h. */
VESC_Servo_Handle_t *VESC_Servo_Init(const VESC_Servo_Config_t *config)
{
    if (config == NULL)
    {
        return NULL;
    }
    if ((config->gear_ratio <= 0.0f) || (config->max_speed_deg_s <= 0.0f) || (config->max_accel_deg_s2 <= 0.0f))
    {
        return NULL;
    }
    if (config->pid_i_max <= 0.0f)
    {
        return NULL; /* нулевой/отрицательный анти-виндап - интегратор либо не работает, либо не ограничен */
    }
    if (config->limit_switch_pressed_state == VESC_CUSTOM_SENSOR_NONE)
    {
        return NULL; /* NONE - это "данных ещё не было", не физическое состояние концевика */
    }
    if (config->error_start_correcting_deg <= 0.0f)
    {
        return NULL; /* иначе коррекция будет запускаться даже при нулевой ошибке - постоянное дрожание */
    }
    if ((config->error_stop_deg < 0.0f) || (config->error_stop_deg > config->error_start_correcting_deg))
    {
        return NULL; /* порог остановки должен быть в [0, порог_запуска] - иначе гистерезис не работает */
    }
    if ((config->brake_at_target_fraction < 0.0f) || (config->brake_at_target_fraction > 1.0f))
    {
        return NULL; /* доля тока в VESC_CAN_SendCurrentBrakeRel - только 0.0..1.0 */
    }
    if (config->limit_max_deg <= config->limit_min_deg)
    {
        return NULL; /* зона лимитов должна быть непустой */
    }
    if ((config->working_min_deg < config->limit_min_deg) || (config->working_max_deg > config->limit_max_deg)
        || (config->working_max_deg <= config->working_min_deg))
    {
        return NULL; /* рабочий диапазон должен быть непустым и целиком помещаться в зону лимитов */
    }
    if ((config->home_position_deg < config->limit_min_deg) || (config->home_position_deg > config->limit_max_deg))
    {
        return NULL; /* точка хоуминга обязана лежать внутри зоны лимитов */
    }

    /* Веску регистрирует сама серва - пользователю motor_vesc напрямую
     * трогать больше не нужно (см. шапку vesc_servo.h). Поля памяти
     * положения RTC сознательно не заполняются (остаются нулевыми/NULL) -
     * эта функция нижнего уровня этим слоем не используется и не должна
     * включаться для вески, обёрнутой сервой (см. предупреждение в .h). */
    VESC_Config_t vesc_cfg = {0};
    vesc_cfg.hcan       = config->hcan;
    vesc_cfg.vesc_id    = config->vesc_id;
    vesc_cfg.pole_count = config->pole_count;

    VESC_Handle_t *vesc = VESC_CAN_Init(&vesc_cfg);
    if (vesc == NULL)
    {
        return NULL; /* ошибка конфигурации/периферии CAN - см. VESC_CAN_Init() в motor_vesc.h */
    }
    if (vesc_servo_find_by_vesc(vesc) != NULL)
    {
        return NULL; /* эта веска уже обёрнута другой сервой - см. предупреждение в vesc_servo.h:
                       * колбэк телеметрии на веску только один, вторая серва его бы просто отобрала
                       * и молча перестала получать события */
    }

    VESC_Servo_Handle_t *s = vesc_servo_find_free_slot();
    if (s == NULL)
    {
        return NULL; /* исчерпан VESC_SERVO_MAX_SERVOS */
    }

    memset(s, 0, sizeof(*s));
    s->cfg  = *config;
    s->vesc = vesc;

    if (s->cfg.telemetry_timeout_ms == 0U) { s->cfg.telemetry_timeout_ms = 200U;   }
    if (s->cfg.homing_timeout_ms == 0U)     { s->cfg.homing_timeout_ms = 15000U;  }

    s->used         = 1U;
    s->state        = VESC_SERVO_STATE_DISABLED;
    s->homing_state = VESC_SERVO_HOMING_IDLE;

    vesc_servo_refresh_telemetry(s);

    /* Подписка на телеметрию вески - с этого момента контур сам крутится
     * по приходу STATUS_4 (см. vesc_servo_telemetry_handler выше). */
    VESC_CAN_SetTelemetryCallback(s->vesc, vesc_servo_telemetry_handler);

    return s;
}

/** Возвращает текущий угол выходного вала (см. подробности в .h про
 *  достоверность до хоуминга). */
float VESC_Servo_GetPositionDeg(VESC_Servo_Handle_t *s)
{
    if (s == NULL)
    {
        return 0.0f;
    }
    return vesc_servo_output_deg(s);
}

/** Возвращает текущую заданную конечную цель. */
float VESC_Servo_GetTargetDeg(VESC_Servo_Handle_t *s)
{
    return (s != NULL) ? s->target_deg : 0.0f;
}

/** Возвращает &s->telemetry (см. пояснение в vesc_servo.h). */
const VESC_Servo_Telemetry_t *VESC_Servo_GetTelemetry(VESC_Servo_Handle_t *s)
{
    return (s != NULL) ? &s->telemetry : NULL;
}

/** 1, если хоуминг/калибровка когда-либо успешно завершались. */
uint8_t VESC_Servo_IsHomed(VESC_Servo_Handle_t *s)
{
    return ((s != NULL) && (s->homing_state == VESC_SERVO_HOMING_DONE)) ? 1U : 0U;
}

/** 1, если серва READY и гистерезис сейчас не считает нужным корректировать
 *  позицию (см. s->moving/error_stop_deg). */
uint8_t VESC_Servo_IsAtTarget(VESC_Servo_Handle_t *s)
{
    if ((s == NULL) || (s->state != VESC_SERVO_STATE_READY))
    {
        return 0U;
    }
    return (uint8_t)(s->moving == 0U);
}

/** Возвращает общее состояние сервы. */
VESC_Servo_State_t VESC_Servo_GetState(VESC_Servo_Handle_t *s)
{
    return (s != NULL) ? s->state : VESC_SERVO_STATE_DISABLED;
}

/** Возвращает текущий/последний этап хоуминга. */
VESC_Servo_HomingState_t VESC_Servo_GetHomingState(VESC_Servo_Handle_t *s)
{
    return (s != NULL) ? s->homing_state : VESC_SERVO_HOMING_IDLE;
}

/** Публичная обёртка над vesc_servo_read_limit() для внешней диагностики. */
uint8_t VESC_Servo_ReadLimitSwitch(VESC_Servo_Handle_t *s)
{
    if (s == NULL)
    {
        return 0U;
    }
    return vesc_servo_read_limit(s);
}

/** Возвращает нижнюю границу зоны лимитов из конфига. */
float VESC_Servo_GetLimitMinDeg(VESC_Servo_Handle_t *s)
{
    return (s != NULL) ? s->cfg.limit_min_deg : 0.0f;
}

/** Возвращает верхнюю границу зоны лимитов из конфига. */
float VESC_Servo_GetLimitMaxDeg(VESC_Servo_Handle_t *s)
{
    return (s != NULL) ? s->cfg.limit_max_deg : 0.0f;
}

/** Возвращает нижнюю границу рабочего диапазона из конфига. */
float VESC_Servo_GetWorkingMinDeg(VESC_Servo_Handle_t *s)
{
    return (s != NULL) ? s->cfg.working_min_deg : 0.0f;
}

/** Возвращает верхнюю границу рабочего диапазона из конфига. */
float VESC_Servo_GetWorkingMaxDeg(VESC_Servo_Handle_t *s)
{
    return (s != NULL) ? s->cfg.working_max_deg : 0.0f;
}

/* ========================================================================
 *  Публичный API - управление
 * ====================================================================== */

/** Запускает процедуру хоуминга: проверяет концевик прямо сейчас (по
 *  телеметрии кастомного статуса вески) и, в зависимости от того, нажат
 *  ли он уже, стартует сразу с этапа BACKOFF (концевик изначально нажат -
 *  едем прочь от него) либо с этапа SEEK (концевик отпущен - сперва едем
 *  к нему); дальше процедура продвигается сама по приходу STATUS_4 (см.
 *  vesc_servo_telemetry_handler). Подробности - см. vesc_servo.h. */
HAL_StatusTypeDef VESC_Servo_StartHoming(VESC_Servo_Handle_t *s)
{
    if (s == NULL)
    {
        return HAL_ERROR;
    }
    if (!VESC_CAN_IsAlive(s->vesc, s->cfg.telemetry_timeout_ms))
    {
        return HAL_ERROR; /* без свежей телеметрии не видно, куда едем - запускать хоуминг вслепую небезопасно */
    }
    if ((s->vesc->telemetry.rx_mask & VESC_CAN_RXMASK_STATUS_7) == 0U)
    {
        return HAL_ERROR; /* от вески ещё ни разу не приходил кастомный статус с концевиком */
    }

    uint8_t pressed = vesc_servo_read_limit(s);

    s->state                   = VESC_SERVO_STATE_HOMING;
    s->homing_phase_start_tick = HAL_GetTick();
    s->homing_state            = pressed ? VESC_SERVO_HOMING_BACKOFF : VESC_SERVO_HOMING_SEEK;

    vesc_servo_refresh_telemetry(s);
    return HAL_OK;
}

/** Задаёт новую конечную цель контура позиции, обрезая её по зоне лимитов
 *  (см. vesc_servo_set_target/vesc_servo_clamp_to_limits). Профиль и
 *  гистерезис НЕ сбрасываются принудительно - если серва уже едет
 *  (s->moving == 1), она продолжает движение от текущей позиции профиля к
 *  новой цели без рывка; если стоит (s->moving == 0), запуск коррекции к
 *  новой цели решится гистерезисом на следующем приходе STATUS_4. */
HAL_StatusTypeDef VESC_Servo_SetPosition(VESC_Servo_Handle_t *s, float target_deg)
{
    if (s == NULL)
    {
        return HAL_ERROR;
    }
    return vesc_servo_set_target(s, target_deg);
}

/** Задаёт цель в нормализованном виде (-1.0..1.0 относительно рабочего
 *  диапазона working_min_deg/working_max_deg), переводит в градусы и
 *  делегирует в vesc_servo_set_target() (та же обрезка по зоне лимитов,
 *  что и у VESC_Servo_SetPosition()). Значения вне -1.0..1.0 линейно
 *  экстраполируются относительно рабочего диапазона - см. подробности в
 *  vesc_servo.h. */
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

/** Ручная калибровка "на лету": пересчитывает output_offset_deg так, чтобы
 *  текущий угол выходного вала стал равен actual_position_deg, сбрасывает
 *  профиль/ПИД/гистерезис на эту же точку и переводит серву в READY с
 *  хоумингом, засчитанным как пройденный. Подробности - см. vesc_servo.h. */
HAL_StatusTypeDef VESC_Servo_SetCurrentPosition(VESC_Servo_Handle_t *s, float actual_position_deg)
{
    if (s == NULL)
    {
        return HAL_ERROR;
    }
    if (!s->wrap_initialized)
    {
        return HAL_ERROR; /* ещё нет ни одного отсчёта телеметрии - не от чего считать офсет */
    }

    s->output_offset_deg = actual_position_deg - (s->motor_unwrapped_deg / s->cfg.gear_ratio);

    s->target_deg        = vesc_servo_clamp_to_limits(s, actual_position_deg);
    s->moving              = 0U;
    s->profile_pos_deg   = actual_position_deg;
    s->profile_vel_deg_s = 0.0f;
    s->pid_integral        = 0.0f;
    s->has_last_error      = 0U;

    s->homing_state = VESC_SERVO_HOMING_DONE;
    s->state        = VESC_SERVO_STATE_READY;

    vesc_servo_refresh_telemetry(s);
    return HAL_OK;
}

/** Включает контур позиции без повторного хоуминга, продолжая от текущей
 *  фактической позиции (профиль/цель сбрасываются на неё же - рывка не
 *  будет). Если за время VESC_SERVO_STATE_DISABLED вал свободно провернулся
 *  и фактический угол оказался за зоной лимитов - цель обрезается по зоне
 *  лимитов (см. vesc_servo_clamp_to_limits), и коррекция активируется
 *  немедленно (см. beyond_limits в vesc_servo_position_step), а не остаётся
 *  стоять снаружи; иначе коррекция изначально не активна. */
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

    float actual = vesc_servo_output_deg(s);
    s->target_deg        = vesc_servo_clamp_to_limits(s, actual);
    s->moving              = 0U;
    s->profile_pos_deg   = actual;
    s->profile_vel_deg_s = 0.0f;
    s->pid_integral        = 0.0f;
    s->has_last_error      = 0U;

    s->state = VESC_SERVO_STATE_READY;
    vesc_servo_refresh_telemetry(s);
    return HAL_OK;
}

/** Немедленно останавливает мотор и переводит серву в DISABLED. Прошедший
 *  ранее хоуминг не сбрасывается. Разворачивание угла мотора продолжает
 *  работать (обработчик телеметрии по-прежнему подписан и вызывается). */
HAL_StatusTypeDef VESC_Servo_Disable(VESC_Servo_Handle_t *s)
{
    if (s == NULL)
    {
        return HAL_ERROR;
    }
    vesc_servo_stop_motor(s);
    s->moving = 0U;
    s->state  = VESC_SERVO_STATE_DISABLED;
    vesc_servo_refresh_telemetry(s);
    return HAL_OK;
}

/** Переключатель "тормозить ли в цели" - см. подробности в vesc_servo.h.
 *  Само значение силы тормоза (brake_at_target_fraction) берётся из
 *  конфига сервы и здесь не меняется. */
HAL_StatusTypeDef VESC_Servo_SetBrakeAtTarget(VESC_Servo_Handle_t *s, uint8_t enabled)
{
    if (s == NULL)
    {
        return HAL_ERROR;
    }
    s->brake_at_target_enabled = enabled ? 1U : 0U;
    return HAL_OK;
}

/** Задаёт (или снимает, если callback == NULL) обработчик, ретранслируемый
 *  из vesc_servo_telemetry_handler() на каждый принятый статусный пакет -
 *  см. подробности в vesc_servo.h. */
HAL_StatusTypeDef VESC_Servo_SetTelemetryCallback(VESC_Servo_Handle_t *s, VESC_Servo_TelemetryCallback_t callback)
{
    if (s == NULL)
    {
        return HAL_ERROR;
    }
    s->telemetry_callback = callback;
    return HAL_OK;
}

/** Необязательная диагностика полной потери связи (см. подробности и
 *  предупреждение о пределах событийной модели в vesc_servo.h) - на сам
 *  контур позиции не влияет (он и так продвигается по приходу телеметрии),
 *  нужна только для того, чтобы VESC_Servo_GetState() отражал полную
 *  тишину на шине, которую сам обработчик телеметрии заметить не может по
 *  определению (он вызывается только когда пакет ДЕЙСТВИТЕЛЬНО пришёл). */
void VESC_Servo_CheckAlive(VESC_Servo_Handle_t *s)
{
    if (s == NULL)
    {
        return;
    }
    if (VESC_CAN_IsAlive(s->vesc, s->cfg.telemetry_timeout_ms))
    {
        return; /* телеметрия свежая - всё в порядке */
    }
    if ((s->state != VESC_SERVO_STATE_READY) && (s->state != VESC_SERVO_STATE_HOMING))
    {
        return; /* уже DISABLED/FAULT - команды и так не шлём, повторно останавливать нечего */
    }

    vesc_servo_stop_motor(s);
    if (s->state == VESC_SERVO_STATE_HOMING)
    {
        s->homing_state = VESC_SERVO_HOMING_FAILED;
    }
    s->moving = 0U;
    s->state  = VESC_SERVO_STATE_FAULT;
    vesc_servo_refresh_telemetry(s);
}
