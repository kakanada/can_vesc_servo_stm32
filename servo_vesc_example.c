/**
 ******************************************************************************
 * @file    vesc_servo_example.c
 * @brief   Пример использования vesc_servo.h/.c: как ожидается применять
 *          модуль в реальном проекте (серва сама создаёт и обслуживает
 *          веску, событийная модель, отдельный тик контура вызывать не
 *          нужно).
 *
 * @author  Mechanic
 * @date    12.08.2026
 * @copyright Свободное некоммерческое использование и модификация -
 *          PolyForm Noncommercial License 1.0.0, см. LICENSE в корне библиотеки.
 *
 *          Сценарий:
 *            1. При старте программы - один раз инициализируем серву
 *               (vesc_servo). VESC_CAN_Init() из motor_vesc вызывать не
 *               нужно - VESC_Servo_Init() сама регистрирует веску (шина,
 *               CAN ID, число полюсов - прямо в VESC_Servo_Config_t) и сама
 *               подписывается на её телеметрию. Хоуминг ПРИ ЭТОМ НЕ
 *               запускаем.
 *            2. Когда удобно (например по внешней команде/кнопке) -
 *               запускаем хоуминг ОДИН РАЗ.
 *            3. Дальше в своей логике - просто вызываем
 *               VESC_Servo_SetPosition() по мере расчёта новой цели, когда
 *               бы это ни понадобилось.
 *            4. Опционально - если вашему коду нужно знать о приходе
 *               телеметрии сразу же (например, чтобы тут же переслать
 *               что-то в общий пакет телеметрии системы) - регистрируем
 *               свой обработчик через VESC_Servo_SetTelemetryCallback().
 *            5. Опционально - изредка (например раз в секунду) вызываем
 *               VESC_Servo_CheckAlive(), если важно, чтобы состояние сервы
 *               достоверно показывало полную потерю связи с веской.
 *
 *          Единственное, что осталось общим с motor_vesc и по-прежнему
 *          требуется - подключение HAL-callback'ов CAN/FDCAN (см. пункт 6
 *          и шапку vesc_servo.h) - воткните их РОВНО ТАК ЖЕ, как в
 *          motor_vesc.h. Работать с остальным API motor_vesc напрямую
 *          (VESC_CAN_Init, VESC_CAN_SendXxx, VESC_CAN_SetTelemetryCallback)
 *          для обычного использования сервы не нужно - только заполните
 *          VESC_Servo_Config_t, всё остальное сделает сама серва.
 *
 *          Это НЕ готовый к сборке файл (нет main()/HAL_Init() и т.п.) - это
 *          иллюстрация порядка вызовов API, вставьте нужные части в свой
 *          проект.
 ******************************************************************************
 */

#include "vesc_servo.h"

/* Хэндл CAN-периферии из CubeMX (hcan1 для bxCAN / hfdcan1 для FDCAN) -
 * объявлен где-то в вашем main.c, здесь только extern-объявление. */
extern VESC_CAN_HandleTypeDef hcan1;

/* Указатель на серву - живёт всё время работы программы. Отдельного
 * указателя на веску заводить не нужно - при желании читать её "сырую"
 * телеметрию она доступна как g_wheel_servo->vesc->telemetry. */
static VESC_Servo_Handle_t *g_wheel_servo = NULL;

/* Опережающее объявление - определение см. в пункте 4 ниже, используется
 * уже в VESC_Servo_Example_Setup() (пункт 1). */
void VESC_Servo_Example_OnTelemetry(VESC_Servo_Handle_t *s, VESC_CAN_PacketId_t status_id);

/* ------------------------------------------------------------------------ */
/*  1. Инициализация - вызвать ОДИН РАЗ при старте программы                */
/* ------------------------------------------------------------------------ */

/**
 * @brief  Регистрирует серву - VESC_Servo_Init() сама создаёт веску внутри
 *         себя (шина + CAN ID + число полюсов - прямо в конфиге) и сама
 *         подписывается на её телеметрию. Хоуминг не запускает - это
 *         делает отдельно VESC_Servo_Example_StartHoming() ниже, когда вы
 *         сами решите, что пора (см. пояснение в шапке файла).
 */
void VESC_Servo_Example_Setup(void)
{
    VESC_Servo_Config_t servo_cfg = {0};

    /* --- Веска (было отдельным VESC_CAN_Init() - теперь прямо здесь) --- */
    servo_cfg.hcan       = &hcan1;
    servo_cfg.vesc_id    = 42;   /* CAN ID вески, см. VESC Tool */
    servo_cfg.pole_count = 14;   /* число полюсов мотора        */

    /* --- Контур позиции --- */
    servo_cfg.gear_ratio       = 20.0f;  /* редуктор 20:1                    */
    servo_cfg.max_speed_deg_s  = 60.0f;  /* не быстрее 60°/с на выходном валу */
    servo_cfg.max_accel_deg_s2 = 120.0f; /* разгон/торможение за 0.5 с до максимума */
    /* Убедитесь, что STATUS_4 в VESC Tool (App Settings -> General -> CAN
     * Status Message Rate) идёт достаточно часто под эти max_speed_deg_s -
     * см. формулу в комментарии над max_speed_deg_s в vesc_servo.h. */

    servo_cfg.pid_kp   = 2.0f;
    servo_cfg.pid_ki   = 0.5f;
    servo_cfg.pid_kd   = 0.05f;
    servo_cfg.pid_i_max = 20.0f;

    servo_cfg.error_start_correcting_deg = 10.0f; /* поехали корректировать, если разошлись на 10°  */
    servo_cfg.error_stop_deg             = 5.0f;  /* остановились, как только сошлись до 5° и меньше */

    servo_cfg.brake_at_target_fraction = 0.5f; /* 50% от макс. тока вески - см. VESC_Servo_SetBrakeAtTarget */

    /* --- Зона лимитов (жёсткая защита хода) и рабочий диапазон (для
     * VESC_Servo_SetPositionNormalized) - см. подробности в vesc_servo.h --- */
    servo_cfg.limit_min_deg   = -100.0f; /* дальше этого серва физически не должна доехать никогда */
    servo_cfg.limit_max_deg   =  100.0f;
    servo_cfg.working_min_deg =  -90.0f; /* -1.0 в SetPositionNormalized() */
    servo_cfg.working_max_deg =   90.0f; /* +1.0 в SetPositionNormalized() */

    /* --- Концевик - кастомный статус самой вески (byte[1] STATUS_7, см.
     * motor_vesc.h): у нашего физического концевика "нажато" соответствует
     * VESC_CUSTOM_SENSOR_PIN_SET (зависит от того, как разведён датчик -
     * подберите под свою схему). --- */
    servo_cfg.limit_switch_pressed_state = VESC_CUSTOM_SENSOR_PIN_SET;

    servo_cfg.homing_seek_speed_deg_s    =  15.0f; /* едем К концевику   */
    servo_cfg.homing_backoff_speed_deg_s = -10.0f; /* едем ОТ концевика  */
    servo_cfg.home_position_deg          = 0.0f;   /* здесь и будет ноль */
    servo_cfg.homing_timeout_ms          = 15000U;

    servo_cfg.telemetry_timeout_ms = 200U;

    g_wheel_servo = VESC_Servo_Init(&servo_cfg);
    /* g_wheel_servo == NULL - ошибка конфигурации/периферии, обработайте под свой проект */

    /* Хотим, чтобы вал держался тормозом, а не просто "отпускался", когда
     * серва считает, что она уже на месте: */
    VESC_Servo_SetBrakeAtTarget(g_wheel_servo, 1U);

    /* Хотим сразу узнавать о приходе любой телеметрии - см. пункт 4 ниже */
    VESC_Servo_SetTelemetryCallback(g_wheel_servo, VESC_Servo_Example_OnTelemetry);
}

/* ------------------------------------------------------------------------ */
/*  2. Хоуминг - вызвать, когда решите, что пора (не обязательно при старте) */
/* ------------------------------------------------------------------------ */

/**
 * @brief  Запускает поиск нуля. Вызовите, например, по кнопке/команде с
 *         верхнего уровня, либо автоматически спустя какое-то время после
 *         старта, когда телеметрия от вески уже точно пошла. Неблокирующая -
 *         сама процедура продвигается автоматически по приходу телеметрии,
 *         дополнительно вызывать ничего не нужно.
 * @retval 1 - хоуминг запущен; 0 - не готова (нет связи с веской/концевиком) -
 *         попробуйте ещё раз чуть позже.
 */
uint8_t VESC_Servo_Example_StartHoming(void)
{
    return (VESC_Servo_StartHoming(g_wheel_servo) == HAL_OK) ? 1U : 0U;
}

/* ------------------------------------------------------------------------ */
/*  3. Задание цели - вызывать когда угодно, по мере расчёта нового угла    */
/* ------------------------------------------------------------------------ */

/**
 * @brief  Пример места, где ваша собственная логика (телеметрия с других
 *         осей, задание оператора, траектория и т.п.) считает новый
 *         целевой угол и сообщает его серве. Контур сам продвинет привод к
 *         этой цели по мере прихода телеметрии от вески - вызывать здесь
 *         больше ничего не нужно. Цель автоматически обрезается по зоне
 *         лимитов - выйти за неё через эту функцию невозможно.
 */
void VESC_Servo_Example_OnNewTargetComputed(float desired_angle_deg)
{
    if (VESC_Servo_GetState(g_wheel_servo) == VESC_SERVO_STATE_READY)
    {
        VESC_Servo_SetPosition(g_wheel_servo, desired_angle_deg);
    }
}

/**
 * @brief  То же самое, но если удобнее задавать цель не в градусах, а в
 *         диапазоне -1.0..1.0 относительно рабочего диапазона (джойстик,
 *         слайдер и т.п.) - см. подробности в vesc_servo.h.
 */
void VESC_Servo_Example_OnNewNormalizedTarget(float normalized)
{
    if (VESC_Servo_GetState(g_wheel_servo) == VESC_SERVO_STATE_READY)
    {
        VESC_Servo_SetPositionNormalized(g_wheel_servo, normalized);
    }
}

/* ------------------------------------------------------------------------ */
/*  4. (Опционально) Свой колбэк телеметрии сервы                           */
/* ------------------------------------------------------------------------ */

/**
 * @brief  Вызывается серва на КАЖДЫЙ принятый статусный пакет вески (не
 *         только STATUS_4). На момент вызова уже свежи и
 *         s->vesc->telemetry (сырая телеметрия вески: ток, скорость,
 *         температура и т.п.), и s->telemetry (готовая телеметрия сервы:
 *         угол выходного вала, цель, состояние). Здесь удобно сразу
 *         скопировать нужные поля, например в общий пакет телеметрии
 *         системы, не дожидаясь следующего удобного опроса.
 *
 * @warning Вызывается ИЗ ПРЕРЫВАНИЯ - быстро, без блокировок (см.
 *          подробности в vesc_servo.h).
 */
void VESC_Servo_Example_OnTelemetry(VESC_Servo_Handle_t *s, VESC_CAN_PacketId_t status_id)
{
    (void)s;
    (void)status_id;

    /* Пример: скопировать угол выходного вала и ток мотора в volatile-поля,
     * которые основной код читает в удобное для себя время. */
    /* g_system_telemetry.wheel_angle_deg = s->telemetry.position_deg; */
    /* g_system_telemetry.wheel_current_a = s->vesc->telemetry.current; */
}

/* ------------------------------------------------------------------------ */
/*  5. (Опционально) Диагностика полной потери связи - вызывать изредка     */
/* ------------------------------------------------------------------------ */

/**
 * @brief  Пример периодической (например, раз в секунду) housekeeping-
 *         задачи. Не обязательна для нормальной работы контура - только
 *         для того, чтобы VESC_Servo_GetState() достоверно показывал
 *         VESC_SERVO_STATE_FAULT при полной тишине на шине (см. подробности
 *         в vesc_servo.h).
 */
void VESC_Servo_Example_HousekeepingTick(void)
{
    VESC_Servo_CheckAlive(g_wheel_servo);
}

/* ------------------------------------------------------------------------ */
/*  6. HAL-callback'и CAN - ЕДИНСТВЕННОЕ, что осталось общим с motor_vesc,  */
/*     см. подробную шпаргалку в motor_vesc.h и в шапке vesc_servo.h        */
/* ------------------------------------------------------------------------ */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    VESC_CAN_RxFifo0_Handler(hcan, 0);
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) { VESC_CAN_TxComplete_Handler(hcan); }
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) { VESC_CAN_TxComplete_Handler(hcan); }
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) { VESC_CAN_TxComplete_Handler(hcan); }
