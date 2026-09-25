# vesc_servo — справочник API

> Этот файл — сжатый пересказ контракта `vesc_servo.h`. При любом расхождении между этим
> справочником и `.h`-файлом (сигнатуры, точные условия ошибок, единицы измерения) первичен `.h` —
> там же живут полные списки причин возврата `NULL`/`HAL_ERROR` и подробные `@warning`.

## Оглавление

- [Типы](#типы)
  - [`VESC_Servo_State_t`](#vesc_servo_state_t)
  - [`VESC_Servo_HomingState_t`](#vesc_servo_homingstate_t)
  - [`VESC_Servo_HomingMode_t`](#vesc_servo_homingmode_t)
  - [`VESC_Servo_Telemetry_t`](#vesc_servo_telemetry_t)
  - [`VESC_Servo_TelemetryCallback_t`](#vesc_servo_telemetrycallback_t)
  - [`VESC_Servo_Config_t`](#vesc_servo_config_t)
  - [`VESC_Servo_Handle_t`](#vesc_servo_handle_t)
- [Инициализация](#инициализация)
  - [`VESC_Servo_Init`](#vesc_servo_init)
- [Управление](#управление)
  - [`VESC_Servo_StartHoming`](#vesc_servo_starthoming)
  - [`VESC_Servo_SetPosition`](#vesc_servo_setposition)
  - [`VESC_Servo_SetPositionNormalized`](#vesc_servo_setpositionnormalized)
  - [`VESC_Servo_SetCurrentPosition`](#vesc_servo_setcurrentposition)
  - [`VESC_Servo_Enable`](#vesc_servo_enable)
  - [`VESC_Servo_Disable`](#vesc_servo_disable)
  - [`VESC_Servo_SetBrakeAtTarget`](#vesc_servo_setbrakeattarget)
  - [`VESC_Servo_SetTelemetryCallback`](#vesc_servo_settelemetrycallback)
  - [`VESC_Servo_CheckAlive`](#vesc_servo_checkalive)
- [Чтение состояния](#чтение-состояния)
  - [`VESC_Servo_GetPositionDeg`](#vesc_servo_getpositiondeg)
  - [`VESC_Servo_GetTargetDeg`](#vesc_servo_gettargetdeg)
  - [`VESC_Servo_GetTelemetry`](#vesc_servo_gettelemetry)
  - [`VESC_Servo_IsHomed`](#vesc_servo_ishomed)
  - [`VESC_Servo_IsAtTarget`](#vesc_servo_isattarget)
  - [`VESC_Servo_GetState`](#vesc_servo_getstate)
  - [`VESC_Servo_GetHomingState`](#vesc_servo_gethomingstate)
  - [`VESC_Servo_ReadLimitSwitch`](#vesc_servo_readlimitswitch)
  - [`VESC_Servo_GetLimitMinDeg` / `GetLimitMaxDeg`](#vesc_servo_getlimitmindeg--getlimitmaxdeg)
  - [`VESC_Servo_GetWorkingMinDeg` / `GetWorkingMaxDeg`](#vesc_servo_getworkingmindeg--getworkingmaxdeg)
- [Логирование (опционально)](#логирование-опционально)

## Типы

### `VESC_Servo_State_t`

Общее состояние сервы.

| Значение | Смысл |
|---|---|
| `VESC_SERVO_STATE_DISABLED` | Контур выключен, команды на веску не шлются. Начальное состояние после `VESC_Servo_Init()`. |
| `VESC_SERVO_STATE_HOMING` | Идёт поиск нуля, см. `VESC_Servo_HomingState_t`. |
| `VESC_SERVO_STATE_READY` | Позиция достоверна, доступен `VESC_Servo_SetPosition()`. |
| `VESC_SERVO_STATE_FAULT` | Хоуминг провалился по таймауту, либо `VESC_Servo_CheckAlive()` обнаружила потерю телеметрии, либо разрыв между `STATUS_4` превысил `max_step_dt_ms`. Выход — `VESC_Servo_StartHoming()`, `VESC_Servo_SetCurrentPosition()`, либо `VESC_Servo_Enable()` (только если хоуминг уже проходили и телеметрия снова свежая). |

### `VESC_Servo_HomingState_t`

Этап процедуры поиска нуля (валиден, пока `state == HOMING`, либо как история последнего хоуминга).

| Значение | Смысл |
|---|---|
| `VESC_SERVO_HOMING_IDLE` | Хоуминг ни разу не запускался. |
| `VESC_SERVO_HOMING_SEEK` | Этап 1: едем к концевику. Пропускается, если концевик уже нажат на старте. |
| `VESC_SERVO_HOMING_BACKOFF` | Этап 2: отъезжаем от концевика до отпускания — момент отпускания и есть ноль. |
| `VESC_SERVO_HOMING_DONE` | Хоуминг (или ручная калибровка) успешно завершены, серва в `READY`. |
| `VESC_SERVO_HOMING_FAILED` | Хоуминг не завершился успехом (таймаут этапа, потеря телеметрии, либо явный `VESC_Servo_Disable()` посреди хоуминга). |

### `VESC_Servo_HomingMode_t`

Способ определения нулевой точки отсчёта угла — поле `homing_mode` в `VESC_Servo_Config_t`. Все 4
варианта равноправны, выбираются одним полем конфига под конкретное железо. Независимо от значения,
`VESC_Servo_StartHoming()`/`SetCurrentPosition()` остаются доступны для ручного вызова в любой момент.

| Значение | Что происходит | Когда использовать |
|---|---|---|
| `VESC_SERVO_HOMING_MODE_REQUIRED` (0, по умолчанию) | Ничего автоматически — `DISABLED`, пока не вызван `VESC_Servo_StartHoming()`. | Есть концевик, нужна физическая повторяемость нуля. |
| `VESC_SERVO_HOMING_MODE_ZERO_AT_BOOT` | Автоматически на первой телеметрии: текущий угол становится `home_position_deg`, сразу `READY`. | Концевика нет, повторяемость нуля между включениями не важна. |
| `VESC_SERVO_HOMING_MODE_MANUAL_EXTERNAL` | Как `REQUIRED`, но вместо `StartHoming()` вызывается `VESC_Servo_SetCurrentPosition()` со значением от внешнего источника. | Ноль известен из источника, не связанного с концевиком/телеметрией вески. |
| `VESC_SERVO_HOMING_MODE_TRUST_ABSOLUTE` | Автоматически на первой телеметрии: угол принимается КАК ЕСТЬ от вески, без коррекции. Сразу `READY`. | Абсолютный энкодер на моторе, откалиброванный при сборке. |

**⚠️ `ZERO_AT_BOOT`:** ноль не физически воспроизводим между включениями — см. `vesc_servo.h`.
**⚠️ `TRUST_ABSOLUTE`:** при `gear_ratio > 1` однозначен только в пределах первого оборота мотора от
истинного нуля — см. подробное предупреждение в `vesc_servo.h`.

### `VESC_Servo_Telemetry_t`

Готовый снимок состояния сервы, обновляется автоматически на каждый статусный пакет вески.

| Поле | Тип | Описание |
|---|---|---|
| `position_deg` | `float` | Текущий угол выходного вала, градусы. |
| `target_deg` | `float` | Текущая заданная цель, градусы. |
| `error_deg` | `float` | `target_deg - position_deg`, градусы. |
| `moving` | `uint8_t` | 1, если коррекция сейчас активна (см. гистерезис). |
| `state` | `VESC_Servo_State_t` | Общее состояние сервы. |
| `homing_state` | `VESC_Servo_HomingState_t` | Этап/история хоуминга. |

### `VESC_Servo_TelemetryCallback_t`

```c
typedef void (*VESC_Servo_TelemetryCallback_t)(VESC_Servo_Handle_t *s, VESC_CAN_PacketId_t status_id);
```

Тип обработчика для `VESC_Servo_SetTelemetryCallback()`. Вызывается на каждый статусный пакет вески
этой сервы (все 7 видов, не только `STATUS_4`), из прерывания приёма CAN — должен быть быстрым и не
блокирующим.

| Параметр | Описание |
|---|---|
| `s` | Хэндл сервы, чьей вески пришёл статус. |
| `status_id` | Тип пакета — значение `VESC_CAN_PacketId_t` из `motor_vesc.h`. |

### `VESC_Servo_Config_t`

Конфигурация для `VESC_Servo_Init()`.

| Поле | Тип | Требование | Назначение |
|---|---|---|---|
| `bus` | `CANMGR_Handle_t*` | не NULL | Хэндл шины `can_manager` (результат `CANMGR_Init()`), на которой сидит веска. |
| `vesc_id` | `uint8_t` | 0..255 | CAN ID вески (VESC Tool: App Settings → General → VESC ID). |
| `pole_count` | `uint8_t` | чётное, ≠0 | Число полюсов мотора — пересчёт эл. RPM ↔ мех. RPM. |
| `gear_ratio` | `float` | > 0 | Передаточное число редуктора = обороты мотора / обороты выходного вала. |
| `max_speed_deg_s` | `float` | > 0 | Максимальная скорость выходного вала, град/с. |
| `max_accel_deg_s2` | `float` | > 0 | Максимальное ускорение выходного вала, град/с². |
| `max_step_dt_ms` | `uint32_t` | 0 ⇒ 100 | Верхний предел интервала между приходами `STATUS_4`, мс. |
| `pid_kp` | `float` | — | Коэффициент П слежения за профилем, (град/с) / град. |
| `pid_ki` | `float` | — | Коэффициент И слежения за профилем, (град/с) / (град·с). |
| `pid_kd` | `float` | — | Коэффициент Д слежения за профилем, (град/с) / (град/с). |
| `pid_i_max` | `float` | > 0 | Анти-виндап: предел вклада И-члена в команду, град/с. |
| `error_start_correcting_deg` | `float` | > 0 | Порог запуска коррекции, градусы. |
| `error_stop_deg` | `float` | 0 ≤ x ≤ `error_start_correcting_deg` | Порог остановки коррекции, градусы. |
| `brake_at_target_fraction` | `float` | 0.0 ≤ x ≤ 1.0 | Доля тормозного тока при удержании в цели. |
| `limit_min_deg` / `limit_max_deg` | `float` | `limit_max_deg` > `limit_min_deg` | Границы зоны лимитов (жёсткая защита хода), градусы. |
| `working_min_deg` / `working_max_deg` | `float` | внутри зоны лимитов, `max` > `min` | Границы рабочего диапазона (шкала для нормализованного управления), градусы. |
| `homing_mode` | `VESC_Servo_HomingMode_t` | одно из значений enum | Способ определения нуля — см. `VESC_Servo_HomingMode_t` выше. 0 ⇒ `REQUIRED` (прежнее поведение). |
| `limit_switch_pressed_state` | `VESC_CustomSensorState_t` | ≠ `VESC_CUSTOM_SENSOR_NONE` | Какое состояние кастомного статуса вески считать «концевик нажат». Используется `StartHoming()` независимо от `homing_mode`. |
| `homing_seek_speed_deg_s` | `float` | — | Скорость выходного вала на этапе SEEK, град/с. |
| `homing_backoff_speed_deg_s` | `float` | — | Скорость выходного вала на этапе BACKOFF, град/с. |
| `home_position_deg` | `float` | внутри зоны лимитов | Угол выходного вала в момент завершения хоуминга (и в момент `ZERO_AT_BOOT`); не используется `TRUST_ABSOLUTE`. |
| `homing_timeout_ms` | `uint32_t` | 0 ⇒ 15000 | Таймаут на каждый этап хоуминга, мс. |
| `telemetry_timeout_ms` | `uint32_t` | 0 ⇒ 200 | Порог свежести `STATUS_4`/`STATUS_7`, мс. |

### `VESC_Servo_Handle_t`

Хэндл сервы, возвращается `VESC_Servo_Init()`. Публичные поля (только чтение):

| Поле | Тип | Описание |
|---|---|---|
| `vesc` | `VESC_Handle_t*` | Хэндл вески, обслуживаемой этой сервой (`motor_vesc.h`). Команды ему вручную не слать. |
| `telemetry` | `VESC_Servo_Telemetry_t` | Готовый снимок состояния сервы. |
| `state` | `VESC_Servo_State_t` | Текущее общее состояние. |
| `homing_state` | `VESC_Servo_HomingState_t` | Текущий/последний этап хоуминга. |

Остальные поля структуры — внутреннее состояние контура, доступны только для того, чтобы структуру
можно было объявить целиком (без malloc); напрямую не используются.

## Инициализация

### `VESC_Servo_Init`

```c
VESC_Servo_Handle_t *VESC_Servo_Init(const VESC_Servo_Config_t *config);
```

Регистрирует веску (шина/CAN ID/число полюсов из конфига) и серву поверх неё, подписывается на
телеметрию вески. Серва создаётся в `VESC_SERVO_STATE_DISABLED`. Дальнейшее зависит от
`config->homing_mode` (см. [`VESC_Servo_HomingMode_t`](#vesc_servo_homingmode_t)):
`REQUIRED`/`MANUAL_EXTERNAL` — ничего автоматически, ждём явного вызова;
`ZERO_AT_BOOT`/`TRUST_ABSOLUTE` — серва сама перейдёт в `READY` на первой телеметрии от вески.

| Параметр | Описание |
|---|---|
| `config` | Заполненная конфигурация, см. [`VESC_Servo_Config_t`](#vesc_servo_config_t). |

**Возврат:** указатель на `VESC_Servo_Handle_t`, либо `NULL` при ошибке конфигурации (`config ==
NULL`, нефинитное число в любом float-поле, нарушение любого требования из таблицы конфигурации
выше, ошибка регистрации вески в `motor_vesc`, повторная регистрация уже обёрнутой вески, либо
исчерпан `VESC_SERVO_MAX_SERVOS`) — точный список условий см. в `vesc_servo.h`.

## Управление

### `VESC_Servo_StartHoming`

```c
HAL_StatusTypeDef VESC_Servo_StartHoming(VESC_Servo_Handle_t *s);
```

Запускает поиск нуля по концевику. Неблокирующая. Может прервать активную коррекцию — мотор перед
стартом хоуминга останавливается.

| Параметр | Описание |
|---|---|
| `s` | Хэндл сервы. |

**Возврат:** `HAL_OK`; `HAL_ERROR`, если `s == NULL`, телеметрия `STATUS_4` не свежая, либо
`STATUS_7` (концевик) ни разу не приходил или устарел.

### `VESC_Servo_SetPosition`

```c
HAL_StatusTypeDef VESC_Servo_SetPosition(VESC_Servo_Handle_t *s, float target_deg);
```

Задаёт целевой угол выходного вала. Обрезается по зоне лимитов. Неблокирующая, можно вызывать
повторно в любой момент.

| Параметр | Описание |
|---|---|
| `s` | Хэндл сервы. |
| `target_deg` | Целевой угол выходного вала, градусы. `NaN`/`Inf` отклоняются. |

**Возврат:** `HAL_OK`; `HAL_ERROR`, если `s == NULL`, `target_deg` не конечное число, либо серва не
в `VESC_SERVO_STATE_READY`.

### `VESC_Servo_SetPositionNormalized`

```c
HAL_StatusTypeDef VESC_Servo_SetPositionNormalized(VESC_Servo_Handle_t *s, float normalized);
```

Задаёт цель в диапазоне `-1.0..1.0` относительно рабочего диапазона (`working_min_deg` /
`working_max_deg`). Значения вне `-1.0..1.0` линейно экстраполируются, но итог всё равно обрезается
по зоне лимитов. В остальном ведёт себя как `VESC_Servo_SetPosition()`.

| Параметр | Описание |
|---|---|
| `s` | Хэндл сервы. |
| `normalized` | `-1.0..1.0` (либо шире, см. выше). `NaN`/`Inf` отклоняются. |

**Возврат:** `HAL_OK`; `HAL_ERROR`, если `s == NULL`, итоговый угол не конечное число, либо серва не
в `VESC_SERVO_STATE_READY`.

### `VESC_Servo_SetCurrentPosition`

```c
HAL_StatusTypeDef VESC_Servo_SetCurrentPosition(VESC_Servo_Handle_t *s, float actual_position_deg);
```

Ручная калибровка «на лету» без физического хоуминга. После вызова серва переходит в
`VESC_SERVO_STATE_READY`, хоуминг считается пройденным. Может прервать активную коррекцию/хоуминг —
мотор перед этим останавливается.

| Параметр | Описание |
|---|---|
| `s` | Хэндл сервы. |
| `actual_position_deg` | Реальный угол выходного вала прямо сейчас, градусы. `NaN`/`Inf` отклоняются. |

**Возврат:** `HAL_OK`; `HAL_ERROR`, если `s == NULL`, `actual_position_deg` не конечное число, либо
ещё ни разу не приходила телеметрия вески.

### `VESC_Servo_Enable`

```c
HAL_StatusTypeDef VESC_Servo_Enable(VESC_Servo_Handle_t *s);
```

Включает контур позиции после `VESC_Servo_Disable()` без повторного хоуминга, либо выводит из
`VESC_SERVO_STATE_FAULT` (если он вызван потерей связи и телеметрия уже снова свежая).

| Параметр | Описание |
|---|---|
| `s` | Хэндл сервы. |

**Возврат:** `HAL_OK`; `HAL_ERROR`, если `s == NULL`, хоуминг/калибровка ни разу не выполнялись,
либо телеметрия `STATUS_4` сейчас не свежая.

### `VESC_Servo_Disable`

```c
HAL_StatusTypeDef VESC_Servo_Disable(VESC_Servo_Handle_t *s);
```

Немедленно останавливает мотор и переводит серву в `VESC_SERVO_STATE_DISABLED`. Пройденный ранее
хоуминг не сбрасывается. Если вызвана посреди `HOMING` — хоуминг засчитывается как неудавшийся.

| Параметр | Описание |
|---|---|
| `s` | Хэндл сервы. |

**Возврат:** `HAL_OK`; `HAL_ERROR`, если `s == NULL`.

### `VESC_Servo_SetBrakeAtTarget`

```c
HAL_StatusTypeDef VESC_Servo_SetBrakeAtTarget(VESC_Servo_Handle_t *s, uint8_t enabled);
```

Включает/выключает удержание вала тормозным током вески, пока коррекция остановлена гистерезисом.
Сила тормоза берётся из `brake_at_target_fraction` конфига.

| Параметр | Описание |
|---|---|
| `s` | Хэндл сервы. |
| `enabled` | 1 — тормозить заданной силой; 0 — слать скорость 0 (без удержания). |

**Возврат:** `HAL_OK`; `HAL_ERROR`, если `s == NULL`.

### `VESC_Servo_SetTelemetryCallback`

```c
HAL_StatusTypeDef VESC_Servo_SetTelemetryCallback(VESC_Servo_Handle_t *s, VESC_Servo_TelemetryCallback_t callback);
```

Задаёт (или снимает, если `callback == NULL`) обработчик на каждый принятый статусный пакет
телеметрии вески этой сервы.

| Параметр | Описание |
|---|---|
| `s` | Хэндл сервы. |
| `callback` | Функция-обработчик, см. [`VESC_Servo_TelemetryCallback_t`](#vesc_servo_telemetrycallback_t), либо `NULL`. |

**Возврат:** `HAL_OK`; `HAL_ERROR`, если `s == NULL`.

### `VESC_Servo_CheckAlive`

```c
void VESC_Servo_CheckAlive(VESC_Servo_Handle_t *s);
```

Необязательная диагностика: проверяет свежесть `STATUS_4`, и если телеметрия устарела —
останавливает мотор и переводит серву в `VESC_SERVO_STATE_FAULT`. На сам контур позиции не влияет
(он и так продвигается по приходу телеметрии).

| Параметр | Описание |
|---|---|
| `s` | Хэндл сервы. |

**Возврат:** нет (`void`).

## Чтение состояния

### `VESC_Servo_GetPositionDeg`

```c
float VESC_Servo_GetPositionDeg(VESC_Servo_Handle_t *s);
```

Текущий угол выходного вала, градусы. `0.0`, если `s == NULL` или ещё не было ни одного прихода
телеметрии.

### `VESC_Servo_GetTargetDeg`

```c
float VESC_Servo_GetTargetDeg(VESC_Servo_Handle_t *s);
```

Текущая заданная цель, градусы. `0.0`, если `s == NULL`.

### `VESC_Servo_GetTelemetry`

```c
const VESC_Servo_Telemetry_t *VESC_Servo_GetTelemetry(VESC_Servo_Handle_t *s);
```

Указатель на `s->telemetry`. `NULL`, если `s == NULL`.

### `VESC_Servo_IsHomed`

```c
uint8_t VESC_Servo_IsHomed(VESC_Servo_Handle_t *s);
```

1, если хоуминг/калибровка когда-либо успешно завершались. 0 иначе (в т.ч. если `s == NULL`).

### `VESC_Servo_IsAtTarget`

```c
uint8_t VESC_Servo_IsAtTarget(VESC_Servo_Handle_t *s);
```

1, если серва `READY` и коррекция сейчас не активна. 0 иначе (в т.ч. если `s == NULL`).

### `VESC_Servo_GetState`

```c
VESC_Servo_State_t VESC_Servo_GetState(VESC_Servo_Handle_t *s);
```

Текущее общее состояние сервы. `VESC_SERVO_STATE_DISABLED`, если `s == NULL`.

### `VESC_Servo_GetHomingState`

```c
VESC_Servo_HomingState_t VESC_Servo_GetHomingState(VESC_Servo_Handle_t *s);
```

Текущий/последний этап хоуминга. `VESC_SERVO_HOMING_IDLE`, если `s == NULL`.

### `VESC_Servo_ReadLimitSwitch`

```c
uint8_t VESC_Servo_ReadLimitSwitch(VESC_Servo_Handle_t *s);
```

Мгновенное состояние концевика по последней принятой телеметрии. 1 — нажат. 0, если `s == NULL`.

### `VESC_Servo_GetLimitMinDeg` / `GetLimitMaxDeg`

```c
float VESC_Servo_GetLimitMinDeg(VESC_Servo_Handle_t *s);
float VESC_Servo_GetLimitMaxDeg(VESC_Servo_Handle_t *s);
```

Границы зоны лимитов из конфига, градусы. `0.0`, если `s == NULL`.

### `VESC_Servo_GetWorkingMinDeg` / `GetWorkingMaxDeg`

```c
float VESC_Servo_GetWorkingMinDeg(VESC_Servo_Handle_t *s);
float VESC_Servo_GetWorkingMaxDeg(VESC_Servo_Handle_t *s);
```

Границы рабочего диапазона из конфига, градусы. `0.0`, если `s == NULL`.

## Логирование (опционально)

Включается `#define LOGGER_ENABLE_VESC_SERVO` (интеграция с `stm32_logger`, коды
`LOG_CODE_VESC_SERVO_*`) — без него не подключается вовсе. Подробности и полный список кодов — см.
README.md и раздел «ОПЦИОНАЛЬНАЯ ИНТЕГРАЦИЯ С stm32_logger» в `vesc_servo.h`.
