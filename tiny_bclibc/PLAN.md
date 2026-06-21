# tiny_bclibc — план реалізації

Підмножина `bclibc` на чистому C99, **header-only**.  
Логічно сумісна з оригіналом, але без динамічного виділення пам'яті всередині бібліотеки,
без ексепшнів, без логування та без Euler-інтегратора.

**Цільові платформи:** PC (x86-64 / ARM64) · RP2040 (Cortex-M0+) · RP2350 (Cortex-M33 / RISC-V Hazard3) · ESP32-S3 (Xtensa LX7)

---

## 1. Що залишаємо / що прибираємо

| Можливість | bclibc (C++) | tiny_bclibc (C) |
|---|---|---|
| RK4-інтегратор | ✅ | ✅ |
| Euler-інтегратор | ✅ | ❌ |
| Dense output (`BaseTrajSeq`) | ✅ | ❌ |
| Фільтрована траєкторія | ✅ | ✅ (caller-буфер) |
| `integrate_at` (single-point) | ✅ | ✅ |
| `find_apex` | ✅ | ✅ |
| `find_zero_angle` (Ridder's) | ✅ | ✅ |
| `find_max_range` | ✅ | ✅ |
| Власні хендлери | ✅ | ❌ |
| `IntegrateCallable` / `integrate_func` pointer | ✅ | ❌ завжди RK4 |
| Ексепшни | ✅ | ❌ → коди помилок |
| Логування | ✅ | ❌ |
| `scope_guard` | ✅ | ❌ |
| Thread mutex на engine | ✅ | ❌ функції stateless |
| `std::vector` | ✅ | ❌ → `const T* + int32_t size` |
| Управління пам'яттю | внутрішнє | ❌ caller owns everything |
| float / double вибір | ❌ завжди double | ✅ `real_t` via define |
| Окремі `.c` файли | ✅ | ❌ **header-only** |

---

## 2. Цільові платформи — важливі обмеження

### RP2040 (Cortex-M0+)
- Немає апаратного FPU — float і double через software emulation (compiler-rt)
- Double приблизно вдвічі повільніший за float на M0+
- SRAM 264 KB; стек FreeRTOS задачі зазвичай 2–8 KB
- **Рекомендація:** `TBCLIBC_USE_FLOAT`

### RP2350 (Cortex-M33 / RISC-V Hazard3)
- Апаратний FPU для **single-precision** (hardware float, software double)
- SRAM 520 KB
- C11 `_Thread_local` доступний при pico-sdk + FreeRTOS

### ESP32-S3 (Xtensa LX7, dual core)
- Апаратний FPU для **single-precision**
- 512 KB SRAM + опційний PSRAM
- FreeRTOS; `__thread` підтримується в IDF ≥ 5

### PC (x86-64 / ARM64)
- Hardware double FPU; будь-який `real_t`
- `_Thread_local` підтримується скрізь

### Загальні вимоги
- Стандарт **C99** (GCC / Clang / MSVC)
- Без `malloc`/`free` всередині бібліотеки
- Без `stdio.h` у публічних заголовках (тільки `<math.h>`, `<stdint.h>`, `<string.h>`, `<stdarg.h>`)
- Природне вирівнювання структур (ARM Cortex-M0+ не підтримує unaligned access)

---

## 3. Дуальний режим: header-only + compiled (.so/.dll)

Бібліотека підтримує **два режими** через один макрос `TBCLIBC_FUNC`:

| Define при компіляції | `TBCLIBC_FUNC` | Результат |
|---|---|---|
| нічого (default) | `static inline` | header-only, кожна TU — своя копія |
| `TBCLIBC_BUILD_SHARED` | `__attribute__((visibility("default")))` / `__declspec(dllexport)` | компіляція `.so`/`.dll` |
| `TBCLIBC_USE_SHARED` | `extern` / `__declspec(dllimport)` | споживання готового `.so`/`.dll` |

Єдиний `.c` файл `src/tbclibc_impl.c` містить лише:
```c
#define TBCLIBC_BUILD_SHARED
#include "tbclibc.h"
```
Він компілюється тільки коли потрібна зібрана бібліотека. Для embedded і header-only режиму він не потрібен взагалі.

### Дрібні helper-функції (`v3d.h`, `interp.h`)

Завжди `static inline` — вони занадто малі щоб їх експортувати.  
Для них використовується окремий макрос `TBCLIBC_INLINE_FUNC` (завжди `static inline`).

### Буфер помилок у compiled режимі

В header-only: `static` всередині `static inline` функції → per-TU буфер.  
В compiled (`.so`): буфер стає справжнім `TBCLIBC_THREAD_LOCAL` глобалом в `tbclibc_impl.c`  
→ один буфер на програму (або на потік), як і очікується від shared library.

---

## 4. Структура файлів

```
tiny_bclibc/
├── PLAN.md
├── include/
│   ├── tbclibc.h                         # umbrella include
│   └── tiny_bclibc/
│       ├── platform.h                    # real_t, TBCLIBC_FUNC, TBCLIBC_THREAD_LOCAL
│       ├── v3d.h                         # 3D-вектор (оновити double → real_t)
│       ├── base_types.h                  # enum, config, structs + factories
│       ├── interp.h                      # PCHIP / Hermite / linear
│       ├── traj_data.h                   # BaseTrajData, TrajectoryData + factory
│       └── engine.h                      # RK4, filter, zero-finding, публічний API
└── src/
    └── tbclibc_impl.c                    # один рядок: #include "tbclibc.h" для .so/.dll
```

`src/tbclibc_impl.c` не компілюється в header-only режимі.

---

## 5. `platform.h` — `real_t`, `TBCLIBC_FUNC`, thread-local

```c
#ifndef TBCLIBC_PLATFORM_H
#define TBCLIBC_PLATFORM_H

// ── Числовий тип ──────────────────────────────────────────────────
#ifdef TBCLIBC_USE_FLOAT
  typedef float  real_t;
  #define TBCLIBC_SQRT    sqrtf
  #define TBCLIBC_FABS    fabsf
  #define TBCLIBC_ATAN2   atan2f
  #define TBCLIBC_COS     cosf
  #define TBCLIBC_SIN     sinf
  #define TBCLIBC_POW     powf
  #define TBCLIBC_EXP     expf
  #define REAL_C(x)       x##f
#else
  typedef double real_t;
  #define TBCLIBC_SQRT    sqrt
  #define TBCLIBC_FABS    fabs
  #define TBCLIBC_ATAN2   atan2
  #define TBCLIBC_COS     cos
  #define TBCLIBC_SIN     sin
  #define TBCLIBC_POW     pow
  #define TBCLIBC_EXP     exp
  #define REAL_C(x)       x
#endif

// ── Видимість / linkage публічних функцій ─────────────────────────
//
//  (нічого)           → header-only: static inline
//  TBCLIBC_BUILD_SHARED → компіляція .so/.dll: export символи
//  TBCLIBC_USE_SHARED   → споживання .so/.dll: import символи
//
// Дрібні helpers (v3d, interp) завжди TBCLIBC_INLINE_FUNC.

#if defined(TBCLIBC_BUILD_SHARED)
#  ifdef _WIN32
#    define TBCLIBC_FUNC      __declspec(dllexport)
#  else
#    define TBCLIBC_FUNC      __attribute__((visibility("default")))
#  endif
#elif defined(TBCLIBC_USE_SHARED)
#  ifdef _WIN32
#    define TBCLIBC_FUNC      __declspec(dllimport)
#  else
#    define TBCLIBC_FUNC      extern
#  endif
#else
  // Default: header-only
#  define TBCLIBC_FUNC        static inline
#endif

// Завжди inline — для tiny helpers (v3d, interp, утиліти)
#define TBCLIBC_INLINE_FUNC   static inline

// ── Thread-local storage ──────────────────────────────────────────
// Bare-metal (RP2040 без RTOS) → визначити TBCLIBC_NO_THREAD_LOCAL
#ifndef TBCLIBC_THREAD_LOCAL
#  if defined(TBCLIBC_NO_THREAD_LOCAL)
#    define TBCLIBC_THREAD_LOCAL
#  elif defined(_MSC_VER)
#    define TBCLIBC_THREAD_LOCAL __declspec(thread)
#  elif defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
#    define TBCLIBC_THREAD_LOCAL _Thread_local
#  elif defined(__GNUC__) || defined(__clang__)
#    define TBCLIBC_THREAD_LOCAL __thread
#  else
#    define TBCLIBC_THREAD_LOCAL
#  endif
#endif

#endif // TBCLIBC_PLATFORM_H
```

---

## 6. Обробка помилок

Кожна публічна функція повертає `int32_t` (код стану).  
Текст помилки зберігається у thread-local буфері і читається через `tbclibc_last_error()`.

```c
// base_types.h
typedef enum TBCLIBC_Status {
    TBCLIBC_OK                = 0,
    TBCLIBC_ERR_RUNTIME       = 1,   // загальна помилка інтегратора
    TBCLIBC_ERR_OUT_OF_RANGE  = 2,   // ціль за межею максимальної дальності
    TBCLIBC_ERR_ZERO_FINDING  = 3,   // не вдалось знайти кут нульовки
    TBCLIBC_ERR_INTERCEPTION  = 4,   // точка перехоплення не знайдена
    TBCLIBC_ERR_INVALID_ARG   = 5,   // NULL-pointer, calc_step <= 0 тощо
    TBCLIBC_ERR_BUF_TOO_SMALL = 6,   // буфер замалий, але дані частково записані
} TBCLIBC_Status;
```

### `tbclibc_integrate` — семантика виходів

Функція має **два окремих виходи для кількості точок**:

| Параметр | Значення |
|---|---|
| `*out_written` | скільки точок реально записано у `out_buf` |
| `*out_total` | скільки точок було б усього (повний підрахунок) |

```
out_buf == NULL, buf_capacity == 0  →  out_written=0,  out_total=N  (чистий підрахунок)
out_buf != NULL, buf_capacity >= N  →  out_written=N,  out_total=N  (все влізло, TBCLIBC_OK)
out_buf != NULL, buf_capacity <  N  →  out_written=K,  out_total=N  (частково, TBCLIBC_ERR_BUF_TOO_SMALL)
                                        де K = buf_capacity
```

Повернені значення при переповненні:
- `*out_written` = скільки записано (= `buf_capacity`) — явно, без обчислень
- `*out_total` = скільки потрібно — для наступного виклику з правильним буфером
- `*out_reason` = коректна причина зупинки інтеграції
- Буфер містить перші `buf_capacity` точок — дані **не відкидаються**
- Повернутий код = `TBCLIBC_ERR_BUF_TOO_SMALL` (не фатальна, інформаційна)

**Типовий патерн «не знаю розмір» (dual-pass):**
```c
int32_t written, total, reason;

// Прохід 1: підрахунок (без алокацій, один RK4-прохід)
tbclibc_integrate(props, req, NULL, 0, &written, &total, &reason);

// Прохід 2: запис у правильний буфер
TBCLIBC_TrajectoryData buf[total];  // або heap на стороні caller-а
tbclibc_integrate(props, req, buf, total, &written, &total, &reason);
```

**Якщо розмір відомий заздалегідь і буфер достатній — один прохід, `TBCLIBC_OK`.**

### Буфер помилки — різна поведінка залежно від режиму

**Header-only (default):**
```c
// static всередині static inline → per-TU буфер
// Прийнятно: caller перевіряє одразу після виклику в тій самій TU
static inline const char *tbclibc_last_error(void) {
    static TBCLIBC_THREAD_LOCAL char buf[512];
    return buf;
}
static inline void tbclibc__set_error(const char *msg) {
    char *buf = (char *)tbclibc_last_error();
    int i = 0;
    while (msg[i] && i < 511) { buf[i] = msg[i]; i++; }
    buf[i] = '\0';
}
```

**Compiled (.so/.dll) — `TBCLIBC_BUILD_SHARED`:**
```c
// В tbclibc_impl.c: справжній глобальний thread-local буфер
// → один буфер на програму (або на потік), стандартна shared-library поведінка
static TBCLIBC_THREAD_LOCAL char s_tbclibc_error[512];

const char *tbclibc_last_error(void) { return s_tbclibc_error; }
static inline void tbclibc__set_error(const char *msg) { /* strncpy до s_tbclibc_error */ }
```

Щоб обидва варіанти компілювались з одного `engine.h`, визначення `tbclibc_last_error` та
`tbclibc__set_error` охороняється `#ifdef TBCLIBC_BUILD_SHARED` / `#else`.

> `vsnprintf` не використовується — фіксовані рядки-константи для кожного коду.  
> Caller перевіряє `int32_t` код і за потреби форматує число сам.

---

## 7. Модель пам'яті

**Правило: `tiny_bclibc` ніколи не викликає `malloc`/`free`.**

| Ресурс | Власник |
|---|---|
| `TBCLIBC_ShotProps` | caller (стек або статика) |
| `TBCLIBC_CurvePoint[]` (PCHIP-крива) | caller — передається як буфер у `tbclibc_build_shot_props` |
| `TBCLIBC_Wind[]` | caller — pointer зберігається у ShotProps |
| `TBCLIBC_TrajectoryData[]` (output) | caller — передається з `capacity` |
| `TBCLIBC_BaseTrajData` (для `integrate_at`) | caller — один елемент |
| Внутрішній filter state | 3 × `BaseTrajData` **на стеку** функції інтеграції |
| RK4 stage vectors | ~12 × `TBCLIBC_V3dT` **на стеку** RK4-функції |

### Орієнтовний стековий бюджет

| Компонент | double | float |
|---|---|---|
| FilterState (3 × BaseTrajData) | ~216 B | ~108 B |
| RK4 проміжні вектори (k1–k4, pos, vel) | ~288 B | ~144 B |
| Локальні скаляри / адреси | ~64 B | ~64 B |
| **Разом** | **~568 B** | **~316 B** |

Безпечно для RP2040 / RP2350 / ESP32-S3 при типовому стеку FreeRTOS задачі 2–4 KB.

---

## 8. Ключові типи (`base_types.h`)

```c
typedef struct TBCLIBC_Config {
    real_t  cStepMultiplier;
    real_t  cZeroFindingAccuracy;
    real_t  cMinimumVelocity;
    real_t  cMaximumDrop;
    int32_t cMaxIterations;
    real_t  cGravityConstant;
    real_t  cMinimumAltitude;
} TBCLIBC_Config;

typedef struct TBCLIBC_CurvePoint {  // один PCHIP кубічний сегмент
    real_t a, b, c, d;
} TBCLIBC_CurvePoint;

typedef struct TBCLIBC_Atmosphere {
    real_t t0;            // базова температура (°C)
    real_t a0;            // базова висота (ft)
    real_t p0;            // базовий тиск (hPa)
    real_t mach;          // швидкість звуку (fps)
    real_t density_ratio;
    real_t cLowestTempC;
} TBCLIBC_Atmosphere;

typedef struct TBCLIBC_Coriolis {
    real_t  sin_lat, cos_lat, sin_az, cos_az;
    real_t  range_east, range_north;
    real_t  cross_east, cross_north;
    int32_t flat_fire_only;       // 0 = 3D Coriolis, 1 = flat-fire
    real_t  muzzle_velocity_fps;
} TBCLIBC_Coriolis;

typedef struct TBCLIBC_Wind {
    real_t velocity_fps;
    real_t direction_from_rad;
    real_t until_distance_ft;
    real_t max_distance_ft;      // sentinel: 9e9
} TBCLIBC_Wind;

typedef struct TBCLIBC_WindSock {  // живе всередині ShotProps
    const TBCLIBC_Wind *winds;
    int32_t             count;
    int32_t             current_idx;
    real_t              next_range;
    TBCLIBC_V3dT        last_vector;  // кешований вектор поточного сегменту
} TBCLIBC_WindSock;

typedef struct TBCLIBC_ShotProps {
    real_t  bc;
    real_t  muzzle_velocity;
    real_t  weight;
    real_t  diameter;
    real_t  length;
    real_t  stability_coefficient;
    real_t  sight_height;
    real_t  twist;
    real_t  barrel_elevation;
    real_t  barrel_azimuth;
    real_t  cant_cosine;
    real_t  cant_sine;
    real_t  alt0;
    real_t  calc_step;
    int32_t filter_flags;

    TBCLIBC_Atmosphere atmo;
    TBCLIBC_Coriolis   coriolis;
    TBCLIBC_WindSock   wind_sock;

    const TBCLIBC_CurvePoint *curve;     // caller-буфер PCHIP-кривої
    const real_t             *mach_list; // оригінальний Mach-масив caller-а
    int32_t                   curve_count;
} TBCLIBC_ShotProps;

// User-facing (не engine-ready)
typedef struct TBCLIBC_Shot {
    real_t        bc;
    real_t        weight_grain;
    real_t        diameter_inch;
    real_t        length_inch;
    real_t        muzzle_velocity_fps;
    real_t        sight_height_ft;
    real_t        twist_inch;
    real_t        temp_c;
    real_t        pressure_hpa;        // 0 = вакуум
    real_t        altitude_ft;
    real_t        humidity;
    const real_t *mach_data;           // drag table (паралельні масиви)
    const real_t *cd_data;
    int32_t       drag_table_size;
    const TBCLIBC_Wind *winds;
    int32_t       wind_count;
    real_t        look_angle_rad;
    real_t        barrel_elevation_rad;
    real_t        barrel_azimuth_rad;
    real_t        cant_angle_rad;
    real_t        latitude_deg;        // NaN = вимкнути Coriolis
    real_t        azimuth_deg;
    TBCLIBC_Config config;
} TBCLIBC_Shot;
```

Фабрики (`atmosphere_from_conditions`, `coriolis_from_lat_az`, `build_pchip_curve`) —
`static inline` у `base_types.h`.

---

## 9. Тип даних траєкторії (`traj_data.h`)

```c
typedef struct TBCLIBC_BaseTrajData {
    real_t time;
    real_t px, py, pz;
    real_t vx, vy, vz;
    real_t mach;
} TBCLIBC_BaseTrajData;   // 9 × real_t = 72 B (double) / 36 B (float)

typedef struct TBCLIBC_TrajectoryData {
    real_t  time;
    real_t  distance_ft;
    real_t  velocity_fps;
    real_t  mach;
    real_t  height_ft;
    real_t  slant_height_ft;
    real_t  drop_angle_rad;
    real_t  windage_ft;
    real_t  windage_angle_rad;
    real_t  slant_distance_ft;
    real_t  angle_rad;
    real_t  density_ratio;
    real_t  drag;
    real_t  energy_ft_lb;
    real_t  ogw_lb;
    int32_t flag;
} TBCLIBC_TrajectoryData;  // 15 × real_t + int32 = 124 B / 64 B

typedef enum TBCLIBC_InterpKey {
    TBCLIBC_KEY_TIME  = 0,
    TBCLIBC_KEY_MACH  = 1,
    TBCLIBC_KEY_POS_X = 2,
    TBCLIBC_KEY_POS_Y = 3,
    TBCLIBC_KEY_POS_Z = 4,
    TBCLIBC_KEY_VEL_X = 5,
    TBCLIBC_KEY_VEL_Y = 6,
    TBCLIBC_KEY_VEL_Z = 7,
} TBCLIBC_InterpKey;

typedef struct TBCLIBC_TrajectoryRequest {
    real_t  range_limit_ft;
    real_t  range_step_ft;
    real_t  time_step;
    int32_t filter_flags;
} TBCLIBC_TrajectoryRequest;
```

`tbclibc_traj_data_from_props(props, base, flag, out)` — `static inline` фабрика у `traj_data.h`.

---

## 10. `interp.h` — static inline

```c
static inline real_t tbclibc_hermite(
    real_t x,
    real_t xk, real_t xk1,
    real_t yk, real_t yk1,
    real_t mk, real_t mk1);

static inline real_t tbclibc_interpolate3pt(
    real_t x,
    real_t x0, real_t x1, real_t x2,
    real_t y0, real_t y1, real_t y2);

static inline int32_t tbclibc_interpolate2pt(
    real_t x,
    real_t x0, real_t y0,
    real_t x1, real_t y1,
    real_t *result);
```

---

## 11. `engine.h` — публічний API + реалізація

Публічні функції використовують `TBCLIBC_FUNC` — поведінка залежить від режиму збірки.  
Внутрішні мають префікс `tbclibc__` і завжди `static inline`.

```c
// ── Підготовка ───────────────────────────────────────────────────

// Збирає ShotProps з TBCLIBC_Shot.
// curve_buf — caller-allocated, мінімум shot->drag_table_size елементів.
TBCLIBC_FUNC int32_t tbclibc_build_shot_props(
    const TBCLIBC_Shot  *shot,
    TBCLIBC_CurvePoint  *curve_buf,
    TBCLIBC_ShotProps   *out);

// ── Інтеграція ───────────────────────────────────────────────────

// out_buf == NULL, buf_capacity == 0  →  count-only (нічого не пише)
// *out_written — скільки реально записано у out_buf
// *out_total   — скільки було б усього (для dual-pass або перевірки)
// При buf_capacity < total: TBCLIBC_ERR_BUF_TOO_SMALL,
//   out_buf містить перші buf_capacity точок, out_reason коректний
TBCLIBC_FUNC int32_t tbclibc_integrate(
    const TBCLIBC_ShotProps         *props,
    const TBCLIBC_TrajectoryRequest *req,
    TBCLIBC_TrajectoryData          *out_buf,        // NULL = count-only
    int32_t                          buf_capacity,   // 0 = count-only
    int32_t                         *out_written,    // записано у buf
    int32_t                         *out_total,      // всього точок
    int32_t                         *out_reason);

// out_full може бути NULL.
TBCLIBC_FUNC int32_t tbclibc_integrate_at(
    const TBCLIBC_ShotProps *props,
    int32_t                  key,
    real_t                   target_value,
    TBCLIBC_BaseTrajData    *out_raw,
    TBCLIBC_TrajectoryData  *out_full);

// ── Запити ───────────────────────────────────────────────────────

TBCLIBC_FUNC int32_t tbclibc_find_apex(
    const TBCLIBC_ShotProps *props,
    TBCLIBC_TrajectoryData  *out);

TBCLIBC_FUNC int32_t tbclibc_find_zero_angle(
    const TBCLIBC_ShotProps *props,
    real_t                   distance_ft,
    real_t                  *out_angle_rad);

TBCLIBC_FUNC int32_t tbclibc_find_max_range(
    const TBCLIBC_ShotProps *props,
    real_t                   low_deg,
    real_t                   high_deg,
    real_t                  *out_range_ft,
    real_t                  *out_angle_rad);

// ── Утиліти (завжди inline — малі, чисті функції) ────────────────

TBCLIBC_INLINE_FUNC real_t tbclibc_get_correction(real_t distance_ft, real_t offset_ft);
TBCLIBC_INLINE_FUNC real_t tbclibc_calculate_energy(real_t weight_grain, real_t velocity_fps);
TBCLIBC_INLINE_FUNC real_t tbclibc_calculate_ogw(real_t weight_grain, real_t velocity_fps);

TBCLIBC_FUNC const char *tbclibc_last_error(void);
```

### Внутрішні символи в `engine.h`

Внутрішні функції **завжди** `static inline` (навіть у compiled режимі) — вони не є
частиною публічного ABI і не потрапляють у таблицю символів `.so`.

```c
// Filter state — на стеку caller-функції інтеграції
typedef struct {
    TBCLIBC_BaseTrajData buf[3];
    int32_t              n;
} tbclibc__FilterState;

// Головний RK4-цикл — розділяється між integrate / integrate_at / find_zero_angle
// через ctx-callback замість handler-ів
static inline int32_t tbclibc__run_rk4(
    const TBCLIBC_ShotProps         *props,
    const TBCLIBC_TrajectoryRequest *req,
    int32_t (*on_step)(const TBCLIBC_BaseTrajData *pt, void *ctx),
    void    *ctx,
    int32_t *out_reason);

static inline void tbclibc__rk4_acceleration(
    const TBCLIBC_ShotProps *props,
    real_t t,
    TBCLIBC_V3dT pos, TBCLIBC_V3dT vel,
    TBCLIBC_V3dT *out_acc);

static inline void tbclibc__rk4_step(
    const TBCLIBC_ShotProps *props,
    real_t t, real_t dt,
    TBCLIBC_V3dT pos, TBCLIBC_V3dT vel,
    TBCLIBC_V3dT *pos_out, TBCLIBC_V3dT *vel_out);
```

`BCLIBC_ValueGuard` (RAII) замінюється на ручне збереження/відновлення
`barrel_elevation` у `tbclibc_find_zero_angle` та `tbclibc_find_max_range`.

---

## 12. `v3d.h` — оновлення

- `double` → `real_t` у всіх полях і сигнатурах
- Порогові значення: `1e-10` → `REAL_C(1e-10)`, `1e-20` → `REAL_C(1e-20)`  
  (запобігає double-literal warning у float-режимі на строгих компіляторах)
- Всі функції вже `static inline` — без змін

---

## 13. `tbclibc.h` — umbrella

```c
#ifndef TBCLIBC_H
#define TBCLIBC_H

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "tiny_bclibc/platform.h"
#include "tiny_bclibc/v3d.h"
#include "tiny_bclibc/base_types.h"
#include "tiny_bclibc/interp.h"
#include "tiny_bclibc/traj_data.h"
#include "tiny_bclibc/engine.h"

#endif // TBCLIBC_H
```

---

## 14. Порядок реалізації

| # | Статус | Крок | Файл |
|---|---|---|---|
| 1 | ✅ | Platform defines: `real_t`, `TBCLIBC_FUNC`, `TBCLIBC_THREAD_LOCAL` | `platform.h` |
| 2 | ✅ | Оновити v3d (double → real_t, REAL_C) | `v3d.h` |
| 3 | ✅ | Base types: enum, structs, константи | `base_types.h` |
| 4 | ✅ | Base types: фабрики (atmosphere, coriolis, PCHIP build, windsock) | `base_types.h` |
| 5 | ✅ | Interpolation | `interp.h` |
| 6 | ✅ | Traj data: structs + `TBCLIBC_TrajectoryData_from_props` | `traj_data.h` |
| 7 | ✅ | Engine: error buffer (header-only / compiled) | `engine.h` |
| 8 | ✅ | Engine: `tbclibc__Essential` + `tbclibc__run_rk4` (RK4 loop) | `engine.h` |
| 9 | ✅ | Engine: `tbclibc_integrate` + filter (range/time/apex/mach/zero) | `engine.h` |
| 10 | ✅ | Engine: `tbclibc_integrate_at` (SinglePointHandler) | `engine.h` |
| 11 | ✅ | Engine: `tbclibc_find_apex` | `engine.h` |
| 12 | ✅ | Engine: `tbclibc_find_zero_angle` (Ridder's) + `tbclibc_find_max_range` (golden-section) | `engine.h` |
| 13 | ✅ | Engine: `tbclibc_build_shot_props` + утиліти | `engine.h` |
| 14 | ✅ | Umbrella | `tbclibc.h` |
| 15 | ✅ | Impl TU для compiled режиму | `src/tbclibc_impl.c` |
| 16 | ✅ | CMake: версіонування з git + INTERFACE/SHARED/STATIC targets + install/export | `CMakeLists.txt` |
| 17 | ✅ | `version.h.in`, `tbclibc-config.cmake.in`, `Makefile` | кореневі файли |
| 18 | ✅ | Тест ідентичності bclibc ↔ tbclibc (C++17 harness) | `tests/test_identity.cpp` |
| 19 | 🔲 | `tbclibc_shotprops_size()` в `engine.h` | `engine.h` |
| 20 | 🔲 | ctypes-рушій `TBClibcEngine` → `EngineProtocol` | `python/tbclibc_engine.py` |
| 21 | 🔲 | Запуск `pytest` py_ballisticcalc з `TBClibcEngine` | `tests/conftest.py` |

---

## 15. CMake

```cmake
# ── Спільні налаштування ─────────────────────────────────────────
set(TBCLIBC_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

option(TBCLIBC_USE_FLOAT        "Use float instead of double"             OFF)
option(TBCLIBC_NO_THREAD_LOCAL  "Disable _Thread_local (bare-metal)"      OFF)

# ── Режим 1: header-only (INTERFACE) — default ───────────────────
add_library(tiny_bclibc INTERFACE)

target_include_directories(tiny_bclibc INTERFACE ${TBCLIBC_INCLUDE_DIR})
target_compile_features(tiny_bclibc INTERFACE c_std_99)

if(NOT PICO_SDK AND NOT ESP_IDF)
    target_link_libraries(tiny_bclibc INTERFACE m)   # -lm на Linux/GCC
endif()

if(TBCLIBC_USE_FLOAT)
    target_compile_definitions(tiny_bclibc INTERFACE TBCLIBC_USE_FLOAT)
endif()
if(TBCLIBC_NO_THREAD_LOCAL)
    target_compile_definitions(tiny_bclibc INTERFACE TBCLIBC_NO_THREAD_LOCAL)
endif()

# ── Режим 2: compiled .so / .dll (опціонально) ───────────────────
option(TBCLIBC_BUILD_COMPILED "Build tiny_bclibc as shared library (.so/.dll)" OFF)

if(TBCLIBC_BUILD_COMPILED)
    add_library(tiny_bclibc_shared SHARED src/tbclibc_impl.c)

    target_include_directories(tiny_bclibc_shared PUBLIC ${TBCLIBC_INCLUDE_DIR})
    target_compile_features(tiny_bclibc_shared PUBLIC c_std_99)
    target_link_libraries(tiny_bclibc_shared PRIVATE m)

    # При компіляції самої .so: TBCLIBC_BUILD_SHARED → export
    # Для споживачів .so:       TBCLIBC_USE_SHARED   → import
    target_compile_definitions(tiny_bclibc_shared
        PRIVATE   TBCLIBC_BUILD_SHARED
        INTERFACE TBCLIBC_USE_SHARED
    )

    if(TBCLIBC_USE_FLOAT)
        target_compile_definitions(tiny_bclibc_shared PUBLIC TBCLIBC_USE_FLOAT)
    endif()
    if(TBCLIBC_NO_THREAD_LOCAL)
        target_compile_definitions(tiny_bclibc_shared PUBLIC TBCLIBC_NO_THREAD_LOCAL)
    endif()

    # Приховати не-API символи (internal static inline вже не потрапляють,
    # але явний прапор усуває будь-які випадкові витоки)
    if(NOT MSVC)
        target_compile_options(tiny_bclibc_shared PRIVATE -fvisibility=hidden)
    endif()
endif()
```

### `src/tbclibc_impl.c`
```c
// Єдина TU, що компілює всі функції для .so/.dll
#define TBCLIBC_BUILD_SHARED
#include "tbclibc.h"
```

### Рекомендації per-platform

| Платформа | `TBCLIBC_USE_FLOAT` | `TBCLIBC_NO_THREAD_LOCAL` | `TBCLIBC_BUILD_COMPILED` |
|---|---|---|---|
| PC | OFF | OFF | за потребою |
| RP2040 | **ON** | **ON** (bare-metal) | OFF |
| RP2350 | **ON** | OFF (FreeRTOS + C11) | OFF |
| ESP32-S3 (IDF ≥ 5) | **ON** | OFF | OFF |

---

## 16. `bclibc/ffi` vs `tiny_bclibc`

| | `bclibc/ffi` | `tiny_bclibc` |
|---|---|---|
| Реалізація | C++ engine + thin C wrapper | чистий C, header-only |
| Output-пам'ять | ffi виділяє heap, caller `free()` | caller передає свій буфер |
| Залежності | C++ runtime, STL | `<math.h>`, `<stdint.h>`, `<string.h>` |
| float / double | завжди double | `real_t` via define |
| Dense trajectory | ✅ | ❌ |
| Euler | ✅ | ❌ |
| Embedded-ready | ❌ | ✅ |
| Dart FFI / ffigen | ✅ (основний) | опційно |

---

## 17. MicroPython native модуль (майбутній шар поверх `tiny_bclibc`)

`tiny_bclibc` — це бібліотека. MicroPython native модуль — окремий шар-обгортка,
який перекладає MPY-типи в C-структури та викликає `tbclibc_*` функції.

### Чому стандартний `malloc`/`free` небезпечний у native MPY

MicroPython має власний GC-heap, ізольований від системного heap.
GC може переміщувати об'єкти під час збірки, тому raw C-pointer на MPY-об'єкт
може стати dangling між викликами. На RP2040 / ESP32 системний `malloc` взагалі
може не бути слінкованим або брати з того ж обмеженого пулу.

### Правильні алокатори в native MPY

```c
#include "py/runtime.h"
#include "py/gc.h"

m_malloc(size)                      // alloc з MP-heap
m_free(ptr)                         // free з MP-heap
m_realloc(ptr, new_size)            // realloc на MP-heap
gc_alloc(size, ALLOC_FLAG_HAS_FINALIZER)  // для об'єктів з деструктором
m_new(type, n)                      // типізований alloc: m_new(TBCLIBC_TrajectoryData, n)
m_del(type, ptr, n)                 // типізований free
```

### Як `tiny_bclibc` вписується в MPY

Оскільки `tiny_bclibc` **ніколи не алокує** — native MPY модуль повністю контролює пам'ять:

```c
// Native MPY обгортка для integrate()
STATIC mp_obj_t mpy_tbclibc_integrate(mp_obj_t shot_obj, mp_obj_t req_obj) {
    TBCLIBC_ShotProps  props;   // на стеку MPY-задачі
    TBCLIBC_CurvePoint curve_buf[MAX_DRAG_POINTS];  // на стеку або static

    tbclibc_build_shot_props(/* ... */, curve_buf, &props);

    // Pass 1: підрахунок — нуль алокацій
    int32_t written, total, reason;
    tbclibc_integrate(&props, &req, NULL, 0, &written, &total, &reason);

    // Pass 2: виділяємо через MPY allocator і пишемо
    TBCLIBC_TrajectoryData *buf = m_new(TBCLIBC_TrajectoryData, total);
    tbclibc_integrate(&props, &req, buf, total, &written, &total, &reason);

    // Пакуємо buf у MPY list/tuple/bytearray і повертаємо
    // ...
    m_del(TBCLIBC_TrajectoryData, buf, total);
    return result;
}
```

### Режим збірки залежно від MPY-порту

На **всіх** платформах native MPY модуль є динамічно завантажуваним файлом
(`.mpy` з native code на embedded, `.so` на Unix-порті).
У **обох** випадках `tiny_bclibc` використовується у **header-only** режимі —
всі `static inline` функції компілюються **в тіло самого native модуля**.
Окремий `tiny_bclibc_shared.so` для MPY **не потрібен**.

| MPY порт | Формат native модуля | Режим tiny_bclibc | `TBCLIBC_USE_FLOAT` | `TBCLIBC_NO_THREAD_LOCAL` |
|---|---|---|---|---|
| Unix (Linux/macOS) | `.mpy` native (динамічний) | header-only | OFF | OFF |
| RP2040 | `.mpy` native (динамічний) | header-only | **ON** | **ON** |
| RP2350 | `.mpy` native (динамічний) | header-only | **ON** | **ON** |
| ESP32-S3 | `.mpy` native (динамічний) | header-only | **ON** | **ON** |

`TBCLIBC_BUILD_COMPILED` (окрема `.so`) — тільки для **не-MPY** споживачів:
standalone C/C++ застосунки на PC, Dart FFI, або інші FFI-прив'язки.

> На embedded: native `.mpy` модуль компілюється toolchain-ом MicroPython
> (gcc/clang + `mpy-cross` або `USER_C_MODULES`), після чого файл копіюється
> на flash-filesystem пристрою. `import bclibc` завантажує його динамічно у runtime.

### Обмеження пам'яті на embedded MPY

| Платформа | MPY heap (типово) | Одна точка `TrajectoryData` (float) |
|---|---|---|
| RP2040 | ~192 KB | 64 B |
| RP2350 | ~400 KB | 64 B |
| ESP32-S3 | ~256 KB (без PSRAM) | 64 B |

При 1000 точок: 64 KB — прийнятно. При 5000 точок: 320 KB — критично для RP2040.
Dual-pass дозволяє caller-у вирішити чи вистачає пам'яті **до** другого проходу.

---

## 18. Тест ідентичності розрахунків bclibc ↔ tbclibc

**Мета:** довести математичну та логічну сумісність — при однакових вхідних даних кожна
функція `tbclibc` дає той самий результат що і відповідний виклик оригінального C++ `bclibc`.

### 18.1 Структура тесту

```
tests/
├── test_identity.cpp       # main — запускає усі кейси
├── fixture.hpp             # спільні вхідні дані (Shot, Winds, DragTable)
├── compare.hpp             # функції порівняння з tolerance
└── CMakeLists.txt          # підключає bclibc та tbclibc::headers
```

`test_identity.cpp` компілюється як **C++17** — щоб одночасно підключити
C++ `bclibc` (через `bclibc/engine.hpp`) та C `tbclibc` (через `tbclibc.h`).
Обидві бібліотеки отримують **одні й ті самі** double-значення, порівнюються
кожне поле результату.

### 18.2 Вхідні фікстури

| Фікстура | Опис |
|---|---|
| `G7_BASIC` | куля G7 BC=0.307, MV=3000fps, без вітру, без Коріолісу |
| `G7_WIND` | та сама куля, 10mph бічний вітер, 2 сегменти |
| `G7_CORIOLIS` | lat=45°, az=0° (північ), 3D Коріоліс |
| `G7_FLOAT` | той самий G7_BASIC але `real_t = float` (окремий прохід) |
| `SHORT_RANGE` | calc_step=0.5, range=100yd — перевірка boundary |

Drag-таблиця: стандартна G7 (точки 0.0–5.0 Mach) з `bclibc/drag_tables.hpp`.

### 18.3 Функції що перевіряються

| Функція tbclibc | Оригінал bclibc | Що порівнюється |
|---|---|---|
| `tbclibc_integrate` | `Engine::integrate` | кожна точка: всі 15 полів `TrajectoryData` + `flag` |
| `tbclibc_integrate_at` | `Engine::integrate_at` | всі поля `BaseTrajData` (raw) + `TrajectoryData` (full) |
| `tbclibc_find_apex` | `Engine::find_apex` | всі поля `TrajectoryData` |
| `tbclibc_find_zero_angle` | `Engine::find_zero_angle` | кут в радіанах |
| `tbclibc_find_max_range` | `Engine::find_max_range` | дальність (ft) + кут (rad) |

### 18.4 Допустимі відхилення

```cpp
// compare.hpp
static const double ABS_TOL = 1e-9;   // поля позиції/часу (ft, s)
static const double REL_TOL = 1e-9;   // поля швидкості, mach, drag, energy
static const double ANGLE_TOL = 1e-9; // кути (rad)

// Для float-режиму: tbclibc_float vs bclibc_double → ширший допуск
static const float  FLOAT_REL_TOL = 1e-4f;  // ~7 знаків float vs 15 double
```

> Ідеальна мета — **бітово ідентичні** результати в double-режимі.
> Єдине допустиме відхилення — округлення при конвертації `double` → `float`
> у float-режимі.

### 18.5 Порядок запуску тесту

```bash
# З кореня репозиторію:
mkdir -p build_test && cd build_test
cmake .. -DTBCLIBC_BUILD_IDENTITY_TEST=ON -DCMAKE_BUILD_TYPE=Debug
make -j$(nproc)
./tests/test_identity
```

Або через `tiny_bclibc/Makefile`:
```bash
make test-identity   # (target додати у Makefile)
```

### 18.6 Критерій «зроблено»

- [ ] Усі 5 функцій пройшли на фікстурах `G7_BASIC`, `G7_WIND`, `G7_CORIOLIS`
- [ ] Відхилення ≤ `ABS_TOL` для всіх полів у double-режимі
- [ ] Float-режим (G7_FLOAT) проходить з `FLOAT_REL_TOL`
- [ ] Тест підключається до CI (GitHub Actions) разом з основним bclibc

---

## 19. ctypes-рушій для py_ballisticcalc (TBClibcEngine)

**Мета:** підсунути `libtbclibc.so` як рушій у вже наявний тест-харнес `py_ballisticcalc`.
Якщо всі `pytest` тести проходять — математична ідентичність доведена без написання
нового тест-коду.

### 19.1 Структура файлів

```
tiny_bclibc/python/
├── __init__.py
└── tbclibc_engine.py          # ctypes wrapper → EngineProtocol

~/pyproj/py-ballisticcalc/
└── tests/
    └── conftest.py            # додати TBClibcEngine до parametrize
```

### 19.2 ctypes структури

Відображають C-структури `tbclibc.h` один-до-одного.

```python
import ctypes, math, os

c_real = ctypes.c_double   # або c_float при TBCLIBC_USE_FLOAT

class TBCLIBC_Config(ctypes.Structure):
    _fields_ = [
        ("cStepMultiplier",      c_real),
        ("cZeroFindingAccuracy", c_real),
        ("cMinimumVelocity",     c_real),
        ("cMaximumDrop",         c_real),
        ("cMaxIterations",       ctypes.c_int32),
        ("cGravityConstant",     c_real),
        ("cMinimumAltitude",     c_real),
    ]

class TBCLIBC_Wind(ctypes.Structure):
    _fields_ = [
        ("velocity_fps",       c_real),
        ("direction_from_rad", c_real),
        ("until_distance_ft",  c_real),
        ("max_distance_ft",    c_real),
    ]

class TBCLIBC_Shot(ctypes.Structure):
    _fields_ = [
        ("bc",                   c_real),
        ("weight_grain",         c_real),
        ("diameter_inch",        c_real),
        ("length_inch",          c_real),
        ("muzzle_velocity_fps",  c_real),
        ("sight_height_ft",      c_real),
        ("twist_inch",           c_real),
        ("temp_c",               c_real),
        ("pressure_hpa",         c_real),
        ("altitude_ft",          c_real),
        ("humidity",             c_real),
        ("mach_data",            ctypes.POINTER(c_real)),
        ("cd_data",              ctypes.POINTER(c_real)),
        ("drag_table_size",      ctypes.c_int32),
        ("winds",                ctypes.POINTER(TBCLIBC_Wind)),
        ("wind_count",           ctypes.c_int32),
        ("look_angle_rad",       c_real),
        ("barrel_elevation_rad", c_real),
        ("barrel_azimuth_rad",   c_real),
        ("cant_angle_rad",       c_real),
        ("latitude_deg",         c_real),   # NaN = вимкнути Коріоліс
        ("azimuth_deg",          c_real),
        ("config",               TBCLIBC_Config),
    ]

class TBCLIBC_TrajectoryData(ctypes.Structure):
    _fields_ = [
        ("time",              c_real), ("distance_ft",       c_real),
        ("velocity_fps",      c_real), ("mach",              c_real),
        ("height_ft",         c_real), ("slant_height_ft",   c_real),
        ("drop_angle_rad",    c_real), ("windage_ft",        c_real),
        ("windage_angle_rad", c_real), ("slant_distance_ft", c_real),
        ("angle_rad",         c_real), ("density_ratio",     c_real),
        ("drag",              c_real), ("energy_ft_lb",      c_real),
        ("ogw_lb",            c_real), ("flag",              ctypes.c_int32),
    ]

class TBCLIBC_TrajectoryRequest(ctypes.Structure):
    _fields_ = [
        ("range_limit_ft", c_real),
        ("range_step_ft",  c_real),
        ("time_step",      c_real),
        ("filter_flags",   ctypes.c_int32),
    ]
```

`TBCLIBC_ShotProps` — великий внутрішній struct. У Python він непотрібний напряму:
`tbclibc_build_shot_props` записує його у caller-буфер. Розмір отримуємо через
єдину допоміжну функцію в `.so`:

```c
/* додати в engine.h */
TBCLIBC_FUNC size_t tbclibc_shotprops_size(void) {
    return sizeof(TBCLIBC_ShotProps);
}
```

```python
_sz = lib.tbclibc_shotprops_size()
props_buf = ctypes.create_string_buffer(_sz)  # opaque blob
```

### 19.3 Ключові перетворення Shot → TBCLIBC_Shot

```python
from py_ballisticcalc.unit import Distance, Velocity, Weight, Angular, Pressure, Temperature

def _shot_to_c(shot: Shot, config: BaseEngineConfig) -> tuple[TBCLIBC_Shot, list, list, list]:
    dm = shot.ammo.dm
    atmo = shot.atmo

    # drag table → паралельні масиви (mach_data, cd_data)
    mach_arr = (c_real * len(dm.drag_table))(*[p["Mach"] for p in dm.drag_table])
    cd_arr   = (c_real * len(dm.drag_table))(*[p["CD"]   for p in dm.drag_table])

    # winds
    winds_py = shot.winds or []
    winds_arr = (TBCLIBC_Wind * max(len(winds_py), 1))()
    for i, w in enumerate(winds_py):
        winds_arr[i].velocity_fps       = w.velocity >> Velocity.FPS
        winds_arr[i].direction_from_rad = w.direction_from >> Angular.Radian
        winds_arr[i].until_distance_ft  = (w.until_distance >> Distance.Foot
                                            if w.until_distance else 9e9)
        winds_arr[i].max_distance_ft    = 9e9

    mv = shot.ammo.get_velocity_for_temp(atmo.powder_temp) >> Velocity.FPS

    cfg = TBCLIBC_Config(
        cStepMultiplier      = config.cStepMultiplier,
        cZeroFindingAccuracy = config.cZeroFindingAccuracy,
        cMinimumVelocity     = config.cMinimumVelocity,
        cMaximumDrop         = config.cMaximumDrop,
        cMaxIterations       = config.cMaxIterations,
        cGravityConstant     = config.cGravityConstant,
        cMinimumAltitude     = config.cMinimumAltitude,
    )

    lat = shot.latitude  if shot.latitude  is not None else math.nan
    az  = shot.azimuth   if shot.azimuth   is not None else math.nan

    s = TBCLIBC_Shot(
        bc                   = dm.BC,
        weight_grain         = dm.weight >> Weight.Grain,
        diameter_inch        = dm.diameter >> Distance.Inch,
        length_inch          = (dm.length >> Distance.Inch) if dm.length else 0.0,
        muzzle_velocity_fps  = mv,
        sight_height_ft      = shot.weapon.sight_height >> Distance.Foot,
        twist_inch           = (shot.weapon.twist >> Distance.Inch) if shot.weapon.twist else 0.0,
        temp_c               = atmo._t0,
        pressure_hpa         = 0.0 if atmo.density_ratio == 0.0 else atmo._p0,
        altitude_ft          = atmo._a0,
        humidity             = atmo.humidity,
        mach_data            = mach_arr,
        cd_data              = cd_arr,
        drag_table_size      = len(dm.drag_table),
        winds                = winds_arr,
        wind_count           = len(winds_py),
        look_angle_rad       = shot.look_angle >> Angular.Radian,
        barrel_elevation_rad = shot.barrel_elevation >> Angular.Radian,
        barrel_azimuth_rad   = shot.barrel_azimuth >> Angular.Radian,
        cant_angle_rad       = shot.cant_angle >> Angular.Radian,
        latitude_deg         = lat,
        azimuth_deg          = az,
        config               = cfg,
    )
    return s, mach_arr, cd_arr, winds_arr   # тримаємо живими до кінця виклику
```

### 19.4 Основний клас TBClibcEngine

```python
from py_ballisticcalc.engines.base_engine import BaseEngineConfigDict, create_base_engine_config
from py_ballisticcalc.trajectory_data import HitResult, TrajectoryData, TrajFlag
from py_ballisticcalc.unit import Angular, Distance, Velocity, Weight
from py_ballisticcalc.shot import Shot, ShotProps
from py_ballisticcalc.exceptions import ZeroFindingError, OutOfRangeError

TBCLIBC_OK               = 0
TBCLIBC_ERR_ZERO_FINDING = 3
TBCLIBC_ERR_OUT_OF_RANGE = 2

class TBClibcEngine:
    """EngineProtocol implementation backed by libtbclibc.so via ctypes."""

    APEX_IS_MAX_RANGE_RADIANS = math.pi / 2 * 0.99   # same as BaseIntegrationEngine

    def __init__(self, config: BaseEngineConfigDict | None = None, lib_path: str | None = None):
        if lib_path is None:
            lib_path = _find_lib()
        self._lib = _load_lib(lib_path)
        self._config = create_base_engine_config(config)
        self._props_size = self._lib.tbclibc_shotprops_size()
        self._curve_size = ctypes.sizeof(TBCLIBC_CurvePoint)  # додати struct

    def _build_props(self, shot: Shot):
        s, *refs = _shot_to_c(shot, self._config)
        curve_buf = (TBCLIBC_CurvePoint * s.drag_table_size)()
        props_buf = ctypes.create_string_buffer(self._props_size)
        rc = self._lib.tbclibc_build_shot_props(
            ctypes.byref(s), curve_buf, props_buf)
        if rc != TBCLIBC_OK:
            raise RuntimeError(self._lib.tbclibc_last_error().decode())
        return props_buf, curve_buf, refs  # тримаємо curve_buf живим

    def integrate(self, shot_info, max_range, dist_step=None, time_step=0.0,
                  filter_flags=TrajFlag.NONE, **kwargs) -> HitResult:
        props_buf, curve_buf, refs = self._build_props(shot_info)
        req = TBCLIBC_TrajectoryRequest(
            range_limit_ft = max_range >> Distance.Foot,
            range_step_ft  = (dist_step >> Distance.Foot) if dist_step else 0.0,
            time_step      = time_step,
            filter_flags   = int(filter_flags),
        )
        written = ctypes.c_int32(0)
        total   = ctypes.c_int32(0)
        reason  = ctypes.c_int32(0)
        # Pass 1: count
        self._lib.tbclibc_integrate(props_buf, ctypes.byref(req),
                                     None, 0,
                                     ctypes.byref(written), ctypes.byref(total),
                                     ctypes.byref(reason))
        n = total.value
        buf = (TBCLIBC_TrajectoryData * n)()
        # Pass 2: fill
        self._lib.tbclibc_integrate(props_buf, ctypes.byref(req),
                                     buf, n,
                                     ctypes.byref(written), ctypes.byref(total),
                                     ctypes.byref(reason))
        trajectory = [_traj_to_py(buf[i]) for i in range(written.value)]
        props = ShotProps.from_shot(shot_info)
        return HitResult(trajectory=trajectory, props=props,
                         reason=_reason_str(reason.value))

    def zero_angle(self, shot_info: Shot, distance: Distance) -> Angular:
        props_buf, curve_buf, refs = self._build_props(shot_info)
        angle = c_real(0.0)
        rc = self._lib.tbclibc_find_zero_angle(
            props_buf, distance >> Distance.Foot, ctypes.byref(angle))
        if rc == TBCLIBC_ERR_ZERO_FINDING:
            raise ZeroFindingError(self._lib.tbclibc_last_error().decode())
        if rc == TBCLIBC_ERR_OUT_OF_RANGE:
            raise OutOfRangeError(self._lib.tbclibc_last_error().decode())
        return Angular(angle.value, Angular.Radian)
```

### 19.5 Підключення до pytest

```python
# ~/pyproj/py-ballisticcalc/tests/conftest.py  (додати)
import sys
sys.path.insert(0, "/path/to/tiny_bclibc/python")
from tbclibc_engine import TBClibcEngine

# У list pytest параметрів для loaded_engine_instance:
# params=[..., TBClibcEngine, ...]
```

Або як окремий conftest у `tiny_bclibc/tests/`:
```python
# tiny_bclibc/tests/conftest.py
import pytest
from tiny_bclibc.python.tbclibc_engine import TBClibcEngine

@pytest.fixture(params=[TBClibcEngine])
def loaded_engine_instance(request):
    return request.param
```

і запуск:
```bash
cd ~/pyproj/py-ballisticcalc
pytest tests/test_trajectory.py tests/test_zeros.py \
    --co --engine tbclibc  # або через conftest override
```

### 19.6 Критерій «зроблено»

- [ ] `tbclibc_shotprops_size()` додано в `engine.h`
- [ ] `tiny_bclibc/python/tbclibc_engine.py` реалізовано та компілюється без помилок
- [ ] `pytest tests/test_trajectory.py` проходить з `TBClibcEngine` (tolerance ≤ 0.1% від bclibc)
- [ ] `pytest tests/test_zeros.py` проходить
- [ ] `pytest tests/test_trajectory.py -k G7` ← конкретний маркер для CI
