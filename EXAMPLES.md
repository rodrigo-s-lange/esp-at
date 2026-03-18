# esp_at - Examples

## 1) Minimal setup

```c
#include "esp_err.h"
#include "esp_at.h"

void app_main(void)
{
    ESP_ERROR_CHECK(esp_at_init(false));

    // Built-ins now work:
    // AT, AT+MAC, AT+VER, AT+VERSION, AT+FREE, AT+RESET, AT+REBOOT
}
```

## 2) Simple callback with `esp_at_add`

```c
#include "esp_err.h"
#include "esp_at.h"

static void func_helloworld(void)
{
    AT(B "hello world");
}

void app_main(void)
{
    ESP_ERROR_CHECK(esp_at_init(false));
    ESP_ERROR_CHECK(esp_at_add("HW", func_helloworld));
}
```

## 3) Command with parameter

```c
#include <errno.h>
#include <stdlib.h>
#include "esp_err.h"
#include "esp_at.h"

static int s_setpoint = 0;

static void cmd_setpoint(const char *param)
{
    if (param == NULL) {
        AT("SETPOINT=%d", s_setpoint);
        return;
    }

    char *end = NULL;
    errno = 0;
    long v = strtol(param, &end, 10);
    if (errno != 0 || end == param || *end != '\0') {
        AT(R "ERROR: invalid parameter");
        return;
    }

    if (v < 0 || v > 100) {
        AT(R "ERROR: range 0..100");
        return;
    }

    s_setpoint = (int)v;
    AT(G "OK");
}

void app_main(void)
{
    ESP_ERROR_CHECK(esp_at_init(false));
    ESP_ERROR_CHECK(esp_at_register_cmd("AT+SETPOINT", cmd_setpoint));
}
```

## 4) Utility command: `AT+MILLIS`

```c
#include <inttypes.h>
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_at.h"

static void cmd_millis(void)
{
    uint64_t ms = (uint64_t)(esp_timer_get_time() / 1000ULL);
    AT(C "millis: %" PRIu64, ms);
}

void app_main(void)
{
    ESP_ERROR_CHECK(esp_at_init(false));
    ESP_ERROR_CHECK(esp_at_add("MILLIS", cmd_millis));
}
```

## 5) With internal AT logs enabled

```c
void app_main(void)
{
    ESP_ERROR_CHECK(esp_at_init(true));
}
```

## 6) WebTerm integration

```c
ESP_ERROR_CHECK(esp_at_init(false));
ESP_ERROR_CHECK(esp_network_init());
ESP_ERROR_CHECK(esp_webterm_init(6000));
```
