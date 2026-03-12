# esp_at - Exemplos de uso

Este arquivo mostra formas praticas de usar a lib `esp_at` no `main` sem precisar alterar o parser.

## 1) Setup minimo

```c
#include "esp_err.h"
#include "esp_at.h"

void app_main(void)
{
    ESP_ERROR_CHECK(esp_at_init());

    // A partir daqui, comandos built-in ja funcionam:
    // AT, AT+MAC, AT+VER, AT+VERSION, AT+RESET, AT+REBOOT
}
```

## 2) Callback simples (sem parametro) com `esp_at_add`

Objetivo: criar `AT+HW` chamando uma funcao `void func(void)`.

```c
#include "esp_err.h"
#include "esp_at.h"

static void func_helloworld(void)
{
    AT(B "hello world");
}

void app_main(void)
{
    ESP_ERROR_CHECK(esp_at_init());
    ESP_ERROR_CHECK(esp_at_add("HW", func_helloworld));
    // Tambem aceita: esp_at_add("AT+HW", func_helloworld);
}
```

Teste no terminal:

```text
AT+HW
hello world
```

## 3) Comando com parametro (`AT+SETPOINT=25`)

Para comandos com parametro, use `esp_at_register_cmd("AT+SETPOINT", handler)`.

```c
#include <stdlib.h>
#include <errno.h>
#include "esp_err.h"
#include "esp_at.h"

static int s_setpoint = 0;

static void cmd_setpoint(const char *param)
{
    // Consulta: AT+SETPOINT
    if (param == NULL) {
        AT("SETPOINT=%d", s_setpoint);
        return;
    }

    // Escrita: AT+SETPOINT=25
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
    ESP_ERROR_CHECK(esp_at_init());
    ESP_ERROR_CHECK(esp_at_register_cmd("AT+SETPOINT", cmd_setpoint));
}
```

Teste no terminal:

```text
AT+SETPOINT
SETPOINT=0

AT+SETPOINT=25
OK

AT+SETPOINT
SETPOINT=25
```

## 4) Exemplo util: `AT+MILLIS`

```c
#include <inttypes.h>
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_at.h"

static void cmd_millis(void)
{
    uint64_t ms = (uint64_t)(esp_timer_get_time() / 1000ULL);
    AT(C "millis: %" PRIu64, ms);
}

void app_main(void)
{
    ESP_ERROR_CHECK(esp_at_init());
    ESP_ERROR_CHECK(esp_at_add("MILLIS", cmd_millis));
}
```

Teste:

```text
AT+MILLIS
millis: 12345
```

## 5) Regras praticas

- Use `esp_at_add(...)` para comandos sem parametro.
- Use `esp_at_register_cmd(...)` para comandos com ou sem parametro.
- Se o usuario enviar `AT+CMD=...`, o handler recebe `param != NULL`.
- Se o usuario enviar `AT+CMD`, o handler recebe `param == NULL`.
- `AT(...)` fecha linha com `CRLF`; `at(...)` escreve sem quebra de linha.
- `AT_I/W/E/D` geram logs com timestamp.

## 6) Integracao com WebTerm

Se o projeto estiver com WebTerm habilitado (via `esp_webterm_init(port)`), os mesmos comandos acima funcionam via serial e via pagina web.

Fluxo tipico no `main`:

```c
ESP_ERROR_CHECK(esp_at_init());
ESP_ERROR_CHECK(esp_network_init(...));   // conforme seu projeto
ESP_ERROR_CHECK(esp_webterm_init(6000));  // opcional

ESP_ERROR_CHECK(esp_at_add("HW", func_helloworld));
ESP_ERROR_CHECK(esp_at_register_cmd("AT+SETPOINT", cmd_setpoint));
```

