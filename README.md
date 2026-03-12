# esp_at

AT command parser over UART0 for the ESP32 family, with ANSI color output and dynamic command registration.

## Features

- Dynamic thread-safe command registration at runtime
- ANSI color tokens via compile-time string concatenation — zero runtime overhead
- Local echo and backspace support
- Timestamped log functions (AT_I / AT_W / AT_E / AT_D)
- Two-phase dispatch: exact match + parametric (`AT+CMD="value"` with quote stripping)

## Targets

`esp32` · `esp32-c3` · `esp32-c6` · `esp32-s3`

## Usage

```c
#include "esp_at.h"

esp_at_init();
esp_at_register_cmd("AT+TEMP", handle_temp);

// In handler:
void handle_temp(const char *param) {
    if (param == NULL) {
        AT(G "Temp: " Y "%.1f°C" W, read_temp());
    }
}

// Output anywhere, any task:
AT(G "OK");
AT_I(C "Sistema iniciado — heap: %lu", esp_get_free_heap_size());
AT(B "millis = " W "%llu" B " microseconds", esp_timer_get_time() / 1000);
```

## Built-in commands

| Command | Response |
|---|---|
| `AT` | `OK` |
| `AT+MAC` | eFuse MAC address |
| `AT+VER` / `AT+VERSION` | Firmware + IDF version |
| `AT+RESET` / `AT+REBOOT` | Restart chip |

## More examples

- See `EXAMPLES.md` for practical `app_main` examples:
  - `esp_at_add("HW", cb)` for simple callbacks
  - `AT+SETPOINT=25` with `esp_at_register_cmd(...)`
  - `AT+MILLIS` and output/log patterns

## Color tokens

```c
AT(G "verde " Y "amarelo " R "vermelho " B "azul " W "reset");
AT(O "laranja " C "ciano " P "roxo " M "magenta " K "rosa" W);
```

## Dependencies

- ESP-IDF >= 5.0.0

## Install

```bash
idf.py add-dependency "rodrigo-s-lange/esp_at>=1.0.0"
```

Or as git submodule:

```bash
git submodule add https://github.com/rodrigo-s-lange/esp-at.git components/esp_at
```

## License

MIT — © 2026 Rodrigo S. Lange
