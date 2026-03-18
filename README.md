# esp_at

AT command parser for the ESP32 family, with dynamic command registration and optional internal logging.

## Features

- Dynamic thread-safe command registration at runtime
- ANSI color tokens via compile-time string concatenation
- Local echo and backspace support
- Timestamped log functions: `AT_I`, `AT_W`, `AT_E`, `AT_D`
- Two-phase dispatch: exact match + parametric commands (`AT+CMD=...`)
- Transport selected per target:
  - `ESP32`: UART0
  - `ESP32-S3/C3/C6`: USB Serial/JTAG

## Targets

`esp32` - `esp32-c3` - `esp32-c6` - `esp32-s3`

## Usage

```c
#include "esp_at.h"

static void handle_temp(const char *param)
{
    (void)param;
    AT(G "Temp: " Y "%.1f C" W, read_temp());
}

void app_main(void)
{
    ESP_ERROR_CHECK(esp_at_init(false));
    ESP_ERROR_CHECK(esp_at_register_cmd("AT+TEMP", handle_temp));
}
```

## Init policy

`esp_at_init(log_enabled)` controls only the internal logs of the AT engine.

- `esp_at_init(false)`
  - AT works normally
  - parser and transport logs stay disabled
- `esp_at_init(true)`
  - parser and transport logs are enabled

## Built-in commands

| Command | Response |
|---|---|
| `AT` | `OK` |
| `AT+MAC` | eFuse MAC address |
| `AT+VER` / `AT+VERSION` | Firmware + IDF version |
| `AT+FREE` | Heap, fragmentation estimate and filesystem usage |
| `AT+RESET` / `AT+REBOOT` | Restart chip |

## More examples

See `EXAMPLES.md` for practical `app_main` examples.

## Color tokens

```c
AT(G "green " Y "yellow " R "red " B "blue " W "reset");
AT(O "orange " C "cyan " P "purple " M "magenta " K "pink" W);
```

## Dependencies

- ESP-IDF >= 5.0.0

## Install

```bash
idf.py add-dependency "rodrigo-s-lange/esp_at>=1.0.0"
```

## License

MIT - Copyright (c) 2026
