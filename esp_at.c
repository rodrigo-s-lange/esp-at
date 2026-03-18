/**
 * @file    esp_at.c
 * @brief   Implementacao do subsistema de comandos AT via transporte serial.
 *
 * Para registrar um novo comando em outro componente:
 * @code
 *   esp_at_register_cmd("AT+FOO", handle_foo);
 *   // void handle_foo(const char *param)
 *   // param = NULL  -> comando sem '=' (ex: AT+FOO)
 *   // param = "val" -> valor de AT+FOO="val" (aspas removidas)
 * @endcode
 */

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/uart.h"
#include "sdkconfig.h"

#if CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C6
#include "driver/usb_serial_jtag.h"
#define AT_USE_USB_SERIAL_JTAG 1
#else
#define AT_USE_USB_SERIAL_JTAG 0
#endif
#include "esp_heap_caps.h"
#include "esp_littlefs.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_image_format.h"
#include "esp_chip_info.h"

#include "esp_at.h"

/* ======================================================================== */
/* Configuracao                                                              */
/* ======================================================================== */

#define AT_UART_NUM          UART_NUM_0
#define AT_UART_BAUD         115200
#define AT_BUF_SIZE          256
#define AT_LOG_HDR_SIZE      32
#define AT_TASK_STACK        4096
#define AT_MAX_CMDS          100
#define AT_MAX_SINKS         4
#define AT_INJECT_QUEUE_LEN  8
#define AT_CMD_MAX_LEN       32

static const char *TAG = "esp_at";

static esp_err_t _at_transport_init(void);
static int _at_transport_read_byte(uint8_t *ch, TickType_t timeout_ticks);
static void _at_transport_write(const char *data, size_t len);
static const char *_at_transport_name(void);
static bool _at_transport_needs_local_echo(void);

/* ======================================================================== */
/* Estado interno                                                            */
/* ======================================================================== */

typedef struct {
    const char              *cmd;
    const char              *example;
    at_handler_t             handler;
    esp_at_simple_handler_t  simple_handler;
    bool                     help_visible;
} at_cmd_t;

typedef struct {
    esp_at_output_sink_t sink;
    void                *ctx;
} at_sink_t;

static at_cmd_t          s_table[AT_MAX_CMDS];
static at_sink_t         s_sinks[AT_MAX_SINKS];
static char              s_owned_cmds[AT_MAX_CMDS][AT_CMD_MAX_LEN];
static int               s_cmd_count   = 0;
static SemaphoreHandle_t s_mutex       = NULL;
static QueueHandle_t     s_inject_q    = NULL;
static bool              s_initialized = false;
static bool              s_log_enabled = false;

static int _find_cmd_index(const char *cmd)
{
    if (cmd == NULL) return -1;
    for (int i = 0; i < s_cmd_count; i++) {
        if (s_table[i].cmd != NULL && strcmp(s_table[i].cmd, cmd) == 0) {
            return i;
        }
    }
    return -1;
}

static bool _cmd_uses_owned_slot(int idx)
{
    if (idx < 0 || idx >= AT_MAX_CMDS) return false;
    return s_table[idx].cmd == s_owned_cmds[idx];
}

/* ======================================================================== */
/* Saida interna                                                             */
/* ======================================================================== */

static size_t _safe_vsnprintf(char *dst, size_t dst_len, const char *fmt, va_list args)
{
    int n = vsnprintf(dst, dst_len, fmt, args);
    if (n < 0) return 0;
    if ((size_t)n >= dst_len) return dst_len - 1;
    return (size_t)n;
}

static void _emit_output(const char *data, size_t len)
{
    if (data == NULL || len == 0) return;

    _at_transport_write(data, len);

    at_sink_t sinks[AT_MAX_SINKS] = {0};
    int sink_count = 0;

    if (s_mutex != NULL) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        for (int i = 0; i < AT_MAX_SINKS; i++) {
            if (s_sinks[i].sink != NULL) {
                sinks[sink_count++] = s_sinks[i];
            }
        }
        xSemaphoreGive(s_mutex);
    }

    for (int i = 0; i < sink_count; i++) {
        sinks[i].sink(data, len, sinks[i].ctx);
    }
}

static void _at_log(char level, const char *fmt, va_list args)
{
    uint64_t us  = (uint64_t)esp_timer_get_time();
    uint32_t sec = (uint32_t)(us / 1000000ULL);

    char line[AT_BUF_SIZE + AT_LOG_HDR_SIZE];
    int hlen = snprintf(line, sizeof(line),
                        "\033[0m[%c %02"PRIu32":%02"PRIu32":%02"PRIu32"] ",
                        level, sec / 3600u, (sec % 3600u) / 60u, sec % 60u);
    if (hlen < 0 || hlen >= (int)sizeof(line)) return;

    size_t body_len = _safe_vsnprintf(line + hlen, sizeof(line) - (size_t)hlen, fmt, args);
    size_t total = (size_t)hlen + body_len;

    if (total + 6 <= sizeof(line)) {
        memcpy(line + total, "\033[0m\r\n", 6);
        total += 6;
    }

    _emit_output(line, total);
}

/* ======================================================================== */
/* Handlers built-in                                                        */
/* ======================================================================== */

static void handle_at(const char *param)
{
    (void)param;
    AT(G "OK");
}

static void handle_at_uppercase(const char *param)
{
    (void)param;
    AT(Y "ERROR: use UPPERCASE" W " expected: AT+<CMD>");
}

static void handle_at_help(const char *param)
{
    (void)param;
    AT(G "Comandos AT registrados:");
    for (int i = 0; i < s_cmd_count; i++) {
        if (!s_table[i].help_visible) {
            continue;
        }
        if (s_table[i].example != NULL && s_table[i].example[0] != '\0') {
            AT(G "  %-16s " W "%s", s_table[i].cmd, s_table[i].example);
        } else {
            AT(G "  %s", s_table[i].cmd);
        }
    }
}

static void handle_at_mac(const char *param)
{
    (void)param;
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    at(O "MAC " W "%02X%02X%02X%02X%02X%02X",
       mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    AT(O "  %02X.%02X.%02X.%02X.%02X.%02X",
       mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void handle_at_reset(const char *param)
{
    (void)param;
    AT(R "Reiniciando...");
    esp_restart();
}

static void handle_at_ver(const char *param)
{
    (void)param;
    AT(O "Firmware " G FIRMWARE_VERSION W " | SDK " C "%s", esp_get_idf_version());
}

static void handle_at_free(const char *param)
{
    (void)param;
    char report[768];
    size_t off = 0U;

    uint32_t uptime_s = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    size_t heap_free = esp_get_free_heap_size();
    size_t heap_min = esp_get_minimum_free_heap_size();
    size_t heap_largest = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    size_t internal_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    size_t internal_largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    size_t total_heap = heap_caps_get_total_size(MALLOC_CAP_8BIT);
    size_t total_internal = heap_caps_get_total_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    unsigned heap_frag_pct = 0U;
    unsigned internal_frag_pct = 0U;

    if (heap_free > 0U && heap_largest <= heap_free) {
        heap_frag_pct = (unsigned)(100U - (unsigned)((heap_largest * 100U) / heap_free));
    }
    if (internal_free > 0U && internal_largest <= internal_free) {
        internal_frag_pct = (unsigned)(100U - (unsigned)((internal_largest * 100U) / internal_free));
    }

    size_t lfs_total = 0;
    size_t lfs_used = 0;
    esp_err_t lfs_err = esp_littlefs_info("storage", &lfs_total, &lfs_used);
    unsigned lfs_used_pct = 0U;

    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_partition_pos_t running_pos = {
        .offset = running != NULL ? running->address : 0U,
        .size = running != NULL ? running->size : 0U,
    };
    esp_image_metadata_t image = {0};
    esp_err_t app_err = (running != NULL) ? esp_image_verify(ESP_IMAGE_VERIFY_SILENT, &running_pos, &image) : ESP_ERR_NOT_FOUND;
    unsigned app_used_pct = 0U;
    size_t app_used = 0U;

    if (app_err == ESP_OK && running != NULL && running->size > 0U) {
        app_used = image.image_len;
        app_used_pct = (unsigned)((app_used * 100U) / running->size);
    }
    if (lfs_err == ESP_OK && lfs_total > 0U && lfs_total >= lfs_used) {
        lfs_used_pct = (unsigned)((lfs_used * 100U) / lfs_total);
    }

    off += (size_t)snprintf(report + off, sizeof(report) - off,
                            G "Memoria:" W "\r\n"
                            C "  heap_total     : " W "%u bytes\r\n"
                            C "  heap_free      : " W "%u bytes\r\n"
                            C "  heap_min       : " W "%u bytes\r\n"
                            C "  largest_block  : " W "%u bytes\r\n"
                            C "  heap_frag      : " W "%u%%\r\n"
                            C "  internal_total : " W "%u bytes\r\n"
                            C "  internal_free  : " W "%u bytes\r\n"
                            C "  internal_block : " W "%u bytes\r\n"
                            C "  internal_frag  : " W "%u%%\r\n"
                            C "  uptime         : " W "%u s\r\n",
                            (unsigned)total_heap,
                            (unsigned)heap_free,
                            (unsigned)heap_min,
                            (unsigned)heap_largest,
                            heap_frag_pct,
                            (unsigned)total_internal,
                            (unsigned)internal_free,
                            (unsigned)internal_largest,
                            internal_frag_pct,
                            (unsigned)uptime_s);
    if (off >= sizeof(report)) off = sizeof(report) - 1U;

    if (app_err == ESP_OK && running != NULL) {
        off += (size_t)snprintf(report + off, sizeof(report) - off,
                                G "Aplicacao:" W "\r\n"
                                C "  used           : " W "%u / %u bytes\r\n"
                                C "  used_pct       : " W "%u%%\r\n",
                                (unsigned)app_used,
                                (unsigned)running->size,
                                app_used_pct);
    } else {
        off += (size_t)snprintf(report + off, sizeof(report) - off,
                                Y "Aplicacao: indisponivel (%s)" W "\r\n",
                                esp_err_to_name(app_err));
    }
    if (off >= sizeof(report)) off = sizeof(report) - 1U;

    if (lfs_err == ESP_OK && lfs_total >= lfs_used) {
        off += (size_t)snprintf(report + off, sizeof(report) - off,
                                G "LittleFS:" W "\r\n"
                                C "  total          : " W "%u bytes\r\n"
                                C "  used           : " W "%u bytes\r\n"
                                C "  free           : " W "%u bytes\r\n"
                                C "  used_pct       : " W "%u%%\r\n",
                                (unsigned)lfs_total,
                                (unsigned)lfs_used,
                                (unsigned)(lfs_total - lfs_used),
                                lfs_used_pct);
    } else {
        off += (size_t)snprintf(report + off, sizeof(report) - off,
                                Y "LittleFS: indisponivel (%s)" W "\r\n",
                                esp_err_to_name(lfs_err));
    }
    if (off >= sizeof(report)) off = sizeof(report) - 1U;

    memcpy(report + off, "\033[0m", 4);
    off += 4U;
    _emit_output(report, off);
}

static void handle_at_sys(const char *param)
{
    (void)param;

    esp_chip_info_t chip = {0};
    size_t internal_ram = 0;
    size_t psram = 0;

    esp_chip_info(&chip);
    internal_ram = heap_caps_get_total_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    psram = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);

    AT(C "Sistema:");
    AT(C "  target         : " W "%s", CONFIG_IDF_TARGET);
    AT(C "  model          : " W "%d", (int)chip.model);
    AT(C "  revision       : " W "%d", chip.revision);
    AT(C "  cores          : " W "%d", chip.cores);
    AT(C "  cpu_mhz_cfg    : " W "%d", CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ);
    AT(C "  ram_total      : " W "%lu bytes", (unsigned long)internal_ram);
    AT(C "  psram_total    : " W "%lu bytes", (unsigned long)psram);
    AT(C "  heap_free      : " W "%lu bytes", (unsigned long)esp_get_free_heap_size());
    AT(C "  heap_min       : " W "%lu bytes", (unsigned long)esp_get_minimum_free_heap_size());
}

/* ======================================================================== */
/* Dispatch - 2 fases                                                        */
/* ======================================================================== */

static char *_extract_param(char *buf, size_t eq_pos)
{
    char  *val  = buf + eq_pos + 1;
    size_t vlen = strlen(val);
    if (vlen >= 2 && val[0] == '"' && val[vlen - 1] == '"') {
        val[vlen - 1] = '\0';
        val++;
    }
    return val;
}

static bool _starts_with_lowercase_at(const char *line)
{
    if (line == NULL) return false;
    if (line[0] != 'a' || line[1] != 't') return false;

    return (line[2] == '\0' || line[2] == '+' || line[2] == '?' || line[2] == '=');
}

static void dispatch(char *line)
{
    if (_starts_with_lowercase_at(line)) {
        handle_at_uppercase(line);
        return;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    /* Fase 1 - exact match */
    for (int i = 0; i < s_cmd_count; i++) {
        if (strcmp(line, s_table[i].cmd) == 0) {
            at_handler_t h = s_table[i].handler;
            esp_at_simple_handler_t sh = s_table[i].simple_handler;
            xSemaphoreGive(s_mutex);
            if (sh != NULL) {
                sh();
            } else {
                h(NULL);
            }
            return;
        }
    }

    /* Fase 2 - prefix match com '=' */
    char *eq = strchr(line, '=');
    if (eq != NULL) {
        size_t clen = (size_t)(eq - line);
        for (int i = 0; i < s_cmd_count; i++) {
            if (s_table[i].handler == NULL) continue; /* comandos simplificados nao recebem '=' */
            if (strlen(s_table[i].cmd) == clen &&
                strncmp(line, s_table[i].cmd, clen) == 0) {
                at_handler_t h = s_table[i].handler;
                xSemaphoreGive(s_mutex);
                h(_extract_param(line, clen));
                return;
            }
        }
    }

    xSemaphoreGive(s_mutex);
    AT(R "ERROR: unknown command");
}

static void _dispatch_line(const char *line)
{
    if (line == NULL || line[0] == '\0') return;

    char work[AT_BUF_SIZE];
    strncpy(work, line, sizeof(work) - 1);
    work[sizeof(work) - 1] = '\0';

    if (s_log_enabled) {
        ESP_LOGI(TAG, "< %s", work);
    }
    dispatch(work);
}

/* ======================================================================== */
/* Task de leitura                                                           */
/* ======================================================================== */

static void at_task(void *arg)
{
    (void)arg;

    char    buf[AT_BUF_SIZE];
    int     pos = 0;
    uint8_t ch;
    char injected[AT_BUF_SIZE];

    while (1) {
        if (xQueueReceive(s_inject_q, injected, 0) == pdTRUE) {
            _dispatch_line(injected);
            continue;
        }

        if (_at_transport_read_byte(&ch, pdMS_TO_TICKS(20)) <= 0) continue;

        if (ch == '\r' || ch == '\n') {
            if (_at_transport_needs_local_echo()) {
                _emit_output("\r\n", 2);
            }
            if (pos > 0) {
                buf[pos] = '\0';
                _dispatch_line(buf);
                pos = 0;
            }
        } else if (ch == 0x08 || ch == 0x7F) {
            if (pos > 0) {
                pos--;
                if (_at_transport_needs_local_echo()) {
                    _emit_output("\x08 \x08", 3);
                }
            }
        } else if (pos < AT_BUF_SIZE - 1) {
            buf[pos++] = (char)ch;
            if (_at_transport_needs_local_echo()) {
                _emit_output((const char *)&ch, 1);
            }
        }
    }
}

/* ======================================================================== */
/* API publica                                                               */
/* ======================================================================== */

static esp_err_t _at_transport_init(void)
{
#if AT_USE_USB_SERIAL_JTAG
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    return usb_serial_jtag_driver_install(&cfg);
#else
    const uart_config_t cfg = {
        .baud_rate  = AT_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_param_config(AT_UART_NUM, &cfg);
    if (err != ESP_OK) return err;
    return uart_driver_install(AT_UART_NUM, AT_BUF_SIZE * 2, 0, 0, NULL, 0);
#endif
}

static int _at_transport_read_byte(uint8_t *ch, TickType_t timeout_ticks)
{
    if (ch == NULL) return -1;

#if AT_USE_USB_SERIAL_JTAG
    return (int)usb_serial_jtag_read_bytes(ch, 1, timeout_ticks);
#else
    return uart_read_bytes(AT_UART_NUM, ch, 1, timeout_ticks);
#endif
}

static void _at_transport_write(const char *data, size_t len)
{
    if (data == NULL || len == 0) return;

#if AT_USE_USB_SERIAL_JTAG
    size_t written = 0;
    while (written < len) {
        size_t to_send = len - written;
        if (to_send > 64U) {
            to_send = 64U;
        }

        int chunk = usb_serial_jtag_write_bytes(data + written, to_send, pdMS_TO_TICKS(50));
        if (chunk > 0) {
            written += (size_t)chunk;
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }
    (void)usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(100));
#else
    (void)uart_write_bytes(AT_UART_NUM, data, len);
#endif
}

static const char *_at_transport_name(void)
{
#if AT_USE_USB_SERIAL_JTAG
    return "usb_serial_jtag";
#else
    return "uart0";
#endif
}

static bool _at_transport_needs_local_echo(void)
{
    return true;
}

esp_err_t esp_at_init(bool log_enabled)
{
    if (s_initialized) return ESP_ERR_INVALID_STATE;

    s_log_enabled = log_enabled;
    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) return ESP_ERR_NO_MEM;

    s_inject_q = xQueueCreate(AT_INJECT_QUEUE_LEN, AT_BUF_SIZE);
    if (s_inject_q == NULL) {
        vSemaphoreDelete(s_mutex);
        s_mutex = NULL;
        return ESP_ERR_NO_MEM;
    }

    esp_at_register_cmd_example("AT",         handle_at,              "AT_TESTE");
    esp_at_register_cmd_example("AT+HELP",    handle_at_help,         "AT+HELP");
    esp_at_register_cmd_example("AT+MAC",     handle_at_mac,          "AT+MAC");
    esp_at_register_cmd_example("AT+VER",     handle_at_ver,          "AT+VER");
    esp_at_register_cmd_example("AT+VERSION", handle_at_ver,          "AT+VERSION");
    esp_at_register_cmd_example("AT+FREE",    handle_at_free,         "AT+FREE");
    esp_at_register_cmd_example("AT+RESET",   handle_at_reset,        "AT+RESET");
    esp_at_register_cmd_example("AT+REBOOT",  handle_at_reset,        "AT+REBOOT");
    esp_at_register_cmd_example("AT+SYS",     handle_at_sys,          "AT+SYS");

    ESP_ERROR_CHECK(_at_transport_init());

    xTaskCreate(at_task, "at_task", AT_TASK_STACK, NULL, 5, NULL);

    s_initialized = true;
    if (s_log_enabled) {
        ESP_LOGI(TAG, "inicializado - %d comando(s), transport=%s", s_cmd_count, _at_transport_name());
    }
    return ESP_OK;
}

esp_err_t esp_at_register_cmd(const char *cmd, at_handler_t handler)
{
    return esp_at_register_cmd_example(cmd, handler, NULL);
}

esp_err_t esp_at_register_cmd_example(const char *cmd, at_handler_t handler, const char *example)
{
    if (cmd == NULL || handler == NULL) return ESP_ERR_INVALID_ARG;

    if (s_mutex == NULL) {
        int idx = _find_cmd_index(cmd);
        if (idx >= 0) {
            s_table[idx].example = example;
            s_table[idx].handler = handler;
            s_table[idx].simple_handler = NULL;
            s_table[idx].help_visible = true;
            return ESP_OK;
        }
        if (s_cmd_count >= AT_MAX_CMDS) return ESP_ERR_NO_MEM;
        s_table[s_cmd_count].cmd            = cmd;
        s_table[s_cmd_count].example        = example;
        s_table[s_cmd_count].handler        = handler;
        s_table[s_cmd_count].simple_handler = NULL;
        s_table[s_cmd_count].help_visible   = true;
        s_cmd_count++;
        return ESP_OK;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int idx = _find_cmd_index(cmd);
    if (idx >= 0) {
        s_table[idx].example = example;
        s_table[idx].handler = handler;
        s_table[idx].simple_handler = NULL;
        s_table[idx].help_visible = true;
        xSemaphoreGive(s_mutex);
        return ESP_OK;
    }
    if (s_cmd_count >= AT_MAX_CMDS) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_NO_MEM;
    }
    s_table[s_cmd_count].cmd            = cmd;
    s_table[s_cmd_count].example        = example;
    s_table[s_cmd_count].handler        = handler;
    s_table[s_cmd_count].simple_handler = NULL;
    s_table[s_cmd_count].help_visible   = true;
    s_cmd_count++;
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t esp_at_set_help_visible(const char *cmd, bool visible)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (cmd == NULL || cmd[0] == '\0') return ESP_ERR_INVALID_ARG;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int idx = _find_cmd_index(cmd);
    if (idx < 0) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_NOT_FOUND;
    }
    s_table[idx].help_visible = visible;
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t esp_at_unregister_cmd(const char *cmd)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (cmd == NULL || cmd[0] == '\0') return ESP_ERR_INVALID_ARG;

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    int idx = _find_cmd_index(cmd);
    if (idx < 0) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_NOT_FOUND;
    }

    for (int i = idx; i < s_cmd_count - 1; i++) {
        bool next_uses_owned = _cmd_uses_owned_slot(i + 1);
        at_cmd_t next = s_table[i + 1];

        memcpy(s_owned_cmds[i], s_owned_cmds[i + 1], sizeof(s_owned_cmds[i]));
        s_table[i] = next;
        if (next_uses_owned) {
            s_table[i].cmd = s_owned_cmds[i];
        }
    }

    memset(&s_table[s_cmd_count - 1], 0, sizeof(s_table[s_cmd_count - 1]));
    memset(s_owned_cmds[s_cmd_count - 1], 0, sizeof(s_owned_cmds[s_cmd_count - 1]));
    s_cmd_count--;

    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t esp_at_add(const char *suffix, esp_at_simple_handler_t handler)
{
    return esp_at_add_example(suffix, handler, NULL);
}

esp_err_t esp_at_add_example(const char *suffix, esp_at_simple_handler_t handler, const char *example)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (suffix == NULL || suffix[0] == '\0' || handler == NULL) return ESP_ERR_INVALID_ARG;

    char full_cmd[AT_CMD_MAX_LEN];
    int n = 0;
    if (strncmp(suffix, "AT+", 3) == 0 || strcmp(suffix, "AT") == 0) {
        n = snprintf(full_cmd, sizeof(full_cmd), "%s", suffix);
    } else {
        n = snprintf(full_cmd, sizeof(full_cmd), "AT+%s", suffix);
    }
    if (n <= 0 || n >= (int)sizeof(full_cmd)) return ESP_ERR_INVALID_SIZE;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int idx = _find_cmd_index(full_cmd);
    if (idx >= 0) {
        s_table[idx].example = example;
        s_table[idx].handler = NULL;
        s_table[idx].simple_handler = handler;
        s_table[idx].help_visible = true;
        xSemaphoreGive(s_mutex);
        return ESP_OK;
    }
    if (s_cmd_count >= AT_MAX_CMDS) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_NO_MEM;
    }

    memcpy(s_owned_cmds[s_cmd_count], full_cmd, (size_t)n + 1U);
    s_table[s_cmd_count].cmd            = s_owned_cmds[s_cmd_count];
    s_table[s_cmd_count].example        = example;
    s_table[s_cmd_count].handler        = NULL;
    s_table[s_cmd_count].simple_handler = handler;
    s_table[s_cmd_count].help_visible   = true;
    s_cmd_count++;
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t esp_at_feed_line(const char *line)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (line == NULL || line[0] == '\0') return ESP_ERR_INVALID_ARG;

    char msg[AT_BUF_SIZE] = {0};
    strncpy(msg, line, sizeof(msg) - 1);

    size_t len = strlen(msg);
    while (len > 0 && (msg[len - 1] == '\r' || msg[len - 1] == '\n')) {
        msg[--len] = '\0';
    }
    if (len == 0) return ESP_ERR_INVALID_ARG;

    if (xQueueSend(s_inject_q, msg, pdMS_TO_TICKS(50)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t esp_at_register_output_sink(esp_at_output_sink_t sink, void *ctx)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (sink == NULL) return ESP_ERR_INVALID_ARG;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (int i = 0; i < AT_MAX_SINKS; i++) {
        if (s_sinks[i].sink == sink && s_sinks[i].ctx == ctx) {
            xSemaphoreGive(s_mutex);
            return ESP_OK;
        }
    }
    for (int i = 0; i < AT_MAX_SINKS; i++) {
        if (s_sinks[i].sink == NULL) {
            s_sinks[i].sink = sink;
            s_sinks[i].ctx  = ctx;
            xSemaphoreGive(s_mutex);
            return ESP_OK;
        }
    }
    xSemaphoreGive(s_mutex);
    return ESP_ERR_NO_MEM;
}

esp_err_t esp_at_unregister_output_sink(esp_at_output_sink_t sink, void *ctx)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (sink == NULL) return ESP_ERR_INVALID_ARG;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (int i = 0; i < AT_MAX_SINKS; i++) {
        if (s_sinks[i].sink == sink && s_sinks[i].ctx == ctx) {
            s_sinks[i].sink = NULL;
            s_sinks[i].ctx  = NULL;
            xSemaphoreGive(s_mutex);
            return ESP_OK;
        }
    }
    xSemaphoreGive(s_mutex);
    return ESP_ERR_NOT_FOUND;
}

bool esp_at_is_initialized(void)
{
    return s_initialized;
}

void AT(const char *fmt, ...)
{
    char buf[AT_BUF_SIZE + 6];
    va_list args;
    va_start(args, fmt);
    size_t len = _safe_vsnprintf(buf, sizeof(buf) - 6, fmt, args);
    va_end(args);
    if (len == 0) return;

    memcpy(buf + len, "\033[0m\r\n", 6);
    _emit_output(buf, len + 6);
}

void at(const char *fmt, ...)
{
    char buf[AT_BUF_SIZE];
    va_list args;
    va_start(args, fmt);
    size_t len = _safe_vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (len == 0) return;

    _emit_output(buf, len);
}

void AT_I(const char *fmt, ...) { va_list a; va_start(a, fmt); _at_log('I', fmt, a); va_end(a); }
void AT_W(const char *fmt, ...) { va_list a; va_start(a, fmt); _at_log('W', fmt, a); va_end(a); }
void AT_E(const char *fmt, ...) { va_list a; va_start(a, fmt); _at_log('E', fmt, a); va_end(a); }
void AT_D(const char *fmt, ...) { va_list a; va_start(a, fmt); _at_log('D', fmt, a); va_end(a); }
