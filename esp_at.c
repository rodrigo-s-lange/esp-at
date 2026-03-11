/**
 * @file    esp_at.c
 * @brief   Implementação do subsistema de comandos AT via UART.
 *
 * Para registrar um novo comando em outro componente:
 * @code
 *   esp_at_register_cmd("AT+FOO", handle_foo);
 *   // void handle_foo(const char *param)
 *   // param = NULL  → comando sem '=' (ex: AT+FOO)
 *   // param = "val" → valor de AT+FOO="val" (aspas removidas)
 * @endcode
 */

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_at.h"

/* ======================================================================== */
/* Configuração                                                              */
/* ======================================================================== */

#define AT_UART_NUM     UART_NUM_0
#define AT_UART_BAUD    115200
#define AT_BUF_SIZE     256
#define AT_LOG_HDR_SIZE 32
#define AT_TASK_STACK   4096
#define AT_MAX_CMDS     24           /**< Suporta até 24 comandos registrados */

static const char *TAG = "esp_at";

/* ======================================================================== */
/* Estado interno                                                            */
/* ======================================================================== */

typedef struct {
    const char   *cmd;
    at_handler_t  handler;
} at_cmd_t;

static at_cmd_t          s_table[AT_MAX_CMDS];
static int               s_cmd_count   = 0;
static SemaphoreHandle_t s_mutex       = NULL;
static bool              s_initialized = false;

/* ======================================================================== */
/* Saída interna                                                             */
/* ======================================================================== */

static void _at_log(char level, const char *fmt, va_list args)
{
    uint64_t us  = (uint64_t)esp_timer_get_time();
    uint32_t sec = (uint32_t)(us / 1000000ULL);

    char line[AT_BUF_SIZE + AT_LOG_HDR_SIZE];
    int hlen = snprintf(line, sizeof(line),
                        "\033[0m[%c %02"PRIu32":%02"PRIu32":%02"PRIu32"] ",
                        level, sec / 3600u, (sec % 3600u) / 60u, sec % 60u);
    if (hlen < 0 || hlen >= (int)sizeof(line)) return;

    int blen = vsnprintf(line + hlen, sizeof(line) - (size_t)hlen, fmt, args);
    if (blen < 0) return;

    int total = hlen + blen;
    if (total + 6 < (int)sizeof(line)) {
        memcpy(line + total, "\033[0m\r\n", 6);
        total += 6;
    }
    uart_write_bytes(AT_UART_NUM, line, (size_t)total);
}

/* ======================================================================== */
/* Handlers built-in                                                         */
/* ======================================================================== */

static void handle_at(const char *param)
{
    (void)param;
    AT(G "OK");
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

/* ======================================================================== */
/* Dispatch — 2 fases                                                        */
/* ======================================================================== */

/**
 * @brief  Extrai valor após '=' removendo aspas. Modifica buf in-place.
 * @param[in,out] buf     Linha AT.
 * @param[in]     eq_pos  Índice do '=' em buf.
 * @return Ponteiro para o valor (dentro de buf), sem aspas.
 */
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

static void dispatch(char *line)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);

    /* Fase 1 — exact match */
    for (int i = 0; i < s_cmd_count; i++) {
        if (strcmp(line, s_table[i].cmd) == 0) {
            at_handler_t h = s_table[i].handler;
            xSemaphoreGive(s_mutex);
            h(NULL);
            return;
        }
    }

    /* Fase 2 — prefix match com '=' */
    char *eq = strchr(line, '=');
    if (eq != NULL) {
        size_t clen = (size_t)(eq - line);
        for (int i = 0; i < s_cmd_count; i++) {
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

/* ======================================================================== */
/* Task de leitura                                                           */
/* ======================================================================== */

static void at_task(void *arg)
{
    char    buf[AT_BUF_SIZE];
    int     pos = 0;
    uint8_t ch;

    while (1) {
        if (uart_read_bytes(AT_UART_NUM, &ch, 1, portMAX_DELAY) <= 0) continue;

        if (ch == '\r' || ch == '\n') {
            uart_write_bytes(AT_UART_NUM, "\r\n", 2);
            if (pos > 0) {
                buf[pos] = '\0';
                ESP_LOGI(TAG, "< %s", buf);
                dispatch(buf);
                pos = 0;
            }
        } else if (ch == 0x08 || ch == 0x7F) {
            if (pos > 0) {
                pos--;
                uart_write_bytes(AT_UART_NUM, "\x08 \x08", 3);
            }
        } else if (pos < AT_BUF_SIZE - 1) {
            buf[pos++] = (char)ch;
            uart_write_bytes(AT_UART_NUM, &ch, 1);
        }
    }
}

/* ======================================================================== */
/* API pública                                                               */
/* ======================================================================== */

esp_err_t esp_at_init(void)
{
    if (s_initialized) return ESP_ERR_INVALID_STATE;

    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) return ESP_ERR_NO_MEM;

    esp_at_register_cmd("AT",         handle_at);
    esp_at_register_cmd("AT+MAC",     handle_at_mac);
    esp_at_register_cmd("AT+VER",     handle_at_ver);
    esp_at_register_cmd("AT+VERSION", handle_at_ver);
    esp_at_register_cmd("AT+RESET",   handle_at_reset);
    esp_at_register_cmd("AT+REBOOT",  handle_at_reset);

    const uart_config_t cfg = {
        .baud_rate  = AT_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(AT_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_driver_install(AT_UART_NUM, AT_BUF_SIZE * 2, 0, 0, NULL, 0));

    xTaskCreate(at_task, "at_task", AT_TASK_STACK, NULL, 5, NULL);

    s_initialized = true;
    ESP_LOGI(TAG, "inicializado — %d comando(s)", s_cmd_count);
    return ESP_OK;
}

esp_err_t esp_at_register_cmd(const char *cmd, at_handler_t handler)
{
    if (cmd == NULL || handler == NULL) return ESP_ERR_INVALID_ARG;

    if (s_mutex == NULL) {
        if (s_cmd_count >= AT_MAX_CMDS) return ESP_ERR_NO_MEM;
        s_table[s_cmd_count].cmd     = cmd;
        s_table[s_cmd_count].handler = handler;
        s_cmd_count++;
        return ESP_OK;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_cmd_count >= AT_MAX_CMDS) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_NO_MEM;
    }
    s_table[s_cmd_count].cmd     = cmd;
    s_table[s_cmd_count].handler = handler;
    s_cmd_count++;
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

void AT(const char *fmt, ...)
{
    char buf[AT_BUF_SIZE];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf) - 6, fmt, args);
    va_end(args);
    if (len <= 0) return;
    memcpy(buf + len, "\033[0m\r\n", 6);
    uart_write_bytes(AT_UART_NUM, buf, (size_t)(len + 6));
}

void at(const char *fmt, ...)
{
    char buf[AT_BUF_SIZE];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (len > 0) uart_write_bytes(AT_UART_NUM, buf, (size_t)len);
}

void AT_I(const char *fmt, ...) { va_list a; va_start(a, fmt); _at_log('I', fmt, a); va_end(a); }
void AT_W(const char *fmt, ...) { va_list a; va_start(a, fmt); _at_log('W', fmt, a); va_end(a); }
void AT_E(const char *fmt, ...) { va_list a; va_start(a, fmt); _at_log('E', fmt, a); va_end(a); }
void AT_D(const char *fmt, ...) { va_list a; va_start(a, fmt); _at_log('D', fmt, a); va_end(a); }
