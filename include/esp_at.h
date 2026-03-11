/**
 * @file    esp_at.h
 * @brief   Interface de comandos AT via UART para família ESP32.
 *
 * Implementa um parser de linha AT sobre UART0 com echo local,
 * suporte a backspace, registro dinâmico de comandos e saída colorida
 * via tokens ANSI de concatenação em tempo de compilação.
 *
 * Fluxo de uso:
 * @code
 *   esp_at_init();
 *   esp_at_register_cmd("AT+VER", handle_version);
 *
 *   // Saída com cores (concatenação compile-time, zero overhead runtime):
 *   AT(G "tensão: " Y "%.2f V" W, tensao);
 *   AT_I(G "Sistema " W "iniciado — núcleo %d", xPortGetCoreID());
 * @endcode
 *
 * Drivers Espressif utilizados internamente:
 *  - esp_driver_uart : uart_param_config(), uart_driver_install(),
 *                      uart_read_bytes(), uart_write_bytes()
 *  - esp_mac.h       : esp_efuse_mac_get_default()
 *  - esp_timer.h     : esp_timer_get_time()  (timestamp dos logs)
 *  - freertos        : xTaskCreate(), xSemaphoreCreateMutex(),
 *                      xSemaphoreTake(), xSemaphoreGive()
 *
 * @note Thread-safe: AT(), at(), AT_I/W/E/D() e esp_at_register_cmd()
 *       podem ser chamados de qualquer task após esp_at_init().
 *
 * @warning Os tokens de cor são macros de string literal de uma letra
 *          (G, Y, R …). Evite usar esses nomes como variáveis locais
 *          nas unidades de compilação que incluem este header.
 *
 * @par Repositório
 *   https://github.com/rodrigo-s-lange/esp_at
 *
 * @par Registry
 *   rodrigo-s-lange/esp_at
 */

#pragma once

#include <stddef.h>
#include <stdarg.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ======================================================================== */
/* Tokens de cor ANSI — concatenação de string em tempo de compilação       */
/* ======================================================================== */
/**
 * @defgroup at_colors Tokens de cor ANSI
 * @{
 *
 * Cada token é uma string literal com o código de escape ANSI correspondente.
 * Por ser apenas um literal, o compilador os concatena com os demais strings
 * adjacentes sem nenhum custo em tempo de execução.
 *
 * Uso:
 * @code
 *   AT(R "falha: " Y "%d erros" W, n);
 *   at(G "ok" W " | " C "%.1f°C" W, temp);
 * @endcode
 */
#define G  "\033[32m"           /**< Green   (verde)        */
#define g  "\033[32m"
#define Y  "\033[33m"           /**< Yellow  (amarelo)      */
#define y  "\033[33m"
#define R  "\033[31m"           /**< Red     (vermelho)     */
#define r  "\033[31m"
#define B  "\033[34m"           /**< Blue    (azul)         */
#define b  "\033[34m"
#define W  "\033[0m"            /**< White / Reset          */
#define w  "\033[0m"
#define O  "\033[38;5;208m"     /**< Orange  (laranja)      */
#define o  "\033[38;5;208m"
#define P  "\033[35m"           /**< Purple  (roxo)         */
#define p  "\033[35m"
#define C  "\033[36m"           /**< Cyan    (ciano)        */
#define c  "\033[36m"
#define M  "\033[95m"           /**< Magenta (magenta)      */
#define m  "\033[95m"
#define K  "\033[38;5;213m"     /**< Pink    (rosa)  — K de pinK */
#define k  "\033[38;5;213m"
/** @} */

/* ======================================================================== */
/* Tipo do handler de comando AT                                            */
/* ======================================================================== */

/**
 * @brief  Protótipo da função de tratamento de um comando AT.
 *
 * @param[in] param  NULL  → comando recebido sem '=' (ex: AT+FOO).
 *                   !NULL → valor após '=', sem aspas (ex: AT+FOO="bar" → "bar").
 *
 * O handler é executado dentro da `at_task` — não bloquear por longos períodos
 * nem chamar APIs que dependam de outras tasks com prioridade menor.
 */
typedef void (*at_handler_t)(const char *param);

/* ======================================================================== */
/* Inicialização                                                             */
/* ======================================================================== */

/**
 * @brief  Inicializa o subsistema esp_at.
 *
 * Configura a UART0 a 115200-8N1, instala o driver de baixo nível
 * (uart_driver_install) e cria a task FreeRTOS de leitura de linha.
 * Registra internamente os comandos AT, AT+MAC, AT+VER, AT+RESET.
 *
 * Deve ser chamada uma única vez, antes de qualquer outra função
 * desta API. Não é seguro chamá-la novamente sem reinicializar o chip.
 *
 * @return ESP_OK                Inicialização bem-sucedida.
 * @return ESP_ERR_INVALID_STATE Já inicializado.
 * @return Outros                Erros da camada UART (esp_err_t).
 */
esp_err_t esp_at_init(void);

/* ======================================================================== */
/* Registro de comandos                                                      */
/* ======================================================================== */

/**
 * @brief  Registra um novo comando AT em tempo de execução.
 *
 * Adiciona o par (string de comando, handler) na tabela interna.
 * A comparação é feita com strcmp — maiúsculas/minúsculas importam.
 * Thread-safe: usa mutex interno.
 *
 * @param[in] cmd      String do comando (ex: "AT+VER"). Deve permanecer
 *                     válida durante toda a vida útil do sistema
 *                     (literal ou alocação estática).
 * @param[in] handler  Função do tipo `at_handler_t` chamada quando o
 *                     comando for recebido.  Executada dentro da
 *                     `at_task` — não bloquear por longos períodos.
 *
 * @return ESP_OK                Comando registrado com sucesso.
 * @return ESP_ERR_NO_MEM        Tabela de comandos cheia (AT_MAX_CMDS).
 * @return ESP_ERR_INVALID_ARG   cmd ou handler nulos.
 * @return ESP_ERR_INVALID_STATE esp_at_init() não foi chamado.
 */
esp_err_t esp_at_register_cmd(const char *cmd, at_handler_t handler);

/* ======================================================================== */
/* Saída — funções de escrita                                               */
/* ======================================================================== */

/**
 * @brief  Envia string formatada via UART e adiciona \r\n + reset de cor.
 *
 * @code
 *   AT(G "tensão: " Y "%.2f" W " V", tensao);
 * @endcode
 *
 * @param[in] fmt  String de formato printf-style (pode conter tokens de cor).
 * @param[in] ...  Argumentos variádicos.
 */
void AT(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/**
 * @brief  Envia string formatada via UART sem newline nem reset de cor.
 *
 * @code
 *   at(G "CPU: ");
 *   at(Y "%d MHz" W " | ", freq);
 *   AT(C "heap: %lu", heap_free);   // esta fecha a linha
 * @endcode
 *
 * @param[in] fmt  String de formato printf-style.
 * @param[in] ...  Argumentos variádicos.
 */
void at(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/* ======================================================================== */
/* Saída — log com timestamp                                                */
/* ======================================================================== */

/**
 * @brief  Log nível INFO com timestamp.
 * Formato: \033[0m[I HH:MM:SS] <mensagem>\033[0m\r\n
 * @param[in] fmt  String de formato (pode conter tokens de cor).
 * @param[in] ...  Argumentos variádicos.
 */
void AT_I(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/**
 * @brief  Log nível WARN com timestamp.
 * @param[in] fmt  String de formato.
 * @param[in] ...  Argumentos variádicos.
 */
void AT_W(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/**
 * @brief  Log nível ERROR com timestamp.
 * @param[in] fmt  String de formato.
 * @param[in] ...  Argumentos variádicos.
 */
void AT_E(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/**
 * @brief  Log nível DEBUG com timestamp.
 * @param[in] fmt  String de formato.
 * @param[in] ...  Argumentos variádicos.
 */
void AT_D(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

#ifdef __cplusplus
}
#endif
