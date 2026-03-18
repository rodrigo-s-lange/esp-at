/**
 * @file    esp_at.h
 * @brief   Interface de comandos AT via UART ou USB Serial/JTAG para família ESP32.
 *
 * Implementa um parser de linha AT sobre um transporte serial com echo local,
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
 * @warning Os tokens de cor são macros de string literal de uma letra maiúscula
 *          (G, Y, R, B, W, O, P, C, M, K). Evite usar esses nomes como
 *          variáveis locais nas unidades de compilação que incluem este header.
 *
 * @par Repositório
 *   https://github.com/rodrigo-s-lange/esp-at
 *
 * @par Registry
 *   rodrigo-s-lange/esp_at
 */

#pragma once

#include <stdbool.h>
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
#define Y  "\033[33m"           /**< Yellow  (amarelo)      */
#define R  "\033[31m"           /**< Red     (vermelho)     */
#define B  "\033[34m"           /**< Blue    (azul)         */
#define W  "\033[0m"            /**< White / Reset          */
#define O  "\033[38;5;208m"     /**< Orange  (laranja)      */
#define P  "\033[35m"           /**< Purple  (roxo)         */
#define C  "\033[36m"           /**< Cyan    (ciano)        */
#define M  "\033[95m"           /**< Magenta (magenta)      */
#define K  "\033[38;5;213m"     /**< Pink    (rosa)  — K de pinK */
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

/**
 * @brief Prototipo simplificado para comandos AT sem parametro.
 */
typedef void (*esp_at_simple_handler_t)(void);

/**
 * @brief Callback para espelhar a saida do terminal AT em outros transportes.
 *
 * @param[in] data  Buffer transmitido (nao necessariamente terminado em '\0').
 * @param[in] len   Tamanho do buffer em bytes.
 * @param[in] ctx   Contexto de usuario fornecido no registro do sink.
 */
typedef void (*esp_at_output_sink_t)(const char *data, size_t len, void *ctx);

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
esp_err_t esp_at_init(bool log_enabled);

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

/**
 * @brief Registra um comando AT com exemplo opcional para o HELP.
 *
 * @param[in] cmd      String do comando (ex: "AT+VER").
 * @param[in] handler  Handler do comando.
 * @param[in] example  Exemplo de uso exibido no HELP (opcional).
 * @return ESP_OK em sucesso.
 */
esp_err_t esp_at_register_cmd_example(const char *cmd, at_handler_t handler, const char *example);

/**
 * @brief Remove um comando AT previamente registrado.
 *
 * @param[in] cmd String completa do comando (ex: "AT+VER").
 * @return ESP_OK em sucesso.
 * @return ESP_ERR_NOT_FOUND se o comando nao existir.
 */
esp_err_t esp_at_unregister_cmd(const char *cmd);

/**
 * @brief Registra comando no formato simplificado "AT+<suffix>".
 *
 * Exemplo:
 * @code
 *   static void func_helloworld(void) { AT(B "hello world"); }
 *   esp_at_add("HW", func_helloworld); // comando: AT+HW
 * @endcode
 *
 * @param[in] suffix  Sufixo do comando sem "AT+" (ex: "HW").
 *                    Tambem aceita comando completo (ex: "AT+HW").
 * @param[in] handler Callback sem parametros.
 * @return ESP_OK em sucesso.
 */
esp_err_t esp_at_add(const char *suffix, esp_at_simple_handler_t handler);

/**
 * @brief Registra comando simplificado com exemplo opcional para o HELP.
 *
 * @param[in] suffix   Sufixo sem "AT+" ou comando completo.
 * @param[in] handler  Callback sem parametros.
 * @param[in] example  Exemplo de uso exibido no HELP (opcional).
 * @return ESP_OK em sucesso.
 */
esp_err_t esp_at_add_example(const char *suffix, esp_at_simple_handler_t handler, const char *example);

/**
 * @brief Injeta uma linha completa no parser AT.
 *
 * A linha e processada como se tivesse sido recebida pela serial. Nao inclua
 * CR/LF no final.
 *
 * @param[in] line Linha AT (ex: "AT+VER").
 * @return ESP_OK em sucesso.
 */
esp_err_t esp_at_feed_line(const char *line);

/**
 * @brief Registra um sink para receber toda saida do terminal AT.
 *
 * A saida continua sendo escrita na UART; o sink recebe uma copia dos bytes.
 *
 * @param[in] sink Callback de saida.
 * @param[in] ctx  Contexto de usuario.
 * @return ESP_OK em sucesso.
 */
esp_err_t esp_at_register_output_sink(esp_at_output_sink_t sink, void *ctx);

/**
 * @brief Remove um sink previamente registrado.
 *
 * @param[in] sink Callback registrada.
 * @param[in] ctx  Contexto associado.
 * @return ESP_OK em sucesso.
 */
esp_err_t esp_at_unregister_output_sink(esp_at_output_sink_t sink, void *ctx);

/**
 * @brief Informa se o subsistema AT ja foi inicializado.
 */
bool esp_at_is_initialized(void);

/* ======================================================================== */
/* Saída — funções de escrita                                               */
/* ======================================================================== */

/**
 * @brief  Envia string formatada via UART e adiciona \r\n + reset de cor.
 *
 * @code
 *   AT(G "tensão: " Y "%.2f" W " V", tensao);
 *   AT(B "millis = " W "%llu" B " microseconds", esp_timer_get_time() / 1000);
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

/*
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

