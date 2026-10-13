/**
 * @file    jq6500.h
 * @brief   JQ6500 MP3 Module — non-blocking STM32 HAL driver
 *
 * Architecture overview
 * ─────────────────────
 *  TX  →  HAL_UART_Transmit_IT          (interrupt-driven, returns immediately)
 *  Control cmd response ("OK") → absorbed by background ReceiveToIdle_IT
 *                                 during the mandatory 200 ms post-cmd delay
 *  Query response → HAL_UARTEx_ReceiveToIdle_IT fires when UART line goes idle
 *  All delays → HAL_GetTick() comparisons driven by JQ6500_Process()
 *
 * Why the 200 ms post-command delay matters
 * ──────────────────────────────────────────
 * The JQ6500 Arduino reference library waits ~200 ms after every control
 * command before returning (it reads bytes until a 150 ms inter-byte gap
 * expires after the "OK" response).  Sending the next command before the
 * module has fully processed the previous one causes silent failures —
 * most critically: SetLoopMode does not take effect before Play, so the
 * module plays only the first track and then stops instead of looping.
 * JQ6500_CMD_POST_DELAY_MS matches that timing exactly.
 *
 * All query results are delivered through a single event callback.
 * No HAL_Delay(), no blocking receive — the MCU is free the entire time.
 *
 * Integration checklist
 * ─────────────────────
 *  1. UART configured as 9600 8N1, global interrupt enabled in NVIC.
 *  2. Wire two HAL weak callbacks in stm32xxxx_it.c (or main.c):
 *
 *       void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
 *           if (huart == jq.huart) JQ6500_TxCpltCallback(&jq);
 *       }
 *       void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
 *           if (huart == jq.huart) JQ6500_RxEventCallback(&jq, size);
 *       }
 *
 *  3. Call JQ6500_Process() as fast as possible (main loop or 1 ms task).
 *
 * Callback context rules
 * ──────────────────────
 *  JQ6500_EVT_CMD_DONE  — fired from JQ6500_Process() (main-loop context).
 *                          Safe to call any JQ6500 API directly here.
 *  JQ6500_EVT_QUERY_DONE/ERROR — fired from UART RX ISR context.
 *                          Do NOT call JQ6500 API here.  Set a flag; act in loop.
 *
 * @note Requires HAL_UARTEx_ReceiveToIdle_IT (STM32Cube HAL >= ~2021).
 *       Include your STM32 family header before this file (CubeMX main.h works).
 */

#ifndef JQ6500_H
#define JQ6500_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"   /* CubeMX: pulls in the correct stm32xxxx_hal.h */
#include <stdint.h>
#include <stdbool.h>

/* ═══════════════════════════════════════════════════════════════════════════
 * Tuneable constants
 * ═══════════════════════════════════════════════════════════════════════════*/
#define JQ6500_MAX_VOLUME              30U   /**< Upper volume limit (0–30)               */
#define JQ6500_CMD_POST_DELAY_MS      200U   /**< Mandatory wait after every control cmd  */
#define JQ6500_RESET_DELAY_MS        1000U   /**< Wait after Reset (overrides above)      */
#define JQ6500_STARTUP_DELAY_MS      750U   /**< Wait after power-up before first cmd    */
#define JQ6500_DEFAULT_RX_TIMEOUT    1000U   /**< ms before query RX watchdog fires       */
#define JQ6500_TX_TIMEOUT_MS          100U   /**< ms before TX watchdog fires             */
#define JQ6500_STATUS_SAMPLES           5U   /**< Samples for GetStatusRobust             */
#define JQ6500_STATUS_SAMPLE_DELAY_MS  10U   /**< ms between robust-status polls          */
#define JQ6500_RX_BUF_SIZE             32U   /**< Must hold longest module response       */

#define JQ6500_STATUS_POLL_MS          2000U /**< Auto-play status poll interval          */
#define JQ6500_AUTO_NEXT_FALLBACK         0  /**< 1 = send Next() if status is PAUSED/STOPPED */

#ifndef JQ6500_PROVIDE_HAL_CALLBACKS
#define JQ6500_PROVIDE_HAL_CALLBACKS      1  /**< 0 = you wire HAL_UART_* callbacks yourself */
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 * Public enumerations
 * ═══════════════════════════════════════════════════════════════════════════*/

typedef enum { JQ6500_OK = 0, JQ6500_ERROR, JQ6500_BUSY } JQ6500_Result;

typedef enum {
    JQ6500_EQ_NORMAL = 0, JQ6500_EQ_POP, JQ6500_EQ_ROCK,
    JQ6500_EQ_JAZZ,       JQ6500_EQ_CLASSIC, JQ6500_EQ_BASS
} JQ6500_EqMode;

typedef enum {
    JQ6500_LOOP_ALL = 0, JQ6500_LOOP_FOLDER, JQ6500_LOOP_ONE,
    JQ6500_LOOP_RAM,     JQ6500_LOOP_ONE_STOP
} JQ6500_LoopMode;

typedef enum { JQ6500_SRC_SDCARD = 0x01, JQ6500_SRC_FLASH = 0x04 } JQ6500_Source;
typedef enum { JQ6500_FOLDER_PREV = 0x00, JQ6500_FOLDER_NEXT = 0x01 } JQ6500_FolderDir;

typedef enum {
    JQ6500_STATUS_STOPPED = 0,
    JQ6500_STATUS_PLAYING = 1,
    JQ6500_STATUS_PAUSED  = 2
} JQ6500_Status;

typedef enum {
    JQ6500_QUERY_NONE           =  0,
    JQ6500_QUERY_STATUS         =  1,
    JQ6500_QUERY_STATUS_ROBUST  =  2,
    JQ6500_QUERY_VOLUME         =  3,
    JQ6500_QUERY_EQ             =  4,
    JQ6500_QUERY_LOOP           =  5,
    JQ6500_QUERY_VERSION        =  6,
    JQ6500_QUERY_COUNT_SD       =  7,
    JQ6500_QUERY_COUNT_FLASH    =  8,
    JQ6500_QUERY_COUNT_FOLDERS  =  9,
    JQ6500_QUERY_SD_INDEX       = 10,
    JQ6500_QUERY_FLASH_INDEX    = 11,
    JQ6500_QUERY_POSITION       = 12,
    JQ6500_QUERY_LENGTH         = 13,
    JQ6500_QUERY_FILENAME       = 14,
} JQ6500_QueryId;

typedef enum {
    JQ6500_EVT_CMD_DONE,   /**< Control command fully completed (incl. post-delay) */
    JQ6500_EVT_QUERY_DONE, /**< Query returned; see queryId / value / fileName      */
    JQ6500_EVT_ERROR,      /**< TX watchdog or parse failure; see queryId           */
} JQ6500_EventType;

typedef struct {
    JQ6500_EventType type;
    JQ6500_QueryId   queryId;
    uint16_t         value;
    const char      *fileName;  /**< Non-NULL only for QUERY_FILENAME; internal buf */
} JQ6500_Event;

typedef void (*JQ6500_EventCb)(const JQ6500_Event *evt, void *ctx);

/* ═══════════════════════════════════════════════════════════════════════════
 * Internal state (in header because it lives inside the handle)
 * ═══════════════════════════════════════════════════════════════════════════*/
/** @cond INTERNAL */
typedef enum {
    JQ6500_INT_IDLE  = 0,
    JQ6500_INT_TX,       /**< HAL_UART_Transmit_IT running              */
    JQ6500_INT_RX,       /**< HAL_UARTEx_ReceiveToIdle_IT for a query   */
    JQ6500_INT_DELAY,    /**< Tick-based delay (post-cmd or reset wait) */
} JQ6500_IntState;

typedef enum {
    JQ6500_AFTER_DONE          = 0,
    JQ6500_AFTER_STATUS_SAMPLE,
} JQ6500_AfterDelay;
/** @endcond */

typedef enum {
    JQ6500_APP_IDLE = 0,     /* auto-play not started             */
    JQ6500_APP_WAIT_BOOT,    /* waiting for module to boot        */
    JQ6500_APP_WAIT_RESET,   /* waiting for Reset CMD_DONE        */
    JQ6500_APP_WAIT_VOLUME,  /* waiting for SetVolume CMD_DONE    */
    JQ6500_APP_WAIT_LOOP,    /* waiting for SetLoopMode CMD_DONE  */
    JQ6500_APP_PLAYING       /* playing; periodic status check    */
} JQ6500_AppState;


/* ═══════════════════════════════════════════════════════════════════════════
 * Handle — user-allocated; treat fields as opaque
 * ═══════════════════════════════════════════════════════════════════════════*/
typedef struct {
    UART_HandleTypeDef *huart;

    uint8_t txBuf[6];
    uint8_t rxBuf[JQ6500_RX_BUF_SIZE];

    JQ6500_IntState   intState;
    JQ6500_AfterDelay afterDelay;
    uint32_t          tickMark;
    uint32_t          delayMs;
    uint32_t          postTxDelayMs;  /**< >0 overrides CMD_POST_DELAY_MS (Reset) */

    JQ6500_QueryId pendingQueryId;
    uint32_t       rxTimeoutMs;

    uint8_t statusCounts[3];
    uint8_t statusSamplesLeft;

    JQ6500_EventCb eventCb;      /* internal dispatcher */
    void          *eventCtx;

    /* user pass-through callback (optional) */
    JQ6500_EventCb userCb;
    void          *userCtx;

    /* auto-play layer */
    JQ6500_AppState  appState;
    uint8_t          appVolume;
    JQ6500_LoopMode  appLoop;
    uint32_t         statusPollTick;
    volatile uint8_t needNext;
} JQ6500_Handle;

/* ═══════════════════════════════════════════════════════════════════════════
 * Public API
 * ═══════════════════════════════════════════════════════════════════════════*/

void JQ6500_Init(JQ6500_Handle      *hJQ,
                 UART_HandleTypeDef *huart,
                 uint32_t            rxTimeoutMs,
                 JQ6500_EventCb      eventCb,
                 void               *ctx);

void JQ6500_Process(JQ6500_Handle *hJQ);
/** Run Reset -> SetVolume -> SetLoopMode -> Play, then poll status. */
JQ6500_Result JQ6500_StartAutoPlay(JQ6500_Handle *hJQ, uint8_t volume, JQ6500_LoopMode loop);
void JQ6500_TxCpltCallback(JQ6500_Handle *hJQ);
void JQ6500_RxEventCallback(JQ6500_Handle *hJQ, uint16_t size);

bool JQ6500_IsBusy(const JQ6500_Handle *hJQ);

/* ── Control commands (return BUSY if occupied) ─────────────────────────── */
JQ6500_Result JQ6500_Play(JQ6500_Handle *hJQ);
JQ6500_Result JQ6500_Pause(JQ6500_Handle *hJQ);
JQ6500_Result JQ6500_Next(JQ6500_Handle *hJQ);
JQ6500_Result JQ6500_Prev(JQ6500_Handle *hJQ);
JQ6500_Result JQ6500_Reset(JQ6500_Handle *hJQ);
JQ6500_Result JQ6500_Sleep(JQ6500_Handle *hJQ);
JQ6500_Result JQ6500_PlayIndex(JQ6500_Handle *hJQ, uint16_t index);
JQ6500_Result JQ6500_PlayFolderFile(JQ6500_Handle *hJQ, uint8_t folder, uint8_t file);
JQ6500_Result JQ6500_ChangeFolder(JQ6500_Handle *hJQ, JQ6500_FolderDir dir);
JQ6500_Result JQ6500_VolUp(JQ6500_Handle *hJQ);
JQ6500_Result JQ6500_VolDown(JQ6500_Handle *hJQ);
JQ6500_Result JQ6500_SetVolume(JQ6500_Handle *hJQ, uint8_t volume);
JQ6500_Result JQ6500_SetEqualizer(JQ6500_Handle *hJQ, JQ6500_EqMode eq);
JQ6500_Result JQ6500_SetLoopMode(JQ6500_Handle *hJQ, JQ6500_LoopMode loop);
JQ6500_Result JQ6500_SetSource(JQ6500_Handle *hJQ, JQ6500_Source source);

/* ── Query commands (result via event callback) ─────────────────────────── */
JQ6500_Result JQ6500_GetStatus(JQ6500_Handle *hJQ);
JQ6500_Result JQ6500_GetStatusRobust(JQ6500_Handle *hJQ);
JQ6500_Result JQ6500_GetVolume(JQ6500_Handle *hJQ);
JQ6500_Result JQ6500_GetEqualizer(JQ6500_Handle *hJQ);
JQ6500_Result JQ6500_GetLoopMode(JQ6500_Handle *hJQ);
JQ6500_Result JQ6500_GetVersion(JQ6500_Handle *hJQ);
JQ6500_Result JQ6500_CountSDFiles(JQ6500_Handle *hJQ);
JQ6500_Result JQ6500_CountFlashFiles(JQ6500_Handle *hJQ);
JQ6500_Result JQ6500_CountSDFolders(JQ6500_Handle *hJQ);
JQ6500_Result JQ6500_GetSDCurrentIndex(JQ6500_Handle *hJQ);
JQ6500_Result JQ6500_GetFlashCurrentIndex(JQ6500_Handle *hJQ);
JQ6500_Result JQ6500_GetPosition(JQ6500_Handle *hJQ);
JQ6500_Result JQ6500_GetLength(JQ6500_Handle *hJQ);
JQ6500_Result JQ6500_GetFileName(JQ6500_Handle *hJQ);

#ifdef __cplusplus
}
#endif
#endif /* JQ6500_H */
