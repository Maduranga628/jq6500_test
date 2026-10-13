/**
 * @file  jq6500.c
 * @brief JQ6500 MP3 Module — non-blocking STM32 HAL implementation
 *
 * State machine
 * ─────────────
 *
 *   IDLE ──StartTx()──► TX  ──TxCplt, ctrl──► DELAY(200ms) ──elapsed──► IDLE
 *                        │                         │                      fire CMD_DONE
 *                        │   TxCplt, query         │ background RxToIdle absorbs "OK"
 *                        └──────────────────► RX ──┘──parse──► IDLE fire QUERY_DONE
 *                                                  └──timeout──► IDLE fire ERROR
 *
 *  Reset uses postTxDelayMs = 500 ms instead of the default CMD_POST_DELAY_MS.
 *
 *  Background receiver
 *  ───────────────────
 *  A ReceiveToIdle_IT is always armed while the driver is in IDLE or DELAY
 *  state.  This absorbs the "OK" response after every control command and
 *  catches unsolicited "STOP" messages sent by the module when a track ends.
 *  Without this, STM32F1's single-byte receive register overflows and sets
 *  the ORE flag, killing all further UART reception.
 *
 *  Before every arm, __HAL_UART_CLEAR_OREFLAG is called.  On STM32F1 this
 *  reads SR then DR, which is the hardware-mandated sequence to clear ORE.
 */

#include "jq6500.h"
#include <string.h>

/* ═══════════════════════════════════════════════════════════════════════════
 * Frame bytes
 * ═══════════════════════════════════════════════════════════════════════════*/
#define JQ_START  0x7EU
#define JQ_END    0xEFU

/* ═══════════════════════════════════════════════════════════════════════════
 * Command byte table
 * ═══════════════════════════════════════════════════════════════════════════*/
#define CMD_NEXT            0x01U
#define CMD_PREV            0x02U
#define CMD_PLAY_INDEX      0x03U
#define CMD_VOL_UP          0x04U
#define CMD_VOL_DN          0x05U
#define CMD_SET_VOLUME      0x06U
#define CMD_SET_EQ          0x07U
#define CMD_SET_SOURCE      0x09U
#define CMD_SLEEP           0x0AU
#define CMD_RESET           0x0CU
#define CMD_PLAY            0x0DU
#define CMD_PAUSE           0x0EU
#define CMD_CHANGE_FOLDER   0x0FU
#define CMD_SET_LOOP        0x11U
#define CMD_PLAY_FOLDER     0x12U
#define CMD_GET_STATUS      0x42U
#define CMD_GET_VOLUME      0x43U
#define CMD_GET_EQ          0x44U
#define CMD_GET_LOOP        0x45U
#define CMD_GET_VERSION     0x46U
#define CMD_COUNT_SD        0x47U
#define CMD_COUNT_FLASH     0x49U
#define CMD_GET_SD_INDEX    0x4BU
#define CMD_GET_FLASH_INDEX 0x4DU
#define CMD_GET_POSITION    0x50U
#define CMD_GET_LENGTH      0x51U
#define CMD_GET_FILENAME    0x52U
#define CMD_COUNT_FOLDERS   0x53U

/* ═══════════════════════════════════════════════════════════════════════════
 * Private helpers
 * ═══════════════════════════════════════════════════════════════════════════*/

/**
 * @brief  Arm a background receive so all bytes sent by the module while the
 *         driver is idle/delaying are captured rather than causing a UART
 *         overrun error (ORE) that would kill all further communication.
 *
 *         Before arming we clear the ORE flag.  On STM32F1 the hardware
 *         mandates reading SR then DR; the HAL macro does exactly that.
 *         Clearing it here prevents HAL_UARTEx_ReceiveToIdle_IT from
 *         immediately bouncing back with an error callback.
 */
static void ArmBackgroundRx(JQ6500_Handle *hJQ)
{
    /* Clear ORE (and NE/FE/PE on F1 — same read-SR-then-read-DR sequence) */
    __HAL_UART_CLEAR_OREFLAG(hJQ->huart);

    (void)HAL_UARTEx_ReceiveToIdle_IT(hJQ->huart,
                                      hJQ->rxBuf,
                                      JQ6500_RX_BUF_SIZE);
}

/**
 * @brief  Fire the user callback, then re-arm background RX if the driver
 *         returned to IDLE (i.e. the callback did not start a new command).
 */
static void FireEvent(JQ6500_Handle    *hJQ,
                      JQ6500_EventType  type,
                      JQ6500_QueryId    queryId,
                      uint16_t          value,
                      const char       *fileName)
{
    if (hJQ->eventCb != NULL) {
        const JQ6500_Event evt = { type, queryId, value, fileName };
        hJQ->eventCb(&evt, hJQ->eventCtx);
    }
    if (hJQ->intState == JQ6500_INT_IDLE) {
        ArmBackgroundRx(hJQ);
    }
}

/**
 * @brief  Parse a 4-character ASCII-hex string (e.g. "001E") → uint16_t.
 */
static JQ6500_Result ParseHex(const uint8_t *buf, uint16_t size, uint16_t *out)
{
    if (size < 4U) {
        return JQ6500_ERROR;
    }
    uint16_t val = 0U;
    for (uint8_t i = 0U; i < 4U; i++) {
        uint8_t c = buf[i];
        val <<= 4U;
        if      (c >= (uint8_t)'0' && c <= (uint8_t)'9') { val |= (uint16_t)(c - (uint8_t)'0');        }
        else if (c >= (uint8_t)'A' && c <= (uint8_t)'F') { val |= (uint16_t)(c - (uint8_t)'A' + 10U); }
        else if (c >= (uint8_t)'a' && c <= (uint8_t)'f') { val |= (uint16_t)(c - (uint8_t)'a' + 10U); }
        else                                              { return JQ6500_ERROR; }
    }
    *out = val;
    return JQ6500_OK;
}

#if JQ6500_PROVIDE_HAL_CALLBACKS
static JQ6500_Handle *s_activeHandle = NULL;
#endif

/* Internal event dispatcher: runs the auto-play sequence, then forwards to user. */
static void AppEventCb(const JQ6500_Event *evt, void *ctx)
{
    JQ6500_Handle *hJQ = (JQ6500_Handle *)ctx;

    if (evt->type == JQ6500_EVT_CMD_DONE) {          /* main-loop context */
        switch (hJQ->appState) {
            case JQ6500_APP_WAIT_RESET:
                hJQ->appState = JQ6500_APP_WAIT_VOLUME;
                (void)JQ6500_SetVolume(hJQ, hJQ->appVolume);
                break;
            case JQ6500_APP_WAIT_VOLUME:
                hJQ->appState = JQ6500_APP_WAIT_LOOP;
                (void)JQ6500_SetLoopMode(hJQ, hJQ->appLoop);
                break;
            case JQ6500_APP_WAIT_LOOP:
                hJQ->appState = JQ6500_APP_PLAYING;
                hJQ->statusPollTick = HAL_GetTick();
                (void)JQ6500_PlayIndex(hJQ, 1);
                break;
            default:
                break;
        }
    }
    else if (evt->type == JQ6500_EVT_QUERY_DONE &&
             evt->queryId == JQ6500_QUERY_STATUS) {  /* ISR context: flag only */
#if JQ6500_AUTO_NEXT_FALLBACK
        if (evt->value == JQ6500_STATUS_PAUSED || evt->value == JQ6500_STATUS_STOPPED) {
            hJQ->needNext = 1U;
        }
#endif
    }

    if (hJQ->userCb != NULL) {
        hJQ->userCb(evt, hJQ->userCtx);
    }
}

/**
 * @brief  Build a JQ6500 frame into txBuf and start HAL_UART_Transmit_IT.
 *         Callers must ensure correct state before calling.
 *
 * Frame: [START][LEN][CMD][ARG0?][ARG1?][END]   LEN = nargs+2
 */
static JQ6500_Result StartTx(JQ6500_Handle  *hJQ,
                              uint8_t         cmd,
                              const uint8_t  *args,
                              uint8_t         nargs,
                              JQ6500_QueryId  queryId)
{
    uint8_t idx = 0U;
    hJQ->txBuf[idx++] = JQ_START;
    hJQ->txBuf[idx++] = (uint8_t)(nargs + 2U);
    hJQ->txBuf[idx++] = cmd;
    for (uint8_t i = 0U; i < nargs; i++) {
        hJQ->txBuf[idx++] = args[i];
    }
    hJQ->txBuf[idx++] = JQ_END;

    /* Stop any active background receive before transmitting */
    HAL_UART_AbortReceive(hJQ->huart);

    hJQ->pendingQueryId = queryId;
    hJQ->intState       = JQ6500_INT_TX;
    hJQ->tickMark       = HAL_GetTick();

    if (HAL_UART_Transmit_IT(hJQ->huart, hJQ->txBuf, idx) != HAL_OK) {
        hJQ->intState = JQ6500_INT_IDLE;
        return JQ6500_ERROR;
    }
    return JQ6500_OK;
}

/**
 * @brief  IDLE-checked wrapper around StartTx for all public API commands.
 */
static JQ6500_Result StartCmd(JQ6500_Handle  *hJQ,
                               uint8_t         cmd,
                               const uint8_t  *args,
                               uint8_t         nargs,
                               JQ6500_QueryId  queryId,
                               uint32_t        postTxDelayMs)
{
    if (hJQ->intState != JQ6500_INT_IDLE) {
        return JQ6500_BUSY;
    }
    hJQ->postTxDelayMs = postTxDelayMs;
    hJQ->afterDelay    = JQ6500_AFTER_DONE;
    return StartTx(hJQ, cmd, args, nargs, queryId);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Public API — initialisation
 * ═══════════════════════════════════════════════════════════════════════════*/

void JQ6500_Init(JQ6500_Handle      *hJQ,
                 UART_HandleTypeDef *huart,
                 uint32_t            rxTimeoutMs,
                 JQ6500_EventCb      userCb,     /* optional, may be NULL */
                 void               *userCtx)
{
    memset(hJQ, 0, sizeof(*hJQ));
    hJQ->huart       = huart;
    hJQ->rxTimeoutMs = (rxTimeoutMs == 0U) ? JQ6500_DEFAULT_RX_TIMEOUT : rxTimeoutMs;
    hJQ->eventCb     = AppEventCb;
    hJQ->eventCtx    = hJQ;
    hJQ->userCb      = userCb;
    hJQ->userCtx     = userCtx;
    hJQ->intState    = JQ6500_INT_IDLE;
    hJQ->appState    = JQ6500_APP_IDLE;

#if JQ6500_PROVIDE_HAL_CALLBACKS
    s_activeHandle = hJQ;
#endif

    ArmBackgroundRx(hJQ);
}

JQ6500_Result JQ6500_StartAutoPlay(JQ6500_Handle *hJQ, uint8_t volume, JQ6500_LoopMode loop)
{
    if (hJQ->intState != JQ6500_INT_IDLE) return JQ6500_BUSY;
    hJQ->appVolume      = volume;
    hJQ->appLoop        = loop;
    hJQ->statusPollTick = HAL_GetTick();     /* reused as boot timestamp */
    hJQ->appState       = JQ6500_APP_WAIT_BOOT;
    return JQ6500_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Public API — engine
 * ═══════════════════════════════════════════════════════════════════════════*/

void JQ6500_Process(JQ6500_Handle *hJQ)
{
    const uint32_t elapsed = HAL_GetTick() - hJQ->tickMark;

    switch (hJQ->intState) {

        /* ── TX watchdog ─────────────────────────────────────────────── */
        case JQ6500_INT_TX:
            if (elapsed >= JQ6500_TX_TIMEOUT_MS) {
                HAL_UART_Abort(hJQ->huart);
                hJQ->intState = JQ6500_INT_IDLE;
                FireEvent(hJQ, JQ6500_EVT_ERROR, hJQ->pendingQueryId, 0U, NULL);
            }
            break;

        /* ── RX watchdog (queries only) ──────────────────────────────── */
        case JQ6500_INT_RX:
            if (elapsed >= hJQ->rxTimeoutMs) {
                HAL_UART_AbortReceive(hJQ->huart);
                hJQ->intState = JQ6500_INT_IDLE;
                FireEvent(hJQ, JQ6500_EVT_ERROR, hJQ->pendingQueryId, 0U, NULL);
            }
            break;

        /* ── Tick-based delay ────────────────────────────────────────── */
        case JQ6500_INT_DELAY:
            if (elapsed >= hJQ->delayMs) {

                if (hJQ->afterDelay == JQ6500_AFTER_STATUS_SAMPLE) {
                    /* Send the next robust-status sample */
                    (void)StartTx(hJQ, CMD_GET_STATUS, NULL, 0U,
                                  JQ6500_QUERY_STATUS_ROBUST);
                } else {
                    /* Delay elapsed — fire CMD_DONE from main-loop context */
                    hJQ->intState = JQ6500_INT_IDLE;
                    FireEvent(hJQ, JQ6500_EVT_CMD_DONE, JQ6500_QUERY_NONE, 0U, NULL);
                }
            }
            break;

        default:
            break;
    }

    if (hJQ->appState == JQ6500_APP_WAIT_BOOT && hJQ->intState == JQ6500_INT_IDLE) {
        if ((HAL_GetTick() - hJQ->statusPollTick) >= JQ6500_STARTUP_DELAY_MS) {
            hJQ->appState = JQ6500_APP_WAIT_RESET;
            if (JQ6500_Reset(hJQ) != JQ6500_OK) {
                hJQ->appState = JQ6500_APP_IDLE;
            }
        }
    }

    /* ── Auto-play housekeeping (main-loop context) ──────────────────── */
    if (hJQ->appState == JQ6500_APP_PLAYING && hJQ->intState == JQ6500_INT_IDLE) {
        if (hJQ->needNext) {
            hJQ->needNext = 0U;
            (void)JQ6500_Next(hJQ);
        } else if ((HAL_GetTick() - hJQ->statusPollTick) >= JQ6500_STATUS_POLL_MS) {
            hJQ->statusPollTick = HAL_GetTick();
            (void)JQ6500_GetStatus(hJQ);
        }
    }
}

/* ── Interrupt-context callbacks ────────────────────────────────────────── */

void JQ6500_TxCpltCallback(JQ6500_Handle *hJQ)
{
    if (hJQ->intState != JQ6500_INT_TX) {
        return;   /* Spurious callback guard */
    }

    if (hJQ->pendingQueryId == JQ6500_QUERY_NONE) {
        /*
         * Control command TX complete.
         *
         * Enter a fixed post-command delay before firing CMD_DONE.  This
         * matches the ~200 ms the Arduino library waits after "OK" (it reads
         * with a 150 ms inter-byte timeout after the "OK" arrives).  The
         * module needs this time to fully apply commands like SetLoopMode
         * before the next command (Play) is sent; without it, LOOP_ALL silently
         * fails and only the first track plays.
         *
         * During the delay the background receiver absorbs the "OK" response
         * and any other unsolicited bytes without bothering the state machine.
         */
        uint32_t delayMs = (hJQ->postTxDelayMs > 0U) ? hJQ->postTxDelayMs
                                                       : JQ6500_CMD_POST_DELAY_MS;
        hJQ->postTxDelayMs = 0U;
        hJQ->delayMs       = delayMs;
        hJQ->afterDelay    = JQ6500_AFTER_DONE;
        hJQ->intState      = JQ6500_INT_DELAY;
        hJQ->tickMark      = HAL_GetTick();

        ArmBackgroundRx(hJQ);   /* absorb "OK" / "ERROR" / unsolicited bytes */

    } else {
        /*
         * Query command TX complete — arm receiver for the real response.
         */
        hJQ->intState = JQ6500_INT_RX;
        hJQ->tickMark = HAL_GetTick();

        (void)HAL_UARTEx_ReceiveToIdle_IT(hJQ->huart,
                                          hJQ->rxBuf,
                                          JQ6500_RX_BUF_SIZE);
    }
}

void JQ6500_RxEventCallback(JQ6500_Handle *hJQ, uint16_t size)
{
    /*
     * IDLE: unsolicited message (e.g. "STOP" when a track ends naturally).
     * DELAY: "OK" arriving during the post-command settle time.
     * Both cases: discard bytes and re-arm to keep catching future messages.
     */
    if (hJQ->intState == JQ6500_INT_IDLE ||
        hJQ->intState == JQ6500_INT_DELAY) {
        ArmBackgroundRx(hJQ);
        return;
    }

    if (hJQ->intState != JQ6500_INT_RX) {
        return;   /* Spurious callback guard */
    }

    /* ── All remaining paths are query responses ─────────────────────── */
    const JQ6500_QueryId qid = hJQ->pendingQueryId;

    /* ── Filename: raw ASCII, variable length ────────────────────────── */
    if (qid == JQ6500_QUERY_FILENAME) {
        const uint16_t cap = JQ6500_RX_BUF_SIZE - 1U;
        hJQ->rxBuf[(size < cap) ? size : cap] = '\0';
        hJQ->intState = JQ6500_INT_IDLE;
        FireEvent(hJQ, JQ6500_EVT_QUERY_DONE, qid, 0U, (const char *)hJQ->rxBuf);
        return;
    }

    /* ── All other queries: 4-char ASCII hex → uint16_t ─────────────── */
    uint16_t val;
    if (ParseHex(hJQ->rxBuf, size, &val) != JQ6500_OK) {
        hJQ->intState = JQ6500_INT_IDLE;
        FireEvent(hJQ, JQ6500_EVT_ERROR, qid, 0U, NULL);
        return;
    }

    /* ── Robust status: accumulate samples ───────────────────────────── */
    if (qid == JQ6500_QUERY_STATUS_ROBUST) {
        if ((uint8_t)val < 3U) {
            hJQ->statusCounts[(uint8_t)val]++;
        }
        hJQ->statusSamplesLeft--;

        if (hJQ->statusSamplesLeft > 0U) {
            hJQ->afterDelay = JQ6500_AFTER_STATUS_SAMPLE;
            hJQ->delayMs    = JQ6500_STATUS_SAMPLE_DELAY_MS;
            hJQ->intState   = JQ6500_INT_DELAY;
            hJQ->tickMark   = HAL_GetTick();
            ArmBackgroundRx(hJQ);   /* keep background active during sample delay */
        } else {
            uint8_t best = 0U;
            for (uint8_t i = 1U; i < 3U; i++) {
                if (hJQ->statusCounts[i] > hJQ->statusCounts[best]) {
                    best = i;
                }
            }
            hJQ->intState = JQ6500_INT_IDLE;
            FireEvent(hJQ, JQ6500_EVT_QUERY_DONE, qid, (uint16_t)best, NULL);
        }
        return;
    }

    /* ── Flash index: hardware reports (actual_index - 1) ───────────── */
    if (qid == JQ6500_QUERY_FLASH_INDEX) {
        val++;
    }

    hJQ->intState = JQ6500_INT_IDLE;
    FireEvent(hJQ, JQ6500_EVT_QUERY_DONE, qid, val, NULL);
}

bool JQ6500_IsBusy(const JQ6500_Handle *hJQ)
{
    return (hJQ->intState != JQ6500_INT_IDLE);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Control commands
 * ═══════════════════════════════════════════════════════════════════════════*/

JQ6500_Result JQ6500_Play(JQ6500_Handle *hJQ) {
    return StartCmd(hJQ, CMD_PLAY, NULL, 0U, JQ6500_QUERY_NONE, 0U);
}
JQ6500_Result JQ6500_Pause(JQ6500_Handle *hJQ) {
    return StartCmd(hJQ, CMD_PAUSE, NULL, 0U, JQ6500_QUERY_NONE, 0U);
}
JQ6500_Result JQ6500_Next(JQ6500_Handle *hJQ) {
    return StartCmd(hJQ, CMD_NEXT, NULL, 0U, JQ6500_QUERY_NONE, 0U);
}
JQ6500_Result JQ6500_Prev(JQ6500_Handle *hJQ) {
    return StartCmd(hJQ, CMD_PREV, NULL, 0U, JQ6500_QUERY_NONE, 0U);
}
JQ6500_Result JQ6500_Reset(JQ6500_Handle *hJQ) {
    return StartCmd(hJQ, CMD_RESET, NULL, 0U, JQ6500_QUERY_NONE, JQ6500_RESET_DELAY_MS);
}
JQ6500_Result JQ6500_Sleep(JQ6500_Handle *hJQ) {
    return StartCmd(hJQ, CMD_SLEEP, NULL, 0U, JQ6500_QUERY_NONE, 0U);
}
JQ6500_Result JQ6500_PlayIndex(JQ6500_Handle *hJQ, uint16_t index) {
    uint8_t args[2] = { (uint8_t)(index >> 8U), (uint8_t)(index & 0xFFU) };
    return StartCmd(hJQ, CMD_PLAY_INDEX, args, 2U, JQ6500_QUERY_NONE, 0U);
}
JQ6500_Result JQ6500_PlayFolderFile(JQ6500_Handle *hJQ, uint8_t folder, uint8_t file) {
    uint8_t args[2] = { folder, file };
    return StartCmd(hJQ, CMD_PLAY_FOLDER, args, 2U, JQ6500_QUERY_NONE, 0U);
}
JQ6500_Result JQ6500_ChangeFolder(JQ6500_Handle *hJQ, JQ6500_FolderDir dir) {
    uint8_t arg = (uint8_t)dir;
    return StartCmd(hJQ, CMD_CHANGE_FOLDER, &arg, 1U, JQ6500_QUERY_NONE, 0U);
}
JQ6500_Result JQ6500_VolUp(JQ6500_Handle *hJQ) {
    return StartCmd(hJQ, CMD_VOL_UP, NULL, 0U, JQ6500_QUERY_NONE, 0U);
}
JQ6500_Result JQ6500_VolDown(JQ6500_Handle *hJQ) {
    return StartCmd(hJQ, CMD_VOL_DN, NULL, 0U, JQ6500_QUERY_NONE, 0U);
}
JQ6500_Result JQ6500_SetVolume(JQ6500_Handle *hJQ, uint8_t volume) {
    if (volume > JQ6500_MAX_VOLUME) volume = JQ6500_MAX_VOLUME;
    return StartCmd(hJQ, CMD_SET_VOLUME, &volume, 1U, JQ6500_QUERY_NONE, 0U);
}
JQ6500_Result JQ6500_SetEqualizer(JQ6500_Handle *hJQ, JQ6500_EqMode eq) {
    uint8_t arg = (uint8_t)eq;
    return StartCmd(hJQ, CMD_SET_EQ, &arg, 1U, JQ6500_QUERY_NONE, 0U);
}
JQ6500_Result JQ6500_SetLoopMode(JQ6500_Handle *hJQ, JQ6500_LoopMode loop) {
    uint8_t arg = (uint8_t)loop;
    return StartCmd(hJQ, CMD_SET_LOOP, &arg, 1U, JQ6500_QUERY_NONE, 0U);
}
JQ6500_Result JQ6500_SetSource(JQ6500_Handle *hJQ, JQ6500_Source source) {
    uint8_t arg = (uint8_t)source;
    return StartCmd(hJQ, CMD_SET_SOURCE, &arg, 1U, JQ6500_QUERY_NONE, 0U);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Query commands
 * ═══════════════════════════════════════════════════════════════════════════*/

JQ6500_Result JQ6500_GetStatus(JQ6500_Handle *hJQ) {
    return StartCmd(hJQ, CMD_GET_STATUS, NULL, 0U, JQ6500_QUERY_STATUS, 0U);
}

JQ6500_Result JQ6500_GetStatusRobust(JQ6500_Handle *hJQ)
{
    if (hJQ->intState != JQ6500_INT_IDLE) return JQ6500_BUSY;
    hJQ->statusCounts[0]   = 0U;
    hJQ->statusCounts[1]   = 0U;
    hJQ->statusCounts[2]   = 0U;
    hJQ->statusSamplesLeft = JQ6500_STATUS_SAMPLES;
    hJQ->postTxDelayMs     = 0U;
    hJQ->afterDelay        = JQ6500_AFTER_DONE;
    return StartTx(hJQ, CMD_GET_STATUS, NULL, 0U, JQ6500_QUERY_STATUS_ROBUST);
}

JQ6500_Result JQ6500_GetVolume(JQ6500_Handle *hJQ) {
    return StartCmd(hJQ, CMD_GET_VOLUME, NULL, 0U, JQ6500_QUERY_VOLUME, 0U);
}
JQ6500_Result JQ6500_GetEqualizer(JQ6500_Handle *hJQ) {
    return StartCmd(hJQ, CMD_GET_EQ, NULL, 0U, JQ6500_QUERY_EQ, 0U);
}
JQ6500_Result JQ6500_GetLoopMode(JQ6500_Handle *hJQ) {
    return StartCmd(hJQ, CMD_GET_LOOP, NULL, 0U, JQ6500_QUERY_LOOP, 0U);
}
JQ6500_Result JQ6500_GetVersion(JQ6500_Handle *hJQ) {
    return StartCmd(hJQ, CMD_GET_VERSION, NULL, 0U, JQ6500_QUERY_VERSION, 0U);
}
JQ6500_Result JQ6500_CountSDFiles(JQ6500_Handle *hJQ) {
    return StartCmd(hJQ, CMD_COUNT_SD, NULL, 0U, JQ6500_QUERY_COUNT_SD, 0U);
}
JQ6500_Result JQ6500_CountFlashFiles(JQ6500_Handle *hJQ) {
    return StartCmd(hJQ, CMD_COUNT_FLASH, NULL, 0U, JQ6500_QUERY_COUNT_FLASH, 0U);
}
JQ6500_Result JQ6500_CountSDFolders(JQ6500_Handle *hJQ) {
    return StartCmd(hJQ, CMD_COUNT_FOLDERS, NULL, 0U, JQ6500_QUERY_COUNT_FOLDERS, 0U);
}
JQ6500_Result JQ6500_GetSDCurrentIndex(JQ6500_Handle *hJQ) {
    return StartCmd(hJQ, CMD_GET_SD_INDEX, NULL, 0U, JQ6500_QUERY_SD_INDEX, 0U);
}
JQ6500_Result JQ6500_GetFlashCurrentIndex(JQ6500_Handle *hJQ) {
    return StartCmd(hJQ, CMD_GET_FLASH_INDEX, NULL, 0U, JQ6500_QUERY_FLASH_INDEX, 0U);
}
JQ6500_Result JQ6500_GetPosition(JQ6500_Handle *hJQ) {
    return StartCmd(hJQ, CMD_GET_POSITION, NULL, 0U, JQ6500_QUERY_POSITION, 0U);
}
JQ6500_Result JQ6500_GetLength(JQ6500_Handle *hJQ) {
    return StartCmd(hJQ, CMD_GET_LENGTH, NULL, 0U, JQ6500_QUERY_LENGTH, 0U);
}
JQ6500_Result JQ6500_GetFileName(JQ6500_Handle *hJQ) {
    return StartCmd(hJQ, CMD_GET_FILENAME, NULL, 0U, JQ6500_QUERY_FILENAME, 0U);
}

#if JQ6500_PROVIDE_HAL_CALLBACKS
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (s_activeHandle && huart == s_activeHandle->huart) JQ6500_TxCpltCallback(s_activeHandle);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (s_activeHandle && huart == s_activeHandle->huart) JQ6500_RxEventCallback(s_activeHandle, size);
}
#endif
