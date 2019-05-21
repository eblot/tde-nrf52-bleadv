/**
 * PowerAdv debug traces
 *
 * * @file pn_trace.c
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <limits.h>
#include "nrf_warn_enter.h"
#include "app_util_platform.h"
#include "nordic_common.h"
#include "nrfx_uarte.h"
#include "app_timer.h"
#include "nrf_warn_leave.h"
#include "adv_trace.h"
#include "adv_tools.h"
#include "adv_tracesrcs.h"

#pragma clang diagnostic ignored "-Wunused-function"
#pragma clang diagnostic ignored "-Wunused-macros"

//-----------------------------------------------------------------------------
// Configuration
//-----------------------------------------------------------------------------

// define to emit a trace with timestamp
#define ADV_TRACE_SHOW_TIME
// define to emit a trace with a trace counter
#define ADV_TRACE_SHOW_COUNT
// define to emit a trace with an IRQ context
#undef ADV_TRACE_SHOW_CTX

//-----------------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------------

/** Maximum length of a debug trace. */
#define ADV_TRACE_MSG_LENGTH       127
/** Maximum count of a debug traces in transmit queue. */
#define ADV_TRACE_MSG_COUNT_LOG2   7U // 128

#define MSG_QUEUE_SIZE            (1U << ADV_TRACE_MSG_COUNT_LOG2)
#define MSG_QUEUE_MASK            (MSG_QUEUE_SIZE - 1U)

#define DISABLE_ALL_TRACES_MASK   ((1ULL << 32U) - 1U)
#define PTS_BITS                  (CHAR_BIT * sizeof(uint32_t))  // 32 bits
#define PTS_MASK                  (PTS_BITS - 1)
#define PTS_SHIFT                 5  // 32 bits = 2^5
#define PTL_LVLBITS               3  // 8 level (3 bits)
#define PTL_BITS                  4  // 8 level (3 bits) + 1 bit extra per src
#define PTL_MASK                  ((1U << (PTL_LVLBITS)) -1)
#define PTM_BITS                  (PTL_BITS * PTM_COUNT)
#define PTN_WORDS                 ((PTM_BITS + (PTS_BITS - 1)) / PTS_BITS)

#define _BPLS                     4U  // Byte per line
#define _BPL                      (1U << _BPLS)

#define ADV_TRACE_LOGLEVEL         (1U << 0UL)

#ifdef ADV_TRACE_SHOW_TIME
#define ADV_TRACE_FMT_TIME         "^%08x "
#else // ADV_TRACE_SHOW_TIME
#define ADV_TRACE_FMT_TIME         ""
#endif // !ADV_TRACE_SHOW_TIME
#ifdef ADV_TRACE_SHOW_COUNT
#define ADV_TRACE_FMT_COUNT        ":%02x "
#else // ADV_TRACE_SHOW_COUNT
#define ADV_TRACE_FMT_COUNT        ""
#endif // !ADV_TRACE_SHOW_COUNT
#ifdef ADV_TRACE_SHOW_CTX
#define ADV_TRACE_FMT_CTX          "{%02x} "
#else // ADV_TRACE_SHOW_CTX
#define ADV_TRACE_FMT_CTX          ""
#endif // !ADV_TRACE_SHOW_CTX

#define ADV_TRACE_FMT_HEADER \
   ADV_TRACE_FMT_TIME ADV_TRACE_FMT_COUNT ADV_TRACE_FMT_CTX

ASSERT_COMPILE((1U << PTS_SHIFT) == PTS_BITS);
ASSERT_COMPILE(PTL_COUNT <= 8);
// PTM_COUNT can be augmented, nevertheless for now we want to control
// the enumeration count
ASSERT_COMPILE(PTM_COUNT == 32);

//-----------------------------------------------------------------------------
// Type definitions
//-----------------------------------------------------------------------------

/** A trace message */
struct pa_trace_msg {
   char tm_msg[ADV_TRACE_MSG_LENGTH]; /** Message buffer */
   uint8_t tm_length;                /** Message length */
};

ASSERT_COMPILE(sizeof(struct pa_trace_msg) == 128);

/** Trace queue */
struct pa_trace_queue {
   /** Current index where the next message to be transmitted resides. */
   off_t tq_read_p;
   /** Current index where the next message should be inserted. */
   off_t tq_write_p;
   /** Transmit buffer for messages to be transmitted to the central. */
   struct pa_trace_msg tq_msgs[MSG_QUEUE_SIZE];
};

struct pa_trace {
   uint32_t pt_masks[PTN_WORDS]; /** Levels for each trace source  */
   bool pt_initialized; /** Trace subsystem has been initialised */
   uint8_t pt_count;    /** Overflowing counter to track lost messages */
   bool pt_que_active;  /** FIFO queue is being sent */
   struct pa_trace_queue * pt_que; /** Message queue */
   nrfx_uarte_t pt_uart; /** UART instance */
};

//-----------------------------------------------------------------------------
// Forward declarations
//-----------------------------------------------------------------------------

static struct pa_trace _pa_trace;

static void _nd_trace_uart_event_handler(const nrfx_uarte_event_t * event,
   void * context);
static void _pa_trace_pop_queue(struct pa_trace * trace);
static void _pa_trace_start_queue(struct pa_trace * trace);

//-----------------------------------------------------------------------------
// Static constants
//-----------------------------------------------------------------------------

#if ! defined(NRFX_UARTE_ENABLED)
// Yes, nRF5 SDK UART definition is a mess...
#error Invalid UART configuration
#endif

/** UART debug port configuration */
static const nrfx_uarte_config_t _pa_trace_uart_config = {
   .pseltxd = 18U,  // SWO
   .pselrxd = NRF_UARTE_PSEL_DISCONNECTED,
   .pselcts = NRF_UARTE_PSEL_DISCONNECTED,
   .pselrts = NRF_UARTE_PSEL_DISCONNECTED,
   .p_context = &_pa_trace,
   .hwfc = NRF_UARTE_HWFC_DISABLED,
   .parity = NRF_UARTE_PARITY_EXCLUDED,
   .baudrate = NRF_UARTE_BAUDRATE_1000000,
   .interrupt_priority = 3U,
   //.use_easy_dma = true,
};

/** Log level code */
static const char ADV_TRACE_LOGLEVELS[] = "CDIWEF.";

/** Default log level for each source at start up */
static const uint8_t ADV_TRACE_DEFAULT_LEVELS[] = {
   [PTM_MAIN] = PTL_DEBUG,
   [PTM_SYS] = PTL_DEBUG,
   [PTM_BLE] = PTL_CHATTY,
};

//-----------------------------------------------------------------------------
// Static variables
//-----------------------------------------------------------------------------

struct pa_trace_queue _pa_trace_queue;

/** Trace configuration */
static struct pa_trace _pa_trace = {
   .pt_uart = NRFX_UARTE_INSTANCE(0),
   .pt_que = &_pa_trace_queue,
};

//-----------------------------------------------------------------------------
// Inline function
//-----------------------------------------------------------------------------

/**
 * Provide current timestamp
 *
 * @return timestamp
 */
static inline uint32_t
_pa_trace_time(void)
{
   return app_timer_cnt_get();
}

/**
 * Provide current context (IRQ handler or regular
 *
 * @return masked IPSR
 */
static inline uint32_t
_pa_trace_context(void)
{
   return __get_IPSR() & 0xFFU;
}

/**
 * Discards any stored data in the trace messsage queue.
 *
 * @param[in] que the trace messsage queue instance
 */
static inline void
_pa_trace_queue_flush(struct pa_trace_queue * que)
{
   que->tq_read_p = que->tq_write_p;
}

/**
 * Count how many free slots are available in the trace messsage queue.
 *
 * @param[in] que the trace messsage queue instance
 * @return the number of free bytes in the trace messsage queue
 */
static inline ssize_t
_pa_trace_queue_count_free(const struct pa_trace_queue * que)
{
   ssize_t bytes;

   bytes = que->tq_read_p - que->tq_write_p;
   if ( bytes <= 0 ) {
       bytes += MSG_QUEUE_SIZE;
   }

   return bytes-1;
}

/**
 * Count how many busy slots are available from the trace messsage queue.
 *
 * @param[in] que the trace messsage queue instance
 * @return the number of readable bytes in the trace messsage queue
 */
static inline ssize_t
_pa_trace_queue_count_avail(const struct pa_trace_queue * que)
{
   ssize_t bytes;

   bytes = que->tq_write_p - que->tq_read_p;
   if ( bytes < 0 ) {
       bytes += MSG_QUEUE_SIZE;
   }

   return bytes;
}

/**
 * Reports whether the trace messsage queue is empty or not.
 *
 * @param[in] que the trace messsage queue instance
 * @return @c true if the trace messsage queue contains no item
 */
static inline bool
_pa_trace_queue_is_empty(const struct pa_trace_queue * que)
{
   return que->tq_read_p == que->tq_write_p;
}

/**
 * Reports whether the trace messsage queue is full or not.
 *
 * @param[in] que the trace messsage queue instance
 * @return @c true if the trace messsage queue cannot receive more items
 */
static inline bool
_pa_trace_queue_is_full(const struct pa_trace_queue * que)
{
   return _pa_trace_queue_count_free(que) <= 0;
}

/**
 * Advance the read pointer.
 *
 * @param[in] que the trace messsage queue instance
 */
static inline void
_pa_trace_queue_r_next(struct pa_trace_queue * que)
{
   que->tq_read_p = ((que->tq_read_p + 1) & (off_t)MSG_QUEUE_MASK);
}

/**
 * Advance the write pointer.
 *
 * @param[in] que the trace messsage queue instance
 */
static inline void
_pa_trace_queue_w_next(struct pa_trace_queue * que)
{
   que->tq_write_p = (que->tq_write_p + 1) & (off_t)MSG_QUEUE_MASK;
}

//-----------------------------------------------------------------------------
// Public API
//-----------------------------------------------------------------------------

/**
 * Initialize the trace level at boot time, to enable the proper traces
 * as soon as the system boots up.
 * This implies that kernel parameters should be accessed directly, as they
 * have not been replicated as environment variables at this early stage.
 */
void
pa_trace_init(void)
{
   // disable all traces
   for (unsigned int pos = 0; pos < ARRAY_SIZE(_pa_trace.pt_masks); ++pos) {
      _pa_trace.pt_masks[pos] = DISABLE_ALL_TRACES_MASK;
   }

   nrfx_uarte_uninit(&_pa_trace.pt_uart);

   ret_code_t rc;
   rc = nrfx_uarte_init(&_pa_trace.pt_uart, &_pa_trace_uart_config,
                          &_nd_trace_uart_event_handler);
   (void)rc;

   // emit the initialization trace message to inform the host about
   // the target startup and the actual tick value
   char buffer[80];
   int len;
   len = snprintf(buffer,
                  sizeof(buffer),
                  CRLF ADV_TRACE_FMT_HEADER _MSGLOC_ " %s() tick: %u Hz" CRLF,
                  #ifdef ADV_TRACE_SHOW_TIME
                  0U,
                  #endif // ADV_TRACE_SHOW_TIME
                  #ifdef ADV_TRACE_SHOW_COUNT
                  _pa_trace.pt_count,
                  #endif // ADV_TRACE_SHOW_COUNT
                  #ifdef ADV_TRACE_SHOW_CTX
                  _pa_trace_context(),
                  #endif // ADV_TRACE_SHOW_CTX
                  __func__,
                  (unsigned int)(APP_TIMER_CLOCK_FREQ/
                                 (APP_TIMER_CONFIG_RTC_FREQUENCY+1)));
   _pa_trace.pt_initialized = true;
   pa_print(&buffer[0], (size_t)len);

   for (unsigned int six=0; six<ARRAY_SIZE(ADV_TRACE_DEFAULT_LEVELS); six++) {
      pa_trace_set_source((int)six, ADV_TRACE_DEFAULT_LEVELS[six]);
   }
}

/**
 * Tells whether a message from a source and a verbosity level can be
 * dumped to the debug port, depending on the currently defined value for
 * the source
 *
 * @param[in] source the source of the message
 * @param[in] level the verbosity level
 * @return a boolean value, set if message can be dumped out
 */
bool
pa_trace_is_traceable(int source, enum pa_trace_level level)
{
   // sanity check
   if ( (source >= PTM_COUNT) || (! _pa_trace.pt_initialized) ) {
      return false;
   }

   // index in the trace mask array
   unsigned int pos = (PTL_BITS * (unsigned int)source) >> PTS_SHIFT;
   // bits in the trace mask word
   unsigned int src = (PTL_BITS * (unsigned int)source) & PTS_MASK;

   return level >=
      ((_pa_trace.pt_masks[pos] >> src) & PTL_MASK);
}

/**
 * Tells whether a function input/output message can be dumped to the debug
 * port, depending on the currently defined value for this feature
 *
 * @param[in] source the source of the message
 * @return a boolean value, set if message can be dumped out
 */
bool
pa_trace_is_funcable(int source)
{
   return (pa_trace_is_traceable(source, PTL_CHATTY));
}

/**
 * Report whether the caller is running from IRQ context
 *
 * @return @c true if the current context is an IRQ
 */
bool
pa_trace_is_irq(void)
{
   return !!_pa_trace_context();
}

/**
 * Change the current verbosity level for a source.
 *
 * @param[in] source the source of the message
 * @param[in] level the verbosity level
 */
void
pa_trace_set_source(int source, enum pa_trace_level level)
{
   // sanity checks
   if (source >= PTM_COUNT) {
      return;
   }
   if (level >= PTL_COUNT) {
      return;
   }

   // index in the trace mask array
   unsigned int pos = (PTL_BITS * (unsigned int)source) >> PTS_SHIFT;
   // bits in the trace mask word
   unsigned int src = (PTL_BITS * (unsigned int)source) & PTS_MASK;

   // reset previous trace mask for the source
   _pa_trace.pt_masks[pos] &= ~(PTL_MASK << src);
   // set the new trace mask for the source
   _pa_trace.pt_masks[pos] |= (uint32_t)(level << src);
}

/**
 * Get the current verbosity level for a source.
 *
 * @param[in] source the source of the message
 * @return the verbosity level
 */
enum pa_trace_level
pa_trace_get_source(int source)
{
   // index in the trace mask array
   unsigned int pos = (PTL_BITS * (unsigned int)source) >> PTS_SHIFT;
   // bits in the trace mask word
   unsigned int src = (PTL_BITS * (unsigned int)source) & PTS_MASK;

   return (enum pa_trace_level)((_pa_trace.pt_masks[pos] >> src) & PTL_MASK);
}

/**
 * Prints out the content of a 8-bit descriptor to the debug port
 *
 * @pre should not be invoked from an ISR
 * @param[in] message the ASCII character string to be printed out
 * @param[in] len the count of meaningful characters in the string
 */
int
pa_print(const char * message, size_t len)
{
   struct pa_trace_queue * que = _pa_trace.pt_que;

   if ( ! _pa_trace.pt_initialized ) {
      return 0;
   }

   #ifndef ADV_TRACE_SHOW_CTX
   if ( _pa_trace_context() ) {
      // it is not possible to emit from an IRQ, as either the message FIFO
      // would be corrupted, or the whole printf() calls should be encapsulated
      // in critical sections. Therefore, only count the lost trace
      _pa_trace.pt_count++;
      return 0;
   }
   #endif // ADV_TRACE_SHOW_CTX

   if ( _pa_trace_queue_is_full(que) ) {
      // nothing can be done for now
      _pa_trace.pt_count++;
      return 0;
   }

   struct pa_trace_msg * msg = &que->tq_msgs[que->tq_write_p];
   char * const msg_buf = &msg->tm_msg[0];
   msg->tm_length = (uint8_t)MIN(len, ADV_TRACE_MSG_LENGTH);
   memcpy(msg_buf, message, msg->tm_length);
   _pa_trace.pt_count++;

   // commit this new message to the queue
   _pa_trace_queue_w_next(que);
   _pa_trace_start_queue(&_pa_trace);

   return (int)len;
}


/**
 * Prints a debug trace message to the debug port
 *
 * @pre should not be invoked from an ISR
 * @param[in] level the level of the trace message
 * @param[in] fmt a format string
 * @return the number of printed character
 * @note truncate oversized lines, and append a '...' marker to signal
 *       the truncature
 */
int
pa_trace_printf(enum pa_trace_level level, const char * fmt, ...)
{
   if ( ! _pa_trace.pt_initialized ) {
      return 0;
   }

   #ifndef ADV_TRACE_SHOW_CTX
   if ( _pa_trace_context() ) {
      // see note in #pa_print
      _pa_trace.pt_count++;
      return 0;
   }
   #endif // ADV_TRACE_SHOW_CTX

   struct pa_trace_queue * que = _pa_trace.pt_que;

   if ( _pa_trace_queue_is_full(que) ) {
      // nothing can be done for now
      _pa_trace.pt_count++;
      return 0;
   }

   struct pa_trace_msg * msg = &que->tq_msgs[que->tq_write_p];
   char * const msg_buf = &msg->tm_msg[0];

   #ifdef ADV_TRACE_SHOW_TIME
   uint32_t rt = _pa_trace.pt_initialized ? _pa_trace_time() : 0UL;
   #endif // ADV_TRACE_SHOW_TIME

   int ret;

   ret = snprintf(&msg_buf[0], ADV_TRACE_MSG_LENGTH,
                  ADV_TRACE_FMT_HEADER
                  #ifdef ADV_TRACE_SHOW_TIME
                  , rt
                  #endif // ADV_TRACE_SHOW_TIME
                  #ifdef ADV_TRACE_SHOW_COUNT
                  , _pa_trace.pt_count
                  #endif // ADV_TRACE_SHOW_COUNT
                  #ifdef ADV_TRACE_SHOW_CTX
                  , _pa_trace_context()
                  #endif // ADV_TRACE_SHOW_CTX
   );

   // if log level tracing is enabled, emit a marker
   if ( (level < PTL_COUNT) && ret < (ADV_TRACE_MSG_LENGTH - 2) ) {
      msg_buf[ret++] = ADV_TRACE_LOGLEVELS[level];
      msg_buf[ret++] = ' ';
   }

   va_list ap;

   va_start(ap, fmt);
   ret +=
      vsnprintf(&msg_buf[ret],
                ADV_TRACE_MSG_LENGTH - (unsigned int)ret - 1U, fmt, ap);
   va_end(ap);

   if (ret >= ADV_TRACE_MSG_LENGTH) {
      msg_buf[ADV_TRACE_MSG_LENGTH - 5] = '.';
      msg_buf[ADV_TRACE_MSG_LENGTH - 4] = '.';
      msg_buf[ADV_TRACE_MSG_LENGTH - 3] = '.';
      msg_buf[ADV_TRACE_MSG_LENGTH - 2] = '\n';
      msg_buf[ADV_TRACE_MSG_LENGTH - 1] = '\0';
      ret = ADV_TRACE_MSG_LENGTH;
   }

   _pa_trace.pt_count++;

   // commit this new message to the queue
   msg->tm_length = (uint8_t)ret;
   _pa_trace_queue_w_next(que);
   _pa_trace_start_queue(&_pa_trace);

   return ret;
}

/**
  * Define an alias for pa_printf that may be defined and called from an
  * external component, typically a cross-call from eCos
  */
int pa_printf_ext(const char * fmt, ...)
   __attribute__((weak, alias("pa_printf")));

/**
 * Prints a message to the debug port
 *
 * @pre should not be invoked from an ISR
 * @param[in] fmt a format string
 * @return the number of printed character
 */
int
pa_printf(const char * fmt, ...)
{
   struct pa_trace_queue * que = _pa_trace.pt_que;

   if ( ! _pa_trace.pt_initialized ) {
      return 0;
   }

   if ( _pa_trace_queue_is_full(que) ) {
      // nothing can be done for now
      _pa_trace.pt_count++;
      return 0;
   }

   struct pa_trace_msg * msg = &que->tq_msgs[que->tq_write_p];
   char * const msg_buf = &msg->tm_msg[0];

   va_list ap;
   int ret;

   va_start(ap, fmt);
   ret = vsnprintf(msg_buf, ADV_TRACE_MSG_LENGTH, fmt, ap);
   va_end(ap);

   msg->tm_length = (uint8_t)MIN(ADV_TRACE_MSG_LENGTH, ret);

   // commit this new message to the queue
   _pa_trace_queue_w_next(que);
   _pa_trace_start_queue(&_pa_trace);

   return ret;
}

void
pa_trace_fatal_error(const char * message, size_t length) {
   _pa_trace.pt_que_active = true;

   nrfx_uarte_tx(&_pa_trace.pt_uart, (const uint8_t *)message,
                 (uint8_t)length);
}

#ifdef HAVE_DUMP_HEX
#error not supported with current DMA support
/**
 * Dump the content of a binary buffer as hexadecimal values
 *
 * @pre should not be invoked from an ISR
 * @param[in] buffer the buffer to dump
 * @param[in] len the number of bytes to dump
 */
void
pa_trace_dump_hex(const void * buffer, size_t len)
{
   const unsigned char * buf = (unsigned char *)buffer;

   for (unsigned int pos = 0; pos < len;) {
      unsigned int count = MIN(_BPL, len - pos);

      char hexa[8 + 9 + 2 + 3 * _BPL - 1 + 2 + _BPL + 2 + 1];
      char * phexa  = &hexa[0];
      char * pascii = &hexa[8 + 9 + 2 + 3 * _BPL - 1 + 2];

      // reset the line with space characters
      memset(phexa, ' ', ARRAY_SIZE(hexa));
      phexa += sprintf(phexa, "%08x(+%06x) ", ((unsigned int)buf) + pos, pos);

      unsigned int p;
      for (p = 0; p < count; ++p) {
         unsigned char c = buf[pos + p];
         phexa += sprintf(phexa, "%02x ", c);
         pascii[p] = ((c >= 0x20) && (c < 0x7f)) ? ((char)c) : '.';
      }
      // remove the end-of-string marker before the ASCII text
      phexa[0] = ' ';
      // add up the end-of-line after the ASCII text
      memcpy(&pascii[p + 1], CRLF, ZARRAY_SIZE(CRLF));
      // ready to print the line
      pa_print(hexa, ARRAY_SIZE(hexa));

      pos += count;
   }
   pa_print(CRLF, ZARRAY_SIZE(CRLF));
}
#endif

/**
 * Format the content of a binary buffer as hexadecimal values.
 *
 * @note Destination buffer should be 3* input buffer size + 2 minimum to fit
 * all characters
 *
 * @param[in,out] dst destination string to update
 * @param[in] dlen maximum size ot the destination string
 * @param[in] buffer the buffer to dump
 * @param[in] blen the number of bytes to dump
 * @return the actual count of char in the destination buffer
 */
size_t
pa_trace_build_hex(char * dst, size_t dlen, const void * buffer, size_t blen)
{
   char * hexp = &dst[0];
   ssize_t rem = (ssize_t)dlen;
   for (unsigned int ix=0; ix<blen; ++ix) {
      int count = snprintf(hexp, (size_t)MAX(0, rem),
                           "%02x ", ((const uint8_t*)buffer)[ix]);
      if ( count >= rem ) {
         break;
      }
      rem -= count;
      hexp += count;
   }
   if ( hexp > dst ) {
      hexp--;
   }
   *hexp++ = '\0';

   return (size_t)(hexp-dst);
}

//-----------------------------------------------------------------------------
// Private implementation
//-----------------------------------------------------------------------------

static void
_nd_trace_uart_event_handler(const nrfx_uarte_event_t * event, void * context)
{
   switch ( event->type ) {
      case NRFX_UARTE_EVT_RX_DONE:
         return;
      case NRFX_UARTE_EVT_TX_DONE:
      case NRFX_UARTE_EVT_ERROR:
         break;
   }

   struct pa_trace * trace = (struct pa_trace *)context;
   struct pa_trace_queue * que = trace->pt_que;

   // the message has been consumed by UART DMA, discard it
   CRITICAL_REGION_ENTER();
   _pa_trace_queue_r_next(que);
   CRITICAL_REGION_EXIT();

   _pa_trace_pop_queue(trace);
}

static void
_pa_trace_start_queue(struct pa_trace * trace)
{
   if ( ! trace->pt_que_active ) {
      _pa_trace_pop_queue(&_pa_trace);
   }
}

static void
_pa_trace_pop_queue(struct pa_trace * trace)
{
   struct pa_trace_queue * que = trace->pt_que;

   for(;;) {
      bool empty;
      struct pa_trace_msg * msg;
      CRITICAL_REGION_ENTER();
      empty = _pa_trace_queue_is_empty(que);
      msg = &que->tq_msgs[que->tq_read_p];
      CRITICAL_REGION_EXIT();

      if ( empty ) {
         break;
      }

      _pa_trace.pt_que_active = true;

      ret_code_t rc;

      rc = nrfx_uarte_tx(&trace->pt_uart, (uint8_t *)msg->tm_msg,
                         msg->tm_length);


      if ( NRF_SUCCESS == rc ) {
         return;
      }

      CRITICAL_REGION_ENTER();
      _pa_trace_queue_r_next(que);
      CRITICAL_REGION_EXIT();
   }

   _pa_trace.pt_que_active = false;
}
