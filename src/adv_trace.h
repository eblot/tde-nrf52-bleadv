/**
 * Standard set of macros for debug traces
 *
 * * @defgroup pa_trace @{
 */

#ifndef _ADV_TRACE_H_
#define _ADV_TRACE_H_

#include <stdint.h>
#include <stdbool.h>
#include <sys/types.h>
#include "adv_tools.h"

/**
 * Mandatory definitons
 *    @c PTM_SOURCE the source of traces, as defined in pa_tracemods.h
 *
 * Optional definitions
 *    @c DUMP_METHODS if defined, method 'enter' and 'leave' trace are
 *       produced
 *
 * Notes
 *    Macros with the underscore character prefix are not to be used in
 *    external code.
 */


//------------------------------------------------------------------------------
// type definitions
//------------------------------------------------------------------------------

/** Level of trace */
enum pa_trace_level {
    PTL_CHATTY, /**< Highly verbose trace msgs, such as function calls */
    PTL_DEBUG,  /**< Verbose trace msgs, such as function calls */
    PTL_INFO,   /**< Medium verbosity trace msgs, informational */
    PTL_WARN,   /**< Reduced verbosity trace msgs, warnings */
    PTL_ERROR,  /**< Rare verbosity trace msgs, errors and critical issues */
    PTL_FATAL,  /**< Always emit a trace, bypassing any test. This trace level
                    is reserved for unexpected and unrecoverable issues */
    PTL_OFF = PTL_FATAL, /**< Trace msgs are disabled for the source */
    PTL_COUNT, /**< Watermark, not a trace level */
    PTL_FUNC = PTL_COUNT, /** Special marker for functions, do not use */
};


//------------------------------------------------------------------------------
// default definitions
//------------------------------------------------------------------------------

/** stringification macro, step 2 */
#define _ADV_TRACE_XSTR_(s) _ADV_TRACE_STR_(s)
/** stringification macro, step 1 */
#define _ADV_TRACE_STR_(s) #s

/**
 * Optional definitions
 * @c PTM_NAME component name (appears before the method name in traces)
 */
#ifndef PTM_NAME
#define _PTM_NAME "(noname)"
#else // ! PTM_NAME
#define _PTM_NAME _ADV_TRACE_XSTR_(PTM_NAME)
#endif // PTM_NAME

/*@{
 * @internal
 */
#define __FUNCTIONNAME__ __func__
#define _MSGLOC_ _PTM_NAME "[" _ADV_TRACE_XSTR_(__LINE__) "]"
#ifndef _MSGLOC_
#define _MSGLOC_ ""
#endif // _MSGLOC_
/*@} */


//------------------------------------------------------------------------------
// Trace formatting
//------------------------------------------------------------------------------

void pa_trace_init(void);
int pa_printf(const char* fmt, ...) __attribute__((format(printf,1,2)));
int pa_print(const char * message, size_t len);
int pa_trace_printf(enum pa_trace_level level, const char * fmt, ...)
    __attribute__((format(printf,2,3)));
bool pa_trace_is_traceable(int source, enum pa_trace_level level);
bool pa_trace_is_funcable(int source);
bool pa_trace_is_irq(void);
void pa_trace_set_source(int source, enum pa_trace_level level);
enum pa_trace_level pa_trace_get_source(int source);
void pa_trace_fatal_error(const char * message, size_t length);
#ifdef HAVE_DUMP_HEX
void pa_trace_dump_hex(const void * buf, size_t len);
#endif // HAVE_DUMP_HEX
size_t pa_trace_build_hex(char * dst, size_t dlen, const void * buffer,
                          size_t blen);

extern void app_error_handler(uint32_t c, uint32_t l, const uint8_t * f);

// FORCE_RELEASE_TRACES forces traces to be printed out even in release builds
// use with care, not for production!
// When FORCE_RELEASE_TRACES is not enabled globally, it is is recommended to
// also enable it for pa_trace itself if (and only if) another component uses
// this -dangerous- feature.
#if defined(DEBUG) || defined(FORCE_RELEASE_TRACES)
#  define TPRINTF pa_trace_printf
#else // DEBUG || FORCE_RELEASE_TRACES
#  undef TPRINTF
#endif // ! (DEBUG || FORCE_RELEASE_TRACES)

/// @todo remove me once obsolescence period is over (January 2011)
#ifdef DUMP_THREAD_ID
#error DUMP_THREAD_ID should be removed
#endif // DUMP_THREAD_ID

/// @todo remove me once obsolescence period is over (January 2011)
#ifdef DUMP_METHODS
#error DUMP_METHODS should be removed
#endif // DUMP_METHODS

/*@{
 * @internal
 */

/** Safe compound macro */
#define _CM(body) do { body } while (0)

#define _INMSG    "> "
#define _OUTMSG   "< "
#define _NOMSG    ""

/** console carriage return, linefeed */
#define CRLF "\n"
#define EOL CRLF

/*@} */

/**
 * Test whether a condition evaluate to a true value or emit an error message
 * and return an error code
 */
#define NS_VERIFY_R(cond, msg, ret) if ( !(cond) ) { ROUTR(msg, ret); }

#include "adv_tracesrcs.h"

#if !defined(PTM_SOURCE) && !defined(_ADV_TRACE_H_)
// PTM_SOURCE should be defined
#error "Please define PTM_SOURCE before using pa_trace features"
#endif // PTM_SOURCE

#ifdef TPRINTF
#define ADV_TRACE_MSGV_ACTIVE 1
#  define _MSGV(_pre_, _fmt_, ...) \
    do { \
      if ( pa_trace_is_irq() ) { \
         app_error_handler(0xbadcaca0, __LINE__, (const uint8_t*) __FILE__); \
      } \
      if ( pa_trace_is_funcable(PTM_SOURCE) ) { \
         TPRINTF(PTL_FUNC, _pre_ _MSGLOC_ " %s() " _fmt_, \
                 __FUNCTIONNAME__, ##__VA_ARGS__); } \
    } while (false);

/**
 * Emit a fully formatted trace message, with component name, function and line
 * from which the message is emitted.
 *
 * @param[in] _lvl_ format and emit the message only if the trace level of the
 *                  current source matches or exceeds this level
 * @param[in] _fmt_ printf-like formatter string
 */
#  define MSGV(_lvl_, _fmt_, ...) \
    do { \
      if ( pa_trace_is_irq() ) { \
         app_error_handler(0xbadcaca1, __LINE__, (const uint8_t*) __FILE__); \
      } \
      if ( pa_trace_is_traceable(PTM_SOURCE, (enum pa_trace_level)_lvl_) ) { \
         TPRINTF(_lvl_, _NOMSG _MSGLOC_ " %s() " _fmt_ EOL, \
                 __FUNCTIONNAME__, ##__VA_ARGS__); } \
    } while (false);

/**
 * Emit a short trace message, without any extra information such as the
 * location of the message emitter.
 *
 * @param[in] _lvl_ format and emit the message only if the trace level of the
 *                  current source matches or exceeds this level
 * @param[in] _fmt_ printf-like formatter string
 */
#  define SMSGV(_lvl_, _fmt_, ...) \
    if ( pa_trace_is_traceable(PTM_SOURCE, (enum pa_trace_level)_lvl_) ) \
        { TPRINTF(_lvl_, _fmt_ EOL, ##__VA_ARGS__); }
#  define DPHX(_lvl_, _buf_, _len_) \
    if ( pa_trace_is_traceable(PTM_SOURCE, (enum pa_trace_level)_lvl_) ) \
        { pa_trace_dump_hex(_buf_, _len_) ; }
#  define DPHXM(_lvl_, _buf_, _len_, _fmt_, ...) \
    if ( pa_trace_is_traceable(PTM_SOURCE, (enum pa_trace_level)_lvl_) ) \
        { TPRINTF(_lvl_, _NOMSG _MSGLOC_ " %s() " _fmt_ EOL, \
            __FUNCTIONNAME__, ##__VA_ARGS__); \
          pa_trace_dump_hex(_buf_, _len_) ; }
/**
 * Emit a function-entering trace message.
 *
 * @param[in] _fmt_ printf-like formatter string
 */
#  define INV(_fmt_, ...) \
        _MSGV(_INMSG _NOMSG, _fmt_ EOL, ##__VA_ARGS__)
/**
 * Emit a function-leaving trace message and returns from the current function
 *
 * @param[in] _fmt_ character string to emit (w/o formatter)
 */
#  define ROUT(_fmt_) \
        _CM(_MSGV(_OUTMSG _NOMSG, _fmt_ EOL); return;)
#  define OUTV(_fmt_, ...) \
        _MSGV(_OUTMSG _NOMSG, _fmt_ EOL, ##__VA_ARGS__)
/**
 * Emit a function-leaving trace message and returns the specfied value from
 * the current function
 *
 * @param[in] _fmt_ printf-like formatter string
 * @param[in] _res_ the value to return, also used as the argument of the
 *                  formatter string
 */
#  define ROUTR(_fmt_, _res_) \
        _CM(_MSGV(_OUTMSG _NOMSG, _fmt_ EOL, _res_); return (_res_);)
#else // TPRINTF
#  define _MSGV(_pre_, _fmt_, ...)
#  define MSGV(_lvl_, _fmt_, ...)
#  define DPHX(_lvl_, _buf_, _len_)
#  define DPHXM(_lvl_, _buf_, _len_, _fmt_, ...)
#  define INV(_fmt_, ...)
#  define ROUT(_fmt_)        return
#  define OUTV(_fmt_, ...)
#  define ROUTR(_fmt_, _res_)  return (_res_)
#endif // ! TPRINTF

/** @} */

#endif // _ADV_TRACE_H_
