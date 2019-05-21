/**
 * @file adv_main.c
 **/

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrf_warn_enter.h"
#include "app_error.h"
#include "app_timer.h"
#include "ble.h"
#include "ble_advertising.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_gpio.h"
#include "app_scheduler.h"
#include "nrf_sdh.h"
#include "nrf_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_warn_leave.h"
#include "adv_ble.h"
#include "adv_tools.h"
#include "adv_trace.h"

# pragma clang diagnostic ignored "-Wunused-function"

//-----------------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------------

#define SCHED_QUEUE_SIZE       20
#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE

#define ADV_MAIN_OBSERVER_PRIO 1U

/**
 * Value used as error code on stack dump, can be used to identify stack
 *location on stack unwind.
 */
#define DEAD_BEEF 0xDEADBEEFU

//-----------------------------------------------------------------------------
// Forward declarations
//-----------------------------------------------------------------------------

static void _pa_main_evt_handler(uint32_t sys_evt, void * context);
static void _pa_main_timers_init(void);
static void _sys_evt_dispatch(uint32_t sys_evt);
static void _pa_main_power_manage(void);

//-----------------------------------------------------------------------------
// Variables
//-----------------------------------------------------------------------------

// nRF52 SDK14 has found that messing around with linker (ordered) sections is
// a handy trick to register observers at build time. While this may sound
// clever, this is a recipe for disaster when public SDKs. Anyway, there are
// now no other choices than this crap.
NRF_SDH_SOC_OBSERVER(_pa_main_observer, ADV_MAIN_OBSERVER_PRIO,
                     &_pa_main_evt_handler, NULL);

//-----------------------------------------------------------------------------
// Public API
//-----------------------------------------------------------------------------

/**
 * Function for application main entry.
 */
int
main(void)
{
   pa_trace_init();

   _pa_main_timers_init();
   APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);

   adv_ble_init();

   // Start execution.
   MSGV(PTL_INFO, "Advertiser " ADV_SW_VERSION " starting");
   adv_ble_start();

   // Enter main loop.
   for (;;) {
      app_sched_execute();
      _pa_main_power_manage();
   }
}

/**
 * Callback function for asserts in the SoftDevice.
 *
 * This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product.
 * You need to analyze how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on
 * reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void
assert_nrf_callback(uint16_t line_num, const uint8_t * file_name)
{
   app_error_handler(DEAD_BEEF, line_num, file_name);
}

void __attribute__((noreturn))
app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
   const error_info_t * const error = (const error_info_t * const)info;

   const char * filename = strrchr((const char *)error->p_file_name, '/');
   if ( (uintptr_t)filename > (uintptr_t)error->p_file_name ) {
      filename += 1;
   }

   char fatal_msg[120];
   size_t fatal_len;
   fatal_len = (size_t)snprintf(fatal_msg, sizeof(fatal_msg),
                                "FAULT:%08x PC:%08x @ %s:%d error 0x%04x\n",
                                id, pc, filename, error->line_num,
                                error->err_code);
   pa_trace_fatal_error(fatal_msg, fatal_len);
   #ifndef DEBUG
   NVIC_SystemReset();
   #endif // !DEBUG
   // never reached in RELEASE build, but as Nordic fails to declare the
   // reset API as non-returning, the compiler would complain about a
   // non-returning function potentially returning.
   for(;;) {
      __WFE();
   }
}

//-----------------------------------------------------------------------------
// Private API
//-----------------------------------------------------------------------------

/**
 * Function for the Timer initialization.
 *
 * Initializes the timer module. This creates and starts application timers.
 */
static void
_pa_main_timers_init(void)
{
   // Initialize timer module.
   ret_code_t rc = app_timer_init();
   APP_ERROR_CHECK(rc);
}

/**
 * Dispatch a system event to interested modules.
 * This function is called from the System event interrupt handler after a
 * system event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void
_pa_main_evt_handler(uint32_t sys_evt, void * context _unused)
{
#if FSTORAGE_ENABLED
   // Dispatch the system event to the fstorage module, where it will be
   // dispatched to the Flash Data Storage (FDS) module.
   fs_sys_event_handler(sys_evt);
#endif  // FSTORAGE_ENABLED

   // Dispatch to the Advertising module last, since it will check if there are
   // any pending flash operations in fstorage. Let fstorage process system
   // events first, so that it can report correctly to the Advertising module.
   ble_advertising_t * adv;
   adv_ble_get_advertising(&adv);
   ble_advertising_on_sys_evt(sys_evt, adv);
}

/**
 * Wait for an event
 */
static void
_pa_main_power_manage(void)
{
   ret_code_t rc = sd_app_evt_wait();
   APP_ERROR_CHECK(rc);
}
