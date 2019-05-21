/**
 * Bluetooth advertiser
 * @file adv_ble.c
 * */

#include <stdint.h>
#include <string.h>
#include <limits.h>
#include "nrf_warn_enter.h"
#include "app_error.h"
#include "app_timer.h"
#include "ble.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "ble_dis.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_sdh.h"
#include "nrf_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrf_ble_gatt.h"
#include "nrf_warn_leave.h"
#include "adv_ble.h"
#include "adv_errors.h"
#include "adv_trace.h"
#include "adv_tools.h"


//-----------------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------------

/// @todo for production usage, these values need to be tailored

/** BLE TX power */
#define ADV_BLE_TX_POWER         4 // dBm
/** Delay after disconnection to enter sleep mode (& advertise @ slow pace) */
#define ADV_BLE_SLEEP_DELAY_S    3600  // 1 hour
/** Delay between each background work session in disconnected mode */
#define ADV_BLE_WORKER_PACE_S    5  // seconds
/** Delay without real BLE activity, after which a connection is closed */
#define ADV_BLE_STALL_DELAY_S    120  // seconds
/** Maximum delay for a command to execute. BLE core specs is 30s max */
#define PB_BLE_COMMAND_DELAY_S  10 // seconds
/// @todo enforce this w/ the worker thread to reboot on too long a completion

#define ADV_CHAR_UUID_BASE 0x1001

/** Reply when unsupported features are requested. */
#define APP_FEATURE_NOT_SUPPORTED (BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2)

/// @todo use proper values
#define MANUFACTURER_NAME      "Iroazh"
#define MANUFACTURER_ID        0x0006 // use M$ for now >:)
#define INFO_VERSION           0x1 /** Format version 1 */
#define MODEL_NUMBER           "Advertiser"
#define DEVICE_NAME_STR        "Adv" // keep it *very* short
#define HW_VERSION_TEMPLATE    "M.v.r-w" // only one decimal digit for v & r
#define HW_VERSION_FORMAT      "1.0.0"
#define FW_VERSION_FORMAT      "%u.%u.%u-S%3u-%u.%u"
#define FW_VERSION_TEMPLATE    "vx.y.z-Sddd-B.b"
#define BLE_LINK_VERSION_FIRST 6U // [0..5] are reserved numbers

#define ADV_BLE_OBSERVER_PRIO  2U
#define NORDIC_COMPANY_ID      0x0059U
#define NORDIC_SD_OFFSET       100U

//-----------------------------------------------------------------------------
// Macros
//-----------------------------------------------------------------------------

#define BLE_MAKE_ATTR_DESC(_s_) { .ad_str = (_s_), .ad_size = ZARRAY_SIZE(_s_) }

#define BLE_CHAR_RWN_PROP {.read = 1, .write = 1, .notify = 1}
#define BLE_CHAR_RW_PROP  {.read = 1, .write = 1, .notify = 0}
#define BLE_CHAR_RN_PROP  {.read = 1, .write = 0, .notify = 1}
#define BLE_CHAR_R_PROP   {.read = 1, .write = 0, .notify = 0}
#define BLE_CHAR_W_PROP   {.read = 0, .write = 1, .notify = 0}

/** syntactic sugar to access a BLE attribute storage location */
#define ADV_BLE_VAR(_var_, _mbr_) \
   ((STRUCT_TYPEOF(struct adv_ble_var, _mbr_)*)(_var_))

/** another syntactic sugar to access a BLE attribute storage location */
#define ADV_BLE_ATTR_VAR(_attr_, _mbr_) ADV_BLE_VAR((_attr_)->pa_var, _mbr_)

/** Index in BLE link version table */
#define BLE_LINK_VERSION_IDX(_code_) ((_code_)-(BLE_LINK_VERSION_FIRST))

/** Build a 32-bit compact version number */
#define NORDIC_SOFTDEVICE_VERSION(_sd_,_maj_,_min_,_patch_) \
   (((((_sd_)-(NORDIC_SD_OFFSET))&0xFFU)<<24U)|(((_maj_)&0xFFU)<<16U)| \
    (((_min_)&0xFFU)<<8U)|((_patch_)&0xFFU))

//-----------------------------------------------------------------------------
// Type definition
//-----------------------------------------------------------------------------

/**
 * PowerAdvertiser BLE attributes for service ADV_SERVICE_UUID
 *
 * @note remember that UUID values have a +1 offset (e.g. ADV_ERROR is 0x1001)
 */
enum adv_ble_attr {
   ADV_ERROR,       /**< 00: Last error code */
   ADV_COUNT,       /**< 0f: (watermark) */
};

#define ADV_FIRST   (0)
#define ADV_LAST    (ADV_COUNT)

/** Background worker */
enum adv_ble_background_worker {
   BW_COUNT,      /**< Watermark */
};

/** ADV_ERROR format: Error reporting record */
struct pa_error_desc {
   int8_t  pe_errno; /**< Errno code, see pa_errors.h */
   uint8_t pe_attr;  /**< Attribute as adv_ble_attr */
   uint8_t pe_state; /**< Power engine state (private enumeration) */
   uint8_t pe_comp;  /**< Power engine subcomponent */
   uint32_t pe_payload; /**< Versatile payload, depend on actual error */
};

ASSERT_COMPILE(sizeof(uint64_t) == sizeof(struct pa_error_desc));

/**
 * BLE link version.
 * @see https://www.bluetooth.com/specifications/assigned-numbers/link-layer
 */
struct adv_ble_link_version {
   uint8_t lv_major; /**< Major protocol version */
   uint8_t lv_minor; /**< Minor protocol version */
};

/** Nordic SoftDevice version */
struct adv_ble_nordic_sd_version {
   uint16_t sv_code;     /**< Nordic secret code */
   uint16_t sv_rsv;      /**< Not used for now */
   uint32_t sv_version;  /**< Compact 32-bit version */
};

/**
 * Manufacturer information to broadcast PN status in adveristement.
 * Beware that adverstising payload is sparse, so each byte should be used
 * wisely so that all info (not limited to this record) fit into a single
 * BLE payload.
 * Be also very careful with item alignment, as this structure is mem copied
 * as is.
 */
struct adv_ble_adv_info {
   uint8_t ai_version;  /** Version of record, for compatility purpose */
   struct pa_health {
      uint8_t pa_alert; /**< Alert bitfield */
      uint8_t ph_soc;   /**< State of charge in % */
   } ai_health;
};

ASSERT_COMPILE(sizeof(struct adv_ble_adv_info) == 3U);

/** BLE attribute physical container */
struct adv_ble_var {
   struct pa_error_desc pv_error;  /**< Last error description */
   uint8_t pv_transient[16];   /**< Transcient storage for deferred events */
};

// forward declarations
struct adv_ble;
struct adv_ble_attribute;

/**
 * Attribute writer signature.
 * Whenever a PowerAdvertiser-specific BLE attribute is written from the
 * peer, such a method is run to check if the parameters are valid and if the
 * PowerAdvertiser may serve such a request, and eventually handle the change.
 */
typedef int (* adv_ble_attr_writer_t)(const uint8_t * buf, size_t length);

/**
 * Attribute reader signature.
 * Whenever a PowerAdvertiser-specific BLE attribute is read from the
 * peer, such a method is run to validate and eventually compute the on-demand
 * value.
 */
typedef int (* adv_ble_attr_reader_t)(struct adv_ble_attribute * pa_attr);

/** Callback a worker should invoke on completion */
typedef void (*adv_ble_worker_cb_t)(void);

/** Worker routine signature */
typedef int (*adv_ble_worker_func_t)(void);

/** BLE attribute user description */
struct ble_attr_desc {
   char const * const ad_str;
   size_t ad_size;
};

/** BLE attribute record */
struct adv_ble_attribute {
   /** Universal identifier */
   ble_uuid_t                        pa_uuid;
   /** GATT attribute meta data */
   ble_gatts_attr_md_t const * const pa_attr_md;
   /** Storage size for the attribute value */
   const size_t                      pa_size;
   /** Storage container */
   void                            * pa_var;
   /** Count of meaningful bytes */
   size_t                            pa_length;
   /** GATT characteristic handles */
   ble_gatts_char_handles_t          pa_handles;
   /** Attribute properties */
   const ble_gatt_char_props_t       pa_props;
   /** Reader method, if any, for on-demand readable attribute */
   const adv_ble_attr_reader_t        pa_reader;
   /** Writer method, if any, for writable attribute */
   const adv_ble_attr_writer_t        pa_writer;
   /** Human-readable string to describe the attribute */
   const struct ble_attr_desc        pa_desc;
   /** Variable size argument */
   bool                              pa_varsize;
};

/** Structure to circumvent the limitations of the very poor nRF52 timer API */
struct adv_ble_timer {
   app_timer_t bt_timer; /**< Timer instance */
   app_timer_id_t bt_id; /**< nRF52 API identifier */
   uint32_t bt_expire;   /**< Absolute expiration time in app_timer ticks */
};

/** Background worker configuration */
struct adv_ble_worker {
   unsigned int bw_pace;         /**< Execution period (in seconds) */
   adv_ble_worker_func_t bw_func; /**< Worker routine */
};

/** Worker execution engine */
struct adv_ble_worker_engine {
   bool we_enable;                 /**< @c false to prevent any execution */
   bool we_running;                /**< @c false when no worker is executing */
   unsigned int we_time;           /**< Current engine time, in seconds */
   unsigned int we_last_time;      /**< Last active BLE communication time */
   unsigned int we_worker_ix;      /**< Current worker index */
   app_timer_id_t we_timer_id;     /**< Timer API */
   app_timer_t we_timer;           /**< Timer instance */
   // local worker storage area
   uint8_t we_bat_soc;             /**< Transcient battery SoC */
};

/** Record for delayed attribute writer completion */
struct adv_ble_attr_event {
   const uint8_t * ae_data; /**< Write event data buffer */
   uint16_t ae_length;      /**< Write event data size */
   uint16_t ae_offset;      /**< Write event data offset */
   struct adv_ble_attribute * ae_attr; /**< Write event destination attribute */
};

/** BLE PowerAdvertiser server engine */
struct adv_ble {
   /** PowerAdvertiser service handle */
   uint16_t bp_service_handle;
   /** Marker to speed up client discovery */
   uint16_t bp_last_service_handle;
   /** GATT module instance */
   nrf_ble_gatt_t bp_gatt;
   /** PowerAdvertiser Attributes */
   struct adv_ble_attribute bp_attributes[ADV_COUNT];
   /** Current client connexion, if any */
   uint16_t bp_conn_handle;
   /** Delayed attribute event completion */
   struct adv_ble_attr_event bp_attr_event;
   /** Timer to manage auto disconnection */
   struct adv_ble_timer bp_worker_timer;
   /** PowerAdvertiser is entering sleep and may accept no request */
   bool bp_entering_sleep;
   /** A reboot has been scheduled */
   bool bp_reboot;
   /** Simple anchor to embed SW version string into the final application */
   const char * bp_sw_version;
};

//-----------------------------------------------------------------------------
// Forward declarations
//-----------------------------------------------------------------------------

static void _adv_ble_stack_init(void);
static void _adv_ble_conn_init(void);
static void _adv_ble_gap_init(void);
static void _adv_ble_gatt_init(void);
static void _adv_ble_service_add(void);
static void _adv_ble_dis_init(void);
static void _adv_ble_advertising_init(void);
static void _adv_ble_evt_handler(const ble_evt_t * ble_evt, void * context);
static void _adv_ble_conn_evt(ble_conn_params_evt_t * ble_evt);
static void _adv_ble_conn_error_handler(uint32_t nrf_error);
static void _adv_ble_adv_event_handler(ble_adv_evt_t ble_adv_evt);
static void _adv_ble_add_characteristics(struct adv_ble * blepn);
static struct adv_ble_attribute * _adv_ble_retrieve_attribute(
   enum adv_ble_attr * pa_char, struct adv_ble * blepn,
   ble_uuid_t const * ble_uuid);
static void _adv_ble_write_attr(struct adv_ble * blepn,
   const ble_gatts_evt_write_t * wr_evt);
static void _adv_ble_write_req(struct adv_ble * blepn,
   const ble_gatts_evt_write_t * wr_evt);
static void _adv_ble_read_req(struct adv_ble * blepn,
   const ble_gatts_evt_read_t * rd_evt);
static int _adv_ble_write_attribute(struct adv_ble * blepn,
   struct adv_ble_attribute * pa_attr, const ble_gatts_evt_write_t * wr_evt);
static void _adv_ble_complete_write_req(int retcode);
static void _adv_ble_complete_read_req(int retcode);
static struct adv_ble_attribute * _adv_ble_set_error_on_attr(
   struct adv_ble * blepn, int errno, enum adv_ble_attr pa_char);
static struct adv_ble_attribute * _adv_ble_get_attribute(
   struct adv_ble * blepn, enum adv_ble_attr pa_char);
static void _adv_ble_handle_disconnect(struct adv_ble * blepn);

static void _adv_ble_enter_sleep(struct adv_ble * blepn);
static void _adv_ble_timer_create(void);

static void _adv_ble_worker_start(void);
static void _adv_ble_worker_update_ble_status(bool active);
static void _adv_ble_worker_timer_cb(void * context);
static void _adv_ble_worker_feed(void);
static void _adv_ble_worker_run_next(void);

static void _adv_ble_mac_addr_to_str(char * str, size_t length,
   const ble_gap_addr_t * addr);
static void _adv_ble_disconnect(void);

//-----------------------------------------------------------------------------
// One-time initialized variables
//-----------------------------------------------------------------------------

/** HW version that gets filled in @ run time */
static char _adv_ble_hw_version[ARRAY_SIZE(HW_VERSION_TEMPLATE)];
static char _adv_ble_fw_version[ARRAY_SIZE(FW_VERSION_TEMPLATE)];

//-----------------------------------------------------------------------------
// Variable forward declarations
//-----------------------------------------------------------------------------

static ble_advdata_manuf_data_t _adv_ble_manuf_data;

//-----------------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------------

static char _adv_ble_sw_version[] = "_swver_" ADV_SW_VERSION;

static const struct adv_ble_link_version BLE_LL_VERSIONS[] = {
   [BLE_LINK_VERSION_IDX(6)] = { .lv_major = 0x4U, .lv_minor = 0x0U },
   [BLE_LINK_VERSION_IDX(7)] = { .lv_major = 0x4U, .lv_minor = 0x1U },
   [BLE_LINK_VERSION_IDX(8)] = { .lv_major = 0x4U, .lv_minor = 0x2U },
   [BLE_LINK_VERSION_IDX(9)] = { .lv_major = 0x5U, .lv_minor = 0x0U },
};

/**
 * Nordic SoftDevice versions.
 * @note exposing SD version to the public (through the fw_version atttribute)
 * is a potential security threat as it gives clues on FW weakness. However
 * for now, security is not part of the product specification.
 */
static const struct adv_ble_nordic_sd_version BLE_SD_VERSIONS[] = {
   { .sv_code = 0x008CU,
     .sv_version = NORDIC_SOFTDEVICE_VERSION(132U, 3U, 0U, 0U) },
   { .sv_code = 0x0098U,
     .sv_version = NORDIC_SOFTDEVICE_VERSION(132U, 4U, 0U, 2U) },
   { .sv_code = 0x0099U,
     .sv_version = NORDIC_SOFTDEVICE_VERSION(132U, 4U, 0U, 3U) },
   { .sv_code = 0x009DU,
     .sv_version = NORDIC_SOFTDEVICE_VERSION(132U, 5U, 0U, 0U) },
   { .sv_code = 0x009EU,
     .sv_version = NORDIC_SOFTDEVICE_VERSION(132U, 4U, 0U, 4U) },
   { .sv_code = 0x009FU,
     .sv_version = NORDIC_SOFTDEVICE_VERSION(132U, 4U, 0U, 5U) },
   { .sv_code = 0x00A5U,
     .sv_version = NORDIC_SOFTDEVICE_VERSION(132U, 5U, 1U, 0U) },
   { .sv_code = 0x00A8U,
     .sv_version = NORDIC_SOFTDEVICE_VERSION(132U, 6U, 0U, 0U) },
   { .sv_code = 0x00AFU,
     .sv_version = NORDIC_SOFTDEVICE_VERSION(132U, 6U, 1U, 0U) },
   { .sv_code = 0x00B7U,
     .sv_version = NORDIC_SOFTDEVICE_VERSION(132U, 6U, 1U, 1U) },
};

/** 128-bit UUID base List. */
static const ble_uuid128_t ADV_UUID128 = {
   .uuid128 = {
      // everything is reversed
      // 38d1xxxx-7b25-11e9-8f9e-2a86e4085a59
      0x59, 0x5a, 0x08, 0xe4, 0x86, 0x2a, 0x9e, 0x8f,
      0xe9, 0x11, 0x25, 0x7b, 0x00, 0x00, 0xd1, 0x38,
   }
};

// CCCD should always be writable by the client (SD fails otherwise)
// "The Client Characteristic Configuration declaration is an optional
//  characteristic descriptor that defines how the characteristic may
//  be configured by a specific client [...]
//  What is written in between these cryptic lines is that the CCCD is
//  a WRITABLE descriptor that allows the client, i.e. your MCP or
//  phone, to enable or disable notification or indication."
// Moreover it seems it should always been stored in SD stack, NOT un user
// space or sd_ble_gatts_characteristic_add would fail with INVALID_PARAM,

/** BLE attribute metadata for CCCD (readable/writable) */
static const ble_gatts_attr_md_t _ADV_RW_CCCD_ATTR_MD = {
   .read_perm = { \
      .sm = 1, .lv = 1, \
      }, \
   .write_perm = { \
      .sm = 1, .lv = 1, \
   }, \
   .vloc = BLE_GATTS_VLOC_STACK
};

#if 0
/** BLE attribute metadata (readable/writable) */
static const ble_gatts_attr_md_t _ADV_RW_ATTR_MD = {
   .read_perm = { \
      .sm = 1, .lv = 1, \
      }, \
   .write_perm = { \
      .sm = 1, .lv = 1, \
   }, \
   .vloc = BLE_GATTS_VLOC_USER,
   // we want to control write access, to bound values and/or control access
   // within known time frames.
   // This triggers BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST event on write access
   .wr_auth = 1,
};
#endif

/** BLE attribute metadata (read only) */
static const ble_gatts_attr_md_t _ADV_RO_ATTR_MD = {
   .read_perm = { \
      .sm = 1, .lv = 1, \
      }, \
   .write_perm = { \
      .sm = 0, .lv = 0, \
   }, \
   .vloc = BLE_GATTS_VLOC_USER
};

/** BLE attribute metadata (read only with on-request content generation) */
static const ble_gatts_attr_md_t _ADV_ROD_ATTR_MD = {
   .read_perm = { \
      .sm = 1, .lv = 1, \
      }, \
   .write_perm = { \
      .sm = 0, .lv = 0, \
   }, \
   .vloc = BLE_GATTS_VLOC_USER,
   // we want to control read access, to perform on-demand read out.
   // This triggers BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST event on read access
   .rd_auth = 1,
};

#if 0
/** BLE attribute metadata (readable with dynamic content/writable) */
static const ble_gatts_attr_md_t _ADV_RWD_ATTR_MD = {
   .read_perm = { \
      .sm = 1, .lv = 1, \
      }, \
   .write_perm = { \
      .sm = 1, .lv = 1, \
   }, \
   .vloc = BLE_GATTS_VLOC_USER,
   // we want to control write access and read access
   // This triggers BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST event on both read
   // and write access and on read access
   .wr_auth = 1,
   .rd_auth = 1,
};
#endif

static const ble_conn_params_init_t _CONN_PARAM_INIT = {
   .p_conn_params= NULL,
   // Time from initiating event (connect or start of notification) to
   // first time sd_ble_gap_conn_param_update is called.
   .first_conn_params_update_delay = APP_TIMER_TICKS(500),
   // Time between each call to sd_ble_gap_conn_param_update after the
   // first call. */
   .next_conn_params_update_delay = APP_TIMER_TICKS(60000),
   // Number of attempts before giving up the connection parameter
   // negotiation.
   .max_conn_params_update_count = 3,
   // initiate on connection, not on notification request
   .start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID,
   // do not disconnect if negociation with host fails
   .disconnect_on_fail = false,
   .evt_handler = &_adv_ble_conn_evt,
   .error_handler = &_adv_ble_conn_error_handler,
};

/** Connection parameters */
static const ble_gap_conn_params_t _GAP_CONN_PARAMS = {
   /** Minimum acceptable connection interval */
   .min_conn_interval = (uint16_t)MSEC_TO_UNITS(7.5, UNIT_1_25_MS),
   /** Maximum acceptable connection interval */
   .max_conn_interval =  MSEC_TO_UNITS(15, UNIT_1_25_MS),
   /** Slave latency. */
   .slave_latency = 3,
   /** Connection supervisory timeout. */
   .conn_sup_timeout  = MSEC_TO_UNITS(2000, UNIT_10_MS),
};

static const ble_dis_init_t _DIS_INIT = {
   .manufact_name_str = {
      .length = ZARRAY_SIZE(MANUFACTURER_NAME),
      .p_str  = (uint8_t *)MANUFACTURER_NAME,
   },
   .model_num_str = {
      .length = ZARRAY_SIZE(MODEL_NUMBER),
      .p_str  = (uint8_t *)MODEL_NUMBER,
   },
   .hw_rev_str = {
      .length = ZARRAY_SIZE(_adv_ble_hw_version),
      .p_str = (uint8_t *)_adv_ble_hw_version,
   },
   .fw_rev_str = {
      .length = ZARRAY_SIZE(_adv_ble_fw_version),
      .p_str = (uint8_t *)_adv_ble_fw_version,
   },
   .sw_rev_str = {
      .length = ZARRAY_SIZE(ADV_SW_VERSION),
      .p_str  = (uint8_t *)ADV_SW_VERSION,
   },
   .dis_char_rd_sec = SEC_OPEN,
};

/** UUIDs sent in advertisment packets */
static const ble_uuid_t _ADV_UUIDS[] = {
   {
      .uuid = BLE_UUID_DEVICE_INFORMATION_SERVICE,
      .type = BLE_UUID_TYPE_BLE,
   },
   {
      .uuid = ADV_SERVICE_UUID,
      .type = BLE_UUID_TYPE_VENDOR_BEGIN,
   },
};

/** BLE PowerAdvertiser advertisement configuration */
static const ble_advertising_init_t _ADV_BLE_ADVERTISE_INIT =
{
   .advdata = {
      .name_type = BLE_ADVDATA_FULL_NAME,
      .include_appearance = false,
      .flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE,
      .uuids_complete = {
         .uuid_cnt =  ARRAY_SIZE(_ADV_UUIDS),
         .p_uuids = (ble_uuid_t *)&_ADV_UUIDS[0],
      },
   },
   .srdata = {
      .p_manuf_specific_data = &_adv_ble_manuf_data,
   },
   .config = {
      .ble_adv_fast_enabled  = true,
      .ble_adv_fast_interval = MSEC_TO_UNITS(200, UNIT_0_625_MS),
      .ble_adv_fast_timeout  = ADV_BLE_SLEEP_DELAY_S,
      .ble_adv_slow_enabled  = true,
      .ble_adv_slow_interval = MSEC_TO_UNITS(1000, UNIT_0_625_MS),
      .ble_adv_slow_timeout  = 3600, // 1 hour
   },
   .evt_handler = _adv_ble_adv_event_handler,
   .error_handler = NULL,
};

//-----------------------------------------------------------------------------
// Variables
//-----------------------------------------------------------------------------

static struct adv_ble_var _adv_ble_var;

/** BLE PowerAdvertiser server engine */
static struct adv_ble _adv_ble = {
   .bp_attributes = {
      [ADV_ERROR] = {
         .pa_uuid = {
            .uuid = ADV_CHAR_UUID_BASE + ADV_ERROR,
         },
         .pa_attr_md = &_ADV_ROD_ATTR_MD,
         .pa_size = SIZEOF_MEMBER(struct adv_ble_var, pv_error),
         .pa_var = &_adv_ble_var.pv_error,
         .pa_props = BLE_CHAR_RN_PROP,
         .pa_desc = BLE_MAKE_ATTR_DESC("error"),
      },
   },
   .bp_sw_version = _adv_ble_sw_version,
};

/** Manufacturer data, used to advertise PowerAdvertiser status */
static struct adv_ble_adv_info _adv_ble_adv_info = {
   .ai_version = INFO_VERSION,
};

/**
 * Manufacturer data, used to advertise PowerAdvertiser status in scan response
 * packets
*/
static ble_advdata_manuf_data_t _adv_ble_manuf_data = {
   .company_identifier = MANUFACTURER_ID,
   .data = {
      .size = sizeof(_adv_ble_adv_info),
      .p_data = (uint8_t *)&_adv_ble_adv_info,
   },
};

// nRF52 SDK14 has found that messing around with linker (ordered) sections is
// a handy trick to register observers at build time. While this may sound
// clever, this is a recipe for disaster when public SDKs. Anyway, there are
// now no other choices than this crap.
NRF_SDH_BLE_OBSERVER(_adv_ble_observer, ADV_BLE_OBSERVER_PRIO,
                     &_adv_ble_evt_handler, &_adv_ble);
BLE_ADVERTISING_DEF(_adv_ble_advertising);

/** Background worker engine instance */
static struct adv_ble_worker_engine _adv_ble_worker_engine;

//-----------------------------------------------------------------------------
// Inline functions
//-----------------------------------------------------------------------------

/**
 * Retrieve the attribute characteristic from its address
 *
 * @param[in] pa_attr PN attribute to identify (@c NULL accepted)
 * @return pn characterisitic index, or @c ADV_COUNT on invalid attribute
 */
static enum adv_ble_attr
_adv_ble_attribute_char(const struct adv_ble_attribute * pa_attr)
{
   if ( ! pa_attr ||
        (pa_attr < &_adv_ble.bp_attributes[0]) ||
        (pa_attr >= &_adv_ble.bp_attributes[ADV_COUNT] )) {
      // invalid
      return ADV_COUNT;
   }

   return (enum adv_ble_attr)(pa_attr - &_adv_ble.bp_attributes[0]);
}

//-----------------------------------------------------------------------------
// Module API
//-----------------------------------------------------------------------------

/**
 * Initialize the BLE subsystem.
 */
void
adv_ble_init(void)
{
   _adv_ble_stack_init();
   _adv_ble_gap_init();
   _adv_ble_gatt_init();
   _adv_ble_dis_init();
   _adv_ble_service_add();
   _adv_ble_advertising_init();
   _adv_ble_timer_create();
   // if the next call is performed before GAP init, it fails miserabily
   // as the host receives a request with 0xffff values...
   _adv_ble_conn_init();
}

/**
 * Start up advertising
 */
void
adv_ble_start(void)
{
#if PEER_MANAGER_ENABLED
   if (erase_bonds == true) {
      delete_bonds();
      // Advertising is started by PM_EVT_PEERS_DELETED_SUCEEDED evetnt
   } else
#endif  // PEER_MANAGER_ENABLED
   {
      ret_code_t rc = ble_advertising_start(&_adv_ble_advertising,
                                            BLE_ADV_MODE_FAST);

      APP_ERROR_CHECK(rc);
   }

   _adv_ble_worker_start();
}

/**
 * Kludge for nRF52 v14 SDK ugly API
 *
 * @param[out] adv the advertising instance
 */
void
adv_ble_get_advertising(ble_advertising_t ** adv)
{
   if ( adv ) {
      *adv = &_adv_ble_advertising;
   }
}

//-----------------------------------------------------------------------------
// Private implementation
//-----------------------------------------------------------------------------

/**
 * Initialize the BLE stack, i.e. the SoftDevice and the BLE event interrupt.
 */
static void
_adv_ble_stack_init(void)
{
   ret_code_t rc;

   // Initialize the SoftDevice handler module.
   nrf_sdh_enable_request();

   // Fetch the start address of the application RAM.
   uint32_t ram_start = 0;
   rc = nrf_sdh_ble_app_ram_start_get(&ram_start);
   APP_ERROR_CHECK(rc);

   MSGV(PTL_INFO, "RAM start 0x%08x", ram_start);

   // Overwrite some of the default configurations for the BLE stack.
   const ble_cfg_t common_cfg = {
      .common_cfg = {
         // we need to add ADV PowerAdvertiser service
         // failing to reserve proper space here lead to a NO_MEM error
         // on sd_ble_uuid_vs_add next call.
         .vs_uuid_cfg = {
            .vs_uuid_count = 1,
         },
      },
   };
   rc = sd_ble_cfg_set(BLE_COMMON_CFG_VS_UUID, &common_cfg, ram_start);
   APP_ERROR_CHECK(rc);

   const ble_cfg_t attr_tab_cfg = {
      .gatts_cfg = {
         // we neeed to increase the attribute tab size
         // failing to reserve proper space here lead to a NO_MEM error
         // on sd_ble_gatts_characteristic_add when too many attributes are
         // added
         .attr_tab_size = {
            .attr_tab_size = 2U<<10U,
         },
      },
   };
   rc = sd_ble_cfg_set(BLE_GATTS_CFG_ATTR_TAB_SIZE, &attr_tab_cfg, ram_start);
   APP_ERROR_CHECK(rc);

   // Configure the maximum number of connections.
   const ble_cfg_t role_cfg = {
      .gap_cfg = {
         .role_count_cfg = {
            .periph_role_count = 1,
            .central_role_count = 0,
            .central_sec_count  = 0,
         },
      },
   };
   rc = sd_ble_cfg_set(BLE_GAP_CFG_ROLE_COUNT, &role_cfg, ram_start);
   APP_ERROR_CHECK(rc);

   // Enable BLE stack.
   rc = nrf_sdh_ble_enable(&ram_start);
   APP_ERROR_CHECK(rc);

   // Retrieve SoftDevice version
   // should be called after nrf_sdh_ble_enable()
   ble_version_t ble_sd_ver;
   memset(&ble_sd_ver, 0, sizeof(ble_sd_ver));
   rc = sd_ble_version_get(&ble_sd_ver);
   APP_ERROR_CHECK(rc);

   if ( NORDIC_COMPANY_ID == ble_sd_ver.company_id ) {
      unsigned int ver_ix = BLE_LINK_VERSION_IDX(ble_sd_ver.version_number);
      struct adv_ble_link_version ll_version;
      if ( ver_ix < ARRAY_SIZE(BLE_LL_VERSIONS) ) {
         ll_version = BLE_LL_VERSIONS[ver_ix];
      } else {
         ll_version.lv_major = 0U;
         ll_version.lv_minor = 0U;
         MSGV(PTL_ERROR, "Unsupported BLE link layer code %d",
              ble_sd_ver.version_number);
      }
      uint32_t sd_version = 0U;
      for (unsigned int sdix=0; sdix<ARRAY_SIZE(BLE_SD_VERSIONS); sdix++) {
         if ( BLE_SD_VERSIONS[sdix].sv_code == ble_sd_ver.subversion_number ) {
            sd_version = BLE_SD_VERSIONS[sdix].sv_version;
            break;
         }
      }
      if ( ! sd_version ) {
         MSGV(PTL_ERROR, "Unsupported SD code 0x%04x",
              ble_sd_ver.subversion_number)
      }

      snprintf(_adv_ble_fw_version, sizeof(_adv_ble_fw_version),
               FW_VERSION_FORMAT,
               ((sd_version>>16U) & 0xFFU),
               ((sd_version>>8U) & 0xFFU),
               ((sd_version>>0U) & 0xFFU),
               NORDIC_SD_OFFSET + ((sd_version>>24U) & 0xFFU),
               ll_version.lv_major, ll_version.lv_minor);
   } else {
      memset(_adv_ble_fw_version, 0, sizeof(_adv_ble_fw_version));
   }
}

/**
 * Initialize the Connection Parameters module.
 */
static void
_adv_ble_conn_init(void)
{
   // @ start up, the connection handle is invalid
   // unfortunately, BLE_CONN_HANDLE_INVALID is not zero, so be sure to
   // properly clear out the connection handle with invalid marker
   _adv_ble.bp_conn_handle = BLE_CONN_HANDLE_INVALID;

   ret_code_t rc;
   rc = ble_conn_params_init(&_CONN_PARAM_INIT);
   APP_ERROR_CHECK(rc);
}

/**
 * Function for handling the Connection Parameters Module.
 *
 * @param[in] ble_evt Event received from the Connection Parameters Module.
 */
static void
_adv_ble_conn_evt(ble_conn_params_evt_t * ble_evt)
{
   ret_code_t rc;

   if (ble_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
      rc = sd_ble_gap_disconnect(_adv_ble.bp_conn_handle,
                                 BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
      APP_ERROR_CHECK(rc);
   }
}

/**
 * Handle a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went
 *                       wrong.
 */
static void
_adv_ble_conn_error_handler(uint32_t nrf_error)
{
   MSGV(PTL_ERROR, "Connection error %u", nrf_error);
}

/**
 * GAP (Generic Access Profile) initialization.
 *
 * This function sets up all the necessary GAP parameters of the device
 * including the device name, appearance, and the preferred connection
 * parameters.
 */
void
_adv_ble_gap_init(void)
{
   ret_code_t rc;
   ble_gap_conn_sec_mode_t sec_mode;

   BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

   rc = sd_ble_gap_device_name_set(
         &sec_mode, (const uint8_t *)DEVICE_NAME_STR,
         (uint16_t)ZARRAY_SIZE(DEVICE_NAME_STR));
   APP_ERROR_CHECK(rc);

   rc = sd_ble_gap_appearance_set(BLE_APPEARANCE_UNKNOWN);
   APP_ERROR_CHECK(rc);

   rc = sd_ble_gap_ppcp_set(&_GAP_CONN_PARAMS);
   APP_ERROR_CHECK(rc);

   ble_gap_addr_t mac_addr;

   rc = sd_ble_gap_addr_get(&mac_addr);
   APP_ERROR_CHECK(rc);

   char addr[20];
   _adv_ble_mac_addr_to_str(addr, ARRAY_SIZE(addr), &mac_addr);

   MSGV(PTL_INFO, "BLE MAC: %s", addr);

   APP_ERROR_CHECK(rc);
}

/**
 * Function for initializing the GATT module.
 */
static void
_adv_ble_gatt_init(void)
{
   ret_code_t rc = nrf_ble_gatt_init(&_adv_ble.bp_gatt, NULL);
   APP_ERROR_CHECK(rc);
}

/**
 * Service addition.
 *
 * This function adds the service and the characteristic within it to the local
 * db.
 */
static void
_adv_ble_service_add(void)
{
   ret_code_t rc;

   ble_uuid_t service_uuid = {
      .uuid = ADV_SERVICE_UUID,
   };

   rc = sd_ble_uuid_vs_add(&ADV_UUID128, &service_uuid.type);
   APP_ERROR_CHECK(rc);

   rc = sd_ble_gatts_service_add(
      BLE_GATTS_SRVC_TYPE_PRIMARY, &service_uuid, &_adv_ble.bp_service_handle);
   APP_ERROR_CHECK(rc);

   // Add characteristics
   _adv_ble_add_characteristics(&_adv_ble);
}

/**
 * Device Information Service initialisation
 */
static void
_adv_ble_dis_init(void)
{
   int len;
   len = snprintf(_adv_ble_hw_version, ARRAY_SIZE(_adv_ble_hw_version),
                  HW_VERSION_FORMAT);
   if ( len >= (int)ARRAY_SIZE(_adv_ble_hw_version) ) {
      APP_ERROR_CHECK(NRF_ERROR_INTERNAL);
   }

   ret_code_t rc;
   rc = ble_dis_init(&_DIS_INIT);
   APP_ERROR_CHECK(rc);
}

/**
 * Advertising initialisation
 */
static void
_adv_ble_advertising_init(void)
{
   ret_code_t rc;

   rc = ble_advertising_init(&_adv_ble_advertising, &_ADV_BLE_ADVERTISE_INIT);
   if ( rc ) {
      MSGV(PTL_ERROR, "Cannot start advertising: 0x%x", rc);
   }
   APP_ERROR_CHECK(rc);

   rc = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV,
                                _adv_ble_advertising.adv_handle,
                                ADV_BLE_TX_POWER);
   APP_ERROR_CHECK(rc);
   MSGV(PTL_INFO, "BLE TX power %d dBm", ADV_BLE_TX_POWER);
}

/**
 * Function for handling advertising events.
 *
 * @param[in] ble_adv_evt advertising event.
 */
static void
_adv_ble_adv_event_handler(ble_adv_evt_t ble_adv_evt)
{
   switch (ble_adv_evt) {
      case BLE_ADV_EVT_IDLE:
         MSGV(PTL_INFO, "Would have sleep");
         ble_advertising_start(&_adv_ble_advertising, BLE_ADV_MODE_SLOW);
         break;
      case BLE_ADV_MODE_DIRECTED_HIGH_DUTY:
         MSGV(PTL_INFO, "Directed adv");
         break;
      case BLE_ADV_EVT_DIRECTED:
         MSGV(PTL_INFO, "Directed slow adv");
         break;
      case BLE_ADV_EVT_FAST:
         MSGV(PTL_INFO, "Advertising (fast)");
         break;
      case BLE_ADV_EVT_SLOW:
         MSGV(PTL_INFO, "Advertising (slow)");
         _adv_ble_enter_sleep(&_adv_ble);
         break;
      case BLE_ADV_EVT_FAST_WHITELIST:
         MSGV(PTL_INFO, "Whitelist adv");
         break;
      case BLE_ADV_EVT_SLOW_WHITELIST:
         MSGV(PTL_INFO, "Whitelist slow adv");
         break;
      case BLE_ADV_EVT_WHITELIST_REQUEST:
         MSGV(PTL_INFO, "Whitelist req");
         break;
      case BLE_ADV_EVT_PEER_ADDR_REQUEST:
         MSGV(PTL_INFO, "Peer req");
         break;
      default:
         MSGV(PTL_WARN, "Unknown ADV event: %d", ble_adv_evt);
         break;
   }
}

/**
 * Dispatch a BLE stack event to all modules with a BLE stack event handler.
 *
 * This function is called from the BLE Stack event interrupt handler after a
 * BLE stack event has been received.
 *
 * @param[in] ble_evt Bluetooth stack event.
 * @param[in,out] context struct adv_ble instance
 */
void
_adv_ble_evt_handler(const ble_evt_t * ble_evt, void * context)
{
   struct adv_ble * blepn = (struct adv_ble *)context;

   ret_code_t rc;

   if ( BLE_CONN_HANDLE_INVALID == blepn->bp_conn_handle ) {
      switch ( ble_evt->header.evt_id ) {
         case BLE_GAP_EVT_CONNECTED: // new connection
            break;
         case BLE_GAP_EVT_TIMEOUT: // may occur on advertising timeout
            return;
         case BLE_GAP_EVT_ADV_SET_TERMINATED: // adv buffer released
            return;
         case BLE_GAP_EVT_DISCONNECTED:
            MSGV(PTL_ERROR, "Disconnect");
            _adv_ble_handle_disconnect(blepn);
            return;
         case BLE_GATTS_EVT_HVN_TX_COMPLETE:
            MSGV(PTL_ERROR, "HVN TX complete on closed conn");
            return;
         default:
            MSGV(PTL_ERROR, "EVT 0x%x on closed conn", ble_evt->header.evt_id);
            return;
      }
   } else {
      if ( ble_evt->evt.gattc_evt.conn_handle != blepn->bp_conn_handle ) {
         MSGV(PTL_INFO, "Connection handle mismatch!");
         if ( ble_evt->evt.gattc_evt.conn_handle != BLE_CONN_HANDLE_INVALID ) {
            rc = sd_ble_gap_disconnect(ble_evt->evt.gattc_evt.conn_handle,
                                    BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);

            APP_ERROR_CHECK(rc);
         }
      }
   }

   switch (ble_evt->header.evt_id) {
      case BLE_GAP_EVT_CONNECTED: {
            // be sure to disable the background worker whenever a client
            // is connected
            blepn->bp_reboot = false;
            _adv_ble_worker_update_ble_status(true);
            char addr[20];
            _adv_ble_mac_addr_to_str(addr, ARRAY_SIZE(addr),
               &ble_evt->evt.gap_evt.params.connected.peer_addr);
            MSGV(PTL_INFO, "Connected from %s", addr);
            blepn->bp_conn_handle = ble_evt->evt.gap_evt.conn_handle;
            // restore advertisement configuration that may have been changed
            _adv_ble_advertising.adv_modes_config.ble_adv_fast_timeout =
               _ADV_BLE_ADVERTISE_INIT.config.ble_adv_fast_timeout;
         }
         break;  // BLE_GAP_EVT_CONNECTED

      case BLE_GAP_EVT_DISCONNECTED:
         MSGV(PTL_INFO, "Disconnected");
         blepn->bp_conn_handle = BLE_CONN_HANDLE_INVALID;
         _adv_ble_handle_disconnect(blepn);
         break;  // BLE_GAP_EVT_DISCONNECTED

      case BLE_GAP_EVT_TIMEOUT:
         MSGV(PTL_INFO, "GAP timeout");
         break;

      case BLE_GAP_EVT_CONN_PARAM_UPDATE:
         {
            const ble_gap_conn_params_t * const conn_params =
               &ble_evt->evt.gap_evt.params.conn_param_update.conn_params;
            MSGV(PTL_INFO, "Conn param min:%u max:%u lat:%u sup:%u",
                         conn_params->min_conn_interval,
                         conn_params->max_conn_interval,
                         conn_params->slave_latency,
                         conn_params->conn_sup_timeout);
         }
         break;

      case BLE_EVT_USER_MEM_REQUEST:
         MSGV(PTL_INFO, "User memory request");
         rc = sd_ble_user_mem_reply(ble_evt->evt.gattc_evt.conn_handle, NULL);
         APP_ERROR_CHECK(rc);
         break;  // BLE_EVT_USER_MEM_REQUEST

      case BLE_GATTC_EVT_TIMEOUT:
         // Disconnect on GATT Client timeout event.
         MSGV(PTL_INFO, "GATT Client Timeout");
         rc = sd_ble_gap_disconnect(ble_evt->evt.gattc_evt.conn_handle,
                                    BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
         APP_ERROR_CHECK(rc);
         break;  // BLE_GATTC_EVT_TIMEOUT

      case BLE_GATTS_EVT_SYS_ATTR_MISSING:
         MSGV(PTL_INFO, "Create default sys attr");
         rc = sd_ble_gatts_sys_attr_set(ble_evt->evt.gattc_evt.conn_handle,
                                        NULL, 0U, 0U);
         APP_ERROR_CHECK(rc);
         break;

      case BLE_GATTS_EVT_TIMEOUT:
         // Disconnect on GATT Server timeout event.
         MSGV(PTL_INFO, "GATT Server Timeout");
         rc = sd_ble_gap_disconnect(ble_evt->evt.gatts_evt.conn_handle,
                                    BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
         APP_ERROR_CHECK(rc);
         break;  // BLE_GATTS_EVT_TIMEOUT

      case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST: {
            const ble_gatts_evt_rw_authorize_request_t * req =
               &ble_evt->evt.gatts_evt.params.authorize_request;
            if ( req->type == BLE_GATTS_AUTHORIZE_TYPE_WRITE ) {
               _adv_ble_write_req(&_adv_ble, &req->request.write);
            } else if (req->type == BLE_GATTS_AUTHORIZE_TYPE_READ ) {
               _adv_ble_read_req(&_adv_ble, &req->request.read);
            } else {
               MSGV(PTL_ERROR, "Unexpected R/W request");
            }
         }
         break;

      case BLE_GATTS_EVT_WRITE:
         _adv_ble_worker_feed();
         _adv_ble_write_attr(&_adv_ble, &ble_evt->evt.gatts_evt.params.write);
         break;

      case BLE_GATTS_EVT_HVN_TX_COMPLETE:
         break;

      default:
         MSGV(PTL_INFO, "Event 0x%x on C:%04x",
              ble_evt->header.evt_id, ble_evt->evt.gattc_evt.conn_handle);
         break;
   }
}

/**
 * Add PowerAdvertiser characterstics
 *
 * @param[in,out] blepn BLE PowerAdvertiser engine
 */
static void
_adv_ble_add_characteristics(struct adv_ble * blepn)
{
   ret_code_t rc;

   ble_uuid128_t base_uuid = ADV_UUID128;
   for (unsigned int cix=0; cix<ADV_COUNT; cix++) {
      struct adv_ble_attribute * pa_attr = &blepn->bp_attributes[cix];
      #ifdef DEBUG
      if ( ! pa_attr->pa_size ) {
         // bp_attributes[] is likely not defined for current cix
         APP_ERROR_CHECK(NRF_ERROR_INTERNAL);
      }
      #endif // DEBUG
      rc = sd_ble_uuid_vs_add(&base_uuid, &pa_attr->pa_uuid.type);
      APP_ERROR_CHECK(rc);

      ble_gatts_attr_t attr = {
         .p_uuid = &pa_attr->pa_uuid,
         .p_attr_md = pa_attr->pa_attr_md,
         .init_len = (uint16_t)pa_attr->pa_size,
         .max_len = (uint16_t)pa_attr->pa_size,
         .p_value = pa_attr->pa_var,
      };

      ble_gatts_char_md_t char_md = {
         .char_props = pa_attr->pa_props,
         .p_char_user_desc = (uint8_t const * const) pa_attr->pa_desc.ad_str,
         .char_user_desc_max_size = (uint16_t)pa_attr->pa_desc.ad_size,
         .char_user_desc_size = (uint16_t)pa_attr->pa_desc.ad_size,
         // USERDESC can never be modified
         .p_user_desc_md = &_ADV_RO_ATTR_MD,
         // CCCD is modified by BLE client
         .p_cccd_md = &_ADV_RW_CCCD_ATTR_MD,
      };

      rc = sd_ble_gatts_characteristic_add(blepn->bp_service_handle, &char_md,
                                           &attr, &pa_attr->pa_handles);
      APP_ERROR_CHECK(rc);
   }
}

/**
 * Retrieve a PowerAdvertiser BLE attribute from its UUID
 *
 * @param[out] pa_char optional output value, updated with PowerAdvertiser attribute
 *             index if the attribute is found
 * @param[in,out] blepn BLE PowerAdvertiser engine
 * @param[in] ble_uuid BLE PowerAdvertiser UUID to seek
 * @return the PN attribute
 */
static struct adv_ble_attribute *
_adv_ble_retrieve_attribute(enum adv_ble_attr * pa_char,
                           struct adv_ble * blepn, ble_uuid_t const * ble_uuid)
{
   if ( (ble_uuid->uuid < ADV_CHAR_UUID_BASE) ||
        (ble_uuid->type != BLE_UUID_TYPE_VENDOR_BEGIN) ) {
      _adv_ble_set_error_on_attr(blepn, -PE_INVALID_UUID, ADV_ERROR);
      // we may receive events for all UUID, such as one for notification
      // registering
      MSGV(PTL_INFO, "Not a PN UUID: UUID:%04x Type:%02x",
                   ble_uuid->uuid, ble_uuid->type);
      return NULL;
   }

   uint16_t attr_ix =ble_uuid->uuid - ADV_CHAR_UUID_BASE;
   if ( attr_ix >= ADV_COUNT ) {
      _adv_ble_set_error_on_attr(blepn, -PE_INVALID_UUID, ADV_ERROR);
      MSGV(PTL_ERROR, "Invalid UUID: Attr:%d", attr_ix);
      return NULL;
   }

   struct adv_ble_attribute * pa_attr = &blepn->bp_attributes[attr_ix];

   if ( pa_char ) {
      *pa_char = (enum adv_ble_attr)attr_ix;
   }

   return pa_attr;
}

/**
 * Handle write-to-a-BLE-attribute event.
 * This function is barely used, as all PowerAdvertiser writable attributes are first
 * dispatched to #_adv_ble_write_req for verification, which also handles the
 * write request.
 * This function is therefore only used to detect unexpected/unmanaged write
 * events, or to non-PowerAdvertiser attributes
 *
 * @param[in,out] blepn BLE PowerAdvertiser engine
 * @param[in] wr_evt BLE write event
 */
static void
_adv_ble_write_attr(struct adv_ble * blepn,
                   const ble_gatts_evt_write_t * wr_evt)
{
   if ( BLE_UUID_TYPE_BLE == wr_evt->uuid.type ) {
      switch (wr_evt->uuid.uuid) {
         case BLE_UUID_DESCRIPTOR_CLIENT_CHAR_CONFIG:
            // client is registering notification/indication, let the BLE stack
            // handle this case
            return;
         default:
            MSGV(PTL_ERROR, "Write to std uuid 0x%04x", wr_evt->uuid.uuid);
            return;
      }
   }

   // only to get warnings for a unexpected write event
   (void)_adv_ble_retrieve_attribute(NULL, blepn, &wr_evt->uuid);
}

/**
 * Handle write-to-a-BLE-attribute-request event.
 * This function performs a per-attribute verification - to check value limits
 * for example or any other constraints. It notifies the caller about the
 * write permission, and performs the actual write action once the caller has
 * been notified.
 * Write non-authorisation is used to reject any unexpected value or command.
 *
 * @param[in,out] blepn BLE PowerAdvertiser engine
 * @param[in] wr_evt BLE write event
 */
static void
_adv_ble_write_req(struct adv_ble * blepn,
                  const ble_gatts_evt_write_t * wr_evt)
{
   ret_code_t errcode;

   switch ( wr_evt->op ) {
      case BLE_GATTS_OP_PREP_WRITE_REQ:
      case BLE_GATTS_OP_EXEC_WRITE_REQ_NOW:
      case BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL: {
            const ble_gatts_rw_authorize_reply_params_t auth_reply = {
               .type = BLE_GATTS_AUTHORIZE_TYPE_WRITE,
               .params.write = {
                  .gatt_status = APP_FEATURE_NOT_SUPPORTED,
                  .update = false,
                  .offset = 0,
                  .len = 0,
                  .p_data = NULL,
               },
            };
            MSGV(PTL_ERROR, "Unsupported write request: 0x%04x", wr_evt->op);
            errcode = sd_ble_gatts_rw_authorize_reply(blepn->bp_conn_handle,
                                                      &auth_reply);
            APP_ERROR_CHECK(errcode);
            return;
         }
      default:
         break;
   }

   int rc = -PE_INVALID_UUID;

   enum adv_ble_attr pa_char;
   struct adv_ble_attribute * pa_attr =
       _adv_ble_retrieve_attribute(&pa_char, blepn, &wr_evt->uuid);
   if ( ! pa_attr ) {
      MSGV(PTL_ERROR, "Write to unknown attribute rejected");
   }

   if ( blepn->bp_entering_sleep ) {
      MSGV(PTL_WARN, "PN is entering sleep, no request is accepted");
      pa_attr = NULL;
      rc = -PE_ABORT; // special marker, see below
   }

   if ( pa_attr ) {
      rc = _adv_ble_write_attribute(blepn, pa_attr, wr_evt);

      if ( PE_DEFERRED == rc ) {
         // the completion callback should take care of calling
         // sd_ble_gatts_rw_authorize_reply
         // note that BLE core spec limits execution time to 30 seconds max.
         MSGV(PTL_CHATTY, "Delayed completion for %s",
              pa_attr->pa_desc.ad_str);
         return;
      }
      if ( rc < 0 ) {
         MSGV(PTL_ERROR, "Write to %s failed to execute: %d",
                         pa_attr->pa_desc.ad_str, rc);
      }
   }

   _adv_ble_complete_write_req(rc);

   if ( ! rc && blepn->bp_reboot ) {
      // first trigger a disconnection
      // as the reboot flag has been set, the disconnection event handler
      // will resume with rebooting
      _adv_ble_disconnect();
   }
}

/**
 * Execute a remote request to update a local attribute.
 *
 * @param[in,out] blepn BLE PowerAdvertiser engine
 * @param[in] pa_attr PowerAdvertiser attribute
 * @param[in] wr_evt BLE write event
 * @return @c PE_NO_ERROR on success, a negative error code if value failed
 *         to pass the sanity check
 */
static int
_adv_ble_write_attribute(struct adv_ble * blepn,
                        struct adv_ble_attribute * pa_attr,
                        const ble_gatts_evt_write_t * wr_evt)
{
   if ( wr_evt->op != BLE_GATTS_OP_WRITE_REQ ) {
      MSGV(PTL_ERROR, "Not a write op");
      return -PE_INVALID_COMMAND;
   }
   if ( wr_evt->offset != 0 ) {
      MSGV(PTL_ERROR, "Non-zero offset");
      return -PE_NOT_SUPPORTED;
   }
   if ( pa_attr->pa_varsize ) {
      if ( wr_evt->len > pa_attr->pa_size ) {
         MSGV(PTL_ERROR, "Size overflow");
         return -PE_OVERFLOW;
      }
   } else {
      if ( wr_evt->len != pa_attr->pa_size ) {
         MSGV(PTL_ERROR, "Size mismatch");
         return -PE_INVALID_REQUEST;
      }
   }

   if ( ! pa_attr->pa_writer ) {
      MSGV(PTL_ERROR, "Write request w/ no executer");
      return -PE_READ_ONLY;
   }

   struct adv_ble_attr_event * event = &blepn->bp_attr_event;
   if ( event->ae_data || event->ae_attr ) {
      MSGV(PTL_ERROR, "Previous request never completed");
      // in case of deadlock, a disconnection should reset this
      return -PE_BUSY;
   }

   // valid request, reset the sleep timer
   _adv_ble_worker_feed();

   int rc;

   event->ae_data = wr_evt->data;
   event->ae_length = wr_evt->len;
   event->ae_offset = wr_evt->offset;
   event->ae_attr = pa_attr;

   if ( event->ae_length > sizeof(_adv_ble_var.pv_transient) ) {
      // There should be no reason for this error to occur, except if
      // the attribute storage space would exceed the transcient storage
      // container, which should not occur except if the size of the latter
      // has not been updated when the definition of the former has changed.
      // It is unfortunately difficult to detect at build time without
      // relying on ugly macros, so perform sanity check at run time, which
      // also protect against unexpected/undocumented behaviour of the BLE
      // stack.
      MSGV(PTL_ERROR, "Transcient storage invalid definition");
      rc = -PE_NOT_SUPPORTED;
      return rc;
   }

   rc = pa_attr->pa_writer(&wr_evt->data[0], (size_t)wr_evt->len);

   if ( PE_DEFERRED == rc ) {
      // if completion is deferred, the write event is whipped out from
      // memory (and its memory space re-used), so there should be no longer
      // references to it. Copy the data that needs to be preserved into the
      // special storage container.
      if ( event->ae_length > 0 ) {
         memcpy(_adv_ble_var.pv_transient, event->ae_data, event->ae_length);
         event->ae_data = &_adv_ble_var.pv_transient[0];
      } else {
         event->ae_data = NULL;
         event->ae_length = 0U;
      }
   }

   return rc;
}

/**
 * Complete a attribute request and reply to the peer
 *
 * @param[in] retcode the completion code of the write request
 */
static void
_adv_ble_complete_write_req(int retcode)
{
   struct adv_ble * blepn = &_adv_ble;
   struct adv_ble_attr_event * event = &blepn->bp_attr_event;

   if ( (! event->ae_data) || (! event->ae_attr) ) {
      // this may occur with a deferred completion that fails to update the
      // event
      MSGV(PTL_ERROR, "Nil event");
      retcode = -PE_INTERNAL;
   }

   uint16_t gatt_status;
   if ( retcode < 0 ) {
      if ( retcode != -PE_ABORT ) {
         enum adv_ble_attr pa_char = _adv_ble_attribute_char(event->ae_attr);
         _adv_ble_set_error_on_attr(blepn, retcode, pa_char);
         gatt_status = BLE_GATT_STATUS_ATTERR_WRITE_NOT_PERMITTED;
      } else {
         // there is no specific error code for this case, but this one seems
         // valid (BT core spec V4.2 Vol 3 Part F, section 3.3)
         gatt_status = BLE_GATT_STATUS_ATTERR_UNLIKELY_ERROR;
      }
   } else {
      gatt_status = BLE_GATT_STATUS_SUCCESS;
      event->ae_attr->pa_length = event->ae_length + event->ae_offset;
   };

   const ble_gatts_rw_authorize_reply_params_t auth_reply = {
      .type = BLE_GATTS_AUTHORIZE_TYPE_WRITE,
      .params.write = {
         .gatt_status = gatt_status,
         .update = PE_NO_ERROR == retcode,
         .offset = event->ae_offset,
         .len = event->ae_length,
         .p_data = event->ae_data,
      },
   };

   MSGV(PTL_DEBUG, "%s written, rc %d",
        event->ae_attr ? event->ae_attr->pa_desc.ad_str : "?", retcode);

   event->ae_data = NULL;
   event->ae_length = 0U;
   event->ae_offset = 0U;
   event->ae_attr = NULL;

   ret_code_t errcode;

   errcode = sd_ble_gatts_rw_authorize_reply(blepn->bp_conn_handle,
                                             &auth_reply);
   APP_ERROR_CHECK(errcode);
}

/**
 * Handle read-from-a-BLE-attribute-request event.
 * This function catches any call to an on-demand value request, so that
 * dynamic values can be generated right on time.
 *
 * @param[in,out]  blepn BLE PowerAdvertiser engine
 * @param[in] rd_evt BLE read event
 */
static void
_adv_ble_read_req(struct adv_ble * blepn,
                 const ble_gatts_evt_read_t * rd_evt)
{
   int rc = -PE_INTERNAL;

   enum adv_ble_attr pa_char;
   struct adv_ble_attribute * pa_attr =
       _adv_ble_retrieve_attribute(&pa_char, blepn, &rd_evt->uuid);
   if ( ! pa_attr ) {
      rc = -PE_INVALID_UUID;
      MSGV(PTL_ERROR, "Write to unknown attribute rejected");
   }
   if ( rd_evt->offset != 0 ) {
      MSGV(PTL_ERROR, "Non-zero offset");
      pa_attr = NULL;
      rc = -PE_NOT_SUPPORTED;
   }
   if ( blepn->bp_entering_sleep ) {
      MSGV(PTL_WARN, "PN is entering sleep, no request is accepted");
      pa_attr = NULL;
      rc = -PE_ABORT; // special marker, see below
   }

   struct adv_ble_attr_event * event = &blepn->bp_attr_event;
   if ( event->ae_data || event->ae_attr ) {
      MSGV(PTL_ERROR, "Previous request never completed");
      // in case of deadlock, a disconnection should reset this
      pa_attr = NULL;
      rc = -PE_BUSY;
   }

   if ( pa_attr ) {
      _adv_ble_worker_feed();

      event->ae_attr = pa_attr;

      // clear out the attribute length if it has a variable size...
      if ( pa_attr->pa_varsize ) {
            pa_attr->pa_length = 0;
      } else {
         pa_attr->pa_length = pa_attr->pa_size;
      }

      if ( pa_attr->pa_reader ) {
         event->ae_data = NULL;
         event->ae_length = 0U;
         event->ae_offset = 0U;

         MSGV(PTL_DEBUG, "Read from %s", pa_attr->pa_desc.ad_str);

         rc = pa_attr->pa_reader(pa_attr);

         if ( PE_DEFERRED == rc ) {
            // the completion callback should take care of calling
            // _adv_ble_complete_read_req
            // note that BLE core spec limits execution time to 30 seconds max.
            return;
         }

         if ( rc < 0 ) {
            MSGV(PTL_ERROR, "Read executer failed for %s",
                            pa_attr->pa_desc.ad_str);
         }
      } else {
         MSGV(PTL_CHATTY, "No read executer for %s", pa_attr->pa_desc.ad_str);

         rc = PE_NO_ERROR;
      }
   }

   _adv_ble_complete_read_req(rc);
}

/**
 * Complete a attribute request and reply to the peer
 *
 * @param[in] retcode the completion code of the write request
 */
static void
_adv_ble_complete_read_req(int retcode)
{
   struct adv_ble * blepn = &_adv_ble;
   struct adv_ble_attr_event * event = &blepn->bp_attr_event;

   if ( ! event->ae_attr ) {
      MSGV(PTL_ERROR, "Invalid event record: 0x%08x",
                      (uintptr_t)event->ae_attr);
      retcode = -PE_INTERNAL;
   }
   if ( ! retcode ) {
      if ( ! event->ae_attr->pa_length ) {
         // this may occur with variable size attribute whose reader function
         // failed to update the attribute length
         MSGV(PTL_ERROR, "Nil event");
         retcode = -PE_INTERNAL;
      }
   }

   ret_code_t errcode;
   if ( ! retcode ) {
      const ble_gatts_rw_authorize_reply_params_t reply_auth = {
         .type = BLE_GATTS_AUTHORIZE_TYPE_READ,
         .params.read = {
            .gatt_status = BLE_GATT_STATUS_SUCCESS,
            .update = true,
            .offset = 0U,
            .len = (uint16_t)event->ae_attr->pa_length,
            .p_data = event->ae_attr->pa_var,
         },
      };

      errcode = sd_ble_gatts_rw_authorize_reply(blepn->bp_conn_handle,
                                                &reply_auth);
   } else {
      MSGV(PTL_DEBUG, "Complete failed read: %d", retcode);
      uint16_t gatt_status;

      if ( retcode != -PE_ABORT ) {
         // _adv_ble_attribute_char accepts invalid pointers
         enum adv_ble_attr pa_char = _adv_ble_attribute_char(event->ae_attr);
         _adv_ble_set_error_on_attr(blepn, retcode, pa_char);
         gatt_status = BLE_GATT_STATUS_ATTERR_READ_NOT_PERMITTED;
      } else {
         // there is no specific error code for this case, but this one seems
         // valid (BT core spec V4.2 Vol 3 Part F, section 3.3)
         gatt_status = BLE_GATT_STATUS_ATTERR_UNLIKELY_ERROR;
      }

      const ble_gatts_rw_authorize_reply_params_t reply_auth = {
         .type = BLE_GATTS_AUTHORIZE_TYPE_READ,
         .params.read = {
            .gatt_status = gatt_status,
         },
      };

      errcode = sd_ble_gatts_rw_authorize_reply(blepn->bp_conn_handle,
                                                &reply_auth);
   }
   APP_ERROR_CHECK(errcode);

   event->ae_data = NULL;
   event->ae_length = 0U;
   event->ae_offset = 0U;
   event->ae_attr = NULL;
}

/**
 * Update the ADV_ERROR BLE attribute, to store error information for the client
 *
 * @param[in,out] blepn BLE PowerAdvertiser engine
 * @param[in] errno error code to store
 * @param[in] pa_char attribute access that triggered the error
 */
static struct adv_ble_attribute *
_adv_ble_set_error_on_attr(struct adv_ble * blepn,
                          int errno,
                          enum adv_ble_attr pa_char)
{
   const struct adv_ble_attribute * pa_attr;
   if ( (pa_char < ADV_FIRST) || (pa_char >= ADV_LAST) ) {
      // if the attribute is invalid, force the error attribute to flag the
      // error
      pa_char = ADV_ERROR;
   }
   pa_attr = _adv_ble_get_attribute(blepn, pa_char);
   MSGV(PTL_INFO, "Store error %d for attribute %u %s",
                  errno, pa_char, pa_attr ? pa_attr->pa_desc.ad_str : "?");
   struct adv_ble_attribute * pa_error = _adv_ble_get_attribute(blepn, ADV_ERROR);
   struct pa_error_desc * bpe = (struct pa_error_desc *)pa_error->pa_var;
   bpe->pe_errno = (int8_t)errno;
   bpe->pe_attr = (uint8_t)pa_char;
   return pa_error;
}

/**
 * Retrieve the BLE PowerAdvertiser attribute from its identifier
 *
 * @param[in,out] blepn BLE PowerAdvertiser engine
 * @param[in] pa_char attribute identifier
 * @return PowerAdvertiser attribute or @c NULL if not found
 */
static struct adv_ble_attribute *
_adv_ble_get_attribute(struct adv_ble * blepn,
                      enum adv_ble_attr pa_char)
{
   if ( (pa_char >= ADV_FIRST) && (pa_char < ADV_LAST) ) {
      return &blepn->bp_attributes[pa_char];
   }

   MSGV(PTL_FATAL, "Invalid attribute %u", pa_char);
   APP_ERROR_CHECK(NRF_FAULT_ID_SDK_ASSERT);

   return NULL;
}

/**
 * Handle disconnection
 *
 * @param[in,out] blepn BLE PowerAdvertiser engine
 */
static void
_adv_ble_handle_disconnect(struct adv_ble * blepn)
{
   // cleanup a potential deadlock case
   memset(&blepn->bp_attr_event, 0, sizeof(struct adv_ble_attr_event));
}

/**
 * Prepare the PN to enter sleep mode
 *
 * @param[in,out] blepn BLE PowerAdvertiser engine
 */
static void
_adv_ble_enter_sleep(struct adv_ble * blepn)
{
   // restore advertisement configuration that may have been changed
   _adv_ble_advertising.adv_modes_config.ble_adv_fast_timeout =
      _ADV_BLE_ADVERTISE_INIT.config.ble_adv_fast_timeout;

   blepn->bp_entering_sleep = true;
}

//-----------------------------------------------------------------------------
// Helpers
//-----------------------------------------------------------------------------

/**
 * Build up a readable string with a BLE MAC address from a LE byte address
 *
 * @param[out] str updated with the BLE MAC address string
 * @param[in] length length of output string in byte (should be 18 byte long
 *            or more)
 * @param[in] addr address to convert
 */
static void
_adv_ble_mac_addr_to_str(char * str, size_t length, const ble_gap_addr_t * addr)
{
   ssize_t rem = (ssize_t)length;
   for(unsigned int bix=0; bix<ARRAY_SIZE(addr->addr); bix++) {
      int len = snprintf(str, (size_t)MAX(0, rem),
                         (bix<(ARRAY_SIZE(addr->addr)-1)) ? "%02x:" : "%02x",
                         addr->addr[ARRAY_SIZE(addr->addr)-1-bix]);
      if ( len > rem ) {
         MSGV(PTL_ERROR, "Invalid output string");
         str[0] = '\0';
         return;
      }
      rem -= len;
      str += len;
   }
}

/**
 * Close the current BLE connection, if any.
 */
static void
_adv_ble_disconnect(void)
{
   if ( BLE_CONN_HANDLE_INVALID != _adv_ble.bp_conn_handle ) {
      ret_code_t rc;
      rc = sd_ble_gap_disconnect(_adv_ble.bp_conn_handle,
                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      if ( rc ) {
         MSGV(PTL_WARN, "Cannot disconnect: 0x%04x", rc);
      }
   }
}

/**
 * Create all timers
 */
static void
_adv_ble_timer_create(void)
{
   ret_code_t rc;

   _adv_ble_worker_engine.we_timer_id = &_adv_ble_worker_engine.we_timer;
   rc = app_timer_create(&_adv_ble_worker_engine.we_timer_id,
                         APP_TIMER_MODE_REPEATED,
                         &_adv_ble_worker_timer_cb);
   APP_ERROR_CHECK(rc);
}

//-----------------------------------------------------------------------------
// Background worker timer management
//-----------------------------------------------------------------------------

/**
 * Kick off the background worker engine timer.
 */
static void
_adv_ble_worker_start(void)
{
   ret_code_t rc;
   rc = app_timer_start(_adv_ble_worker_engine.we_timer_id,
                        APP_TIMER_TICKS(ADV_BLE_WORKER_PACE_S*1000),
                        &_adv_ble_worker_engine);
   APP_ERROR_CHECK(rc);

   // reset the relative timer clock
   _adv_ble_worker_engine.we_time = 0U;
   _adv_ble_worker_engine.we_last_time = 0U;
}

/**
 * Inform the worker engine about the BLE status.
 *
 * @param[in] active @c true to tell the engine a connection is active or may
 *                   be re-activated. Workers can only run when no connection
 *                   is active (and not forseen)
 */
static void
_adv_ble_worker_update_ble_status(bool active)
{
   struct adv_ble_worker_engine * we = &_adv_ble_worker_engine;
   MSGV(PTL_INFO, "BLE status: %u", active);
   if ( ! active ) {
      we->we_enable = true;
   } else  {
      // be sure no worker can be run while BLE connection is active
      we->we_enable = false;
   }
}

/**
 * Invoke the timer callback handler, from the application scheduler thread,
 * on timer exhaustion.
 *
 * @param[in,out] context worker engine
 */
static void
_adv_ble_worker_timer_cb(void * context)
{
   struct adv_ble_worker_engine * we = (struct adv_ble_worker_engine *) context;
   we->we_time += ADV_BLE_WORKER_PACE_S;
   if ( ! we->we_enable ) {
      // check if the BLE connection has not been used for a while
      if ( (we->we_time - we->we_last_time) > ADV_BLE_STALL_DELAY_S ) {
         if ( BLE_CONN_HANDLE_INVALID != _adv_ble.bp_conn_handle ) {
            // the timeframe of connection and worker state differ
            // * connection handle is invalidated as soon as the connection
            //   is closed, while
            // * worker enablement is deferred to the slow adverstisement
            //   step
            // meanwhile, the connection is closed but the PowerAdvertiser may
            // expect a reconnection from the (same) peer
            // therefore, the connection handle should be checked
            MSGV(PTL_WARN, "Stalled connection detected, closing");

            _adv_ble_disconnect();
         }
      }
      // execution of workers is disabled for now
      return;
   }
   if ( we->we_running ) {
      MSGV(PTL_WARN, "Worker still running");
      return;
   }
   // prevent any other schedule till all the workers have been run/tested
   we->we_running = true;
   // prepare for first worker
   we->we_worker_ix = 0;
   // kick off the worker execution
   _adv_ble_worker_run_next();
}

/**
 * Tell the worker engine's connection watchdog that some BLE request has been
 * received, to let it know that the connection is still active
 */
static void
_adv_ble_worker_feed(void)
{
   struct adv_ble_worker_engine * we = &_adv_ble_worker_engine;
   we->we_last_time = we->we_time;
}

/**
 * Execute next background worker task
 * This may either be called by the worker timer, or by the completion
 * callback of a background worker
 */
static void
_adv_ble_worker_run_next(void)
{
   struct adv_ble_worker_engine * we = &_adv_ble_worker_engine;

   // all workers have been tested or run, get ready for the next session
   we->we_running = false;
}
