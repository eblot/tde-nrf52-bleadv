/**
 * PowerAdvertiser Bluetooth management
 *
 * @file adv_ble.h
 */

#ifndef _ADV_BLE_H
#define _ADV_BLE_H

#include <stdint.h>
#include <stdbool.h>
#include "nrf_warn_enter.h"
#include "ble.h"
#include "ble_advertising.h"
#include "nrf_warn_leave.h"
#include "bleadv_gitbldver.h"  // automatically generated in build directory

#ifndef ADV_SW_VERSION
#error ADV_SW_VERSION is not defined
#endif // SW_VERSION
#ifdef DEBUG
#define SW_BUILD "D"
#else
#define SW_BUILD "R"
#endif
#define SW_SVN_VERSION ADV_SW_VERSION "." POWERADV_SVNVER "-" SW_BUILD

/** Proprietary BLE UUID for advertiser services */
#define ADV_SERVICE_UUID      0x0071U

void adv_ble_init(void);
void adv_ble_start(void);
void adv_ble_evt_dispatch(ble_evt_t * ble_evt);
void adv_ble_debug_evt(bool enable);
void adv_ble_get_advertising(ble_advertising_t ** adv);

#endif // _ADV_BLE_H
