/**
 * Error codes
 *
 * With the exception of #PE_DEFERRED, all errors codes are used as negative
 * values.
 * Througough the whole PowerAdvertiser implementation, the following convention is
 * used:
 *
 *  * 0 is a successful completion
 *  * > 0 is a deferred completion: a sequence of asynchronous I/O requests
 *    is required to complete the request, the caller is notified through a
 *    dedicated callback
 *  * < 0 is an actual error, the request has failed to complete and aborted.
 *    the actual error code is the negated value of this error code.
 *
 * Error codea are also exposed to the BLE client, through ADV_ERROR attribute.
 */

#ifndef _ADV_ERRORS_H_
#define _ADV_ERRORS_H_

/** @defgroup pa_errors @{ */

/** Supported error codes */
enum pa_error {
   PE_NO_ERROR,            /**< 0: No error, successful */
   PE_DEFERRED,            /**< 1: No error, deferred completion */
   PE_ABORT,               /**< 2: No error, early abort */
   PE_INTERNAL,            /**< 3: Internal error (fatal) */
   PE_IO_ERROR,            /**< 4: I/O or communication error */
   PE_NOT_READY,           /**< 5: Device not ready or not configured */
   PE_NOT_POWERED,         /**< 6: Device is not powered */
   PE_NOT_SUPPORTED,       /**< 7: Feature is not supported */
   PE_OVERFLOW,            /**< 8: Value cannot fit in */
   PE_INVALID_REQUEST,     /**< 9: Request is invalid or bad formatted */
   PE_INVALID_UUID,        /**< 10: BLE or 1-wire UUID is not known */
   PE_READ_ONLY,           /**< 11: BLE attribute cannot be modified */
   PE_OUT_OF_RANGE,        /**< 12: Value is not in valid range  */
   PE_INVALID_CHANNEL,     /**< 13: Invalid WiFi channel */
   PE_INVALID_DURATION,    /**< 14: Invalid duration or delay */
   PE_INVALID_ANTENNA,     /**< 15: Invalid antenna */
   PE_INVALID_POWER,       /**< 16: Invalid RF power */
   PE_INVALID_COMMAND,     /**< 17: Invalid command or argument */
   PE_INVALID_SIZE,        /**< 18: Invalid size */
   PE_BUSY,                /**< 19: Device is already running */
   PE_NO_DEVICE,           /**< 20: Device is not known */
   PE_NO_SLAVE_DEVICE,     /**< 21: 1-wire slave device is not known */
   PE_PROTECTED,           /**< 22: Safety/energy condition not met */
   PE_UNKNOWN,             /**< 23: Unknown error */
   PE_COUNT,               /**< 24: Watermark */
};

/** @} */

#endif // _ADV_ERRORS_H_

