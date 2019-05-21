/**
 * Available trace sources

 * */

#ifndef _ADV_TRACESRCS_H_
#define _ADV_TRACESRCS_H_

/**
 * Source components for the trace debugging helpers.
 */
enum pa_trace_modules
{
    PTM_SRC_00,   /**< Trace source #0 */
    PTM_SRC_01,   /**< Trace source #1 */
    PTM_SRC_02,   /**< Trace source #2 */
    PTM_SRC_03,   /**< Trace source #3 */
    PTM_SRC_04,   /**< Trace source #4 */
    PTM_SRC_05,   /**< Trace source #5 */
    PTM_SRC_06,   /**< Trace source #6 */
    PTM_SRC_07,   /**< Trace source #7 */
    PTM_SRC_08,   /**< Trace source #8 */
    PTM_SRC_09,   /**< Trace source #9 */
    PTM_SRC_10,   /**< Trace source #10 */
    PTM_SRC_11,   /**< Trace source #11 */
    PTM_SRC_12,   /**< Trace source #12 */
    PTM_SRC_13,   /**< Trace source #13 */
    PTM_SRC_14,   /**< Trace source #14 */
    PTM_SRC_15,   /**< Trace source #15 */
    PTM_SRC_16,   /**< Trace source #16 */
    PTM_SRC_17,   /**< Trace source #17 */
    PTM_SRC_18,   /**< Trace source #18 */
    PTM_SRC_19,   /**< Trace source #19 */
    PTM_SRC_20,   /**< Trace source #20 */
    PTM_SRC_21,   /**< Trace source #21 */
    PTM_SRC_22,   /**< Trace source #22 */
    PTM_SRC_23,   /**< Trace source #23 */
    PTM_SRC_24,   /**< Trace source #24 */
    PTM_SRC_25,   /**< Trace source #25 */
    PTM_SRC_26,   /**< Trace source #26 */
    PTM_SRC_27,   /**< Trace source #27 */
    PTM_SRC_28,   /**< Trace source #28 */
    PTM_SRC_29,   /**< Trace source #29 */
    PTM_SRC_30,   /**< Trace source #30 */
    PTM_SRC_31,   /**< Trace source #31 */
    PTM_COUNT
};

#define PTM_MAIN  PTM_SRC_00
#define PTM_SYS   PTM_SRC_01
#define PTM_BLE   PTM_SRC_02
#define PTM_LAST  PTM_SRC_03
/** @} */

#endif // _ADV_TRACESRCS_H_
