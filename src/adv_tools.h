/**
 * Tools and miscelleanous helpers
 *
 * * @defgroup pa_tools @{
 */

#ifndef _ADV_TOOLS_H_
#define _ADV_TOOLS_H_

#include <stdint.h>

//-----------------------------------------------------------------------------
// Macros
//-----------------------------------------------------------------------------

#ifndef ASSERT_COMPILE
/**
 * Compile-time assertion.
 * Aborts the compilation if the c' constant expression expression evaluates
 * to false
 */
#define _JOIN_(_x_,_y_) _DO_JOIN_(_x_,_y_)
#define _DO_JOIN_(_x_,_y_) _x_##_y_
#define ASSERT_COMPILE(_c_) \
  void _JOIN_(assert_compile, __LINE__)(int assert_compile[(_c_)?1:-1])
#endif // ASSERT_COMPILE

#ifndef ERROR_WITH_CONSTANT_INTEGER
#define ERROR_WITH_CONSTANT_INTEGER(_x_) const int _error_array[0] = {[_x_]= 0}
#endif // ERROR_WITH_CONSTANT_INTEGER

/** Compute the size of an array */
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(_a_) (sizeof(_a_)/sizeof(_a_[0]))
#endif // ARRAY_SIZE

/** Compute the length of a zero-terminated string */
#define ZARRAY_SIZE(_a_) (ARRAY_SIZE(_a_)-1)

#ifndef _STRUCT_MEMBER
/** Structure member of a type */
#define _STRUCT_MEMBER(_type_, _mbr_) (((_type_ *)(NULL))->_mbr_)
#endif // SIZEOF

#ifndef STRUCT_TYPEOF
/** Type of a structure member */
#define STRUCT_TYPEOF(_type_, _mbr_) typeof(_STRUCT_MEMBER(_type_, _mbr_))
#endif // SIZEOF

#ifndef SIZEOF_MEMBER
/** Compute the size of a structure member */
#define SIZEOF_MEMBER(_type_, _mbr_) sizeof(_STRUCT_MEMBER(_type_, _mbr_))
#endif // SIZEOF_MEMBER

#ifndef ARRAY_SIZEOF_MEMBER
/** Compute the size of a structure member */
#define ARRAY_SIZEOF_MEMBER(_type_, _mbr_) \
   (sizeof(((_type_ *)(NULL))->_mbr_)/sizeof(((_type_ *)(NULL))->_mbr_[0]))
#endif // ARRAY_SIZEOF_MEMBER

/** Compute the minimum value */
#ifndef MIN
#define MIN(_a_, _b_) ((_a_)<(_b_) ? (_a_):(_b_))
#endif // MIN

/** Compute the maximum value */
#ifndef MAX
#define MAX(_a_, _b_) ((_a_)>(_b_) ? (_a_):(_b_))
#endif // MAX

/** Compute the absolute value */
#define ABS(_a_) ((_a_)>0 ? (_a_):(-(_a_)))

/** Compute the always negative value */
#define NABS(_a_) ((_a_)<0 ? (_a_):(-(_a_)))

/** stringification macro, step 1 */
#define _XSTR_(s) _STR_(s)
/** stringification macro, step 2 */
#define _STR_(s) #s

#ifdef APP_NAME
/** SVN external reference step 3 */
# define __SVN_REFERENCE(_app_,_type_) svn_##_app_##_build_##_type_
/** SVN external reference step 3 */
# define _SVN_REFERENCE(_app_,_type_) __SVN_REFERENCE(_app_, _type_)
/** SVN external reference step 1 */
# define SVN_REFERENCE(_type_) _SVN_REFERENCE(APP_NAME, _type_)
#endif // APP_NAME

#ifdef __GNUC__
/** Tell the compiler a function argument is not used */
#define _unused __attribute__((__unused__))
#else
#define _unused
#endif //

//-----------------------------------------------------------------------------
// Bytes and bits inline function helpers
//-----------------------------------------------------------------------------

/** Force alignement on a 16-bit boundary */
#define ALIGN_UINT16 __attribute__ ((aligned(sizeof(uint16_t))))
/** Force alignement on a 32-bit boundary */
#define ALIGN_UINT32 __attribute__ ((aligned(sizeof(uint32_t))))
/** Force alignement on a 64-bit boundary */
#define ALIGN_UINT64 __attribute__ ((aligned(sizeof(uint64_t))))

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-function"

/**
 * Read a 8-bit integer from a byte stream, using host (little) endianess
 *
 * @param[out] value updated with the 8-bit integer value
 * @param[in] buf the input byte stream
 */
static inline void
get_uint8(uint8_t * value, const uint8_t * buf)
{
   *value = ((uint8_t)buf[0]);
}

/**
 * Read a 16-bit integer from a byte stream, using host (little) endianess
 *
 * @param[out] value updated with the 16-bit integer value
 * @param[in] buf the input byte stream
 */
static inline void
get_uint16(uint16_t * value, const uint8_t * buf)
{
   *value = (uint16_t)(((unsigned int)buf[0]) | (((unsigned int)buf[1]) << 8U));
}

/**
 * Read a 32-bit integer from a byte stream, using host (little) endianess
 *
 * @param[out] value updated with the 32-bit integer value
 * @param[in] buf the input byte stream
 */
static inline void
get_uint32(uint32_t * value, const uint8_t * buf)
{
   *value =
      (uint32_t)(((unsigned int)buf[0]) | (((unsigned int)buf[1]) << 8U) |
                 (((uint32_t)buf[2]) << 16U) | (((uint32_t)buf[3]) << 24U));
}

/**
 * Read a 64-bit integer from a byte stream, using host (little) endianess
 *
 * @param[out] value updated with the 64-bit integer value
 * @param[in] buf the input byte stream
 */
static inline void
get_uint64(uint64_t * value, const uint8_t * buf)
{
   uint32_t low, high;
   get_uint32(&low, buf);
   get_uint32(&high, buf+sizeof(uint32_t));
   *value = ((uint64_t)low) | (((uint64_t)high) << 32U);
}

/**
 * Append a 8-bit integer to a byte stream, using host (little) endianess
 *
 * @param[out] buf the byte stream to receive the integer value
 * @param[in] value the 8-bit integer value to insert
 */
static inline void
set_uint8(uint8_t * buf, uint8_t value)
{
   buf[0] = value;
}

/**
 * Append a 16-bit integer to a byte stream, using host (little) endianess
 *
 * @param[out] buf the byte stream to receive the integer value
 * @param[in] value the 16-bit integer value to insert
 */
static inline void
set_uint16(uint8_t * buf, uint16_t value)
{
   buf[0] = (uint8_t)value;
   buf[1] = (uint8_t)(value >> 8);
}

/**
 * Append a 32-bit integer to a byte stream, using host (little) endianess
 *
 * @param[out] buf the byte stream to receive the integer value
 * @param[in] value the 32-bit integer value to insert
 */
static inline void
set_uint32(uint8_t * buf, uint32_t value)
{
   buf[0] = (uint8_t)value;
   buf[1] = (uint8_t)(value >> 8U);
   buf[2] = (uint8_t)(value >> 16U);
   buf[3] = (uint8_t)(value >> 24U);
}

/**
 * Append a 64-bit integer to a byte stream, using host (little) endianess
 *
 * @param[out] buf the byte stream to receive the integer value
 * @param[in] value the 64-bit integer value to insert
 */
static inline void
set_uint64(uint8_t * buf, uint64_t value)
{
   set_uint32(buf, (uint32_t)value);
   set_uint32(buf+sizeof(uint32_t), (uint32_t)(value >> 32U));
}

#pragma clang diagnostic pop

/** @} */

#endif // _ADV_TOOLS_H_

