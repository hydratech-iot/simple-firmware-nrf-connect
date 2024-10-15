/*
 * File Name: base_type.h
 *
 * Author: Thuan Le
 *
 * Description:
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#pragma once

/* Includes ----------------------------------------------------------- */
#include "base_include.h"

/* Public defines ----------------------------------------------------- */
/**
 * @brief Macro to check the status of an expression.
 *
 * This macro evaluates the given expression and checks.
 * If the status is not `BS_OK`, just return the error code. Otherwise, return BS_OK
 *
 * @param expr The expression to be evaluated.
 * @return The status of the expression.
 */
#define CHECK_STATUS(expr)                             \
    do                                                 \
    {                                                  \
        base_status_t ret = (expr);                    \
        if (BS_OK != ret)                              \
        {                                              \
            assert_failed((char *)__FILE__, __LINE__); \
            return (ret);                              \
        }                                              \
    } while (0)                                        \

/* Public enumerate/structure ----------------------------------------- */

/// @brief Error Status enumeration for function return values
typedef enum
{
    BS_OK = 0x00,     ///< No Error - Okay
    BS_ERROR = 0x01,  ///< Generic Error
    BS_BUSY = 0x02,   ///< Unable to perform the function at this time (busy)
    BS_TIMEOUT = 0x03 ///< Blocking function timed out
} base_status_t;

/**
 * @brief Enumeration representing boolean values.
 */
typedef enum
{
    BS_FALSE = 0x00, /**< Boolean false value. */
    BS_TRUE = 0x01   /**< Boolean true value. */
} bool_t;

/**
 * @brief Structure representing a byte with high and low nibbles.
 */
typedef struct
{
    uint8_t High : 4; /**< High nibble. */
    uint8_t Low : 4;  /**< Low nibble. */
} byte_t;

/* Public macros ------------------------------------------------------ */
#if !defined(NOT)
#define NOT(x) ((x) ? BS_FALSE : BS_TRUE)
#else
#error "NOT operator has been defined"
#endif

#if !defined(MAX)
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

#if !defined(MIN)
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

// Structure padding
#define _addstr1(a, b, c) a##b##c
#define _addstr2(a, b, c) _addstr1(a, b, c)

// Preprocessor assertion
// Example: example.h
//   typedef uint32_t time_s_t;
//   PRE_ASSERT(sizeof(time_s_t) == 4, time_s_t);
#define PRE_ASSERT(exp, msg) typedef char _addstr2(assert_, __LINE__, _##msg)[(exp) ? 1 : -1]

#define assert_param(expr) ((expr) ? (void)0 : assert_failed((char *)__FILE__, __LINE__))

#ifndef ARRAY_NUM_ELEMENTS
/// @brief Returns the number of elements in an array x
/// @param[in] x array variable.
#define ARRAY_NUM_ELEMENTS(x) (sizeof(x) / sizeof(x[0]))
#endif // ARRAY_NUM_ELEMENTS

#define SHIFT_VALUE_32 (32U)
#define SHIFT_VALUE_16 (16U)
#define SHIFT_VALUE_8 (8U)

/**
 * @brief Print the hexadecimal content as an error message.
 */
#define PRINT_HEX_ERR(p_label, p_text, len)\
	({\
		LOG_ERR("---- %s (len: %u): ----", p_label, len);\
		LOG_HEXDUMP_ERR(p_text, len, "Content:");\
		LOG_ERR("---- %s end  ----", p_label);\
	})

/**
 * @brief Print the hexadecimal content as a warning message.
 */
#define PRINT_HEX_WRN(p_label, p_text, len)\
	({\
		LOG_WRN("---- %s (len: %u): ----", p_label, len);\
		LOG_HEXDUMP_WRN(p_text, len, "Content:");\
		LOG_WRN("---- %s end  ----", p_label);\
	})

/**
 * @brief Print the hexadecimal content as an informational message.
 */
#define PRINT_HEX_INF(p_label, p_text, len)\
	({\
		LOG_INF("---- %s (len: %u): ----", p_label, len);\
		LOG_HEXDUMP_INF(p_text, len, "Content:");\
		LOG_INF("---- %s end  ----", p_label);\
	})

/**
 * @brief Print the hexadecimal content as a debug message.
 */
#define PRINT_HEX_DBG(p_label, p_text, len)\
	({\
		LOG_INF("---- %s (len: %u): ----", p_label, len);\
		LOG_HEXDUMP_DBG(p_text, len, "Content:");\
		LOG_INF("---- %s end  ----", p_label);\
	})


#define SYSTEM_DELAY_US(us)\
    ({\
        k_sleep(K_USEC(us));\
    }) 

#define SYSTEM_DELAY_MSEC(ms)\
    ({\
        k_sleep(K_MSEC(ms));\
    }) 

#define SYSTEM_DELAY_SEC(second)\
    ({\
        k_sleep(K_SECONDS(second));\
    }) 

/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
void assert_failed(char *file, uint32_t line);

/* End of file -------------------------------------------------------- */