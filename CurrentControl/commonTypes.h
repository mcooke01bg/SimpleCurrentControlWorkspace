#ifndef __COMMON_TYPES_H__
#define __COMMON_TYPES_H__
/*==================================================================================================
COPYRIGHT 2026 MATIMAR LTD. ALL RIGHTS RESERVED

This software is protected by copyright and intellectual property laws in Bulgaria and other 
countries. The software may be licensed and is not sold to you. You may use the software only 
in accordance with the terms of the license agreement you entered into with Matimar Ltd.
===================================================================================================*/

/*==================================================================================================
  FILE DESCRIPTION

  Contains common type definitions and macro definitions.
===================================================================================================*/

/*==================================================================================================
 DEPENDENCIES
===================================================================================================*/
#include <stdint.h>

/*==================================================================================================
  TYPE DEFINITIONS
===================================================================================================*/
#ifndef _UINT64_DECLARED
typedef uint64_t uint64;
#define	_UINT64_DECLARED
#endif

#ifndef _UINT32_DECLARED
typedef uint32_t uint32;
#define	_UINT32_DECLARED
#endif

#ifndef _UINT16_DECLARED
typedef uint16_t uint16;
#define	_UINT16_DECLARED
#endif

#ifndef _UINT8_DECLARED
typedef uint8_t uint8;
#define	_UINT8_DECLARED
#endif

#ifndef _BOOLEAN_DECLARED
#ifdef __cplusplus
typedef bool boolean;
#else
typedef _Bool boolean;
#endif
#define	_BOOLEAN_DECLARED
#endif

#ifndef _SINT64_DECLARED
typedef int64_t sint64;
#define	_SINT64_DECLARED
#endif

#ifndef _SINT32_DECLARED
typedef int32_t sint32;
#define	_SINT32_DECLARED
#endif

#ifndef _SINT16_DECLARED
typedef int16_t sint16;
#define	_SINT16_DECLARED
#endif

#ifndef _SINT8_DECLARED
typedef int8_t sint8;
#define	_SINT8_DECLARED
#endif

#ifndef _FLOAT32_DECLARED
typedef float float32;
#define	_FLOAT32_DECLARED
#endif

#ifndef _FLOAT64_DECLARED
typedef double float64;
#define	_FLOAT64_DECLARED
#endif

typedef enum
{
    boolFALSE = 0, boolTRUE = 1
} bool_t;

/*==================================================================================================
  MACRO DEFINITIONS
/*==================================================================================================*/
#ifndef NULL
#define NULL ((void *) 0U)
#endif

#ifndef NULL_PTR
#define NULL_PTR ((void *)0x0)
#endif

#endif /* __COMMON_TYPES_H__ */
