/* Copyright 2013-2015 The MathWorks, inc. */
#ifndef _LIBMWMORPHOP_OCV_HPP_
#define _LIBMWMORPHOP_OCV_HPP_

#ifndef LIBMWMORPHOP_OCV_API
#    define LIBMWMORPHOP_OCV_API
#endif

#ifndef EXTERN_C
#  ifdef __cplusplus
#    define EXTERN_C extern "C"
#  else
#    define EXTERN_C extern
#  endif
#endif

#ifdef MATLAB_MEX_FILE
#include "tmwtypes.h"
#else
#include "rtwtypes.h"
#endif

EXTERN_C LIBMWMORPHOP_OCV_API
void dilate_boolean_ocv(const uint8_T* const src, const real64_T* const inDims,
                      const boolean_T* nhood, const real64_T* const nhoodDims,
                      uint8_T* dst);

EXTERN_C LIBMWMORPHOP_OCV_API
void erode_boolean_ocv(const uint8_T* const src, const real64_T* const inDims,
                      const boolean_T* nhood, const real64_T* const nhoodDims,
                      uint8_T* dst);


EXTERN_C LIBMWMORPHOP_OCV_API
void dilate_uint8_ocv(const uint8_T* const src, const real64_T* const inDims,
                      const boolean_T* nhood, const real64_T* const nhoodDims,
                      uint8_T* dst);

EXTERN_C LIBMWMORPHOP_OCV_API
void erode_uint8_ocv(const uint8_T* const src, const real64_T* const inDims,
                      const boolean_T* nhood, const real64_T* const nhoodDims,
                      uint8_T* dst);

EXTERN_C LIBMWMORPHOP_OCV_API
void dilate_uint16_ocv(const uint16_T* const src, const real64_T* const inDims,
                      const boolean_T* nhood, const real64_T* const nhoodDims,
                      uint16_T* dst);

EXTERN_C LIBMWMORPHOP_OCV_API
void erode_uint16_ocv(const uint16_T* const src, const real64_T* const inDims,
                      const boolean_T* nhood, const real64_T* const nhoodDims,
                      uint16_T* dst);

EXTERN_C LIBMWMORPHOP_OCV_API
void dilate_int16_ocv(const int16_T* const src, const real64_T* const inDims,
                      const boolean_T* nhood, const real64_T* const nhoodDims,
                      int16_T* dst);

EXTERN_C LIBMWMORPHOP_OCV_API
void erode_int16_ocv(const int16_T* const src, const real64_T* const inDims,
                      const boolean_T* nhood, const real64_T* const nhoodDims,
                      int16_T* dst);
EXTERN_C LIBMWMORPHOP_OCV_API
void dilate_real32_ocv(const real32_T* const src, const real64_T* const inDims,
                      const boolean_T* nhood, const real64_T* const nhoodDims,
                      real32_T* dst);

EXTERN_C LIBMWMORPHOP_OCV_API
void erode_real32_ocv(const real32_T* const src, const real64_T* const inDims,
                      const boolean_T* nhood, const real64_T* const nhoodDims,
                      real32_T* dst);

EXTERN_C LIBMWMORPHOP_OCV_API
void dilate_real64_ocv(const real64_T* const src, const real64_T* const inDims,
                      const boolean_T* nhood, const real64_T* const nhoodDims,
                      real64_T* dst);

EXTERN_C LIBMWMORPHOP_OCV_API
void erode_real64_ocv(const real64_T* const src, const real64_T* const inDims,
                      const boolean_T* nhood, const real64_T* const nhoodDims,
                      real64_T* dst);


#endif
