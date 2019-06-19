/* Copyright 2011-2017 The MathWorks, Inc. */

#ifndef __XILUTILS_SL_H__
#define __XILUTILS_SL_H__

#ifndef EXTERN_C
    #ifdef __cplusplus
       
        #define EXTERN_C extern "C"
    #else
       
        #define EXTERN_C extern
    #endif
#endif

#ifndef LIBMWXILUTILS_SL_API
   
    #define LIBMWXILUTILS_SL_API
   
    #include "simstruc.h"
#else
    #ifndef MATLAB_MEX_FILE
        #define MATLAB_MEX_FILE
    #endif
   
    #include "simstruct/simstruc.h"
#endif

typedef enum  {
   COLUMN_TO_ROW_TRANSPOSE = 0,
   ROW_TO_COLUMN_TRANSPOSE
} XIL_TRANSPOSE_DIRECTION_T;

typedef enum {
   VARSIZE_GET_INPUT = 0,
   VARSIZE_GET_OUTPUT,
   VARSIZE_SET_OUTPUT
} XIL_VARSIZE_DIRECTION_T;

typedef enum {
   VARSIZE_TYPE_8BIT = 0,
   VARSIZE_TYPE_16BIT,
   VARSIZE_TYPE_32BIT
} XIL_VARSIZE_TYPE_T;

EXTERN_C LIBMWXILUTILS_SL_API int xilSimulinkUtilsCreate(void** const ppXILUtils,
                                                         SimStruct* const S);

EXTERN_C LIBMWXILUTILS_SL_API void xilSimulinkUtilsStaticErrorHandle(const uint8_T nrhs,
                                                                     mxArray* prhs[],
                                                                     SimStruct* const S);

EXTERN_C LIBMWXILUTILS_SL_API void* xilSimulinkUtilsCreateBuiltInSignal (
                                                      void* pXILUtils,
                                                      const char* signalName,
                                                      const char* propagateName,
                                                      int dimension,
                                                      int* dimensions,
                                                      int portIdx,
                                                      bool isComplex,
                                                      double samplePeriod,
                                                      double sampleOffset,
                                                      int decimation,
                                                      int maxPoints,
                                                      const char* entryPath,
                                                      const char* entrySID,
                                                      const int builtInType);

EXTERN_C LIBMWXILUTILS_SL_API void* xilSimulinkUtilsCreateAliasSignal (
                                                      void* pXILUtils,
                                                      const char* signalName,
                                                      const char* propagateName,
                                                      int dimension,
                                                      int* dimensions,
                                                      int portIdx,
                                                      bool isComplex,
                                                      double samplePeriod,
                                                      double sampleOffset,
                                                      int decimation,
                                                      int maxPoints,
                                                      const char* entryPath,
                                                      const char* entrySID,
                                                      const char* aliasName,
                                                      const int builtInType);

EXTERN_C LIBMWXILUTILS_SL_API void* xilSimulinkUtilsCreateEnumSignal (
                                                    void* pXILUtils,
                                                    const char* signalName,
                                                    const char* propagateName,
                                                    int dimension,
                                                    int* dimensions,
                                                    int portIdx,
                                                    bool isComplex,
                                                    double samplePeriod,
                                                    double sampleOffset,
                                                    int decimation,
                                                    int maxPoints,
                                                    const char* entryPath,
                                                    const char* entrySID,
                                                    const char* enumName,
                                                    int enumNumEntries,
                                                    const char_T* enumLabels[],
                                                    const int32_T* enumValues);

EXTERN_C LIBMWXILUTILS_SL_API void* xilSimulinkUtilsCreateFixedPointSlopeBiasSignal (
                                                    void* pXILUtils,
                                                    const char* signalName,
                                                    const char* propagateName,
                                                    int dimension,
                                                    int* dimensions,
                                                    int portIdx,
                                                    bool isComplex,
                                                    double samplePeriod,
                                                    double sampleOffset,
                                                    int decimation,
                                                    int maxPoints,
                                                    const char* entryPath,
                                                    const char* entrySID,
                                                    const int numericType,
                                                    const int signedness,
                                                    const int wordLength,
                                                    const double slopeAdjFactor,
                                                    const int fixedExponent,
                                                    const double bias);

EXTERN_C LIBMWXILUTILS_SL_API void* xilSimulinkUtilsCreateFixedPointBinaryScalingSignal (
                                                     void* pXILUtils,
                                                     const char* signalName,
                                                     const char* propagateName,
                                                     int dimension,
                                                     int* dimensions,
                                                     int portIdx,
                                                     bool isComplex,
                                                     double samplePeriod,
                                                     double sampleOffset,
                                                     int decimation,
                                                     int maxPoints,
                                                     const char* entryPath,
                                                     const char* entrySID,
                                                     const int numericType,
                                                     const int signedness,
                                                     const int wordLength,
                                                     const int fractionLength);

EXTERN_C LIBMWXILUTILS_SL_API void* xilSimulinkUtilsStaticInitMultiRunInstrData();

EXTERN_C LIBMWXILUTILS_SL_API void xilSimulinkUtilsStaticFreeMultiRunInstrData(void* multiRunData);

EXTERN_C LIBMWXILUTILS_SL_API void * xilSimulinkUtilsReserveRowMajorBuf(void * const pXILSimulinkUtilsVoid,
                                                                        size_t sizeBytes);

EXTERN_C LIBMWXILUTILS_SL_API void * xilSimulinkUtilsTranspose(void * const pXILSimulinkUtilsVoid,
                                                               const int dataType,
                                                               void * srcData,
                                                               void * dstData,
                                                               const int * dims,
                                                               size_t nDims,
                                                               XIL_TRANSPOSE_DIRECTION_T direction,
                                                               boolean_T isComplex);

EXTERN_C LIBMWXILUTILS_SL_API void xilGetSetCurrentPortDims(SimStruct * S,
                                                            XIL_VARSIZE_DIRECTION_T direction,
                                                            int sfunIdx,
                                                            int elementOffset,
                                                            int nDims, void * dims,
                                                            XIL_VARSIZE_TYPE_T type);

#endif

