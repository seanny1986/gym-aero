/**
 * @file    asynciocoder_api.hpp
 * @Copyright 2018 The MathWorks, Inc.
 */

#ifndef AsyncioCoder_API_header
#define AsyncioCoder_API_header

#ifdef BUILDING_ASYNCIOCODER
#include "asynciocoder/asynciocoder_util.hpp"
#else
#include "asynciocoder_util.hpp" /* To deal with the fact that PackNGo has no include file hierarchy during test */
#endif

/* For uint64_t */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void* CoderChannel;  /* @todo change to void or Channel instead of void* */
typedef void* CoderInputStream;
typedef void* CoderOutputStream;
typedef void* CoderStream;

/* Channel */
EXTERN_C ASYNCIOCODER_API CoderChannel coderChannelCreate(const char* const devicePlugin,
                                                          const char* const converterPlugin,
                                                          double const inputStreamLimit,
                                                          double const outputStreamLimit,
                                                          char* errorID,
                                                          char* errorText);
EXTERN_C ASYNCIOCODER_API int coderChannelInit(CoderChannel cc,
                                               size_t numArgs,
                                               ...); /* ... is char* propName, char* propType, int
                                                        propLength, and T or T* propValue repeated
                                                        for each of numArgs */
EXTERN_C ASYNCIOCODER_API int coderChannelOpen(CoderChannel cc,
                                               size_t numArgs,
                                               ...); /* ... is char* propName, char* propType, int
                                                        propLength, and T or T* propValue repeated
                                                        for each of numArgs */
EXTERN_C ASYNCIOCODER_API int coderChannelExecute(CoderChannel cc,
                                                  char* command,
                                                  size_t numArgs,
                                                  ...); /* ... is char* propName, char* propType,
                                                           int propLength, and T or T* propValue
                                                           repeated for each of numArgs */
EXTERN_C ASYNCIOCODER_API int coderChannelClose(CoderChannel cc);
EXTERN_C ASYNCIOCODER_API int coderChannelTerm(CoderChannel cc);
EXTERN_C ASYNCIOCODER_API void coderChannelDestroy(CoderChannel cc);
EXTERN_C ASYNCIOCODER_API int coderChannelIsOpen(CoderChannel cc, boolean_T* open);
EXTERN_C ASYNCIOCODER_API int coderChannelGetPropertyValue(CoderChannel cc,
                                                           char* propName,
                                                           char* propType,
                                                           int propLength,
                                                           ...); /* ... is T* property value. */
EXTERN_C ASYNCIOCODER_API CoderInputStream coderChannelGetInputStream(CoderChannel cc);
EXTERN_C ASYNCIOCODER_API CoderOutputStream coderChannelGetOutputStream(CoderChannel cc);

EXTERN_C ASYNCIOCODER_API int coderChannelGetLastError(CoderChannel cc,
                                                       char* errorID,
                                                       char* errorText);
EXTERN_C ASYNCIOCODER_API int coderChannelHasSyncError(CoderChannel cc, boolean_T* hasError);
EXTERN_C ASYNCIOCODER_API int coderChannelGetLastSyncError(CoderChannel cc,
                                                           char* errorID,
                                                           char* errorText);
EXTERN_C ASYNCIOCODER_API int coderChannelHasAsyncError(CoderChannel cc, boolean_T* hasError);
EXTERN_C ASYNCIOCODER_API int coderChannelGetLastAsyncError(CoderChannel cc,
                                                            char* errorID,
                                                            char* errorText);

/* Stream */
EXTERN_C ASYNCIOCODER_API CoderChannel coderStreamGetChannel(CoderStream cs);
EXTERN_C ASYNCIOCODER_API int coderStreamIsSupported(CoderStream cs, boolean_T* supported);
EXTERN_C ASYNCIOCODER_API int coderStreamIsDeviceDone(CoderStream cs, boolean_T* done);
EXTERN_C ASYNCIOCODER_API int coderStreamIsOpen(CoderStream cs, boolean_T* open);
EXTERN_C ASYNCIOCODER_API int coderStreamGetCountDimension(CoderStream cs, double* countDimension);
EXTERN_C ASYNCIOCODER_API int coderStreamGetDataAvailable(CoderStream cs, double* dataAvailable);
EXTERN_C ASYNCIOCODER_API int coderStreamGetSpaceAvailable(CoderStream cs, double* spaceAvailable);
EXTERN_C ASYNCIOCODER_API int coderStreamAddFilter(CoderStream cs,
                                                   const char* const filterPlugin,
                                                   size_t numArgs,
                                                   ...); /* ... is char* propName, char* propType,
                                                            int propLength, and T or T* propValue
                                                            repeated for each of numArgs */
EXTERN_C ASYNCIOCODER_API int coderStreamTuneFilters(CoderStream cs, size_t numArgs, ...);
EXTERN_C ASYNCIOCODER_API int coderStreamFlush(CoderStream cs);

/* InputStream */
EXTERN_C ASYNCIOCODER_API int coderInputStreamPeek(CoderInputStream cs,
                                                   const double countRequested,
                                                   double* countToRead,
                                                   double* bufferCounts,
                                                   double* numBuffersCounts);
EXTERN_C ASYNCIOCODER_API int coderInputStreamReadBuffers(CoderInputStream cs,
                                                          const double countToRead,
                                                          double buffersToRead);
EXTERN_C ASYNCIOCODER_API int coderInputStreamReadBufferData(CoderInputStream cs,
                                                             const int bufferIndex,
                                                             const int itemIndex,
                                                             const char* name,
                                                             const char* type,
                                                             const int length,
                                                             void* data);
EXTERN_C ASYNCIOCODER_API void coderInputStreamFreeBuffer(CoderInputStream cs,
                                                          const int bufferIndex);

/* The following APIs should be used for reading data. They will be needed to support MATLAB Data
 * Arrays */
/*EXTERN_C ASYNCIOCODER_API int coderInputStreamReadStructData(CoderInputStream cs, const double countRequested, double* countRead, size_t numFields, ...); */ /* ... is char* name, char* type, int length, and T* result repeated for each of numFields */
/*EXTERN_C ASYNCIOCODER_API int coderInputStreamReadTypedData(CoderInputStream cs, const double countRequested, double* countRead, size_t numPackets, char*type, ...); */ /* ...  length and T* repeated for each of numPackets */

/* OutputStream */
/* The following APIs should be used for writing data. They will be needed to support MATLAB Data
 * Arrays */
EXTERN_C ASYNCIOCODER_API int coderOutputStreamCreateStructBuffer(
    CoderOutputStream cs,
    const double countDimension,
    const int numDims,
    const int* dims,
    const int numFields,
    ...); /*  ... is char* for each field. */
EXTERN_C ASYNCIOCODER_API int coderOutputStreamCreateTypedBuffer(CoderOutputStream cs,
                                                                 const double countDimension,
                                                                 const int numDims,
                                                                 const int* dims,
                                                                 const char* type);
EXTERN_C ASYNCIOCODER_API int coderOutputStreamWriteStructData(CoderOutputStream cs,
                                                               const int bufferIndex,
                                                               const int itemIndex,
                                                               const char* field,
                                                               const char* type,
                                                               const int length,
                                                               void* data);
EXTERN_C ASYNCIOCODER_API int coderOutputStreamWriteTypedData(CoderOutputStream cs,
                                                              const int bufferIndex,
                                                              const char* type,
                                                              const int length,
                                                              void* data);
EXTERN_C ASYNCIOCODER_API int coderOutputStreamWriteBuffers(CoderOutputStream cs);
/* @todo remove when the above are supported */
EXTERN_C ASYNCIOCODER_API int coderOutputStreamWriteTypedDataOld(
    CoderOutputStream cs,
    double* countWritten,
    size_t numPackets,
    char* type,
    ...); /* ... length and T* repeated for each of numPackets */

/* Utility */
EXTERN_C ASYNCIOCODER_API void coderComputeAbsolutePath(const char* const inputFileName,
                                                        char* outputFileName);
        
#ifdef __cplusplus
}
#endif

#endif
