/* Copyright 2013-2016 The MathWorks, Inc. */

#ifndef XILHostAppSvc_CInterface_h
#define XILHostAppSvc_CInterface_h

#include "coder/connectivity/XILHostAppSvc/XILHostAppSvc.h"
#include "coder/connectivity/XILHostAppSvc/XILHostAppSvc_spec.h"
#include "coder/connectivity/memunit/MemUnitTransformerType.h"

typedef struct {
    void* data;
    mwSize size;
    mwSize bufferDataSize;
    void* nextMemUnit;
    boolean_T readData;
} XIL_IOBuffer_T;

XILHOSTAPPSVC_API_EXTERN_C boolean_T xilHostAppSvcCreate(
        void** const ppXILHostAppSvc,
        void* const pCSVoid,
        void* const pXILUtils,                                          
        XIL_IOBuffer_T* const pIOBuffer,                                
        const uint8_T memUnitSize,                                      
        const uint8_T ioDataTypeSize,
        const boolean_T isUsingCommService);
       
XILHOSTAPPSVC_API_EXTERN_C const uint8_T xilHostAppSvcGetIsResponseComplete(
        void* const pXILHostAppSvc);

XILHOSTAPPSVC_API_EXTERN_C void xilHostAppSvcSetIsResponseComplete(
        void* const pXILHostAppSvc,                                                  
        const uint8_T val);

XILHOSTAPPSVC_API_EXTERN_C const mwSize xilHostAppSvcGetPayloadSizeForOneStep(
        void* const pXILHostAppSvc);

XILHOSTAPPSVC_API_EXTERN_C boolean_T xilHostAppSvcGrowIOBuffer(
        void* const pXILHostAppSvc,                                        
        XIL_IOBuffer_T* const IOBuffer,                                       
        const mwSize requiredSize,                                            
        const uint8_T isVerbose);

XILHOSTAPPSVC_API_EXTERN_C boolean_T xilHostAppSvcSend(
        void* const pXILHostAppSvc,                                      
        void* const src,                                              
        const mwSize size);

XILHOSTAPPSVC_API_EXTERN_C boolean_T xilHostAppSvcWaitForDataFromTarget(
        void* const pXILHostAppSvc);
      
XILHOSTAPPSVC_API_EXTERN_C void xilHostAppSvcDestroy(
        void* const pXILHostAppSvc);

XILHOSTAPPSVC_API_EXTERN_C size_t convertTypeIDToSize(mem_unit_type_T dataTypeId);

XILHOSTAPPSVC_API_EXTERN_C void copyIODataInternal(uint8_T * const dstPtr, uint8_T * const srcPtr, uint8_T **
  const tgtPtrPtr, size_t numElements, size_t cTypeSize);

XILHOSTAPPSVC_API_EXTERN_C void xilReadDataInternal(uint8_T *dst, size_t numElements, mem_unit_type_T dataTypeId, boolean_T suppressByteOrdering, XIL_IOBuffer_T* IOBufferPtr, void* memUnitTransformer);

XILHOSTAPPSVC_API_EXTERN_C boolean_T xilWriteDataInternal(uint8_T *src, size_t numElements, mem_unit_type_T dataTypeId, boolean_T suppressByteOrdering, XIL_IOBuffer_T* IOBufferPtr, void* memUnitTransformer, void* XILService);
#endif
