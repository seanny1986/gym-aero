/* Copyright 1995-2017 The MathWorks, Inc. */

#ifndef SFC_MEX_H

#define SFC_MEX_H

#include "matrix.h"
#include "mex.h"
#include <setjmp.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "tmwtypes.h"
#include "emlrt.h"
#include "utils.h"
#include "messages/slMessagesSfcnBridge.h"

#define MAX_DIMENSIONS (16)
#define SFC_STRBUF_LEN (1024)
#define SF_MEX_ERROR_BUFFER_LEN (2048*2)
#define SF_MEX_ERROR_PREFIX_LEN 256

struct SfDebugInstanceStruct;

LIBMWSF_RUNTIME_API jmp_buf* sf_jit_get_jmpbuf(void);
LIBMWSF_RUNTIME_API jmp_buf * sf_jit_set_jmpbuf(jmp_buf *jumpbufPtr);

LIBMWSF_RUNTIME_API bool sf_mex_set_using_jit_mode(bool val);
LIBMWSF_RUNTIME_API bool sf_mex_using_jit_mode(void);

LIBMWSF_RUNTIME_API SimStruct* sf_mex_set_simstruct_for_jit_mode(SimStruct* S);
LIBMWSF_RUNTIME_API SimStruct* sf_mex_get_simstruct_for_jit_mode(void);

LIBMWSF_RUNTIME_API bool sf_jit_get_error_flag(void);
LIBMWSF_RUNTIME_API void sf_jit_clear_error_flag(void);
LIBMWSF_RUNTIME_API const char* sf_jit_get_error_txt(void);

LIBMWSF_RUNTIME_API void sf_jit_set_tryCatchForJITBoundary(bool);
LIBMWSF_RUNTIME_API bool sf_jit_get_tryCatchForJITBoundary(void);

LIBMWSF_RUNTIME_API void sf_jit_set_doLongJmp(bool doLongjmp);
LIBMWSF_RUNTIME_API bool sf_jit_get_doLongJmp(void);

LIBMWSF_RUNTIME_API void sf_mex_initialize_model_data(SimStruct* S);

LIBMWSF_RUNTIME_API int strcmp_ignore_ws(const char* string1, const char* string2);
LIBMWSF_RUNTIME_API const char* sf_mex_get_error_message_for_read(void);
LIBMWSF_RUNTIME_API unsigned int sf_mex_get_error_message_buffer_length(void);

LIBMWSF_RUNTIME_API const mxArray* sf_mex_evalin(MatlabWorkspaceType workspaceType, unsigned int numReturn, const char* evalFormatStr, ...);

LIBMWSF_RUNTIME_API const mxArray* sf_mex_call(const char* functionName, unsigned int numReturn, unsigned int numArguments, ...);
LIBMWSF_RUNTIME_API const mxArray* sf_mex_call_debug(struct SfDebugInstanceStruct* debugInstance, const char* functionName, unsigned int numReturn, unsigned int numArguments, ...);

LIBMWSF_RUNTIME_API void sf_mex_v_errbuf_createf(char* errorBuffer, const char* fmt, va_list argPtr);
LIBMWSF_RUNTIME_API void sf_mex_errbuf_createf(char* errorBuffer, const char* fmt, ...);
LIBMWSF_RUNTIME_API void sf_mex_v_errbuf_catf(char* errorBuffer, const char* fmt, va_list argPtr);
LIBMWSF_RUNTIME_API void sf_mex_errbuf_catf(char* errorBuffer, const char* fmt, ...);
LIBMWSF_RUNTIME_API void sf_mex_errbuf_copy(char* errorBuffer, const char* inStr);
LIBMWSF_RUNTIME_API void sf_mex_errbuf_clear(char* errorBuffer);
LIBMWSF_RUNTIME_API bool sf_mex_errbuf_isempty(char* errorBuffer);

LIBMWSF_RUNTIME_API void sf_mex_error_message(const char* msg, ...);
LIBMWSF_RUNTIME_API void sf_mex_error_message_no_long_jump(const char* errMsg);
LIBMWSF_RUNTIME_API void sf_mex_error_direct_call(SimStruct* s, const char* fcnName, const char* fileName);
LIBMWSF_RUNTIME_API void sf_mex_warning_message(const char* msg, ...);
LIBMWSF_RUNTIME_API void sf_mex_ignore_message(const char* msg, ...);
LIBMWSF_RUNTIME_API void sf_assertion_failed(const char* expr, const char* in_file, int at_line, const char* msg);
LIBMWSF_RUNTIME_API void sf_clear_error_manager_info(void);
LIBMWSF_RUNTIME_API void sf_set_error_prefix_string(const char* errorPrefixString);
LIBMWSF_RUNTIME_API char* sf_get_error_prefix_string(void);
LIBMWSF_RUNTIME_API void sf_mex_size_one_check(boolean_T aCondition, const char* aName);
LIBMWSF_RUNTIME_API const mxArray* sf_mex_getfield(const mxArray* mxStruct,
                                                   const char* fldName,      
                                                   const char* errName,      
                                                   int index);               
LIBMWSF_RUNTIME_API void sf_mex_setfieldbynum(const mxArray* mxStruct,
                                              int index,
                                              const char* fieldName,
                                              const mxArray* value,
                                              int fieldIdx);
LIBMWSF_RUNTIME_API const mxArray* sf_mex_addfield(const mxArray* mxStruct,
                                                   const mxArray* mxField,   
                                                   const char* fldName,      
                                                   const char* errName,      
                                                   int index);               
LIBMWSF_RUNTIME_API const mxArray* sf_mex_createfield(const mxArray* mxStruct,
                                                      const char* fldName,    
                                                      const char* errName);   
LIBMWSF_RUNTIME_API void sf_mex_check_struct(const struct emlrtMsgIdentifier* aMsgId, const mxArray* s, int32_T nFields, const char** fldNames,
                                             uint32_T nDims, const void* pDims);
LIBMWSF_RUNTIME_API void sf_mex_check_struct_vs(const struct emlrtMsgIdentifier* aMsgId, const mxArray* aSrcMx, int32_T aNFields, const char** aFldNames,
                                                uint32_T aNumDims,          
                                                boolean_T* aDynamic,   
                                                const void* aDimsMax,  
                                                void* aDimsActual);    
LIBMWSF_RUNTIME_API void sf_mex_check_bus_parameter(const mxArray* mxStruct, int32_T nDims, void* matrixSizes, int32_T nFields, const char** fieldNames, const char* paramName, const char* structName);

LIBMWSF_RUNTIME_API void sf_mex_check_cell(
    const struct emlrtMsgIdentifier* aMsgId,
    const mxArray* s,
    uint32_T nDims,
    const void* pDims,
    const boolean_T* aDynamic);
LIBMWSF_RUNTIME_API void sf_mex_check_cell_vs(
    const struct emlrtMsgIdentifier* aMsgId,
    const mxArray* s,
    uint32_T nDims,
    const void* pDims,
    const boolean_T* aDynamic,
    int32_T* aOutSizes);
LIBMWSF_RUNTIME_API void sf_mex_check_cell_with_unassigned_base(
    const struct emlrtMsgIdentifier* aMsgId,
    const mxArray* s);
LIBMWSF_RUNTIME_API void sf_mex_check_mcos_class(
    const struct emlrtMsgIdentifier *aMsgId,
    const mxArray *s,
    const char *aClassName);
LIBMWSF_RUNTIME_API void sf_mex_check_ct_mxarray(
    const struct emlrtMsgIdentifier *aMsgId,
    const mxArray *r,
    const mxArray *c);
LIBMWSF_RUNTIME_API mxArray *sf_mex_create_class_instance(
    const char *aClassName);
LIBMWSF_RUNTIME_API void sf_mex_set_all_properties(
    const mxArray **s,
    int32_T aIndex,
    int32_T aPropCount,
    const char **aPropNames,
    const char **aPropClasses,
    const mxArray **aPropValues);
LIBMWSF_RUNTIME_API void sf_mex_get_all_properties(
    const mxArray *s,
    int32_T aIndex,
    int32_T aPropCount,
    const char **aPropNames,
    const char **aPropClasses,
    const mxArray **aPropValues);
LIBMWSF_RUNTIME_API const mxArray *sf_mex_convert_to_redirect_target(
    const mxArray *s,
    int32_T aIndex,
    const char *aClassName);
LIBMWSF_RUNTIME_API const mxArray *sf_mex_convert_to_redirect_source(
    const mxArray *s,
    int32_T aIndex,
    const char *aClassName);
LIBMWSF_RUNTIME_API bool sf_mex_check_fi(const struct emlrtMsgIdentifier* aMsgId, const mxArray* fi, boolean_T complex, uint32_T nDims,
                                         const void* vDims, const mxArray* fimath, const mxArray* numericType);
LIBMWSF_RUNTIME_API void sf_mex_check_fi_vs(const struct emlrtMsgIdentifier* aMsgId, const mxArray* aSrcFI, boolean_T aComplex,
                                            uint32_T aNumDims,              
                                            const boolean_T* aDynamic, 
                                            const void* aDimsMax,      
                                            void* aDimsActual,         
                                            const mxArray* aFiMath, const mxArray* aNumericType);

LIBMWSF_RUNTIME_API const mxArray* sf_mex_create_fi(const mxArray* aFiMath,
	                                        const mxArray* aNType,
	                                        const char* aFiType,
	                                        const mxArray* aFiVal,
	                                        const boolean_T aIsLocal,
	                                        const boolean_T aForceComplex);

LIBMWSF_RUNTIME_API void sf_mex_check_builtin(const struct emlrtMsgIdentifier* aMsgId, const mxArray* mx, const char* clsName, boolean_T complex, uint32_T nDims, const void* vDims);

LIBMWSF_RUNTIME_API void sf_mex_check_builtin_vs(const struct emlrtMsgIdentifier* aMsgId, const mxArray* mx, const char* clsName, boolean_T complex, uint32_T nDims, const void* pDims, const boolean_T* aDynamic, void* aDimsOut);

LIBMWSF_RUNTIME_API void sf_mex_check_enum(const char* enumName, int numFields, const char** elNames, const int32_T* elValues);
LIBMWSF_RUNTIME_API const mxArray *sf_mex_create_sparse(void *d,int32_T *colidx,int32_T *rowidx,int32_T m,int32_T n,int32_T maxnz,int32_T aClassId,int32_T aComplexity);
LIBMWSF_RUNTIME_API void sf_mex_check_sparse(emlrtMsgIdentifier *aMsgId, mxArray *pa, int32_T *pDims, boolean_T *aDynamic, int32_T aClassId, int32_T aComplexity);

LIBMWSF_RUNTIME_API const mxArray* sf_mex_create_enum(const char* enumName, const mxArray* mxValues);
LIBMWSF_RUNTIME_API int32_T sf_mex_get_enum_element(const mxArray* mxValues, int index);

LIBMWSF_RUNTIME_API const mxArray* sf_mex_createstruct(const char* nameStr,     
                                                       int numFields,           
                                                       const char **fieldNames, 
                                                       int numDims,             
                                                       ...);                    
LIBMWSF_RUNTIME_API const mxArray* sf_mex_createstructarray(const char* aNameStr,     
                                                            int aNumDims,       
                                                            const void* aDims,  
                                                            int numFields,
                                                            const char** fldName);
LIBMWSF_RUNTIME_API const mxArray* sf_mex_create_from_dynamic_string(
    const char* nameStr,      
    void* dataPtr,            
    int dataType,             
    unsigned int isComplex,   
    int indexScheme,          
    unsigned int aElementSize,
    int numDims,              
    ...);                     
LIBMWSF_RUNTIME_API const mxArray* sf_mex_create(const char* nameStr,       
                                                 const void* dataPtr,      
                                                 int dataType,             
                                                 unsigned int isComplex,   
                                                 int indexScheme,          
                                                 unsigned int aElementSize,
                                                 int numDims,              
                                                 ...);                     

LIBMWSF_RUNTIME_API const mxArray* sf_mex_createcellarray(int aNumDims, const int* aDims);
LIBMWSF_RUNTIME_API const mxArray* sf_mex_createcellmatrix(int m, int n);

LIBMWSF_RUNTIME_API void sf_mex_setcell(const mxArray* array, int index, const mxArray* value);
LIBMWSF_RUNTIME_API void sf_mex_setfield(const mxArray* structArray, int index, const char* fieldName, const mxArray* value);
LIBMWSF_RUNTIME_API const mxArray* sf_mex_getcell(const mxArray* array, int index);

LIBMWSF_RUNTIME_API double sf_mex_ml_var_sub(const char* wsDataName,        
                                             MatlabWorkspaceType workspaceType,
                                             uint32_T numDims,                      
                                             ...);                             
LIBMWSF_RUNTIME_API void sf_mex_ml_var_subsasgn(const char* wsDataName,
                                                MatlabWorkspaceType workspaceType,
                                                double value,
                                                int numDims,
                                                ...);
LIBMWSF_RUNTIME_API void sf_mex_assign(const mxArray** sfMatlabDataPtr,
                                       const mxArray* srcMxArray, bool makePersistent);
LIBMWSF_RUNTIME_API const mxArray* sf_mex_dup(const mxArray* srcMxArray);
LIBMWSF_RUNTIME_API void sf_mex_destroy(const mxArray** sfMatlabDataPtr);
LIBMWSF_RUNTIME_API void sf_mex_export(const char* vectName,
                                       const mxArray* mxArrayPtr,
                                       MatlabWorkspaceType workspaceType);
LIBMWSF_RUNTIME_API void sf_mex_import(const struct emlrtMsgIdentifier* aMsgId,
                                       const mxArray* mxArrayPtr, 
                                       void* dataPtr,             
                                       int isStrict,              
                                       int dataType,              
                                       unsigned int isComplex,    
                                       int indexScheme,           
                                       unsigned int aElementSize, 
                                       int numDims,               
                                       ...);                      
LIBMWSF_RUNTIME_API void sf_mex_import_named(const char* nameStr, 
                                             const mxArray* mxArrayPtr, 
                                             void* dataPtr,             
                                             int isStrict,              
                                             int dataType,              
                                             unsigned int isComplex,    
                                             int indexScheme,           
                                             unsigned int aElementSize, 
                                             int numDims,               
                                             ...);                      
LIBMWSF_RUNTIME_API void sf_mex_import_dynamic_string(const char* nameStr,
                      const mxArray *aSrcMx,     
                      void **aDstData);            
LIBMWSF_RUNTIME_API void sf_mex_import_static_string(const char* nameStr,
                      const mxArray *aSrcMx,     
                      void *aDstData);            
LIBMWSF_RUNTIME_API void sf_mex_import_vs(const struct emlrtMsgIdentifier* aMsgId,
                                          const mxArray* aSrcMx,     
                                          void* aDstData,            
                                          int aStrict,               
                                          int aDataType,             
                                          unsigned int aComplex,     
                                          int aIndexScheme,          
                                          unsigned int aElementSz,   
                                          int aNumDims,              
                                          const boolean_T* aDynamic, 
                                          const void* aDimsMax,      
                                          void* aDimsActual);        
LIBMWSF_RUNTIME_API void sf_mex_import_vs_named(const char* nameStr, 
                                                const mxArray* aSrcMx,     
                                                void* aDstData,            
                                                int aStrict,               
                                                int aDataType,             
                                                unsigned int aComplex,     
                                                int aIndexScheme,          
                                                unsigned int aElementSz,   
                                                int aNumDims,              
                                                const boolean_T* aDynamic, 
                                                const void* aDimsMax,      
                                                void* aDimsActual);        
LIBMWSF_RUNTIME_API void sf_mex_import_fi_vs(
    const mxArray* aFiMx,    
    const mxArray* aIntMx,   
    void* aOutData,          
    int32_T aElementSize,    
    uint32_T aDimsMax,       
    void* aDimsActual);      
LIBMWSF_RUNTIME_API  const mxArray* sf_mex_convert_to_double(const mxArray* mxArrayPtr);
LIBMWSF_RUNTIME_API const mxArray* sf_mex_lower_fixpt_mx_array(const mxArray* mxArrayPtr,
                                                               int fixptExponent,
                                                               double fixptSlope,
                                                               double fixPtBias,
                                                               unsigned int nBits,
                                                               unsigned int isSigned);
LIBMWSF_RUNTIME_API double sf_mex_sub(const mxArray* mxArrayPtr,
                                      const char* nameStr,      
                                      uint32_T numDims,              
                                      ...);                     
LIBMWSF_RUNTIME_API void sf_mex_subsasgn(mxArray* mxArrayPtr, 
                                         const char* nameStr,      
                                         double value,
                                         uint32_T numDims,               
                                         ...);                      

LIBMWSF_RUNTIME_API mxArray* sf_mex_decode(const char* encStr);

LIBMWSF_RUNTIME_API const mxArray* sf_mex_duplicatearraysafe(const mxArray** ppMxArray);

LIBMWSF_RUNTIME_API int32_T sf_mex_get_dimension(const mxArray* aMx, int aDim);

LIBMWSF_RUNTIME_API int sf_mex_c_lw_bounds_check(int indexValue, int firstIndex, int lastIndex);
LIBMWSF_RUNTIME_API int sf_mex_lw_bounds_check(int indexValue, int firstIndex, int lastIndex);
#ifdef INT_TYPE_64_IS_SUPPORTED
LIBMWSF_RUNTIME_API int64_T sf_mex_lw_bounds_check_int64(int64_T indexValue, int firstIndex, int lastIndex);
#endif
LIBMWSF_RUNTIME_API void sf_mex_lw_size_eq_check_1d(int dim1, int dim2);
LIBMWSF_RUNTIME_API void sf_mex_lw_size_eq_check_nd(const int* dims1, const int* dims2, int nDims);
LIBMWSF_RUNTIME_API void sf_mex_lw_dim_size_eq_check(int dim1, int dim2, int index);
LIBMWSF_RUNTIME_API  void sf_mex_lw_dim_size_geq_check(int dim1, int dim2, int index);
LIBMWSF_RUNTIME_API   void sf_mex_lw_sub_assign_size_check_1d(int dim1, int dim2);
LIBMWSF_RUNTIME_API   void sf_mex_lw_sub_assign_size_check_nd(const int* dims1, int nDims1, const int* dims2, int nDims2);
LIBMWSF_RUNTIME_API   void sf_mex_lw_for_loop_vector_check(double start, double step, double end, mxClassID classID, int n);
LIBMWSF_RUNTIME_API   void sf_mex_lw_runtime_error_msgid(const char* msgID);

LIBMWSF_RUNTIME_API    double sf_mex_lw_non_negative_check(double val);

#define EML_FOR_LOOP_VECTOR_MSG "Cannot determine the exact number of iterations for a loop with range %g:%g:%g."

#define sf_assertion(expr) ((expr)?(void)0:sf_assertion_failed(#expr,__FILE__,__LINE__,NULL))
#define sf_assertion_msg(expr,msg) ((expr)?(void)0:sf_assertion_failed(#expr,__FILE__,__LINE__,msg))

LIBMWSF_RUNTIME_API void sf_mex_overflow_error(void);
LIBMWSF_RUNTIME_API int* ssGetCurrentInputPortDimensions_wrapper(SimStruct* S, int portNumber);
LIBMWSF_RUNTIME_API int* ssGetCurrentOutputPortDimensions_wrapper(SimStruct* S, int portNumber);
LIBMWSF_RUNTIME_API bool sf_mex_emlrt_context_is_debug_mode(void);
LIBMWSF_RUNTIME_API void sf_mex_emlrt_context_set_debug_mode(bool b);
LIBMWSF_RUNTIME_API void sf_mex_emlrtNameCapturePostProcessR2012a(const mxArray** mxInfo);
LIBMWSF_RUNTIME_API const mxArray* sf_mex_emlrtCoerceToClass(const mxArray* mxArrayPtr, const char* clsName);

LIBMWSF_RUNTIME_API void ssGetSFcnDataStoreNameAddrIdx_wrapper(SimStruct* S, const char* name, void** dsmAddress, int* dsmIndex);
LIBMWSF_RUNTIME_API void ssReadFromDataStore_wrapper(SimStruct* S, int dsmIndex, const char* dsmName);
LIBMWSF_RUNTIME_API void ssReadFromDataStoreElement_wrapper(SimStruct* S, int dsmIndex, const char* dsmName, int elementIndex);
LIBMWSF_RUNTIME_API void ssWriteToDataStore_wrapper(SimStruct* S, int dsmIndex, const char* dsmName);
LIBMWSF_RUNTIME_API void ssWriteToDataStoreElement_wrapper(SimStruct* S, int dsmIndex, const char* dsmName, int ElementIndex);
LIBMWSF_RUNTIME_API void ssAccessDataStore_wrapper(SimStruct* S, int dsmIndex, const char* dsmName, int isReadOnly);
LIBMWSF_RUNTIME_API bool ssGetDSMBlockDiagnosticsEnabled_wrapper(SimStruct* S, int dsmIdx, const char* dsmName);

LIBMWSF_RUNTIME_API void ssMemRegionCreateDescriptor_wrapper(SimStruct* S, int numSubMemRegions, SFcnMemRegionInfo** memRegionDescriptor);
LIBMWSF_RUNTIME_API void ssMemRegionSetFlatSubElement_wrapper(SimStruct* S, SFcnMemRegionInfo* memRegionDescriptor, int subRegionIdx, int busElementIdx, int numFlatIdx, int* flatIdxs);
LIBMWSF_RUNTIME_API void ssReadFromDataStoreRegion_wrapper(SimStruct* S, int dsmIdx, void* dataAddr,  SFcnMemRegionInfo* dsmRegionDescriptor);
LIBMWSF_RUNTIME_API void ssWriteToDataStoreRegion_wrapper(SimStruct* S, int dsmIdx, void* dataAddr, SFcnMemRegionInfo* dsmRegionDescriptor);
LIBMWSF_RUNTIME_API void ssMemRegionDestroyDescriptor_wrapper(SimStruct* S, SFcnMemRegionInfo** memRegionDescriptor);

LIBMWSF_RUNTIME_API void ssUpdateDataStoreLog_wrapper(SimStruct* S, int dsmIdx);

LIBMWSF_RUNTIME_API void* ssGetDWork_wrapper(SimStruct* S, int index);
LIBMWSF_RUNTIME_API real_T* ssGetdX_wrapper(SimStruct* S);
LIBMWSF_RUNTIME_API real_T* ssGetNonsampledZCs_wrapper(SimStruct* S);
LIBMWSF_RUNTIME_API int* ssGetCurrentInputPortDimensions_wrapper(SimStruct* S, int portNumber);
LIBMWSF_RUNTIME_API int* ssGetCurrentOutputPortDimensions_wrapper(SimStruct* S, int portNumber);
LIBMWSF_RUNTIME_API void ssSetCurrentOutputPortDimensions_wrapper(SimStruct* S, int pIdx, int dIdx, int val);
LIBMWSF_RUNTIME_API const void* ssGetInputPortSignal_wrapper(SimStruct* S, int ip);
LIBMWSF_RUNTIME_API InputPtrsType ssGetInputPortSignalPtrs_wrapper(SimStruct* S, int ip);
LIBMWSF_RUNTIME_API void* ssGetOutputPortSignal_wrapper(SimStruct* S, int i);
LIBMWSF_RUNTIME_API real_T* ssGetContStates_wrapper(SimStruct* S);

LIBMWSF_RUNTIME_API int_T     ssGetNumMessagesInQueue_wrapper(SimStruct* S, int qType, int queueID);
LIBMWSF_RUNTIME_API int_T     ssGetNumMessagesInOutputQueue_wrapper(SimStruct* S, int portIdx);
LIBMWSF_RUNTIME_API void*     ssMessageQueuePeekPayload_wrapper(SimStruct* S, int qType, int queueID);
LIBMWSF_RUNTIME_API void*     ssMessageQueuePeekPayloadAtIndex_wrapper(SimStruct* S, int qType, int queueID, int payloadIndex);
LIBMWSF_RUNTIME_API void*     ssMessageOutputQueuePeekPayloadAtIndex_wrapper(SimStruct* S, int portIndex, int payloadIndex);
LIBMWSF_RUNTIME_API void      ssMessagePayloadDestroy_wrapper(SimStruct* S, void* msgHandle);
LIBMWSF_RUNTIME_API ssMessage ssMessageQueuePop_wrapper(SimStruct* S, int qType, int queueID, bool isDesChart, bool isMsgTrigger);
LIBMWSF_RUNTIME_API void      ssMessageSend_wrapper(SimStruct* S, int outputPortNum, void* payload, unsigned int priority);
LIBMWSF_RUNTIME_API void      ssMessageSendLocal_wrapper(SimStruct* S, int queueId, void* payload, unsigned int priority);
LIBMWSF_RUNTIME_API void      ssMessageDestroy_wrapper(SimStruct* S, void* msgData);
LIBMWSF_RUNTIME_API void      ssSetMessagePayload_wrapper(SimStruct* S, void* msgHandle, const void* updatedPayload);
LIBMWSF_RUNTIME_API void*     ssGetMessagePayload_wrapper(SimStruct* S, void* msgHandle);
LIBMWSF_RUNTIME_API void      ssMessageRelay_wrapper(SimStruct* S, int queueType, int pIdx, ssMessage msgObj);
LIBMWSF_RUNTIME_API bool      ssIsMessageQueueFull_wrapper(SimStruct *S, int queueType, int portIdx, int isOutputPort);
LIBMWSF_RUNTIME_API size_t    ssGetNumDroppedMessagesFromLocalQueue_wrapper(SimStruct *S, int queueId);
LIBMWSF_RUNTIME_API size_t    ssGetRecommendedCapacityForLocalQueue_wrapper(SimStruct *S, int queueId);

LIBMWSF_RUNTIME_API mxArray*  mxCreateStructMatrix_wrapper(int numRows, int numCols, int numFields, const char** fieldNames);

LIBMWSF_RUNTIME_API void  sf_memcheck_start(int aMaxHeapSize);
LIBMWSF_RUNTIME_API void  sf_memcheck_end(void);
LIBMWSF_RUNTIME_API void  sf_memcheck_done(void);

#ifndef TRUE
#define TRUE (1U)
#endif

#ifndef FALSE
#define FALSE (0U)
#endif

#ifdef mexPrintf
#undef mexPrintf
#endif
#define mexPrintf sf_mex_printf
#ifdef printf
#undef printf
#endif
#define printf sf_mex_printf

#ifndef sf_MIN
#define sf_MIN(a,b)    (((a) < (b)) ? (a) : (b))
#endif
#ifndef sf_MAX
#define sf_MAX(a,b)    (((a) > (b)) ? (a) : (b))
#endif

#ifndef RT_PI
#define RT_PI          3.14159265358979323846
#endif

#ifndef RT_PIF
#define RT_PIF         3.1415927F
#endif

#ifndef RT_LN_10
#define RT_LN_10       2.30258509299404568402
#endif

#ifndef RT_LN_10F
#define RT_LN_10F      2.3025851F
#endif

#ifndef RT_LOG10E
#define RT_LOG10E      0.43429448190325182765
#endif

#ifndef RT_LOG10EF
#define RT_LOG10EF     0.43429449F
#endif

#ifndef RT_E
#define RT_E           2.7182818284590452354
#endif

#ifndef RT_EF
#define RT_EF          2.7182817F
#endif

#endif
