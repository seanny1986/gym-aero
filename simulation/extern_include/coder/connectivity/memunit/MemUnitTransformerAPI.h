/* Copyright 2011-2016 The MathWorks, Inc. */

#ifndef MemUnitTransformerAPI_h
#define MemUnitTransformerAPI_h

#include "tmwtypes.h"
#include "MemUnitTransformerType.h"

#ifndef EXTERN_C
#ifdef __cplusplus

#define EXTERN_C extern "C"
#else

#define EXTERN_C extern
#endif
#endif

#ifndef CODER_CONNECTIVITY_MEMUNIT_API

#define CODER_CONNECTIVITY_MEMUNIT_API
#endif

typedef enum MEM_UNIT_TRANSFORM_DIRECTION {
    MEM_UNIT_XFORM_OUTBOUND=0,
    MEM_UNIT_XFORM_INBOUND
} mem_unit_xform_direction_T;

#define MEM_UNIT_XFORM_ID(mem_unit_type_val, isInbound) (mem_unit_type_val*2+isInbound)

EXTERN_C CODER_CONNECTIVITY_MEMUNIT_API boolean_T memUnitXformer_Init(
    void ** const ppData,
    const uint_T wordLen, const boolean_T isByteAddressable,
    const boolean_T isLittleEndian, const size_t numTypesWithDiffEndian,
    const mem_unit_type_T * const typesWithDiffEndian);

EXTERN_C CODER_CONNECTIVITY_MEMUNIT_API void memUnitXformer_DoXform(
    const void* const pDataVoid, const unsigned int typeId, void ** pIn,
    void ** pOut, const size_t length);

EXTERN_C CODER_CONNECTIVITY_MEMUNIT_API void memUnitXformer_Destroy(void * const pData);

#endif
