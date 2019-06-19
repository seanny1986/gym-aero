/* Copyright 2013-2014 The MathWorks, Inc. */

#ifndef _sf_runtime_sf_sdi_h_
#define _sf_runtime_sf_sdi_h_

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#include "sf_runtime_spec.h"
#include "tmwtypes.h"

typedef enum {
    SF_DATA_MODE = 0,
    SF_CHILD_ACTIVITY_MODE  = 0x01,
    SF_LEAF_ACTIVITY_MODE   = 0x02,
    SF_SELF_ACTIVITY_MODE   = 0x04
} SFSdiModeFlag;

typedef enum {
    SF_DATA_TYPE_BEGIN = 0,
    SF_DATA_TYPE_DOUBLE = SF_DATA_TYPE_BEGIN,
    SF_DATA_TYPE_SINGLE,
    SF_DATA_TYPE_INT8,
    SF_DATA_TYPE_UINT8,
    SF_DATA_TYPE_INT16,
    SF_DATA_TYPE_UINT16,
    SF_DATA_TYPE_INT32,
    SF_DATA_TYPE_UINT32,
    SF_DATA_TYPE_BOOLEAN
} SF2SdiBuiltInDataTypes;

typedef enum {
    SF_DIMENSIONS_BEGIN = 0,
    SF_DIMENSIONS_MODE_FIXED = SF_DIMENSIONS_BEGIN,
    SF_DIMENSIONS_MODE_VARIABLE
} SF2SdiDimsMode;

typedef enum {
    SF_COMPLEXITY_BEGIN = 0,
    SF_REAL = SF_COMPLEXITY_BEGIN,
    SF_COMPLEX
} SF2SdiComplexity;

typedef enum {
    SF_SAMPLE_TIME_BEGIN = 0,
    SF_SAMPLE_TIME_CONTINUOUS = SF_SAMPLE_TIME_BEGIN,
    SF_SAMPLE_TIME_DISCRETE
} SF2SdiUpdateMethod;

typedef struct SignalExportStruct_t {
    int useCustomName;
    int limitDataPoints;
    int decimate;
    const char* logName;
    const char* signalName; 
    unsigned int maxPoints;
    unsigned int decimation;
} SignalExportStruct;

typedef struct sdiBlockID {
    void* SimStruct;
    char* mdlRefFullPath;
    char* blkPath;
    char* blkSID; 
    char* sharedSCMPath;
    void* sdiRuntime;
} sdiBlockID_t;

LIBMWSF_RUNTIME_API void sdi_database_initialize(struct sdiBlockID* blockId,
                                                 unsigned int* stateSSIDs,
                                                 uint8_T* loggedStatesBuffer, int numStates,
                                                 unsigned int* dataSSIDs,
                                                 uint8_T* loggedDataBuffer, int numData); 

LIBMWSF_RUNTIME_API void sdi_database_terminate(struct sdiBlockID* blockId);

LIBMWSF_RUNTIME_API void sdi_register_self_activity_signal(
    struct sdiBlockID* blockId,
    const char* targetName,
    const char* ssid,
    int uMethod,
    SignalExportStruct *sigExportProps,
    unsigned int ssId);

LIBMWSF_RUNTIME_API void sdi_register_child_activity_signal(
    struct sdiBlockID* blockId,
    const char* targetName,
    const char* parentStateRelativePath,
    int numChildStates,
    const char** stateNames,
    int* values,
    int signalSize,
    int uMethod,
    unsigned int ssId);

LIBMWSF_RUNTIME_API void sdi_register_leaf_activity_signal(
    struct sdiBlockID* blockId,
    const char* targetName,
    const char* parentStateRelativePath,
    int numLeafStates,
    const char** stateNames,
    int* values,
    int uMethod,
    unsigned int ssId);

LIBMWSF_RUNTIME_API void sdi_register_enum_data_type_signal(
    struct sdiBlockID* blockId,
    const char* targetName,
    const char* parentStateRelativePath,
    const char* enumTypeName,
    int numLiterals,
    const char** literals,
    int* enumValues,
    int storageSize, int isSigned,
    int numDims, int* dimsArray,
    int uMethod,
    int dimsMode,
    int complexity,
    SignalExportStruct *sigExportProps,
    unsigned int ssId);

LIBMWSF_RUNTIME_API void sdi_register_string_data_type_signal(
    struct sdiBlockID* blockId,
    const char* targetName,
    const char* parentStateRelativePath,
    int numDims,
    int* dims,
    int uMethod,
    SignalExportStruct *sigExportProps,
    unsigned int ssId);

LIBMWSF_RUNTIME_API void sdi_register_builtin_data_type_signal(
    struct sdiBlockID* blockId,
    const char* targetName,
    const char* parentStateRelativePath,
    int numDims,
    int* dims,
    int updateMethod,
    int typ,
    int dimsMode,
    int complexity,
    SignalExportStruct *sigExportProps,
    const char* units,
    unsigned int ssId);

LIBMWSF_RUNTIME_API void sdi_register_fixed_point_data_type_slope_bias_scaling(
    struct sdiBlockID* blockId,
    const char* targetName,
    const char* parentStateRelativePath,
    int numDims,
    int* dimsArray,
    int uMethod,
    int dimsMode,
    int complexity,
    int numericType,
    unsigned int signedness,
    int wordLength,
    double slopeAdjustFactor,
    int fixedExponent,
    double bias,
    SignalExportStruct *sigExportProps,
    const char* units,
    unsigned int ssId);

LIBMWSF_RUNTIME_API void sdi_register_fixed_point_data_type_binary_point_scaling(
    struct sdiBlockID* blockId,
    const char* targetName,
    const char* parentStateRelativePath,
    int numDims,
    int* dimsArray,
    int uMethod,
    int dimsMode,
    int complexity,
    int numericType,
    unsigned int signedness,
    int wordLength,
    int fractionLength,
    SignalExportStruct *sigExportProps,
    const char* units,
    unsigned int ssId);

LIBMWSF_RUNTIME_API void sdi_register_non_virtual_bus_parent(
    struct sdiBlockID* blockId,
    unsigned int ssId,
    unsigned int busIdx,
    unsigned int parentsIdx,
    unsigned int busSize,
    const char* targetName,
    int numDims,
    int* dimensions);

LIBMWSF_RUNTIME_API void sdi_register_non_virtual_bus_built_in_type_leaf(
    struct sdiBlockID* blockId,
    int uMethod,
    unsigned int ssId,
    unsigned int parentsIdx,
    const char* targetName,
    int numDims,
    int* dimensions,
    int leafComplexity,
    int leafDataType,
    int leafOffset,
    const char* units);

LIBMWSF_RUNTIME_API void sdi_register_non_virtual_bus_fixpt_slope_bias_scaling_type_leaf(
    struct sdiBlockID* blockId,
    int uMethod,
    unsigned int ssId,
    unsigned int parentsIdx,
    const char* targetName,
    int numDims,
    int* dimensions,
    int leafComplexity,
    int numericType,
    unsigned int signedness,
    int wordLength,
    double slopeAdjustFactor,
    int fixedExponent,
    double bias,
    int leafOffset,
    const char* units);

LIBMWSF_RUNTIME_API void sdi_register_non_virtual_bus_fixpt_binary_point_scaling_type_leaf(
    struct sdiBlockID* blockId,
    int uMethod,
    unsigned int ssId,
    unsigned int parentsIdx,
    const char* targetName,
    int numDims,
    int* dimensions,
    int leafComplexity,
    int numericType,
    unsigned int signedness,
    int wordLength,
    int fractionLength,
    int leafOffset,
    const char* units);

LIBMWSF_RUNTIME_API void sdi_register_non_virtual_bus_enum_type_type_leaf(
    struct sdiBlockID* blockId,
    int uMethod,
    unsigned int ssId,
    unsigned int parentsIdx,
    const char* targetName,
    int numDims,
    int* dimensions,
    int leafComplexity,
    const char* enumName,
    int numLiterals,
    const char** literals,
    int* enumValues,
    int storageSize,
    int isSigned,
    int leafOffset);

LIBMWSF_RUNTIME_API void sdi_register_non_virtual_bus_slstring_type_type_leaf(
    struct sdiBlockID* blockId,
    int uMethod,
    unsigned int ssId,
    unsigned int parentsIdx,
    const char* targetName,
    int numDims,
    int* dimensions,
    int leafComplexity,
    unsigned int maxSize,
    bool isDynamic,
    int leafOffset);

LIBMWSF_RUNTIME_API void sdi_register_non_virtual_bus(
    struct sdiBlockID* blockId,
    void* busAddr,
    int uMethod,
    const char* targetName,
    const char* parentStateRelativePath,
    int numDims,
    int* dimensions,
    unsigned int busSize,
    SignalExportStruct *sigExportProps,
    unsigned int ssId);

LIBMWSF_RUNTIME_API int sdi_annotation(int Idx, void* data);
LIBMWSF_RUNTIME_API int sdi_annotation_nodata(int Idx);

LIBMWSF_RUNTIME_API unsigned int sim_mode_is_rtw_gen(SimStruct* S);
LIBMWSF_RUNTIME_API unsigned int sim_mode_is_external(SimStruct* S);
LIBMWSF_RUNTIME_API unsigned int sim_mode_is_modelref_sim(SimStruct* S);

LIBMWSF_RUNTIME_API void sdi_stream_child_activity_signal(struct sdiBlockID* blockId, unsigned int ssId, void* data);
LIBMWSF_RUNTIME_API void sdi_stream_data_signal(struct sdiBlockID* blockId, unsigned int ssId, void* data);
LIBMWSF_RUNTIME_API void sdi_stream_bus_element(struct sdiBlockID* blockId, unsigned int ssId, const char* elementExpr, void* elementData);
LIBMWSF_RUNTIME_API void sdi_update_self_activity_signal(struct sdiBlockID* blockId, unsigned int ssId, int value);
LIBMWSF_RUNTIME_API void sdi_update_leaf_activity_signal(struct sdiBlockID* blockId, unsigned int ssId, int value);

#endif

