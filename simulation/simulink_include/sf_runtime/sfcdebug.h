/* Copyright 1995-2016 The MathWorks, Inc. */

#ifndef RT_DEBUGGER_H
#define RT_DEBUGGER_H

#include <setjmp.h>
#include "mex.h"
#include "tmwtypes.h"
#include "sfc_mex.h"
#include "sf_runtime_spec.h"

#define UNREASONABLE_NUMBER 999999

typedef const mxArray* (*MexFcnForType)(void* S, void* addr);
typedef void (*MexInFcnForType)(void* S, const mxArray*, const char_T*, void* addr);

typedef enum
{
    CLUSTER_STATE,
    SET_STATE
} SfDecomposition;

#ifndef CV_EML_CHECK_TYPE_DEF
#define CV_EML_CHECK_TYPE_DEF

typedef enum
{
    CV_EML_FCN_CHECK = 0,
    CV_EML_IF_CHECK,
    CV_EML_FOR_CHECK,
    CV_EML_WHILE_CHECK,
    CV_EML_COND_CHECK,
    CV_EML_MCDC_CHECK,
    CV_EML_SWITCH_CHECK,
    CV_EML_TESTOBJECTIVE_CHECK,
    CV_EML_RELATIONAL_CHECK,
    CV_EML_CHECK_CNT
} SfCvEmlCheckType;

#endif

#ifndef CV_SCRIPT_CHECK_TYPE_DEF
#define CV_SCRIPT_CHECK_TYPE_DEF

typedef enum
{
    CV_SCRIPT_FCN_CHECK = 0,
    CV_SCRIPT_IF_CHECK,
    CV_SCRIPT_FOR_CHECK,
    CV_SCRIPT_WHILE_CHECK,
    CV_SCRIPT_COND_CHECK,
    CV_SCRIPT_MCDC_CHECK,
    CV_SCRIPT_SWITCH_CHECK,
    CV_SCRIPT_TESTOBJECTIVE_CHECK,
    CV_SCRIPT_RELATIONAL_CHECK,
    CV_SCRIPT_CHECK_CNT
} SfCvScriptCheckType;

#endif

typedef enum
{
    MACHINE_OBJECT = 0
    , CHART_OBJECT = 1
    , TARGET_OBJECT = 2
    , INSTANCE_OBJECT = 3
    , STATE_OBJECT = 4
    , TRANSITION_OBJECT = 5
    , JUNCTION_OBJECT = 6
    , EVENT_OBJECT = 7
    , DATA_OBJECT = 8
    , SIMDATA_OBJECT = 9
    , NOTE_OBJECT = 10
    , UNDO_OBJECT = 11
    , ERROR_OBJECT = 12
    , NULL_OBJECT = 13
    , SCRIPT_OBJECT = 14
    , MESSAGE_OBJECT = 15
} SfDebugObjectType;

typedef enum
{
    STATE_ENTER_ENTRY_FUNCTION_TAG = 0
    , STATE_BEFORE_ENTRY_ACTION_TAG
    , STATE_DURING_ENTRY_ACTION_TAG
    , STATE_AFTER_ENTRY_ACTION_TAG

    , STATE_ENTER_DURING_FUNCTION_TAG
    , STATE_BEFORE_DURING_ACTION_TAG
    , STATE_DURING_DURING_ACTION_TAG
    , STATE_AFTER_DURING_ACTION_TAG

    , STATE_ENTER_EXIT_FUNCTION_TAG
    , STATE_BEFORE_EXIT_ACTION_TAG
    , STATE_DURING_EXIT_ACTION_TAG
    , STATE_AFTER_EXIT_ACTION_TAG

    , STATE_ENTRY_COVERAGE_TAG
    , STATE_DURING_COVERAGE_TAG
    , STATE_EXIT_COVERAGE_TAG
    , STATE_ACTIVE_TAG
    , STATE_INACTIVE_TAG
    , FUNCTION_ACTIVE_TAG
    , FUNCTION_INACTIVE_TAG
    , TOTAL_STATE_TAGS

    , TRANSITION_BEFORE_PROCESSING_TAG = 0
    , TRANSITION_WHEN_VALID_TAG
    , TRANSITION_BEFORE_COND_ACTION_TAG
    , TRANSITION_DURING_COND_ACTION_TAG
    , TRANSITION_AFTER_COND_ACTION_TAG
    , TRANSITION_BEFORE_TRANS_ACTION_TAG
    , TRANSITION_DURING_TRANS_ACTION_TAG
    , TRANSITION_AFTER_TRANS_ACTION_TAG
    , TRANSITION_AFTER_PROCESSING_TAG
    , TRANSITION_ACTIVE_TAG
    , TRANSITION_INACTIVE_TAG
    , TRANSITION_TRIGGER_COVERAGE_TAG
    , TRANSITION_GUARD_COVERAGE_TAG
    , TRANSITION_TRANSITION_ACTION_COVERAGE_TAG
    , TRANSITION_CONDITION_ACTION_COVERAGE_TAG
    , TRANSITION_DECISION_COVERAGE_TAG
    , TOTAL_TRANSITION_TAGS

    , EVENT_ENTER_BROADCAST_FUNCTION_TAG = 0
    , EVENT_BEFORE_BROADCAST_TAG
    , EVENT_AFTER_BROADCAST_TAG
    , TOTAL_EVENT_TAGS

    , MESSAGE_SEND_BREAKPOINT_TAG = 0
    , MESSAGE_REMOVE_BREAKPOINT_TAG
    , TOTAL_MESSAGE_TAGS

    , CHART_ENTER_SFUNCTION_TAG = 0
    , CHART_ENTER_GATEWAY_FUNCTION_TAG
    , CHART_ENTER_ENTRY_FUNCTION_TAG
    , CHART_ENTER_DURING_FUNCTION_TAG
    , CHART_ENTER_EXIT_FUNCTION_TAG
    , CHART_RETURN_TO_SIMULINK_TAG
    , CHART_ACTIVE_TAG
    , TOTAL_CHART_TAGS

    , SCRIPT_LINE_TAG = 0
    , TOTAL_SCRIPT_TAGS

    , EXIT_OUT_OF_FUNCTION_TAG = 999
} SfDebugTagType;

typedef enum
{
    SFDB_OVERFLOW = 0,
    SFDB_DIVIDE_BY_ZERO = 1,
    SFDB_UNSAFE_ENUM = 2,
    SFDB_SATURATE = 3
} SfDebugOverflowType;

struct SfDebugInstanceStruct;

LIBMWSF_RUNTIME_API struct SfDebugInstanceStruct* sf_debug_create_debug_instance_struct(void);

LIBMWSF_RUNTIME_API unsigned int sf_debug_initialize_machine(struct SfDebugInstanceStruct* debugInstance, const char* machineName,
                                                             const char* targetName,
                                                             unsigned int isLibrary,
                                                             unsigned int totalChartCount,
                                                             unsigned int totalDataCount,
                                                             unsigned int totalEventCount,
                                                             unsigned int totalDataChangeEventCount);
LIBMWSF_RUNTIME_API unsigned int sf_debug_initialize_chart(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                           unsigned int chartFileNumber,
                                                           unsigned int totalStateCount,
                                                           unsigned int totalTransitionCount,
                                                           unsigned int totalMessageCount,
                                                           unsigned int totalDataCount,
                                                           unsigned int totalEventCount,
                                                           unsigned int totalDataChangeEventCount,
                                                           unsigned int totalStateEntryEventCount,
                                                           unsigned int totalStateExitEventCount,
                                                           unsigned int totalScriptNumberCount,
                                                           unsigned int* chartNumberPtr,
                                                           unsigned int* instanceNumberPtr,
                                                           void* simStructPtr);

LIBMWSF_RUNTIME_API unsigned int sf_debug_call(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                               unsigned int chartNumber,
                                               unsigned int instanceNumber,
                                               SfDebugObjectType parentObjectType,
                                               SfDebugObjectType objectType,
                                               SfDebugTagType tagType,
                                               unsigned int objectNumber,
                                               unsigned int activeEventNumber,
                                               int optionalInteger,
                                               void* optionalPointer,
                                               double simulationTime,
                                               unsigned int retValue);

LIBMWSF_RUNTIME_API void eml_debug_line_call(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                             unsigned int chartNumber,
                                             unsigned int instanceNumber,
                                             unsigned int objectNumber,
                                             unsigned int activeEventNumber,
                                             int optionalInteger,
                                             double simulationTime,
                                             int isScript);

LIBMWSF_RUNTIME_API unsigned int cv_eval_point(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                               unsigned int chartNumber,
                                               unsigned int instanceNumber,
                                               SfDebugObjectType objectType,
                                               unsigned int objectNumber,
                                               unsigned int objectIndex,
                                               unsigned int retValue);

LIBMWSF_RUNTIME_API int cv_eml_eval(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                    unsigned int chartNumber,
                                    unsigned int instanceNumber,
                                    SfCvEmlCheckType checkType,
                                    unsigned int objectNumber,
                                    unsigned int isState,
                                    unsigned int objectIndex,
                                    int retValue);
LIBMWSF_RUNTIME_API int cv_eval_relational(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                           unsigned int chartNumber,
                                           unsigned int instanceNumber,
                                           unsigned int objectType,
                                           unsigned int objectNumber,
                                           unsigned int objectIndex,
                                           double lhsv,
                                           double rhsv,
                                           int relationalEps,
                                           unsigned int op,
                                           int retValue);

LIBMWSF_RUNTIME_API void cv_eval_basic_block(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                             unsigned int chartNumber,
                                             unsigned int instanceNumber,
                                             unsigned int objectType,
                                             unsigned int objectNumber,
                                             unsigned int objectIndex);

LIBMWSF_RUNTIME_API void cv_eml_init_script(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                            unsigned int chartNumber,
                                            unsigned int instanceNumber,
                                            unsigned int stateNumber,
                                            unsigned int isState,
                                            unsigned int fcnCnt,
                                            unsigned int blockCnt,
                                            unsigned int ifCnt,
                                            unsigned int testobjectiveCnt,
                                            unsigned int saturationCnt,
                                            unsigned int switchCnt,
                                            unsigned int forCnt,
                                            unsigned int whileCnt,
                                            unsigned int condCnt,
                                            unsigned int mcdcCnt);

LIBMWSF_RUNTIME_API void cv_eml_init_fcn(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                         unsigned int chartNumber,
                                         unsigned int instanceNumber,
                                         unsigned int stateNumber,
                                         unsigned int fcnIdx,
                                         const char* name,
                                         int charStart,
                                         int charExprEnd,
                                         int charEnd);

LIBMWSF_RUNTIME_API void cv_eml_init_basic_block(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                 unsigned int chartNumber,
                                                 unsigned int instanceNumber,
                                                 unsigned int stateNumber,
                                                 unsigned int fcnIdx,
                                                 int charStart,
                                                 int charExprEnd,
                                                 int charEnd);

LIBMWSF_RUNTIME_API void cv_eml_init_testobjective(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                   unsigned int chartNumber,
                                                   unsigned int instanceNumber,
                                                   unsigned int stateOrTransNumber,
                                                   unsigned int isState,
                                                   unsigned int fcnIdx,
                                                   const char* name,
                                                   int charStart,
                                                   int charExprEnd,
                                                   int charEnd);
LIBMWSF_RUNTIME_API void cv_eml_init_saturation(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                unsigned int chartNumber,
                                                unsigned int instanceNumber,
                                                unsigned int stateOrTransNumber,
                                                unsigned int isState,
                                                unsigned int fcnIdx,
                                                int charStart,
                                                int charExprEnd,
                                                int charEnd);
LIBMWSF_RUNTIME_API void cv_eml_init_if(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                        unsigned int chartNumber,
                                        unsigned int instanceNumber,
                                        unsigned int stateOrTransNumber,
                                        unsigned int isState,
                                        unsigned int ifIdx,
                                        int charStart,
                                        int charExprEnd,
                                        int charElseStart,
                                        int charEnd);

LIBMWSF_RUNTIME_API void cv_eml_init_for(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                         unsigned int chartNumber,
                                         unsigned int instanceNumber,
                                         unsigned int stateOrTransNumber,
                                         unsigned int isState,
                                         unsigned int ifIdx,
                                         int charStart,
                                         int charExprEnd,
                                         int charEnd);

LIBMWSF_RUNTIME_API void cv_eml_init_while(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                           unsigned int chartNumber,
                                           unsigned int instanceNumber,
                                           unsigned int stateOrTransNumber,
                                           unsigned int isState,
                                           unsigned int ifIdx,
                                           int charStart,
                                           int charExprEnd,
                                           int charEnd);

LIBMWSF_RUNTIME_API void cv_eml_init_mcdc(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                          unsigned int chartNumber,
                                          unsigned int instanceNumber,
                                          unsigned int stateOrTransNumber,
                                          unsigned int isState,
                                          unsigned int mcdcIdx,
                                          int charStart,
                                          int charEnd,
                                          unsigned int condCnt,
                                          unsigned int firstCondIdx,
                                          int* condStart,
                                          int* condEnd,
                                          unsigned int pfxLength,
                                          int* pfixExpr);

LIBMWSF_RUNTIME_API void cv_eml_init_relational(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                unsigned int chartNumber,
                                                unsigned int instanceNumber,
                                                unsigned int stateOrTransNumber,
                                                unsigned int isState,
                                                unsigned int objIdx,
                                                int charStart,
                                                int charEnd,
                                                int relationalEps,
                                                int relationalOp);
LIBMWSF_RUNTIME_API void cv_eml_init_switch(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                            unsigned int chartNumber,
                                            unsigned int instanceNumber,
                                            unsigned int stateOrTransNumber,
                                            unsigned int isState,
                                            unsigned int switchIdx,
                                            int charStart,
                                            int charExprEnd,
                                            int charEnd,
                                            unsigned int caseCnt,
                                            int* caseStart,
                                            int* caseExprEnd);

LIBMWSF_RUNTIME_API int cv_script_eval(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                       unsigned int chartNumber,
                                       unsigned int instanceNumber,
                                       SfCvScriptCheckType checkType,
                                       unsigned int objectNumber,
                                       unsigned int objectIndex,
                                       int retValue);

LIBMWSF_RUNTIME_API void cv_script_init_script(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                               unsigned int chartNumber,
                                               unsigned int instanceNumber,
                                               unsigned int stateNumber,
                                               unsigned int fcnCnt,
                                               unsigned int blockCnt,
                                               unsigned int ifCnt,
                                               unsigned int testobjectiveCnt,
                                               unsigned int saturationCnt,
                                               unsigned int switchCnt,
                                               unsigned int forCnt,
                                               unsigned int whileCnt,
                                               unsigned int condCnt,
                                               unsigned int mcdcCnt);

LIBMWSF_RUNTIME_API void cv_script_init_fcn(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                            unsigned int chartNumber,
                                            unsigned int instanceNumber,
                                            unsigned int stateNumber,
                                            unsigned int fcnIdx,
                                            const char* name,
                                            int charStart,
                                            int charExprEnd,
                                            int charEnd);

LIBMWSF_RUNTIME_API void cv_script_init_basic_block(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                    unsigned int chartNumber,
                                                    unsigned int instanceNumber,
                                                    unsigned int stateNumber,
                                                    unsigned int basicBlockIdx,
                                                    int charStart,
                                                    int charExprEnd,
                                                    int charEnd);

LIBMWSF_RUNTIME_API void cv_script_init_testobjective(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                      unsigned int chartNumber,
                                                      unsigned int instanceNumber,
                                                      unsigned int stateNumber,
                                                      unsigned int fcnIdx,
                                                      const char* name,
                                                      int charStart,
                                                      int charExprEnd,
                                                      int charEnd);

LIBMWSF_RUNTIME_API void cv_script_init_saturation(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                   unsigned int chartNumber,
                                                   unsigned int instanceNumber,
                                                   unsigned int stateNumber,
                                                   unsigned int fcnIdx,
                                                   int charStart,
                                                   int charExprEnd,
                                                   int charEnd);

LIBMWSF_RUNTIME_API void cv_script_init_if(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                           unsigned int chartNumber,
                                           unsigned int instanceNumber,
                                           unsigned int stateNumber,
                                           unsigned int ifIdx,
                                           int charStart,
                                           int charExprEnd,
                                           int charElseStart,
                                           int charEnd);

LIBMWSF_RUNTIME_API void cv_script_init_for(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                            unsigned int chartNumber,
                                            unsigned int instanceNumber,
                                            unsigned int stateNumber,
                                            unsigned int ifIdx,
                                            int charStart,
                                            int charExprEnd,
                                            int charEnd);

LIBMWSF_RUNTIME_API void cv_script_init_while(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                              unsigned int chartNumber,
                                              unsigned int instanceNumber,
                                              unsigned int stateNumber,
                                              unsigned int ifIdx,
                                              int charStart,
                                              int charExprEnd,
                                              int charEnd);

LIBMWSF_RUNTIME_API void cv_script_init_mcdc(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                             unsigned int chartNumber,
                                             unsigned int instanceNumber,
                                             unsigned int stateNumber,
                                             unsigned int mcdcIdx,
                                             int charStart,
                                             int charEnd,
                                             unsigned int condCnt,
                                             unsigned int firstCondIdx,
                                             int* condStart,
                                             int* condEnd,
                                             unsigned int pfxLength,
                                             int* pfixExpr);

LIBMWSF_RUNTIME_API void cv_script_init_relational(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                   unsigned int chartNumber,
                                                   unsigned int instanceNumber,
                                                   unsigned int stateNumber,
                                                   unsigned int objIdx,
                                                   int charStart,
                                                   int charEnd,
                                                   int relationalEps,
                                                   int relationalOp);

LIBMWSF_RUNTIME_API void cv_script_init_switch(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                               unsigned int chartNumber,
                                               unsigned int instanceNumber,
                                               unsigned int stateNumber,
                                               unsigned int switchIdx,
                                               int charStart,
                                               int charExprEnd,
                                               int charEnd,
                                               unsigned int caseCnt,
                                               int* caseStart,
                                               int* caseExprEnd);

LIBMWSF_RUNTIME_API void sf_debug_terminate(struct SfDebugInstanceStruct* debugInstance);
LIBMWSF_RUNTIME_API void sf_debug_store_current_state_configuration(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber, unsigned int chartNumber, unsigned int instanceNumber);
LIBMWSF_RUNTIME_API void sf_debug_restore_previous_state_configuration(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber, unsigned int chartNumber, unsigned int instanceNumber);
LIBMWSF_RUNTIME_API void sf_debug_restore_previous_state_configuration2(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber, unsigned int chartNumber, unsigned int instanceNumber);
LIBMWSF_RUNTIME_API void sf_debug_reset_current_state_configuration(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber, unsigned int chartNumber, unsigned int instanceNumber);
LIBMWSF_RUNTIME_API void sf_debug_check_for_state_inconsistency(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber, unsigned int chartNumber, unsigned int instanceNumber);

LIBMWSF_RUNTIME_API void sf_debug_set_honor_breakpoints(struct SfDebugInstanceStruct* debugInstance, unsigned int honorBreakpointsFlag);
LIBMWSF_RUNTIME_API void* sf_debug_get_chart_instance(struct SfDebugInstanceStruct* debugInstance, double instanceHandle);
LIBMWSF_RUNTIME_API unsigned int sf_debug_get_animation(struct SfDebugInstanceStruct* debugInstance);
LIBMWSF_RUNTIME_API void sf_debug_set_animation(struct SfDebugInstanceStruct* debugInstance, unsigned int animationFlag);
LIBMWSF_RUNTIME_API void sf_debug_animate(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber, unsigned int chartNumber, unsigned int instanceNumber);
LIBMWSF_RUNTIME_API void sf_debug_set_number_of_data_with_change_event_for_machine(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber, unsigned int index, unsigned int dataNumber);

LIBMWSF_RUNTIME_API void sf_debug_set_machine_event_thresholds(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                               unsigned int machineThreshold,
                                                               unsigned int dataChangeThreshold);
LIBMWSF_RUNTIME_API void sf_debug_set_machine_event_scope(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                          unsigned int eventNumber,
                                                          unsigned int eventScope);
LIBMWSF_RUNTIME_API void sf_debug_set_machine_data_thresholds(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                              unsigned int machineDataThreshold);
LIBMWSF_RUNTIME_API void sf_debug_set_machine_data_scope(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                         unsigned int dataNumber,
                                                         unsigned int dataScope);
LIBMWSF_RUNTIME_API void sf_debug_set_machine_data_name(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                        unsigned int dataNumber,
                                                        const char* dataName);
LIBMWSF_RUNTIME_API void sf_debug_set_machine_data_size(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                        unsigned int dataNumber,
                                                        unsigned int numDimensions,
                                                        const unsigned int* dimVector);
LIBMWSF_RUNTIME_API void sf_debug_set_machine_data_type(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                        unsigned int dataNumber,
                                                        unsigned int dataType,
                                                        unsigned int isComplex,
                                                        unsigned int isFixedPoint,
                                                        double bias,
                                                        double slope,
                                                        int exponent);
LIBMWSF_RUNTIME_API void sf_debug_set_chart_data_props(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                       unsigned int chartNumber,
                                                       unsigned int offsetDataNumber,
                                                       unsigned int dataScope,
                                                       unsigned int isInputData,
                                                       unsigned int isOutputData,
                                                       const char* dataName);
LIBMWSF_RUNTIME_API void sf_debug_set_chart_data_compiled_props(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                                unsigned int chartNumber,
                                                                unsigned int instanceNumber,
                                                                unsigned int offsetDataNumber,
                                                                unsigned int dataType,
                                                                unsigned int numDimensions,
                                                                const unsigned int* dimVector,
                                                                unsigned int isFixedPoint,
                                                                unsigned int isSigned,
                                                                int wordLength,
                                                                double bias,
                                                                double slope,
                                                                int exponent,
                                                                unsigned int isComplex,
                                                                MexFcnForType mexOutFcn,
                                                                MexInFcnForType mexInFcn);
LIBMWSF_RUNTIME_API bool sf_debug_check_fi_license(void);

LIBMWSF_RUNTIME_API void sf_debug_symbol_scope_push(struct SfDebugInstanceStruct* debugInstance, unsigned int nVars, unsigned int isNested);
LIBMWSF_RUNTIME_API void sf_debug_symbol_scope_push_eml(struct SfDebugInstanceStruct* debugInstance, unsigned int isNested, unsigned int nFamilies, unsigned int nVars, const char* famNames[], unsigned int* famToVarMap);
LIBMWSF_RUNTIME_API void sf_debug_symbol_scope_pop(struct SfDebugInstanceStruct* debugInstance);
LIBMWSF_RUNTIME_API void sf_debug_symbol_scope_add(struct SfDebugInstanceStruct* debugInstance, const char* name,
                                                   const void* dataPtr,
                                                   MexFcnForType mexOutFcn);

LIBMWSF_RUNTIME_API void sf_debug_symbol_scope_add_importable(struct SfDebugInstanceStruct* debugInstance, const char* name,
                                                              const void* dataPtr,
                                                              MexFcnForType mexOutFcn,
                                                              MexInFcnForType mexInFcn);

LIBMWSF_RUNTIME_API void sf_debug_symbol_scope_add_dyn(struct SfDebugInstanceStruct* debugInstance, const char* name,
                                                       const void* dataPtr,
                                                       const int*  sizePtr,
                                                       const void* elemsSizePtr,
                                                       int        isInitialized,
                                                       const void* mexOutFcn);

LIBMWSF_RUNTIME_API void sf_debug_symbol_scope_add_dyn_importable(struct SfDebugInstanceStruct* debugInstance, const char* name,
                                                                  const void* dataPtr,
                                                                  const int*  sizePtr,
                                                                  const void* elemsSizePtr,
                                                                  int        isInitialized,
                                                                  const void* mexOutFcn,
                                                                  const void* mexInFcn);

LIBMWSF_RUNTIME_API void sf_debug_symbol_scope_add_eml(struct SfDebugInstanceStruct* debugInstance, const void* dataPtr,
                                                       int           familyIdx,
                                                       MexFcnForType mexOutFcn);

LIBMWSF_RUNTIME_API void sf_debug_symbol_scope_add_eml_importable(struct SfDebugInstanceStruct* debugInstance, const void*     dataPtr,
                                                                  int             familyIdx,
                                                                  MexFcnForType   mexOutFcn,
                                                                  MexInFcnForType mexInFcn);

LIBMWSF_RUNTIME_API void sf_debug_symbol_scope_add_eml_dyn(struct SfDebugInstanceStruct* debugInstance, const void* dataPtr,
                                                           const int*  sizePtr,
                                                           const void* elemsSizePtr,
                                                           int        isInitialized,
                                                           int         familyIdx,
                                                           const void* mexOutFcn);

LIBMWSF_RUNTIME_API void sf_debug_symbol_scope_add_eml_dyn_importable(struct SfDebugInstanceStruct* debugInstance, const void* dataPtr,
                                                                      const int*  sizePtr,
                                                                      const void* elemsSizePtr,
                                                                      int         isInitialized,
                                                                      int         familyIdx,
                                                                      const void* mexOutFcn,
                                                                      const void* mexInFcn);

LIBMWSF_RUNTIME_API void sf_debug_symbol_scope_add_eml_dyn_emx(struct SfDebugInstanceStruct* debugInstance, const void* dataPtr,
    const int*  sizePtr,
    const void* elemsSizePtr,
    int        isInitialized,
    int         familyIdx,
    const void* mexOutFcn,
    const void* emxPtr,
    bool isFullEmx);

LIBMWSF_RUNTIME_API void sf_debug_symbol_scope_add_eml_dyn_emx_importable(struct SfDebugInstanceStruct* debugInstance, const void* dataPtr,
    const int*  sizePtr,
    const void* elemsSizePtr,
    int         isInitialized,
    int         familyIdx,
    const void* mexOutFcn,
    const void* mexInFcn,
    const void* emxPtr,
    bool isFullEmx);

LIBMWSF_RUNTIME_API void sf_debug_symbol_switch(struct SfDebugInstanceStruct* debugInstance, unsigned int familyIdx, unsigned int varIdx);

LIBMWSF_RUNTIME_API void sf_debug_symbol_scope_add_verbose(struct SfDebugInstanceStruct* debugInstance, const char* name,
                                                           int type,
                                                           unsigned int numDims,
                                                           unsigned int m,
                                                           unsigned int n,
                                                           unsigned int isFixpt,
                                                           double slope,
                                                           int exponent,
                                                           double bias,
                                                           unsigned int isSigned,
                                                           int wordLength,
                                                           unsigned int isComplex,
                                                           int indexScheme,
                                                           mxArray** pfm,
                                                           const void* basePtr);
LIBMWSF_RUNTIME_API void sf_debug_set_chart_event_scope(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                        unsigned int chartNumber,
                                                        unsigned int offsetEventNumber,
                                                        unsigned int eventScope);
LIBMWSF_RUNTIME_API void sf_debug_set_machine_data_value_ptr(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                             unsigned int dataNumber,
                                                             void* dataValuePtr);
LIBMWSF_RUNTIME_API void sf_debug_set_instance_data_value_ptr(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                              unsigned int chartNumber,
                                                              unsigned int instanceNumber,
                                                              unsigned int offsetDataNumber,
                                                              void* dataValuePtr,
                                                              void* dataSizePtr);
LIBMWSF_RUNTIME_API void sf_debug_unset_instance_data_value_ptr(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                                unsigned int chartNumber,
                                                                unsigned int instanceNumber,
                                                                unsigned int dataNumber);

LIBMWSF_RUNTIME_API void sf_debug_set_number_of_data_with_change_event_for_chart(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                                                 unsigned int chartNumber,
                                                                                 unsigned int index,
                                                                                 unsigned int dataNumber);

LIBMWSF_RUNTIME_API void sf_debug_set_number_of_states_with_entry_event_for_chart(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                                                  unsigned int chartNumber,
                                                                                  unsigned int index,
                                                                                  unsigned int stateNumber);

LIBMWSF_RUNTIME_API void sf_debug_set_number_of_states_with_exit_event_for_chart(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                                                 unsigned int chartNumber,
                                                                                 unsigned int index,
                                                                                 unsigned int stateNumber);

LIBMWSF_RUNTIME_API void sf_debug_set_chart_disable_implicit_casting(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                                     unsigned int chartNumber,
                                                                     unsigned int disableImplicitCasting);
LIBMWSF_RUNTIME_API void sf_debug_set_chart_event_thresholds(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                             unsigned int chartNumber,
                                                             unsigned int dataChangeThreshold,
                                                             unsigned int stateEntryThreshold,
                                                             unsigned int stateExitThreshold);

LIBMWSF_RUNTIME_API void sf_debug_set_chart_state_info(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                       unsigned int chartNumber,
                                                       unsigned int stateNumber,
                                                       unsigned int decomposition,
                                                       unsigned int type);
LIBMWSF_RUNTIME_API void sf_debug_set_chart_decomposition(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                          unsigned int chartNumber,
                                                          unsigned int decomposition);
LIBMWSF_RUNTIME_API void sf_debug_set_chart_substate_count(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                           unsigned int chartNumber,
                                                           unsigned int substateCount);
LIBMWSF_RUNTIME_API void sf_debug_set_chart_substate_index(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                           unsigned int chartNumber,
                                                           unsigned int substateIndex,
                                                           unsigned int substateNumber);
LIBMWSF_RUNTIME_API void sf_debug_set_chart_state_substate_count(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                                 unsigned int chartNumber,
                                                                 unsigned int stateNumber,
                                                                 unsigned int substateCount);
LIBMWSF_RUNTIME_API void sf_debug_set_chart_state_substate_index(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                                 unsigned int chartNumber,
                                                                 unsigned int stateNumber,
                                                                 unsigned int subStateIndex,
                                                                 unsigned int subStateNumber);
LIBMWSF_RUNTIME_API void sf_debug_read_before_write_error(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                          unsigned int chartNumber,
                                                          unsigned int instanceNumber,
                                                          unsigned int dataNumber);

LIBMWSF_RUNTIME_API double sf_debug_data_range_error_wrapper_min_max(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                                     unsigned int chartNumber,
                                                                     unsigned int instanceNumber,
                                                                     unsigned int dataNumber,
                                                                     double dataValue,
                                                                     double dataMin,
                                                                     double dataMax,
                                                                     unsigned int ssIdOfSourceObject, int offsetInSourceObject, int lengthInSourceObject);
LIBMWSF_RUNTIME_API double sf_debug_data_range_error_wrapper_min(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                                 unsigned int chartNumber,
                                                                 unsigned int instanceNumber,
                                                                 unsigned int dataNumber,
                                                                 double dataValue,
                                                                 double dataMin,
                                                                 unsigned int ssIdOfSourceObject, int offsetInSourceObject, int lengthInSourceObject);
LIBMWSF_RUNTIME_API double sf_debug_data_range_error_wrapper_max(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                                 unsigned int chartNumber,
                                                                 unsigned int instanceNumber,
                                                                 unsigned int dataNumber,
                                                                 double dataValue,
                                                                 double dataMax,
                                                                 unsigned int ssIdOfSourceObject, int offsetInSourceObject, int lengthInSourceObject);

LIBMWSF_RUNTIME_API int sf_debug_data_array_bounds_error_check(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                               unsigned int chartNumber,
                                                               unsigned int instanceNumber,
                                                               unsigned int dataNumber,
                                                               int indexValue,
                                                               int firstIndex,
                                                               int lastIndex,
                                                               int dimension,
                                                               int boundsCheckKind);

LIBMWSF_RUNTIME_API void sf_debug_size_eq_check_1d(struct SfDebugInstanceStruct* debugInstance, int dim1, int dim2);
LIBMWSF_RUNTIME_API void sf_debug_size_eq_check_nd(struct SfDebugInstanceStruct* debugInstance, const int* dims1, const int* dims2, int nDims);
LIBMWSF_RUNTIME_API void sf_debug_dim_size_eq_check(struct SfDebugInstanceStruct* debugInstance, int dim1, int dim2, int index);
LIBMWSF_RUNTIME_API void sf_debug_dim_size_geq_check(struct SfDebugInstanceStruct* debugInstance, int dim1, int dim2, int index);
LIBMWSF_RUNTIME_API void sf_debug_sub_assign_size_check_1d(struct SfDebugInstanceStruct* debugInstance, int dim1, int dim2);
LIBMWSF_RUNTIME_API void sf_debug_sub_assign_size_check_nd(struct SfDebugInstanceStruct* debugInstance, const int* dims1, int nDims1, const int* dims2, int nDims2);
LIBMWSF_RUNTIME_API void sf_debug_for_loop_vector_check(struct SfDebugInstanceStruct* debugInstance, double start, double step, double end, mxClassID classID, int n);
LIBMWSF_RUNTIME_API void sf_debug_runtime_error_msgid(struct SfDebugInstanceStruct* debugInstance, const char* msgID);

LIBMWSF_RUNTIME_API void sf_debug_data_runtime_size_mismatch_error_check(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                                         unsigned int chartNumber,
                                                                         unsigned int instanceNumber,
                                                                         const char* arrayName1,
                                                                         const char* arrayName2,
                                                                         unsigned int dimension,
                                                                         int size1,
                                                                         int size2);

#ifdef INT_TYPE_64_IS_SUPPORTED
LIBMWSF_RUNTIME_API int64_T sf_debug_eml_data_array_bounds_error_check_int64(struct SfDebugInstanceStruct* debugInstance,
                                                                             unsigned int machineNumber,
                                                                             unsigned int chartNumber,
                                                                             unsigned int instanceNumber,
                                                                             const char* dataName,
                                                                             int64_T indexValue,
                                                                             int firstIndex,
                                                                             int lastIndex,
                                                                             int dimension,
                                                                             int boundsCheckKind);
#endif

LIBMWSF_RUNTIME_API int sf_debug_eml_data_array_bounds_error_check(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                                   unsigned int chartNumber,
                                                                   unsigned int instanceNumber,
                                                                   const char* dataName,
                                                                   int indexValue,
                                                                   int firstIndex,
                                                                   int lastIndex,
                                                                   int dimension,
                                                                   int boundsCheckKind);

LIBMWSF_RUNTIME_API double sf_debug_integer_check(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                  unsigned int chartNumber,
                                                  unsigned int instanceNumber,
                                                  const char* dataName,
                                                  double value);
LIBMWSF_RUNTIME_API double sf_debug_not_nan_check(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                  unsigned int chartNumber,
                                                  unsigned int instanceNumber,
                                                  const char* dataName,
                                                  double value);
LIBMWSF_RUNTIME_API double sf_debug_non_negative_check(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                       unsigned int chartNumber,
                                                       unsigned int instanceNumber,
                                                       const char* dataName,
                                                       double value);

LIBMWSF_RUNTIME_API unsigned int sf_debug_temporal_threshold(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                             unsigned int chartNumber,
                                                             unsigned int instanceNumber,
                                                             unsigned int threshold,
                                                             unsigned int width,
                                                             SfDebugObjectType objectType,
                                                             unsigned int  objectNumber);

#define SF_DEBUG_CAST_TO_TYPE_PROTO(TYPE,type)                          \
    LIBMWSF_RUNTIME_API type sf_debug_cast_to_ ## type (struct SfDebugInstanceStruct* debugInstance, \
                                                        unsigned int machineNumber, \
                                                        unsigned int chartNumber, \
                                                        unsigned int instanceNumber, \
                                                        double dataValue, \
                                                        unsigned int srcType, \
                                                        int stateOrTransId);

SF_DEBUG_CAST_TO_TYPE_PROTO(SF_SINGLE, real32_T)
SF_DEBUG_CAST_TO_TYPE_PROTO(SF_INT8, int8_T)
SF_DEBUG_CAST_TO_TYPE_PROTO(SF_UINT8, uint8_T)
SF_DEBUG_CAST_TO_TYPE_PROTO(SF_INT16, int16_T)
SF_DEBUG_CAST_TO_TYPE_PROTO(SF_UINT16, uint16_T)
SF_DEBUG_CAST_TO_TYPE_PROTO(SF_INT32, int32_T)
SF_DEBUG_CAST_TO_TYPE_PROTO(SF_UINT32, uint32_T)
#undef SF_DEBUG_CAST_TO_TYPE_PROTO

LIBMWSF_RUNTIME_API double sf_debug_data_range_wrapper(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                       unsigned int chartNumber,
                                                       unsigned int instanceNumber,
                                                       unsigned int dataNumber,
                                                       double dataValue);

LIBMWSF_RUNTIME_API void sf_debug_cv_init_saturation(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                     unsigned int chartNumber,
                                                     unsigned int instanceNumber,
                                                     unsigned int objectType,
                                                     unsigned int transitionNumber,
                                                     unsigned int satCnt,
                                                     unsigned int* txtStartIdx,
                                                     unsigned int* txtEndIdx);

LIBMWSF_RUNTIME_API void sf_debug_cv_init_testobjective(unsigned int machineNumber,
                                                        unsigned int chartNumber,
                                                        unsigned int instanceNumber,
                                                        SfDebugObjectType objectType,
                                                        unsigned int transitionNumber,
                                                        unsigned int testobjectiveCnt,
                                                        unsigned int* txtStartIdx,
                                                        unsigned int* txtEndIdx);

LIBMWSF_RUNTIME_API unsigned int cv_eval_saturation(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                    unsigned int chartNumber,
                                                    unsigned int instanceNumber,
                                                    unsigned int objectType,
                                                    unsigned int objectNumber,
                                                    unsigned int satIndex,
                                                    unsigned int isNeg,
                                                    unsigned int retValue);

LIBMWSF_RUNTIME_API void  cv_saturation_accum(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                              unsigned int chartNumber,
                                              unsigned int instanceNumber,
                                              unsigned int objectType,
                                              unsigned int objectNumber,
                                              unsigned int satIndex,
                                              unsigned int accumMode);

LIBMWSF_RUNTIME_API void cv_eval_testobjective(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                               unsigned int chartNumber,
                                               unsigned int instanceNumber,
                                               SfDebugObjectType objectType,
                                               unsigned int objectNumber,
                                               unsigned int objectIndex,
                                               unsigned int retValue);

LIBMWSF_RUNTIME_API unsigned int cv_eval_assignment(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                    unsigned int chartNumber,
                                                    unsigned int instanceNumber,
                                                    SfDebugObjectType objectType,
                                                    unsigned int objectNumber,
                                                    unsigned int objectIndex,
                                                    unsigned int retValue);

LIBMWSF_RUNTIME_API void sf_debug_cv_init_trans(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                unsigned int chartNumber,
                                                unsigned int instanceNumber,
                                                unsigned int transitionNumber,
                                                int predicateCnt,
                                                unsigned int* txtStartIdx,
                                                unsigned int* txtEndIdx,
                                                unsigned int postFixPredicateTreeCount,
                                                int* postFixPredicateTree);

LIBMWSF_RUNTIME_API void sf_debug_cv_init_assignment(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                unsigned int chartNumber,
                                                unsigned int instanceNumber,
                                                unsigned int objectType,
                                                unsigned int objectNumber,
                                                unsigned int numAssignments,
                                                int* assignmentKeys,
                                                unsigned int* assignTxtStartIdx,
                                                unsigned int* assignTxtEndIdx,
                                                unsigned int numTotalConditions,
                                                int* conditionKeys,
                                                unsigned int* condTxtStartIdx,
                                                unsigned int* condTxtEndIdx,
                                                unsigned int postFixPredicateTreeConcatCount,
                                                int* postFixPredicateTreeConcat,
                                                unsigned int* condTextIdxOffsets,
                                                unsigned int* pptIdxOffsets);

LIBMWSF_RUNTIME_API void sf_debug_cv_init_chart(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                unsigned int chartNumber,
                                                unsigned int instanceNumber,
                                                unsigned int substateCnt,
                                                bool hasDuringSwitch,
                                                bool hasExitSwitch,
                                                bool hasHistSwitch);

LIBMWSF_RUNTIME_API void sf_debug_cv_init_state(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                unsigned int chartNumber,
                                                unsigned int instanceNumber,
                                                unsigned int stateNumber,
                                                unsigned int substateCnt,
                                                bool hasDuringSwitch,
                                                bool hasExitSwitch,
                                                bool hasHistSwitch,
                                                unsigned int onDecCnt,
                                                unsigned int* decStartInd,
                                                unsigned int* decEndInd);

LIBMWSF_RUNTIME_API void sf_debug_overflow_detection(struct SfDebugInstanceStruct* debugInstance, SfDebugOverflowType type, unsigned int instanceNumber,
                                                     unsigned int ssId, int offset, int length);

LIBMWSF_RUNTIME_API void sf_debug_add_script_breakpoints(struct SfDebugInstanceStruct* debugInstance, unsigned int scriptId,
                                                         const mxArray* mx_breakpoints);

LIBMWSF_RUNTIME_API void sf_debug_clear_script_breakpoints(struct SfDebugInstanceStruct* debugInstance, unsigned int scriptId,
                                                           const mxArray* mx_breakpoints);

LIBMWSF_RUNTIME_API void sf_debug_query_script_breakpoints(struct SfDebugInstanceStruct* debugInstance, mxArray* plhs[],
                                                           unsigned int scriptId,
                                                           const mxArray* mx_breakpoints);

LIBMWSF_RUNTIME_API void sf_debug_get_eml_script_id(mxArray* plhs[],
                                                    const char* scriptName);

LIBMWSF_RUNTIME_API unsigned int sf_debug_get_script_id(const char* path);
LIBMWSF_RUNTIME_API void sf_debug_set_script_translation(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                         unsigned int chartNumber,
                                                         unsigned int instanceNumber,
                                                         unsigned int scriptNumber,
                                                         unsigned int scriptId);
LIBMWSF_RUNTIME_API unsigned int sf_debug_register_message_post_receive_info(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                                             unsigned int chartNumber,
                                                                             unsigned int instanceNumber,
                                                                             unsigned int messagePortNumber,
                                                                             double simulationTime,
                                                                             unsigned int isSend);

LIBMWSF_RUNTIME_API void sf_debug_cv_init_testobjectives(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                         unsigned int chartNumber,
                                                         unsigned int instanceNumber,
                                                         SfDebugObjectType objectType,
                                                         unsigned int objectNumber,
                                                         unsigned int testobjCnt,
                                                         unsigned int* txtStartIdx,
                                                         unsigned int* txtEndIdx);

LIBMWSF_RUNTIME_API void sf_debug_cv_init_relationalop(struct SfDebugInstanceStruct* debugInstance, unsigned int machineNumber,
                                                       unsigned int chartNumber,
                                                       unsigned int instanceNumber,
                                                       unsigned int objectType,
                                                       unsigned int objectNumber,
                                                       unsigned int relopCnt,
                                                       unsigned int* txtStartIdx,
                                                       unsigned int* txtEndIdx,
                                                       int* relationalEps,
                                                       int* relationalOp);

#endif
