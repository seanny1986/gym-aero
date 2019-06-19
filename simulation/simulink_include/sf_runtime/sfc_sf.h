/* Copyright 2013-2018 The MathWorks, Inc. */

#ifndef _sf_runtime_sfc_sf_h_
#define _sf_runtime_sfc_sf_h_

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#ifndef FORCE_S_FUNCTION_LEVEL_ONE
#define S_FUNCTION_LEVEL 2
#endif

#if S_FUNCTION_LEVEL == 2
#define MDL_RTW
#define MDL_INITIALIZE_CONDITIONS
#define MDL_SET_WORK_WIDTHS
#define MDL_DISABLE
#define MDL_ENABLE
#define MDL_START
#define MDL_EXT_MODE_EXEC
#define MDL_PROCESS_PARAMETERS
#define MDL_ZERO_CROSSINGS
#define MDL_DERIVATIVES
#define MDL_SIM_STATE
#define MDL_UPDATE
#endif

#define JIT_CONTROL_C_EXCEPTION "JITControlCException"

#if defined(S_FUNCTION_NAME)
#include "mwmathutil.h"
#endif

#include "sf_runtime_spec.h"

#include "mex.h"

#if !defined(S_FUNCTION_NAME)

typedef struct SimStruct_tag SimStruct;
typedef int_T DTypeId;
typedef struct _ssFcnCallExecArgs_tag _ssFcnCallExecArgs;

#else

#include "simstruc.h"
#include "simtarget/slSimTgtClientServerAPIBridge.h"

#endif

#include "ChartInfoStruct.h"

typedef void (*MdlMethodFcnPtr)(SimStruct* S);
LIBMWSF_RUNTIME_API void callMdlMethodAndReportError(MdlMethodFcnPtr callee, SimStruct* S);

LIBMWSF_RUNTIME_API unsigned int sf_rtw_info_uint_prop(const char* spec,
                                                       mxArray* infoStruct,
                                                       unsigned int chartFileNumber,
                                                       const char* uintPropName);
LIBMWSF_RUNTIME_API unsigned int sf_update_buildInfo(SimStruct* S,
                                                     const char* spec,
                                                     mxArray* infoStruct,
                                                     unsigned int chartFileNumber);
LIBMWSF_RUNTIME_API unsigned int sf_is_chart_inlinable(const char* spec,
                                                       mxArray* infoStruct,
                                                       unsigned int chartFileNumber);
LIBMWSF_RUNTIME_API void sf_mark_output_events_with_multiple_callers(SimStruct* S,
                                                                     const char* spec,
                                                                     mxArray* infoStruct,
                                                                     unsigned int chartFileNumber,
                                                                     unsigned numOutputFcnCalls);
LIBMWSF_RUNTIME_API void sf_mark_chart_reusable_outputs(SimStruct* S,
                                                        const char* spec,
                                                        mxArray* infoStruct,
                                                        unsigned int chartFileNumber,
                                                        unsigned numOutputData);
LIBMWSF_RUNTIME_API void sf_mark_chart_expressionable_inputs(SimStruct* S,
                                                             const char* spec,
                                                             mxArray* infoStruct,
                                                             unsigned int chartFileNumber,
                                                             unsigned numInputData);
LIBMWSF_RUNTIME_API unsigned int sim_mode_is_rtw_gen(SimStruct* S);
LIBMWSF_RUNTIME_API unsigned int sim_mode_is_external(SimStruct* S);
LIBMWSF_RUNTIME_API unsigned int sim_mode_is_modelref_sim(SimStruct* S);

LIBMWSF_RUNTIME_API DTypeId sf_get_fixpt_data_type_id(SimStruct* S,
                                                      unsigned int nBits,
                                                      bool isSigned,
                                                      int exponent,
                                                      double slope,
                                                      double bias);
LIBMWSF_RUNTIME_API DTypeId sf_get_enum_data_type_id(SimStruct* S, const char* enumTypeName);
LIBMWSF_RUNTIME_API DTypeId sf_get_param_data_type_id(SimStruct* S, int idx);
LIBMWSF_RUNTIME_API unsigned int sf_mex_listen_for_ctrl_c(SimStruct* S);
LIBMWSF_RUNTIME_API const mxArray* sf_mex_get_sfun_param(SimStruct* S,
                                                         unsigned int paramIndex,
                                                         unsigned int bStrict);
LIBMWSF_RUNTIME_API void* sf_get_runtime_param_data(void* SVoid, int_T paramIndex);
LIBMWSF_RUNTIME_API void sf_call_output_fcn_call(SimStruct* S,
                                                 int eventIndex,
                                                 const char* eventName,
                                                 int checkForInitialization);
LIBMWSF_RUNTIME_API mxArray* sf_load_rtw_optimization_info(const char* machineName,
                                                           const char* mainMachineName);
LIBMWSF_RUNTIME_API void sf_clear_rtw_identifier(SimStruct* S);
LIBMWSF_RUNTIME_API void sf_write_symbol_mapping(SimStruct* S,
                                                 mxArray* infoStruct,
                                                 unsigned int chartFileNumber);
LIBMWSF_RUNTIME_API void sf_call_output_fcn_enable(SimStruct* S,
                                                   int eventIndex,
                                                   const char* eventName,
                                                   int checkForInitialization);
LIBMWSF_RUNTIME_API void sf_call_output_fcn_disable(SimStruct* S,
                                                    int eventIndex,
                                                    const char* eventName,
                                                    int checkForInitialization);
LIBMWSF_RUNTIME_API mxArray* sf_set_rtw_dwork_info(SimStruct* S,
                                               const char* spec,
                                               mxArray* infoStruct,
                                               unsigned chartFileNumber);
LIBMWSF_RUNTIME_API mxArray* sf_get_dwork_info_from_mat_file(SimStruct* S,
                                                             const char* spec,
                                                             mxArray* infoStruct,
                                                             unsigned chartFileNumber,
                                                             const char* propName);
LIBMWSF_RUNTIME_API void sf_set_encoded_dwork_info(SimStruct* S,
                                                   const char* encodedDWorkInfo);

LIBMWSF_RUNTIME_API void sf_set_dwork_info(SimStruct* S, const mxArray* dWorkInfo);

LIBMWSF_RUNTIME_API SimStruct* sf_get_subchart_simstruct(SimStruct* S, const char* blockName);
LIBMWSF_RUNTIME_API mxArray* sf_get_sim_state_info(SimStruct* S);
LIBMWSF_RUNTIME_API void sf_set_sim_state(SimStruct* S, const mxArray* st);

typedef unsigned int (*GlobalMethodDispatcherFcnType)(SimStruct* s,
                                                      const char* machineName,
                                                      unsigned int bufLen,
                                                      const char* specStr,
                                                      int method,
                                                      void* data);

LIBMWSF_RUNTIME_API void sf_refresh_sfun_block_specialization(SimStruct* S);
LIBMWSF_RUNTIME_API mxArray* sf_get_sfun_block_specialization(SimStruct* S, bool refresh);
LIBMWSF_RUNTIME_API void call_sf_machine_global_method_dispatcher(
    SimStruct* S,
    int_T method,
    bool refreshSpec,
    GlobalMethodDispatcherFcnType fcnPtr);
LIBMWSF_RUNTIME_API void loadSimStructFunctionPointers(SimStruct* S);
LIBMWSF_RUNTIME_API int sf_get_output_port_reusable(SimStruct* S, int portNumber);
LIBMWSF_RUNTIME_API double sf_get_time(SimStruct* S);
LIBMWSF_RUNTIME_API double sf_get_start_time(SimStruct* S);
LIBMWSF_RUNTIME_API bool sf_is_first_init_cond(SimStruct* S);
LIBMWSF_RUNTIME_API bool isFirstTimeMDLStart(const SimStruct* S);

LIBMWSF_RUNTIME_API void sf_call_atomic_subchart_user_fcn(int ssIdNumber,
                                                          SimStruct* S,
                                                          void* ioVarPtrs);
LIBMWSF_RUNTIME_API void sf_call_atomic_subchart_auto_fcn(int fcnNumber,
                                                          SimStruct* S,
                                                          void* ioVarPtrs);
LIBMWSF_RUNTIME_API void sf_call_atomic_subchart_event_fcn(int fcnNumber,
                                                           int ssIdNumber,
                                                           SimStruct* S);
LIBMWSF_RUNTIME_API mxArray* sf_call_get_hover_data_for_msg_fcn(SimStruct* chartSimStruct,
                                                                int ssIdNumber);
LIBMWSF_RUNTIME_API mxArray* sf_get_data_value_from_jit_engine(SimStruct* S, int ssidNumber);

LIBMWSF_RUNTIME_API void sf_call_marshall_in_fcn(SimStruct* S,
                                                 const char* fcnInName,
                                                 void* var,
                                                 const char* varName,
                                                 const mxArray* value);
LIBMWSF_RUNTIME_API void sf_call_marshall_in_fcn_jitOff(SimStruct* S,
                                                        void* fcnIn,
                                                        void* var,
                                                        const char* varName,
                                                        const mxArray* value);
LIBMWSF_RUNTIME_API void sf_call_marshall_in_fcn_dynamic(SimStruct* S,
                                                         const char* fcnInName,
                                                         void* var,
                                                         void* var2,
                                                         void* var3,
                                                         const char* varName,
                                                         const mxArray* value);
LIBMWSF_RUNTIME_API void sf_call_marshall_in_fcn_dynamic_jitOff(SimStruct* S,
                                                                void* fcnIn,
                                                                void* data,
                                                                void* size,
                                                                const char* varName,
                                                                const mxArray* value);

LIBMWSF_RUNTIME_API int32_T sf_get_event_number(SimStruct* S);

LIBMWSF_RUNTIME_API mxArray* sf_call_marshall_out_fcn(SimStruct* S,
                                                      const char* fcnOutName,
                                                      void* var,
                                                      const char* varName);
LIBMWSF_RUNTIME_API mxArray* sf_call_marshall_out_fcn_jitOff(SimStruct* S,
                                                             void* fcnOut,
                                                             void* var,
                                                             const char* varName);

LIBMWSF_RUNTIME_API mxArray* sf_call_marshall_out_fcn_dynamic(SimStruct* S,
                                                              const char* fcnOutName,
                                                              void* var,
                                                              void* sizeFirst,
                                                              void* sizeSecond,
                                                              const char* varName);
LIBMWSF_RUNTIME_API mxArray* sf_call_marshall_out_fcn_dynamic_jitOff(SimStruct* S,
                                                                     void* fcnOut,
                                                                     void* var,
                                                                     void* sizeFirst);

LIBMWSF_RUNTIME_API bool sf_is_jit_enabled(SimStruct* S);
LIBMWSF_RUNTIME_API bool sf_is_legacy_debugger_on(SimStruct* S);
LIBMWSF_RUNTIME_API bool sf_is_debugger_on(SimStruct* S);
LIBMWSF_RUNTIME_API bool sf_force_animation(SimStruct* S);
LIBMWSF_RUNTIME_API bool sf_animate_after_update(SimStruct* S);
LIBMWSF_RUNTIME_API bool sf_force_animation_for_chart_instance(SimStruct* S);
LIBMWSF_RUNTIME_API bool sf_force_animation_for_chart_exec_at_init(SimStruct* S);

LIBMWSF_RUNTIME_API void sfcs_call_function_with_try_catch(void* fPtr,
                                                           SimStruct* S,
                                                           int32_T tid,
                                                           void* args);
LIBMWSF_RUNTIME_API void sfcs_update_catalog(SimStruct* S, const char* fcnName, void* fPtr);
LIBMWSF_RUNTIME_API void sfcs_update_catalog_with_sfcs_server_call(SimStruct* S,
                                                                   const char* fcnName);
LIBMWSF_RUNTIME_API void sfcs_call_server_fcn_in_chart(SimStruct* S,
                                                       int tid,
                                                       _ssFcnCallExecArgs* args);

LIBMWSF_RUNTIME_API void sfDoAnimationWrapper(SimStruct* S, bool forceAnimate, bool transAndJunc);
LIBMWSF_RUNTIME_API void setDataBrowseFcn(SimStruct* S, void* aFcn);
LIBMWSF_RUNTIME_API void setLegacyDebuggerFlag(SimStruct* S, const bool aFlag);
LIBMWSF_RUNTIME_API void setDebuggerFlag(SimStruct* S, const bool aFlag);

LIBMWSF_RUNTIME_API mxArray* sf_get_high_sim_state(SimStruct* S);
LIBMWSF_RUNTIME_API void sf_set_high_sim_state(SimStruct* S, const mxArray* st);

LIBMWSF_RUNTIME_API void sf_start_timer(SimStruct* S, int timerId, double delay);
LIBMWSF_RUNTIME_API void sf_cancel_timer(SimStruct* S, int timerId);
LIBMWSF_RUNTIME_API int sf_get_active_timer(SimStruct* S);

LIBMWSF_RUNTIME_API void sf_add_msg_trigger(SimStruct* S, int qType, int qId, int triggerId);
LIBMWSF_RUNTIME_API void sf_remove_msg_trigger(SimStruct* S, int qType, int qId, int triggerId);

LIBMWSF_RUNTIME_API void sf_set_chart_accesses_machine_info(SimStruct* S,
                                                            const char* spec,
                                                            mxArray* infoStruct,
                                                            unsigned int chartFileNumber);

LIBMWSF_RUNTIME_API void* sf_malloc(unsigned int sz);
LIBMWSF_RUNTIME_API void sf_free(void* p);
LIBMWSF_RUNTIME_API void* sf_memcpy(void* dest, const void* src, unsigned int sz);

void sf_check_msg_queue_for_overflow(SimStruct* S,
                                     int queueType,
                                     int queueId,
                                     int isSender,
                                     unsigned int ssIdSrcLoc,
                                     int offsetSrcLoc,
                                     int lengthSrcLoc,
                                     int overflowDiagnostic);

LIBMWSF_RUNTIME_API void sf_pre_call_check_for_send(SimStruct* S,
                                                    unsigned int ssIdSrcLoc,
                                                    int offsetSrcLoc,
                                                    int lengthSrcLoc,
                                                    int queueType,
                                                    int queueId,
                                                    int isSender,
                                                    int overflowDiagnostic);

LIBMWSF_RUNTIME_API void sf_pre_call_check_for_forward(SimStruct* S,
                                                       unsigned int ssIdSrcLoc,
                                                       int offsetSrcLoc,
                                                       int lengthSrcLoc,
                                                       bool isPopped,
                                                       bool isForwarded,
                                                       int queueType,
                                                       int queueId,
                                                       int isSender,
                                                       int overflowDiagnostic);

LIBMWSF_RUNTIME_API void sf_pre_call_check_for_discard(SimStruct* S,
                                                       unsigned int ssIdSrcLoc,
                                                       int offsetSrcLoc,
                                                       int lengthSrcLoc,
                                                       bool isPopped,
                                                       bool isForwarded);

LIBMWSF_RUNTIME_API void sf_throw_overflow_information_for_local_queue(SimStruct* S,
                                                                       unsigned int ssid,
                                                                       int queueIdx);

LIBMWSF_RUNTIME_API int sf_get_queue_overflow_diagnostic_option(SimStruct* S,
                                                                int queueType,
                                                                int queueId);

LIBMWSF_RUNTIME_API void sf_register_codegen_names_for_scoped_functions_defined_by_chart(
    SimStruct* S);

LIBMWSF_RUNTIME_API void sf_set_work_widths(SimStruct* S, const char* encStr);

LIBMWSF_RUNTIME_API void sf_populate_enabled_flags(SimStruct* S, bool* isEnabled);

LIBMWSF_RUNTIME_API void* getRuntimeInstanceInfoAsVoid(SimStruct* S);

LIBMWSF_RUNTIME_API void notify_msg_viewer_of_state_change(SimStruct* S,
                                                           bool isEntry,
                                                           unsigned int ssId);

LIBMWSF_RUNTIME_API void* sf_get_action_exec_block(SimStruct* S, const char* aName);
LIBMWSF_RUNTIME_API void sf_call_action_subsys_output(SimStruct* S, void* execBlockPtr);
LIBMWSF_RUNTIME_API void sf_call_action_subsys_update(SimStruct* S, void* execBlockPtr);
LIBMWSF_RUNTIME_API void sf_call_action_subsys_derivatives(SimStruct* S, void* execBlockPtr);
LIBMWSF_RUNTIME_API void sf_call_action_subsys_enable(SimStruct* S, void* execBlockPtr);
LIBMWSF_RUNTIME_API void sf_call_action_subsys_disable(SimStruct* S, void* execBlockPtr);
LIBMWSF_RUNTIME_API void sf_call_action_subsys_resetStates(SimStruct* S, void* execBlockPtr);
LIBMWSF_RUNTIME_API void sf_call_action_subsys_zeroOutDerivatives(SimStruct* S, void* execBlockPtr);
LIBMWSF_RUNTIME_API void sf_call_action_subsys_zeroCrossings(SimStruct* S, void* execBlockPtr);
LIBMWSF_RUNTIME_API void* sf_get_action_subsys_block_state(SimStruct* S,
                                                           const char* actionSubsysName,
                                                           const char* blkStateName);

LIBMWSF_RUNTIME_API void* slcsGetInputArgFromExecInfoVoid(void* args, int i);
LIBMWSF_RUNTIME_API void* slcsGetOutputArgFromExecInfoVoid(void* args, int i);

LIBMWSF_RUNTIME_API void sf_error_out_about_continuous_sample_time_with_persistent_vars(
    SimStruct* S);
LIBMWSF_RUNTIME_API void sf_error_check_illegal_use_of_simulink_fcn(SimStruct* S,
                                                                    const char* subsysName,
                                                                    unsigned int simFcnSSID,
                                                                    unsigned int userSSID);
LIBMWSF_RUNTIME_API void sf_error_check_illegal_use_of_simulink_fcns(SimStruct* S,
                                                                     mxArray* simFcnInfo);

LIBMWSF_RUNTIME_API bool ssIsMajorTimeStep_wrapper(SimStruct* S);
LIBMWSF_RUNTIME_API void ssSetSolverNeedsReset_wrapper(SimStruct* S);
LIBMWSF_RUNTIME_API void ssSetBlockIsPurelyCombinatorial_wrapper(SimStruct* S,
                                                                 bool isCombinatorial);

#define SF_SIM_RUNNING 0
#define SF_SIM_STOPPED 1
#define SF_SIM_PAUSED 2

#endif
