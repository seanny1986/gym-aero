#ifndef SLEXEC_SIMSTRUCT_BRIDGE_H
#define SLEXEC_SIMSTRUCT_BRIDGE_H
/**
 * @file slexec_simstruct_bridge.h
 *
 * This header is the exported C interface to the simulink execution engine
 * (slexec_simbridge) that involves simstruct. It is used by the Rapid
 * Accelerator target. This file should be included from simstruct (and no
 * earlier) so that we have access to preprocessor variables. 
 *
 * 
 */

/* Copyright 2014-2015 The MathWorks, Inc. */

#include "slexec_simbridge.h"
#include "slexec_parallel.h"
#include <setjmp.h>

/* Guard against early inclusions */
#ifndef _SIMSTRUCT
#error "This file should be included from simstruc.h"
#endif

/* Information required to initialize/run Execution Engine */
typedef struct ssBridgeExecutionInfo_tag {
    /* Root Simstruct */ 
    SimStruct* simstruct_; 

    struct SolverOptions {
        const char* jacobianPatternFileName_; /* solver Jacobian pattern */
    } solverOptions_;

    struct SimulationOptions {
        const char* parameterFileName_;       /* simulation option parameter file name */
        int parameterArrayIndex_;             /* simulation options array index */
        /* time out for executable (is an int as of now) */ 
        int timeLimit_;         
        int enableSLExecSSBridgeFeatureValue_;
        const char* inportFileName_; 
        int* matFileFormat_; 
        const char* simMetadataFilePath_;
    } simulationOptions_;

    struct ModelMethods {
        /* model start */
        void (*start) (void); 
        /* parameter update function */
        void (*outputsParameterSampleTime)(int);
        /* model terminate */
        void (*terminate)(void);
   } modelMethods_;

    struct RuntimeCallbacks {
        /* external mode call backs */ 
        ssBridgeExtModeCallbacks_T* externalModeCallbacks_;
        /* logging function */
        void (*loggingFunction)(SimStruct* );
        void (*setupMMIStateLog)(SimStruct*);
        const char* (*startDataLoggingWithStartTime)(RTWLogInfo*,
                                                const real_T, const real_T, 
                                                const real_T, const char_T **);
        const char* (*rapidReadInportsAndAperiodicHitTimes)(const char*, int*);
        const char* (*rapidCheckRemappings)(void);
        void (*sdiBindObserversAndStartStreamingEngine)(const char*);
    } runtimeCallbacks_;

    struct RuntimeFlags {
        boolean_T* parameterPacketReceived_;
        boolean_T* startPacketReceived_;
    } runtimeFlags_;

    struct RuntimeObjects {
        jmp_buf* longJumpBuffer_;
    } runtimeObjects_;    

    struct ParallelExecution {
        boolean_T enabled_;
        int_T simulatorType_;
        ParallelExecutionOptions options_;
    } parallelExecution_; 

} ssBridgeExecutionInfo_T;

/* Functions required for execution engine initialization */
typedef struct ssBridgeSetupAndTerminationCallbacks_tag{
    void (*getExecutionInfo)(SimStruct*, ssBridgeExecutionInfo_T *);
} ssBridgeSetupAndTerminationCallbacks_T;


/* Run simulation */
SLEXEC_SIMBRIDGE_PUBLISHED_C void ssRunSimulation(
    SimStruct* S,
    ssBridgeSetupAndTerminationCallbacks_T* setupCallbacks);

/* sti is the (also runtime) index of the *sole* rate in the event spec associated with the event */
/* on which you want to call raise */
/* runMultiplicity is the number of times the tasks in the event clock are run when raised */
SLEXEC_SIMBRIDGE_PUBLISHED_C void ssRaiseEventUsingRateIndexWithRunMultiplicity(
    SimStruct* S,
    int_T sti,
    uint_T runMultiplicity);

/* sti is the (also runtime) index of the *sole* rate in the event spec associated with the event */
/* on which you want to call raise */
SLEXEC_SIMBRIDGE_PUBLISHED_C void ssRaiseEventUsingRateIndex(
    SimStruct* S,
    int_T sti);

/* eventRuntimeIndex is the runtime index of the event on which you want to call raise */
/* runMultiplicity is the number of times the tasks in the event clock are run when raised */
SLEXEC_SIMBRIDGE_PUBLISHED_C void ssRaiseEventWithRunMultiplicity(
    SimStruct* S,
    uint_T eventRuntimeIndex,
    uint_T runMultiplicity);

/* eventRuntimeIndex is the runtime index of the event on which you want to raise the event */
SLEXEC_SIMBRIDGE_PUBLISHED_C void ssRaiseEvent(
    SimStruct* S,
    uint_T eventRuntimeIndex);

/* sti is the (also runtime) index of the *sole* rate in the task thay you want to schedule. This
 * task must be scheduled by an event */
SLEXEC_SIMBRIDGE_PUBLISHED_C void ssScheduleTaskUsingRateIndex(
    SimStruct* S,
    int_T sti);

/* taskRuntimeIndex is the runtime index of the task thay you want to schedule. This  task must be
 * scheduled by an event */
SLEXEC_SIMBRIDGE_PUBLISHED_C void ssScheduleTask(
    SimStruct* S,
    uint_T taskRuntimeIndex);

/* sti is the (also runtime) index of the *sole* rate in the task thay you want to disable */
SLEXEC_SIMBRIDGE_PUBLISHED_C void ssDisableTaskUsingRateIndex(
    SimStruct* S,
    int_T sti);

/* taskRuntimeIndex is the runtime index of the task thay you want to disable */
SLEXEC_SIMBRIDGE_PUBLISHED_C void ssDisableTask(
    SimStruct* S,
    uint_T taskRuntimeIndex);

/* sti is the (also runtime) index of the *sole* rate in the task thay you want to enable */
SLEXEC_SIMBRIDGE_PUBLISHED_C void ssEnableTaskUsingRateIndex(
    SimStruct* S,
    int_T sti);

/* taskRuntimeIndex is the runtime index of the task thay you want to enable */
SLEXEC_SIMBRIDGE_PUBLISHED_C void ssEnableTask(
    SimStruct* S,
    uint_T taskRuntimeIndex);

SLEXEC_SIMBRIDGE_PUBLISHED_C void slexecSetNumTicksToNextSampleHitForControllableSampleTime(
    SimStruct* rootS,
    size_t ctrlRateIdx,
    size_t nTicks);

SLEXEC_SIMBRIDGE_PUBLISHED_C size_t slexecGetNumTicksToNextSampleHitForControllableSampleTime(
    SimStruct* rootS,
    size_t ctrlRateIdx);

SLEXEC_SIMBRIDGE_PUBLISHED_C void ssWriteSimMetadata(
    SimStruct* S,
    const char* metadataFilePath
    );

#endif 
