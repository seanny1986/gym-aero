/* Copyright 2016-2018 The MathWorks, Inc. */

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#ifndef SLSIMTGTSLIOCLIENTSRTW_H
#define SLSIMTGTSLIOCLIENTSRTW_H

#include "sl_simtarget_instrumentation_spec.h"

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C void rtwAddMemoryClient(
        void * accessor,
        const void * signalDescriptor,
        void * catalogue,
        char * datasetName,
        const unsigned int clientType);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C void rtwAddR2Client(
    void * accessor,
    const void * signalDescriptor,
    void * catalogue,
    char * datasetName,
    const unsigned int clientType);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C void rtwAddIntrusiveClient(
	void * accessor,
	const void * signalDescriptor,
	void * catalogue,
	char * datasetName,
	const unsigned int clientType);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C unsigned int rtwIsLoggingToFile(
    void * catalogue);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C const char * rtwAddR2SharedResource(
    void * catalogue,
    int errorXMLMode);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C void* rtwGetR2SharedResource(
    void * catalogue);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C void rtwCreateSigstreamSlioClient(
    void * sigstreamManager,
    void * catalogue);

#endif
