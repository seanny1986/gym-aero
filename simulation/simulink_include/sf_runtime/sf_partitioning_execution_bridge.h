/* Copyright 2013-2018 The MathWorks, Inc. */

#ifndef SF_RUNTIME_SF_PARTITIONING_EXECUTION_BRIDGE_H
#define SF_RUNTIME_SF_PARTITIONING_EXECUTION_BRIDGE_H

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#include "sf_runtime/sf_runtime_spec.h"
#include "sf_runtime/utils.h"

LIBMWSF_RUNTIME_API void sf_schedule_partition(SimStruct* S, const char* partitionName);

LIBMWSF_RUNTIME_API void sf_disable_partition(SimStruct* S, const char* partitionName);

LIBMWSF_RUNTIME_API void sf_enable_partition(SimStruct* S, const char* partitionName);

#endif
