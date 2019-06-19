/* Copyright 2016 The MathWorks, Inc. */

#pragma once

typedef enum
{
    SF_DOUBLE  = 0,
    SF_SINGLE,
    SF_INT8,
    SF_UINT8,
    SF_INT16,
    SF_UINT16,
    SF_INT32,
    SF_UINT32,
    SF_INT64,
    SF_UINT64,
    SF_CHAR,
    SF_BOOL,
    SF_MATLAB,
    SF_STRUCT,
    SF_UNKNOWN_ZZ,
    SF_STRING,
    SF_ENUM,
    SF_TOTAL_DATA_TYPES
} SfDataType;

typedef enum
{
    SF_ROW_MAJOR,
    SF_COLUMN_MAJOR
} SfIndexScheme;

