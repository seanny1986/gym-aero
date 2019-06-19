/* Copyright 2017 The MathWorks, Inc. */

#ifndef CPPSHAREDLIB_CTF_PATH_HPP
#define CPPSHAREDLIB_CTF_PATH_HPP

#include "cppsharedlib_path_init.hpp"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <stdio.h>
#include <unistd.h>
#ifdef __APPLE__
#include <mach-o/dyld.h>
#endif
#endif

#include <algorithm>
#include <iostream>
#include <string>

namespace matlab {
namespace cpplib {
namespace detail {

// An environment variable name must be ASCII, so it can be stored in an std::string.
const std::string BASE_CTF_PATH_ENV_VAR_NAME = "CPPSHARED_BASE_CTF_PATH";
const std::u16string BASE_CTF_PATH_ENV_VAR_NAME_U{
    BASE_CTF_PATH_ENV_VAR_NAME.cbegin(), BASE_CTF_PATH_ENV_VAR_NAME.cend()};

const char WINDOWS_PATH_SEP_CH = '\\';
const char COLON_CH = ':';
const char UNIX_PATH_SEP_CH = '/';
const char TILDE_CH = '~';

template<typename CharT>
inline void appendPathSepIfMissing(std::basic_string<CharT> & strPath, CharT pathSep)
{
    if (strPath.empty())
    {
        strPath.append(1, pathSep);
    }
    else
    {
        CharT lastChar = strPath[strPath.length() - 1];
        if (lastChar != static_cast<CharT>(WINDOWS_PATH_SEP_CH) && 
            lastChar != static_cast<CharT>(UNIX_PATH_SEP_CH))
        {
            strPath.append(1, pathSep);
        }
    }
}

template<typename CharT>
inline bool isAbsolute(const std::basic_string<CharT> & strPath)
{
    // The path to a CTF should always be greater than 2 characters.
    return (strPath.length() > 2 && 
        (strPath[0] == static_cast<CharT>(WINDOWS_PATH_SEP_CH) ||
         strPath[0] == static_cast<CharT>(UNIX_PATH_SEP_CH) ||
         strPath[0] == static_cast<CharT>(TILDE_CH) ||
         strPath[1] == static_cast<CharT>(COLON_CH)));
}

#ifdef _WIN32
    const wchar_t WIN_PATH_SEP_W = L'\\';
    const wchar_t * const PATH_SEPS_W = L"/\\";

    const std::wstring BASE_CTF_PATH_ENV_VAR_NAME_W{BASE_CTF_PATH_ENV_VAR_NAME.cbegin(), 
        BASE_CTF_PATH_ENV_VAR_NAME.cend()};
    
    // In the names of the following functions, the suffix "W" can be read as referring
    // to either "Windows" or "wchar_t" (or "wstring").
    inline std::wstring getPathToExecutableW() 
    {
        // There's no guarantee that an ordinary path will fit within MAX_PATH. If it 
        // doesn't fit, we iteratively double the buffer size up to 7 times (which 
        // should be more than enough).
        static const size_t INITIAL_BUFFER_SIZE = MAX_PATH;
        static const size_t MAX_ITERATIONS = 7;
        std::wstring ret;
        DWORD bufferSize = INITIAL_BUFFER_SIZE;
        for (size_t iterations = 0; iterations < MAX_ITERATIONS; ++iterations)
        {
            ret.resize(bufferSize);
            DWORD charsReturned = GetModuleFileNameW(NULL, &ret[0], bufferSize);
            if (charsReturned < ret.length())
            {
                ret.resize(charsReturned);
                return ret;
            }
            else
            {
                bufferSize *= 2;
            }
        }
        return L"";
    }    
    
    // Path will end with a path separator (forward slash or backslash).
    // This differs from UNIX, where only forward slashes are treated
    // as path separators and backslashes are treated as literal characters.
    inline std::wstring getBaseCtfPathFromExecutableW()
    {
        std::wstring ret = getPathToExecutableW();
        size_t lastPathSep = ret.find_last_of(PATH_SEPS_W);
        return ret.substr(0, lastPathSep+1);
    }
    
    inline std::wstring getBaseCtfPathFromEnvVarW()
    {
        return getValueFromEnvVarW(BASE_CTF_PATH_ENV_VAR_NAME_W.c_str());
    }
    
    inline std::wstring getWorkingDirW()
    {
        DWORD bufferSize = GetCurrentDirectoryW(0, NULL);
        std::wstring strPath(bufferSize, wchar_t(0));
        GetCurrentDirectoryW(bufferSize, &strPath[0]);
        return strPath.substr(0, bufferSize-1);
    }
    
    inline std::wstring getPathToCtfW(const std::wstring & ctfAsSpecifiedToInitFunc)
    {
        if (isAbsolute(ctfAsSpecifiedToInitFunc))
        {
            if (fileExistsW(ctfAsSpecifiedToInitFunc))
            {
                return ctfAsSpecifiedToInitFunc;
            }
            else
            {
                return std::wstring();
            }
        }
        else
        {
            std::wstring strPath = getBaseCtfPathFromEnvVarW();
            if (fileExistsInDirW(ctfAsSpecifiedToInitFunc, strPath))
            {
                appendPathSepIfMissing(strPath, WIN_PATH_SEP_W);
                strPath += ctfAsSpecifiedToInitFunc;
                return strPath;
            }
            else
            {
                strPath = getWorkingDirW();
                if (fileExistsInDirW(ctfAsSpecifiedToInitFunc, strPath))
                {
                    appendPathSepIfMissing(strPath, WIN_PATH_SEP_W);
                    strPath += ctfAsSpecifiedToInitFunc;
                    return strPath;
                }
                else
                {
                    strPath = getBaseCtfPathFromExecutableW();
                    if (fileExistsInDirW(ctfAsSpecifiedToInitFunc, strPath))
                    {
                        appendPathSepIfMissing(strPath, WIN_PATH_SEP_W);
                        strPath += ctfAsSpecifiedToInitFunc;
                        return strPath;
                    }
                    else
                    {
                        return std::wstring();
                    }
                }
            }
        }
    }

    inline std::u16string getWorkingDir()
    {
        std::wstring wstr = getWorkingDirW();
        return std::u16string(wstr.cbegin(), wstr.cend());
    }
    
    inline std::u16string getPathToCtf(const std::u16string & ctfAsSpecifiedToInitFunc)
    {
        std::wstring wstrCtfAsSpecifiedToInitFunc(ctfAsSpecifiedToInitFunc.cbegin(), 
            ctfAsSpecifiedToInitFunc.cend());
        const std::wstring wstrRet = getPathToCtfW(wstrCtfAsSpecifiedToInitFunc);
        return std::u16string(wstrRet.cbegin(), wstrRet.cend());
    }

    
#else // non-Windows code follows
    const char16_t UNIX_PATH_SEP_U = static_cast<char16_t>(UNIX_PATH_SEP_CH);
    
    inline std::u16string getWorkingDir()
    {
        char buf[FILENAME_MAX];
        const char * workingDir = getcwd(buf, FILENAME_MAX);
        if (!workingDir)
        {
            return std::u16string();
        }
        else
        {
            return std::u16string(buf, buf + strlen(buf));
        }
    }

    inline std::u16string getBaseCtfPathFromEnvVar()
    {
        return getValueFromEnvVar(BASE_CTF_PATH_ENV_VAR_NAME_U);
    }

    inline std::u16string getPathToExecutable()
    {
        char pBuf[FILENAME_MAX];
#ifdef __linux__        
        int bytes = std::min(readlink("/proc/self/exe", pBuf, FILENAME_MAX), static_cast<ssize_t>(FILENAME_MAX - 1));
        if (bytes >= 0)
        {
            pBuf[bytes] = '\0';
        }
        return std::u16string(pBuf, pBuf+bytes);
#else
        uint32_t size = sizeof(pBuf);
        if (_NSGetExecutablePath(pBuf, &size) == 0)
        {
            // realpath allocates a buffer using malloc. Use a unique_ptr to ensure that it gets freed
            // even if convertUTF8StringToUTF16String() throws an exception.
            std::unique_ptr<char, std::function<void(char*)>> ptrCanonical(
                realpath(pBuf, NULL), [](char * ptr){free(ptr);});
            std::u16string ret = matlab::execution::convertUTF8StringToUTF16String(ptrCanonical.get());
            return ret;
        }
        else
        {
            return std::u16string();
        }
#endif        
    }
    
    // Path will end with a path separator (forward slash only).
    inline std::u16string getBaseCtfPathFromExecutable(const std::u16string & pathToExecutable)
    {
        size_t lastPathSep = pathToExecutable.find_last_of(UNIX_PATH_SEP_U);
        return pathToExecutable.substr(0, lastPathSep+1);
    }
    
#ifdef __APPLE__
const std::u16string CONTENTS_MAC_OS = u".app/Contents/MacOS";

    // Go up three directories. For example, if your executable is 
    // generic_interface/foo_generic.app/Contents/MacOS/foo, look for the
    // last directory separator prior to ".app/Contents/MacOS" and 
    // terminate the string there.
    inline std::u16string getPathOutsideBundle(const std::u16string & pathToExecutable)
    {
        size_t lastPathSep = pathToExecutable.rfind(CONTENTS_MAC_OS);
        if (lastPathSep != std::string::npos && lastPathSep != 0)
        {
            lastPathSep = pathToExecutable.rfind(UNIX_PATH_SEP_U, lastPathSep-1);
            if (lastPathSep != std::string::npos)
            {
                return pathToExecutable.substr(0, lastPathSep+1);
            }
        }
        return std::u16string();
    }
#endif
    
    // The code in this function is similar, but not identical, to the code 
    // in getPathToCtfW(), which is called by the Windows-defined version of
    // getPathToCtf(). While it seems as though the code could be shared, in
    // reality, this would involve having to either (a) write template versions of
    // lower-level functions that are platform-specific anyway or (b) perform
    // unnecessary conversions between strings of different types.
    inline std::u16string getPathToCtf(const std::u16string & ctfAsSpecifiedToInitFunc)
    {
        if (isAbsolute(ctfAsSpecifiedToInitFunc))
        {
            if (fileExists(ctfAsSpecifiedToInitFunc))
            {
                return ctfAsSpecifiedToInitFunc;
            }
            else
            {
                return std::u16string();
            }
        }
        else
        {
            std::u16string strPath = getBaseCtfPathFromEnvVar();
            if (fileExistsInDir(ctfAsSpecifiedToInitFunc, strPath))
            {
                appendPathSepIfMissing(strPath, UNIX_PATH_SEP_U);
                strPath += ctfAsSpecifiedToInitFunc;
                return strPath;
            }
            else
            {
                strPath = getWorkingDir();
                if (fileExistsInDir(ctfAsSpecifiedToInitFunc, strPath))
                {
                    appendPathSepIfMissing(strPath, UNIX_PATH_SEP_U);
                    strPath += ctfAsSpecifiedToInitFunc;
                    return strPath;
                }
                else
                {
                    std::u16string pathToExecutable = getPathToExecutable();
                    strPath = getBaseCtfPathFromExecutable(pathToExecutable);
                    if (fileExistsInDir(ctfAsSpecifiedToInitFunc, strPath))
                    {
                        appendPathSepIfMissing(strPath, UNIX_PATH_SEP_U);
                        strPath += ctfAsSpecifiedToInitFunc;
                        return strPath;
                    }
                    else
                    {
#ifdef __APPLE__
                        strPath = getPathOutsideBundle(pathToExecutable);
                        if (fileExistsInDir(ctfAsSpecifiedToInitFunc, strPath))
                        {
                            appendPathSepIfMissing(strPath, UNIX_PATH_SEP_U);
                            strPath += ctfAsSpecifiedToInitFunc;
                            return strPath;
                        }
#endif
                        return std::u16string();
                    }
                }
            }
        }
    }
    
#endif

}}} // end of nested namespace
        
#endif // CPPSHARED_CTF_PATH_HPP

