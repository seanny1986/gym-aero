/* Copyright 2017 The MathWorks, Inc. */

#ifndef CPPSHARED_LIB_PATH_INIT_HPP
#define CPPSHARED_LIB_PATH_INIT_HPP

#ifdef _WIN32
    // Required for path initialization, which we only need to perform on 
    // Windows.
    #include "matlab_runtime_version.hpp"
    
    #define WIN32_LEAN_AND_MEAN
    #include <windows.h>
#else
    #include <sys/stat.h>
#endif

#include <algorithm>
#include <iostream>
#include <string>

namespace matlab {
namespace cpplib {
namespace detail {

#ifdef _WIN32

    // DLL will be found at, e.g., matlab\runtime\win64\libMatlabCppSharedLib9_3.dll.

    // Construct filename, unpreceded by path.
    inline std::wstring getDllNameExclusiveOfPath() 
    {
        return std::wstring(MATLAB_CPP_SHARED_LIB_BASE_NAME) 
            + MATLAB_CPP_SHARED_LIB_SUFFIX;
    }

    template<typename T, typename Out>
    inline void split(const std::basic_string<T> &s, T delim, Out result) {
        std::basic_stringstream<T> ss;
        ss.str(s);
        std::basic_string<T> item;
        while (std::getline(ss, item, delim)) {
            *(result++) = item;
        }
    }

    template<typename T>
    inline std::vector<std::wstring> split(const std::basic_string<T> &s, T delim) {
        std::vector<std::basic_string<T>> elems;
        split(s, delim, std::back_inserter(elems));
        return elems;
    }        
    
    inline std::wstring getValueFromEnvVarW(const std::wstring &varName)
    {
        DWORD bufferSize = GetEnvironmentVariableW(varName.c_str(), NULL, 0);
        if (bufferSize)
        {
            std::wstring ret(bufferSize, wchar_t(0));
            DWORD getPathEnv = GetEnvironmentVariableW(varName.c_str(), &ret[0], bufferSize);
            ret.resize(bufferSize-1);
            return (getPathEnv ? ret : L"");
        }
        else
        {
            return L"";
        }
    }
    
    inline bool setEnvVarW(const std::wstring & varName, const std::wstring & varValue)
    {
        BOOL ret = SetEnvironmentVariableW(varName.c_str(), varValue.c_str());
        if (!ret)
        {
            DWORD err = GetLastError();
            std::cerr << "Attempt to set environment variable '" << std::string(varName.cbegin(), varName.cend()) << "' "
                << "to '" << std::string(varValue.cbegin(), varValue.cend()) << "' failed with error code " << err << std::endl;
        }
        return ret ? true : false;
    }
        
    inline std::wstring getPathFromEnvVarW()
    {
        return getValueFromEnvVarW(L"PATH");
    }
    
    inline std::vector<std::wstring> getPathElementsFromEnvVarW()
    {
        std::wstring buf = getPathFromEnvVarW();
        return split(buf, L';');
    }
    
    inline bool fileExistsW(const std::wstring & wstrPath)
    {
        DWORD dwAttrib = GetFileAttributesW(wstrPath.c_str());
        return (dwAttrib != INVALID_FILE_ATTRIBUTES && 
            !(dwAttrib & FILE_ATTRIBUTE_DIRECTORY));
    }
        
    inline bool fileExistsInDirW(const std::wstring & wstrFilename, const std::wstring & wstrDir)
    {
        if (wstrDir.empty() || wstrFilename.empty())
        {
            return false;
        }
        std::wstring wstrPath(wstrDir);
        if (wstrPath[wstrPath.length()-1] != L'\\')
        {
            wstrPath += L"\\";
        }
        wstrPath += wstrFilename;
        return fileExistsW(wstrPath);
    }

    /// Get the first directory in the specified collection of directories
    /// that contains a file with the name specified by wstrFilename.
    /// @param wstrFilename file to be found
    /// @param wstrDirs directories to search
    /// @returns First directory in `wstrDirs` that contains `wstrFilename`. Any
    /// trailing backslash will be stripped.
    inline std::wstring getFirstDirWhereFileIsFoundW(const std::wstring & wstrFilename, 
        const std::vector<std::wstring> & wstrDirs)
    {
        bool bFound = false;
        std::wstring pathElementWhereFound;
        for (std::wstring wstrDir : wstrDirs)
        {
            if (fileExistsInDirW(wstrFilename, wstrDir))
            {
                // chop off trailing backslash, if any
                size_t len = wstrDir.length();
                if (wstrDir[len-1] == L'\\')
                {
                    wstrDir.resize(len-1); 
                }
                return wstrDir;
            }
        }
        return L"";
    }
    
    inline std::wstring getFirstDirOnPathWhereFileIsFoundW(const std::wstring & wstrFilename)
    {
        std::vector<std::wstring> pathElements = getPathElementsFromEnvVarW();
        return getFirstDirWhereFileIsFoundW(wstrFilename, pathElements);
    }
    
    inline std::wstring getDirToAddDynamicallyToPathW()
    {
        std::wstring dir = getFirstDirOnPathWhereFileIsFoundW(getDllNameExclusiveOfPath());
        if (!dir.empty())
        {
            dir += L"\\..\\..\\extern\\bin\\win64";
        }
        return dir;
    }

    inline bool loadMatlabDataArrayLibrary()
    {
        std::wstring wpath = getDirToAddDynamicallyToPathW();
        if (wpath.empty())
        {
            return false;
        }
        else
        {
            wpath += L'\\';
            wpath += L"libMatlabDataArray.dll";
            HMODULE hmod = LoadLibraryW(wpath.c_str());
            return (hmod ? true : false);
        }
    }
    
    class MatlabCppSharedLibraryPathInitializer
    {
        public:
        MatlabCppSharedLibraryPathInitializer()
        {
            // Ignore return value.
            loadMatlabDataArrayLibrary();
        }
    };
    
    static MatlabCppSharedLibraryPathInitializer pathInitializer;
    

    inline std::u16string getValueFromEnvVar(const std::u16string &varName)
    {
        std::wstring wstr = getValueFromEnvVarW(std::wstring(varName.cbegin(), varName.cend()));
        return std::u16string(wstr.cbegin(), wstr.cend());
    }
    
    inline bool setEnvVar(const std::u16string & varName, const std::u16string & varValue)
    {
        std::wstring wVarName(varName.cbegin(), varName.cend());
        std::wstring wVarValue(varValue.cbegin(), varValue.cend());
        return setEnvVarW(wVarName, wVarValue);
    }
    
#else // non-Windows code follows

    inline bool fileExists(const std::u16string & strPath)
    {
        struct stat buffer;
        return(stat(matlab::execution::convertUTF16StringToUTF8String(strPath.c_str()).c_str(),
            & buffer) == 0);
    }
        
    inline bool fileExistsInDir(const std::u16string & ustrFilename, const std::u16string & ustrDir)
    {
        if (ustrDir.empty() || ustrFilename.empty())
        {
            return false;
        }
        std::u16string ustrPath(ustrDir);
        if (ustrPath[ustrPath.length()-1] != static_cast<char16_t>('/'))
        {
            ustrPath += static_cast<char16_t>('/');
        }
        ustrPath += ustrFilename;
        return fileExists(ustrPath);
    }
    
    inline std::u16string getValueFromEnvVar(const std::u16string &varName)
    {
        // This is fine because environment variable names are always ASCII.
        std::string sVarName(varName.cbegin(), varName.cend());
        const char * val = std::getenv(sVarName.c_str());
        if ( val == 0 ) {
            return std::u16string();
        }
        else {
            return matlab::execution::convertUTF8StringToUTF16String(val);
        }
    }
    
    inline bool setEnvVar(const std::u16string & varName, const std::u16string & varValue)
    {
        // This is fine because environment variable names are always ASCII.
        std::string sVarName(varName.cbegin(), varName.cend());
        std::string sVarValue = matlab::execution::convertUTF16StringToUTF8String(varValue);
        int ret = setenv(sVarName.c_str(), sVarValue.c_str(), /*overwrite=*/ 1);
        if (ret == -1)
        {
            std::cerr << "setEnvVar(" << sVarName << ", " << sVarValue << " failed." << std::endl;
            return false;
        }
        else
        {
            return true;
        }
    }
        
#endif // platform

}}} // end of nested namespace

#endif // CPPSHARED_LIB_PATH_INIT_HPP

