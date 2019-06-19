/* Copyright 2017 The MathWorks, Inc. */

#ifndef CPPSHAREDLIB_FACTORY_IMPL_HPP
#define CPPSHAREDLIB_FACTORY_IMPL_HPP

#include "../cppsharedlib_factory.hpp"
#include "../cppsharedlib_exception.hpp"
#include "cppsharedlib_ctf_path.hpp"

namespace {

    std::once_flag sessionFlag;
    static std::atomic<bool> initialized(false);
    static std::atomic<bool> terminated(false);
    bool init(const std::vector<std::u16string>& options) {
        try{
            std::vector<char16_t*> options_v(options.size());
            std::transform(options.begin(), options.end(), options_v.begin(), [](const std::u16string& option){ return const_cast<char16_t*>(option.c_str());  });
            runtime_create_session(options_v.data(), options_v.size());
        }
        catch (...) {
            // not much we can do here.
        }
        return true;
    }

    void initIt(const std::vector<std::u16string>& options) {
        initialized = init(options);
    }

    void initSession(const std::vector<std::u16string>& options) {
        if (terminated) {
            throw matlab::cpplib::CppSharedLibException("MATLABApplication is already terminated and cannot be reinitialized.");
        }
        if (initialized) {
            throw matlab::cpplib::CppSharedLibException("MATLABApplication is already initialized.");
        }
        std::call_once(sessionFlag, initIt, options);
    }

    std::function<int(int, const char**)>* userMainFcnPtr;
    extern "C" inline int wrapUserMain(int argc, const char** argv)
    {
        return (*userMainFcnPtr)(argc, argv);
    }
}


namespace matlab {
    namespace cpplib {

        inline std::shared_ptr<MATLABApplication> initMATLABApplication(const MATLABApplicationMode mode, const std::vector<std::u16string>& options)
        {
            if (mode == MATLABApplicationMode::IN_PROCESS) {
                initSession(options);
            }else {
                const std::string STR_OUTPROC = "-outproc";
                const std::u16string U16STR_OUTPROC(STR_OUTPROC.cbegin(), STR_OUTPROC.cend());
                std::vector<std::u16string> newOptions={U16STR_OUTPROC};
                std::copy(options.begin(),options.end(), std::back_inserter(newOptions));
                initSession(newOptions);
            }
            return std::shared_ptr<MATLABApplication>(new MATLABApplication(mode, options));
        }

        inline std::unique_ptr<MATLABLibrary> initMATLABLibrary(std::shared_ptr<MATLABApplication> application, const std::u16string& ctffilename) {
            return initMATLABLibraryAsync(application, ctffilename).get();
        }

        inline FutureResult<std::unique_ptr<MATLABLibrary>> initMATLABLibraryAsync(std::shared_ptr<MATLABApplication> application, const std::u16string& ctffilename) {
            const std::u16string absolutePathToCTF = detail::getPathToCtf(ctffilename);
            if (absolutePathToCTF.empty())
            {
                std::string str("Unable to open file '");
                str += convertUTF16StringToUTF8String(ctffilename);
                str += "'.";
                throw CppSharedLibException(str);
            }
            // absolutePathToCTF is intentionally passed by copy rather than by reference.
            auto startMVMType = [&application, absolutePathToCTF]() {
                bool errFlag = false;
                uint64_t handle = create_mvm_instance(absolutePathToCTF.c_str(), &errFlag);
                if (errFlag) {
                    throw CppSharedLibException("Failed to initialize MATLABLibrary.");
                }
                return std::unique_ptr<MATLABLibrary>(new MATLABLibrary(application, handle));
            };
            std::future<std::unique_ptr<MATLABLibrary>> stdF = std::async(std::launch::async, startMVMType);
            FutureResult<std::unique_ptr<MATLABLibrary>> future(std::move(stdF));
            return (future);
        }

        inline int runMain(std::function<int(std::shared_ptr<MATLABApplication>, int, const char**)> mainFcn, 
                           std::shared_ptr<MATLABApplication>&& appsession, int argc, const char **argv)
        {
            using namespace std::placeholders;
            std::function<int(int, const char**)> userMainFcn = [&](int argc, const char** argv) {return mainFcn(std::move(appsession), argc, argv);};
            userMainFcnPtr = &userMainFcn;
            return cppsharedlib_run_main(wrapUserMain, argc, argv);
        }

    }
}


#endif //CPPSHAREDLIB_FACTORY_IMPL_HPP
