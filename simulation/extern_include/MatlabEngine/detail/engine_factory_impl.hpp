/* Copyright 2017 The MathWorks, Inc. */

#ifndef ENGINE_FACTORY_IMPL_HPP
#define ENGINE_FACTORY_IMPL_HPP

#include <algorithm>
#include <ratio>
#include <cstring>
#include <memory>
#include <future>
#include "../cpp_engine_api.hpp"
#include "../matlab_engine.hpp"
#include "../engine_factory.hpp"
#include "../engine_future.hpp"
#include "../engine_exception.hpp"


namespace {
    void initSession() {
        cpp_engine_create_session();
    }

    bool cancelFind(uintptr_t, bool allowInterrupt) {
        return false;
    }
}

namespace matlab {
    namespace engine {
        using namespace matlab::execution;
        
        inline std::unique_ptr<MATLABEngine> startMATLAB(const std::vector<std::u16string>& options) {
            return startMATLABAsync(options).get();
        }

        inline FutureResult<std::unique_ptr<MATLABEngine>> startMATLABAsync(const std::vector<std::u16string>& options) {
            initSession();
            auto startMATLABType = [options]() {
                std::vector<char16_t*> options_v(options.size());
                std::transform(options.begin(), options.end(), options_v.begin(), [](const std::u16string& option){ return const_cast<char16_t*>(option.c_str());  });
                bool errFlag = false;
                uint64_t matlab = cpp_engine_create_out_of_process_matlab(options_v.data(), options_v.size(), &errFlag);
                if (errFlag) {
                    throw EngineException("MATLAB process cannot be created.");
                }
                return std::unique_ptr<MATLABEngine>(new MATLABEngine(matlab));
            };
            std::future<std::unique_ptr<MATLABEngine>> stdF = std::async(std::launch::async, startMATLABType);
            FutureResult<std::unique_ptr<MATLABEngine>> future(std::move(stdF));
            return future;
        }

        inline std::vector<std::u16string> findMATLAB() {
            initSession();
            char16_t** names;
            size_t size = cpp_engine_find_shared_matlab(&names);
            std::vector<std::u16string> names_v(size);
            for (size_t i = 0; i < size; i++) {
                names_v[i] = names[i];
            }
            cpp_engine_destroy_names(names, size);
            return names_v;
        }

        inline FutureResult<std::vector<std::u16string>> findMATLABAsync() {
            std::future<std::vector<std::u16string> > stdFuture = std::async(std::launch::async, findMATLAB);
            FutureResult<std::vector<std::u16string>> future(std::move(stdFuture), std::make_shared<TaskReference>(&cancelFind));
            return future;
        }

        inline std::unique_ptr<MATLABEngine> connectMATLAB() {
            return connectMATLABAsync().get();
        }

        inline std::unique_ptr<MATLABEngine> connectMATLAB(const std::u16string& name) {
            return connectMATLABAsync(name).get();
        }
        
        inline FutureResult<std::unique_ptr<MATLABEngine> > connectMATLABAsync() {
            std::vector<std::u16string> engines = findMATLAB();
            if (!engines.empty()) {
                return connectMATLABAsync(engines[0]);
            }
            else {
                std::vector<std::u16string> options;
                std::u16string option1 = convertUTF8StringToUTF16String("-r");
                std::u16string option2 = convertUTF8StringToUTF16String("matlab.engine.shareEngine");
                options.push_back(option1);
                options.push_back(option2);
                return startMATLABAsync(options);
            }
        }

        inline FutureResult<std::unique_ptr<MATLABEngine>> connectMATLABAsync(const std::u16string& name) {
            initSession();
            auto connectMATLABType = [name]() {
                bool errFlag = false;
                uint64_t matlab = cpp_engine_attach_shared_matlab(name.c_str(), &errFlag);
                if (errFlag) {
                    throw EngineException("MATLAB session cannot be connected.");
                }
                return std::unique_ptr<MATLABEngine>(new MATLABEngine(matlab));
            };
            std::future<std::unique_ptr<MATLABEngine>> stdF = std::async(std::launch::async, connectMATLABType);
            FutureResult<std::unique_ptr<MATLABEngine>> future(std::move(stdF));
            return future;
        }

        inline void terminateEngineClient() {
            //initialize the session and load the library so the call to engine_terminate_session will not crash.
            initSession();
            cpp_engine_terminate_session();
        }
    }
}


#endif  //ENGINE_FACTORY_IMPL_HPP
