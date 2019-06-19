/* Copyright 2017 The MathWorks, Inc. */

#ifndef CPPSHAREDLIB_FACTORY_HPP
#define CPPSHAREDLIB_FACTORY_HPP

#include "cppsharedlib_util.hpp"

namespace matlab {

    namespace cpplib {

        using namespace matlab::execution;
        
        class MATLABApplication;
        class MATLABLibrary;
        
        std::shared_ptr<matlab::cpplib::MATLABApplication> initMATLABApplication(
            const MATLABApplicationMode mode = MATLABApplicationMode::IN_PROCESS, 
            const std::vector<std::u16string>& options = std::vector<std::u16string>());

        std::unique_ptr<MATLABLibrary> initMATLABLibrary(
            std::shared_ptr<MATLABApplication> application, const std::u16string& ctffilename);

        FutureResult<std::unique_ptr<MATLABLibrary>> initMATLABLibraryAsync(
            std::shared_ptr<MATLABApplication> application, const std::u16string& ctffilename);

        int runMain(std::function<int(std::shared_ptr<MATLABApplication>, int, const char**)> , 
            std::shared_ptr<MATLABApplication>&& appsession,
            int argc, 
            const char **argv);
    }
}


#endif //CPPSHAREDLIB_FACTORY_HPP