/* Copyright 2017 The MathWorks, Inc. */

#ifndef MATLAB_LIBRARY_IMPL_HPP
#define MATLAB_LIBRARY_IMPL_HPP

#include "../matlab_library.hpp"
#include "../cppsharedlib_exception.hpp"

namespace matlab {
    namespace cpplib {

        inline MATLABLibrary::MATLABLibrary(std::shared_ptr<MATLABApplication> application, uint64_t handle)
            : matlab::execution::ExecutionInterface(handle), appPtr_(application) {
        }
        
        inline void MATLABLibrary::waitForFiguresToClose() {
            if (matlabHandle != 0) {
                wait_for_figures_to_close(matlabHandle);
            }
        }

        inline MATLABLibrary::~MATLABLibrary() {
            if (matlabHandle != 0) {
                terminate_mvm_instance(matlabHandle);
                matlabHandle = 0;
            }
        }
    }
}


#endif //MATLAB_LIBRARY_IMPL_HPP