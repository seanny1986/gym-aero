/* Copyright 2017 The MathWorks, Inc. */

#ifndef MATLAB_LIBRARY_HPP
#define MATLAB_LIBRARY_HPP

#include "matlab_application.hpp"
#include <MatlabExecutionInterface/execution_interface.hpp>
#include <vector>
#include <streambuf>
#include <memory>
#include <future>
#include <complex>

namespace matlab {

    namespace cpplib {

        using namespace matlab::execution; 
        class MATLABLibrary : public matlab::execution::ExecutionInterface{
        public:
           /**
            * Destructor
            *
            * @throw none
            */
            ~MATLABLibrary();     
 
            /**
            * wait for all figures to be closed 
            **/
            void waitForFiguresToClose();
 

        private:
            
            friend FutureResult<std::unique_ptr<MATLABLibrary>> initMATLABLibraryAsync(std::shared_ptr<MATLABApplication> application, const std::u16string& ctffilename);
           /**
            * Constructor
            *
            * @param handle - The internal implementation
            * 
            * @throw none
            */
            MATLABLibrary(std::shared_ptr<MATLABApplication> application, uint64_t handle);

            std::shared_ptr<MATLABApplication> appPtr_;

        };

    }
}

#endif //MATLAB_LIBRARY_HPP
