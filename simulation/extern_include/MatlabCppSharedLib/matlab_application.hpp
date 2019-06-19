/* Copyright 2017 The MathWorks, Inc. */

#ifndef MATLAB_APPLICATION_HPP
#define MATLAB_APPLICATION_HPP

#include "cppsharedlib_factory.hpp"

namespace matlab {
    namespace cpplib {

        class MATLABApplication {
        public:
            ~MATLABApplication();
        private:
            MATLABApplication(const MATLABApplicationMode mode, const std::vector<std::u16string>& options);
            friend std::shared_ptr<MATLABApplication> initMATLABApplication(
                const MATLABApplicationMode mode, 
                const std::vector<std::u16string>& options);

            MATLABApplicationMode _mode;
            std::vector<std::u16string> _options;
         };
    }
}

#endif // MATLAB_APPLICATION_HPP 
