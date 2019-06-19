/**
 * Published header for C++ MEX Function
 *
 * Copyright 2017-2018 The MathWorks, Inc.
 */

#ifndef __MEX_FUNCTION_CPP_HPP__
#define __MEX_FUNCTION_CPP_HPP__

#include <vector>
#include <memory>

namespace matlab {
    namespace mex {

        template <typename iterator_type>
        class MexIORange;

        typedef MexIORange<std::vector<matlab::data::Array>::iterator> ArgumentList;

        class Function {

            void* functionImpl;

          public:

            Function();

            virtual ~Function() NOEXCEPT_FALSE;

            /**
             * @param outputs - Collection of matlab::data::Array objects that will returned back to MATLAB
             * @param inputs - Collection of matlab::data::Array objects that are passed to the MEX-file from MATLAB
             */
            virtual void operator()(ArgumentList outputs, ArgumentList inputs);

            std::shared_ptr<matlab::engine::MATLABEngine> getEngine();

            /**
             * Locks the MEX-file preventing MATLAB's 'clear' commands to remove it from memory.
             */
            void mexLock();

            /**
             * Unlocks the MEX-file allowing MATLAB's 'clear' commands to remove it from memory
             */
            void mexUnlock();

            /**
             * @return The name of the MEX function (the source file name of the MEX file)
             */
            std::u16string getFunctionName() const;

        };
    }
}

#endif
