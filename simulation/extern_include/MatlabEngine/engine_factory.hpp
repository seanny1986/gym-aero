/* Copyright 2017 The MathWorks, Inc. */

#ifndef ENGINE_FACTORY_HPP
#define ENGINE_FACTORY_HPP

#include <vector>
#include <string>
#include <memory>
#include "engine_util.hpp"

namespace matlab {

    namespace engine {
        
        using namespace matlab::execution;
        
        class MATLABEngine;

        /**
         * Start MATLAB synchronously with startup options
         *
         * @param options -  Startup options used to launch MATLAB
         * @return std::unique_ptr<MATLABEngine> - A unique pointer of MATLABEngine object
         *
         * @throw EngineException if failed to start MATLAB
         */
        std::unique_ptr<MATLABEngine> startMATLAB(const std::vector<std::u16string>& options = std::vector<std::u16string>());

        /**
        * Start MATLAB asynchronously with startup options
        *
        * @param options -  Startup options used to launch MATLAB
        * @return FutureResult<std::unique_ptr<MATLABEngine>> - A future of a unique pointer of a MATLABEngine object
        *
        * @throw none
        */
        FutureResult<std::unique_ptr<MATLABEngine>> startMATLABAsync(const std::vector<std::u16string>& options = std::vector<std::u16string>());

        /**
        * Find all shared MATLAB sessions synchronously
        *
        * @return std::vector<std::u16string> - A vector of names of shared MATLAB sessions
        * 
        * @throw none
        */
        std::vector<std::u16string> findMATLAB();

        /**
        * Find all shared MATLAB sessions synchronously
        *
        * @return FutureResult<std::vector<std::u16string>> - A future of a vector of names of shared MATLAB sessions
        *
        * @throw none
        */
        FutureResult<std::vector<std::u16string>> findMATLABAsync();

        /**
        * Start or connect to a shared MATLAB session synchronously
        *
        * @return std::unique_ptr<MATLABEngine> - A unique pointer of a MATLABEngine object
        *
        * @throw EngineException if failed to start or connect to a shared MATLAB session
        */
        std::unique_ptr<MATLABEngine> connectMATLAB();

        /**
        * Connect to a shared MATLAB session synchronously
        *
        * @param std::u16string - The name of a shared MATLAB session
        * @return std::unique_ptr<MATLABEngine> - A unique pointer of a MATLABEngine object
        *
        * @throw EngineException if failed to connect to a shared MATLAB session
        */
        std::unique_ptr<MATLABEngine> connectMATLAB(const std::u16string& name);

        /**
        * Start or connect to a shared MATLAB session asynchronously
        *
        * @return FutureResult<std::unique_ptr<MATLABEngine>> - A future of a unique pointer of a MATLABEngine object
        *
        * @throw none
        */
        FutureResult<std::unique_ptr<MATLABEngine>> connectMATLABAsync();

        /**
        * Connect to a shared MATLAB session asynchronously
        *
        * @param std::u16string - The name of a shared MATLAB session
        * @return FutureResult<std::unique_ptr<MATLABEngine>> - A future of a unique pointer of a MATLABEngine object
        *
        * @throw none
        */
        FutureResult<std::unique_ptr<MATLABEngine>> connectMATLABAsync(const std::u16string& name);

        /**
        * Terminate the engine client.  The Engine application will be no longer able to start MATLAB after calling this function.
        *
        * @return void
        *
        * @throw none
        */
        void terminateEngineClient();
    }
}


#endif  //ENGINE_FACTORY_HPP
