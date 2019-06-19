/* Copyright 2017 The MathWorks, Inc. */
 
#ifndef ENGINE_FUTURE_HPP
#define ENGINE_FUTURE_HPP

#include <vector>
#include <streambuf>
#include <memory>
#include <future>

namespace matlab {

    namespace execution {

        using namespace matlab::engine;
        
        /**
        * @class FutureResult<std::unique_ptr<MATLABEngine>>
        * This class is a specialization of FutureResult<T>.
        * matlab::engine::startMATLABAsync and matlab::engine::connectMATLABAsync return instances of this type.
        */
        template <>
        class FutureResult<std::unique_ptr<MATLABEngine>>: public std::future<std::unique_ptr<MATLABEngine>>{
        public:

            /**
            * A constructor of FutureResult that accepts a standard future
            *
            * @param a_future - A standard future
            *
            * @throw none
            */
            FutureResult(std::future<std::unique_ptr<MATLABEngine>>&& a_future);

            /**
            * Move constructor of FutureResult
            *
            * @param a_futureresult - A future result
            *
            * @throw none
            */
            FutureResult(FutureResult<std::unique_ptr<MATLABEngine>>&& a_futureresult);

            /**
            * Assignment operator
            *
            * @param rhs - A future result
            *
            * @throw none
            */
            FutureResult<std::unique_ptr<MATLABEngine>>& operator=(FutureResult<std::unique_ptr<MATLABEngine>>&& rhs);

            FutureResult();

            ~FutureResult();

            /**
            * Swap the contents of this FutureResult with another FutureResult
            *
            * @param a_futureresult -  A FutureResult
            * @return void
            *
            * @throw none
            */
            void swap(FutureResult<std::unique_ptr<MATLABEngine>>& a_futureresult);

            /**
            * Get the MATLAB Engine
            *
            * @return the MATLABEngine unique pointer
            *
            * @throw InterruptedException, EngineException
            */
            std::unique_ptr<MATLABEngine> get();

            /**
            * Create a shared copy of this future result
            *
            * @return a SharedFutureResult
            *
            * @throw none
            */
            SharedFutureResult<std::unique_ptr<MATLABEngine>> share();

            /**
            * Check the future result is valid or not
            *
            * @return true if valid; false otherwise
            *
            * @throw none
            */
            bool valid() const;

            /**
            * Wait until the result is ready
            *
            * @throw none
            */
            void wait() const;

            /**
            * Wait until certain time point
            *
            * @param abs_time - A  time point
            * @return the status of the future result
            *
            * @throw none
            */
            template<class Clock, class Duration>
            std::future_status wait_until(const std::chrono::time_point<Clock, Duration>& abs_time) const;
            template<class Rep, class Period>

            /**
            * Wait for certain amount of time
            *
            * @param rel_time - Time during to wait
            * @return the status of the future result
            *
            * @throw none
            */
            std::future_status wait_for(const std::chrono::duration<Rep, Period>& rel_time) const;
           
            /**
            * Cancel the launch or connect of MATLAB
            * In R2017b, the startAMTLAB and connectMATLAB cannot be canceled.
            *
            * @param allowInterrupt - Interrupt the command or not if it is being processed
            * @return the request is cancel-able or not
            *
            * @throw none
            */
            bool cancel(bool allowInterrupt = true);

        private:
            FutureResult(std::future<std::unique_ptr<MATLABEngine>>&) = delete;
            FutureResult(FutureResult&) = delete;
            FutureResult& operator= (FutureResult&) = delete;
            std::future<std::unique_ptr<MATLABEngine>> future;
            friend SharedFutureResult<std::unique_ptr<MATLABEngine>>;
        };

        /**
        * @class SharedFutureResult<std::unique_ptr<MATLABEngine>>
        * This class is a specialization of SharedFutureResult<T>.
        * It can be created by calling FutureResult<std::unique_ptr<MATLABEngine>>.shared()
        */
        template <>
        class SharedFutureResult<std::unique_ptr<MATLABEngine>>: public std::shared_future<std::unique_ptr<MATLABEngine>>{
        public:
            SharedFutureResult();

            ~SharedFutureResult();

            /**
            * Swap the contents of this SharedFutureResult with another SharedFutureResult
            *
            * @param a_sharedfuture -  A SharedFutureResult
            * @return void
            *
            * @throw none
            */
            void swap(SharedFutureResult<std::unique_ptr<MATLABEngine>>& a_sharedfuture);

            /**
            * Copy constructor
            */
            SharedFutureResult(const SharedFutureResult& a_sharedfuture);

            /**
            * Move constructor
            */
            SharedFutureResult(SharedFutureResult&& a_sharedfuture);

            /**
            * Move constructor that accepts a FutureResult
            */
            SharedFutureResult(FutureResult<std::unique_ptr<MATLABEngine>>&& a_futureresult);

            /**
            * Move assignment operator
            *
            * @param rhs - A shared future result
            *
            * @throw none
            */
            SharedFutureResult<std::unique_ptr<MATLABEngine>>& operator=(SharedFutureResult<std::unique_ptr<MATLABEngine>>&& rhs);

            /**
            * Assignment operator
            *
            * @param rhs - A shared future result
            *
            * @throw none
            */
            SharedFutureResult<std::unique_ptr<MATLABEngine>>& operator=(const SharedFutureResult<std::unique_ptr<MATLABEngine>>& rhs);

            /**
            * Get the MATLAB Engine
            *
            * @return the MATLABEngine unique pointer
            *
            * @throw InterruptedException, EngineException
            */
            const std::unique_ptr<MATLABEngine>& get() const;

            /**
            * Check the future result is valid or not
            *
            * @return true if valid; false otherwise
            *
            * @throw none
            */
            bool valid() const;

            /**
            * Wait until the result is ready
            *
            * @throw none
            */
            void wait() const;

            /**
            * Wait until certain time point
            *
            * @param abs_time - A  time point
            * @return the status of the future result
            *
            * @throw none
            */
            template<class Clock, class Duration>
            std::future_status wait_until(const std::chrono::time_point<Clock, Duration>& abs_time) const;
            
            /**
            * Wait for certain amount of time
            *
            * @param rel_time - Time during to wait
            * @return the status of the future result
            *
            * @throw none
            */
            template<class Rep, class Period>
            std::future_status wait_for(const std::chrono::duration<Rep, Period>& rel_time) const;
            
            /**
            * Cancel the execution of a MATLAB command
            * In R2017b, the startAMTLAB and connectMATLAB cannot be canceled.
            *
            * @param allowInterrupt - Interrupt the command or not if it is being processed
            * @return the request is cancel-able or not
            *
            * @throw none
            */
            bool cancel(bool allowInterrupt = true);

        private:
            std::shared_future<std::unique_ptr<MATLABEngine>> sharedFuture;
        };
    }
}


#endif /* ENGINE_FUTURE_HPP */