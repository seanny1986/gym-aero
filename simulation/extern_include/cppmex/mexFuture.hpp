/* Copyright 2018 The MathWorks, Inc. */

#ifndef __MEX_FUTURE_HPP__
#define __MEX_FUTURE_HPP__

#include <vector>
#include <streambuf>
#include <memory>
#include <future>
#include "mexTaskReference.hpp"
#include "mexMatlabEngine.hpp"

namespace matlab {

    namespace engine {
        
        typedef std::basic_streambuf<char16_t> StreamBuffer;

        template <class T>
        class FutureResult;

        template<class T>
        class SharedFutureResult;

        template <class T>
        class FutureResult : public std::future<T> {
        public:
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
            void swap(FutureResult<T>& a_futureresult);

            /**
            * A constructor of FutureResult that accepts a standard future
            *
            * @param a_future - A standard future
            *
            * @throw none
            */
            FutureResult(std::future<T>&& a_future);

            /**
            * A constructor of FutureResult that accepts a standard future and a task reference
            *
            * @param a_future - A standard future
            * @param a_taskreference - A shared pointer of task reference
            *
            * @throw none
            */
            FutureResult(std::future<T>&& a_future, std::shared_ptr<TaskReference> a_taskreference);

            /**
            * A constructor of FutureResult that accepts a standard future, a task reference, and buffers
            *
            * @param a_future - A standard future
            * @param a_taskreference - A shared pointer of task reference
            * @param outputBuffer - A shared pointer of StreamBuffer used to store standard output
            * @param errorBuffer - A shared pointer of StreamBuffer used to store standard error
            *
            * @throw none
            */
            FutureResult(std::future<T>&& a_future,
                const std::shared_ptr<TaskReference>& a_taskreference,
                const std::shared_ptr<StreamBuffer>& outputBuffer,
                const std::shared_ptr<StreamBuffer>& errorBuffer
            );

            /**
            * Move constructor of FutureResult
            *
            * @param a_futureresult - A future result
            *
            * @throw none
            */
            FutureResult(FutureResult<T>&& a_futureresult);

            /**
            * Assignment operator
            *
            * @param rhs - A future result
            *
            * @throw none
            */
            FutureResult<T>& operator=(FutureResult<T>&& rhs);

            /**
            * Create a shared copy of this future result
            *
            * @return a SharedFutureResult
            *
            * @throw none
            */
            SharedFutureResult<T> share();

            /**
            * Get the result of MATLAB function call
            *
            * @return the result
            *
            * @throw MatlabSyntaxException, MatlabExecutionException, TypeConversionException, MatlabNotAvailableException, CancelledException, InterruptedException
            */
            T get();


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
            * Cancel the execution of a MATLAB command
            *
            * @param allowInterrupt - Interrupt the command or not if it is being processed
            * @return the request is cancel-able or not
            *
            * @throw none
            */
            bool cancel(bool allowInterrupt = true);

            /**
            * Get the task reference
            *
            * @return the shared pointer of the task reference
            *
            * @throw none
            */
            std::shared_ptr<TaskReference> getTaskReference();

        private:
            FutureResult(std::future<T>&) = delete;
            FutureResult(const FutureResult&) = delete;
            FutureResult& operator= (FutureResult&) = delete;

            std::future<T> future;
            std::shared_ptr<TaskReference> taskReference;
            std::weak_ptr<StreamBuffer> output;
            std::weak_ptr<StreamBuffer> error;
            friend SharedFutureResult<T>;
        };

        template <class T>
        class SharedFutureResult : public std::shared_future <T> {
        public:
            SharedFutureResult();

            ~SharedFutureResult();

            /**
            * Swap the contents of this SharedFutureResult with another SharedFutureResult
            *
            * @param a_sharedfuture - A SharedFutureResult
            * @return void
            *
            * @throw none
            */
            void swap(SharedFutureResult<T>& a_sharedfuture);

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
            SharedFutureResult(FutureResult<T>&& a_futureresult);

            /**
            * Move assignment operator
            *
            * @param rhs - A shared future result
            *
            * @throw none
            */
            SharedFutureResult<T>& operator=(SharedFutureResult<T>&& rhs);

            /**
            * Assignment operator
            *
            * @param rhs - A shared future result
            *
            * @throw none
            */
            SharedFutureResult<T>& operator=(const SharedFutureResult<T>& rhs);

            /**
            * Get the result of MATLAB function call
            *
            * @return the result
            *
            * @throw MatlabSyntaxException, MatlabExecutionException, TypeConversionException, MatlabNotAvailableException, CancelledException, InterruptedException
            */
            decltype(std::declval<std::shared_future<T>>().get()) get() const;

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
            * Cancel the execution of a MATLAB command
            *
            * @param allowInterrupt - Interrupt the command or not if it is being processed
            * @return the request is cancel-able or not
            *
            * @throw none
            */
            bool cancel(bool allowInterrupt = true);

        private:
            std::shared_future<T> sharedFuture;
            std::shared_ptr<TaskReference> taskReference;
        };
    }
}

#endif //__MEX_FUTURE_HPP__