/* Copyright 2018 The MathWorks, Inc. */

#ifndef __MEX_FUTURE_IMPL_HPP__
#define __MEX_FUTURE_IMPL_HPP__

#include <vector>
#include <streambuf>
#include <memory>
#include <future>
#include "../mexFuture.hpp"

namespace matlab {
    namespace engine {

        template<class T>
        FutureResult<T>::FutureResult() :std::future<T>(), taskReference() {}

        template<class T>
        FutureResult<T>::~FutureResult() {
        }

        template<class T>
        void FutureResult<T>::swap(FutureResult<T>& a_futureresult) {
            taskReference.swap(a_futureresult.taskReference);
            std::swap(future, a_futureresult.future);
            std::swap(*static_cast<std::future<T>*>(this), static_cast<std::future<T>&>(a_futureresult));
        }

        template<class T>
        FutureResult<T>::FutureResult(std::future<T>&& a_future) : future(std::move(a_future)), taskReference()  {
        }

        template<class T>
        FutureResult<T>::FutureResult(std::future<T>&& a_future, std::shared_ptr<TaskReference> a_taskreference) : std::future<T>(), future(std::move(a_future)), taskReference(a_taskreference), output(), error() {
        }

        template<class T>
        FutureResult<T>::FutureResult(std::future<T>&& a_future,
            const std::shared_ptr<TaskReference>& a_taskreference,
            const std::shared_ptr<StreamBuffer>& outputBuffer,
            const std::shared_ptr<StreamBuffer>& errorBuffer
            ) : std::future<T>(), future(std::move(a_future)), taskReference(a_taskreference), output(outputBuffer), error(errorBuffer) {
        }

        template<class T>
        FutureResult<T>::FutureResult(FutureResult<T>&& a_futureresult) : std::future<T>(), future(), taskReference() {
            swap(a_futureresult);
        }

        template<class T>
        FutureResult<T>& FutureResult<T>::operator=(FutureResult<T>&& rhs) {
            this->swap(rhs);
            return *this;
        }

        template<class T>
        T FutureResult<T>::get() {
            return future.get();
        }

        template<class T>
        SharedFutureResult<T> FutureResult<T>::share() {
            return SharedFutureResult<T>(std::move(*this));
        }

        template<class T>
        bool FutureResult<T>::valid() const {
            return future.valid();
        }

        template<class T>
        void FutureResult<T>::wait() const {
            future.wait();
        }

        template<class T>
        template<class Clock, class Duration>
        std::future_status FutureResult<T>::wait_until(const std::chrono::time_point<Clock, Duration>& abs_time) const {
            return future.wait_until(abs_time);
        }

        template<class T>
        template<class Rep, class Period>
        std::future_status FutureResult<T>::wait_for(const std::chrono::duration<Rep, Period>& rel_time) const {
            return future.wait_for(rel_time);
        }

        template<class T>
        bool FutureResult<T>::cancel(bool allowInterrupt) {
           return taskReference->cancel(allowInterrupt);
        }

        template<class T>
        std::shared_ptr<TaskReference> FutureResult<T>::getTaskReference() { return taskReference; }

        template<class T>
        SharedFutureResult<T>::SharedFutureResult() :std::shared_future<T>() {
        }

        template<class T>
        SharedFutureResult<T>::~SharedFutureResult() {
        }

        template<class T>
        void SharedFutureResult<T>::swap(SharedFutureResult<T>& a_sharedfuture) {
            taskReference.swap(a_sharedfuture.taskReference);
            std::swap(sharedFuture, a_sharedfuture.sharedFuture);
            std::swap(*static_cast<std::shared_future<T>*>(this), static_cast<std::shared_future<T>&>(a_sharedfuture));
        }

        template<class T>
        SharedFutureResult<T>::SharedFutureResult(const SharedFutureResult& a_sharedfuture) : std::shared_future<T>(), sharedFuture(a_sharedfuture.sharedFuture), taskReference(a_sharedfuture.taskReference) {
        }

        template<class T>
        SharedFutureResult<T>::SharedFutureResult(SharedFutureResult&& a_sharedfuture) : std::shared_future<T>(), sharedFuture(), taskReference() {
            swap(a_sharedfuture);
        }

        template<class T>
        SharedFutureResult<T>::SharedFutureResult(FutureResult<T>&& a_futureresult) : std::shared_future<T>(), sharedFuture(std::move(a_futureresult.future)), taskReference() {
            taskReference.swap(a_futureresult.taskReference);
        }

        template<class T>
        SharedFutureResult<T>& SharedFutureResult<T>::operator=(SharedFutureResult<T>&& rhs) {
            swap(rhs);
            return *this;
        }

        template<class T>
        SharedFutureResult<T>& SharedFutureResult<T>::operator=(const SharedFutureResult<T>& rhs) {
            *(static_cast<std::shared_future<T>*>(this)) = rhs;
            sharedFuture = rhs.sharedFuture;
            taskReference= rhs.taskReference;
            return *this;
        }

        template<class T>
        decltype(std::declval<std::shared_future<T>>().get()) SharedFutureResult<T>::get() const {
            return sharedFuture.get();
        }

        template<class T>
        bool SharedFutureResult<T>::valid() const {
            return sharedFuture.valid();
        }

        template<class T>
        void SharedFutureResult<T>::wait() const {
            return sharedFuture.wait();
        }

        template<class T>
        template<class Clock, class Duration>
        std::future_status SharedFutureResult<T>::wait_until(const std::chrono::time_point<Clock, Duration>& abs_time) const {
            return sharedFuture.wait_until(abs_time);
        }
        
        template<class T>
        template<class Rep, class Period>
        std::future_status SharedFutureResult<T>::wait_for(const std::chrono::duration<Rep, Period>& rel_time) const {
            return sharedFuture.wait_for(rel_time);
        }
        
        template<class T>
        bool SharedFutureResult<T>::cancel(bool allowInterrupt) {
            return taskReference->cancel(allowInterrupt);
        }
    }
}

#endif //__MEX_FUTURE_IMPL_HPP__