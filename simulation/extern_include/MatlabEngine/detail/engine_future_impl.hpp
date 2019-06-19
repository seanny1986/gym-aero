/* Copyright 2017 The MathWorks, Inc. */
 
#ifndef ENGINE_FUTURE_IMPL_HPP
#define ENGINE_FUTURE_IMPL_HPP

#include <vector>
#include <streambuf>
#include <memory>
#include <future>
#include "../engine_future.hpp"
#include "../cpp_engine_api.hpp"
#include "../engine_exception.hpp"

namespace matlab {
    namespace execution {

        using namespace matlab::engine;
        
        inline FutureResult<std::unique_ptr<MATLABEngine>>::FutureResult() :std::future<std::unique_ptr<MATLABEngine>>(), future() {}

        inline void FutureResult<std::unique_ptr<MATLABEngine>>::swap(FutureResult<std::unique_ptr<MATLABEngine>>& a_futureresult) {
            std::swap(future, a_futureresult.future);
            std::swap(*static_cast<std::future<std::unique_ptr<MATLABEngine>>*>(this), static_cast<std::future<std::unique_ptr<MATLABEngine>>&>(a_futureresult));
        }

        inline FutureResult<std::unique_ptr<MATLABEngine>>::FutureResult(std::future<std::unique_ptr<MATLABEngine>>&& a_future):std::future<std::unique_ptr<MATLABEngine>>(), future(std::move(a_future)) {

        }

        inline FutureResult<std::unique_ptr<MATLABEngine>>::FutureResult(FutureResult<std::unique_ptr<MATLABEngine>>&& rhs):std::future<std::unique_ptr<MATLABEngine>>(), future() {
            swap(rhs);
        }

        inline FutureResult<std::unique_ptr<MATLABEngine>>& FutureResult<std::unique_ptr<MATLABEngine>>::operator=(FutureResult<std::unique_ptr<MATLABEngine>>&& rhs) {
            swap(rhs);
            return *this;
        }

        inline FutureResult<std::unique_ptr<MATLABEngine>>::~FutureResult() {
        }
        
        inline std::unique_ptr<MATLABEngine> FutureResult<std::unique_ptr<MATLABEngine>>::get() {
            return future.get();
        }

        inline SharedFutureResult<std::unique_ptr<MATLABEngine>> FutureResult<std::unique_ptr<MATLABEngine>>::share() {
            return SharedFutureResult<std::unique_ptr<MATLABEngine>>(std::move(*this));
        }

        inline bool FutureResult<std::unique_ptr<MATLABEngine>>::valid() const {
            return future.valid();
        }

        inline void FutureResult<std::unique_ptr<MATLABEngine>>::wait() const {
            return future.wait();
        }

        template<class Clock, class Duration>
        std::future_status FutureResult<std::unique_ptr<MATLABEngine>>::wait_until(const std::chrono::time_point<Clock, Duration>& abs_time) const {
            return future.wait_until(abs_time);
        }

        template<class Rep, class Period>
        std::future_status FutureResult<std::unique_ptr<MATLABEngine>>::wait_for(const std::chrono::duration<Rep, Period>& rel_time) const {
            return future.wait_for(rel_time);
        }

        inline bool FutureResult<std::unique_ptr<MATLABEngine>>::cancel(bool allowInterrupt) {
            return false;
        }


        inline SharedFutureResult<std::unique_ptr<MATLABEngine>>::SharedFutureResult():std::shared_future<std::unique_ptr<MATLABEngine>>(), sharedFuture() {
        }

        inline SharedFutureResult<std::unique_ptr<MATLABEngine>>::~SharedFutureResult() {
        }

        inline SharedFutureResult<std::unique_ptr<MATLABEngine>>::SharedFutureResult(const SharedFutureResult& a_sharedfuture):std::shared_future<std::unique_ptr<MATLABEngine>>(), sharedFuture(a_sharedfuture.sharedFuture) {
        }

        inline void SharedFutureResult<std::unique_ptr<MATLABEngine>>::swap(SharedFutureResult<std::unique_ptr<MATLABEngine>>& a_sharedfuture) {
            std::swap(sharedFuture, a_sharedfuture.sharedFuture);
            std::swap(*static_cast<std::shared_future<std::unique_ptr<MATLABEngine>>*>(this), static_cast<std::shared_future<std::unique_ptr<MATLABEngine>>&>(a_sharedfuture));
        }


        inline SharedFutureResult<std::unique_ptr<MATLABEngine>>::SharedFutureResult(SharedFutureResult&& a_sharedfuture):std::shared_future<std::unique_ptr<MATLABEngine>>(), sharedFuture() {
            swap(a_sharedfuture);
        }

        inline SharedFutureResult<std::unique_ptr<MATLABEngine>>::SharedFutureResult(FutureResult<std::unique_ptr<MATLABEngine>>&& a_futureresult):std::shared_future<std::unique_ptr<MATLABEngine>>(), sharedFuture(std::move(a_futureresult.future)) {
        }

        inline SharedFutureResult<std::unique_ptr<MATLABEngine>>& SharedFutureResult<std::unique_ptr<MATLABEngine>>::operator=(SharedFutureResult<std::unique_ptr<MATLABEngine>>&& rhs) {
            swap(rhs);
            return *this;
        }

        inline SharedFutureResult<std::unique_ptr<MATLABEngine>>& SharedFutureResult<std::unique_ptr<MATLABEngine>>::operator=(const SharedFutureResult<std::unique_ptr<MATLABEngine>>& rhs) {
            *(static_cast<std::shared_future<std::unique_ptr<MATLABEngine>>*>(this)) = rhs;
            sharedFuture = rhs.sharedFuture;
            return *this;
        }

        inline const std::unique_ptr<MATLABEngine>& SharedFutureResult<std::unique_ptr<MATLABEngine>>::get() const {
            return sharedFuture.get();
        }

        inline bool SharedFutureResult<std::unique_ptr<MATLABEngine>>::valid() const {
            return sharedFuture.valid();
        }

        inline void SharedFutureResult<std::unique_ptr<MATLABEngine>>::wait() const {
            return sharedFuture.wait();
        }

        template<class Clock, class Duration>
        std::future_status SharedFutureResult<std::unique_ptr<MATLABEngine>>::wait_until(const std::chrono::time_point<Clock, Duration>& abs_time) const {
            return sharedFuture.wait_until(abs_time);
        }

        template<class Rep, class Period>
        std::future_status SharedFutureResult<std::unique_ptr<MATLABEngine>>::wait_for(const std::chrono::duration<Rep, Period>& rel_time) const {
            return sharedFuture.wait_for(rel_time);
        }

        inline bool SharedFutureResult<std::unique_ptr<MATLABEngine>>::cancel(bool allowInterrupt) {
            return false;
        }

    }
}
#endif /* ENGINE_FUTURE_IMPL_HPP */