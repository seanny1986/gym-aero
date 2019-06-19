/* Copyright 2017 The MathWorks, Inc. */
 
#ifndef CPPSHAREDLIB_FUTURE_HPP
#define CPPSHAREDLIB_FUTURE_HPP

#include <vector>
#include <streambuf>
#include <memory>
#include <future>

namespace matlab {
    namespace cpplib {
        class MATLABLibrary;
    }
}

namespace matlab {

    namespace execution {
        
        using namespace matlab::cpplib;
        
        namespace runtime_future_detail {
            struct Impl {
                uint64_t handle;
                bool state;
                bool cancelled;
                bool interrupted;
                Impl();
                Impl(uint64_t aHandle);
                ~Impl();
            };
        }

        template <>
        class FutureResult<std::unique_ptr<MATLABLibrary>>: public std::future<std::unique_ptr<MATLABLibrary>>{
        public:
            FutureResult(std::shared_ptr<runtime_future_detail::Impl> other);

            FutureResult(std::future<std::unique_ptr<MATLABLibrary>>&& rhs);

            FutureResult(FutureResult<std::unique_ptr<MATLABLibrary>>&& rhs);

            FutureResult<std::unique_ptr<MATLABLibrary>>& operator=(FutureResult<std::unique_ptr<MATLABLibrary>>&& rhs);

            FutureResult();

            ~FutureResult();

            void swap(FutureResult<std::unique_ptr<MATLABLibrary>>& rhs);

            std::unique_ptr<MATLABLibrary> get();

            SharedFutureResult<std::unique_ptr<MATLABLibrary>> share();

            bool valid() const;

            void wait() const;

            template<class Clock, class Duration>
            std::future_status wait_until(const std::chrono::time_point<Clock, Duration>& abs_time) const;
            template<class Rep, class Period>

            std::future_status wait_for(const std::chrono::duration<Rep, Period>& rel_time) const;
           
            bool cancel(bool allowInterrupt = true);

        private:
            FutureResult(std::future<std::unique_ptr<MATLABLibrary>>&) = delete;
            FutureResult(FutureResult&) = delete;
            FutureResult& operator= (FutureResult&) = delete;
            std::future<std::unique_ptr<MATLABLibrary>> future;
            std::shared_ptr<runtime_future_detail::Impl> impl;
            friend SharedFutureResult<std::unique_ptr<MATLABLibrary>>;
        };

        template <>
        class SharedFutureResult<std::unique_ptr<MATLABLibrary>>: public std::shared_future<std::unique_ptr<MATLABLibrary>>{
        public:
            SharedFutureResult();

            ~SharedFutureResult();

            void swap(SharedFutureResult<std::unique_ptr<MATLABLibrary>>& rhs);

            SharedFutureResult(const SharedFutureResult& rhs);

            SharedFutureResult(SharedFutureResult&& rhs);

            SharedFutureResult(FutureResult<std::unique_ptr<MATLABLibrary>>&& rhs);

            SharedFutureResult<std::unique_ptr<MATLABLibrary>>& operator=(SharedFutureResult<std::unique_ptr<MATLABLibrary>>&& rhs);

            SharedFutureResult<std::unique_ptr<MATLABLibrary>>& operator=(const SharedFutureResult<std::unique_ptr<MATLABLibrary>>& rhs);

            const std::unique_ptr<MATLABLibrary>& get() const;

            bool valid() const;

            void wait() const;

            template<class Clock, class Duration>
            std::future_status wait_until(const std::chrono::time_point<Clock, Duration>& abs_time) const;
            
            template<class Rep, class Period>
            std::future_status wait_for(const std::chrono::duration<Rep, Period>& rel_time) const;
            
            bool cancel(bool allowInterrupt = true);

        private:
            std::shared_future<std::unique_ptr<MATLABLibrary>> sharedFuture;
            std::shared_ptr<runtime_future_detail::Impl> impl;
        };
    }
}

namespace matlab {
    namespace execution { 

        namespace runtime_future_detail {
            inline Impl::Impl(uint64_t aHandle) : handle(aHandle), state(true), cancelled(false), interrupted(false) {
            }
            inline Impl::Impl() : handle(0), state(true), cancelled(false), interrupted(false) {
            }
            inline Impl::~Impl(){
            }
        }

        inline FutureResult<std::unique_ptr<MATLABLibrary>>::FutureResult() : std::future<std::unique_ptr<MATLABLibrary>>(), future(), impl() {}

        inline FutureResult<std::unique_ptr<MATLABLibrary>>::FutureResult(std::shared_ptr<runtime_future_detail::Impl> rhs) : std::future<std::unique_ptr<MATLABLibrary>>(), future(), impl(rhs) {
        }

        inline void FutureResult<std::unique_ptr<MATLABLibrary>>::swap(FutureResult<std::unique_ptr<MATLABLibrary>>& rhs) {
            impl.swap(rhs.impl);
            std::swap(future, rhs.future);
            std::swap(*static_cast<std::future<std::unique_ptr<MATLABLibrary>>*>(this), static_cast<std::future<std::unique_ptr<MATLABLibrary>>&>(rhs));
        }

        inline FutureResult<std::unique_ptr<MATLABLibrary>>::FutureResult(std::future<std::unique_ptr<MATLABLibrary>>&& rhs) : std::future<std::unique_ptr<MATLABLibrary>>(), future(std::move(rhs)), impl() {

        }

        inline FutureResult<std::unique_ptr<MATLABLibrary>>::FutureResult(FutureResult<std::unique_ptr<MATLABLibrary>>&& rhs) : std::future<std::unique_ptr<MATLABLibrary>>(), future(), impl() {
            swap(rhs);
        }

        inline FutureResult<std::unique_ptr<MATLABLibrary>>& FutureResult<std::unique_ptr<MATLABLibrary>>::operator=(FutureResult<std::unique_ptr<MATLABLibrary>>&& rhs) {
            swap(rhs);
            return *this;
        }

        inline FutureResult<std::unique_ptr<MATLABLibrary>>::~FutureResult() {
        }
        
        inline std::unique_ptr<MATLABLibrary> FutureResult<std::unique_ptr<MATLABLibrary>>::get() {
            return future.get();
        }

        inline SharedFutureResult<std::unique_ptr<MATLABLibrary>> FutureResult<std::unique_ptr<MATLABLibrary>>::share() {
            return SharedFutureResult<std::unique_ptr<MATLABLibrary>>(std::move(*this));
        }

        inline bool FutureResult<std::unique_ptr<MATLABLibrary>>::valid() const {
            return future.valid();
        }

        inline void FutureResult<std::unique_ptr<MATLABLibrary>>::wait() const {
            return future.wait();
        }

        template<class Clock, class Duration>
        std::future_status FutureResult<std::unique_ptr<MATLABLibrary> >::wait_until(const std::chrono::time_point<Clock, Duration>& abs_time) const {
            return future.wait_until(abs_time);
        }

        template<class Rep, class Period>
        std::future_status FutureResult<std::unique_ptr<MATLABLibrary> >::wait_for(const std::chrono::duration<Rep, Period>& rel_time) const {
            return future.wait_for(rel_time);
        }

        inline bool FutureResult<std::unique_ptr<MATLABLibrary> >::cancel(bool allowInterrupt) {
            /*if (allowInterrupt) {
                std::async(std::launch::async, [this](){ 
                    std::unique_ptr<MATLABLibrary> enginePtr = future.get();
                    enginePtr->disconnect();
                });
                impl->interrupted = true;
                return true;
            }
            impl->interrupted = false;
            impl->cancelled = false;*/
            return false;
        }


        inline SharedFutureResult<std::unique_ptr<MATLABLibrary>>::SharedFutureResult() : std::shared_future<std::unique_ptr<MATLABLibrary>>(), sharedFuture(), impl() {
        }

        inline SharedFutureResult<std::unique_ptr<MATLABLibrary>>::~SharedFutureResult() {
        }

        inline SharedFutureResult<std::unique_ptr<MATLABLibrary>>::SharedFutureResult(const SharedFutureResult& rhs) : std::shared_future<std::unique_ptr<MATLABLibrary>>(), sharedFuture(rhs.sharedFuture), impl(rhs.impl) {
        }

        inline void SharedFutureResult<std::unique_ptr<MATLABLibrary>>::swap(SharedFutureResult<std::unique_ptr<MATLABLibrary>>& rhs) {
            impl.swap(rhs.impl);
            std::swap(sharedFuture, rhs.sharedFuture);
            std::swap(*static_cast<std::shared_future<std::unique_ptr<MATLABLibrary>>*>(this), static_cast<std::shared_future<std::unique_ptr<MATLABLibrary>>&>(rhs));
        }


        inline SharedFutureResult<std::unique_ptr<MATLABLibrary>>::SharedFutureResult(SharedFutureResult&& rhs) : std::shared_future<std::unique_ptr<MATLABLibrary>>(), sharedFuture(), impl() {
            swap(rhs);
        }

        inline SharedFutureResult<std::unique_ptr<MATLABLibrary>>::SharedFutureResult(FutureResult<std::unique_ptr<MATLABLibrary>>&& rhs) : std::shared_future<std::unique_ptr<MATLABLibrary>>(), sharedFuture(std::move(rhs.future)), impl() {
            impl.swap(rhs.impl);
        }

        inline SharedFutureResult<std::unique_ptr<MATLABLibrary>>& SharedFutureResult<std::unique_ptr<MATLABLibrary>>::operator=(SharedFutureResult<std::unique_ptr<MATLABLibrary>>&& rhs) {
            swap(rhs);
            return *this;
        }

        inline SharedFutureResult<std::unique_ptr<MATLABLibrary>>& SharedFutureResult<std::unique_ptr<MATLABLibrary>>::operator=(const SharedFutureResult<std::unique_ptr<MATLABLibrary>>& rhs) {
            *(static_cast<std::shared_future<std::unique_ptr<MATLABLibrary>>*>(this)) = rhs;
            sharedFuture = rhs.sharedFuture;
            impl = rhs.impl;
            return *this;
        }

        inline const std::unique_ptr<MATLABLibrary>& SharedFutureResult<std::unique_ptr<MATLABLibrary>>::get() const {
            return sharedFuture.get();
        }

        inline bool SharedFutureResult<std::unique_ptr<MATLABLibrary>>::valid() const {
            return sharedFuture.valid();
        }

        inline void SharedFutureResult<std::unique_ptr<MATLABLibrary>>::wait() const {
            return sharedFuture.wait();
        }

        template<class Clock, class Duration>
        std::future_status SharedFutureResult<std::unique_ptr<MATLABLibrary>>::wait_until(const std::chrono::time_point<Clock, Duration>& abs_time) const {
            return sharedFuture.wait_until(abs_time);
        }

        template<class Rep, class Period>
        std::future_status SharedFutureResult<std::unique_ptr<MATLABLibrary>>::wait_for(const std::chrono::duration<Rep, Period>& rel_time) const {
            return sharedFuture.wait_for(rel_time);
        }

        inline bool SharedFutureResult<std::unique_ptr<MATLABLibrary>>::cancel(bool allowInterrupt) {
            if (allowInterrupt) {
                //TODO
                //engine_cancel_matlab_async(impl->handle);
                impl->interrupted = true;
                return true;
            }
            impl->interrupted = false;
            impl->cancelled = false;
            return false;
        }

    }
}
#endif // CPPSHAREDLIB_FUTURE_HPP 