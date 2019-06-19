/* Copyright 2014-2018 The MathWorks, Inc. */

#ifndef MDA_ARRAY_HPP_
#define MDA_ARRAY_HPP_

#ifdef matrix_h
#error Using MATLAB Data API with C Matrix API is not supported.
#endif

#include "matlab_data_array_defs.hpp"
#include "matlab_extdata_defs.hpp"
#include "ArrayDimensions.hpp"
#include "ArrayType.hpp"
#include "ArrayElementRef.hpp"
#include "MemoryLayout.hpp"

#include "detail/publish_util.hpp"
#include "detail/FunctionType.hpp"
#include "detail/HelperFunctions.hpp"

#include <functional>
#include <stdint.h>
#include <exception>


namespace matlab {
namespace data {

template <template <class> class IteratorType, class ElementType>
class Range;
template <typename T>
class TypedIterator;

namespace impl {
class ArrayImpl;
}
namespace detail {
class Access;
}

/**
 * Array provides an API to access general information about an
 * array.  Specific subclasses will provide access to type-specific
 * data for different kinds of arrays.
 */
class Array {
  public:
    static const ArrayType type = ArrayType::UNKNOWN;

    /**
     * Default constructor - creates an empty Array
     * @return - newly constructed Array
     * @throw - none
     */
    Array() MW_NOEXCEPT {
        typedef impl::ArrayImpl* (*ArrayCreateEmptyFcnPtr)();
        static const ArrayCreateEmptyFcnPtr fcn = detail::resolveFunction<ArrayCreateEmptyFcnPtr>(
            detail::FunctionType::ARRAY_CREATE_EMPTY);
        pImpl = std::shared_ptr<impl::ArrayImpl>(fcn(), [](impl::ArrayImpl* ptr) {
            typedef void (*ArrayDestroyFcnPtr)(impl::ArrayImpl*);
            static const ArrayDestroyFcnPtr fcn2 =
                detail::resolveFunction<ArrayDestroyFcnPtr>(detail::FunctionType::ARRAY_DESTROY);
            fcn2(ptr);
        });
    }

    /**
     * Destructor
     *
     * @throw none
     */
    virtual ~Array() MW_NOEXCEPT {
    }

    /**
     * Move constructor
     *
     * @param - rhs Array value to be moved
     * @return - newly constructed Array
     * @throw none
     */
    Array(Array&& rhs) MW_NOEXCEPT : pImpl(std::move(rhs.pImpl)) {
    }

    /**
     * Move assignment operator
     *
     * @param - rhs Array value to be moved
     * @return - the updated Array
     * @throw none
     */
    Array& operator=(Array&& rhs) MW_NOEXCEPT {
        pImpl = std::move(rhs.pImpl);
        return *this;
    }

    /**
     * Assignment operator. The updated Array becomes a shared copy of the input Array
     *
     * @param - rhs Array value to be copied
     * @return - the updated Array
     * @throw none
     */
    Array& operator=(const Array& rhs) MW_NOEXCEPT {
        pImpl = rhs.pImpl;
        return *this;
    }

    /**
     * Copy consructor. The newly constructed Array becomes a shared copy of the input Array
     *
     * @param - rhs Array value to be copied
     * @return - new constructed Array
     * @throw none
     */
    Array(const Array& rhs) MW_NOEXCEPT : pImpl(rhs.pImpl) {
    }

    /**
     * Returns the ArrayType of the array
     *
     * @return - the ArrayType
     * @throw InvalidArrayType - if array type returned is not recognized as valid
     */
    ArrayType getType() const {
        typedef int (*ArrayGetTypeFcnPtr)(impl::ArrayImpl*, int*);
        static const ArrayGetTypeFcnPtr fcn =
            detail::resolveFunction<ArrayGetTypeFcnPtr>(detail::FunctionType::ARRAY_GET_TYPE);
        int type;
        detail::throwIfError(fcn(pImpl.get(), &type));
        return static_cast<ArrayType>(type);
    }

    /**
     * Returns the memory layout of the array
     *
     * @return - the MemroyLayout
     * @throw InvalidMemoryLayout - if array type returned is not recognized as valid
     */
    MemoryLayout getMemoryLayout() const {
        typedef int (*ArrayGetMemoryLayoutFcnPtr)(impl::ArrayImpl*, int*);
        static const ArrayGetMemoryLayoutFcnPtr fcn =
            detail::resolveFunction<ArrayGetMemoryLayoutFcnPtr>(
                detail::FunctionType::ARRAY_GET_MEMORY_LAYOUT);
        int layout;
        detail::throwIfError(fcn(pImpl.get(), &layout));
        return static_cast<MemoryLayout>(layout);
    }

    /**
     * Get the array's dimensions.
     *
     * @return ArrayDimensions vector of each dimension.
     * @throw none
     */
    ArrayDimensions getDimensions() const MW_NOEXCEPT {
        size_t numDims = 0;
        size_t* dims = nullptr;
        typedef void (*ArrayGetDimensionsFcnPtr)(impl::ArrayImpl*, size_t*, size_t**);
        static const ArrayGetDimensionsFcnPtr fcn =
            detail::resolveFunction<ArrayGetDimensionsFcnPtr>(
                detail::FunctionType::ARRAY_GET_DIMENSIONS);
        fcn(pImpl.get(), &numDims, &dims);
        return ArrayDimensions(dims, dims + numDims);
    }

    /**
     * Get the number of elements in this array
     *
     * @return the number of elements in the array
     * @throw none
     */
    size_t getNumberOfElements() const MW_NOEXCEPT {
        typedef size_t (*ArrayGetNumElementsFcnPtr)(impl::ArrayImpl*);
        static const ArrayGetNumElementsFcnPtr fcn =
            detail::resolveFunction<ArrayGetNumElementsFcnPtr>(
                detail::FunctionType::ARRAY_GET_NUM_ELEMENTS);
        return fcn(pImpl.get());
    }

    /**
     * Determine if this is an empty array
     *
     * @return true if the array is empty
     * @throw none
     */
    bool isEmpty() const MW_NOEXCEPT {
        typedef bool (*ArrayIsEmptyFcnPtr)(impl::ArrayImpl*);
        static const ArrayIsEmptyFcnPtr fcn =
            detail::resolveFunction<ArrayIsEmptyFcnPtr>(detail::FunctionType::ARRAY_IS_EMPTY);
        return fcn(pImpl.get());
    }

    /**
     * Enables [] indexing on a array.
     *
     * The return value ArrayElementRef<false> allows the element of the array to be
     * modified or retrieved: For example:
     *     arr[1][1] = 5.5;
     *     double val = arr[0][3];
     *
     * @param idx - the first array index
     * @return ArrayElementRef<false> - contains the index specified
     * @throw InvalidArrayIndexException - if the index is invalid
     * @throw CantIndexIntoEmptyArrayException - if the array is empty
     */
    ArrayElementRef<false> operator[](size_t idx) {
        impl::ArrayImpl* newImpl = nullptr;
        typedef bool (*ArrayUnshareFcnPtr)(impl::ArrayImpl*, bool, impl::ArrayImpl**);
        static const ArrayUnshareFcnPtr fcn =
            detail::resolveFunction<ArrayUnshareFcnPtr>(detail::FunctionType::ARRAY_UNSHARE);
        if (fcn(pImpl.get(), pImpl.unique(), &newImpl)) {
            pImpl.reset(newImpl, [](impl::ArrayImpl* ptr) {
                typedef void (*ArrayDestroyFcnPtr)(impl::ArrayImpl*);
                static const ArrayDestroyFcnPtr fcn2 = detail::resolveFunction<ArrayDestroyFcnPtr>(
                    detail::FunctionType::ARRAY_DESTROY);
                fcn2(ptr);
            });
        }
        detail::ReferenceImpl* impl = nullptr;
        typedef int (*ArrayCreateReferenceFcnPtr)(impl::ArrayImpl*, size_t,
                                                  detail::ReferenceImpl**);
        static const ArrayCreateReferenceFcnPtr fcn3 =
            detail::resolveFunction<ArrayCreateReferenceFcnPtr>(
                detail::FunctionType::ARRAY_CREATE_REFERENCE);
        detail::throwIfError(fcn3(pImpl.get(), idx, &impl));
        return detail::Access::createObj<ArrayElementRef<false>>(impl);
    }

    /**
     * Enables [] indexing on a const array.
     *
     * The return value ArrayElementRef<true> allows the element of the array to be
     * retrieved, but not modified: For example:
     *     double val = arr[0][3];
     *
     * @param idx - the first array index
     * @return ArrayElementRef<true> - contains the index specified
     * @throw InvalidArrayIndexException - if the index is invalid
     * @throw CantIndexIntoEmptyArrayException - if the array is empty
     */
    ArrayElementRef<true> operator[](size_t idx) const {
        detail::ReferenceImpl* impl = nullptr;
        typedef int (*ArrayCreateReferenceFcnPtr)(impl::ArrayImpl*, size_t,
                                                  detail::ReferenceImpl**);
        static const ArrayCreateReferenceFcnPtr fcn =
            detail::resolveFunction<ArrayCreateReferenceFcnPtr>(
                detail::FunctionType::ARRAY_CREATE_REFERENCE);
        detail::throwIfError(fcn(pImpl.get(), idx, &impl));
        return detail::Access::createObj<ArrayElementRef<true>>(impl);
    }

  protected:
    Array(impl::ArrayImpl* impl) MW_NOEXCEPT
        : pImpl(std::shared_ptr<impl::ArrayImpl>(impl, [](impl::ArrayImpl* ptr) {
            typedef void (*ArrayDestroyFcnPtr)(impl::ArrayImpl*);
            static const ArrayDestroyFcnPtr fcn2 =
                detail::resolveFunction<ArrayDestroyFcnPtr>(detail::FunctionType::ARRAY_DESTROY);
            fcn2(ptr);
        })) {
    }

    std::shared_ptr<impl::ArrayImpl> pImpl;

    friend class detail::Access;

    template <typename T>
    friend Range<TypedIterator, T> getWritableElements(Array& a);
};
} // namespace data
} // namespace matlab

#endif
