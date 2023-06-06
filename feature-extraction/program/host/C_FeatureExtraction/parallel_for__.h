#pragma once

#include "Range.h"
#include "define.h"
#include <functional>


class ParallelLoopBody_
{
public:
    virtual ~ParallelLoopBody_();
    virtual void operator() (const Range& range) const = 0;
};
ParallelLoopBody_::~ParallelLoopBody_() {}




void parallel_for__(const Range& range, const ParallelLoopBody_& body, double nstripes)
{
#ifdef OPENCV_TRACE
    CV__TRACE_OPENCV_FUNCTION_NAME_("parallel_for", 0);
    CV_TRACE_ARG_VALUE(range_start, "range.start", (int64)range.start);
    CV_TRACE_ARG_VALUE(range_end, "range.end", (int64)range.end);
    CV_TRACE_ARG_VALUE(nstripes, "nstripes", (int64)nstripes);
#endif

    // CV_INSTRUMENT_REGION_MT_FORK();
    if (range.empty())
        return;

#ifdef CV_PARALLEL_FRAMEWORK
    static std::atomic<bool> flagNestedParallelFor(false);
    bool isNotNestedRegion = !flagNestedParallelFor.load();
    if (isNotNestedRegion)
        isNotNestedRegion = !flagNestedParallelFor.exchange(true);
    if (isNotNestedRegion)
    {
        try
        {
            parallel_for_impl(range, body, nstripes);
            flagNestedParallelFor = false;
        }
        catch (...)
        {
            flagNestedParallelFor = false;
            throw;
        }
    }
    else // nested parallel_for_() calls are not parallelized
#endif // CV_PARALLEL_FRAMEWORK
    {
        CV_UNUSED(nstripes);
        body(range);
    }
}

class ParallelLoopBodyLambdaWrapper_ : public ParallelLoopBody_
{
private:
    std::function<void(const Range&)> m_functor;
public:
    ParallelLoopBodyLambdaWrapper_(std::function<void(const Range&)> functor) :
        m_functor(functor)
    { }

    void operator() (const Range& range) const
    {
        m_functor(range);
    }
};


inline void parallel_for__(const Range& range, std::function<void(const Range&)> functor, double nstripes = -1.)
{
    parallel_for__(range, ParallelLoopBodyLambdaWrapper_(functor), nstripes);
}
