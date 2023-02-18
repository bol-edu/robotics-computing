#pragma once
//#include <opencv2/opencv.hpp>

using namespace std;




class  RNG__
{
public:
    enum {
        UNIFORM = 0,
        NORMAL = 1
    };

    /** @brief constructor

    These are the RNG constructors. The first form sets the state to some
    pre-defined value, equal to 2\*\*32-1 in the current implementation. The
    second form sets the state to the specified value. If you passed state=0
    , the constructor uses the above default value instead to avoid the
    singular random number sequence, consisting of all zeros.
    */
    RNG__();
    /** @overload
    @param state 64-bit value used to initialize the RNG.
    */
    RNG__(uint64 state);
    /**The method updates the state using the MWC algorithm and returns the
    next 32-bit random number.*/
    unsigned next();

    /**Each of the methods updates the state using the MWC algorithm and
    returns the next random number of the specified type. In case of integer
    types, the returned number is from the available value range for the
    specified type. In case of floating-point types, the returned value is
    from [0,1) range.
    */
    operator uchar();
    /** @overload */
    operator schar();
    /** @overload */
    operator ushort();
    /** @overload */
    operator short();
    /** @overload */
    operator unsigned();
    /** @overload */
    operator int();
    /** @overload */
    operator float();
    /** @overload */
    operator double();

    /** @brief returns a random integer sampled uniformly from [0, N).

    The methods transform the state using the MWC algorithm and return the
    next random number. The first form is equivalent to RNG::next . The
    second form returns the random number modulo N , which means that the
    result is in the range [0, N) .
    */
    unsigned operator ()();
    /** @overload
    @param N upper non-inclusive boundary of the returned random number.
    */
    unsigned operator ()(unsigned N);

    /** @brief returns uniformly distributed integer random number from [a,b) range

    The methods transform the state using the MWC algorithm and return the
    next uniformly-distributed random number of the specified type, deduced
    from the input parameter type, from the range [a, b) . There is a nuance
    illustrated by the following sample:

    @code{.cpp}
    RNG rng;

    // always produces 0
    double a = rng.uniform(0, 1);

    // produces double from [0, 1)
    double a1 = rng.uniform((double)0, (double)1);

    // produces float from [0, 1)
    float b = rng.uniform(0.f, 1.f);

    // produces double from [0, 1)
    double c = rng.uniform(0., 1.);

    // may cause compiler error because of ambiguity:
    //  RNG::uniform(0, (int)0.999999)? or RNG::uniform((double)0, 0.99999)?
    double d = rng.uniform(0, 0.999999);
    @endcode

    The compiler does not take into account the type of the variable to
    which you assign the result of RNG::uniform . The only thing that
    matters to the compiler is the type of a and b parameters. So, if you
    want a floating-point random number, but the range boundaries are
    integer numbers, either put dots in the end, if they are constants, or
    use explicit type cast operators, as in the a1 initialization above.
    @param a lower inclusive boundary of the returned random number.
    @param b upper non-inclusive boundary of the returned random number.
    */
    int uniform(int a, int b);
    /** @overload */
    float uniform(float a, float b);
    /** @overload */
    double uniform(double a, double b);

    /** @brief Fills arrays with random numbers.

    @param mat 2D or N-dimensional matrix; currently matrices with more than
    4 channels are not supported by the methods, use Mat::reshape as a
    possible workaround.
    @param distType distribution type, RNG::UNIFORM or RNG::NORMAL.
    @param a first distribution parameter; in case of the uniform
    distribution, this is an inclusive lower boundary, in case of the normal
    distribution, this is a mean value.
    @param b second distribution parameter; in case of the uniform
    distribution, this is a non-inclusive upper boundary, in case of the
    normal distribution, this is a standard deviation (diagonal of the
    standard deviation matrix or the full standard deviation matrix).
    @param saturateRange pre-saturation flag; for uniform distribution only;
    if true, the method will first convert a and b to the acceptable value
    range (according to the mat datatype) and then will generate uniformly
    distributed random numbers within the range [saturate(a), saturate(b)),
    if saturateRange=false, the method will generate uniformly distributed
    random numbers in the original range [a, b) and then will saturate them,
    it means, for example, that
    <tt>theRNG().fill(mat_8u, RNG::UNIFORM, -DBL_MAX, DBL_MAX)</tt> will likely
    produce array mostly filled with 0's and 255's, since the range (0, 255)
    is significantly smaller than [-DBL_MAX, DBL_MAX).

    Each of the methods fills the matrix with the random values from the
    specified distribution. As the new numbers are generated, the RNG state
    is updated accordingly. In case of multiple-channel images, every
    channel is filled independently, which means that RNG cannot generate
    samples from the multi-dimensional Gaussian distribution with
    non-diagonal covariance matrix directly. To do that, the method
    generates samples from multi-dimensional standard Gaussian distribution
    with zero mean and identity covariation matrix, and then transforms them
    using transform to get samples from the specified Gaussian distribution.
    */
    //void fill(InputOutputArray mat, int distType, InputArray a, InputArray b, bool saturateRange = false);

    /** @brief Returns the next random number sampled from the Gaussian distribution
    @param sigma standard deviation of the distribution.

    The method transforms the state using the MWC algorithm and returns the
    next random number from the Gaussian distribution N(0,sigma) . That is,
    the mean value of the returned random numbers is zero and the standard
    deviation is the specified sigma .
    */
    double gaussian(double sigma);

    uint64 state;

    bool operator ==(const RNG__& other) const;
};



inline RNG__::RNG__(uint64 _state) { state = _state ? _state : 0xffffffff; }

inline int RNG__::uniform(int a, int b) { return a == b ? a : (int)(next() % (b - a) + a); }

inline unsigned RNG__::next()
{
    state = (uint64)(unsigned)state * /*CV_RNG_COEFF*/ 4164903690U + (unsigned)(state >> 32);
    return (unsigned)state;
}