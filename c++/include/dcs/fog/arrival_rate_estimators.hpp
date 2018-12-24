/* vim: set tabstop=4 expandtab shiftwidth=4 softtabstop=4: */

/**
 * \file dcs/fog/arrival_rate_estimators.hpp
 *
 * \brief Estimators for arrival rates.
 *
 * \author Marco Guazzone (marco.guazzone@gmail.com)
 *  
 * <hr/>
 *  
 * Copyright 2017 Marco Guazzone (marco.guazzone@gmail.com)
 *  
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *  
 *     http://www.apache.org/licenses/LICENSE-2.0
 *  
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef DCS_FOG_ARRIVAL_RATE_ESTIMATORS_HPP
#define DCS_FOG_ARRIVAL_RATE_ESTIMATORS_HPP


#include <algorithm>
#include <dcs/fog/random.hpp>
#include <dcs/macro.hpp>
#include <limits>
#include <random>


#define DCS_FOG_ARRIVAL_RATE_ESTIMATORS_MAKE_CLONE(B,D) \
    B* clone() const \
    { \
        return new D(static_cast<const D&>(*this)); \
    }

namespace dcs { namespace fog {

//template <typename BaseT, typename DerivedT>
//struct clonable_t: public BaseT
//{
//    BaseT* clone() const
//    {
//        return new DerivedT(static_cast<const DerivedT&>(*this));
//    }
//}; // clonable_t

template <typename RealT>
struct arrival_rate_estimator_t
{
    virtual ~arrival_rate_estimator_t() { }

    virtual void collect(RealT rate) = 0;

    virtual RealT estimate() = 0;

    virtual void reset() = 0;

    virtual arrival_rate_estimator_t<RealT>* clone() const = 0;
}; // arrival_rate_estimator_t


template <typename RealT>
class max_arrival_rate_estimator_t: public arrival_rate_estimator_t<RealT>
{
public:
    max_arrival_rate_estimator_t()
    : max_rate_(0)
    {
    }

    virtual void collect(RealT rate)
    {
        if (rate > max_rate_)
        {
            max_rate_ = rate;
        }
    }

    virtual RealT estimate()
    {
        return max_rate_;
    }

    virtual void reset()
    {
        max_rate_ = 0;
    }

    DCS_FOG_ARRIVAL_RATE_ESTIMATORS_MAKE_CLONE(arrival_rate_estimator_t<RealT>, max_arrival_rate_estimator_t<RealT>)

private:
    RealT max_rate_;
}; // max_arrival_rate_estimator_t


template <typename RealT>
class perturbed_max_arrival_rate_estimator_t: public max_arrival_rate_estimator_t<RealT>
{
private:
    typedef max_arrival_rate_estimator_t<RealT> base_type;


public:
    static constexpr RealT default_mean = 0;
    static constexpr RealT default_standard_deviation = 1;


public:
    explicit perturbed_max_arrival_rate_estimator_t(random_number_engine_t& rng, RealT mu = default_mean, RealT sd = default_standard_deviation)
    : rng_(rng),
      white_noise_rvg_(mu, sd)
    {
    }

    virtual RealT estimate()
    {
        RealT max_rate = base_type::estimate();
        return std::max(0.0, max_rate*(1.0+white_noise_rvg_(rng_)));
    }

    DCS_FOG_ARRIVAL_RATE_ESTIMATORS_MAKE_CLONE(max_arrival_rate_estimator_t<RealT>, perturbed_max_arrival_rate_estimator_t<RealT>)

private:
    random_number_engine_t& rng_; ///< Engine fo random number generation
    std::normal_distribution<RealT> white_noise_rvg_; ///< White noise percentange error
}; // perturbed_max_arrival_rate_estimator_t
template <typename RealT>
constexpr RealT perturbed_max_arrival_rate_estimator_t<RealT>::default_mean;
template <typename RealT>
constexpr RealT perturbed_max_arrival_rate_estimator_t<RealT>::default_standard_deviation;


template <typename RealT>
class uniform_max_arrival_rate_estimator_t: public max_arrival_rate_estimator_t<RealT>
{
private:
    typedef max_arrival_rate_estimator_t<RealT> base_type;


public:
    explicit uniform_max_arrival_rate_estimator_t(random_number_engine_t& rng)
    : rng_(rng)
    {
    }

    virtual RealT estimate()
    {
        RealT max_rate = base_type::estimate();
        return std::uniform_real_distribution<RealT>(0, max_rate)(rng_);
    }

    DCS_FOG_ARRIVAL_RATE_ESTIMATORS_MAKE_CLONE(max_arrival_rate_estimator_t<RealT>, uniform_max_arrival_rate_estimator_t<RealT>)

private:
    random_number_engine_t& rng_; ///< Engine fo random number generation
}; // uniform_max_arrival_rate_estimator_t


template <typename RealT>
class uniform_min_max_arrival_rate_estimator_t: public arrival_rate_estimator_t<RealT>
{
public:
    explicit uniform_min_max_arrival_rate_estimator_t(random_number_engine_t& rng)
    : rng_(rng),
      min_rate_(+std::numeric_limits<RealT>::infinity()),
      max_rate_(0)
    {
    }

    void collect(RealT rate)
    {
        if (rate > max_rate_)
        {
            max_rate_ = rate;
        }
        if (rate < min_rate_)
        {
            min_rate_ = rate;
        }
    }

    virtual RealT estimate()
    {
        return std::uniform_real_distribution<RealT>(std::min(min_rate_, max_rate_), max_rate_)(rng_);
    }

    void reset()
    {
        min_rate_ = +std::numeric_limits<RealT>::infinity();
        max_rate_ = 0;
    }

    DCS_FOG_ARRIVAL_RATE_ESTIMATORS_MAKE_CLONE(arrival_rate_estimator_t<RealT>, uniform_min_max_arrival_rate_estimator_t<RealT>)

private:
    random_number_engine_t& rng_; ///< Engine fo random number generation
    RealT min_rate_;
    RealT max_rate_;
}; // uniform_min_max_arrival_rate_estimator_t


template <typename RealT>
class most_recently_observed_arrival_rate_estimator_t: public arrival_rate_estimator_t<RealT>
{
public:
    explicit most_recently_observed_arrival_rate_estimator_t()
    : mro_(0)
    {
    }

    void collect(RealT rate)
    {
        mro_ = rate;
    }

    virtual RealT estimate()
    {
        return mro_;
    }

    void reset()
    {
        mro_ = 0;
    }

    DCS_FOG_ARRIVAL_RATE_ESTIMATORS_MAKE_CLONE(arrival_rate_estimator_t<RealT>, most_recently_observed_arrival_rate_estimator_t<RealT>)

private:
    RealT mro_;
}; // most_recently_observed_arrival_rate_estimator_t


template <typename RealT>
class perturbed_most_recently_observed_arrival_rate_estimator_t: public most_recently_observed_arrival_rate_estimator_t<RealT>
{
private:
    typedef most_recently_observed_arrival_rate_estimator_t<RealT> base_type;


public:
    static constexpr RealT default_mean = 0;
    static constexpr RealT default_standard_deviation = 1;


public:
    explicit perturbed_most_recently_observed_arrival_rate_estimator_t(random_number_engine_t& rng, RealT mu = default_mean, RealT sd = default_standard_deviation)
    : rng_(rng),
      white_noise_rvg_(mu, sd)
    {
    }

    virtual RealT estimate()
    {
        RealT mro_rate = base_type::estimate();
        //return std::max(0.0, mro_rate*(1.0+white_noise_rvg_(rng_)));
        auto const err = white_noise_rvg_(rng_);
        auto const new_rate = std::max(0.0, mro_rate*(1.0+err));
DCS_DEBUG_TRACE("ARRIVAL RATE ESTIMATION: " << mro_rate << " -> " << new_rate << " (error: " << err << ", mean: " << white_noise_rvg_.mean() << ", sd: " << white_noise_rvg_.stddev() << ")");
        return new_rate;
    }

    DCS_FOG_ARRIVAL_RATE_ESTIMATORS_MAKE_CLONE(most_recently_observed_arrival_rate_estimator_t<RealT>, perturbed_most_recently_observed_arrival_rate_estimator_t<RealT>)

private:
    random_number_engine_t& rng_; ///< Engine fo random number generation
    std::normal_distribution<RealT> white_noise_rvg_; ///< White noise percentange error
}; // perturbed_most_recently_observed_arrival_rate_estimator_t
template <typename RealT>
constexpr RealT perturbed_most_recently_observed_arrival_rate_estimator_t<RealT>::default_mean;
template <typename RealT>
constexpr RealT perturbed_most_recently_observed_arrival_rate_estimator_t<RealT>::default_standard_deviation;


template <typename RealT>
class ewma_arrival_rate_estimator_t: public arrival_rate_estimator_t<RealT>
{
public:
    static constexpr RealT default_smoothing_factor = 0.95;


public:
    explicit ewma_arrival_rate_estimator_t(RealT smooth_factor = default_smoothing_factor)
    : smooth_factor_(smooth_factor),
      ewma_(0),
      first_(true)
    {
    }

    void collect(RealT rate)
    {
        if (first_)
        {
            ewma_ = rate;
            first_ = false;
        }
        else
        {
            ewma_ = smooth_factor_*rate + (1-smooth_factor_)*ewma_;
        }
    }

    virtual RealT estimate()
    {
        return ewma_;
    }

    void reset()
    {
        ewma_ = 0;
        first_ = true;
    }

    DCS_FOG_ARRIVAL_RATE_ESTIMATORS_MAKE_CLONE(arrival_rate_estimator_t<RealT>, ewma_arrival_rate_estimator_t<RealT>)

private:
    RealT smooth_factor_;
    RealT ewma_;
    bool first_;
}; // ewma_arrival_rate_estimator_t


template <typename RealT>
class beta_arrival_rate_estimator_t: public arrival_rate_estimator_t<RealT>
{
public:
    static constexpr RealT default_shape1 = 1;
    static constexpr RealT default_shape2 = 1;
    static constexpr RealT default_lower_bound = 0;
    static constexpr RealT default_upper_bound = 1;


public:
    explicit beta_arrival_rate_estimator_t(random_number_engine_t& rng, RealT shape1 = default_shape1, RealT shape2 = default_shape2, RealT lower = default_lower_bound, RealT upper = default_upper_bound)
    : rng_(rng),
      beta_(shape1, shape2, lower, upper)
    {
    }

    void collect(RealT rate)
    {
        DCS_MACRO_SUPPRESS_UNUSED_VARIABLE_WARNING( rate );
    }

    virtual RealT estimate()
    {
        return beta_(rng_);
    }

    void reset()
    {
        // empty
    }

    DCS_FOG_ARRIVAL_RATE_ESTIMATORS_MAKE_CLONE(arrival_rate_estimator_t<RealT>, beta_arrival_rate_estimator_t<RealT>)

private:
    random_number_engine_t& rng_; ///< Engine fo random number generation
    beta_distribution<RealT> beta_; ///< Random variate generator for the Beta distribution
}; // uniform_min_max_arrival_rate_estimator_t

}} // Namespace dcs::fog


#endif // DCS_FOG_ARRIVAL_RATE_ESTIMATORS_HPP
