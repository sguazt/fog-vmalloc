/* vim: set tabstop=4 expandtab shiftwidth=4 softtabstop=4: */

/**
 * \file dcs/fog/random.hpp
 *
 * \brief Utility for random number generation.
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

#ifndef DCS_FOG_RANDOM_HPP
#define DCS_FOG_RANDOM_HPP


#include <boost/random/beta_distribution.hpp>
#include <random>


namespace dcs { namespace fog {

class random_number_engine_t
{
private:
    //typedef std::defatul_random_engine eng_impl_type;
    typedef std::mt19937 eng_impl_type;


public:
    typedef typename eng_impl_type::result_type result_type;

    constexpr static result_type default_seed = eng_impl_type::default_seed;


public:
    explicit random_number_engine_t(result_type seed = default_seed)
    : eng_(seed)
    {
    }

    virtual ~random_number_engine_t()
    {
        // Empty
    }

    /// Initialize the random engine with the given seed
    void seed(result_type value = default_seed)
    {
        eng_.seed(value);
    }

    /// Generate a new random number
    result_type operator()()
    {
        return eng_();
    }

    static result_type constexpr min()
    {
        return eng_impl_type::min();
    }

    static result_type constexpr max()
    {
        return eng_impl_type::max();
    }

private:
    eng_impl_type eng_;
}; // random_number_engine_t


template <typename T>
class degenerate_distribution
{
public:
	typedef T result_type;

	struct param_type
	{
		public:
			typedef degenerate_distribution<result_type> distribution_type;
			explicit param_type(result_type v = result_type())
			: v_(v)
			{
			}

			result_type value() const
			{
				return v_;
			}

			friend bool operator==(const param_type& lhs, const param_type& rhs)
			{
				return lhs.v_ == rhs.v_;
			}

			friend bool operator!=(const param_type& lhs, const param_type& rhs)
			{
				return !(lhs == rhs);
			}

		private:
			T v_;
	};

public:
	explicit degenerate_distribution(T v = T())
	: p_(v)
	{
	}

	explicit degenerate_distribution(const param_type& p)
	: p_(p)
	{
	}

	void reset() { }

	result_type value() const { return p_.value(); }

	param_type param() const { return p_; }

	void param(const param_type& p) { p_ = p; }

	result_type min() const { return p_.value(); }

	result_type max() const { return p_.value(); }

	template <typename URNGT>
	result_type operator()(URNGT& rng)
	{
		(void) rng;
		return p_.value();
	}

	friend bool operator==(const degenerate_distribution& lhs, const degenerate_distribution& rhs)
	{
		return lhs.p_ == rhs.p_;
	}

	friend bool operator!=(const degenerate_distribution& lhs, const degenerate_distribution& rhs)
	{
		return !(lhs == rhs);
	}

private:
	param_type p_;
}; // degenerate_distribution


/**
 * \brief Beta distribution with four parameteres.
 *
 * This is the general form of the Beta distribution with 4 parameters: two shape parameters, lower bound and upper bound.
 */
template <typename T>
class beta_distribution
{
public:
	typedef T result_type;

	struct param_type
	{
		public:
			typedef beta_distribution<result_type> distribution_type;
			explicit param_type(result_type alpha = 1, result_type beta = 1, result_type lower = 0, result_type upper = 1)
			: alpha_(alpha),
              beta_(beta),
              lower_(lower),
              upper_(upper)
			{
			}

			result_type alpha() const
			{
				return alpha_;
			}

			result_type beta() const
			{
				return beta;
			}

			result_type lower() const
			{
				return lower_;
			}

			result_type upper() const
			{
				return upper_;
			}

			friend bool operator==(const param_type& lhs, const param_type& rhs)
			{
				return  lhs.alpha_ == rhs.alpha_
                     && lhs.beta_ == rhs.beta_
                     && lhs.lower_ == rhs.lower_
                     && lhs.upper_ == rhs.upper_;
			}

			friend bool operator!=(const param_type& lhs, const param_type& rhs)
			{
				return !(lhs == rhs);
			}

		private:
			T alpha_;
			T beta_;
			T lower_;
			T upper_;
	};

public:
	explicit beta_distribution(T alpha = 1, T beta = 1, T lower = 0, T upper = 1)
	: p_(alpha, beta, lower, upper),
      beta01_(alpha, beta)
	{
	}

	explicit beta_distribution(const param_type& p)
	: p_(p),
      beta01_(p.alpha(), p.beta())
	{
	}

	void reset() { }

	result_type alpha() const { return p_.alpha(); }

	result_type beta() const { return p_.beta(); }

	result_type lower() const { return p_.lower(); }

	result_type upper() const { return p_.upper(); }

	param_type param() const { return p_; }

	void param(const param_type& p) { p_ = p; }

	result_type min() const { return p_.lower(); }

	result_type max() const { return p_.upper(); }

	template <typename URNGT>
	result_type operator()(URNGT& rng)
	{
        // If Y~Beta(p,q,a,b) and X~Beta(p,q,0,1), then:
        //   y = x(b-a) + a
        // or, equivalently:
        //   x = \frac{y-a}{b-a}.
        //
        // References:
        // - https://en.wikipedia.org/wiki/Beta_distribution#Four_parameters_2
        // - http://www.itl.nist.gov/div898/handbook/eda/section3/eda364.htm#FORMULAS
        //   where location=a, scale=(b-a)
        //

        result_type x = beta01_(rng);
		return x*(p_.upper()-p_.lower()) + p_.lower();
	}

	friend bool operator==(const beta_distribution& lhs, const beta_distribution& rhs)
	{
		return lhs.p_ == rhs.p_;
	}

	friend bool operator!=(const beta_distribution& lhs, const beta_distribution& rhs)
	{
		return !(lhs == rhs);
	}

private:
	param_type p_;
    boost::random::beta_distribution<T> beta01_;
}; // beta_distribution

}} // Namespace dcs::fog


#endif // DCS_FOG_RANDOM_HPP
