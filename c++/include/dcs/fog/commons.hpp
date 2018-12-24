/* vim: set tabstop=4 expandtab shiftwidth=4 softtabstop=4: */

/**
 * \file dcs/fog/commons.hpp
 *
 * \brief Commons definitions.
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

#ifndef DCS_FOG_COMMONS_HPP
#define DCS_FOG_COMMONS_HPP


#include <iostream>


namespace dcs { namespace fog {

#if 0
enum vm_allocation_policy_t
{
    max_profit_vm_allocation_policy,
    min_cost_vm_allocation_policy
}; // vm_allocation_policy_t

enum fp_revenue_policy_t
{
    by_service_revenue_policy,
    by_vm_revenue_policy
}; // fp_revenue_policy_t


enum fp_penalty_policy_t
{
    by_service_penalty_policy,
    by_vm_penalty_policy
}; // fp_penalty_policy_t


enum fp_penalty_model_t
{
    binary_penalty_model,
    distance_penalty_model
}; // fp_penalty_model_t
#endif


enum arrival_rate_estimation_t
{
    beta_arrival_rate_estimation,
    ewma_arrival_rate_estimation,
    max_arrival_rate_estimation,
    most_recently_observed_arrival_rate_estimation,
    perturbed_max_arrival_rate_estimation,
    perturbed_most_recently_observed_arrival_rate_estimation,
    uniform_max_arrival_rate_estimation,
    uniform_min_max_arrival_rate_estimation
}; // arrival_rate_estimation_t


enum user_mobility_model_category_t
{
    fixed_user_mobility_model,
    random_waypoint_user_mobility_model,
    step_user_mobility_model
}; // user_mobility_model_category_t


enum vm_allocation_policy_category_t
{
    optimal_vm_allocation_policy,
    bahreini2017_match_vm_allocation_policy,
    bahreini2017_match_alt_vm_allocation_policy
}; // vm_allocation_policy_category_t


template <typename CharT, typename CharTraitsT>
std::basic_ostream<CharT,CharTraitsT>& operator<<(std::basic_ostream<CharT,CharTraitsT>& os, arrival_rate_estimation_t est)
{
    switch (est)
    {
        case beta_arrival_rate_estimation:
            os << "beta";
            break;
        case ewma_arrival_rate_estimation:
            os << "ewma";
            break;
        case max_arrival_rate_estimation:
            os << "max";
            break;
        case most_recently_observed_arrival_rate_estimation:
            os << "mro";
            break;
        case perturbed_max_arrival_rate_estimation:
            os << "perturb-max";
            break;
        case perturbed_most_recently_observed_arrival_rate_estimation:
            os << "perturb-mro";
            break;
        case uniform_max_arrival_rate_estimation:
            os << "unif-max";
            break;
        case uniform_min_max_arrival_rate_estimation:
            os << "unif-min-max";
            break;
    }

    return os;
}

template <typename CharT, typename CharTraitsT>
std::basic_ostream<CharT,CharTraitsT>& operator<<(std::basic_ostream<CharT,CharTraitsT>& os, user_mobility_model_category_t cat)
{
    switch (cat)
    {
        case fixed_user_mobility_model:
            os << "fixed";
            break;
        case random_waypoint_user_mobility_model:
            os << "random_waypoint";
            break;
        case step_user_mobility_model:
            os << "step";
            break;
    }

    return os;
}

template <typename CharT, typename CharTraitsT>
std::basic_ostream<CharT,CharTraitsT>& operator<<(std::basic_ostream<CharT,CharTraitsT>& os, vm_allocation_policy_category_t cat)
{
    switch (cat)
    {
        case optimal_vm_allocation_policy:
            os << "optimal";
            break;
        case bahreini2017_match_vm_allocation_policy:
            os << "bahreini2017_match";
            break;
        case bahreini2017_match_alt_vm_allocation_policy:
            os << "bahreini2017_match_alt";
            break;
    }

    return os;
}

#if 0
template <typename CharT, typename CharTraitsT>
std::basic_ostream<CharT,CharTraitsT>& operator<<(std::basic_ostream<CharT,CharTraitsT>& os, fp_penalty_model_t model)
{
    switch (model)
    {
        case binary_penalty_model:
            os << "binary";
            break;
        case distance_penalty_model:
            os << "distance";
            break;
    }

    return os;
}

template <typename CharT, typename CharTraitsT>
std::basic_ostream<CharT,CharTraitsT>& operator<<(std::basic_ostream<CharT,CharTraitsT>& os, fp_penalty_policy_t policy)
{
    switch (policy)
    {
        case by_service_penalty_policy:
            os << "service";
            break;
        case by_vm_penalty_policy:
            os << "vm";
            break;
    }

    return os;
}

template <typename CharT, typename CharTraitsT>
std::basic_ostream<CharT,CharTraitsT>& operator<<(std::basic_ostream<CharT,CharTraitsT>& os, fp_revenue_policy_t policy)
{
    switch (policy)
    {
        case by_service_revenue_policy:
            os << "service";
            break;
        case by_vm_revenue_policy:
            os << "vm";
            break;
    }

    return os;
}

template <typename CharT, typename CharTraitsT>
std::basic_ostream<CharT,CharTraitsT>& operator<<(std::basic_ostream<CharT,CharTraitsT>& os, vm_allocation_policy_t policy)
{
    switch (policy)
    {
        case max_profit_vm_allocation_policy:
            os << "max-profit";
            break;
        case min_cost_vm_allocation_policy:
            os << "min-cost";
            break;
    }

    return os;
}
#endif

}} // Namespace dcs::fog

#endif // DCS_FOG_COMMONS_HPP
