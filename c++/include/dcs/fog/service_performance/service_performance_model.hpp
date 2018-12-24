/* vim: set tabstop=4 expandtab shiftwidth=4 softtabstop=4: */

/**
 * \file dcs/fog/service_performance/service_performance_model.hpp
 *
 * \brief Base class for service performance models
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

#ifndef DCS_FOG_SERVICE_PERFORMANCE_SERVICE_PERFORMANCE_MODEL_HPP
#define DCS_FOG_SERVICE_PERFORMANCE_SERVICE_PERFORMANCE_MODEL_HPP


#include <cstddef>


namespace dcs { namespace fog {

template <typename RealT>
struct service_performance_model_t
{
    RealT average_response_time(RealT arrival_rate, RealT service_rate, std::size_t num_vms)
    {
        return this->do_average_response_time(arrival_rate, service_rate, num_vms);
    }

    std::size_t min_num_vms(RealT arrival_rate, RealT service_rate, RealT target_delay, RealT tol)
    {
        return this->do_min_num_vms(arrival_rate, service_rate, target_delay, tol);
    }

    virtual ~service_performance_model_t() { }


private:
    virtual std::size_t do_min_num_vms(RealT arrival_rate, RealT service_rate, RealT target_delay, RealT tol) = 0;

    virtual RealT do_average_response_time(RealT arrival_rate, RealT service_rate, std::size_t num_vms) = 0;
}; // service_performance_model_t

}} // Namespace dcs::fog

#endif // DCS_FOG_SERVICE_PERFORMANCE_SERVICE_PERFORMANCE_MODEL_HPP
