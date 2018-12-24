/* vim: set tabstop=4 expandtab shiftwidth=4 softtabstop=4: */

/**
 * \file dcs/fog/scenario.hpp
 *
 * \brief Experimental scenario.
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

#ifndef DCS_FOG_SCENARIO_HPP
#define DCS_FOG_SCENARIO_HPP


#include <boost/algorithm/string.hpp>
#include <cctype>
#include <cstddef>
#include <dcs/assert.hpp>
#include <dcs/debug.hpp>
#include <dcs/exception.hpp>
#include <dcs/fog/commons.hpp>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
//#include <tuple>
#include <utility>
#include <vector>


namespace dcs { namespace fog {

/// Experimental scenario
template <typename RealT>
class scenario_t
{
private:
    static constexpr double default_fp_vm_allocation_interval = 0;
    static const fog::arrival_rate_estimation_t default_svc_arrival_rate_estimation = fog::max_arrival_rate_estimation;
    static constexpr double default_svc_delay_tolerance = 1e-5;


public:
    scenario_t()
    : fp_vm_allocation_interval(default_fp_vm_allocation_interval),
      num_fn_categories(0),
      num_svc_categories(0),
      num_vm_categories(0),
      svc_arrival_rate_estimation(default_svc_arrival_rate_estimation),
      svc_delay_tolerance(default_svc_delay_tolerance)
    {
    }


    std::vector<RealT> fn_max_powers; ///< FN max power consumptions (in kW), by FN category
    std::vector<RealT> fn_min_powers; ///< FN min power consumptions (in kW), by FN category
    RealT fp_electricity_costs; ///< FP electricity cost plans (in $/kWh)
    std::vector<RealT> fp_fn_asleep_costs; ///< FP costs for powering off a powered-on FN, by FN category
    std::vector<RealT> fp_fn_awake_costs; ///< FP costs for powering on a powered-off FN, by FN category
    std::vector<std::size_t> fp_num_svcs; ///< Number of services, by service category
    std::vector<std::size_t> fp_num_fns; ///< Number of FNs, by FN category
    std::vector<RealT> fp_svc_penalties; ///< FP penalties (in $/service) for violating service QoS (i.e., the max delay), by service category
    std::vector<RealT> fp_svc_revenues; ///< FP revenues (in $/service) for running services, by service category
    double fp_vm_allocation_interval; ///< The time interval at which the VM allocation algorithm activates (in terms of simulated time)
    vm_allocation_policy_category_t fp_vm_allocation_policy; ///< The policy to use to allocate VMs
    std::size_t num_fn_categories; ///< Number of fog node (FN) categories
    std::size_t num_svc_categories; ///< Number of service categories
    std::size_t num_vm_categories; ///< Number of virtual machine (VM) categories
    fog::arrival_rate_estimation_t svc_arrival_rate_estimation; ///< The way service arrival rate are predicted
    std::vector<RealT> svc_arrival_rate_estimation_params; ///< The parameters to pass to the specified arrival rate estimation method
    RealT svc_delay_tolerance; ///< The relative tolerance to set in the service performance model
    std::vector<RealT> svc_arrival_rates; ///< Single-user request arrival rates for services, by service category
    std::vector<RealT> svc_max_arrival_rates; ///< Max aggregate (i.e., multiple-users) request arrival rates for services, by service category
    std::vector<RealT> svc_max_delays; ///< Max delays tolerated by services, by service category
    user_mobility_model_category_t svc_user_mobility_model; ///< The user mobility model
    std::map<std::string,std::vector<std::string>> svc_user_mobility_model_params; ///< The parameters for the user mobility model specified as a sequence of <key,value> pairs
    std::vector<std::vector<RealT>> svc_vm_service_rates; ///< Service rate of every VM associated with each service, by service category and VM category
    std::vector<std::vector<RealT>> vm_cpu_requirements; ///< VM CPU requirements, by VM category and FN category
    std::vector<std::vector<RealT>> vm_ram_requirements; ///< VM RAM requirements, by VM category and FN category
    std::vector<RealT> vm_allocation_costs; ///< VM allocation costs, by VM category
}; // scenario_t

template <typename CharT, typename CharTraitsT, typename RealT>
std::basic_ostream<CharT,CharTraitsT>& operator<<(std::basic_ostream<CharT,CharTraitsT>& os, const scenario_t<RealT>& s)
{
    os  << "num_fn_categories=" << s.num_fn_categories
        << ", " << "num_svc_categories=" << s.num_svc_categories
        << ", " << "num_vm_categories=" << s.num_vm_categories;

    os << ", " << "svc.arrival_rates=[";
    for (std::size_t i = 0; i < s.svc_arrival_rates.size(); ++i)
    {
        if (i > 0)
        {
            os << ", ";
        }
        os << s.svc_arrival_rates[i];
    }
    os << "]";
    os << ", " << "svc.max_arrival_rates=[";
    for (std::size_t i = 0; i < s.svc_max_arrival_rates.size(); ++i)
    {
        if (i > 0)
        {
            os << ", ";
        }
        os << s.svc_max_arrival_rates[i];
    }
    os << "]";
    os << ", " << "svc.max_delays=[";
    for (std::size_t i = 0; i < s.svc_max_delays.size(); ++i)
    {
        if (i > 0)
        {
            os << ", ";
        }
        os << s.svc_max_delays[i];
    }
    os << "]";
    os << ", " << "svc.vm_service_rates=[";
    for (std::size_t i = 0; i < s.svc_vm_service_rates.size(); ++i)
    {
        os << "[";
        for (std::size_t j = 0; j < s.svc_vm_service_rates[i].size(); ++j)
        {
            if (j > 0)
            {
                os << ", ";
            }
            os << s.svc_vm_service_rates[i][j];
        }
        os << "]";
    }
    os << "]";
    os << ", " << "fp.num_svcs=[";
    for (std::size_t i = 0; i < s.fp_num_svcs.size(); ++i)
    {
        if (i > 0)
        {
            os << ", ";
        }
        os << s.fp_num_svcs[i];
    }
    os << "]";
    os << ", " << "fp.num_fns=[";
    for (std::size_t i = 0; i < s.fp_num_fns.size(); ++i)
    {
        if (i > 0)
        {
            os << ", ";
        }
        os << s.fp_num_fns[i];
    }
    os << "]";
    os << ", " << "fp.electricity_costs=" << s.fp_electricity_costs;
    os << ", " << "fp.fn_asleep_costs=[";
    for (std::size_t i = 0; i < s.fp_fn_asleep_costs.size(); ++i)
    {
        if (i > 0)
        {   
            os << ", ";
        }
        os << s.fp_fn_asleep_costs[i];
    }
    os << "]";
    os << ", " << "fp.fn_awake_costs=[";
    for (std::size_t i = 0; i < s.fp_fn_awake_costs.size(); ++i)
    {
        if (i > 0)
        {   
            os << ", ";
        }
        os << s.fp_fn_awake_costs[i];
    }
    os << "]";
    os << ", " << "fp.svc_revenues=[";
    for (std::size_t i = 0; i < s.fp_svc_revenues.size(); ++i)
    {
        if (i > 0)
        {
            os << ", ";
        }
        os << s.fp_svc_revenues[i];
    }
    os << "]";
    os << ", " << "fp.svc_penalties=[";
    for (std::size_t i = 0; i < s.fp_svc_penalties.size(); ++i)
    {
        if (i > 0)
        {
            os << ", ";
        }
        os << s.fp_svc_penalties[i];
    }
    os << "]";
    os << ", " << "fn.min_powers=[";
    for (std::size_t i = 0; i < s.fn_min_powers.size(); ++i)
    {
        if (i > 0)
        {
            os << ", ";
        }
        os << s.fn_min_powers[i];
    }
    os << "]";
    os << ", " << "fn.max_powers=[";
    for (std::size_t i = 0; i < s.fn_max_powers.size(); ++i)
    {
        if (i > 0)
        {
            os << ", ";
        }
        os << s.fn_max_powers[i];
    }
    os << "]";
    os << ", " << "vm_cpu_requirements=[";
    for (std::size_t i = 0; i < s.vm_cpu_requirements.size(); ++i)
    {
        if (i > 0)
        {
            os << " ";
        }

        os << "[";
        for (std::size_t j = 0; j < s.vm_cpu_requirements[i].size(); ++j)
        {
            if (j > 0)
            {
                os << ", ";
            }
            os << s.vm_cpu_requirements[i][j];
        }
        os << "]";
    }
    os << "]";
    os << ", " << "vm_ram_requirements=[";
    for (std::size_t i = 0; i < s.vm_ram_requirements.size(); ++i)
    {
        if (i > 0)
        {
            os << " ";
        }

        os << "[";
        for (std::size_t j = 0; j < s.vm_ram_requirements[i].size(); ++j)
        {
            if (j > 0)
            {
                os << ", ";
            }
            os << s.vm_ram_requirements[i][j];
        }
        os << "]";
    }
    os << ", " << "vm_allocation_costs=[";
    for (std::size_t i = 0; i < s.vm_allocation_costs.size(); ++i)
    {
        if (i > 0)
        {
            os << ", ";
        }

        os << s.vm_allocation_costs[i];
    }
    os << "]";
    os << "]";

    os << ", fp.vm_allocation_interval=" << s.fp_vm_allocation_interval;
    os << ", svc.arrival_rate_estimation=" << s.svc_arrival_rate_estimation;
    os << ", svc.arrival_rate_estimation_params=" << s.svc_arrival_rate_estimation_params;
    os << ", svc.delay_tolerance=" << s.svc_delay_tolerance;
    os << ", svc.user_mobility_model=" << s.svc_user_mobility_model;
    os << ", svc.user_mobility_model_params=[";
    for (auto const& keyval_pair : s.svc_user_mobility_model_params)
    {
        //os << keyval_pair.first << " " << keyval_pair.second << ",";
        for (auto const& value : keyval_pair.second)
        {
            os << keyval_pair.first << " " << value << ",";
        }
    }
    os << "]";
    os << ", fp.vm_allocation_policy=" << s.fp_vm_allocation_policy;

    return os;
}

template <typename RealT>
scenario_t<RealT> make_scenario(const std::string& fname)
{
    DCS_ASSERT(!fname.empty(),
               DCS_EXCEPTION_THROW(std::invalid_argument, "Invalid scenario file name"));

    scenario_t<RealT> s;

    std::ifstream ifs(fname.c_str());

    DCS_ASSERT(ifs,
               DCS_EXCEPTION_THROW(std::runtime_error, "Cannot open scenario file"));

    std::ostringstream oss;
    std::size_t lineno = 0;
    for (std::string line; std::getline(ifs, line); )
    {
        ++lineno;

        std::size_t pos = 0;
        for (; pos < line.length() && std::isspace(line[pos]); ++pos)
        {
            ; // empty
        }
        if (pos > 0)
        {
            line = line.substr(pos);
        }
        if (line.empty() || line.at(0) == '#')
        {
            // Skip either empty or comment lines
            continue;
        }

        boost::to_lower(line);

        if (boost::istarts_with(line, "num_fn_categories"))
        {
            std::istringstream iss(line);

            // Move to '='
            iss.ignore(std::numeric_limits<std::streamsize>::max(), '=');
            DCS_ASSERT(iss.good(),
                       DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('=' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

            iss >> s.num_fn_categories;
        }
        else if (boost::istarts_with(line, "num_svc_categories"))
        {
            std::istringstream iss(line);

            // Move to '='
            iss.ignore(std::numeric_limits<std::streamsize>::max(), '=');
            DCS_ASSERT(iss.good(),
                       DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('=' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

            iss >> s.num_svc_categories;
        }
        else if (boost::istarts_with(line, "num_vm_categories"))
        {
            std::istringstream iss(line);

            // Move to '='
            iss.ignore(std::numeric_limits<std::streamsize>::max(), '=');
            DCS_ASSERT(iss.good(),
                       DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('=' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

            iss >> s.num_vm_categories;
        }
        else if (boost::istarts_with(line, "svc.arrival_rates"))
        {
            std::istringstream iss(line);

            // Move to '='
            iss.ignore(std::numeric_limits<std::streamsize>::max(), '=');
            DCS_ASSERT(iss.good(),
                       DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('=' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

            // Move to '['
            iss.ignore(std::numeric_limits<std::streamsize>::max(), '[');
            DCS_ASSERT(iss.good(),
                       DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('[' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

            s.svc_arrival_rates.clear(); // Clear this container in case of this parameter has been repeated more than once in the scenario file
            s.svc_arrival_rates.resize(s.num_svc_categories);
            for (std::size_t i = 0; i < s.num_svc_categories; ++i)
            {
                iss >> s.svc_arrival_rates[i];
            }
        }
        else if (boost::istarts_with(line, "svc.max_arrival_rates"))
        {
            std::istringstream iss(line);

            // Move to '='
            iss.ignore(std::numeric_limits<std::streamsize>::max(), '=');
            DCS_ASSERT(iss.good(),
                       DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('=' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

            // Move to '['
            iss.ignore(std::numeric_limits<std::streamsize>::max(), '[');
            DCS_ASSERT(iss.good(),
                       DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('[' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

            s.svc_max_arrival_rates.clear(); // Clear this container in case of this parameter has been repeated more than once in the scenario file
            s.svc_max_arrival_rates.resize(s.num_svc_categories);
            for (std::size_t i = 0; i < s.num_svc_categories; ++i)
            {
                iss >> s.svc_max_arrival_rates[i];
            }
        }
        else if (boost::istarts_with(line, "svc.max_delays"))
        {
            std::istringstream iss(line);

            // Move to '='
            iss.ignore(std::numeric_limits<std::streamsize>::max(), '=');
            DCS_ASSERT(iss.good(),
                       DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('=' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

            // Move to '['
            iss.ignore(std::numeric_limits<std::streamsize>::max(), '[');
            DCS_ASSERT(iss.good(),
                       DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('[' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

            s.svc_max_delays.clear(); // Clear this container in case of this parameter has been repeated more than once in the scenario file
            s.svc_max_delays.resize(s.num_svc_categories);
            for (std::size_t i = 0; i < s.num_svc_categories; ++i)
            {
                iss >> s.svc_max_delays[i];
            }
        }
        else if (boost::istarts_with(line, "svc.vm_service_rates"))
        {
            std::istringstream iss(line);

            // Move to '='
            iss.ignore(std::numeric_limits<std::streamsize>::max(), '=');
            DCS_ASSERT(iss.good(),
                       DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('=' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

            // Move to '['
            iss.ignore(std::numeric_limits<std::streamsize>::max(), '[');
            DCS_ASSERT(iss.good(),
                       DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('[' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

            s.svc_vm_service_rates.clear(); // Clear this container in case of this parameter has been repeated more than once in the scenario file
            s.svc_vm_service_rates.resize(s.num_svc_categories);
            for (std::size_t i = 0; i < s.num_svc_categories; ++i)
            {
                // Move to '['
                iss.ignore(std::numeric_limits<std::streamsize>::max(), '[');
                DCS_ASSERT(iss.good(),
                           DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('[' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

                s.svc_vm_service_rates[i].resize(s.num_vm_categories);
                for (std::size_t j = 0; j < s.num_vm_categories; ++j)
                {
                    iss >> s.svc_vm_service_rates[i][j];
                }

                // Move to ']'
                iss.ignore(std::numeric_limits<std::streamsize>::max(), ']');
                DCS_ASSERT(iss.good(),
                           DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file (']' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));
            }
        }
        else if (boost::istarts_with(line, "fp.num_svcs"))
        {
            std::istringstream iss(line);

            // Move to '='
            iss.ignore(std::numeric_limits<std::streamsize>::max(), '=');
            DCS_ASSERT(iss.good(),
                       DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('=' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

            // Move to '['
            iss.ignore(std::numeric_limits<std::streamsize>::max(), '[');
            DCS_ASSERT(iss.good(),
                       DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('[' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

            s.fp_num_svcs.clear(); // Clear this container in case of this parameter has been repeated more than once in the scenario file
            s.fp_num_svcs.resize(s.num_svc_categories);
            for (std::size_t i = 0; i < s.num_svc_categories; ++i)
            {
                iss >> s.fp_num_svcs[i];
            }
        }
        else if (boost::istarts_with(line, "fp.num_fns"))
        {
            std::istringstream iss(line);

            // Move to '='
            iss.ignore(std::numeric_limits<std::streamsize>::max(), '=');
            DCS_ASSERT(iss.good(),
                       DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('=' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

            // Move to '['
            iss.ignore(std::numeric_limits<std::streamsize>::max(), '[');
            DCS_ASSERT(iss.good(),
                       DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('[' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

            s.fp_num_fns.clear(); // Clear this container in case of this parameter has been repeated more than once in the scenario file
            s.fp_num_fns.resize(s.num_fn_categories);
            for (std::size_t i = 0; i < s.num_fn_categories; ++i)
            {
                iss >> s.fp_num_fns[i];
            }
        }
        else if (boost::istarts_with(line, "fp.electricity_costs"))
        {
            std::istringstream iss(line);

            // Move to '='
            iss.ignore(std::numeric_limits<std::streamsize>::max(), '=');
            DCS_ASSERT(iss.good(),
                       DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('=' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

            iss >> s.fp_electricity_costs;
        }
        else if (boost::istarts_with(line, "fp.fn_asleep_costs"))
        {
            std::istringstream iss(line);

            // Move to '='
            iss.ignore(std::numeric_limits<std::streamsize>::max(), '=');
            DCS_ASSERT(iss.good(),
                       DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('=' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

            // Move to '['
            iss.ignore(std::numeric_limits<std::streamsize>::max(), '[');
            DCS_ASSERT(iss.good(),
                       DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('[' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

            s.fp_fn_asleep_costs.clear(); // Clear this container in case of this parameter has been repeated more than once in the scenario file
            s.fp_fn_asleep_costs.resize(s.num_fn_categories);
            for (std::size_t i = 0; i < s.num_fn_categories; ++i)
            {
                iss >> s.fp_fn_asleep_costs[i];
            }
        }
        else if (boost::istarts_with(line, "fp.fn_awake_costs"))
        {
            std::istringstream iss(line);

            // Move to '='
            iss.ignore(std::numeric_limits<std::streamsize>::max(), '=');
            DCS_ASSERT(iss.good(),
                       DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('=' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

            // Move to '['
            iss.ignore(std::numeric_limits<std::streamsize>::max(), '[');
            DCS_ASSERT(iss.good(),
                       DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('[' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

            s.fp_fn_awake_costs.clear(); // Clear this container in case of this parameter has been repeated more than once in the scenario file
            s.fp_fn_awake_costs.resize(s.num_fn_categories);
            for (std::size_t i = 0; i < s.num_fn_categories; ++i)
            {
                iss >> s.fp_fn_awake_costs[i];
            }
        }
        else if (boost::istarts_with(line, "fp.svc_revenues"))
        {
            std::istringstream iss(line);

            // Move to '='
            iss.ignore(std::numeric_limits<std::streamsize>::max(), '=');
            DCS_ASSERT(iss.good(),
                       DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('=' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

            // Move to '['
            iss.ignore(std::numeric_limits<std::streamsize>::max(), '[');
            DCS_ASSERT(iss.good(),
                       DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('[' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

            s.fp_svc_revenues.clear(); // Clear this container in case of this parameter has been repeated more than once in the scenario file
            s.fp_svc_revenues.resize(s.num_svc_categories);
            for (std::size_t i = 0; i < s.num_svc_categories; ++i)
            {
                iss >> s.fp_svc_revenues[i];
            }
        }
        else if (boost::istarts_with(line, "fp.svc_penalties"))
        {
            std::istringstream iss(line);

            // Move to '='
            iss.ignore(std::numeric_limits<std::streamsize>::max(), '=');
            DCS_ASSERT(iss.good(),
                       DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('=' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

            // Move to '['
            iss.ignore(std::numeric_limits<std::streamsize>::max(), '[');
            DCS_ASSERT(iss.good(),
                       DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('[' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

            s.fp_svc_penalties.clear(); // Clear this container in case of this parameter has been repeated more than once in the scenario file
            s.fp_svc_penalties.resize(s.num_svc_categories);
            for (std::size_t i = 0; i < s.num_svc_categories; ++i)
            {
                iss >> s.fp_svc_penalties[i];
            }
        }
        else if (boost::istarts_with(line, "fn.min_powers"))
        {
            std::istringstream iss(line);

            // Move to '='
            iss.ignore(std::numeric_limits<std::streamsize>::max(), '=');
            DCS_ASSERT(iss.good(),
                       DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('=' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

            // Move to '['
            iss.ignore(std::numeric_limits<std::streamsize>::max(), '[');
            DCS_ASSERT(iss.good(),
                       DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('[' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

            s.fn_min_powers.clear(); // Clear this container in case of this parameter has been repeated more than once in the scenario file
            s.fn_min_powers.resize(s.num_fn_categories);
            for (std::size_t i = 0; i < s.num_fn_categories; ++i)
            {
                iss >> s.fn_min_powers[i];
            }

            //// Move to ']'
            //iss.ignore(std::numeric_limits<std::streamsize>::max(), ']');
            //DCS_ASSERT(iss.good(),
            //           DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file (']' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));
        }
        else if (boost::istarts_with(line, "fn.max_powers"))
        {
            std::istringstream iss(line);

            // Move to '='
            iss.ignore(std::numeric_limits<std::streamsize>::max(), '=');
            DCS_ASSERT(iss.good(),
                       DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('=' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

            // Move to '['
            iss.ignore(std::numeric_limits<std::streamsize>::max(), '[');
            DCS_ASSERT(iss.good(),
                       DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('[' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

            s.fn_max_powers.clear(); // Clear this container in case of this parameter has been repeated more than once in the scenario file
            s.fn_max_powers.resize(s.num_fn_categories);
            for (std::size_t i = 0; i < s.num_fn_categories; ++i)
            {
                iss >> s.fn_max_powers[i];
            }

            //// Move to ']'
            //iss.ignore(std::numeric_limits<std::streamsize>::max(), ']');
            //DCS_ASSERT(iss.good(),
            //         DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file (']' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));
        }
        else if (boost::istarts_with(line, "vm.cpu_requirements"))
        {
            std::istringstream iss(line);

            // Move to '='
            iss.ignore(std::numeric_limits<std::streamsize>::max(), '=');
            DCS_ASSERT(iss.good(),
                       DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('=' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

            // Move to '['
            iss.ignore(std::numeric_limits<std::streamsize>::max(), '[');
            DCS_ASSERT(iss.good(),
                       DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('[' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

            s.vm_cpu_requirements.clear(); // Clear this container in case of this parameter has been repeated more than once in the scenario file
            s.vm_cpu_requirements.resize(s.num_vm_categories);
            for (std::size_t i = 0; i < s.num_vm_categories; ++i)
            {
                // Move to '['
                iss.ignore(std::numeric_limits<std::streamsize>::max(), '[');
                DCS_ASSERT(iss.good(),
                           DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('[' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

                s.vm_cpu_requirements[i].resize(s.num_fn_categories);
                for (std::size_t j = 0; j < s.num_fn_categories; ++j)
                {
                    iss >> s.vm_cpu_requirements[i][j];
                }

                // Move to ']'
                iss.ignore(std::numeric_limits<std::streamsize>::max(), ']');
                DCS_ASSERT(iss.good(),
                           DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file (']' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));
            }
        }
        else if (boost::istarts_with(line, "vm.ram_requirements"))
        {
            std::istringstream iss(line);

            // Move to '='
            iss.ignore(std::numeric_limits<std::streamsize>::max(), '=');
            DCS_ASSERT(iss.good(),
                       DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('=' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

            // Move to '['
            iss.ignore(std::numeric_limits<std::streamsize>::max(), '[');
            DCS_ASSERT(iss.good(),
                       DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('[' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

            s.vm_ram_requirements.clear(); // Clear this container in case of this parameter has been repeated more than once in the scenario file
            s.vm_ram_requirements.resize(s.num_vm_categories);
            for (std::size_t i = 0; i < s.num_vm_categories; ++i)
            {
                // Move to '['
                iss.ignore(std::numeric_limits<std::streamsize>::max(), '[');
                DCS_ASSERT(iss.good(),
                           DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('[' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

                s.vm_ram_requirements[i].resize(s.num_fn_categories);
                for (std::size_t j = 0; j < s.num_fn_categories; ++j)
                {
                    iss >> s.vm_ram_requirements[i][j];
                }

                // Move to ']'
                iss.ignore(std::numeric_limits<std::streamsize>::max(), ']');
                DCS_ASSERT(iss.good(),
                           DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file (']' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));
            }
        }
        else if (boost::istarts_with(line, "vm.allocation_costs"))
        {
            std::istringstream iss(line);

            // Move to '='
            iss.ignore(std::numeric_limits<std::streamsize>::max(), '=');
            DCS_ASSERT(iss.good(),
                       DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('=' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

            // Move to '['
            iss.ignore(std::numeric_limits<std::streamsize>::max(), '[');
            DCS_ASSERT(iss.good(),
                       DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('[' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

            s.vm_allocation_costs.clear(); // Clear this container in case of this parameter has been repeated more than once in the scenario file
            s.vm_allocation_costs.resize(s.num_vm_categories);
            for (std::size_t i = 0; i < s.num_vm_categories; ++i)
            {
                iss >> s.vm_allocation_costs[i];
            }
        }
        else if (boost::istarts_with(line, "fp.vm_allocation_interval"))
        {
            std::istringstream iss(line);

            // Move to '='
            iss.ignore(std::numeric_limits<std::streamsize>::max(), '=');
            DCS_ASSERT(iss.good(),
                       DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('=' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

            iss >> s.fp_vm_allocation_interval;
        }
        else if (boost::istarts_with(line, "svc.arrival_rate_estimation_params"))
        {
            std::istringstream iss(line);

            // Move to '='
            iss.ignore(std::numeric_limits<std::streamsize>::max(), '=');
            DCS_ASSERT(iss.good(),
                       DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('=' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

            // Move to '['
            iss.ignore(std::numeric_limits<std::streamsize>::max(), '[');
            DCS_ASSERT(iss.good(),
                       DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('[' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

            s.svc_arrival_rate_estimation_params.clear(); // Clear this container in case of this parameter has been repeated more than once in the scenario file
            while (iss.good() && iss.peek() != ']')
            {
                RealT param = 0;

                iss >> param;

                s.svc_arrival_rate_estimation_params.push_back(param);
            }
        }
        else if (boost::istarts_with(line, "svc.arrival_rate_estimation"))
        {
            std::istringstream iss(line);

            // Move to '='
            iss.ignore(std::numeric_limits<std::streamsize>::max(), '=');
            DCS_ASSERT(iss.good(),
                       DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('=' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

            std::string str;
            iss >> str;
            boost::to_lower(str);
            if (str == "beta")
            {
                s.svc_arrival_rate_estimation = fog::uniform_min_max_arrival_rate_estimation;
            }
            else if (str == "ewma")
            {
                s.svc_arrival_rate_estimation = fog::ewma_arrival_rate_estimation;
            }
            else if (str == "max")
            {
                s.svc_arrival_rate_estimation = fog::max_arrival_rate_estimation;
            }
            else if (str == "mro")
            {
                s.svc_arrival_rate_estimation = fog::most_recently_observed_arrival_rate_estimation;
            }
            else if (str == "perturb-max")
            {
                s.svc_arrival_rate_estimation = fog::perturbed_max_arrival_rate_estimation;
            }
            else if (str == "perturb-mro")
            {
                s.svc_arrival_rate_estimation = fog::perturbed_most_recently_observed_arrival_rate_estimation;
            }
            else if (str == "unif-max")
            {
                s.svc_arrival_rate_estimation = fog::uniform_max_arrival_rate_estimation;
            }
            else if (str == "unif-min-max")
            {
                s.svc_arrival_rate_estimation = fog::uniform_min_max_arrival_rate_estimation;
            }
            else
            {
                DCS_EXCEPTION_THROW(std::runtime_error, "Unknown service arrival rate estimation");
            }
        }
        else if (boost::istarts_with(line, "svc.delay_tolerance"))
        {
            std::istringstream iss(line);

            // Move to '='
            iss.ignore(std::numeric_limits<std::streamsize>::max(), '=');
            DCS_ASSERT(iss.good(),
                       DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('=' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

            iss >> s.svc_delay_tolerance;
        }
        else if (boost::istarts_with(line, "svc.user_mobility_model_params"))
        {
            std::istringstream iss(line);

            // Move to '='
            iss.ignore(std::numeric_limits<std::streamsize>::max(), '=');
            DCS_ASSERT(iss.good(),
                       DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('=' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

            // Move to '['
            iss.ignore(std::numeric_limits<std::streamsize>::max(), '[');
            DCS_ASSERT(iss.good(),
                       DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('[' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

            s.svc_user_mobility_model_params.clear(); // Clear this container in case of this parameter has been repeated more than once in the scenario file
            while (iss.good() && iss.peek() != ']')
            {
                std::string param_name;
                std::string param_value;

                iss >> param_name;
                iss >> param_value;
                boost::to_lower(param_name);
                if (param_value.back() == ']')
                {
                    param_value.pop_back();
                    iss.putback(']');
                }

                s.svc_user_mobility_model_params[param_name].push_back(param_value);
            }
        }
        else if (boost::istarts_with(line, "svc.user_mobility_model")) ///NOTE: this must come after "svc.user_mobility_model_params" otherwise the match would satisfy both "svc.user_mobility_model" and "svc.user_mobility_model_params" keys 
        {
            std::istringstream iss(line);

            // Move to '='
            iss.ignore(std::numeric_limits<std::streamsize>::max(), '=');
            DCS_ASSERT(iss.good(),
                       DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('=' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

            std::string str;
            iss >> str;
            boost::to_lower(str);
            if (str == "fixed")
            {
                s.svc_user_mobility_model = fog::fixed_user_mobility_model;
            }
            else if (str == "random-waypoint")
            {
                s.svc_user_mobility_model = fog::random_waypoint_user_mobility_model;
            }
            else if (str == "step")
            {
                s.svc_user_mobility_model = fog::step_user_mobility_model;
            }
            else
            {
                DCS_EXCEPTION_THROW(std::runtime_error, "Unknown user mobility model '" + str + "'");
            }
        }
        else if (boost::istarts_with(line, "fp.vm_allocation_policy"))
        {
            std::istringstream iss(line);

            // Move to '='
            iss.ignore(std::numeric_limits<std::streamsize>::max(), '=');
            DCS_ASSERT(iss.good(),
                       DCS_EXCEPTION_THROW(std::runtime_error, "Malformed scenario file ('=' is missing at line " + stringify(lineno) + " and column " + stringify(iss.tellg()) + ")"));

            std::string str;
            iss >> str;
            boost::to_lower(str);
            if (str == "optimal")
            {
                s.fp_vm_allocation_policy = fog::optimal_vm_allocation_policy;
            }
            else if (str == "bahreini2017_match")
            {
                s.fp_vm_allocation_policy = fog::bahreini2017_match_vm_allocation_policy;
            }
            else if (str == "bahreini2017_match_alt")
            {
                s.fp_vm_allocation_policy = fog::bahreini2017_match_alt_vm_allocation_policy;
            }
            else
            {
                DCS_EXCEPTION_THROW(std::runtime_error, "Unknown VM allocation policy '" + str + "'");
            }
        }
    }

    // TODO: Assign default values
    // ...

    // Post-parsing consistency checks
    DCS_ASSERT(s.num_fn_categories > 0,
               DCS_EXCEPTION_THROW(std::runtime_error, "Unexpected number of FN categories"));
    DCS_ASSERT(s.num_svc_categories > 0,
               DCS_EXCEPTION_THROW(std::runtime_error, "Unexpected number of service categories"));
    DCS_ASSERT(s.svc_arrival_rates.size() == s.num_svc_categories,
               DCS_EXCEPTION_THROW(std::runtime_error, "Unexpected number of service categories in service arrival rates by service category"));
    DCS_ASSERT(s.svc_max_delays.size() == s.num_svc_categories,
               DCS_EXCEPTION_THROW(std::runtime_error, "Unexpected number of service categories in service maximum delays by service categoriy"));
    DCS_ASSERT(s.svc_vm_service_rates.size() == s.num_svc_categories,
               DCS_EXCEPTION_THROW(std::runtime_error, "Unexpected number of service categories in service VM service rates by service category and VM category"));
    for (std::size_t i = 0; i < s.num_svc_categories; ++i)
    {
        DCS_ASSERT(s.svc_vm_service_rates[i].size() == s.num_vm_categories,
                   DCS_EXCEPTION_THROW(std::runtime_error, "Unexpected number of VM categories for service " + stringify(i) + " in service VM service rates by service category and VM category"));
    }
    DCS_ASSERT(s.fp_num_svcs.size() == s.num_svc_categories,
               DCS_EXCEPTION_THROW(std::runtime_error, "Unexpected number of service categories in number of services"));
    DCS_ASSERT(s.fp_num_fns.size() == s.num_fn_categories,
               DCS_EXCEPTION_THROW(std::runtime_error, "Unexpected number of FN categories in number of FNs"));
    DCS_ASSERT(s.fp_electricity_costs >= 0,
               DCS_EXCEPTION_THROW(std::runtime_error, "Unexpected negative value in electricity costs"));
    DCS_ASSERT(s.fp_fn_asleep_costs.size() == s.num_fn_categories,
               DCS_EXCEPTION_THROW(std::runtime_error, "Unexpected number of FN categories in FN asleep costs"));
    DCS_ASSERT(s.fp_fn_awake_costs.size() == s.num_fn_categories,
               DCS_EXCEPTION_THROW(std::runtime_error, "Unexpected number of FN categories in FN asleep costs"));
    DCS_ASSERT(s.fp_svc_revenues.size() == s.num_svc_categories,
               DCS_EXCEPTION_THROW(std::runtime_error, "Unexpected number of service categories in service revenues"));
    DCS_ASSERT(s.fp_svc_penalties.size() == s.num_svc_categories,
               DCS_EXCEPTION_THROW(std::runtime_error, "Unexpected number of service categories in service penalties"));
    DCS_ASSERT(s.fn_min_powers.size() == s.num_fn_categories,
               DCS_EXCEPTION_THROW(std::runtime_error, "Unexpected number of FN categories in FN min power consumptions by FN category"));
    DCS_ASSERT(s.fn_max_powers.size() == s.num_fn_categories,
               DCS_EXCEPTION_THROW(std::runtime_error, "Unexpected number of FN categories in FN max power consumptions by FN category"));

    return s;
}

}} // Namespace dcs::fog

#endif // DCS_FOG_SCENARIO_HPP
