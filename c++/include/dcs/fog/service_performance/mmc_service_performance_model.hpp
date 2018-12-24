/* vim: set tabstop=4 expandtab shiftwidth=4 softtabstop=4: */

/**
 * \file dcs/fog/service_performance/mmc_service_performance_model.hpp
 *
 * \brief Service performance model based on the M/M/c queues.
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

#ifndef DCS_FOG_SERVICE_PERFORMANCE_MMC_SERVICE_PERFORMANCE_MODEL_HPP
#define DCS_FOG_SERVICE_PERFORMANCE_MMC_SERVICE_PERFORMANCE_MODEL_HPP


#include <array>
#include <cstddef>
#include <cstring>
#include <dcs/debug.hpp>
#include <dcs/logging.hpp>
#if 0
#include <dcs/fog/MMc.hpp>
#elif 0
//#include <trivedi/TrivediPerc.hpp>
#include <trivedi/TrivediMMc.hpp>
#else
#include <dcs/math/traits/float.hpp>
#endif
#include <dcs/fog/service_performance/service_performance_model.hpp>
#include <sstream>


namespace dcs { namespace fog {

template <typename RealT>
class mmc_service_performance_model_t: public service_performance_model_t<RealT>
{
#if 0
private:
    static constexpr const char* py_module = "percentileTrivedi";
    static constexpr const char* py_function_min_num_vms = "main";
    static constexpr const char* py_function_avg_rt = "avgRespTime";


public:
    mmc_service_performance_model_t()
    {
        initialize_py_script();
    }

    ~mmc_service_performance_model_t()
    {
        finalize_py_script();
    }

private:
    std::size_t do_min_num_vms(RealT arrival_rate, RealT service_rate, RealT target_delay, RealT tol)
    {
        //MMc<RealT> mmc(arrival_rate, service_rate, target_delay, tol);
        //return mmc.computeQueueParameters(true);
        const std::size_t py_num_args = 8;

        // Setup arguments to pass to Python script

        std::array<char*,py_num_args> py_args_;
        std::size_t sz = 0;
        std::size_t arg = 0;
        std::string str;

        // arg 0: not used (however, we can't pass a null pointer because Python doesn't like it)
        sz = 1;
        py_args_[arg] = new char[sz];
        std::strncpy(py_args_[arg], "", 1);
        // arg 1: module name
        ++arg;
        sz = std::strlen(py_module)+1;
        py_args_[arg] = new char[sz];
        std::strncpy(py_args_[arg], py_module, sz);
        // arg 2: function name
        ++arg;
        sz = std::strlen(py_function_min_num_vms)+1;
        py_args_[arg] = new char[sz];
        std::strncpy(py_args_[arg], py_function_min_num_vms, sz);
        // arg 3: arrival rate
        ++arg;
        {
            std::ostringstream oss;
            oss << arrival_rate;
            str = oss.str();
            sz = str.size()+1;
        }
        py_args_[arg] = new char[sz];
        std::strncpy(py_args_[arg], str.c_str(), sz);
        // arg 4: service rate
        ++arg;
        {
            std::ostringstream oss;
            oss << service_rate;
            str = oss.str();
            sz = str.size()+1;
        }
        py_args_[arg] = new char[sz];
        std::strncpy(py_args_[arg], str.c_str(), sz);
        // arg 5: delay
        ++arg;
        {
            std::ostringstream oss;
            oss << target_delay;
            str = oss.str();
            sz = str.size()+1;
        }
        py_args_[arg] = new char[sz];
        std::strncpy(py_args_[arg], str.c_str(), sz);
        // arg 6: delay
        ++arg;
        {
            std::ostringstream oss;
            oss << tol;
            str = oss.str();
            sz = str.size()+1;
        }
        py_args_[arg] = new char[sz];
        std::strncpy(py_args_[arg], str.c_str(), sz);
        // arg 7: debug?
        ++arg;
        {
            std::ostringstream oss;
#ifdef DCS_DEBUG
            oss << true;
#else
            oss << false;
#endif // DCS_DEBUG
            str = oss.str();
            sz = str.size()+1;
        }
        py_args_[arg] = new char[sz];
        std::strncpy(py_args_[arg], str.c_str(), sz);

        // sanity check
        DCS_DEBUG_ASSERT( py_num_args == py_args_.size() );

        // - Call Python script
DCS_DEBUG_TRACE("ARGS to NEXTPLEASE - arg[0]: " << py_args_[0] << ", arg[1]: " << py_args_[1] << ", arg[2]: " << py_args_[2] << ", arg[3]: " << py_args_[3] << ", arg[4]: " << py_args_[4] << ", arg[5]: " << py_args_[5] << ", arg[6]: " << py_args_[6] << ", arg[7]: " << py_args_[7]);//XXX
        auto num_vms = py_wrap_.nextPlease(py_args_.size(), py_args_.data());
        if (num_vms < 0)
        {
            throw std::runtime_error("Error while calling Python script for Trivedi's M/M/c service performance model");
        }

        // - Clean-up arguments for Python script
        for (std::size_t i = 0; i < py_args_.size(); ++i)
        {
            if (py_args_[i])
            {
                delete[] py_args_[i];
                py_args_[i] = 0;
            }
        }

        return static_cast<std::size_t>(num_vms);
    }

    RealT do_average_response_time(RealT arrival_rate, RealT service_rate, std::size_t num_vms)
    {
        // Setup arguments to pass to Python script

        const std::size_t py_num_args = 7;
        std::array<char*,py_num_args> py_args_;

        std::size_t sz = 0;
        std::size_t arg = 0;
        std::string str;

        // arg 0: not used (however, we can't pass a null pointer because Python doesn't like it)
        sz = 1;
        py_args_[arg] = new char[sz];
        std::strncpy(py_args_[arg], "", 1);
        // arg 1: module name
        ++arg;
        sz = std::strlen(py_module)+1;
        py_args_[arg] = new char[sz];
        std::strncpy(py_args_[arg], py_module, sz);
        // arg 2: function name
        ++arg;
        sz = std::strlen(py_function_avg_rt)+1;
        py_args_[arg] = new char[sz];
        std::strncpy(py_args_[arg], py_function_avg_rt, sz);
        // arg 3: arrival rate
        ++arg;
        {
            std::ostringstream oss;
            oss << arrival_rate;
            str = oss.str();
            sz = str.size()+1;
        }
        py_args_[arg] = new char[sz];
        std::strncpy(py_args_[arg], str.c_str(), sz);
        // arg 4: service rate
        ++arg;
        {
            std::ostringstream oss;
            oss << service_rate;
            str = oss.str();
            sz = str.size()+1;
        }
        py_args_[arg] = new char[sz];
        std::strncpy(py_args_[arg], str.c_str(), sz);
        // arg 5: num VMs
        ++arg;
        {
            std::ostringstream oss;
            oss << num_vms;
            str = oss.str();
            sz = str.size()+1;
        }
        py_args_[arg] = new char[sz];
        std::strncpy(py_args_[arg], str.c_str(), sz);
        // arg 6: debug?
        ++arg;
        {
            std::ostringstream oss;
#ifdef DCS_DEBUG
            oss << true;
#else
            oss << false;
#endif // DCS_DEBUG
            str = oss.str();
            sz = str.size()+1;
        }
        py_args_[arg] = new char[sz];
        std::strncpy(py_args_[arg], str.c_str(), sz);

        // sanity check
        DCS_DEBUG_ASSERT( py_num_args == py_args_.size() );

        // - Call Python script
DCS_DEBUG_TRACE("ARGS to AVGRESPTIME - arg[0]: " << py_args_[0] << ", arg[1]: " << py_args_[1] << ", arg[2]: " << py_args_[2] << ", arg[3]: " << py_args_[3] << ", arg[4]: " << py_args_[4] << ", arg[5]: " << py_args_[5] << ", arg[6]: " << py_args_[6]);//XXX
        auto rt = py_wrap_.avgResponseTime(py_args_.size(), py_args_.data());

        // - Clean-up arguments for Python script
        for (std::size_t i = 0; i < py_args_.size(); ++i)
        {
            if (py_args_[i])
            {
                delete[] py_args_[i];
                py_args_[i] = 0;
            }
        }

        return static_cast<RealT>(rt);
    }

    void initialize_py_script()
    {
        const std::size_t py_num_args = 2;
        std::array<char*,py_num_args> py_args_;

        // Setup arguments to pass to Python script

        std::size_t sz = 0;
        std::size_t arg = 0;
        std::string str;

        // arg 0: not used (however, we can't pass a null pointer because Python doesn't like it)
        sz = 1;
        py_args_[arg] = new char[sz];
        std::strncpy(py_args_[arg], "", 1);
        // arg 1: module name
        ++arg;
        sz = std::strlen(py_module)+1;
        py_args_[arg] = new char[sz];
        std::strncpy(py_args_[arg], py_module, sz);
#if 0
        // arg 2: not used
        ++arg;
        sz = /*std::strlen(py_function)+*/1;
        py_args_[arg] = new char[sz];
        std::strncpy(py_args_[arg], ""/*py_function*/, sz);
        // arg 3: not used
        ++arg;
        sz = 1;
        py_args_[arg] = new char[sz];
        std::strncpy(py_args_[arg], "", 1);
        // arg 4: not used
        ++arg;
        sz = 1;
        py_args_[arg] = new char[sz];
        std::strncpy(py_args_[arg], "", 1);
        // arg 5: not used
        ++arg;
        sz = 1;
        py_args_[arg] = new char[sz];
        std::strncpy(py_args_[arg], "", 1);
        // arg 6: not used
        ++arg;
        sz = 1;
        py_args_[arg] = new char[sz];
        std::strncpy(py_args_[arg], "", 1);
#endif

        // sanity check
        DCS_DEBUG_ASSERT( py_num_args == py_args_.size() );

        // - Setup
//DCS_DEBUG_TRACE("ARGS to SETUP - arg[0]: " << py_args_[0] << ", arg[1]: " << py_args_[1] << ", arg[2]: " << py_args_[2] << ", arg[3]: " << py_args_[3] << ", arg[4]: " << py_args_[4] << ", arg[5]: " << py_args_[5] << ", arg[6]: " << py_args_[6]);//XXX
DCS_DEBUG_TRACE("ARGS to SETUP - arg[0]: " << py_args_[0] << ", arg[1]: " << py_args_[1]);//XXX
        auto setup_ret = py_wrap_.setup(py_args_.size(), py_args_.data());
        if (setup_ret < 0)
        {
            throw std::runtime_error("Unable to initialize the Python script");
        }

        // - Clean-up arguments for Python script
        for (std::size_t i = 0; i < py_args_.size(); ++i)
        {
            if (py_args_[i])
            {
                delete[] py_args_[i];
                py_args_[i] = 0;
            }
        }
    }

    void finalize_py_script()
    {
        // - Clean-up
        auto dismiss_ret = py_wrap_.dismiss();
        if (dismiss_ret < 0)
        {
            dcs::log_warn(DCS_LOGGING_AT, "Problems when finalizing the Python script");
        }

        // - Clean-up arguments for Python script
        //for (std::size_t i = 0; i < py_args_.size(); ++i)
        //{
        //    if (py_args_[i])
        //    {
        //        delete[] py_args_[i];
        //        py_args_[i] = 0;
        //    }
        //}
    }

private:
    TrivediPerc py_wrap_;
    //std::array<char*,py_num_args> py_args_;

#elif 0

private:
    std::size_t do_min_num_vms(RealT arrival_rate, RealT service_rate, RealT target_delay, RealT tol)
    {
        bool debug = false;

#ifdef DCS_DEBUG
        debug = true;
#endif //DCS_DEBUG

        return py_wrap_.num_servers(arrival_rate, service_rate, target_delay, tol, debug);
    }

    RealT do_average_response_time(RealT arrival_rate, RealT service_rate, std::size_t num_vms)
    {
        bool debug = false;

#ifdef DCS_DEBUG
        debug = true;
#endif //DCS_DEBUG

        return 0;//py_wrap_.average_response_time(arrival_rate, service_rate, num_vms, debug);
    }

private:
    TrivediMMc py_wrap_;
#else

private:
    std::size_t do_min_num_vms(RealT arrival_rate, RealT service_rate, RealT target_delay, RealT tol)
    {
        if (target_delay < (1/service_rate))
        {
            //logging.warning("model NOT feasible: response time < service time")
            return std::numeric_limits<std::size_t>::max();
        }

        return MMc_num_servers(arrival_rate, service_rate, target_delay, tol);
    }

    RealT do_average_response_time(RealT arrival_rate, RealT service_rate, std::size_t num_vms)
    {
        return MMc_avg_response_time(arrival_rate, service_rate, num_vms);
    }

    static RealT factorial(std::size_t n)
    {
        RealT f = 1;
        while (n >= 2)
        {
            f *= n--;
        }
        //for (std::size_t k = 2; k <= n; ++k)
        //{
        //    f *= k;
        //}
        return f;
    }

    static std::size_t MMc_num_servers(RealT lambda, RealT mu, RealT max_rt, RealT tol)
    {
        if (dcs::math::float_traits<RealT>::essentially_equal(lambda, 0))
        {
            return 0;
        }

        std::size_t c = 0;
        while (true)
        {
            ++c;

            // Check for stability
            if (dcs::math::float_traits<RealT>::essentially_greater_equal(lambda/(c*mu), 1.0))
            {
                continue;
            }

            auto rt = MMc_avg_response_time(lambda, mu, c);

            DCS_DEBUG_TRACE("lambda: " << lambda << ", mu: " << mu << ", c: " << c << " -> rt: " << rt << " vs. max RT: " << max_rt << " (tol: " << tol << ")");

            // Check for target response time
            if (dcs::math::float_traits<RealT>::essentially_less_equal(rt, max_rt, tol))
            {
                // Found a value of c such that the achieved response time is <= max response time
                DCS_DEBUG_TRACE("Found (lambda: " << lambda << ", mu: " << mu << ", c: " << c << " -> rt: " << rt << " vs. max RT: " << max_rt << " (tol: " << tol << "))");
                break;
            }
        }

        return c;
    }

    static RealT MMc_avg_response_time(RealT lambda, RealT mu, std::size_t c)
    {
DCS_DEBUG_TRACE("MMc avg response time - check for lambda " << lambda);
        if (dcs::math::float_traits<RealT>::essentially_equal(lambda, 0))
        {
            return 0;
        }

        auto rho = lambda/(c*mu);
        if (dcs::math::float_traits<RealT>::essentially_greater_equal(rho, 1.0))
        {
            std::ostringstream oss;
            oss << "System is not stable (lambda: " << lambda << ", mu: " << mu << ", c: " << c << ")";
            dcs::log_warn(DCS_LOGGING_AT, oss.str());

            return std::numeric_limits<RealT>::infinity();
        }

        if (c == 1)
        {
            return (1.0/mu)/(1.0-rho);
        }

        auto pm = MMc_Pm(lambda, mu, c);
        auto avgK = c*rho+(rho/(1-rho))*pm;

        return avgK/lambda;
    }

    static RealT MMc_pi0(RealT lambda, RealT mu, std::size_t c)
    {
        auto rho = lambda/(c*mu);
        auto part1 = (std::pow(c*rho,c)/factorial(c))*(1.0/(1.0-rho));
        RealT part2 = 0.0;
        for (std::size_t k = 0; k < c; ++k)
        {
            part2 += std::pow(c*rho,k)/factorial(k);
        }

        return 1/(part1+part2);
    }

    static RealT MMc_Pm(RealT lambda, RealT mu, std::size_t c)
    {
        auto rho = lambda/(c*mu);
        auto pi0 = MMc_pi0(lambda, mu, c);

        return (std::pow(c*rho, c)/(factorial(c)*(1-rho)))*pi0;
    }

#endif
}; // mmc_service_performance_model_t

}} // Namespace dcs::fog

#endif // DCS_FOG_SERVICE_PERFORMANCE_MMC_SERVICE_PERFORMANCE_MODEL_HPP
