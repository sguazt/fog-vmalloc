/* vim: set tabstop=4 expandtab shiftwidth=4 softtabstop=4: */

/**
 * \file dcs/fog/experiment.hpp
 *
 * \brief Function and types for performing experiments.
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

#ifndef DCS_FOG_EXPERIMENT_HPP
#define DCS_FOG_EXPERIMENT_HPP


#include <algorithm>
#include <array>
#include <boost/smart_ptr.hpp>
#include <cstddef>
#include <ctime>
#include <dcs/assert.hpp>
#include <dcs/debug.hpp>
#include <dcs/exception.hpp>
#include <dcs/fog/arrival_rate_estimators.hpp>
#include <dcs/fog/commons.hpp>
//#include <dcs/fog/MMc.hpp>
#include <dcs/fog/random.hpp>
#include <dcs/fog/simulator.hpp>
#include <dcs/fog/service_performance.hpp>
#include <dcs/fog/statistics.hpp>
#include <dcs/fog/user_mobility.hpp>
#include <dcs/fog/util.hpp>
#include <dcs/fog/vm_allocation.hpp>
#include <dcs/math/function/clamp.hpp>
#include <dcs/logging.hpp>
#include <fstream>
#include <functional>
#include <initializer_list>
#include <iostream>
#include <limits>
//#include <RndWalkPnt.hpp>
#include <memory>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>


namespace dcs { namespace fog {

/// Type for simulation experiments
template <typename RealT>
class experiment_t: public simulator_t<RealT>
{
private:
    typedef simulator_t<RealT> base_type;

    enum verbosity_level_t
    {
        none = 0,
        low = 1,
        low_medium = 2,
        medium = 5,
        high = 9
    }; // verbosity_level_t

    enum event_tag_t
    {
        vm_allocation_trigger_event
    }; // event_tag_t

    struct vm_allocation_trigger_event_state_t: public event_state_t
    {
        RealT start_time = -1;
        RealT stop_time = -1;
    }; // vm_allocation_trigger_event_state_t

    static const char csv_field_quote_ch = '"';
    static const char csv_field_sep_ch = ',';
    static constexpr const char* csv_field_na_value = "NA";
    static constexpr const char* csv_field_interval_stats_tag = "INTERVAL";
    static constexpr const char* csv_field_replication_stats_tag = "REPLICATION";
    static constexpr const char* csv_field_simulation_stats_tag = "SIMULATION";

    static constexpr std::size_t default_num_fn_categories = 0;
    static constexpr std::size_t default_num_svc_categories = 0;
    static constexpr std::size_t default_num_vm_categories = 0;
    static constexpr RealT default_fp_electricity_costs = 0;
    static constexpr RealT default_optim_relative_tolerance = 0;
    static constexpr RealT default_optim_time_limit = -1;
    static constexpr RealT default_ci_level = 0.95;
    static constexpr RealT default_ci_rel_precision = 0.04;
    static constexpr RealT default_service_delay_tolerance = 0;
    static constexpr verbosity_level_t default_verbosity = none;
    static constexpr arrival_rate_estimation_t default_svc_arr_rate_estimation = max_arrival_rate_estimation;
    static constexpr RealT default_fp_vm_allocation_interval = 0;
    static constexpr std::size_t default_num_fns = 0;
    static constexpr std::size_t default_num_svcs = 0;

    //static constexpr std::size_t rwp_num_args = 3;
    //static constexpr const char* rwp_py_module = "rndWayPoint";
    //static constexpr const char* rwp_py_function = "nextPlease";

public:
    experiment_t()
    : num_fn_categories_(default_num_fn_categories),
      num_svc_categories_(default_num_svc_categories),
      num_vm_categories_(default_num_vm_categories),
      fp_electricity_costs_(default_fp_electricity_costs),
      optim_relative_tolerance_(default_optim_relative_tolerance),
      optim_time_limit_(default_optim_time_limit),
      ci_level_(default_ci_level),
      ci_rel_precision_(default_ci_rel_precision),
      service_delay_tolerance_(default_service_delay_tolerance),
      verbosity_(default_verbosity),
      svc_arr_rate_estimation_(default_svc_arr_rate_estimation),
      //fp_revenue_policy_(by_vm_revenue_policy),
      //fp_penalty_policy_(by_service_penalty_policy),
      //fp_penalty_model_(distance_penalty_model),
      fp_vm_allocation_interval_(default_fp_vm_allocation_interval),
      num_fns_(default_num_fns),
      num_svcs_(default_num_svcs)
    {
////FIXME Uncomment and updates this below when all works
//#if 1
//        // Initialize Python script for the mobility model
//        // - Prepare the arguments (i.e., module and file names)
//# if 1 //FIXME: enable the code fragment below to use std:array instead of plain char**
//        std::size_t sz = 0;
//        // arg 0: not used
//        rwp_args_[0] = 0;
//        // arg 1: module name
//        sz = std::strlen(rwp_py_module)+1;
//        rwp_args_[1] = new char[sz];
//        std::strncpy(rwp_args_[1], rwp_py_module, sz);
//        // arg 2: function name
//        sz = std::strlen(rwp_py_function)+1;
//        rwp_args_[2] = new char[sz];
//        std::strncpy(rwp_args_[2], rwp_py_function, sz);
//        // sanity check
//        DCS_DEBUG_ASSERT( rwp_num_args == rwp_args_.size() );
//        // - Setup
//        rwp_.setup(rwp_args_.size(), rwp_args_.data());
//# else
//        rwp_args_ = new char*[rwp_num_args];
//        std::size_t sz = 0;
//        // arg 0: not used
//        rwp_args_[0] = 0;
//        // arg 1: module name
//        sz = std::strlen(rwp_py_module)+1;
//        rwp_args_[1] = new char[sz];
//        std::strncpy(rwp_args_[1], rwp_py_module, sz);
//        // arg 2: function name
//        sz = std::strlen(rwp_py_function)+1;
//        rwp_args_[2] = new char[sz];
//        std::strncpy(rwp_args_[2], rwp_py_function, sz);
//        // - Setup
//        rwp_.setup(rwp_num_args, rwp_args_);
//# endif
//#endif
//#if 0//XXX: this below is just for testing
//        char *argv[3];
//        argv[0] = 0;  // What value?
//        argv[1] = strdup("rndWayPoint"); //FIXME: memleak
//        argv[2] = strdup("nextPlease"); //FIXME: memleak
//        //char *argv[4];
//        //argv[0] = 0; // What value?
//        //argv[1] = strdup("rndWayPoint"); //FIXME: memleak
//        //argv[2] = strdup("nextPlease"); //FIXME: memleak
//        //argv[3] = strdup("/home/sguazt/Projects/src/fog-vmalloc/python/config.ini"); //FIXME: memleak
//        //rwp_.setup(sizeof argv/sizeof argv[0], argv);
//        //FIXME: mettere il file config.ini nella stessa dir da cui si lancia l'app
//#endif
    }

//    ~experiment_t()
//    {
////FIXME Uncomment and updates this below when all works
//#if 1
//        rwp_.dismiss();
//# if 1 //FIXME: enable the code fragment below to use std:array instead of plain char**
//        for (std::size_t i = 0; i < rwp_args_.size(); ++i)
//        {
//            if (rwp_args_[i])
//            {
//                delete[] rwp_args_[i];
//                rwp_args_[i] = 0;
//            }
//        }
//# else
//        for (std::size_t i = 0; i < rwp_num_args; ++i)
//        {
//            if (rwp_args_[i])
//            {
//                delete[] rwp_args_[i];
//                rwp_args_[i] = 0;
//            }
//        }
//        delete[] rwp_args_;
//# endif
//#endif
//    }

    void random_number_generator(random_number_engine_t& rng)
    {
        rng_ = rng;
    }

    random_number_engine_t& random_number_generator() const
    {
        return rng_;
    }

    void num_fog_node_categories(std::size_t value)
    {
        num_fn_categories_ = value;
    }

    std::size_t num_fog_node_categories() const
    {
        return num_fn_categories_;
    }

    void num_service_categories(std::size_t value)
    {
        num_svc_categories_ = value;
    }

    std::size_t num_service_categories() const
    {
        return num_svc_categories_;
    }

    void num_virtual_machine_categories(std::size_t value)
    {
        num_vm_categories_ = value;
    }

    std::size_t num_virtual_machine_categories() const
    {
        return num_vm_categories_;
    }

    template <typename IterT>
    void service_arrival_rates(IterT first, IterT last)
    {
        svc_arr_rates_.assign(first, last);
    }

    void service_arrival_rates(std::size_t svc_category, RealT max_rate)
    {
        if (svc_category >= svc_arr_rates_.size())
        {
            svc_arr_rates_.resize(svc_category+1);
        }

        svc_arr_rates_[svc_category] = max_rate;
    }

    std::vector<RealT> service_arrival_rates() const
    {
        return svc_arr_rates_;
    }

    template <typename IterT>
    void max_service_arrival_rates(IterT first, IterT last)
    {
        svc_max_arr_rates_.assign(first, last);
    }

    void max_service_arrival_rates(std::size_t svc_category, RealT max_rate)
    {
        if (svc_category >= svc_max_arr_rates_.size())
        {
            svc_max_arr_rates_.resize(svc_category+1);
        }

        svc_max_arr_rates_[svc_category] = max_rate;
    }

    std::vector<RealT> max_service_arrival_rates() const
    {
        return svc_max_arr_rates_;
    }

    template <typename IterT>
    void max_service_delays(IterT first, IterT last)
    {
        svc_max_delays_.assign(first, last);
    }

    void max_service_delay(std::size_t svc_category, RealT max_delay)
    {
        if (svc_category >= svc_max_delays_.size())
        {
            svc_max_delays_.resize(svc_category+1);
        }

        svc_max_delays_[svc_category] = max_delay;
    }

    std::vector<RealT> max_service_delays() const
    {
        return svc_max_delays_;
    }

/*
    template <typename IterT>
    void virtual_machine_categories(IterT first, IterT last)
    {
        svc_vm_categories_.assign(first, last);
    }

    void virtual_machine_category(std::size_t svc_category, std::size_t vm_category)
    {
        if (svc_category >= svc_vm_categories_.size())
        {
            svc_vm_categories_.resize(svc_category+1);
        }

        svc_vm_categories_[svc_category] = vm_category;
    }

    std::vector<std::size_t> virtual_machine_categories() const
    {
        return svc_vm_categories_;
    }
*/

    template <typename IterT>
    void virtual_machine_service_rates(IterT first, IterT last)
    {
        svc_vm_service_rates_.assign(first, last);
    }

    template <typename IterT>
    void virtual_machine_service_rate(std::size_t svc_category, IterT first, IterT last)
    {
        if (svc_category >= svc_vm_service_rates_.size())
        {
            svc_vm_service_rates_.resize(svc_category+1);
        }

        svc_vm_service_rates_[svc_category].assign(first, last);
    }

    void virtual_machine_service_rate(std::size_t svc_category, std::size_t vm_category, RealT service_rate)
    {
        if (svc_category >= svc_vm_service_rates_.size())
        {
            svc_vm_service_rates_.resize(svc_category+1);
        }

        if (vm_category >= svc_vm_service_rates_[svc_category].size())
        {
            svc_vm_service_rates_[svc_category].resize(vm_category+1);
        }

        svc_vm_service_rates_[svc_category][vm_category] = service_rate;
    }

    std::vector<std::vector<RealT>> virtual_machine_service_rates() const
    {
        return svc_vm_service_rates_;
    }

    template <typename IterT>
    void num_services(IterT first, IterT last)
    {
        fp_num_svcs_.assign(first, last);
    }

    void num_services(std::size_t svc_category, std::size_t num_svcs)
    {
        if (svc_category >= fp_num_svcs_.size())
        {
            fp_num_svcs_.resize(svc_category+1);
        }

        fp_num_svcs_[svc_category] = num_svcs;
    }

    std::vector<std::size_t> num_services() const
    {
        return fp_num_svcs_;
    }

    template <typename IterT>
    void num_fog_nodes(IterT first, IterT last)
    {
        fp_num_fns_.assign(first, last);
    }

    void num_fog_nodes(std::size_t fn_category, std::size_t num_fns)
    {
        if (fn_category >= fp_num_fns_.size())
        {
            fp_num_fns_.resize(fn_category+1);
        }

        fp_num_fns_[fn_category] = num_fns;
    }

    std::vector<std::size_t> num_fog_nodes() const
    {
        return fp_num_fns_;
    }

    void electricity_costs(RealT cost)
    {
        fp_electricity_costs_ = cost;
    }

    RealT electricity_costs() const
    {
        return fp_electricity_costs_;
    }

    template <typename IterT>
    void service_revenues(IterT first, IterT last)
    {
        fp_svc_revenues_.assign(first, last);
    }

    void service_revenue(std::size_t svc_category, RealT revenue)
    {
        if (svc_category >= fp_svc_revenues_.size())
        {
            fp_svc_revenues_.resize(svc_category+1);
        }

        fp_svc_revenues_[svc_category] = revenue;
    }

    std::vector<RealT> service_revenues() const
    {
        return fp_svc_revenues_;
    }

    template <typename IterT>
    void service_penalties(IterT first, IterT last)
    {
        fp_svc_penalties_.assign(first, last);
    }

    void service_penalty(std::size_t svc_category, RealT penalty)
    {
        if (svc_category >= fp_svc_penalties_.size())
        {
            fp_svc_penalties_.resize(svc_category+1);
        }

        fp_svc_penalties_[svc_category] = penalty;
    }

    std::vector<RealT> service_penalties() const
    {
        return fp_svc_penalties_;
    }

    template <typename IterT>
    void fog_node_asleep_costs(IterT first, IterT last)
    {
        fp_fn_asleep_costs_.assign(first, last);
    }

    void fog_node_asleep_cost(std::size_t fn_category, RealT cost)
    {
        if (fn_category >= fp_fn_asleep_costs_.size())
        {
            fp_fn_asleep_costs_.resize(fn_category+1);
        }

        fp_fn_asleep_costs_[fn_category] = cost;
    }

    std::vector<RealT> fog_node_asleep_costs() const
    {
        return fp_fn_asleep_costs_;
    }

    template <typename IterT>
    void fog_node_awake_costs(IterT first, IterT last)
    {
        fp_fn_awake_costs_.assign(first, last);
    }

    void fog_node_awake_cost(std::size_t fn_category, RealT cost)
    {
        if (fn_category >= fp_fn_awake_costs_.size())
        {
            fp_fn_awake_costs_.resize(fn_category+1);
        }

        fp_fn_awake_costs_[fn_category] = cost;
    }

    std::vector<RealT> fog_node_awake_costs() const
    {
        return fp_fn_awake_costs_;
    }

    template <typename IterT>
    void fog_node_min_power_consumptions(IterT first, IterT last)
    {
        fn_min_powers_.assign(first, last);
    }

    void fog_node_min_power_consumption(std::size_t fn_category, RealT watts)
    {
        if (fn_category >= fn_min_powers_.size())
        {
            fn_min_powers_.resize(fn_category+1);
        }

        fn_min_powers_[fn_category] = watts;
    }

    std::vector<RealT> fog_node_min_power_consumptions() const
    {
        return fn_min_powers_;
    }

    template <typename IterT>
    void fog_node_max_power_consumptions(IterT first, IterT last)
    {
        fn_max_powers_.assign(first, last);
    }

    void fog_node_max_power_consumption(std::size_t fn_category, RealT watts)
    {
        if (fn_category >= fn_min_powers_.size())
        {
            fn_max_powers_.resize(fn_category+1);
        }

        fn_max_powers_[fn_category] = watts;
    }

    std::vector<RealT> fog_node_max_power_consumptions() const
    {
        return fn_max_powers_;
    }

    template <typename IterT>
    void virtual_machine_cpu_requirements(IterT first, IterT last)
    {
        vm_cpu_requirements_.assign(first, last);
    }

    template <typename IterT>
    void virtual_machine_cpu_requirements(std::size_t vm_category, IterT first, IterT last)
    {
        if (vm_category >= vm_cpu_requirements_.size())
        {
            vm_cpu_requirements_.resize(vm_category+1);
        }

        vm_cpu_requirements_[vm_category].assign(first, last);
    }

    void virtual_machine_cpu_requirement(std::size_t vm_category, std::size_t fn_category, RealT requirement)
    {
        if (vm_category >= vm_cpu_requirements_.size())
        {
            vm_cpu_requirements_.resize(vm_category+1);
        }
        if (fn_category >= vm_cpu_requirements_[vm_category].size())
        {
            vm_cpu_requirements_[vm_category].resize(fn_category+1);
        }

        vm_cpu_requirements_[vm_category][fn_category] = requirement;
    }

    std::vector<std::vector<RealT>> virtual_machine_cpu_requirements() const
    {
        return vm_cpu_requirements_;
    }

    template <typename IterT>
    void virtual_machine_ram_requirements(IterT first, IterT last)
    {
        vm_ram_requirements_.assign(first, last);
    }

    template <typename IterT>
    void virtual_machine_ram_requirements(std::size_t vm_category, IterT first, IterT last)
    {
        if (vm_category >= vm_ram_requirements_.size())
        {
            vm_ram_requirements_.resize(vm_category+1);
        }

        vm_ram_requirements_[vm_category].assign(first, last);
    }

    void virtual_machine_ram_requirement(std::size_t vm_category, std::size_t fn_category, RealT requirement)
    {
        if (vm_category >= vm_ram_requirements_.size())
        {
            vm_ram_requirements_.resize(vm_category+1);
        }
        if (fn_category >= vm_ram_requirements_[vm_category].size())
        {
            vm_ram_requirements_[vm_category].resize(fn_category+1);
        }

        vm_ram_requirements_[vm_category][fn_category] = requirement;
    }

    std::vector<std::vector<RealT>> virtual_machine_ram_requirements() const
    {
        return vm_ram_requirements_;
    }

    template <typename IterT>
    void virtual_machine_allocation_costs(IterT first, IterT last)
    {
        vm_cat_alloc_costs_.assign(first, last);
    }

    void virtual_machine_allocation_costs(std::size_t vm_category, RealT cost)
    {
        if (vm_category >= vm_cat_alloc_costs_.size())
        {
            vm_cat_alloc_costs_.resize(vm_category+1);
        }

        vm_cat_alloc_costs_[vm_category] = cost;
    }

    std::vector<RealT> virtual_machine_allocation_costs() const
    {
        return vm_cat_alloc_costs_;
    }

    void fp_vm_allocation_trigger_interval(RealT value)
    {
        fp_vm_allocation_interval_ = value;
    }

    RealT fp_vm_allocation_trigger_interval() const
    {
        return fp_vm_allocation_interval_;
    }

    void optimization_relative_tolerance(RealT value)
    {
        optim_relative_tolerance_ = value;
    }

    RealT optimization_relative_tolerance() const
    {
        return optim_relative_tolerance_;
    }

    void optimization_max_duration(RealT value)
    {
        optim_time_limit_ = value;
    }

    RealT optimization_max_duration() const
    {
        return optim_time_limit_;
    }

    void output_stats_data_file(const std::string& path)
    {
        output_stats_data_file_ = path;
    }

    std::string output_stats_data_file() const
    {
        return output_stats_data_file_;
    }

    void output_trace_data_file(const std::string& path)
    {
        output_trace_data_file_ = path;
    }

    std::string output_trace_data_file() const
    {
        return output_trace_data_file_;
    }

    void confidence_interval_level(RealT value)
    {
        ci_level_ = value;
    }

    RealT confidence_interval_level() const
    {
        return ci_level_;
    }

    void confidence_interval_relative_precision(RealT value)
    {
        ci_rel_precision_ = value;
    }

    RealT confidence_interval_relative_precision() const
    {
        return ci_rel_precision_;
    }

    void service_delay_tolerance(RealT value)
    {
        service_delay_tolerance_ = value;
    }

    RealT service_delay_tolerance() const
    {
        return service_delay_tolerance_;
    }

    void verbosity_level(int value)
    {
        verbosity_ = value;
    }

    int verbosity_level() const
    {
        return verbosity_;
    }

    void service_arrival_rate_estimation(arrival_rate_estimation_t value)
    {
        svc_arr_rate_estimation_ = value;
    }

    arrival_rate_estimation_t service_arrival_rate_estimation() const
    {
        return svc_arr_rate_estimation_;
    }

    //void service_arrival_rate_estimation_perturbed_max_stdev(RealT value)
    //{
    //    svc_arr_rate_estimation_perturb_max_sd_ = value;
    //}

    //RealT service_arrival_rate_estimation_perturbed_max_stdev() const
    //{
    //    return svc_arr_rate_estimation_perturb_max_sd_;
    //}

    template <typename IterT>
    void service_arrival_rate_estimation_params(IterT first, IterT last)
    {
        svc_arr_rate_estimation_params_.assign(first, last);
    }

    std::vector<RealT> service_arrival_rate_estimation_params() const
    {
        return svc_arr_rate_estimation_params_;
    }

//    void fp_revenue_policy(fp_revenue_policy_t value)
//    {
//        fp_revenue_policy_ = value;
//    }

//    fp_revenue_policy_t fp_revenue_policy() const
//    {
//        return fp_revenue_policy_;
//    }

//    void fp_penalty_model(fp_penalty_model_t value)
//    {
//        fp_penalty_model_ = value;
//    }

//    fp_penalty_model_t fp_penalty_model() const
//    {
//        return fp_penalty_model_;
//    }

//    void fp_penalty_policy(fp_penalty_policy_t value)
//    {
//        fp_penalty_policy_ = value;
//    }

//    fp_penalty_policy_t fp_penalty_policy() const
//    {
//        return fp_penalty_policy_;
//    }

//    void fp_vm_allocation_policy(vm_allocation_policy_t value)
//    {
//        fp_vm_alloc_policy_ = value;
//    }

//    vm_allocation_policy_t fp_vm_allocation_policy() const
//    {
//        return fp_vm_alloc_policy_;
//    }

    void user_mobility_model(const std::shared_ptr<user_mobility_model_t>& p_model)
    {
        p_mob_model_ = p_model;
    }

    std::shared_ptr<user_mobility_model_t> user_mobility_model() const
    {
        return p_mob_model_;
    }

    void vm_allocation_solver(const std::shared_ptr<base_vm_allocation_solver_t<RealT>>& p_solver)
    {
        p_vm_alloc_solver_ = p_solver;
    }

    std::shared_ptr<base_vm_allocation_solver_t<RealT>> vm_allocation_solver() const
    {
        return p_vm_alloc_solver_;
    }

    void multislot_vm_allocation_solver(const std::shared_ptr<base_multislot_vm_allocation_solver_t<RealT>>& p_solver)
    {
        p_multislot_vm_alloc_solver_ = p_solver;
    }

    std::shared_ptr<base_multislot_vm_allocation_solver_t<RealT>> multislot_vm_allocation_solver() const
    {
        return p_multislot_vm_alloc_solver_;
    }


private:
    template <typename IterT>
    static bool check_stats(IterT first, IterT last)
    {
        while (first != last)
        {
            if (!(*first)->done() && !(*first)->unstable())
            {
                return false;
            }

            ++first;
        }

        return true;
    }

    void do_initialize_simulation()
    {
        // Fills FN data structures and compute the total number of FNs
        num_fns_ = 0;
        fn_categories_.clear();
        for (std::size_t fnc = 0; fnc < num_fn_categories_; ++fnc)
        {
            const std::size_t nfns = fp_num_fns_[fnc];

            for (std::size_t i = 0; i < nfns; ++i)
            {
                fn_categories_.push_back(fnc);
            }

            num_fns_ += nfns;
        }
        initial_fn_power_states_.resize(num_fns_, false); //FIXME: currently the initial FN power status is always set to "powered-off"

        // Fills service data structures and computes the total number of services
        num_svcs_ = 0;
        svc_categories_.clear();
        for (std::size_t svc_cat = 0; svc_cat < num_svc_categories_; ++svc_cat)
        {
            const std::size_t nsvcs = fp_num_svcs_[svc_cat];

            for (std::size_t i = 0; i < nsvcs; ++i)
            {
                svc_categories_.push_back(svc_cat);
            }

            num_svcs_ += nsvcs;
        }
        if (svc_arr_rates_.size() < num_svc_categories_)
        {
            svc_arr_rates_.resize(num_svc_categories_, std::numeric_limits<RealT>::infinity());
        }
        if (svc_max_arr_rates_.size() < num_svc_categories_)
        {
            svc_max_arr_rates_.resize(num_svc_categories_, std::numeric_limits<RealT>::infinity());
        }
        if (svc_max_delays_.size() < num_svc_categories_)
        {
            svc_max_delays_.resize(num_svc_categories_, std::numeric_limits<RealT>::infinity());
        }

        // Reset arrival rate estimators
        svc_arr_rate_estimators_.resize(num_svcs_);
        for (std::size_t svc = 0; svc < num_svcs_; ++svc)
        {
            switch (svc_arr_rate_estimation_)
            {
                case beta_arrival_rate_estimation:
                    if (svc_arr_rate_estimation_params_.size() >= 4)
                    {
                        svc_arr_rate_estimators_[svc] = std::make_shared<beta_arrival_rate_estimator_t<RealT>>(rng_, svc_arr_rate_estimation_params_[0], svc_arr_rate_estimation_params_[1], svc_arr_rate_estimation_params_[2], svc_arr_rate_estimation_params_[3]);
                    }
                    else
                    {
                        svc_arr_rate_estimators_[svc] = std::make_shared<beta_arrival_rate_estimator_t<RealT>>(rng_);
                    }
                    break;
                case ewma_arrival_rate_estimation:
                    if (svc_arr_rate_estimation_params_.size() > 0)
                    {
                        svc_arr_rate_estimators_[svc] = std::make_shared<ewma_arrival_rate_estimator_t<RealT>>(svc_arr_rate_estimation_params_[0]);
                    }
                    else
                    {
                        svc_arr_rate_estimators_[svc] = std::make_shared<ewma_arrival_rate_estimator_t<RealT>>();
                    }
                    break;
                case max_arrival_rate_estimation:
                    svc_arr_rate_estimators_[svc] = std::make_shared<max_arrival_rate_estimator_t<RealT>>();
                    break;
                case most_recently_observed_arrival_rate_estimation:
                    svc_arr_rate_estimators_[svc] = std::make_shared<most_recently_observed_arrival_rate_estimator_t<RealT>>();
                    break;
                case perturbed_max_arrival_rate_estimation:
                    svc_arr_rate_estimators_[svc] = std::make_shared<perturbed_max_arrival_rate_estimator_t<RealT>>(rng_,
                                                                                                                    svc_arr_rate_estimation_params_.size() > 0 ? svc_arr_rate_estimation_params_[0] : perturbed_max_arrival_rate_estimator_t<RealT>::default_mean,
                                                                                                                    svc_arr_rate_estimation_params_.size() > 1 ? svc_arr_rate_estimation_params_[1] : perturbed_max_arrival_rate_estimator_t<RealT>::default_standard_deviation);
                    break;
                case perturbed_most_recently_observed_arrival_rate_estimation:
                    svc_arr_rate_estimators_[svc] = std::make_shared<perturbed_most_recently_observed_arrival_rate_estimator_t<RealT>>(rng_,
                                                                                                                                       svc_arr_rate_estimation_params_.size() > 0 ? svc_arr_rate_estimation_params_[0] : perturbed_max_arrival_rate_estimator_t<RealT>::default_mean,
                                                                                                                                       svc_arr_rate_estimation_params_.size() > 1 ? svc_arr_rate_estimation_params_[1] : perturbed_max_arrival_rate_estimator_t<RealT>::default_standard_deviation);
                    break;
                case uniform_max_arrival_rate_estimation:
                    svc_arr_rate_estimators_[svc] = std::make_shared<uniform_max_arrival_rate_estimator_t<RealT>>(rng_);
                    break;
                case uniform_min_max_arrival_rate_estimation:
                    svc_arr_rate_estimators_[svc] = std::make_shared<uniform_min_max_arrival_rate_estimator_t<RealT>>(rng_);
                    break;
            }
        }

        initial_fn_vm_allocations_.resize(num_fns_); //FIXME: currently the initial VM allocation is always set to "no allocation"

        // Initialize confidence interval variables
        // * Local optimization
        //  - Predicted profit statistics
        fp_pred_profit_ci_stats_ = std::make_shared<ci_mean_estimator_t<RealT>>(ci_level_, ci_rel_precision_);
        fp_pred_profit_ci_stats_->name("LocalPredProfit");
        //  - Real profit statistics
        fp_real_profit_ci_stats_ = std::make_shared<ci_mean_estimator_t<RealT>>(ci_level_, ci_rel_precision_);
        fp_real_profit_ci_stats_->name("LocalRealProfit");
        //  - Predicted number of powered-on FNs statitics
        fp_pred_num_fns_ci_stats_ = std::make_shared<ci_mean_estimator_t<RealT>>(ci_level_, ci_rel_precision_);
        fp_pred_num_fns_ci_stats_->name("LocalPredNumFNs");
        //  - Real number of powered-on FNs statitics
        fp_real_num_fns_ci_stats_ = std::make_shared<ci_mean_estimator_t<RealT>>(ci_level_, ci_rel_precision_);
        fp_real_num_fns_ci_stats_->name("LocalRealNumFNs");
        //  - Predicted achieved service delays
        svc_pred_delay_ci_stats_.resize(num_svcs_);
        for (std::size_t svc = 0; svc < num_svcs_; ++svc)
        {
            std::ostringstream oss;

            //oss.str("");
            oss << "LocalPredDelay_{" << svc << "}";
            svc_pred_delay_ci_stats_[svc] = std::make_shared<ci_mean_estimator_t<RealT>>(ci_level_, ci_rel_precision_);
            svc_pred_delay_ci_stats_[svc]->name(oss.str());
        }
        //  - Real achieved service delays
        svc_real_delay_ci_stats_.resize(num_svcs_);
        for (std::size_t svc = 0; svc < num_svcs_; ++svc)
        {
            std::ostringstream oss;

            //oss.str("");
            oss << "LocalRealDelay_{" << svc << "}";
            svc_real_delay_ci_stats_[svc] = std::make_shared<ci_mean_estimator_t<RealT>>(ci_level_, ci_rel_precision_);
            svc_real_delay_ci_stats_[svc]->name(oss.str());
        }
        // * Global optimization
        //  - Predicted profit statistics
        global_fp_pred_profit_ci_stats_ = std::make_shared<ci_mean_estimator_t<RealT>>(ci_level_, ci_rel_precision_);
        global_fp_pred_profit_ci_stats_->name("GlobalPredProfit");
        //  - Real profit statistics
        global_fp_real_profit_ci_stats_ = std::make_shared<ci_mean_estimator_t<RealT>>(ci_level_, ci_rel_precision_);
        global_fp_real_profit_ci_stats_->name("GlobalRealProfit");
        //  - Predicted number of powered-on FNs statitics
        global_fp_pred_num_fns_ci_stats_ = std::make_shared<ci_mean_estimator_t<RealT>>(ci_level_, ci_rel_precision_);
        global_fp_pred_num_fns_ci_stats_->name("GlobalPredNumFNs");
        //  - Real number of powered-on FNs statitics
        global_fp_real_num_fns_ci_stats_ = std::make_shared<ci_mean_estimator_t<RealT>>(ci_level_, ci_rel_precision_);
        global_fp_real_num_fns_ci_stats_->name("GlobalRealNumFNs");

        // Initialize output files

        if (!output_stats_data_file_.empty())
        {
            stats_dat_ofs_.open(output_stats_data_file_.c_str());

            DCS_ASSERT(stats_dat_ofs_,
                       DCS_EXCEPTION_THROW(std::runtime_error, "Unable to open output stats data file"));

            stats_dat_ofs_  << csv_field_quote_ch  << "Timestamp" << csv_field_quote_ch
                            << csv_field_sep_ch << csv_field_quote_ch << "Tag" << csv_field_quote_ch
                            << csv_field_sep_ch << csv_field_quote_ch << "Replication" << csv_field_quote_ch
                            << csv_field_sep_ch << csv_field_quote_ch << "VM Allocation Start Time" << csv_field_quote_ch
                            << csv_field_sep_ch << csv_field_quote_ch << "VM Allocation Duration" << csv_field_quote_ch;
            // Headers for interval stats
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_quote_ch << "Interval - Local VM Alloc - FP - Predicted Profit" << csv_field_quote_ch;
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_quote_ch << "Interval - Local VM Alloc - FP - Real Profit" << csv_field_quote_ch;
            for (std::size_t svc = 0; svc < num_svcs_; ++svc)
            {
                stats_dat_ofs_  << csv_field_sep_ch << csv_field_quote_ch << "Interval - Local VM Alloc - Service " << svc << " - Predicted Delay" << csv_field_quote_ch
                                << csv_field_sep_ch << csv_field_quote_ch << "Interval - Local VM Alloc - Service " << svc << " - Predicted Delay vs. Max Delay" << csv_field_quote_ch;
                stats_dat_ofs_  << csv_field_sep_ch << csv_field_quote_ch << "Interval - Local VM Alloc - Service " << svc << " - Real Delay" << csv_field_quote_ch
                                << csv_field_sep_ch << csv_field_quote_ch << "Interval - Local VM Alloc - Service " << svc << " - Real Delay vs. Max Delay" << csv_field_quote_ch;
            }
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_quote_ch << "Interval - Local VM Alloc - FP - Predicted #FNs" << csv_field_quote_ch;
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_quote_ch << "Interval - Local VM Alloc - FP - Real #FNs" << csv_field_quote_ch;
            // Headers for replication stats
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_quote_ch << "Replication - Local VM Alloc - FP - Predicted Profit" << csv_field_quote_ch;
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_quote_ch << "Replication - Local VM Alloc - FP - Real Profit" << csv_field_quote_ch;
            for (std::size_t svc = 0; svc < num_svcs_; ++svc)
            {
                stats_dat_ofs_  << csv_field_sep_ch << csv_field_quote_ch << "Replication - Local VM Alloc - Service " << svc << " - Predicted Delay" << csv_field_quote_ch
                                << csv_field_sep_ch << csv_field_quote_ch << "Replication - Local VM Alloc - Service " << svc << " - Predicted Delay vs. Max Delay" << csv_field_quote_ch;
                stats_dat_ofs_  << csv_field_sep_ch << csv_field_quote_ch << "Replication - Local VM Alloc - Service " << svc << " - Real Delay" << csv_field_quote_ch
                                << csv_field_sep_ch << csv_field_quote_ch << "Replication - Local VM Alloc - Service " << svc << " - Real Delay vs. Max Delay" << csv_field_quote_ch;
            }
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_quote_ch << "Replication - Local VM Alloc - Predicted #FNs" << csv_field_quote_ch;
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_quote_ch << "Replication - Local VM Alloc - Real #FNs" << csv_field_quote_ch;
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_quote_ch << "Replication - Global VM Alloc - FP - Predicted Profit" << csv_field_quote_ch;
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_quote_ch << "Replication - Global VM Alloc - FP - Real Profit" << csv_field_quote_ch;
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_quote_ch << "Replication - Global VM Alloc - FP - Predicted #FNs" << csv_field_quote_ch;
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_quote_ch << "Replication - Global VM Alloc - FP - Real #FNs" << csv_field_quote_ch;
            // Headers for simulation stats
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_quote_ch << "Simulation - Local VM Alloc - FP - Mean Predicted Profit" << csv_field_quote_ch
                            << csv_field_sep_ch << csv_field_quote_ch << "Simulation - Local VM Alloc - FP - S.D. Predicted Profit" << csv_field_quote_ch;
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_quote_ch << "Simulation - Local VM Alloc - FP - Mean Real Profit" << csv_field_quote_ch
                            << csv_field_sep_ch << csv_field_quote_ch << "Simulation - Local VM Alloc - FP - S.D. Real Profit" << csv_field_quote_ch;
            for (std::size_t svc = 0; svc < num_svcs_; ++svc)
            {
                stats_dat_ofs_  << csv_field_sep_ch << csv_field_quote_ch << "Simulation - Local VM Alloc - Service " << svc << " - Mean Predicted Delay" << csv_field_quote_ch
                                << csv_field_sep_ch << csv_field_quote_ch << "Simulation - Local VM Alloc - Service " << svc << " - S.D. Predicted Delay" << csv_field_quote_ch
                                << csv_field_sep_ch << csv_field_quote_ch << "Simulation - Local VM Alloc - Service " << svc << " - Mean Predicted Delay vs. Max Delay" << csv_field_quote_ch;
                stats_dat_ofs_  << csv_field_sep_ch << csv_field_quote_ch << "Simulation - Local VM Alloc - Service " << svc << " - Mean Real Delay" << csv_field_quote_ch
                                << csv_field_sep_ch << csv_field_quote_ch << "Simulation - Local VM Alloc - Service " << svc << " - S.D. Real Delay" << csv_field_quote_ch
                                << csv_field_sep_ch << csv_field_quote_ch << "Simulation - Local VM Alloc - Service " << svc << " - Mean Real Delay vs. Max Delay" << csv_field_quote_ch;
            }
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_quote_ch << "Simulation - Local VM Alloc - FP - Mean Predicted #FNs" << csv_field_quote_ch
                            << csv_field_sep_ch << csv_field_quote_ch << "Simulation - Local VM Alloc - FP - S.D. Predicted #FNs" << csv_field_quote_ch;
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_quote_ch << "Simulation - Local VM Alloc - FP - Mean Real #FNs" << csv_field_quote_ch
                            << csv_field_sep_ch << csv_field_quote_ch << "Simulation - Local VM Alloc - FP - S.D. Real #FNs" << csv_field_quote_ch;
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_quote_ch << "Simulation - Global VM Alloc - FP - Mean Predicted Profit" << csv_field_quote_ch
                            << csv_field_sep_ch << csv_field_quote_ch << "Simulation - Global VM Alloc - FP - S.D. Predicted Profit" << csv_field_quote_ch;
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_quote_ch << "Simulation - Global VM Alloc - FP - Mean Real Profit" << csv_field_quote_ch
                            << csv_field_sep_ch << csv_field_quote_ch << "Simulation - Global VM Alloc - FP - S.D. Real Profit" << csv_field_quote_ch;
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_quote_ch << "Simulation - Global VM Alloc - FP - Mean Predicted #FNs" << csv_field_quote_ch
                            << csv_field_sep_ch << csv_field_quote_ch << "Simulation - Global VM Alloc - FP - S.D. Predicted #FNs" << csv_field_quote_ch;
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_quote_ch << "Simulation - Global VM Alloc - FP - Mean Real #FNs" << csv_field_quote_ch
                            << csv_field_sep_ch << csv_field_quote_ch << "Simulation - Global VM Alloc - FP - S.D. Real #FNs" << csv_field_quote_ch;
            stats_dat_ofs_ << std::endl;
        }

        if (!output_trace_data_file_.empty())
        {
            trace_dat_ofs_.open(output_trace_data_file_.c_str());

            DCS_ASSERT(trace_dat_ofs_,
                       DCS_EXCEPTION_THROW(std::runtime_error, "Unable to open output trace data file"));

            trace_dat_ofs_  << csv_field_quote_ch  << "Timestamp" << csv_field_quote_ch
                            << csv_field_sep_ch << csv_field_quote_ch  << "Replication" << csv_field_quote_ch
                            << csv_field_sep_ch << csv_field_quote_ch << "VM Allocation Start Time" << csv_field_quote_ch
                            << csv_field_sep_ch << csv_field_quote_ch << "VM Allocation Duration" << csv_field_quote_ch;
            trace_dat_ofs_  << csv_field_sep_ch << csv_field_quote_ch << "FP - Predicted Profit" << csv_field_quote_ch;
            trace_dat_ofs_  << csv_field_sep_ch << csv_field_quote_ch << "FP - Real Profit" << csv_field_quote_ch;
            for (std::size_t svc = 0; svc < num_svcs_; ++svc)
            {
                trace_dat_ofs_  << csv_field_sep_ch << csv_field_quote_ch << "Service " << svc << " - Predicted Arrival Rate" << csv_field_quote_ch
                                << csv_field_sep_ch << csv_field_quote_ch << "Service " << svc << " - Delay" << csv_field_quote_ch;
                trace_dat_ofs_  << csv_field_sep_ch << csv_field_quote_ch << "Service " << svc << " - Real Arrival Rate" << csv_field_quote_ch
                                << csv_field_sep_ch << csv_field_quote_ch << "Service " << svc << " - Real Delay" << csv_field_quote_ch;
            }
            trace_dat_ofs_  << csv_field_sep_ch << csv_field_quote_ch << "FP - Predicted #FNs" << csv_field_quote_ch;
            trace_dat_ofs_  << csv_field_sep_ch << csv_field_quote_ch << "FP - Real #FNs" << csv_field_quote_ch;
            trace_dat_ofs_ << std::endl;
         }
    }

    void do_finalize_simulation()
    {
        // Write stats to file
        if (stats_dat_ofs_.is_open())
        {
            auto const cur_timestamp = std::time(nullptr);

            stats_dat_ofs_  << cur_timestamp // Timestamp
                            << csv_field_sep_ch << csv_field_quote_ch << csv_field_simulation_stats_tag << csv_field_quote_ch // Stats tag
                            << csv_field_sep_ch << csv_field_na_value // Placeholder for replication number
                            << csv_field_sep_ch << csv_field_na_value // Placeholder for VM allocation start time
                            << csv_field_sep_ch << csv_field_na_value; // Placeholder for VM allocation duration
            // Output interval stats as NA since they are not available at simulation granularity
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_na_value; // Placeholder for local predicted profit
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_na_value; // Placeholder for local real profit
            for (std::size_t svc = 0; svc < num_svcs_; ++svc)
            {
                stats_dat_ofs_  << csv_field_sep_ch << csv_field_na_value // Placeholder for local predicted service delay
                                << csv_field_sep_ch << csv_field_na_value; // Placeholder for local predicted service delay vs. max delay
                stats_dat_ofs_  << csv_field_sep_ch << csv_field_na_value // Placeholder for local real service delay
                                << csv_field_sep_ch << csv_field_na_value; // Placeholder for local real service delay vs. max delay
            }
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_na_value; // Placeholder for local predicted #FNs
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_na_value; // Placeholder for local real #FNs
            // Output replication stats as NA since they are not available at simulation granularity
            // - Local VM allocation
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_na_value; // Placeholder for local predicted profit
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_na_value; // Placeholder for local real profit
            for (std::size_t svc = 0; svc < num_svcs_; ++svc)
            {
                stats_dat_ofs_  << csv_field_sep_ch << csv_field_na_value // Placeholder for local predicted service delay
                                << csv_field_sep_ch << csv_field_na_value; // Placeholder for local predicted service delay vs. max delay
                stats_dat_ofs_  << csv_field_sep_ch << csv_field_na_value // Placeholder for local real service delay
                                << csv_field_sep_ch << csv_field_na_value; // Placeholder for local real service delay vs. max delay
            }
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_na_value; // Placeholder for local predicted #FNs
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_na_value; // Placeholder for local real #FNs
            // - Global VM allocation
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_na_value; // Placeholder for global predicted profit
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_na_value; // Placeholder for global real profit
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_na_value; // Placeholder for global predicted #FNs
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_na_value; // Placeholder for global real #FNs
            // Output overall stats
            // - Local VM allocation
            stats_dat_ofs_  << csv_field_sep_ch << fp_pred_profit_ci_stats_->estimate() // Local predicted profit (mean)
                            << csv_field_sep_ch << fp_pred_profit_ci_stats_->standard_deviation(); // Local real profit (s.d.)
            stats_dat_ofs_  << csv_field_sep_ch << fp_real_profit_ci_stats_->estimate() // Local real profit (mean)
                            << csv_field_sep_ch << fp_real_profit_ci_stats_->standard_deviation(); // Local real profit (s.d.)
            for (std::size_t svc = 0; svc < num_svcs_; ++svc)
            {
                auto const svc_cat = svc_categories_[svc];
                stats_dat_ofs_  << csv_field_sep_ch << svc_pred_delay_ci_stats_[svc]->estimate() // Local predicted service delay (mean)
                                << csv_field_sep_ch << svc_pred_delay_ci_stats_[svc]->standard_deviation() // Local predicted service delay (s.d.)
                                << csv_field_sep_ch << relative_increment(svc_pred_delay_ci_stats_[svc]->estimate(), svc_max_delays_[svc_cat]); // Local predicted service delay vs. max delay
                stats_dat_ofs_  << csv_field_sep_ch << svc_real_delay_ci_stats_[svc]->estimate() // Local real service delay (mean)
                                << csv_field_sep_ch << svc_real_delay_ci_stats_[svc]->standard_deviation() // Local real service delay (s.d.)
                                << csv_field_sep_ch << relative_increment(svc_real_delay_ci_stats_[svc]->estimate(), svc_max_delays_[svc_cat]); // Local real service delay vs. max delay
            }
            stats_dat_ofs_  << csv_field_sep_ch << fp_pred_num_fns_ci_stats_->estimate() // Local predicted #FNs (mean)
                            << csv_field_sep_ch << fp_pred_num_fns_ci_stats_->standard_deviation(); // Local predicted #FNs (s.d.)
            stats_dat_ofs_  << csv_field_sep_ch << fp_real_num_fns_ci_stats_->estimate() // Local real #FNs (mean)
                            << csv_field_sep_ch << fp_real_num_fns_ci_stats_->standard_deviation(); // Local real #FNs (s.d.)
            // - Global VM allocation
            stats_dat_ofs_  << csv_field_sep_ch << global_fp_pred_profit_ci_stats_->estimate() // Global predicted profit (mean)
                            << csv_field_sep_ch << global_fp_pred_profit_ci_stats_->standard_deviation(); // Global predicted profit (s.d.)
            stats_dat_ofs_  << csv_field_sep_ch << global_fp_real_profit_ci_stats_->estimate() // Global real profit (mean)
                            << csv_field_sep_ch << global_fp_real_profit_ci_stats_->standard_deviation(); // Global real profit (s.d.)
            stats_dat_ofs_  << csv_field_sep_ch << global_fp_pred_num_fns_ci_stats_->estimate() // Global predicted #FNs (mean)
                            << csv_field_sep_ch << global_fp_pred_num_fns_ci_stats_->standard_deviation(); // Global predicted #FNs (s.d.)
            stats_dat_ofs_  << csv_field_sep_ch << global_fp_real_num_fns_ci_stats_->estimate() // Global predicted #FNs (mean)
                            << csv_field_sep_ch << global_fp_real_num_fns_ci_stats_->standard_deviation(); // Global predicted #FNs (s.d.)
            stats_dat_ofs_ << std::endl;
        }

        // Close output files
        if (stats_dat_ofs_.is_open())
        {
            stats_dat_ofs_.close();
        }
        if (trace_dat_ofs_.is_open())
        {
            trace_dat_ofs_.close();
        }

        // Output some info
        if (verbosity_ > none)
        {
            DCS_LOGGING_STREAM << "-- FINAL CONFIDENCE INTERVALS OUTPUTS:" << std::endl;
            DCS_LOGGING_STREAM << "  * Local VM Allocation:" << std::endl;
            DCS_LOGGING_STREAM << "   - FP" << std::endl;
            DCS_LOGGING_STREAM << "    - Predicted profit statistics: " << fp_pred_profit_ci_stats_->estimate() << " (s.d. " << fp_pred_profit_ci_stats_->standard_deviation() << ") [" << fp_pred_profit_ci_stats_->lower() << ", " << fp_pred_profit_ci_stats_->upper() << "] (rel. prec.: " << fp_pred_profit_ci_stats_->relative_precision() << ", size: " << fp_pred_profit_ci_stats_->size() << ", target size: " << fp_pred_profit_ci_stats_->target_size() << ", unstable: " << std::boolalpha << fp_pred_profit_ci_stats_->unstable() << ")" << std::endl;
            DCS_LOGGING_STREAM << "    - Real profit statistics: " << fp_real_profit_ci_stats_->estimate() << " (s.d. " << fp_real_profit_ci_stats_->standard_deviation() << ") [" << fp_real_profit_ci_stats_->lower() << ", " << fp_real_profit_ci_stats_->upper() << "] (rel. prec.: " << fp_real_profit_ci_stats_->relative_precision() << ", size: " << fp_real_profit_ci_stats_->size() << ", target size: " << fp_real_profit_ci_stats_->target_size() << ", unstable: " << std::boolalpha << fp_real_profit_ci_stats_->unstable() << ")" << std::endl;
            DCS_LOGGING_STREAM << "    - Predicted #FNs statistics: " << fp_pred_num_fns_ci_stats_->estimate() << " (s.d. " << fp_pred_num_fns_ci_stats_->standard_deviation() << ") [" << fp_pred_num_fns_ci_stats_->lower() << ", " << fp_pred_num_fns_ci_stats_->upper() << "] (rel. prec.: " << fp_pred_num_fns_ci_stats_->relative_precision() << ", size: " << fp_pred_num_fns_ci_stats_->size() << ", target size: " << fp_pred_num_fns_ci_stats_->target_size() << ", unstable: " << std::boolalpha << fp_pred_num_fns_ci_stats_->unstable() << ")" << std::endl;
            DCS_LOGGING_STREAM << "    - Real #FNs statistics: " << fp_real_num_fns_ci_stats_->estimate() << " (s.d. " << fp_real_num_fns_ci_stats_->standard_deviation() << ") [" << fp_real_num_fns_ci_stats_->lower() << ", " << fp_real_num_fns_ci_stats_->upper() << "] (rel. prec.: " << fp_real_num_fns_ci_stats_->relative_precision() << ", size: " << fp_real_num_fns_ci_stats_->size() << ", target size: " << fp_real_num_fns_ci_stats_->target_size() << ", unstable: " << std::boolalpha << fp_real_num_fns_ci_stats_->unstable() << ")" << std::endl;
            for (std::size_t svc = 0; svc < num_svcs_; ++svc)
            {
                DCS_LOGGING_STREAM << "   - Service " << svc << std::endl;
                DCS_LOGGING_STREAM << "    - Predicted delay statistics: " << svc_pred_delay_ci_stats_[svc]->estimate() << " (s.d. " << svc_pred_delay_ci_stats_[svc]->standard_deviation() << ") [" << svc_pred_delay_ci_stats_[svc]->lower() << ", " << svc_pred_delay_ci_stats_[svc]->upper() << "] (rel. prec.: " << svc_pred_delay_ci_stats_[svc]->relative_precision() << ", size: " << svc_pred_delay_ci_stats_[svc]->size() << ", target size: " << svc_pred_delay_ci_stats_[svc]->target_size() << ", unstable: " << std::boolalpha << svc_pred_delay_ci_stats_[svc]->unstable() << ")" << std::endl;
                DCS_LOGGING_STREAM << "    - Real delay statistics: " << svc_real_delay_ci_stats_[svc]->estimate() << " (s.d. " << svc_real_delay_ci_stats_[svc]->standard_deviation() << ") [" << svc_real_delay_ci_stats_[svc]->lower() << ", " << svc_real_delay_ci_stats_[svc]->upper() << "] (rel. prec.: " << svc_real_delay_ci_stats_[svc]->relative_precision() << ", size: " << svc_real_delay_ci_stats_[svc]->size() << ", target size: " << svc_real_delay_ci_stats_[svc]->target_size() << ", unstable: " << std::boolalpha << svc_real_delay_ci_stats_[svc]->unstable() << ")" << std::endl;
            }
            DCS_LOGGING_STREAM << "  * Global VM Allocation:" << std::endl;
            DCS_LOGGING_STREAM << "   - FP" << std::endl;
            DCS_LOGGING_STREAM << "    - Predicted profit statistics: " << global_fp_pred_profit_ci_stats_->estimate() << " (s.d. " << global_fp_pred_profit_ci_stats_->standard_deviation() << ") [" << global_fp_pred_profit_ci_stats_->lower() << ", " << global_fp_pred_profit_ci_stats_->upper() << "] (rel. prec.: " << global_fp_pred_profit_ci_stats_->relative_precision() << ", size: " << global_fp_pred_profit_ci_stats_->size() << ", target size: " << global_fp_pred_profit_ci_stats_->target_size() << ", unstable: " << std::boolalpha << global_fp_pred_profit_ci_stats_->unstable() << ")" << std::endl;
            DCS_LOGGING_STREAM << "    - Real profit statistics: " << global_fp_real_profit_ci_stats_->estimate() << " (s.d. " << global_fp_real_profit_ci_stats_->standard_deviation() << ") [" << global_fp_real_profit_ci_stats_->lower() << ", " << global_fp_real_profit_ci_stats_->upper() << "] (rel. prec.: " << global_fp_real_profit_ci_stats_->relative_precision() << ", size: " << global_fp_real_profit_ci_stats_->size() << ", target size: " << global_fp_real_profit_ci_stats_->target_size() << ", unstable: " << std::boolalpha << global_fp_real_profit_ci_stats_->unstable() << ")" << std::endl;
            DCS_LOGGING_STREAM << "    - Predicted #FNs statistics: " << global_fp_pred_num_fns_ci_stats_->estimate() << " (s.d. " << global_fp_pred_num_fns_ci_stats_->standard_deviation() << ") [" << global_fp_pred_num_fns_ci_stats_->lower() << ", " << global_fp_pred_num_fns_ci_stats_->upper() << "] (rel. prec.: " << global_fp_pred_num_fns_ci_stats_->relative_precision() << ", size: " << global_fp_pred_num_fns_ci_stats_->size() << ", target size: " << global_fp_pred_num_fns_ci_stats_->target_size() << ", unstable: " << std::boolalpha << global_fp_pred_num_fns_ci_stats_->unstable() << ")" << std::endl;
            DCS_LOGGING_STREAM << "    - Real #FNs statistics: " << global_fp_real_num_fns_ci_stats_->estimate() << " (s.d. " << global_fp_real_num_fns_ci_stats_->standard_deviation() << ") [" << global_fp_real_num_fns_ci_stats_->lower() << ", " << global_fp_real_num_fns_ci_stats_->upper() << "] (rel. prec.: " << global_fp_real_num_fns_ci_stats_->relative_precision() << ", size: " << global_fp_real_num_fns_ci_stats_->size() << ", target size: " << global_fp_real_num_fns_ci_stats_->target_size() << ", unstable: " << std::boolalpha << global_fp_real_num_fns_ci_stats_->unstable() << ")" << std::endl;
        }
    }

    void do_initialize_replication()
    {
        // Initialize FN power states
        //rep_fn_power_states_.resize(num_fns_);
        //std::fill(rep_fn_power_states_.begin(), rep_fn_power_states_.end(), false);
        rep_fn_power_states_.assign(initial_fn_power_states_.begin(), initial_fn_power_states_.end());
 
        // Initialize VM allocation
        rep_fn_vm_allocations_.assign(initial_fn_vm_allocations_.begin(), initial_fn_vm_allocations_.end());

        // Initialize members used for the global VM allocation
        rep_global_vm_alloc_duration_ = 0;
        rep_global_vm_alloc_interval_num_ = 0;
        rep_global_svc_vm_cat_real_min_num_vms_.clear();
        rep_global_svc_vm_cat_predicted_min_num_vms_.clear();

        // Initialize replication stats
        // - Local VM allocation
        rep_fp_pred_profits_ = 0;
        rep_fp_real_profits_ = 0;
        rep_fp_pred_num_fns_ = std::make_shared<mean_estimator_t<RealT>>();
        rep_fp_pred_num_fns_->name("LocalPredNumFNs");
        rep_fp_real_num_fns_ = std::make_shared<mean_estimator_t<RealT>>();
        rep_fp_real_num_fns_->name("LocalRealNumFNs");
        rep_svc_pred_delays_.resize(num_svcs_);
        for (std::size_t svc = 0; svc < num_svcs_; ++svc)
        {
            std::ostringstream oss;

            oss.str("");
            oss << "LocalRealDelay_{" << svc << "}";
            rep_svc_pred_delays_[svc] = std::make_shared<mean_estimator_t<RealT>>();
            rep_svc_pred_delays_[svc]->name(oss.str());
        }
        rep_svc_real_delays_.resize(num_svcs_);
        for (std::size_t svc = 0; svc < num_svcs_; ++svc)
        {
            std::ostringstream oss;

            oss.str("");
            oss << "LocalRealDelay_{" << svc << "}";
            rep_svc_real_delays_[svc] = std::make_shared<mean_estimator_t<RealT>>();
            rep_svc_real_delays_[svc]->name(oss.str());
        }
        // - Global VM allocation
        rep_global_fp_pred_profits_ = 0;
        rep_global_fp_real_profits_ = 0;
        rep_global_fp_pred_num_fns_ = std::make_shared<mean_estimator_t<RealT>>();
        rep_global_fp_pred_num_fns_->name("GlobalPredNumFNs");
        rep_global_fp_real_num_fns_ = std::make_shared<mean_estimator_t<RealT>>();
        rep_global_fp_real_num_fns_->name("GlobalRealNumFNs");


        // Schedule initial events

        auto p_state = std::make_shared<vm_allocation_trigger_event_state_t>();
        p_state->start_time = this->simulated_time();
        p_state->stop_time = this->simulated_time() + fp_vm_allocation_interval_;
        this->schedule_event(p_state->stop_time, vm_allocation_trigger_event, p_state);
    }

    void do_finalize_replication()
    {
        global_allocate_vms();

        auto const cur_timestamp = std::time(nullptr);

        // Collect stats
        // - For local VM allocation
        fp_pred_profit_ci_stats_->collect(rep_fp_pred_profits_);
        fp_real_profit_ci_stats_->collect(rep_fp_real_profits_);
        //fp_pred_num_fns_ci_stats_->collect(rep_fp_pred_num_fns_);
        fp_pred_num_fns_ci_stats_->collect(rep_fp_pred_num_fns_->estimate());
        //fp_real_num_fns_ci_stats_->collect(rep_fp_real_num_fns_);
        fp_real_num_fns_ci_stats_->collect(rep_fp_real_num_fns_->estimate());
        for (std::size_t svc = 0; svc < num_svcs_; ++svc)
        {
            svc_pred_delay_ci_stats_[svc]->collect(rep_svc_pred_delays_[svc]->estimate());
            svc_real_delay_ci_stats_[svc]->collect(rep_svc_real_delays_[svc]->estimate());
        }
        // - For global VM allocation
        global_fp_pred_profit_ci_stats_->collect(rep_global_fp_pred_profits_);
        global_fp_real_profit_ci_stats_->collect(rep_global_fp_real_profits_);
        global_fp_pred_num_fns_ci_stats_->collect(rep_global_fp_pred_num_fns_->estimate());
        global_fp_real_num_fns_ci_stats_->collect(rep_global_fp_real_num_fns_->estimate());

        if (verbosity_ >= low)
        {
            DCS_LOGGING_STREAM << "-- REPLICATION #" << this->num_replications() << std::endl;

            if (verbosity_ >= low_medium)
            {
                DCS_LOGGING_STREAM << " * SUMMARY OUTPUTS:" << std::endl;
                DCS_LOGGING_STREAM << "  - Local VM allocation: " << std::endl;
                DCS_LOGGING_STREAM << "   - Total Predicted Profits: " << rep_fp_pred_profits_ << std::endl;
                DCS_LOGGING_STREAM << "   - Total Real Profits: " << rep_fp_real_profits_ << std::endl;
                DCS_LOGGING_STREAM << "   - Total Predicted #FNs: " << rep_fp_pred_num_fns_->estimate() << std::endl;
                DCS_LOGGING_STREAM << "   - Total Real #FNs: " << rep_fp_real_num_fns_->estimate() << std::endl;
                DCS_LOGGING_STREAM << "   - Total Predicted Delays: [" << num_svcs_ << "]{";
                for (std::size_t svc = 0; svc < num_svcs_; ++svc)
                {
                    if (svc > 0)
                    {
                        DCS_LOGGING_STREAM << ",";
                    }

                    DCS_LOGGING_STREAM << rep_svc_pred_delays_[svc]->estimate();
                }
                DCS_LOGGING_STREAM << "}" << std::endl;
                DCS_LOGGING_STREAM << "   - Total Real Delays: [" << num_svcs_ << "]{";
                for (std::size_t svc = 0; svc < num_svcs_; ++svc)
                {
                    if (svc > 0)
                    {
                        DCS_LOGGING_STREAM << ",";
                    }

                    DCS_LOGGING_STREAM << rep_svc_real_delays_[svc]->estimate();
                }
                DCS_LOGGING_STREAM << "}" << std::endl;
                DCS_LOGGING_STREAM << "  - Global VM allocation: " << std::endl;
                DCS_LOGGING_STREAM << "   - Total Predicted Profits: " << rep_global_fp_pred_profits_ << std::endl;
                DCS_LOGGING_STREAM << "   - Total Real Profits: " << rep_global_fp_real_profits_ << std::endl;
                DCS_LOGGING_STREAM << "   - Total Predicted #FNs: " << rep_global_fp_pred_num_fns_->estimate() << std::endl;
                DCS_LOGGING_STREAM << "   - Total Real #FNs: " << rep_global_fp_real_num_fns_->estimate() << std::endl;
            }

            DCS_LOGGING_STREAM << "-- CONFIDENCE INTERVALS OUTPUTS:" << std::endl;
            DCS_LOGGING_STREAM << "  * Local VM allocation" << std::endl;
            DCS_LOGGING_STREAM << "   - FP" << std::endl;
            DCS_LOGGING_STREAM << "    - Predicted profit statistics: " << fp_pred_profit_ci_stats_->estimate() << " (s.d. " << fp_pred_profit_ci_stats_->standard_deviation() << ") [" << fp_pred_profit_ci_stats_->lower() << ", " << fp_pred_profit_ci_stats_->upper() << "] (rel. prec.: " << fp_pred_profit_ci_stats_->relative_precision() << ", size: " << fp_pred_profit_ci_stats_->size() << ", target size: " << fp_pred_profit_ci_stats_->target_size() << ", unstable: " << std::boolalpha << fp_pred_profit_ci_stats_->unstable() << ")" << std::endl;
            DCS_LOGGING_STREAM << "    - Real profit statistics: " << fp_real_profit_ci_stats_->estimate() << " (s.d. " << fp_real_profit_ci_stats_->standard_deviation() << ") [" << fp_real_profit_ci_stats_->lower() << ", " << fp_real_profit_ci_stats_->upper() << "] (rel. prec.: " << fp_real_profit_ci_stats_->relative_precision() << ", size: " << fp_real_profit_ci_stats_->size() << ", target size: " << fp_real_profit_ci_stats_->target_size() << ", unstable: " << std::boolalpha << fp_real_profit_ci_stats_->unstable() << ")" << std::endl;
            DCS_LOGGING_STREAM << "    - Predicted #FNs statistics: " << fp_pred_num_fns_ci_stats_->estimate() << " (s.d. " << fp_pred_num_fns_ci_stats_->standard_deviation() << ") [" << fp_pred_num_fns_ci_stats_->lower() << ", " << fp_pred_num_fns_ci_stats_->upper() << "] (rel. prec.: " << fp_pred_num_fns_ci_stats_->relative_precision() << ", size: " << fp_pred_num_fns_ci_stats_->size() << ", target size: " << fp_pred_num_fns_ci_stats_->target_size() << ", unstable: " << std::boolalpha << fp_pred_num_fns_ci_stats_->unstable() << ")" << std::endl;
            DCS_LOGGING_STREAM << "    - Real #FNs statistics: " << fp_real_num_fns_ci_stats_->estimate() << " (s.d. " << fp_real_num_fns_ci_stats_->standard_deviation() << ") [" << fp_real_num_fns_ci_stats_->lower() << ", " << fp_real_num_fns_ci_stats_->upper() << "] (rel. prec.: " << fp_real_num_fns_ci_stats_->relative_precision() << ", size: " << fp_real_num_fns_ci_stats_->size() << ", target size: " << fp_real_num_fns_ci_stats_->target_size() << ", unstable: " << std::boolalpha << fp_real_num_fns_ci_stats_->unstable() << ")" << std::endl;
            for (std::size_t svc = 0; svc < num_svcs_; ++svc)
            {
                DCS_LOGGING_STREAM << "   - Service " << svc << std::endl;
                DCS_LOGGING_STREAM << "    - Predicted delay statistics: " << svc_pred_delay_ci_stats_[svc]->estimate() << " (s.d. " << svc_pred_delay_ci_stats_[svc]->standard_deviation() << ") [" << svc_pred_delay_ci_stats_[svc]->lower() << ", " << svc_pred_delay_ci_stats_[svc]->upper() << "] (rel. prec.: " << svc_pred_delay_ci_stats_[svc]->relative_precision() << ", size: " << svc_pred_delay_ci_stats_[svc]->size() << ", target size: " << svc_pred_delay_ci_stats_[svc]->target_size() << ", unstable: " << std::boolalpha << svc_pred_delay_ci_stats_[svc]->unstable() << ")" << std::endl;
                DCS_LOGGING_STREAM << "    - Real delay statistics: " << svc_real_delay_ci_stats_[svc]->estimate() << " (s.d. " << svc_real_delay_ci_stats_[svc]->standard_deviation() << ") [" << svc_real_delay_ci_stats_[svc]->lower() << ", " << svc_real_delay_ci_stats_[svc]->upper() << "] (rel. prec.: " << svc_real_delay_ci_stats_[svc]->relative_precision() << ", size: " << svc_real_delay_ci_stats_[svc]->size() << ", target size: " << svc_real_delay_ci_stats_[svc]->target_size() << ", unstable: " << std::boolalpha << svc_real_delay_ci_stats_[svc]->unstable() << ")" << std::endl;
            }
            DCS_LOGGING_STREAM << "  * Global VM allocation" << std::endl;
            DCS_LOGGING_STREAM << "   + FP" << std::endl;
            DCS_LOGGING_STREAM << "    - Predicted profit statistics: " << global_fp_pred_profit_ci_stats_->estimate() << " (s.d. " << global_fp_pred_profit_ci_stats_->standard_deviation() << ") [" << global_fp_pred_profit_ci_stats_->lower() << ", " << global_fp_pred_profit_ci_stats_->upper() << "] (rel. prec.: " << global_fp_pred_profit_ci_stats_->relative_precision() << ", size: " << global_fp_pred_profit_ci_stats_->size() << ", target size: " << global_fp_pred_profit_ci_stats_->target_size() << ", unstable: " << std::boolalpha << global_fp_pred_profit_ci_stats_->unstable() << ")" << std::endl;
            DCS_LOGGING_STREAM << "    - Real profit statistics: " << global_fp_real_profit_ci_stats_->estimate() << " (s.d. " << global_fp_real_profit_ci_stats_->standard_deviation() << ") [" << global_fp_real_profit_ci_stats_->lower() << ", " << global_fp_real_profit_ci_stats_->upper() << "] (rel. prec.: " << global_fp_real_profit_ci_stats_->relative_precision() << ", size: " << global_fp_real_profit_ci_stats_->size() << ", target size: " << global_fp_real_profit_ci_stats_->target_size() << ", unstable: " << std::boolalpha << global_fp_real_profit_ci_stats_->unstable() << ")" << std::endl;
            DCS_LOGGING_STREAM << "    - Predicted #FNs statistics: " << global_fp_pred_num_fns_ci_stats_->estimate() << " (s.d. " << global_fp_pred_num_fns_ci_stats_->standard_deviation() << ") [" << global_fp_pred_num_fns_ci_stats_->lower() << ", " << global_fp_pred_num_fns_ci_stats_->upper() << "] (rel. prec.: " << global_fp_pred_num_fns_ci_stats_->relative_precision() << ", size: " << global_fp_pred_num_fns_ci_stats_->size() << ", target size: " << global_fp_pred_num_fns_ci_stats_->target_size() << ", unstable: " << std::boolalpha << global_fp_pred_num_fns_ci_stats_->unstable() << ")" << std::endl;
            DCS_LOGGING_STREAM << "    - Real #FNs statistics: " << global_fp_real_num_fns_ci_stats_->estimate() << " (s.d. " << global_fp_real_num_fns_ci_stats_->standard_deviation() << ") [" << global_fp_real_num_fns_ci_stats_->lower() << ", " << global_fp_real_num_fns_ci_stats_->upper() << "] (rel. prec.: " << global_fp_real_num_fns_ci_stats_->relative_precision() << ", size: " << global_fp_real_num_fns_ci_stats_->size() << ", target size: " << global_fp_real_num_fns_ci_stats_->target_size() << ", unstable: " << std::boolalpha << global_fp_real_num_fns_ci_stats_->unstable() << ")" << std::endl;
        }

        // Output to file
        if (stats_dat_ofs_.is_open())
        {
            stats_dat_ofs_  << cur_timestamp // Timestamp
                            << csv_field_sep_ch << csv_field_quote_ch << csv_field_replication_stats_tag << csv_field_quote_ch // Stats tag
                            << csv_field_sep_ch << this->num_replications() // Replication number
                            << csv_field_sep_ch << csv_field_na_value // Placeholder for VM allocation start time
                            << csv_field_sep_ch << csv_field_na_value; // Placeholder for VM allocation duration
            // Output interval stats as NA since they are not available at replication granularity
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_na_value; // Placeholder for local predicted profit
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_na_value; // Placeholder for local real profit
            for (std::size_t svc = 0; svc < num_svcs_; ++svc)
            {
                stats_dat_ofs_  << csv_field_sep_ch << csv_field_na_value // Placeholder for local predicted service delay
                                << csv_field_sep_ch << csv_field_na_value; // Placeholder for local predicted service delay vs. max delay
                stats_dat_ofs_  << csv_field_sep_ch << csv_field_na_value // Placeholder for local real service delay
                                << csv_field_sep_ch << csv_field_na_value; // Placeholder for local real service delay vs. max delay
            }
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_na_value; // Placeholder for local predicted #FNs
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_na_value; // Placeholder for local real #FNs
            // Output final replication stats
            // - Local VM allocation
            stats_dat_ofs_  << csv_field_sep_ch << rep_fp_pred_profits_; // Local predicted profit
            stats_dat_ofs_  << csv_field_sep_ch << rep_fp_real_profits_; // Local real profit
            for (std::size_t svc = 0; svc < num_svcs_; ++svc)
            {
                auto const svc_cat = svc_categories_[svc];
                stats_dat_ofs_  << csv_field_sep_ch << rep_svc_pred_delays_[svc]->estimate() // Local predicted service delay
                                << csv_field_sep_ch << relative_increment(rep_svc_pred_delays_[svc]->estimate(), svc_max_delays_[svc_cat]); // Local predicted service delay vs. max delay
                stats_dat_ofs_  << csv_field_sep_ch << rep_svc_real_delays_[svc]->estimate() // Local real service delay
                                << csv_field_sep_ch << relative_increment(rep_svc_real_delays_[svc]->estimate(), svc_max_delays_[svc_cat]); // Local real service delay vs. max delay
            }
            stats_dat_ofs_  << csv_field_sep_ch << rep_fp_pred_num_fns_->estimate(); // Local predicted #FNs
            stats_dat_ofs_  << csv_field_sep_ch << rep_fp_real_num_fns_->estimate(); // Local real #FNs
            // - Global VM allocation
            stats_dat_ofs_  << csv_field_sep_ch << rep_global_fp_pred_profits_; // Global predicted profit
            stats_dat_ofs_  << csv_field_sep_ch << rep_global_fp_real_profits_; // Global real profit
            stats_dat_ofs_  << csv_field_sep_ch << rep_global_fp_pred_num_fns_->estimate(); // Global predicted #FNs
            stats_dat_ofs_  << csv_field_sep_ch << rep_global_fp_real_num_fns_->estimate(); // Global real #FNs
            // Output overall stats
            // - Local VM allocation
            stats_dat_ofs_  << csv_field_sep_ch << fp_pred_profit_ci_stats_->estimate() // Local predicted profit (mean)
                            << csv_field_sep_ch << fp_pred_profit_ci_stats_->standard_deviation(); // Local predicted profit (s.d.)
            stats_dat_ofs_  << csv_field_sep_ch << fp_real_profit_ci_stats_->estimate() // Local real profit (mean)
                            << csv_field_sep_ch << fp_real_profit_ci_stats_->standard_deviation(); // Local real profit (s.d.)
            for (std::size_t svc = 0; svc < num_svcs_; ++svc)
            {
                auto const svc_cat = svc_categories_[svc];
                stats_dat_ofs_  << csv_field_sep_ch << svc_pred_delay_ci_stats_[svc]->estimate() // Local predicted service delay (mean)
                                << csv_field_sep_ch << svc_pred_delay_ci_stats_[svc]->standard_deviation() // Local predicted service delay (s.d.)
                                << csv_field_sep_ch << relative_increment(svc_pred_delay_ci_stats_[svc]->estimate(), svc_max_delays_[svc_cat]); // Local predicted service delay vs. max delay
                stats_dat_ofs_  << csv_field_sep_ch << svc_real_delay_ci_stats_[svc]->estimate() // Local real service delay (mean)
                                << csv_field_sep_ch << svc_real_delay_ci_stats_[svc]->standard_deviation() // Local real service delay (s.d.)
                                << csv_field_sep_ch << relative_increment(svc_real_delay_ci_stats_[svc]->estimate(), svc_max_delays_[svc_cat]); // Local real service delay vs. max delay
            }
            stats_dat_ofs_  << csv_field_sep_ch << fp_pred_num_fns_ci_stats_->estimate() // Local predicted #FNs (mean)
                            << csv_field_sep_ch << fp_pred_num_fns_ci_stats_->standard_deviation(); // Local predicted #FNs (s.d.)
            stats_dat_ofs_  << csv_field_sep_ch << fp_real_num_fns_ci_stats_->estimate() // Local real #FNs (mean)
                            << csv_field_sep_ch << fp_real_num_fns_ci_stats_->standard_deviation(); // Local real #FNs (s.d.)
            // - Global VM allocation
            stats_dat_ofs_  << csv_field_sep_ch << global_fp_pred_profit_ci_stats_->estimate() // Global predicted profit (mean)
                            << csv_field_sep_ch << global_fp_pred_profit_ci_stats_->standard_deviation(); // Global predicted profit (s.d.)
            stats_dat_ofs_  << csv_field_sep_ch << global_fp_real_profit_ci_stats_->estimate() // Global real profit (mean)
                            << csv_field_sep_ch << global_fp_real_profit_ci_stats_->standard_deviation(); // Global real profit (s.d.)
            stats_dat_ofs_  << csv_field_sep_ch << global_fp_pred_num_fns_ci_stats_->estimate() // Global predicted #FNs (mean)
                            << csv_field_sep_ch << global_fp_pred_num_fns_ci_stats_->standard_deviation(); // Global predicted #FNs (s.d.)
            stats_dat_ofs_  << csv_field_sep_ch << global_fp_real_num_fns_ci_stats_->estimate() // Global real #FNs (mean)
                            << csv_field_sep_ch << global_fp_real_num_fns_ci_stats_->standard_deviation(); // Global real #FNs (s.d.)
            stats_dat_ofs_ << std::endl;
        }
    }

    bool do_check_end_of_replication() const
    {
        return false;
    }

    bool do_check_end_of_simulation() const
    {
        return  check_stats(&fp_pred_profit_ci_stats_, &fp_pred_profit_ci_stats_+1)
                && check_stats(&fp_real_profit_ci_stats_, &fp_real_profit_ci_stats_+1)
                && check_stats(&global_fp_pred_profit_ci_stats_, &global_fp_pred_profit_ci_stats_+1)
                && check_stats(&global_fp_real_profit_ci_stats_, &global_fp_real_profit_ci_stats_+1);
    }


    void do_process_event(const std::shared_ptr<event_t<RealT>>& p_event)
    {
        switch (p_event->tag)
        {
            case vm_allocation_trigger_event:
                this->process_vm_allocation_trigger_event(p_event);
                break;
            default:
                dcs::log_warn(DCS_LOGGING_AT, "Unable to process events with tag " + stringify(p_event->tag));
                break;
        }
    }

    void process_vm_allocation_trigger_event(const std::shared_ptr<event_t<RealT>>& p_event)
    {
        // check: event is available
        DCS_ASSERT(p_event,
                   DCS_EXCEPTION_THROW(std::invalid_argument, "Event is not available"));

        auto p_state = std::dynamic_pointer_cast<vm_allocation_trigger_event_state_t>(p_event->p_state);
        // check: down-cast OK
        DCS_DEBUG_ASSERT( p_state );

        DCS_DEBUG_TRACE("Processing 'VM_ALLOCATION_TRIGGER' event - start: " << p_state->start_time << ", stop: " << p_state->stop_time << " (time: " << this->simulated_time() << ")");

        this->allocate_vms(*p_state);

        // Schedule a new VM allocation trigger event

        p_state = std::make_shared<vm_allocation_trigger_event_state_t>();
        p_state->start_time = this->simulated_time();
        p_state->stop_time = this->simulated_time() + fp_vm_allocation_interval_;
        this->schedule_event(p_state->stop_time, vm_allocation_trigger_event, p_state);

        ++rep_global_vm_alloc_interval_num_;
    }

    void allocate_vms(const vm_allocation_trigger_event_state_t& vm_alloc_state)
    {
        auto const cur_timestamp = std::time(nullptr);

        auto const vm_alloc_start_time = vm_alloc_state.start_time;
        auto const vm_alloc_stop_time = vm_alloc_state.stop_time;
        auto const vm_alloc_duration = vm_alloc_stop_time - vm_alloc_start_time;

        rep_global_vm_alloc_duration_ += vm_alloc_duration;

        std::normal_distribution<RealT> white_noise_rvg;

        // Determines the arrival rate of the requests according to the number of users arrived in the last interval

        std::vector<RealT> svc_predicted_arr_rates(num_svcs_);
        std::vector<RealT> svc_real_arr_rates(num_svcs_);
        std::vector<std::vector<std::vector<RealT>>> svc_vm_cat_predicted_delays(num_svcs_);
        std::vector<std::vector<std::vector<RealT>>> svc_vm_cat_real_delays(num_svcs_);
        std::vector<std::vector<std::size_t>> svc_vm_cat_predicted_min_num_vms(num_svcs_);
        std::vector<std::vector<std::size_t>> svc_vm_cat_real_min_num_vms(num_svcs_);
        rep_global_svc_vm_cat_predicted_min_num_vms_.resize(rep_global_svc_vm_cat_predicted_min_num_vms_.size()+1);
        rep_global_svc_vm_cat_predicted_min_num_vms_[rep_global_vm_alloc_interval_num_].resize(num_svcs_);
        rep_global_svc_vm_cat_real_min_num_vms_.resize(rep_global_svc_vm_cat_real_min_num_vms_.size()+1);
        rep_global_svc_vm_cat_real_min_num_vms_[rep_global_vm_alloc_interval_num_].resize(num_svcs_);
        for (std::size_t svc = 0; svc < num_svcs_; ++svc)
        {
            auto const svc_cat = svc_categories_[svc];

            std::size_t max_num_users = 0;

            max_num_users = p_mob_model_->next();
DCS_DEBUG_TRACE("SVC: " << svc << " - Mobility model - max num users: " << max_num_users);//XXX

            //auto const pred_arr_rate = std::min(max_num_users*svc_arr_rates_[svc_cat], svc_max_arr_rates_[svc_cat]);
            auto pred_arr_rate = max_num_users > 0 ? max_num_users*svc_arr_rates_[svc_cat] : 0;
DCS_DEBUG_TRACE("SVC: " << svc << " - Predicted arrival rate: " << pred_arr_rate << " vs. max arrival rate: " << svc_max_arr_rates_[svc_cat]);//XXX
            pred_arr_rate = std::min(pred_arr_rate, svc_max_arr_rates_[svc_cat]);

            svc_arr_rate_estimators_[svc]->collect(pred_arr_rate);

            //auto const real_arr_rate = std::min(svc_arr_rate_estimators_[svc]->estimate(), svc_max_arr_rates_[svc_cat]);
            auto real_arr_rate = svc_arr_rate_estimators_[svc]->estimate();
DCS_DEBUG_TRACE("SVC: " << svc << " - Real arrival rate: " << real_arr_rate << " vs. max arrival rate: " << svc_max_arr_rates_[svc_cat]);//XXX
            real_arr_rate = std::min(real_arr_rate, svc_max_arr_rates_[svc_cat]);

            svc_real_arr_rates[svc] = real_arr_rate;
            svc_predicted_arr_rates[svc] = pred_arr_rate;

            svc_vm_cat_predicted_delays[svc].resize(num_vm_categories_);
            svc_vm_cat_real_delays[svc].resize(num_vm_categories_);
            svc_vm_cat_predicted_min_num_vms[svc].resize(num_vm_categories_);
            svc_vm_cat_real_min_num_vms[svc].resize(num_vm_categories_);
            rep_global_svc_vm_cat_predicted_min_num_vms_[rep_global_vm_alloc_interval_num_][svc].resize(num_vm_categories_);
            rep_global_svc_vm_cat_real_min_num_vms_[rep_global_vm_alloc_interval_num_][svc].resize(num_vm_categories_);
            for (std::size_t vm_cat = 0; vm_cat < num_vm_categories_; ++vm_cat)
            {
                // Compute delays for this service according to real arrival rate
DCS_DEBUG_TRACE("CHECK - SVC: " << svc << ", VM Category: " << vm_cat << " - Estimating the min number of VMs with Trivedi's MMc - real arrival rate: " << real_arr_rate << ", service rate: " << svc_vm_service_rates_[svc_cat][vm_cat] << ", delay: " << svc_max_delays_[svc_cat] << ", tol: " << service_delay_tolerance_);
                auto real_min_num_vms = svc_perf_model_.min_num_vms(real_arr_rate, svc_vm_service_rates_[svc_cat][vm_cat], svc_max_delays_[svc_cat], service_delay_tolerance_);
                //auto svc_cat_real_delay = svc_perf_model_.average_response_time(real_arr_rate, svc_vm_service_rates_[svc_cat][vm_cat], real_min_num_vms);
                svc_vm_cat_real_delays[svc][vm_cat].resize(real_min_num_vms+1, std::numeric_limits<RealT>::infinity());
                if (real_min_num_vms > 0)
                {
                    for (std::size_t nvms = 1; nvms <= real_min_num_vms; ++nvms)
                    {
                        svc_vm_cat_real_delays[svc][vm_cat][nvms] = svc_perf_model_.average_response_time(real_arr_rate, svc_vm_service_rates_[svc_cat][vm_cat], nvms);
                    }
                }
                else
                {
                    svc_vm_cat_real_delays[svc][vm_cat][0] = 0;
                }
                svc_vm_cat_real_min_num_vms[svc][vm_cat] = real_min_num_vms;
                rep_global_svc_vm_cat_real_min_num_vms_[rep_global_vm_alloc_interval_num_][svc][vm_cat] = real_min_num_vms;
#if 0
                MMc<RealT> check_svc_perf_model(real_arr_rate, svc_vm_service_rates_[svc_cat][vm_cat], svc_max_delays_[svc_cat], service_delay_tolerance_);
                auto check_real_min_num_vms = check_svc_perf_model.computeQueueParameters(true);
                //check_svc_perf_model.getDelays(&svc_vm_cat_real_delays[svc][vm_cat]);
                //svc_vm_cat_real_min_num_vms[svc][vm_cat] = check_real_min_num_vms;
DCS_DEBUG_TRACE("CHECK SVC: " << svc << ", VM Category: " << vm_cat << " - M/M/c - real min number of VMs - old way: " << check_real_min_num_vms << ", new way: " << real_min_num_vms);//XXX
DCS_DEBUG_TRACE("CHECK SVC: " << svc << ", VM Category: " << vm_cat << " - M/M/c - real delay - old way: " << check_svc_perf_model.getDelay(check_real_min_num_vms) << ", new way: " << svc_vm_cat_real_delays[svc][vm_cat][real_min_num_vms]);//XXX
#endif

                // Predict delays for this service according to predicted arrival rate
DCS_DEBUG_TRACE("CHECK: SVC: " << svc << ", VM Category: " << vm_cat << " - estimating the min number of VMs with Trivedi's MMc - predicted arrival rate: " << pred_arr_rate << ", service rate: " << svc_vm_service_rates_[svc_cat][vm_cat] << ", delay: " << svc_max_delays_[svc_cat] << ", tol: " << service_delay_tolerance_);
                auto pred_min_num_vms = svc_perf_model_.min_num_vms(pred_arr_rate, svc_vm_service_rates_[svc_cat][vm_cat], svc_max_delays_[svc_cat], service_delay_tolerance_);
                //auto svc_cat_predicted_delay = svc_perf_model_.average_response_time(pred_arr_rate, svc_vm_service_rates_[svc_cat][vm_cat], pred_min_num_vms);
                svc_vm_cat_predicted_delays[svc][vm_cat].resize(pred_min_num_vms+1, std::numeric_limits<RealT>::infinity());
                if (pred_min_num_vms > 0)
                {
                    for (std::size_t nvms = 1; nvms <= pred_min_num_vms; ++nvms)
                    {
                        svc_vm_cat_predicted_delays[svc][vm_cat][nvms] = svc_perf_model_.average_response_time(pred_arr_rate, svc_vm_service_rates_[svc_cat][vm_cat], nvms);
                    }
                }
                else
                {
                    svc_vm_cat_predicted_delays[svc][vm_cat][0] = 0;
                }
                svc_vm_cat_predicted_min_num_vms[svc][vm_cat] = pred_min_num_vms;
                rep_global_svc_vm_cat_predicted_min_num_vms_[rep_global_vm_alloc_interval_num_][svc][vm_cat] = pred_min_num_vms;
#if 0
                check_svc_perf_model = MMc<RealT>(pred_arr_rate, svc_vm_service_rates_[svc_cat][vm_cat], svc_max_delays_[svc_cat], service_delay_tolerance_);
                auto check_pred_min_num_vms = check_svc_perf_model.computeQueueParameters(true);
                //check_svc_perf_model.getDelays(&svc_vm_cat_predicted_delays[svc][vm_cat]);
                //svc_vm_cat_predicted_min_num_vms[svc][vm_cat] = check_pred_min_num_vms;
DCS_DEBUG_TRACE("CHECK SVC: " << svc << ", VM Category: " << vm_cat << " - M/M/c - pred min number of VMs - old way: " << check_pred_min_num_vms << ", new way: " << pred_min_num_vms);//XXX
DCS_DEBUG_TRACE("CHECK SVC: " << svc << ", VM Category: " << vm_cat << " - M/M/c - pred delay - old way: " << check_svc_perf_model.getDelay(check_pred_min_num_vms) << ", new way: " << svc_vm_cat_predicted_delays[svc][vm_cat][pred_min_num_vms]);//XXX
#endif

                DCS_DEBUG_TRACE("Service: " << svc << ", max number of users: " << max_num_users << ", max arrival rate: " << svc_max_arr_rates_[svc_cat] << ", real arrival rate: " << real_arr_rate << ", predicted arrival rate: " << pred_arr_rate << ", service rate: " << svc_vm_service_rates_[svc_cat][vm_cat] << ", max delay: " << svc_max_delays_[svc_cat] << " -> Real min number of VMs: " << real_min_num_vms << ", Predicted min number of VMs: " << pred_min_num_vms << ", Real delay: " << svc_vm_cat_real_delays[svc][vm_cat].back() << ", Predicted delay: " << svc_vm_cat_predicted_delays[svc][vm_cat].back());
            }

            svc_arr_rate_estimators_[svc]->reset();
        }

        // Interval-related stats
        RealT fp_interval_pred_profits = std::numeric_limits<RealT>::quiet_NaN();
        RealT fp_interval_real_profits = std::numeric_limits<RealT>::quiet_NaN();
        RealT fp_interval_pred_num_fns = std::numeric_limits<RealT>::quiet_NaN();
        RealT fp_interval_real_num_fns = std::numeric_limits<RealT>::quiet_NaN();
        std::vector<RealT> svc_interval_pred_delays(num_svcs_, std::numeric_limits<RealT>::quiet_NaN());
        std::vector<RealT> svc_interval_real_delays(num_svcs_, std::numeric_limits<RealT>::quiet_NaN());

        // Allocate VMs to the FP

        //std::vector<std::size_t> fns;
        //std::vector<std::size_t> svcs;
        //std::vector<std::size_t> vms;
        std::vector<bool> fn_power_states(rep_fn_power_states_.begin(), rep_fn_power_states_.end()); //NOTE: we don't use rep_fn_power_states_ because we need to use the same FN power states both in the optimization for the predicted workload and in the one for the real workload
        std::vector<std::map<std::size_t, std::pair<std::size_t,std::size_t>>> fn_vm_allocations(rep_fn_vm_allocations_.begin(), rep_fn_vm_allocations_.end()); //NOTE: we don't use rep_fn_vm_allocations_ because we need to use the same VM allocations both in the optimization for the predicted workload and in the one for the real workload

        //for (std::size_t fn = 0; fn < num_fns_; ++fn)
        //{
        //    //fns.push_back(fn);
        //    fn_power_states.push_back(rep_fn_power_states_[fn]);
        //}

        //for (std::size_t fn = 0; fn < num_fns_; ++fn)
        //{
        //    fn_vm_allocations.push_back(rep_fn_vm_allocations_[fn]);
        //}

        vm_allocation_t<RealT> vm_alloc;

//        std::unique_ptr<base_vm_allocation_solver_t<RealT>> p_vm_alloc_solver;
//#if 1
//        p_vm_alloc_solver = std::make_unique<optimal_vm_allocation_solver_t<RealT>>(optim_relative_tolerance_, optim_time_limit_);
//#else
//        p_vm_alloc_solver = std::make_unique<bahreini2017_mcappim_vm_allocation_solver_t<RealT>>();
//#endif

        // Compute VM allocation according to predicted workload

        vm_alloc = p_vm_alloc_solver_->solve(//fns,
                                            fn_categories_,
                                            fn_power_states,
                                            fn_vm_allocations,
                                            fn_min_powers_,
                                            fn_max_powers_,
                                            vm_cpu_requirements_,
                                            //vm_ram_requirements_,
                                            vm_cat_alloc_costs_,
                                            svc_categories_,
                                            svc_vm_cat_predicted_min_num_vms,
                                            fp_svc_revenues_,
                                            fp_svc_penalties_,
                                            fp_electricity_costs_,
                                            fp_fn_asleep_costs_,
                                            fp_fn_awake_costs_);

        if (vm_alloc.solved)
        {
#ifdef DCS_DEBUG
            DCS_DEBUG_TRACE( "--- VM ALLOCATION SOLUTION (predicted workload) -------------------------------[" );
            DCS_DEBUG_TRACE( "- Objective value: " << vm_alloc.objective_value << " (revenue: " << vm_alloc.revenue << ", cost: " << vm_alloc.cost << ")");
            DCS_DEBUG_TRACE( "- Solved: " << std::boolalpha << vm_alloc.solved << ", Optimal: " << std::boolalpha << vm_alloc.optimal << std::noboolalpha );

            DCS_DEBUG_TRACE( "- FN-VM allocations:" );
            for (std::size_t fn = 0; fn < vm_alloc.fn_vm_allocations.size(); ++fn)
            {
                DCS_DEBUG_STREAM << "FN #" << fn << " -> {";
                for (auto const& svc_vms : vm_alloc.fn_vm_allocations[fn])
                {
                    auto const svc = svc_vms.first;
                    auto const& vmcat_nvms = svc_vms.second;
                    DCS_DEBUG_STREAM << ", SVC #" << svc << " -> <VM Category: " << vmcat_nvms.first << ", #VMs: " << vmcat_nvms.second << ">";
                }
                DCS_DEBUG_STREAM << "}" << std::endl;
            }

            DCS_DEBUG_TRACE( "- FN power status:" );
            for (std::size_t fn = 0; fn < vm_alloc.fn_power_states.size(); ++fn)
            {
                DCS_DEBUG_STREAM << "FN #" << fn << " = " << std::boolalpha << vm_alloc.fn_power_states[fn] << std::noboolalpha << std::endl;
            }

            DCS_DEBUG_TRACE( "- FN CPU allocations:" );
            for (std::size_t fn = 0; fn < vm_alloc.fn_cpu_allocations.size(); ++fn)
            {
                DCS_DEBUG_STREAM << "FN #" << fn << " = " << vm_alloc.fn_cpu_allocations[fn] << std::endl;
            }

            DCS_DEBUG_TRACE( "]-------------------------------------------------------------------------------" ); 
#endif // DCS_DEBUG 

            if (!check_vm_allocation_solution(vm_alloc))
            {
                DCS_EXCEPTION_THROW( std::runtime_error, "Returned VM allocation solution is not consistent" );
            }

            // - Compute the profit for the whole interval
            auto const profit = (vm_alloc.revenue-vm_alloc.cost)*vm_alloc_duration;

            DCS_DEBUG_TRACE( "FP - Predicted workload - VM allocation objective value: " << vm_alloc.objective_value << " => profit: " << profit << " (revenue rate: " << vm_alloc.revenue << ", cost rate: " << vm_alloc.cost << ", interval duration: " << vm_alloc_duration << ")");

            // Update state and stats

            // - Update achieved profit
            fp_interval_pred_profits = profit;

            // - Update the info with the achieved delays
            //   Compute the number of VMs allocated (both on this FP's FNs and on other FP's FNs) for every service this FP runs and update achieved delays stats
            for (std::size_t svc = 0; svc < num_svcs_; ++svc)
            {
                std::size_t svc_num_alloc_vms = 0;
                std::size_t svc_vm_cat = 0;
                for (std::size_t fn = 0; fn < num_fns_; ++fn)
                {
                    if (vm_alloc.fn_vm_allocations.at(fn).count(svc) > 0)
                    {
                        svc_vm_cat = vm_alloc.fn_vm_allocations.at(fn).at(svc).first; // NOTE: all VMs allocated to the same service (including those allocated on different FNs) belong to the same category.
                        svc_num_alloc_vms += vm_alloc.fn_vm_allocations.at(fn).at(svc).second;
                        //rep_fn_vm_allocations_[fn][svc] = std::make_pair(vm_alloc.fn_vm_allocations.at(fn).at(svc).first, vm_alloc.fn_vm_allocations.at(fn).at(svc).second); //NO: this does not erase previous mappings
                    }
                }

                // Sanity checks
                assert( svc_vm_cat_predicted_delays.size() > svc );
                assert( svc_vm_cat_predicted_delays[svc].size() > svc_vm_cat );
                assert( svc_vm_cat_predicted_delays[svc][svc_vm_cat].size() >= svc_num_alloc_vms );

                svc_interval_pred_delays[svc] = svc_vm_cat_predicted_delays[svc][svc_vm_cat][svc_num_alloc_vms];
            }
            rep_fn_vm_allocations_.assign(vm_alloc.fn_vm_allocations.begin(), vm_alloc.fn_vm_allocations.end());

            // - Update the power status of each FN and the stats concerning the number of powered-on FNs
            fp_interval_pred_num_fns = 0;
            for (std::size_t i = 0; i < num_fns_; ++i)
            {
                if (vm_alloc.fn_power_states[i])
                {
                    fp_interval_pred_num_fns += 1;
                    rep_fn_power_states_[i] = true;
                }
                else
                {
                    rep_fn_power_states_[i] = false;
                }
            }
        }
        else
        {
            DCS_DEBUG_TRACE( "FP - Predicted workload - The VM assignment problem is infeasible" );
        }
DCS_DEBUG_TRACE("SOLUTION VM ALLOCATIONS: " << vm_alloc.fn_vm_allocations);//XXX
DCS_DEBUG_TRACE("LOCAL VM ALLOCATIONS: " << fn_vm_allocations);//XXX
DCS_DEBUG_TRACE("REP VM ALLOCATIONS: " << rep_fn_vm_allocations_);//XXX

        // Compute VM allocation according to real workload

#if defined(DCS_FOG_VMALLOC_REAL_WORKLOAD_ALLOCATE_ALL)

        // Solve again the optimization model by optimally reassigning VMs to FNs
        vm_alloc = p_vm_alloc_solver_->solve(//fns,
                                            fn_categories_,
                                            fn_power_states,
                                            fn_min_powers_,
                                            fn_max_powers_,
                                            vm_cpu_requirements_,
                                            //vm_ram_requirements_,
                                            svc_categories_,
                                            svc_vm_cat_real_min_num_vms,
                                            fp_svc_revenues_,
                                            fp_svc_penalties_,
                                            fp_electricity_costs_,
                                            fp_fn_asleep_costs_,
                                            fp_fn_awake_costs_);

        if (vm_alloc.solved)
        {
#ifdef DCS_DEBUG
            DCS_DEBUG_TRACE( "--- VM ALLOCATION SOLUTION (real workload) ------------------------------------[" );
            DCS_DEBUG_TRACE( "- Objective value: " << vm_alloc.objective_value << " (revenue: " << vm_alloc.revenue << ", cost: " << vm_alloc.cost << ")");
            DCS_DEBUG_TRACE( "- Solved: " << std::boolalpha << vm_alloc.solved << ", Optimal: " << std::boolalpha << vm_alloc.optimal << std::noboolalpha );

            DCS_DEBUG_TRACE( "- FN-VM allocations:" );
            for (std::size_t fn = 0; fn < vm_alloc.fn_vm_allocations.size(); ++fn)
            {
                DCS_DEBUG_STREAM << "FN #" << fn << " -> {";
                for (auto const& svc_vms : vm_alloc.fn_vm_allocations[fn])
                {
                    auto const svc = svc_vms.first;
                    auto const& vmcat_nvms = svc_vms.second;
                    DCS_DEBUG_STREAM << ", SVC #" << svc << " -> <VM Category: " << vmcat_nvms.first << ", #VMs: " << vmcat_nvms.second << ">";
                }
                DCS_DEBUG_STREAM << "}" << std::endl;
            }

            DCS_DEBUG_TRACE( "- FN power status:" );
            for (std::size_t fn = 0; fn < vm_alloc.fn_power_states.size(); ++fn)
            {
                DCS_DEBUG_STREAM << "FN #" << fn << " = " << std::boolalpha << vm_alloc.fn_power_states[fn] << std::noboolalpha << std::endl;
            }

            DCS_DEBUG_TRACE( "- FN CPU allocations:" );
            for (std::size_t fn = 0; fn < vm_alloc.fn_cpu_allocations.size(); ++fn)
            {
                DCS_DEBUG_STREAM << "FN #" << fn << " = " << vm_alloc.fn_cpu_allocations[fn] << std::endl;
            }

            DCS_DEBUG_TRACE( "]-------------------------------------------------------------------------------" ); 
#endif // DCS_DEBUG 

            if (!check_vm_allocation_solution(vm_alloc))
            {
                DCS_EXCEPTION_THROW( std::runtime_error, "Returned VM allocation solution is not consistent" );
            }

            // - Compute the profit for the whole interval
            auto const profit = (vm_alloc.revenue-vm_alloc.cost)*vm_alloc_duration;

            DCS_DEBUG_TRACE( "FP - Real workload - VM allocation objective value: " << vm_alloc.objective_value << " => profit: " << profit << " (revenue rate: " << vm_alloc.revenue << ", cost rate: " << vm_alloc.cost << ", interval duration: " << vm_alloc_duration << ")");

            // Update stats

            // - Update achieved profit
            fp_interval_real_profits = profit;

            // - Update the info with the achieved delays
            //   Compute the number of VMs allocated (both on this FP's FNs and on other FP's FNs) for every service this FP runs and update achieved delays stats
            for (std::size_t svc = 0; svc < num_svcs_; ++svc)
            {
                std::size_t svc_num_alloc_vms = 0;
                std::size_t svc_vm_cat = 0;
                for (std::size_t fn = 0; fn < num_fns_; ++fn)
                {
                    if (vm_alloc.fn_vm_allocations.at(fn).count(svc) > 0)
                    {
                        svc_vm_cat = vm_alloc.fn_vm_allocations.at(fn).at(svc).first; // NOTE: all VMs allocated to the same service (including those allocated on different FNs) belong to the same category.
                        svc_num_alloc_vms += vm_alloc.fn_vm_allocations.at(fn).at(svc).second;
                    }
                }

                // Sanity checks
                assert( svc_vm_cat_real_delays.size() > svc );
                assert( svc_vm_cat_real_delays[svc].size() > svc_vm_cat );
                assert( svc_vm_cat_real_delays[svc][svc_vm_cat].size() >= svc_num_alloc_vms );

                svc_interval_real_delays[svc] = svc_vm_cat_real_delays[svc][svc_vm_cat][svc_num_alloc_vms];
            }

            // - Update the power status of each FN and the stats with the number of powered-on FNs
            fp_interval_real_num_fns = 0;
            for (std::size_t i = 0; i < num_fns_; ++i)
            {
                if (vm_alloc.fn_power_states[i])
                {
                    fp_interval_real_num_fns += 1;
                    //rep_fn_power_states_[i] = true; //XXX: already done in the optimization for the "predicted" workload
                }
                else
                {
                    //rep_fn_power_states_[i] = false; //XXX: already done in the optimization for the "predicted" workload
                }
            }
        }
        else
        {
            DCS_DEBUG_TRACE( "FP - Real workload - The VM assignment problem is infeasible" );
        }

#elif defined(DCS_FOG_VMALLOC_REAL_WORKLOAD_ALLOCATE_WITH_FIXED_FNS)

        // Solve the VM allocation problem by limiting the FNs to the ones powered-on by the solution of the VM allocation problem for the predicted workload
        //std::vector<std::size_t> fixed_fns;
        std::set<std::size_t> fixed_fns;
        //std::vector<std::size_t> fixed_fn_categories;
        for (std::size_t fn = 0; fn < num_fns_; ++fn)
        {
            if (vm_alloc.fn_power_states[fn])
            {
                //fixed_fns.push_back(fn);
                //fixed_fn_categories.push_back(fn_categories_[fn]);
                fixed_fns.insert(fn);
            }
        }
        vm_alloc = p_vm_alloc_solver_->solve_with_fixed_fns(fixed_fns,
                                                           fn_categories_,
                                                           fn_power_states,
                                                           fn_vm_allocations,
                                                           fn_min_powers_,
                                                           fn_max_powers_,
                                                           vm_cpu_requirements_,
                                                           //vm_ram_requirements_,
                                                           vm_cat_alloc_costs_,
                                                           svc_categories_,
                                                           svc_vm_cat_real_min_num_vms,
                                                           fp_svc_revenues_,
                                                           fp_svc_penalties_,
                                                           fp_electricity_costs_,
                                                           fp_fn_asleep_costs_,
                                                           fp_fn_awake_costs_);

        if (vm_alloc.solved)
        {
#ifdef DCS_DEBUG
            DCS_DEBUG_TRACE( "--- VM ALLOCATION SOLUTION (real workload) ------------------------------------[" );
            DCS_DEBUG_TRACE( "- Objective value: " << vm_alloc.objective_value << " (revenue: " << vm_alloc.revenue << ", cost: " << vm_alloc.cost << ")");
            DCS_DEBUG_TRACE( "- Solved: " << std::boolalpha << vm_alloc.solved << ", Optimal: " << std::boolalpha << vm_alloc.optimal << std::noboolalpha );

            DCS_DEBUG_TRACE( "- FN-VM allocations:" );
            for (std::size_t fn = 0; fn < vm_alloc.fn_vm_allocations.size(); ++fn)
            {
                DCS_DEBUG_STREAM << "FN #" << fn << " -> {";
                for (auto const& svc_vms : vm_alloc.fn_vm_allocations[fn])
                {
                    auto const svc = svc_vms.first;
                    auto const& vmcat_nvms = svc_vms.second;
                    DCS_DEBUG_STREAM << ", SVC #" << svc << " -> <VM Category: " << vmcat_nvms.first << ", #VMs: " << vmcat_nvms.second << ">";
                }
                DCS_DEBUG_STREAM << "}" << std::endl;
            }

            DCS_DEBUG_TRACE( "- FN power status:" );
            for (std::size_t fn = 0; fn < vm_alloc.fn_power_states.size(); ++fn)
            {
                DCS_DEBUG_STREAM << "FN #" << fn << " = " << std::boolalpha << vm_alloc.fn_power_states[fn] << std::noboolalpha << std::endl;
            }

            DCS_DEBUG_TRACE( "- FN CPU allocations:" );
            for (std::size_t fn = 0; fn < vm_alloc.fn_cpu_allocations.size(); ++fn)
            {
                DCS_DEBUG_STREAM << "FN #" << fn << " = " << vm_alloc.fn_cpu_allocations[fn] << std::endl;
            }

            DCS_DEBUG_TRACE( "]-------------------------------------------------------------------------------" ); 
#endif // DCS_DEBUG 

            if (!check_vm_allocation_solution(vm_alloc))
            {
                DCS_EXCEPTION_THROW( std::runtime_error, "Returned VM allocation solution is not consistent" );
            }

            // - Compute the profit for the whole interval
            auto const profit = (vm_alloc.revenue-vm_alloc.cost)*vm_alloc_duration;

            DCS_DEBUG_TRACE( "FP - Real workload - VM allocation objective value: " << vm_alloc.objective_value << " => profit: " << profit << " (revenue rate: " << vm_alloc.revenue << ", cost rate: " << vm_alloc.cost << ", interval duration: " << vm_alloc_duration << ")");

            // Update stats

            // - Update achieved profit
            fp_interval_real_profits = profit;

            // - Update the info with the achieved delays
            //   Compute the number of VMs allocated (both on this FP's FNs and on other FP's FNs) for every service this FP runs and update achieved delays stats
            for (std::size_t svc = 0; svc < num_svcs_; ++svc)
            {
                std::size_t svc_num_alloc_vms = 0;
                std::size_t svc_vm_cat = 0;
                for (std::size_t fn = 0; fn < num_fns_; ++fn)
                {
                    if (vm_alloc.fn_vm_allocations.at(fn).count(svc) > 0)
                    {
                        svc_vm_cat = vm_alloc.fn_vm_allocations.at(fn).at(svc).first; // NOTE: all VMs allocated to the same service (including those allocated on different FNs) belong to the same category.
                        svc_num_alloc_vms += vm_alloc.fn_vm_allocations.at(fn).at(svc).second;
                    }
                }

                // Sanity checks
                assert( svc_vm_cat_real_delays.size() > svc );
                assert( svc_vm_cat_real_delays[svc].size() > svc_vm_cat );
                assert( svc_vm_cat_real_delays[svc][svc_vm_cat].size() >= svc_num_alloc_vms );

                svc_interval_real_delays[svc] = svc_vm_cat_real_delays[svc][svc_vm_cat][svc_num_alloc_vms];
            }

            // - Update the power status of each FN and the stats with the number of powered-on FNs
            fp_interval_real_num_fns = 0;
            for (std::size_t i = 0; i < num_fns_; ++i)
            {
                DCS_DEBUG_ASSERT( rep_fn_power_states_[i] == vm_alloc.fn_power_states[i] ); // rep_fn_power_states_ has been set previously during VM allocation for predicted workload

                if (vm_alloc.fn_power_states[i])
                {
                    fp_interval_real_num_fns += 1;
                    //rep_fn_power_states_[i] = true; //XXX: already done in the optimization for the "predicted" workload
                }
                else
                {
                    //rep_fn_power_states_[i] = false; //XXX: already done in the optimization for the "predicted" workload
                }
            }
        }
        else
        {
            DCS_DEBUG_TRACE( "FP - Real workload - The VM assignment problem is infeasible" );
        }

#elif defined(DCS_FOG_VMALLOC_REAL_WORKLOAD_ALLOCATE_NONE)

        DCS_DEBUG_TRACE("Real Workload:");
        DCS_DEBUG_TRACE("- FN Power States (resulting from predicted workload): " << vm_alloc.fn_power_states);
        DCS_DEBUG_TRACE("- FN - VM Allocations (resulting from predicted workload): " << vm_alloc.fn_vm_allocations);
        DCS_DEBUG_TRACE("- Service Minimum Number of VMs by Service Category and VM Category: " << svc_vm_cat_real_min_num_vms);

        // Update stats starting from the one obtained for the predicted workload

        fp_interval_real_profits = fp_interval_pred_profits;
        fp_interval_real_num_fns = fp_interval_pred_num_fns;
        svc_interval_real_delays = svc_interval_pred_delays; //FIXME: actually not used

        // Find out what VM category and how many VMs of that category have been allocated for each service
        std::vector<std::pair<std::size_t,std::size_t>> svc_allocs(num_svcs_, std::make_pair(0,0));
        for (auto const& svc_map : vm_alloc.fn_vm_allocations)
        {
            for (auto const& svc_vms : svc_map)
            {
                auto svc = svc_vms.first;
                auto vm_cat = svc_vms.second.first;
                auto num_vms = svc_vms.second.second;

                if (svc_allocs[svc].second > 0)
                {
                    DCS_ASSERT( svc_allocs[svc].first == vm_cat,
                                DCS_EXCEPTION_THROW( std::logic_error,
                                                     "The VM allocated to a service must be of the same category" ) );

                    num_vms += svc_allocs[svc].second;
                }
                svc_allocs[svc] = std::make_pair(vm_cat, num_vms);
            }
        }

        // Update stats
        for (std::size_t svc = 0; svc < num_svcs_; ++svc)
        {
            auto svc_cat = svc_categories_[svc];
            auto alloc_vm_cat = svc_allocs[svc].first;
            auto alloc_num_vms = svc_allocs[svc].second;

DCS_DEBUG_TRACE("Compare #VMs required by predicted workload: " << svc_vm_cat_predicted_min_num_vms[svc][alloc_vm_cat] << " vs. #VMs allocated: " << alloc_num_vms);//XXX
            if (svc_vm_cat_predicted_min_num_vms[svc][alloc_vm_cat] <= alloc_num_vms)
            {
                // In the allocation for predicted workload all required VMs have been allocated
                // Now check what happens for the real workload. We have two options:
                // 1. The real workload requires more VMs than those allocated for the predicted workload -> add penalty costs
                // 2. The real workload requires less VMs than those allocated for the predicted workload -> subtract the revenues earned for the additional allocated VMs

DCS_DEBUG_TRACE("Compare #VMs required by real workload: " << svc_vm_cat_real_min_num_vms[svc][alloc_vm_cat] << " vs. #VMs allocated: " << alloc_num_vms);//XXX
                if (svc_vm_cat_real_min_num_vms[svc][alloc_vm_cat] > alloc_num_vms)
                {
                    // Less VMs have been allocated than required by the real workload -> add penalty costs
DCS_DEBUG_TRACE("REAL WORKLOAD - SVC: " << svc << " - Subtracting penalty: " << fp_svc_penalties_[svc_cat] << " from profit: " << fp_interval_real_profits);//XXX
                    fp_interval_real_profits -= fp_svc_penalties_[svc_cat];
                }
                else if (svc_vm_cat_real_min_num_vms[svc][alloc_vm_cat] < alloc_num_vms)
                {
                    // More VMs have been allocated than required by the real workload -> subtract revenues of useless VMs
DCS_DEBUG_TRACE("REAL WORKLOAD - SVC: " << svc << " - Subtracting revenue: " << ((alloc_num_vms-svc_vm_cat_real_min_num_vms[svc][alloc_vm_cat])*fp_svc_revenues_[svc_cat]) << " from profit: " << fp_interval_real_profits);//XXX
                    fp_interval_real_profits -= (alloc_num_vms-svc_vm_cat_real_min_num_vms[svc][alloc_vm_cat])*fp_svc_revenues_[svc_cat];
                }
            }
        }

        DCS_DEBUG_TRACE( "FP - Real workload => profit: " << fp_interval_real_profits << " (interval duration: " << vm_alloc_duration << ")");

#else

#error Allocation method for real workload is not defined

#endif // DCS_FOG_VMALLOC_REAL_WORKLOAD_ALLOCATE_...

        // Collect replication stats

        rep_fp_pred_profits_ += fp_interval_pred_profits;
        rep_fp_real_profits_ += fp_interval_real_profits;
        rep_fp_pred_num_fns_->collect(fp_interval_pred_num_fns);
        rep_fp_real_num_fns_->collect(fp_interval_real_num_fns);
        for (std::size_t svc = 0; svc < num_svcs_; ++svc)
        {
            rep_svc_pred_delays_[svc]->collect(svc_interval_pred_delays[svc]);
            rep_svc_real_delays_[svc]->collect(svc_interval_real_delays[svc]);
        }

        // Outputs some information

        if (verbosity_ >= medium)
        {
            DCS_LOGGING_STREAM << "-- INTERVAL OUTPUTS:" << std::endl;
            DCS_LOGGING_STREAM << "- Local Predicted Profits: " << fp_interval_pred_profits << std::endl;
            DCS_LOGGING_STREAM << "- Local Real Profits: " << fp_interval_real_profits << std::endl;
            DCS_LOGGING_STREAM << "- Local Predicted #FNs: " << fp_interval_pred_num_fns << std::endl;
            DCS_LOGGING_STREAM << "- Local Real #FNs: " << fp_interval_real_num_fns << std::endl;
            DCS_LOGGING_STREAM << "- Local Predicted Delays: [" << num_svcs_ << "]{";
            for (std::size_t svc = 0; svc < num_svcs_; ++svc)
            {
                if (svc > 0)
                {
                    DCS_LOGGING_STREAM << ",";
                }

                DCS_LOGGING_STREAM << svc_interval_pred_delays[svc];
            }
            DCS_LOGGING_STREAM << "}" << std::endl;
            DCS_LOGGING_STREAM << " - Local Real Delays: [" << num_svcs_ << "]{";
            for (std::size_t svc = 0; svc < num_svcs_; ++svc)
            {
                if (svc > 0)
                {
                    DCS_LOGGING_STREAM << ",";
                }

                DCS_LOGGING_STREAM << svc_interval_real_delays[svc];
            }
            DCS_LOGGING_STREAM << "}" << std::endl;

            if (verbosity_ >= high)
            {
                DCS_LOGGING_STREAM << "-- INCREMENTAL AVERAGED INTERVAL OUTPUTS:" << std::endl;
                DCS_LOGGING_STREAM << "- Incremental Local Predicted Profits: " << rep_fp_pred_profits_ << std::endl;
                DCS_LOGGING_STREAM << "- Incremental Local Real Profits: " << rep_fp_real_profits_ << std::endl;
                DCS_LOGGING_STREAM << "- Incremental Local Predicted #FNs: " << rep_fp_pred_num_fns_->estimate() << std::endl;
                DCS_LOGGING_STREAM << "- Incremental Local Real #FNs: " << rep_fp_real_num_fns_->estimate() << std::endl;
                DCS_LOGGING_STREAM << "- Incremental Local Predicted Delays: [" << num_svcs_ << "]{";
                for (std::size_t svc = 0; svc < num_svcs_; ++svc)
                {
                    if (svc > 0)
                    {
                        DCS_LOGGING_STREAM << ",";
                    }

                    DCS_LOGGING_STREAM << rep_svc_pred_delays_[svc]->estimate();
                }
                DCS_LOGGING_STREAM << "}" << std::endl;
                DCS_LOGGING_STREAM << "- Incremental Local Real Delays: [" << num_svcs_ << "]{";
                for (std::size_t svc = 0; svc < num_svcs_; ++svc)
                {
                    if (svc > 0)
                    {
                        DCS_LOGGING_STREAM << ",";
                    }

                    DCS_LOGGING_STREAM << rep_svc_real_delays_[svc]->estimate();
                }
                DCS_LOGGING_STREAM << "}" << std::endl;
            }
        }

        // Output to file
        if (stats_dat_ofs_.is_open())
        {
            stats_dat_ofs_  << cur_timestamp // Timestamp
                            << csv_field_sep_ch << csv_field_quote_ch << csv_field_interval_stats_tag << csv_field_quote_ch // Stats tag
                            << csv_field_sep_ch << this->num_replications() // Replication number
                            << csv_field_sep_ch << vm_alloc_start_time // VM allocation start time
                            << csv_field_sep_ch << vm_alloc_duration; // VM allocation duration
            // Output interval stats
            stats_dat_ofs_  << csv_field_sep_ch << fp_interval_pred_profits; // Local predicted profit
            stats_dat_ofs_  << csv_field_sep_ch << fp_interval_real_profits; // Local real profit
            for (std::size_t svc = 0; svc < num_svcs_; ++svc)
            {
                auto const svc_cat = svc_categories_[svc];
                stats_dat_ofs_  << csv_field_sep_ch << svc_interval_pred_delays[svc] // Local predicted service delay
                                << csv_field_sep_ch << relative_increment(svc_interval_pred_delays[svc], svc_max_delays_[svc_cat]); // Local predicted service delay vs. max delay
                stats_dat_ofs_  << csv_field_sep_ch << svc_interval_real_delays[svc] // Local real service delay
                                << csv_field_sep_ch << relative_increment(svc_interval_real_delays[svc], svc_max_delays_[svc_cat]); // Local real service delay vs. max delay
            }
            stats_dat_ofs_  << csv_field_sep_ch << fp_interval_pred_num_fns; // Local predicted #FNs
            stats_dat_ofs_  << csv_field_sep_ch << fp_interval_real_num_fns; // Local real #FNs
            // Output replication stats: since current replication is not done yet we output incremental stats
            // - Local VM allocation
            stats_dat_ofs_  << csv_field_sep_ch << rep_fp_pred_profits_; // Local predicted profit
            stats_dat_ofs_  << csv_field_sep_ch << rep_fp_real_profits_; // Local real profit
            for (std::size_t svc = 0; svc < num_svcs_; ++svc)
            {
                auto const svc_cat = svc_categories_[svc];
                stats_dat_ofs_  << csv_field_sep_ch << rep_svc_pred_delays_[svc]->estimate() // Local predicted service delay
                                << csv_field_sep_ch << relative_increment(rep_svc_pred_delays_[svc]->estimate(), svc_max_delays_[svc_cat]); // Local predicted service delay vs. max delay
                stats_dat_ofs_  << csv_field_sep_ch << rep_svc_real_delays_[svc]->estimate() // Local real service delay
                                << csv_field_sep_ch << relative_increment(rep_svc_real_delays_[svc]->estimate(), svc_max_delays_[svc_cat]); // Local real service delay vs. max delay
            }
            stats_dat_ofs_  << csv_field_sep_ch << rep_fp_pred_num_fns_->estimate(); // Local predicted #FNs
            stats_dat_ofs_  << csv_field_sep_ch << rep_fp_real_num_fns_->estimate(); // Local real #FNs
            // - Global VM allocation (not available at this point)
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_na_value; // Global predicted profit
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_na_value; // Global real profit
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_na_value; // Global predicted #FNs
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_na_value; // Global real #FNs
            // Output overall stats as NA since they are not available at interval granularity
            // - Local VM allocation
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_na_value // Local predicted profit (mean)
                            << csv_field_sep_ch << csv_field_na_value; // Local predicted profit (s.d.)
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_na_value // Local real profit (mean)
                            << csv_field_sep_ch << csv_field_na_value; // Local real profit (s.d.)
            for (std::size_t svc = 0; svc < num_svcs_; ++svc)
            {
                stats_dat_ofs_  << csv_field_sep_ch << csv_field_na_value // Local predicted service delay (mean)
                                << csv_field_sep_ch << csv_field_na_value // Local predicted service delay (s.d.)
                                << csv_field_sep_ch << csv_field_na_value; // Local predicted service delay vs. max delay
                stats_dat_ofs_  << csv_field_sep_ch << csv_field_na_value // Local real service delay (mean)
                                << csv_field_sep_ch << csv_field_na_value // Local real service delay (s.d.)
                                << csv_field_sep_ch << csv_field_na_value; // Local real service delay vs. max delay
            }
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_na_value // Local predicted #FNs (mean)
                            << csv_field_sep_ch << csv_field_na_value; // Local predicted #FNs (s.d.)
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_na_value // Local real #FNs (mean)
                            << csv_field_sep_ch << csv_field_na_value; // Local real #FNs (s.d.)
            // - Global VM allocation
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_na_value // Global predicted profit (mean)
                            << csv_field_sep_ch << csv_field_na_value; // Global predicted profit (s.d.)
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_na_value // Global real profit (mean)
                            << csv_field_sep_ch << csv_field_na_value; // Global real profit (s.d.)
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_na_value // Global predicted #FNs (mean)
                            << csv_field_sep_ch << csv_field_na_value; // Global predicted #FNs (s.d.)
            stats_dat_ofs_  << csv_field_sep_ch << csv_field_na_value // Global real #FNs (mean)
                            << csv_field_sep_ch << csv_field_na_value; // Global real #FNs (s.d.)
            stats_dat_ofs_ << std::endl;
        }
    }

    void global_allocate_vms()
    {
        auto num_time_slots = rep_global_svc_vm_cat_predicted_min_num_vms_.size();

        // Allocate VMs to the FP

        std::vector<std::size_t> fns;

        for (std::size_t fn = 0; fn < num_fns_; ++fn)
        {
            fns.push_back(fn);
        }

        multislot_vm_allocation_t<RealT> vm_alloc;

//        std::unique_ptr<base_multislot_vm_allocation_solver_t<RealT>> p_vm_alloc_solver;
//
//        p_vm_alloc_solver = std::make_unique<optimal_multislot_vm_allocation_solver_t<RealT>>(optim_relative_tolerance_, optim_time_limit_);

        // Compute VM allocation according to predicted workload

        vm_alloc = p_multislot_vm_alloc_solver_->solve(//fns,
                                            fn_categories_,
                                            initial_fn_power_states_,
                                            initial_fn_vm_allocations_,
                                            fn_min_powers_,
                                            fn_max_powers_,
                                            vm_cpu_requirements_,
                                            //vm_ram_requirements_,
                                            vm_cat_alloc_costs_,
                                            svc_categories_,
                                            rep_global_svc_vm_cat_predicted_min_num_vms_,
                                            fp_svc_revenues_,
                                            fp_svc_penalties_,
                                            fp_electricity_costs_,
                                            fp_fn_asleep_costs_,
                                            fp_fn_awake_costs_);

        if (vm_alloc.solved)
        {
#ifdef DCS_DEBUG
            DCS_DEBUG_TRACE( "--- MULTISLOT VM ALLOCATION SOLUTION (predicted workload) ---------------------[" );
            DCS_DEBUG_TRACE( "- Objective value: " << vm_alloc.objective_value << " (revenue: " << vm_alloc.revenue << ", cost: " << vm_alloc.cost << ")");
            DCS_DEBUG_TRACE( "- Solved: " << std::boolalpha << vm_alloc.solved << ", Optimal: " << std::boolalpha << vm_alloc.optimal << std::noboolalpha );

            for (std::size_t slot = 0; slot < vm_alloc.fn_vm_allocations.size(); ++slot)
            {
                DCS_DEBUG_TRACE( "**** SLOT " << slot);
                DCS_DEBUG_TRACE( "  - FN-VM allocations:" );
                for (std::size_t fn = 0; fn < vm_alloc.fn_vm_allocations[slot].size(); ++fn)
                {
                    DCS_DEBUG_STREAM << "  FN #" << fn << " -> {";
                    for (auto const& svc_vms : vm_alloc.fn_vm_allocations[slot][fn])
                    {
                        auto const svc = svc_vms.first;
                        auto const& vmcat_nvms = svc_vms.second;
                        DCS_DEBUG_STREAM << ", SVC #" << svc << " -> <VM Category: " << vmcat_nvms.first << ", #VMs: " << vmcat_nvms.second << ">";
                    }
                    DCS_DEBUG_STREAM << "}" << std::endl;
                }

                DCS_DEBUG_TRACE( "  - FN power status:" );
                for (std::size_t fn = 0; fn < vm_alloc.fn_power_states[slot].size(); ++fn)
                {
                    DCS_DEBUG_STREAM << "  FN #" << fn << " = " << std::boolalpha << vm_alloc.fn_power_states[slot][fn] << std::noboolalpha << std::endl;
                }

                DCS_DEBUG_TRACE( "  - FN CPU allocations:" );
                for (std::size_t fn = 0; fn < vm_alloc.fn_cpu_allocations[slot].size(); ++fn)
                {
                    DCS_DEBUG_STREAM << "  FN #" << fn << " = " << vm_alloc.fn_cpu_allocations[slot][fn] << std::endl;
                }
            }

            DCS_DEBUG_TRACE( "]-------------------------------------------------------------------------------" ); 
#endif // DCS_DEBUG 

            if (!check_vm_allocation_solution(vm_alloc))
            {
                DCS_EXCEPTION_THROW( std::runtime_error, "Returned VM allocation solution is not consistent" );
            }

            // - Compute the profit for the whole interval
            //auto const profit = vm_alloc.revenue-vm_alloc.cost;
            auto const profit = vm_alloc.profit;

            DCS_DEBUG_TRACE( "FP - Predicted workload - Global VM allocation objective value: " << vm_alloc.objective_value << " => profit: " << profit << " (revenue: " << vm_alloc.revenue << ", cost: " << vm_alloc.cost << ", interval duration: " << rep_global_vm_alloc_duration_ << ")");

            // Update state and stats

            // - Update achieved profit
            rep_global_fp_pred_profits_ = profit;

/*
            // - Update the info with the achieved delays
            //   Compute the number of VMs allocated (both on this FP's FNs and on other FP's FNs) for every service this FP runs and update achieved delays stats
            for (std::size_t t = 0; t < num_time_slots; ++t)
            {
                for (std::size_t svc = 0; svc < num_svcs_; ++svc)
                {
                    std::size_t svc_num_alloc_vms = 0;
                    std::size_t svc_vm_cat = 0;
                    for (std::size_t fn = 0; fn < num_fns_; ++fn)
                    {
                        if (vm_alloc.fn_vm_allocations[t].at(fn).count(svc) > 0)
                        {
                            svc_vm_cat = vm_alloc.fn_vm_allocations[t].at(fn).at(svc).first; // NOTE: all VMs allocated to the same service (including those allocated on different FNs) belong to the same category.
                            svc_num_alloc_vms += vm_alloc.fn_vm_allocations[t].at(fn).at(svc).second;
                        }
                    }

                    // Sanity checks
                    assert( rep_global_svc_vm_cat_predicted_delays_[t].size() > svc );
                    assert( rep_global_svc_vm_cat_predicted_delays_[t][svc].size() > svc_vm_cat );
                    assert( rep_global_svc_vm_cat_predicted_delays_[t][svc][svc_vm_cat].size() >= svc_num_alloc_vms );

                    rep_global_svc_pred_delays_[svc] = rep_global_svc_vm_cat_predicted_delays_[t][svc][svc_vm_cat][svc_num_alloc_vms];
                }
            }
*/

            // - Update the power status of each FN and the stats concerning the number of powered-on FNs
            for (std::size_t t = 0; t < num_time_slots; ++t)
            {
                std::size_t fp_interval_pred_num_fns = 0;
                for (std::size_t i = 0; i < fns.size(); ++i)
                {
                    if (vm_alloc.fn_power_states[t][i])
                    {
                        fp_interval_pred_num_fns += 1;
                    }
                }
                rep_global_fp_pred_num_fns_->collect(fp_interval_pred_num_fns);
            }
        }
        else
        {
            DCS_DEBUG_TRACE( "FP - Predicted workload - The global VM assignment problem is infeasible" );
        }

        // Compute VM allocation according to real workload

#if defined(DCS_FOG_VMALLOC_REAL_WORKLOAD_ALLOCATE_ALL)

        vm_alloc = p_multislot_vm_alloc_solver_->solve(fn_categories_,
                                            initial_fn_power_states_,
                                            initial_fn_vm_allocations_,
                                            fn_min_powers_,
                                            fn_max_powers_,
                                            vm_cpu_requirements_,
                                            //vm_ram_requirements_,
                                            vm_cat_alloc_costs_,
                                            svc_categories_,
                                            rep_global_svc_vm_cat_real_min_num_vms_,
                                            fp_svc_revenues_,
                                            fp_svc_penalties_,
                                            fp_electricity_costs_,
                                            fp_fn_asleep_costs_,
                                            fp_fn_awake_costs_);

        if (vm_alloc.solved)
        {
#ifdef DCS_DEBUG
            DCS_DEBUG_TRACE( "--- MULTISLOT VM ALLOCATION SOLUTION (predicted workload) ---------------------[" );
            DCS_DEBUG_TRACE( "- Objective value: " << vm_alloc.objective_value << " (revenue: " << vm_alloc.revenue << ", cost: " << vm_alloc.cost << ")");
            DCS_DEBUG_TRACE( "- Solved: " << std::boolalpha << vm_alloc.solved << ", Optimal: " << std::boolalpha << vm_alloc.optimal << std::noboolalpha );

            for (std::size_t slot = 0; slot < vm_alloc.fn_vm_allocations.size(); ++slot)
            {
                DCS_DEBUG_TRACE( "**** SLOT " << slot);
                DCS_DEBUG_TRACE( "  - FN-VM allocations:" );
                for (std::size_t fn = 0; fn < vm_alloc.fn_vm_allocations[slot].size(); ++fn)
                {
                    DCS_DEBUG_STREAM << "  FN #" << fn << " -> {";
                    for (auto const& svc_vms : vm_alloc.fn_vm_allocations[slot][fn])
                    {
                        auto const svc = svc_vms.first;
                        auto const& vmcat_nvms = svc_vms.second;
                        DCS_DEBUG_STREAM << ", SVC #" << svc << " -> <VM Category: " << vmcat_nvms.first << ", #VMs: " << vmcat_nvms.second << ">";
                    }
                    DCS_DEBUG_STREAM << "}" << std::endl;
                }

                DCS_DEBUG_TRACE( "  - FN power status:" );
                for (std::size_t fn = 0; fn < vm_alloc.fn_power_states[slot].size(); ++fn)
                {
                    DCS_DEBUG_STREAM << "  FN #" << fn << " = " << std::boolalpha << vm_alloc.fn_power_states[slot][fn] << std::noboolalpha << std::endl;
                }

                DCS_DEBUG_TRACE( "  - FN CPU allocations:" );
                for (std::size_t fn = 0; fn < vm_alloc.fn_cpu_allocations[slot].size(); ++fn)
                {
                    DCS_DEBUG_STREAM << "  FN #" << fn << " = " << vm_alloc.fn_cpu_allocations[slot][fn] << std::endl;
                }
            }

            DCS_DEBUG_TRACE( "]-------------------------------------------------------------------------------" ); 
#endif // DCS_DEBUG 

            if (!check_vm_allocation_solution(vm_alloc))
            {
                DCS_EXCEPTION_THROW( std::runtime_error, "Returned VM allocation solution is not consistent" );
            }

            // - Compute the profit for the whole interval
            //auto const profit = vm_alloc.revenue-vm_alloc.cost;
            auto const profit = vm_alloc.profit;

            DCS_DEBUG_TRACE( "FP - Real workload - Global VM allocation objective value: " << vm_alloc.objective_value << " => profit: " << profit << " (revenue: " << vm_alloc.revenue << ", cost: " << vm_alloc.cost << ", interval duration: " << rep_global_vm_alloc_duration_ << ")");

            // Update state and stats

            // - Update achieved profit
            rep_global_fp_real_profits_ = profit;

/*
            // - Update the info with the achieved delays
            //   Compute the number of VMs allocated (both on this FP's FNs and on other FP's FNs) for every service this FP runs and update achieved delays stats
            for (std::size_t t = 0; t < num_time_slots; ++t)
            {
                for (std::size_t svc = 0; svc < num_svcs_; ++svc)
                {
                    std::size_t svc_num_alloc_vms = 0;
                    std::size_t svc_vm_cat = 0;
                    for (std::size_t fn = 0; fn < num_fns_; ++fn)
                    {
                        if (vm_alloc.fn_vm_allocations[t].at(fn).count(svc) > 0)
                        {
                            svc_vm_cat = vm_alloc.fn_vm_allocations[t].at(fn).at(svc).first; // NOTE: all VMs allocated to the same service (including those allocated on different FNs) belong to the same category.
                            svc_num_alloc_vms += vm_alloc.fn_vm_allocations[t].at(fn).at(svc).second;
                        }
                    }

                    // Sanity checks
                    assert( rep_global_svc_vm_cat_real_delays_[t].size() > svc );
                    assert( rep_global_svc_vm_cat_real_delays_[t][svc].size() > svc_vm_cat );
                    assert( rep_global_svc_vm_cat_real_delays_[t][svc][svc_vm_cat].size() >= svc_num_alloc_vms );

                    rep_global_svc_real_delays_[svc] = rep_global_svc_vm_cat_real_delays_[t][svc][svc_vm_cat][svc_num_alloc_vms];
                }
            }
*/

            // - Update the power status of each FN and the stats concerning the number of powered-on FNs
            for (std::size_t t = 0; t < num_time_slots; ++t)
            {
                std::size_t fp_interval_real_num_fns = 0;
                for (std::size_t i = 0; i < fns.size(); ++i)
                {
                    if (vm_alloc.fn_power_states[t][i])
                    {
                        fp_interval_real_num_fns += 1;
                    }
                }
                rep_global_fp_real_num_fns_->collect(fp_interval_real_num_fns);
            }
        }
        else
        {
            DCS_DEBUG_TRACE( "FP - Real workload - The global VM assignment problem is infeasible" );
        }

#elif defined(DCS_FOG_VMALLOC_REAL_WORKLOAD_ALLOCATE_WITH_FIXED_FNS)

        std::vector<std::set<std::size_t>> fixed_fns(num_time_slots);
        for (std::size_t t = 0; t < num_time_slots; ++t)
        {
            for (std::size_t fn = 0; fn < num_fns_; ++fn)
            {
                if (vm_alloc.fn_power_states[t][fn])
                {
                    fixed_fns[t].insert(fn);
                }
            }
        }
        vm_alloc = p_multislot_vm_alloc_solver_->solve_with_fixed_fns(fixed_fns,
                                            fn_categories_,
                                            initial_fn_power_states_,
                                            initial_fn_vm_allocations_,
                                            fn_min_powers_,
                                            fn_max_powers_,
                                            vm_cpu_requirements_,
                                            //vm_ram_requirements_,
                                            vm_cat_alloc_costs_,
                                            svc_categories_,
                                            rep_global_svc_vm_cat_real_min_num_vms_,
                                            fp_svc_revenues_,
                                            fp_svc_penalties_,
                                            fp_electricity_costs_,
                                            fp_fn_asleep_costs_,
                                            fp_fn_awake_costs_);

        if (vm_alloc.solved)
        {
#ifdef DCS_DEBUG
            DCS_DEBUG_TRACE( "--- MULTISLOT VM ALLOCATION SOLUTION (predicted workload) ---------------------[" );
            DCS_DEBUG_TRACE( "- Objective value: " << vm_alloc.objective_value << " (revenue: " << vm_alloc.revenue << ", cost: " << vm_alloc.cost << ")");
            DCS_DEBUG_TRACE( "- Solved: " << std::boolalpha << vm_alloc.solved << ", Optimal: " << std::boolalpha << vm_alloc.optimal << std::noboolalpha );

            for (std::size_t slot = 0; slot < vm_alloc.fn_vm_allocations.size(); ++slot)
            {
                DCS_DEBUG_TRACE( "**** SLOT " << slot);
                DCS_DEBUG_TRACE( "  - FN-VM allocations:" );
                for (std::size_t fn = 0; fn < vm_alloc.fn_vm_allocations[slot].size(); ++fn)
                {
                    DCS_DEBUG_STREAM << "  FN #" << fn << " -> {";
                    for (auto const& svc_vms : vm_alloc.fn_vm_allocations[slot][fn])
                    {
                        auto const svc = svc_vms.first;
                        auto const& vmcat_nvms = svc_vms.second;
                        DCS_DEBUG_STREAM << ", SVC #" << svc << " -> <VM Category: " << vmcat_nvms.first << ", #VMs: " << vmcat_nvms.second << ">";
                    }
                    DCS_DEBUG_STREAM << "}" << std::endl;
                }

                DCS_DEBUG_TRACE( "  - FN power status:" );
                for (std::size_t fn = 0; fn < vm_alloc.fn_power_states[slot].size(); ++fn)
                {
                    DCS_DEBUG_STREAM << "  FN #" << fn << " = " << std::boolalpha << vm_alloc.fn_power_states[slot][fn] << std::noboolalpha << std::endl;
                }

                DCS_DEBUG_TRACE( "  - FN CPU allocations:" );
                for (std::size_t fn = 0; fn < vm_alloc.fn_cpu_allocations[slot].size(); ++fn)
                {
                    DCS_DEBUG_STREAM << "  FN #" << fn << " = " << vm_alloc.fn_cpu_allocations[slot][fn] << std::endl;
                }
            }

            DCS_DEBUG_TRACE( "]-------------------------------------------------------------------------------" ); 
#endif // DCS_DEBUG 

            if (!check_vm_allocation_solution(vm_alloc))
            {
                DCS_EXCEPTION_THROW( std::runtime_error, "Returned VM allocation solution is not consistent" );
            }

            // - Compute the profit for the whole interval
            //auto const profit = vm_alloc.revenue-vm_alloc.cost;
            auto const profit = vm_alloc.profit;

            DCS_DEBUG_TRACE( "FP - Real workload - Global VM allocation objective value: " << vm_alloc.objective_value << " => profit: " << profit << " (revenue: " << vm_alloc.revenue << ", cost: " << vm_alloc.cost << ", interval duration: " << rep_global_vm_alloc_duration_ << ")");

            // Update state and stats

            // - Update achieved profit
            rep_global_fp_real_profits_ = profit;

/*
            // - Update the info with the achieved delays
            //   Compute the number of VMs allocated (both on this FP's FNs and on other FP's FNs) for every service this FP runs and update achieved delays stats
            for (std::size_t t = 0; t < num_time_slots; ++t)
            {
                for (std::size_t svc = 0; svc < num_svcs_; ++svc)
                {
                    std::size_t svc_num_alloc_vms = 0;
                    std::size_t svc_vm_cat = 0;
                    for (std::size_t fn = 0; fn < num_fns_; ++fn)
                    {
                        if (vm_alloc.fn_vm_allocations[t].at(fn).count(svc) > 0)
                        {
                            svc_vm_cat = vm_alloc.fn_vm_allocations[t].at(fn).at(svc).first; // NOTE: all VMs allocated to the same service (including those allocated on different FNs) belong to the same category.
                            svc_num_alloc_vms += vm_alloc.fn_vm_allocations[t].at(fn).at(svc).second;
                        }
                    }

                    // Sanity checks
                    assert( rep_global_svc_vm_cat_real_delays_[t].size() > svc );
                    assert( rep_global_svc_vm_cat_real_delays_[t][svc].size() > svc_vm_cat );
                    assert( rep_global_svc_vm_cat_real_delays_[t][svc][svc_vm_cat].size() >= svc_num_alloc_vms );

                    rep_global_svc_real_delays_[svc] = rep_global_svc_vm_cat_real_delays_[t][svc][svc_vm_cat][svc_num_alloc_vms];
                }
            }
*/

            // - Update the power status of each FN and the stats concerning the number of powered-on FNs
            for (std::size_t t = 0; t < num_time_slots; ++t)
            {
                std::size_t fp_interval_real_num_fns = 0;
                for (std::size_t i = 0; i < fns.size(); ++i)
                {
                    if (vm_alloc.fn_power_states[t][i])
                    {
                        fp_interval_real_num_fns += 1;
                    }
                }
                rep_global_fp_real_num_fns_->collect(fp_interval_real_num_fns);
            }
        }
        else
        {
            DCS_DEBUG_TRACE( "FP - Real workload - The global VM assignment problem is infeasible" );
        }

#elif defined(DCS_FOG_VMALLOC_REAL_WORKLOAD_ALLOCATE_NONE)

        DCS_DEBUG_TRACE("Real Workload:");
        DCS_DEBUG_TRACE("- FN Power States (resulting from predicted workload): " << vm_alloc.fn_power_states);
        DCS_DEBUG_TRACE("- FN - VM Allocations (resulting from predicted workload): " << vm_alloc.fn_vm_allocations);
        DCS_DEBUG_TRACE("- Service Minimum Number of VMs by Service Category and VM Category: " << rep_global_svc_vm_cat_real_min_num_vms_);

        // Update stats starting from the one obtained for the predicted workload

        rep_global_fp_real_profits_ = rep_global_fp_pred_profits_;
        rep_global_fp_real_num_fns_ = rep_global_fp_pred_num_fns_;
        //rep_global_svc_real_delays = rep_global_svc_pred_delays; //FIXME: actually not used

        for (std::size_t t = 0; t < num_time_slots; ++t)
        {
            // Find out what VM category and how many VMs of that category have been allocated for each service
            std::vector<std::pair<std::size_t,std::size_t>> svc_allocs(num_svcs_, std::make_pair(0,0));
            for (auto const& svc_map : vm_alloc.fn_vm_allocations[t])
            {
                for (auto const& svc_vms : svc_map)
                {
                    auto svc = svc_vms.first;
                    auto vm_cat = svc_vms.second.first;
                    auto num_vms = svc_vms.second.second;

                    if (svc_allocs[svc].second > 0)
                    {
                        DCS_ASSERT( svc_allocs[svc].first == vm_cat,
                                    DCS_EXCEPTION_THROW( std::logic_error,
                                                         "The VM allocated to a service must be of the same category" ) );

                        num_vms += svc_allocs[svc].second;
                    }
                    svc_allocs[svc] = std::make_pair(vm_cat, num_vms);
                }
            }

            // Update stats
            for (std::size_t svc = 0; svc < num_svcs_; ++svc)
            {
                auto svc_cat = svc_categories_[svc];
                auto alloc_vm_cat = svc_allocs[svc].first;
                auto alloc_num_vms = svc_allocs[svc].second;

DCS_DEBUG_TRACE("Time slot #" << (t+1) << " - Compare #VMs required by predicted workload: " << rep_global_svc_vm_cat_predicted_min_num_vms_[t][svc][alloc_vm_cat] << " vs. #VMs allocated: " << alloc_num_vms);//XXX
                if (rep_global_svc_vm_cat_predicted_min_num_vms_[t][svc][alloc_vm_cat] <= alloc_num_vms)
                {
                    // In the allocation for predicted workload all required VMs have been allocated
                    // Now check what happens for the real workload. We have two options:
                    // 1. The real workload requires more VMs than those allocated for the predicted workload -> add penalty costs
                    // 2. The real workload requires less VMs than those allocated for the predicted workload -> subtract the revenues earned for the additional allocated VMs

DCS_DEBUG_TRACE("Time slot #" << (t+1) << " - Compare #VMs required by real workload: " << rep_global_svc_vm_cat_real_min_num_vms_[t][svc][alloc_vm_cat] << " vs. #VMs allocated: " << alloc_num_vms);//XXX
                    if (rep_global_svc_vm_cat_real_min_num_vms_[t][svc][alloc_vm_cat] > alloc_num_vms)
                    {
                        // Less VMs have been allocated than required by the real workload -> add penalty costs
    DCS_DEBUG_TRACE("Time slot #" << (t+1) << " - REAL WORKLOAD - SVC: " << svc << " - Subtracting penalty: " << fp_svc_penalties_[svc_cat] << " from profit: " << rep_global_fp_real_profits_);//XXX
                        rep_global_fp_real_profits_ -= fp_svc_penalties_[svc_cat];
                    }
                    else if (rep_global_svc_vm_cat_real_min_num_vms_[t][svc][alloc_vm_cat] < alloc_num_vms)
                    {
                        // More VMs have been allocated than required by the real workload -> subtract revenues of useless VMs
    DCS_DEBUG_TRACE("Time slot #" << (t+1) << " - REAL WORKLOAD - SVC: " << svc << " - Subtracting revenue: " << ((alloc_num_vms-rep_global_svc_vm_cat_real_min_num_vms_[t][svc][alloc_vm_cat])*fp_svc_revenues_[svc_cat]) << " from profit: " << rep_global_fp_real_profits_);//XXX
                        rep_global_fp_real_profits_ -= (alloc_num_vms-rep_global_svc_vm_cat_real_min_num_vms_[t][svc][alloc_vm_cat])*fp_svc_revenues_[svc_cat];
                    }
                }
            }
        }

        DCS_DEBUG_TRACE( "FP - Real workload => profit: " << rep_global_fp_real_profits_ << " (interval duration: " << rep_global_vm_alloc_duration_ << ")");

#else

#error Allocation method for real workload is not defined

#endif // DCS_FOG_VMALLOC_REAL_WORKLOAD_ALLOCATE_...
    }


private:
    std::size_t num_fn_categories_; ///< Number of fog node (FN) categories
    std::size_t num_svc_categories_; ///< Number of service categories
    std::size_t num_vm_categories_; ///< Number of virtual machine (VM) categories
    std::vector<RealT> svc_arr_rates_; ///< Single-user request arrival rates for services, by service category
    std::vector<RealT> svc_max_arr_rates_; ///< Max aggregate (i.e., multiple-users) request arrival rates for services, by service category
    std::vector<RealT> svc_max_delays_; ///< Max delays tolerated by services, by service category
    //std::vector<std::size_t> svc_vm_categories_; ///< Category of the VMs associated with each service, by service category (VMs of the same service belong to the same category).
    std::vector<std::vector<RealT>> svc_vm_service_rates_; ///< Service rate of every VM associated with each service, by service category and VM category
    std::vector<std::size_t> fp_num_svcs_; ///< Number of services, by FP and per service category
    std::vector<std::size_t> fp_num_fns_; ///< Number of FNs, by FP and per FN category
    RealT fp_electricity_costs_; ///< FP electricity cost plans (in $/kWh)
    std::vector<RealT> fp_svc_revenues_; ///< FP revenues (in $/service) for running services, by FP and service category
    std::vector<RealT> fp_svc_penalties_; ///< FP penalties (in $/service) for violating service QoS (i.e., the max delay), by FP and service category
    std::vector<RealT> fp_fn_asleep_costs_; ///< FP costs for powering off a powered-on FN, by FP and FN category
    std::vector<RealT> fp_fn_awake_costs_; ///< FP costs for powering on a powered-off FN, by FP and FN category
    std::vector<RealT> fn_min_powers_; ///< FN min power consumptions (in kW), by FN category
    std::vector<RealT> fn_max_powers_; ///< FN max power consumptions (in kW), by FN category
    std::vector<std::vector<RealT>> vm_cpu_requirements_; ///< VM CPU requirements, by VM category and FN category
    std::vector<std::vector<RealT>> vm_ram_requirements_; ///< VM RAM requirements, by VM category and FN category
    std::vector<RealT> vm_cat_alloc_costs_; ///< VM allocation costs, by VM category
    RealT optim_relative_tolerance_; ///< The relative tolerance option to set in the optimizer
    RealT optim_time_limit_; ///< The time limit option to set in the optimizer
    std::string output_stats_data_file_; ///< The path to the output stats data file
    std::string output_trace_data_file_; ///< The path to the output trace data file
    RealT ci_level_; ///< Confidence level for confidence interval estimators
    RealT ci_rel_precision_; ///< Relative precision of the half-width of the confidence intervals used for stopping the simulation
    RealT service_delay_tolerance_; ///< The relative tolerance to set in the service performance model
    int verbosity_; ///< The verbosity level: 0 for 'minimum' and 9 for 'maximum' verbosity level
    arrival_rate_estimation_t svc_arr_rate_estimation_; ///< The way service arrival rates are estimated
    //RealT svc_arr_rate_estimation_perturb_max_sd_; ///< The standard deviation to use in the perturbed max arrival rate estimation
    std::vector<RealT> svc_arr_rate_estimation_params_; ///< The parameters to pass to the specified arrival rate estimation approach
    //fp_revenue_policy_t fp_revenue_policy_; ///< The revenue policy for FPs
    //fp_penalty_policy_t fp_penalty_policy_; ///< The penalty policy for FPs
    //fp_penalty_model_t fp_penalty_model_; ///< The penalty model for FPs
    RealT fp_vm_allocation_interval_; ///< The activating time of the VM allocation algorithm
    //vm_allocation_policy_t fp_vm_alloc_policy_; ///< The policy to follow for allocating VMs
    random_number_engine_t rng_;
    std::size_t num_fns_; ///< Total number of FNs
    std::size_t num_svcs_; ///< Total number of services
    std::vector<std::size_t> fn_categories_; ///< Map an FN to its category
    std::vector<std::size_t> svc_categories_; ///< Map a service to its category
    std::vector<std::shared_ptr<arrival_rate_estimator_t<RealT>>> svc_arr_rate_estimators_; ///< Arrival rate estimators, by service
    std::vector<bool> initial_fn_power_states_; ///< The initial FN power status to use at the beginning of a replication (and in the global VM allocation), by FN
    std::vector<std::map<std::size_t, std::pair<std::size_t, std::size_t>>> initial_fn_vm_allocations_; ///< The initial VMs allocation to FNs to use at the beginning of a replication (and in the global VM allocation), by FN and service
    std::ofstream stats_dat_ofs_;
    std::ofstream trace_dat_ofs_;
    // BEGIN of members related to local VM allocation
    RealT rep_fp_pred_profits_; ///< FP predicted profits in a single replication
    RealT rep_fp_real_profits_; ///< FP real profits in a single replication, by FP
    std::shared_ptr<mean_estimator_t<RealT>> rep_fp_pred_num_fns_; ///< FP predicted number of powered-on FNs in a single replication
    std::shared_ptr<mean_estimator_t<RealT>> rep_fp_real_num_fns_; ///< FP real number of powered-on FNs in a single replication
    std::vector<std::shared_ptr<mean_estimator_t<RealT>>> rep_svc_pred_delays_; ///< Service predicted delays in a single replication, by service
    std::vector<std::shared_ptr<mean_estimator_t<RealT>>> rep_svc_real_delays_; ///< Service real delays in a single replication, by service
    std::vector<bool> rep_fn_power_states_; ///< The FN power status along the replication (it is updated every time the local VM allocation problem is solved), by FN
    std::vector<std::map<std::size_t, std::pair<std::size_t, std::size_t>>> rep_fn_vm_allocations_; ///< The VM allocations along the replication (it is updated every time the local VM allocation problem is solved), by FN and service
    std::shared_ptr<ci_mean_estimator_t<RealT>> fp_pred_profit_ci_stats_; // FP predicted profits spanning the whole simulation
    std::shared_ptr<ci_mean_estimator_t<RealT>> fp_real_profit_ci_stats_; // FP real profits spanning the whole simulation
    std::shared_ptr<ci_mean_estimator_t<RealT>> fp_pred_num_fns_ci_stats_; // FP predicted number of powered-on FNs
    std::shared_ptr<ci_mean_estimator_t<RealT>> fp_real_num_fns_ci_stats_; // FP real number of powered-on FNs
    std::vector<std::shared_ptr<ci_mean_estimator_t<RealT>>> svc_pred_delay_ci_stats_; // Service predicted delays spanning the whole simulation, by service
    std::vector<std::shared_ptr<ci_mean_estimator_t<RealT>>> svc_real_delay_ci_stats_; // Service real delays spanning the whole simulation, by service
    // END of members related to local VM allocation
    // BEGIN of members related to global VM allocation
    RealT rep_global_vm_alloc_duration_; ///< The sum of all VM allocation interval (this usually is the same as the replication duration)
    std::size_t rep_global_vm_alloc_interval_num_; ///< Progress number (starting from zero) denoting the current VM allocation interval
    std::vector<std::vector<RealT>> rep_global_svc_predicted_arr_rates_; ///< Predicted arrival rates as seen during a replication, by interval and service
    std::vector<std::vector<RealT>> rep_global_svc_real_arr_rates_; ///< Real arrival rates as seen during a replication, by interval and service
    std::vector<std::vector<std::vector<std::size_t>>> rep_global_svc_vm_cat_predicted_min_num_vms_; ///< Min number of VMs for predicted arrival rates along the whole replication, by interval, service and VM category
    std::vector<std::vector<std::vector<std::size_t>>> rep_global_svc_vm_cat_real_min_num_vms_; ///< Min number of VMs for real arrival rates along the whole replication, by interval, service and VM category
    RealT rep_global_fp_pred_profits_; ///< FP predicted profits for the global VM allocation in a single replication
    std::shared_ptr<mean_estimator_t<RealT>> rep_global_fp_pred_num_fns_; ///< FP predicted number of powered-on FNs in a single replication
    std::shared_ptr<ci_mean_estimator_t<RealT>> global_fp_pred_profit_ci_stats_; // FP predicted profits for the global VM allocation, spanning the whole simulation
    std::shared_ptr<ci_mean_estimator_t<RealT>> global_fp_pred_num_fns_ci_stats_; // FP predicted number of powered-on FNs
    RealT rep_global_fp_real_profits_; ///< FP real profits for the global VM allocation in a single replication
    std::shared_ptr<mean_estimator_t<RealT>> rep_global_fp_real_num_fns_; ///< FP real number of powered-on FNs in a single replication
    std::shared_ptr<ci_mean_estimator_t<RealT>> global_fp_real_profit_ci_stats_; // FP real profits for the global VM allocation, spanning the whole simulation
    std::shared_ptr<ci_mean_estimator_t<RealT>> global_fp_real_num_fns_ci_stats_; // FP real number of powered-on FNs
    // END of members related to global VM allocation
//#if 1 //FIXME: enable the code fragment below to use std:array instead of plain char**
//    std::array<char*,rwp_num_args> rwp_args_; /// Arguments to pass to the Python script for the mobility model
//#else
//    char** rwp_args_; /// Arguments to pass to the Python script for the mobility model
//#endif
//    RndWalkPnt rwp_; ///< Interface to the Python script for the mobility model
    std::shared_ptr<user_mobility_model_t> p_mob_model_; ///< The user mobility model
    mmc_service_performance_model_t<RealT> svc_perf_model_;
    std::shared_ptr<base_vm_allocation_solver_t<RealT>> p_vm_alloc_solver_;
    std::shared_ptr<base_multislot_vm_allocation_solver_t<RealT>> p_multislot_vm_alloc_solver_;
}; // experiment_t


template <typename CharT, typename CharTraitsT, typename RealT>
std::basic_ostream<CharT,CharTraitsT>& operator<<(std::basic_ostream<CharT,CharTraitsT>& os, const experiment_t<RealT>& exp)
{
    os  << "num_fn_categories: " << exp.num_fog_node_categories()
        << ", " << "num_svc_categories: " << exp.num_service_categories()
        << ", " << "num_vm_categories: " << exp.num_virtual_machine_categories();

    os << ", " << "svc_arrival_rates: [";
    {
        auto svc_arrival_rates = exp.service_arrival_rates();
        for (std::size_t i = 0; i < svc_arrival_rates.size(); ++i)
        {
            if (i > 0)
            {
                os << ", ";
            }
            os << svc_arrival_rates[i];
        }
    }
    os << "]";
    os << ", " << "svc_max_arrival_rates: [";
    {
        auto svc_max_arrival_rates = exp.max_service_arrival_rates();
        for (std::size_t i = 0; i < svc_max_arrival_rates.size(); ++i)
        {
            if (i > 0)
            {
                os << ", ";
            }
            os << svc_max_arrival_rates[i];
        }
    }
    os << "]";
    os << ", " << "svc_max_delays: [";
    {
        auto svc_max_delays = exp.max_service_delays();
        for (std::size_t i = 0; i < svc_max_delays.size(); ++i)
        {
            if (i > 0)
            {
                os << ", ";
            }
            os << svc_max_delays[i];
        }
    }
    os << "]";
//    os << ", " << "svc_vm_categories: [";
//    {
//        auto svc_vm_categories = exp.virtual_machine_categories();
//        for (std::size_t i = 0; i < svc_vm_categories.size(); ++i)
//        {
//            if (i > 0)
//            {
//                os << ", ";
//            }
//            os << svc_vm_categories[i];
//        }
//    }
//    os << "]";
    os << ", " << "svc_vm_service_rates: [";
    {
        auto svc_vm_service_rates = exp.virtual_machine_service_rates();
        for (std::size_t i = 0; i < svc_vm_service_rates.size(); ++i)
        {
            os << "[";
            for (std::size_t j = 0; j < svc_vm_service_rates[i].size(); ++j)
            {
                    if (j > 0)
                    {
                        os << ", ";
                    }
                    os << svc_vm_service_rates[i][j];
            }
            os << "]";
        }
    }
    os << "]";
    os << ", " << "fp_num_svcs: [";
    {
        auto fp_num_svcs = exp.num_services();
        for (std::size_t i = 0; i < fp_num_svcs.size(); ++i)
        {
            if (i > 0)
            {
                os << ", ";
            }
            os << fp_num_svcs[i];
        }
    }
    os << "]";
    os << ", " << "fp_num_fns: [";
    {
        auto fp_num_fns = exp.num_fog_nodes();
        for (std::size_t i = 0; i < fp_num_fns.size(); ++i)
        {
            if (i > 0)
            {
                os << ", ";
            }
            os << fp_num_fns[i];
        }
    }
    os << "]";
    os << ", " << "fp_electricity_costs: " << exp.electricity_costs();
    os << ", " << "fp_fn_asleep_costs: [";
    {
        auto fp_fn_asleep_costs = exp.fog_node_asleep_costs();
        for (std::size_t i = 0; i < fp_fn_asleep_costs.size(); ++i)
        {
            if (i > 0)
            {   
                os << ", ";
            }
            os << fp_fn_asleep_costs[i];
        }
    }
    os << "]";
    os << ", " << "fp_fn_awake_costs: [";
    {
        auto fp_fn_awake_costs = exp.fog_node_awake_costs();
        for (std::size_t i = 0; i < fp_fn_awake_costs.size(); ++i)
        {
            if (i > 0)
            {   
                os << ", ";
            }
            os << fp_fn_awake_costs[i];
        }
    }
    os << "]";
    os << ", " << "fp_svc_revenues: [";
    {
        auto fp_svc_revenues = exp.service_revenues();
        for (std::size_t i = 0; i < fp_svc_revenues.size(); ++i)
        {
            if (i > 0)
            {
                os << ", ";
            }
            os << fp_svc_revenues[i];
        }
    }
    os << "]";
    os << ", " << "fp_svc_penalties: [";
    {
        auto fp_svc_penalties = exp.service_penalties();
        for (std::size_t i = 0; i < fp_svc_penalties.size(); ++i)
        {
            if (i > 0)
            {
                os << ", ";
            }
            os << fp_svc_penalties[i];
        }
    }
    os << "]";
    os << ", " << "fn_min_powers: [";
    {
        auto fn_min_powers = exp.fog_node_min_power_consumptions();
        for (std::size_t i = 0; i < fn_min_powers.size(); ++i)
        {
            if (i > 0)
            {
                os << ", ";
            }
            os << fn_min_powers[i];
        }
    }
    os << "]";
    os << ", " << "fn_max_powers: [";
    {
        auto fn_max_powers = exp.fog_node_max_power_consumptions();
        for (std::size_t i = 0; i < fn_max_powers.size(); ++i)
        {
            if (i > 0)
            {
                os << ", ";
            }
            os << fn_max_powers[i];
        }
    }
    os << "]";
    os << ", " << "vm_cpu_requirements: [";
    {
        auto vm_cpu_requirements = exp.virtual_machine_cpu_requirements();
        for (std::size_t i = 0; i < vm_cpu_requirements.size(); ++i)
        {
            if (i > 0)
            {
                os << " ";
            }

            os << "[";
            for (std::size_t j = 0; j < vm_cpu_requirements[i].size(); ++j)
            {
                if (j > 0)
                {
                    os << ", ";
                }
                os << vm_cpu_requirements[i][j];
            }
            os << "]";
        }
    }
    os << "]";
    os << ", " << "vm_ram_requirements: [";
    {
        auto vm_ram_requirements = exp.virtual_machine_ram_requirements();
        for (std::size_t i = 0; i < vm_ram_requirements.size(); ++i)
        {
            if (i > 0)
            {
                os << " ";
            }

            os << "[";
            for (std::size_t j = 0; j < vm_ram_requirements[i].size(); ++j)
            {
                if (j > 0)
                {
                    os << ", ";
                }
                os << vm_ram_requirements[i][j];
            }
            os << "]";
        }
    }
    os << "]";
    os << ", " << "vm_allocation_costs: [";
    {
        auto vm_allocation_costs = exp.virtual_machine_allocation_costs();
        for (std::size_t i = 0; i < vm_allocation_costs.size(); ++i)
        {
            if (i > 0)
            {
                os << ", ";
            }
            os << vm_allocation_costs[i];
        }
    }
    os << "]";
    os << ", " << "vm-allocation-trigger-interval: " << exp.fp_vm_allocation_trigger_interval();
    //os << ", " << "fp-penalty-model: " << exp.fp_penalty_model();
    //os << ", " << "fp-penalty-policy: " << exp.fp_penalty_policy();
    //os << ", " << "fp-revenue-policy: " << exp.fp_revenue_policy();
    //os << ", " << "fp-vm-allocation-policy: " << exp.fp_vm_allocation_policy();
    os << ", " << "optimization-relative-tolerance: " << exp.optimization_relative_tolerance();
    os << ", " << "optimization-max-duration: " << exp.optimization_max_duration();
    os << ", " << "output-stats-data-file: " << exp.output_stats_data_file();
    os << ", " << "output-trace-data-file: " << exp.output_trace_data_file();
    os << ", " << "sim-confidence-interval-level: " << exp.confidence_interval_level();
    os << ", " << "sim-confidence-interval-relative-precision: " << exp.confidence_interval_relative_precision();
    os << ", " << "sim-max-num-replications: " << exp.max_num_replications();
    os << ", " << "sim-max-replication-duration: " << exp.max_replication_duration();
    os << ", " << "service-delay-tolerance: " << exp.service_delay_tolerance();
    os << ", " << "service-arrival-rate-estimation: " << exp.service_arrival_rate_estimation();
    //os << ", " << "service-arrival-rate-estimation-perturb-max-stdev: " << exp.service_arrival_rate_estimation_perturbed_max_stdev();
    os << ", " << "service-arrival-rate-estimation-params: " << exp.service_arrival_rate_estimation_params();
    os << ", " << "verbosity: " << exp.verbosity_level();

    return os;
}

}} // Namespace dcs::fog

#endif // DCS_FOG_EXPERIMENT_HPP
