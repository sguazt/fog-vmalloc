/* vim: set tabstop=4 expandtab shiftwidth=4 softtabstop=4: */

/**
 * \file dcs/fog/vm_allocation/commons.hpp
 *
 * \brief Solution to the VMs allocation problem.
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

#ifndef DCS_FOG_VM_ALLOCATION_COMMONS_HPP
#define DCS_FOG_VM_ALLOCATION_COMMONS_HPP


#include <cstddef>
#include <limits>
#include <map>
#include <vector>
#include <utility>


namespace dcs { namespace fog {

template <typename RealT>
struct vm_allocation_t
{
	vm_allocation_t()
	: solved(false),
	  optimal(false),
	  objective_value(std::numeric_limits<RealT>::quiet_NaN()),
	  revenue(std::numeric_limits<RealT>::quiet_NaN()),
	  cost(std::numeric_limits<RealT>::quiet_NaN())
	{
	}


	bool solved;
	bool optimal;
	RealT objective_value;
	RealT profit;
	RealT revenue;
	RealT cost;
	std::vector<std::map<std::size_t,std::pair<std::size_t,std::size_t>>> fn_vm_allocations; // For each FN, there is a collection of <service => <VM category,number>> mappings
	std::vector<bool> fn_power_states;
    std::vector<RealT> fn_cpu_allocations;
    //std::vector<RealT> svc_achieved_delays;
}; // vm_allocation_t


template <typename RealT>
struct multislot_vm_allocation_t
{
	multislot_vm_allocation_t()
	: solved(false),
	  optimal(false),
	  objective_value(std::numeric_limits<RealT>::quiet_NaN()),
	  revenue(std::numeric_limits<RealT>::quiet_NaN()),
	  cost(std::numeric_limits<RealT>::quiet_NaN())
	{
	}


	bool solved;
	bool optimal;
	RealT objective_value;
	RealT profit;
	RealT revenue;
	RealT cost;
	std::vector<std::vector<std::map<std::size_t,std::pair<std::size_t,std::size_t>>>> fn_vm_allocations; // For each time slot and FN, there is a collection of <service => <VM category,number>> mappings
	std::vector<std::vector<bool>> fn_power_states; // For each time slot and FN, tells whether a given FN is to be powered on or not
    std::vector<std::vector<RealT>> fn_cpu_allocations; // For each time slot and FN, gives the amount of used CPU for a given FN
    //std::vector<RealT> svc_achieved_delays;
}; // multislot_vm_allocation_t


template <typename RealT>
struct base_vm_allocation_solver_t
{
    virtual vm_allocation_t<RealT> solve(//const std::vector<std::size_t>& fns, // Holds the identity of FNs in FN' (i.e., fns[i]=k -> FN k \in FN')
                                         const std::vector<std::size_t>& fn_categories, // Maps every FN to its FN category
                                         const std::vector<bool>& fn_power_states, // The power status of each FN
                                         const std::vector<std::map<std::size_t,std::pair<std::size_t,std::size_t>>>& fn_vm_allocations, // Current VM allocations, by FN and service
                                         const std::vector<RealT>& fn_cat_min_powers, // The min power consumption of FNs by FN category
                                         const std::vector<RealT>& fn_cat_max_powers, // The max power consumption of FNs by FN category
                                         const std::vector<std::vector<RealT>>& vm_cat_fn_cat_cpu_specs, // The CPU requirement of VMs by VM category and FN category
                                         //const std::vector<std::vector<RealT>>& vm_cat_fn_cat_ram_specs, // The RAM requirement of VMs by VM category and FN category
                                         const std::vector<RealT>& vm_cat_alloc_costs, // // The cost to allocate a VM on a FN (e.g., cost to boot a VM or to live-migrate its state), by VM category
                                         const std::vector<std::size_t>& svc_categories, // Service categories by service
                                         const std::vector<std::vector<std::size_t>>& svc_cat_vm_cat_min_num_vms, // The min number of VMs required to achieve QoS, by service category and VM category
                                         const std::vector<RealT>& fp_svc_cat_revenues, // Monetary revenues by service
                                         const std::vector<RealT>& fp_svc_cat_penalties, // Monetary penalties by service
                                         const RealT fp_electricity_cost, // Electricty cost (in $/Wh) of FP
                                         const std::vector<RealT>& fp_fn_cat_asleep_costs, // Cost to power-off a FN by FN category
                                         const std::vector<RealT>& fp_fn_cat_awake_costs, // Cost to power-on a FN by FN category
                                         RealT deltat = 1 // Length of the time interval
                                    ) const = 0;

    virtual vm_allocation_t<RealT> solve_with_fixed_fns(const std::set<std::size_t>& fixed_fns,
                                         const std::vector<std::size_t>& fn_categories, // Maps every FN to its FN category
                                         const std::vector<bool>& fn_power_states, // The power status of each FN
                                         const std::vector<std::map<std::size_t,std::pair<std::size_t,std::size_t>>>& fn_vm_allocations, // Current VM allocations, by FN and service
                                         const std::vector<RealT>& fn_cat_min_powers, // The min power consumption of FNs by FN category
                                         const std::vector<RealT>& fn_cat_max_powers, // The max power consumption of FNs by FN category
                                         const std::vector<std::vector<RealT>>& vm_cat_fn_cat_cpu_specs, // The CPU requirement of VMs by VM category and FN category
                                         //const std::vector<std::vector<RealT>>& vm_cat_fn_cat_ram_specs, // The RAM requirement of VMs by VM category and FN category
                                         const std::vector<RealT>& vm_cat_alloc_costs, // // The cost to allocate a VM on a FN (e.g., cost to boot a VM or to live-migrate its state), by VM category
                                         const std::vector<std::size_t>& svc_categories, // Service categories by service
                                         const std::vector<std::vector<std::size_t>>& svc_cat_vm_cat_min_num_vms, // The min number of VMs required to achieve QoS, by service category and VM category
                                         const std::vector<RealT>& fp_svc_cat_revenues, // Monetary revenues by service
                                         const std::vector<RealT>& fp_svc_cat_penalties, // Monetary penalties by service
                                         const RealT fp_electricity_cost, // Electricty cost (in $/Wh) of FP
                                         const std::vector<RealT>& fp_fn_cat_asleep_costs, // Cost to power-off a FN by FN category
                                         const std::vector<RealT>& fp_fn_cat_awake_costs, // Cost to power-on a FN by FN category
                                         RealT deltat = 1 // Length of the time interval
                                    ) const = 0;
}; // base_vm_allocation_solver_t


template <typename RealT>
struct base_multislot_vm_allocation_solver_t
{
    virtual multislot_vm_allocation_t<RealT> solve(//const std::vector<std::size_t>& fns, // Holds the identity of FNs in FN' (i.e., fns[i]=k -> FN k \in FN')
                                                   const std::vector<std::size_t>& fn_categories, // Maps every FN to its FN category
                                                   const std::vector<bool>& fn_power_states, // The power status of each FN
                                                   const std::vector<std::map<std::size_t,std::pair<std::size_t,std::size_t>>>& fn_vm_allocations, // Current VM allocations, by FN and service
                                                   const std::vector<RealT>& fn_cat_min_powers, // The min power consumption of FNs by FN category
                                                   const std::vector<RealT>& fn_cat_max_powers, // The max power consumption of FNs by FN category
                                                   const std::vector<std::vector<RealT>>& vm_cat_fn_cat_cpu_specs, // The CPU requirement of VMs by VM category and FN category
                                                   //const std::vector<std::vector<RealT>>& vm_cat_fn_cat_ram_specs, // The RAM requirement of VMs by VM category and FN category
                                                   const std::vector<RealT>& vm_cat_alloc_costs, // // The cost to allocate a VM on a FN (e.g., cost to boot a VM or to live-migrate its state), by VM category
                                                   const std::vector<std::size_t>& svc_categories, // Service categories by service
                                                   const std::vector<std::vector<std::vector<std::size_t>>>& svc_cat_vm_cat_min_num_vms, // The min number of VMs required to achieve QoS, by time slot, service category and VM category
                                                   const std::vector<RealT>& fp_svc_cat_revenues, // Monetary revenues by service
                                                   const std::vector<RealT>& fp_svc_cat_penalties, // Monetary penalties by service
                                                   const RealT fp_electricity_cost, // Electricty cost (in $/Wh) of FP
                                                   const std::vector<RealT>& fp_fn_cat_asleep_costs, // Cost to power-off a FN by FN category
                                                   const std::vector<RealT>& fp_fn_cat_awake_costs, // Cost to power-on a FN by FN category
                                                   RealT deltat = 1 // Length of the time interval
                                            ) const = 0;

    virtual multislot_vm_allocation_t<RealT> solve_with_fixed_fns(const std::vector<std::set<std::size_t>>& fixed_fns, // For each time slot, the set of selected FNs to use for the VM allocation (if in a given time slot the set is empty, any FN can be used)
                                                                  const std::vector<std::size_t>& fn_categories, // Maps every FN to its FN category
                                                                  const std::vector<bool>& fn_power_states, // The power status of each FN
                                                                  const std::vector<std::map<std::size_t,std::pair<std::size_t,std::size_t>>>& fn_vm_allocations, // Current VM allocations, by FN and service
                                                                  const std::vector<RealT>& fn_cat_min_powers, // The min power consumption of FNs by FN category
                                                                  const std::vector<RealT>& fn_cat_max_powers, // The max power consumption of FNs by FN category
                                                                  const std::vector<std::vector<RealT>>& vm_cat_fn_cat_cpu_specs, // The CPU requirement of VMs by VM category and FN category
                                                                  //const std::vector<std::vector<RealT>>& vm_cat_fn_cat_ram_specs, // The RAM requirement of VMs by VM category and FN category
                                                                  const std::vector<RealT>& vm_cat_alloc_costs, // // The cost to allocate a VM on a FN (e.g., cost to boot a VM or to live-migrate its state), by VM category
                                                                  const std::vector<std::size_t>& svc_categories, // Service categories by service
                                                                  const std::vector<std::vector<std::vector<std::size_t>>>& svc_cat_vm_cat_min_num_vms, // The min number of VMs required to achieve QoS, by time slot, service category and VM category
                                                                  const std::vector<RealT>& fp_svc_cat_revenues, // Monetary revenues by service
                                                                  const std::vector<RealT>& fp_svc_cat_penalties, // Monetary penalties by service
                                                                  const RealT fp_electricity_cost, // Electricty cost (in $/Wh) of FP
                                                                  const std::vector<RealT>& fp_fn_cat_asleep_costs, // Cost to power-off a FN by FN category
                                                                  const std::vector<RealT>& fp_fn_cat_awake_costs, // Cost to power-on a FN by FN category
                                                                  RealT deltat = 1 // Length of the time interval
                                                            ) const = 0;
}; // base_multislot_vm_allocation_solver_t


template <typename RealT>
bool check_vm_allocation_solution(const vm_allocation_t<RealT>& vm_alloc)
{
    for (std::size_t fn = 0; fn < vm_alloc.fn_cpu_allocations.size(); ++fn)
    {
        auto const share = vm_alloc.fn_cpu_allocations[fn];

        if (share > 1)
        {
            DCS_DEBUG_TRACE("[WARN] CPU share overflow (FN: " << fn << ", share: " << share << ")");//XXX
            return false;
        }
    }
    for (std::size_t fn = 0; fn < vm_alloc.fn_vm_allocations.size(); ++fn)
    {
        if (vm_alloc.fn_vm_allocations[fn].size() > 0 && !vm_alloc.fn_power_states[fn])
        {
            DCS_DEBUG_TRACE("[WARN] VMs assigned to a powered-off FN (FN: " << fn << ")");//XXX
            return false;
        }
    }

    return true;
}

template <typename RealT>
bool check_vm_allocation_solution(const multislot_vm_allocation_t<RealT>& vm_alloc)
{
    for (std::size_t t = 0; t < vm_alloc.fn_cpu_allocations.size(); ++t)
    {
        for (std::size_t fn = 0; fn < vm_alloc.fn_cpu_allocations[t].size(); ++fn)
        {
            auto const share = vm_alloc.fn_cpu_allocations[t][fn];

            if (share > 1)
            {
                DCS_DEBUG_TRACE("[WARN] CPU share overflow (FN: " << fn << ", share: " << share << ")");//XXX
                return false;
            }
        }
    }
    for (std::size_t t = 0; t < vm_alloc.fn_vm_allocations.size(); ++t)
    {
        for (std::size_t fn = 0; fn < vm_alloc.fn_vm_allocations[t].size(); ++fn)
        {
            if (vm_alloc.fn_vm_allocations[t][fn].size() > 0 && !vm_alloc.fn_power_states[t][fn])
            {
                DCS_DEBUG_TRACE("[WARN] VMs assigned to a powered-off FN (FN: " << fn << ")");//XXX
                return false;
            }
        }
    }

    return true;
}

}} // Namespace dcs::fog


#endif // DCS_FOG_VM_ALLOCATION_COMMONS_HPP
