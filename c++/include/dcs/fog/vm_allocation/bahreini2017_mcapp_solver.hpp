/* vim: set tabstop=4 expandtab shiftwidth=4 softtabstop=4: */

/**
 * \file dcs/fog/vm_allocation_solvers.hpp
 *
 * \brief Solver for the VM allocation problem based on (Bahreini et al.,2017).
 *
 * REFERENCS
 * - Tayebeh Bahreini and Daniel Grosu,
 *   "Efficient Placement of Multi-Component Applications in Edge Computing Systems,"
 *   Proc. of the ACM SEC'17, Oct 12-14, San Jose, CA USA, 2017.
 * .
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

#ifndef DCS_FOG_VM_ALLOCATION_BAHREINE2017_MCAPP_HPP
#define DCS_FOG_VM_ALLOCATION_BAHREINE2017_MCAPP_HPP


#include <cmath>
#include <cstddef>
#include <dcs/assert.hpp>
#include <dcs/debug.hpp>
#include <dcs/exception.hpp>
#include <dcs/fog/commons.hpp>
#include <dcs/fog/io.hpp>
#include <dcs/fog/vm_allocation/commons.hpp>
#include <dcs/logging.hpp>
#include <dcs/macro.hpp>
#include <dcs/math/traits/float.hpp>
#include <iostream>
#include <limits>
#include <ortools/algorithms/hungarian.h>
#include <set>
#include <sstream>
#include <stdexcept>
#include <vector>


namespace dcs { namespace fog {

/**
 * \brief Solver for the VM allocation problem based on (Bahreini et al.,2017).
 *
 * This class implements the MCAPP-IM algorithm proposed by
 * (Bhareini et al, 2017).
 * Specifically, since we don't model communication costs, this class implements
 * the variant called MATCH of the proposed algorithm which only uses the
 * Hungarian algorithm.
 *
 * REFERENCES
 * - Tayebeh Bahreini and Daniel Grosu,
 *   "Efficient Placement of Multi-Component Applications in Edge Computing Systems,"
 *   Proc. of the ACM SEC'17, Oct 12-14, San Jose, CA USA, 2017.
 * .
 */
template <typename RealT>
class bahreini2017_mcappim_vm_allocation_solver_t: public base_vm_allocation_solver_t<RealT>
{
public:
#if 0
    vm_allocation_t<RealT> solve(const std::vector<std::size_t>& fn_categories, // Maps every FN to its FN category
                                 const std::vector<bool>& fn_power_states, // The power status of each FN
                                 const std::vector<RealT>& fn_cat_min_powers, // The min power consumption of FNs by FN category
                                 const std::vector<RealT>& fn_cat_max_powers, // The max power consumption of FNs by FN category
                                 const std::vector<std::vector<RealT>>& vm_cat_fn_cat_cpu_specs, // The CPU requirement of VMs by VM category and FN category
                                 //const std::vector<std::vector<RealT>>& vm_cat_fn_cat_ram_specs, // The RAM requirement of VMs by VM category and FN category
                                 const std::vector<std::size_t>& svc_categories, // Service categories by service
                                 const std::vector<std::vector<std::size_t>>& svc_cat_vm_cat_min_num_vms, // The min number of VMs required to achieve QoS, by service category and VM category
                                 const std::vector<RealT>& fp_svc_cat_revenues, // Monetary revenues by service
                                 const std::vector<RealT>& fp_svc_cat_penalties, // Monetary penalties by service
                                 const RealT fp_electricity_cost, // Electricty cost (in $/Wh) of FP
                                 const std::vector<RealT>& fp_fn_cat_asleep_costs, // Cost to power-off a FN by FN category
                                 const std::vector<RealT>& fp_fn_cat_awake_costs, // Cost to power-on a FN by FN category
                                 RealT deltat = 1 // Length of the time interval
                            ) const
    {
        DCS_DEBUG_TRACE("Finding VM allocation by solving the assignment problem:");
        DCS_DEBUG_TRACE("- Number of FNs: " << fn_categories.size());
        DCS_DEBUG_TRACE("- FN Categories: " << fn_categories);
        DCS_DEBUG_TRACE("- FN Power States: " << fn_power_states);
        DCS_DEBUG_TRACE("- FN Mininimum Power Consumption by FN Category: " << fn_cat_min_powers);
        DCS_DEBUG_TRACE("- FN Maximum Power Consumption by FN Category: " << fn_cat_max_powers);
        DCS_DEBUG_TRACE("- VM CPU requirements by VM Category and FN Category: " << vm_cat_fn_cat_cpu_specs);
        //DCS_DEBUG_TRACE("- VM RAM requirements by VM Category and FN Category: " << vm_cat_fn_cat_ram_specs);
        DCS_DEBUG_TRACE("- Service of FNs: " << svc_categories.size());
        DCS_DEBUG_TRACE("- Service Categories: " << svc_categories);
        DCS_DEBUG_TRACE("- Service Minimum Number of VMs by Service Category and VM Category: " << svc_cat_vm_cat_min_num_vms);
        DCS_DEBUG_TRACE("- FP Service Revenues by Service Category: " << fp_svc_cat_revenues);
        DCS_DEBUG_TRACE("- FP Service Penalties by Service Category: " << fp_svc_cat_penalties);
        DCS_DEBUG_TRACE("- FP Energy Cost: " << fp_electricity_cost);
        DCS_DEBUG_TRACE("- FN On->Off Cost by FN Category: " << fp_fn_cat_asleep_costs);
        DCS_DEBUG_TRACE("- FN Off->On Cost by FN Category: " << fp_fn_cat_awake_costs);
        DCS_DEBUG_TRACE("- Length of the time interval: " << deltat);

        // Compute the cost of assigning a VM to an FN
        // In the assignment problem, a VM is considered a task and an FN is an agent (or server).
        // Thus the solution of the assignment problem is a mapping of 1 VM to a 1 FN and vice versa (no more than 1 VM is allowed on the same FN).

        auto const nfns = fn_categories.size();
        auto const nsvcs = svc_categories.size();

        DCS_ASSERT( nfns == fn_categories.size(),
                    DCS_EXCEPTION_THROW( std::logic_error,
                                         "FN categories container has a wrong size" ) );

        DCS_ASSERT( nfns == fn_power_states.size(),
                    DCS_EXCEPTION_THROW( std::logic_error,
                                         "FN power states container has a wrong size" ) );

        std::unordered_map<std::size_t, std::size_t> svc_cat_vm_categories; // Map service category to the best VM category to use (where "best" is defined in terms of less used CPU)
        std::vector<std::size_t> vm_categories; // Map a VM to its VM category
        std::vector<std::size_t> vm_services; // Map a VM to its service
        std::size_t nvms = 0;
        std::set<std::size_t> fncat_set;
        for (std::size_t fn = 0; fn < nfns; ++fn)
        {
            auto const fn_cat = fn_categories[fn];

            fncat_set.insert(fn_cat);
        }

        for (std::size_t svc = 0; svc < nsvcs; ++svc)
        {
            auto const svc_cat = svc_categories[svc];

            RealT best_tot_cpu_share = 0;
            std::size_t best_fn_cat = 0;
            std::size_t best_vm_cat = 0;
            //for (std::size_t fn = 0; fn < nfns; ++fn)
            //{
            //    auto const fn_cat = fn_categories[fn];
            for (auto const fn_cat : fncat_set)
            {

                // Choose the VM category that, to satisfy the service QoS, requires less CPU capacity
                //for (auto const vm_cat : svc_cat_vm_cat_min_num_vms[svc_cat])
                for (std::size_t vm_cat = 0; vm_cat < svc_cat_vm_cat_min_num_vms[svc_cat].size(); ++vm_cat)
                {
                    auto const tot_cpu_share = vm_cat_fn_cat_cpu_specs[vm_cat][fn_cat]*svc_cat_vm_cat_min_num_vms[svc_cat][vm_cat];

                    if (tot_cpu_share < best_tot_cpu_share)
                    {
                        best_tot_cpu_share = tot_cpu_share;
                        best_fn_cat = fn_cat;
                        best_vm_cat = vm_cat;
                    }
                }
            }

            svc_cat_vm_categories[svc_cat] = best_vm_cat;

            auto svc_nvms = svc_cat_vm_cat_min_num_vms[svc_cat][best_vm_cat];

            vm_services.insert(vm_services.end(), svc_nvms, svc);
            vm_categories.insert(vm_categories.end(), svc_nvms, best_vm_cat);
            nvms += svc_nvms;
        }

        std::vector<std::vector<double>> costs; // Cost matrix C, where C_{ij} is the cost of running VM i on FN j
        std::unordered_map<int, int> direct_assignment; // Map each VM to the FN where it has been allocated
        std::unordered_map<int, int> reverse_assignment; // Map each FN to the assigned VM

        // Compute the cost of assignment VM i to FN j as follows:
        //   C[i,j] = <VM-allocation-cost> + <power-on-cost>
        // where:
        // - <VM-allocation-cost> is the electricity cost spent to allocate and run VM i on FN i and is computed as follows:
        //    (W^{min}_j + (W^{max}_j-W^{min}_j)*U_i)*E
        //   with W^{min}_j and W^{max}_j are the min and max power consumption (in Wh) of FN j, U_i is the CPU requirement of VM i, and E is the electricity cost (in $/Wh).
        // - <power-on-cost> is the electricity cost spent to power on FN i and is computer as follows:
        //    W^{max}_j*K^{on}_j*E
        //   with K^{on}_j is the percentage of an hour taken to power on FN j (e.g., if FN j takes 1 minute to power on, K^{on}_j = 60sec/3600sec)
        //
        costs.resize(nvms);
        for (std::size_t vm = 0; vm < nvms; ++vm)
        {
            DCS_ASSERT( vm < vm_categories.size(),
                        DCS_EXCEPTION_THROW( std::logic_error,
                                             "VM sequence number is out-of-bound" ) );

            auto const vm_cat = vm_categories[vm];

            costs[vm].resize(nfns);
            for (std::size_t fn = 0; fn < nfns; ++fn)
            {
                auto const fn_cat = fn_categories[fn];

                costs[vm][fn] = (fn_cat_min_powers[fn_cat] + (fn_cat_max_powers[fn_cat]-fn_cat_min_powers[fn_cat])*vm_cat_fn_cat_cpu_specs[vm_cat][fn_cat])*fp_electricity_cost;

                // Check if FN must be powered on
                if (!fn_power_states[fn])
                {
                    costs[vm][fn] += fp_fn_cat_awake_costs[fn_cat]/deltat;
                }
            }
        }

DCS_DEBUG_TRACE("COST MATRIX (nvms x nfns : " << nvms << " x " << nfns << "): " << costs);//XXX
        operations_research::MinimizeLinearAssignment(costs, &direct_assignment, &reverse_assignment);

DCS_DEBUG_TRACE("SOLVED");
#ifdef DCS_DEBUG
        DCS_DEBUG_TRACE( "-------------------------------------------------------------------------------[" );
        DCS_DEBUG_TRACE("- Direct assignments:" );
        for (auto const& vm_fn : direct_assignment)
        {
            auto const vm = vm_fn.first;
            auto const fn = vm_fn.second;

            DCS_DEBUG_STREAM << "VM = " << vm << " -> FN = " << fn << " (Cost = " << costs[vm][fn] << ")" << std::endl;
        }
        DCS_DEBUG_TRACE("- Reverse assignments:" );
        for (auto const& fn_vm : reverse_assignment)
        {
            auto const fn = fn_vm.first;
            auto const vm = fn_vm.second;

            DCS_DEBUG_STREAM << "FN = " << fn << " -> VM = " << vm << " (Cost = " << costs[vm][fn] << ")" << std::endl;
        }
        DCS_DEBUG_TRACE( "]-------------------------------------------------------------------------------" );
#endif // DCS_DEBUG

        vm_allocation_t<RealT> solution;

        solution.objective_value = 0;
        solution.solved = true;
        solution.optimal = false;
        solution.fn_vm_allocations.resize(nfns);
        solution.fn_cpu_allocations.resize(nfns, 0);
        solution.fn_power_states.resize(nfns, false);
        solution.revenue = 0;
        solution.cost = 0;
        for (auto const& vm_fn : direct_assignment)
        {
            auto const vm = vm_fn.first;
            auto const fn = vm_fn.second;

            DCS_ASSERT( fn < fn_categories.size(),
                        DCS_EXCEPTION_THROW( std::logic_error,
                                             "FN sequence number is out-of-bound" ) );
            DCS_ASSERT( vm < vm_categories.size() && vm < vm_services.size(),
                        DCS_EXCEPTION_THROW( std::logic_error,
                                             "VM sequence number is out-of-bound" ) );

            auto const fn_cat = fn_categories[fn];
            auto const vm_cat = vm_categories[vm];
            auto const svc = vm_services[vm];
            //auto const svc_cat = svc_categories[svc];

            solution.fn_vm_allocations[fn][svc] = std::make_pair(vm_cat, 1);
            solution.fn_power_states[fn] = true;
            solution.fn_cpu_allocations[fn] = vm_cat_fn_cat_cpu_specs[vm_cat][fn_cat];
            solution.objective_value += - costs[vm][fn];
        }
        for (std::size_t fn = 0; fn < nfns; ++fn)
        {
            auto const fn_cat = fn_categories[fn];

DCS_DEBUG_TRACE("FN: " << fn << " - old power state: " << fn_power_states[fn] << " - new power state: " << solution.fn_power_states[fn]);//XXX
            if (fn_power_states[fn] && !solution.fn_power_states[fn])
            {
                // Add power-off costs
DCS_DEBUG_TRACE("Adding asleep-cost: " << fp_fn_cat_awake_costs[fn_cat]/deltat << " to " << solution.cost);
                solution.cost += fp_fn_cat_asleep_costs[fn_cat]/deltat;
            }
            //else if (!fn_power_states[fn] && solution.fn_power_states[fn])
            //{
            //    // Add power-on costs
            //    solution.cost += fp_fn_cat_awake_costs[fn_cat]/deltat;
            //}
            else if (solution.fn_power_states[fn])
            {
                //NO: already accounted in the costs matrix
                //if (!fn_power_states[fn])
                //{
                //    // Add power-on costs
//DCS_DEBUG_TRACE("Adding awake-cost: " << fp_fn_cat_awake_costs[fn_cat]/deltat << " to " << solution.cost);
                //    solution.cost += fp_fn_cat_awake_costs[fn_cat]/deltat;
                //}

                if (reverse_assignment.count(fn) > 0)
                {
                    auto const vm = reverse_assignment[fn];
                    auto const svc = vm_services[vm];
                    auto const svc_cat = svc_categories[svc];

                    solution.cost += costs[vm][fn];
                    solution.revenue += fp_svc_cat_revenues[svc_cat];
                }
            }
        }
        std::set<std::size_t> svc_with_penalty;
        for (std::size_t vm = 0; vm < nvms; ++vm)
        {
            if (direct_assignment.count(vm) == 0)
            {
                auto const svc = vm_services[vm];

                svc_with_penalty.insert(svc);
            }
        }
        for (auto const svc : svc_with_penalty)
        {
            auto const svc_cat = svc_categories[svc];

            solution.cost += fp_svc_cat_penalties[svc_cat];
        }
        //solution.objective_value = solution.revenue - solution.cost;

        return solution;
    }
#endif

    vm_allocation_t<RealT> solve(const std::vector<std::size_t>& fn_categories, // Maps every FN to its FN category
                                 const std::vector<bool>& fn_power_states, // The power status of each FN
                                 const std::vector<std::map<std::size_t,std::pair<std::size_t,std::size_t>>>& fn_vm_allocations, // Current VM allocations, by FN and service
                                 const std::vector<RealT>& fn_cat_min_powers, // The min power consumption of FNs by FN category
                                 const std::vector<RealT>& fn_cat_max_powers, // The max power consumption of FNs by FN category
                                 const std::vector<std::vector<RealT>>& vm_cat_fn_cat_cpu_specs, // The CPU requirement of VMs by VM category and FN category
                                 //const std::vector<std::vector<RealT>>& vm_cat_fn_cat_ram_specs, // The RAM requirement of VMs by VM category and FN category
                                 const std::vector<RealT>& vm_cat_alloc_costs, // The cost to allocate a VM on a FN (e.g., cost to boot a VM or to live-migrate its state), by VM category
                                 const std::vector<std::size_t>& svc_categories, // Service categories by service
                                 const std::vector<std::vector<std::size_t>>& svc_cat_vm_cat_min_num_vms, // The min number of VMs required to achieve QoS, by service category and VM category
                                 const std::vector<RealT>& fp_svc_cat_revenues, // Monetary revenues by service
                                 const std::vector<RealT>& fp_svc_cat_penalties, // Monetary penalties by service
                                 const RealT fp_electricity_cost, // Electricty cost (in $/Wh) of FP
                                 const std::vector<RealT>& fp_fn_cat_asleep_costs, // Cost to power-off a FN by FN category
                                 const std::vector<RealT>& fp_fn_cat_awake_costs, // Cost to power-on a FN by FN category
                                 RealT deltat = 1 // Length of the time interval
                            ) const
    {
        return do_solve(fn_categories,
                        fn_power_states,
                        fn_vm_allocations,
                        std::set<std::size_t>(), // Any FN can be selected
                        fn_cat_min_powers,
                        fn_cat_max_powers,
                        vm_cat_fn_cat_cpu_specs,
                        //vm_cat_fn_cat_ram_specs,
                        vm_cat_alloc_costs,
                        svc_categories,
                        svc_cat_vm_cat_min_num_vms,
                        fp_svc_cat_revenues,
                        fp_svc_cat_penalties,
                        fp_electricity_cost,
                        fp_fn_cat_asleep_costs,
                        fp_fn_cat_awake_costs,
                        deltat);
    }

    vm_allocation_t<RealT> solve_with_fixed_fns(const std::set<std::size_t>& fixed_fns,
                                                const std::vector<std::size_t>& fn_categories, // Maps every FN to its FN category
                                                const std::vector<bool>& fn_power_states, // The power status of each FN
                                                const std::vector<std::map<std::size_t,std::pair<std::size_t,std::size_t>>>& fn_vm_allocations, // Current VM allocations, by FN and service
                                                const std::vector<RealT>& fn_cat_min_powers, // The min power consumption of FNs by FN category
                                                const std::vector<RealT>& fn_cat_max_powers, // The max power consumption of FNs by FN category
                                                const std::vector<std::vector<RealT>>& vm_cat_fn_cat_cpu_specs, // The CPU requirement of VMs by VM category and FN category
                                                //const std::vector<std::vector<RealT>>& vm_cat_fn_cat_ram_specs, // The RAM requirement of VMs by VM category and FN category
                                                const std::vector<RealT>& vm_cat_alloc_costs, // The cost to allocate a VM on a FN (e.g., cost to boot a VM or to live-migrate its state), by VM category
                                                const std::vector<std::size_t>& svc_categories, // Service categories by service
                                                const std::vector<std::vector<std::size_t>>& svc_cat_vm_cat_min_num_vms, // The min number of VMs required to achieve QoS, by service category and VM category
                                                const std::vector<RealT>& fp_svc_cat_revenues, // Monetary revenues by service
                                                const std::vector<RealT>& fp_svc_cat_penalties, // Monetary penalties by service
                                                const RealT fp_electricity_cost, // Electricty cost (in $/Wh) of FP
                                                const std::vector<RealT>& fp_fn_cat_asleep_costs, // Cost to power-off a FN by FN category
                                                const std::vector<RealT>& fp_fn_cat_awake_costs, // Cost to power-on a FN by FN category
                                                RealT deltat = 1 // Length of the time interval
                            ) const
    {
        return do_solve(fn_categories,
                        fn_power_states,
                        fn_vm_allocations,
                        fixed_fns,
                        fn_cat_min_powers,
                        fn_cat_max_powers,
                        vm_cat_fn_cat_cpu_specs,
                        //vm_cat_fn_cat_ram_specs,
                        vm_cat_alloc_costs,
                        svc_categories,
                        svc_cat_vm_cat_min_num_vms,
                        fp_svc_cat_revenues,
                        fp_svc_cat_penalties,
                        fp_electricity_cost,
                        fp_fn_cat_asleep_costs,
                        fp_fn_cat_awake_costs,
                        deltat);
    }

#if 0
    vm_allocation_t<RealT> solve_with_fixed_fns(const std::vector<std::size_t>& fn_categories, // Maps every FN to its FN category
                                                const std::vector<RealT>& fn_cat_min_powers, // The min power consumption of FNs by FN category
                                                const std::vector<RealT>& fn_cat_max_powers, // The max power consumption of FNs by FN category
                                                const std::vector<std::vector<RealT>>& vm_cat_fn_cat_cpu_specs, // The CPU requirement of VMs by VM category and FN category
                                                //const std::vector<std::vector<RealT>>& vm_cat_fn_cat_ram_specs, // The RAM requirement of VMs by VM category and FN category
                                                const std::vector<std::size_t>& svc_categories, // Service categories by service
                                                const std::vector<std::vector<std::size_t>>& svc_cat_vm_cat_min_num_vms, // The min number of VMs required to achieve QoS, by service category and VM category
                                                const std::vector<RealT>& fp_svc_cat_revenues, // Monetary revenues by service
                                                const std::vector<RealT>& fp_svc_cat_penalties, // Monetary penalties by service
                                                const RealT fp_electricity_cost, // Electricty cost (in $/Wh) of FP
                                                RealT deltat = 1 // Length of the time interval
                                            ) const
    {
        DCS_DEBUG_TRACE("Finding VM allocation by solving the assignment problem:");
        DCS_DEBUG_TRACE("- Number of FNs: " << fn_categories.size());
        DCS_DEBUG_TRACE("- FN Categories: " << fn_categories);
        DCS_DEBUG_TRACE("- FN Mininimum Power Consumption by FN Category: " << fn_cat_min_powers);
        DCS_DEBUG_TRACE("- FN Maximum Power Consumption by FN Category: " << fn_cat_max_powers);
        DCS_DEBUG_TRACE("- VM CPU requirements by VM Category and FN Category: " << vm_cat_fn_cat_cpu_specs);
        //DCS_DEBUG_TRACE("- VM RAM requirements by VM Category and FN Category: " << vm_cat_fn_cat_ram_specs);
        DCS_DEBUG_TRACE("- Service of FNs: " << svc_categories.size());
        DCS_DEBUG_TRACE("- Service Categories: " << svc_categories);
        DCS_DEBUG_TRACE("- Service Minimum Number of VMs by Service Category and VM Category: " << svc_cat_vm_cat_min_num_vms);
        DCS_DEBUG_TRACE("- FP Service Revenues by Service Category: " << fp_svc_cat_revenues);
        DCS_DEBUG_TRACE("- FP Service Penalties by Service Category: " << fp_svc_cat_penalties);
        DCS_DEBUG_TRACE("- FP Energy Cost: " << fp_electricity_cost);
        DCS_DEBUG_TRACE("- Length of the time interval: " << deltat);

        // Compute the cost of assigning a VM to an FN
        // In the assignment problem, a VM is considered a task and an FN is an agent (or server).
        // Thus the solution of the assignment problem is a mapping of 1 VM to a 1 FN and vice versa (no more than 1 VM is allowed on the same FN).

        auto const nfns = fn_categories.size();
        auto const nsvcs = svc_categories.size();

        DCS_ASSERT( nfns == fn_categories.size(),
                    DCS_EXCEPTION_THROW( std::logic_error,
                                         "FN categories container has a wrong size" ) );

        std::unordered_map<std::size_t, std::size_t> svc_cat_vm_categories; // Map service category to the best VM category to use (where "best" is defined in terms of less used CPU)
        std::vector<std::size_t> vm_categories; // Map a VM to its VM category
        std::vector<std::size_t> vm_services; // Map a VM to its service
        std::size_t nvms = 0;
        std::set<std::size_t> fncat_set;
        for (std::size_t fn = 0; fn < nfns; ++fn)
        {
            auto const fn_cat = fn_categories[fn];

            fncat_set.insert(fn_cat);
        }

        for (std::size_t svc = 0; svc < nsvcs; ++svc)
        {
            auto const svc_cat = svc_categories[svc];

            RealT best_tot_cpu_share = 0;
            std::size_t best_fn_cat = 0;
            std::size_t best_vm_cat = 0;
            //for (std::size_t fn = 0; fn < nfns; ++fn)
            //{
            //    auto const fn_cat = fn_categories[fn];
            for (auto const fn_cat : fncat_set)
            {

                // Choose the VM category that, to satisfy the service QoS, requires less CPU capacity
                //for (auto const vm_cat : svc_cat_vm_cat_min_num_vms[svc_cat])
                for (std::size_t vm_cat = 0; vm_cat < svc_cat_vm_cat_min_num_vms[svc_cat].size(); ++vm_cat)
                {
                    auto const tot_cpu_share = vm_cat_fn_cat_cpu_specs[vm_cat][fn_cat]*svc_cat_vm_cat_min_num_vms[svc_cat][vm_cat];

                    if (tot_cpu_share < best_tot_cpu_share)
                    {
                        best_tot_cpu_share = tot_cpu_share;
                        best_fn_cat = fn_cat;
                        best_vm_cat = vm_cat;
                    }
                }
            }

            svc_cat_vm_categories[svc_cat] = best_vm_cat;

            auto svc_nvms = svc_cat_vm_cat_min_num_vms[svc_cat][best_vm_cat];

            vm_services.insert(vm_services.end(), svc_nvms, svc);
            vm_categories.insert(vm_categories.end(), svc_nvms, best_vm_cat);
            nvms += svc_nvms;
        }

        std::vector<std::vector<double>> costs; // Cost matrix C, where C_{ij} is the cost of running VM i on FN j
        std::unordered_map<int, int> direct_assignment; // Map each VM to the FN where it has been allocated
        std::unordered_map<int, int> reverse_assignment; // Map each FN to the assigned VM

        // Compute the cost of assignment VM i to FN j as follows:
        //   C[i,j] = <VM-allocation-cost> + <power-on-cost>
        // where:
        // - <VM-allocation-cost> is the electricity cost spent to allocate and run VM i on FN i and is computed as follows:
        //    (W^{min}_j + (W^{max}_j-W^{min}_j)*U_i)*E
        //   with W^{min}_j and W^{max}_j are the min and max power consumption (in Wh) of FN j, U_i is the CPU requirement of VM i, and E is the electricity cost (in $/Wh).
        // - <power-on-cost> is the electricity cost spent to power on FN i and is computer as follows:
        //    W^{max}_j*K^{on}_j*E
        //   with K^{on}_j is the percentage of an hour taken to power on FN j (e.g., if FN j takes 1 minute to power on, K^{on}_j = 60sec/3600sec)
        //
        costs.resize(nvms);
        for (std::size_t vm = 0; vm < nvms; ++vm)
        {
            DCS_ASSERT( vm < vm_categories.size(),
                        DCS_EXCEPTION_THROW( std::logic_error,
                                             "VM sequence number is out-of-bound" ) );

            auto const vm_cat = vm_categories[vm];

            costs[vm].resize(nfns);
            for (std::size_t fn = 0; fn < nfns; ++fn)
            {
                auto const fn_cat = fn_categories[fn];

                costs[vm][fn] = (fn_cat_min_powers[fn_cat] + (fn_cat_max_powers[fn_cat]-fn_cat_min_powers[fn_cat])*vm_cat_fn_cat_cpu_specs[vm_cat][fn_cat])*fp_electricity_cost;
            }
        }

DCS_DEBUG_TRACE("COST MATRIX (nvms x nfns : " << nvms << " x " << nfns << "): " << costs);//XXX
        operations_research::MinimizeLinearAssignment(costs, &direct_assignment, &reverse_assignment);

DCS_DEBUG_TRACE("SOLVED");
#ifdef DCS_DEBUG
        DCS_DEBUG_TRACE( "-------------------------------------------------------------------------------[" );
        DCS_DEBUG_TRACE("- Direct assignments:" );
        for (auto const& vm_fn : direct_assignment)
        {
            auto const vm = vm_fn.first;
            auto const fn = vm_fn.second;

            DCS_DEBUG_STREAM << "VM = " << vm << " -> FN = " << fn << " (Cost = " << costs[vm][fn] << ")" << std::endl;
        }
        DCS_DEBUG_TRACE("- Reverse assignments:" );
        for (auto const& fn_vm : reverse_assignment)
        {
            auto const fn = fn_vm.first;
            auto const vm = fn_vm.second;

            DCS_DEBUG_STREAM << "FN = " << fn << " -> VM = " << vm << " (Cost = " << costs[vm][fn] << ")" << std::endl;
        }
        DCS_DEBUG_TRACE( "]-------------------------------------------------------------------------------" );
#endif // DCS_DEBUG

        vm_allocation_t<RealT> solution;

        solution.objective_value = 0;
        solution.solved = true;
        solution.optimal = false;
        solution.fn_vm_allocations.resize(nfns);
        solution.fn_cpu_allocations.resize(nfns, 0);
        solution.fn_power_states.resize(nfns, false);
        solution.revenue = 0;
        solution.cost = 0;
        for (auto const& vm_fn : direct_assignment)
        {
            auto const vm = vm_fn.first;
            auto const fn = vm_fn.second;

            DCS_ASSERT( fn < fn_categories.size(),
                        DCS_EXCEPTION_THROW( std::logic_error,
                                             "FN sequence number is out-of-bound" ) );
            DCS_ASSERT( vm < vm_categories.size() && vm < vm_services.size(),
                        DCS_EXCEPTION_THROW( std::logic_error,
                                             "VM sequence number is out-of-bound" ) );

            auto const fn_cat = fn_categories[fn];
            auto const vm_cat = vm_categories[vm];
            auto const svc = vm_services[vm];
            //auto const svc_cat = svc_categories[svc];

            solution.fn_vm_allocations[fn][svc] = std::make_pair(vm_cat, 1);
            solution.fn_power_states[fn] = true;
            solution.fn_cpu_allocations[fn] = vm_cat_fn_cat_cpu_specs[vm_cat][fn_cat];
            solution.objective_value += - costs[vm][fn];
        }
        for (std::size_t fn = 0; fn < nfns; ++fn)
        {
            auto const fn_cat = fn_categories[fn];

DCS_DEBUG_TRACE("FN: " << fn << " - new power state: " << solution.fn_power_states[fn]);//XXX
            if (solution.fn_power_states[fn])
            {
                //NO: already accounted in the costs matrix
                //if (!fn_power_states[fn])
                //{
                //    // Add power-on costs
//DCS_DEBUG_TRACE("Adding awake-cost: " << fp_fn_cat_awake_costs[fn_cat]/deltat << " to " << solution.cost);
                //    solution.cost += fp_fn_cat_awake_costs[fn_cat]/deltat;
                //}

                if (reverse_assignment.count(fn) > 0)
                {
                    auto const vm = reverse_assignment[fn];
                    auto const svc = vm_services[vm];
                    auto const svc_cat = svc_categories[svc];

                    solution.cost += costs[vm][fn];
                    solution.revenue += fp_svc_cat_revenues[svc_cat];
                }
            }
        }
        std::set<std::size_t> svc_with_penalty;
        for (std::size_t vm = 0; vm < nvms; ++vm)
        {
            if (direct_assignment.count(vm) == 0)
            {
                auto const svc = vm_services[vm];

                svc_with_penalty.insert(svc);
            }
        }
        for (auto const svc : svc_with_penalty)
        {
            auto const svc_cat = svc_categories[svc];

            solution.cost += fp_svc_cat_penalties[svc_cat];
        }
        //solution.objective_value = solution.revenue - solution.cost;

        return solution;
    }
#endif


private:
    vm_allocation_t<RealT> do_solve(const std::vector<std::size_t>& fn_categories, // Maps every FN to its FN category
                                    const std::vector<bool>& fn_power_states, // The power status of each FN
                                    const std::vector<std::map<std::size_t,std::pair<std::size_t,std::size_t>>>& fn_vm_allocations, // Current VM allocations, by FN and service
                                    const std::set<std::size_t>& fixed_fns, // The set of selected FNs to use for the VM allocation (if empty, any FN can be used)
                                    const std::vector<RealT>& fn_cat_min_powers, // The min power consumption of FNs by FN category
                                    const std::vector<RealT>& fn_cat_max_powers, // The max power consumption of FNs by FN category
                                    const std::vector<std::vector<RealT>>& vm_cat_fn_cat_cpu_specs, // The CPU requirement of VMs by VM category and FN category
                                    //const std::vector<std::vector<RealT>>& vm_cat_fn_cat_ram_specs, // The RAM requirement of VMs by VM category and FN category
                                    const std::vector<RealT>& vm_cat_alloc_costs, // The cost to allocate a VM on a FN (e.g., cost to boot a VM or to live-migrate its state), by VM category
                                    const std::vector<std::size_t>& svc_categories, // Service categories by service
                                    const std::vector<std::vector<std::size_t>>& svc_cat_vm_cat_min_num_vms, // The min number of VMs required to achieve QoS, by service category and VM category
                                    const std::vector<RealT>& fp_svc_cat_revenues, // Monetary revenues by service
                                    const std::vector<RealT>& fp_svc_cat_penalties, // Monetary penalties by service
                                    const RealT fp_electricity_cost, // Electricty cost (in $/Wh) of FP
                                    const std::vector<RealT>& fp_fn_cat_asleep_costs, // Cost to power-off a FN by FN category
                                    const std::vector<RealT>& fp_fn_cat_awake_costs, // Cost to power-on a FN by FN category
                                    RealT deltat = 1 // Length of the time interval
                                ) const
    {
        DCS_DEBUG_TRACE("Finding VM allocation by solving the assignment problem:");
        DCS_DEBUG_TRACE("- Number of FNs: " << fn_categories.size());
        DCS_DEBUG_TRACE("- FN Categories: " << fn_categories);
        DCS_DEBUG_TRACE("- FN Power States: " << fn_power_states);
        DCS_DEBUG_TRACE("- FN - VM Allocations: " << fn_vm_allocations);
        DCS_DEBUG_TRACE("- FN Fixed: " << fixed_fns);
        DCS_DEBUG_TRACE("- FN Mininimum Power Consumption by FN Category: " << fn_cat_min_powers);
        DCS_DEBUG_TRACE("- FN Maximum Power Consumption by FN Category: " << fn_cat_max_powers);
        DCS_DEBUG_TRACE("- VM CPU requirements by VM Category and FN Category: " << vm_cat_fn_cat_cpu_specs);
        //DCS_DEBUG_TRACE("- VM RAM requirements by VM Category and FN Category: " << vm_cat_fn_cat_ram_specs);
        DCS_DEBUG_TRACE("- VM Allocation Costs by VM Category: " << vm_cat_alloc_costs);
        DCS_DEBUG_TRACE("- Service of FNs: " << svc_categories.size());
        DCS_DEBUG_TRACE("- Service Categories: " << svc_categories);
        DCS_DEBUG_TRACE("- Service Minimum Number of VMs by Service Category and VM Category: " << svc_cat_vm_cat_min_num_vms);
        DCS_DEBUG_TRACE("- FP Service Revenues by Service Category: " << fp_svc_cat_revenues);
        DCS_DEBUG_TRACE("- FP Service Penalties by Service Category: " << fp_svc_cat_penalties);
        DCS_DEBUG_TRACE("- FP Energy Cost: " << fp_electricity_cost);
        DCS_DEBUG_TRACE("- FN On->Off Cost by FN Category: " << fp_fn_cat_asleep_costs);
        DCS_DEBUG_TRACE("- FN Off->On Cost by FN Category: " << fp_fn_cat_awake_costs);
        DCS_DEBUG_TRACE("- Length of the time interval: " << deltat);

        // Compute the cost of assigning a VM to an FN
        // In the assignment problem, a VM is considered a task and an FN is an agent (or server).
        // Thus the solution of the assignment problem is a mapping of 1 VM to a 1 FN and vice versa (no more than 1 VM is allowed on the same FN).

        auto const nfns = fn_categories.size();
        auto const nsvcs = svc_categories.size();

        DCS_ASSERT( nfns == fn_categories.size(),
                    DCS_EXCEPTION_THROW( std::logic_error,
                                         "FN categories container has a wrong size" ) );

        DCS_ASSERT( nfns == fn_power_states.size(),
                    DCS_EXCEPTION_THROW( std::logic_error,
                                         "FN power states container has a wrong size" ) );

        std::unordered_map<std::size_t, std::size_t> svc_cat_vm_categories; // Map service category to the best VM category to use (where "best" is defined in terms of less used CPU)
        std::vector<std::size_t> vm_categories; // Map a VM to its VM category
        std::vector<std::size_t> vm_services; // Map a VM to its service
        std::size_t nvms = 0;
        std::set<std::size_t> fncat_set;
        for (std::size_t fn = 0; fn < nfns; ++fn)
        {
            auto const fn_cat = fn_categories[fn];

            fncat_set.insert(fn_cat);
        }

        for (std::size_t svc = 0; svc < nsvcs; ++svc)
        {
            auto const svc_cat = svc_categories[svc];

            RealT best_tot_cpu_share = std::numeric_limits<RealT>::infinity();
            //std::size_t best_fn_cat = 0;
            std::size_t best_vm_cat = 0;
            //for (std::size_t fn = 0; fn < nfns; ++fn)
            //{
            //    auto const fn_cat = fn_categories[fn];
            for (auto const fn_cat : fncat_set)
            {

                // Choose the VM category that, to satisfy the service QoS, requires less CPU capacity
                //for (auto const vm_cat : svc_cat_vm_cat_min_num_vms[svc_cat])
                for (std::size_t vm_cat = 0; vm_cat < svc_cat_vm_cat_min_num_vms[svc_cat].size(); ++vm_cat)
                {
                    auto const tot_cpu_share = vm_cat_fn_cat_cpu_specs[vm_cat][fn_cat]*svc_cat_vm_cat_min_num_vms[svc_cat][vm_cat];

DCS_DEBUG_TRACE("Looking for VM CATEGORY - SVC: " << svc << " (cat: " << svc_cat << "), FN CAT: " << fn_cat << ", VM CAT: " << vm_cat << ", NUM: " << svc_cat_vm_cat_min_num_vms[svc_cat][vm_cat] << ", REQ: " << vm_cat_fn_cat_cpu_specs[vm_cat][fn_cat] << " -> TOT SHARE: " << tot_cpu_share << " vs. BEST SHARE: " << best_tot_cpu_share);//XXX
                    if (tot_cpu_share < best_tot_cpu_share)
                    {
                        best_tot_cpu_share = tot_cpu_share;
                        //best_fn_cat = fn_cat;
                        best_vm_cat = vm_cat;
                    }
                }
            }

            svc_cat_vm_categories[svc_cat] = best_vm_cat;

            auto svc_nvms = svc_cat_vm_cat_min_num_vms[svc_cat][best_vm_cat];

            vm_services.insert(vm_services.end(), svc_nvms, svc);
DCS_DEBUG_TRACE("Update VM_SERVICES: " << vm_services);//XXX
            vm_categories.insert(vm_categories.end(), svc_nvms, best_vm_cat);
DCS_DEBUG_TRACE("Update VM_CATEGORIES: " << vm_categories);//XXX
            nvms += svc_nvms;
        }

        std::vector<std::vector<double>> costs; // Cost matrix C, where C_{ij} is the cost of running VM i on FN j
        std::unordered_map<int, int> direct_assignment; // Map each VM to the FN where it has been allocated
        std::unordered_map<int, int> reverse_assignment; // Map each FN to the assigned VM

        // Compute the cost of assignment VM i to FN j as follows:
        //   C[i,j] = <VM-allocation-cost> + <power-on-cost>
        // where:
        // - <VM-allocation-cost> is the electricity cost spent to allocate and run VM i on FN i and is computed as follows:
        //    (W^{min}_j + (W^{max}_j-W^{min}_j)*U_i)*E
        //   with W^{min}_j and W^{max}_j are the min and max power consumption (in Wh) of FN j, U_i is the CPU requirement of VM i, and E is the electricity cost (in $/Wh).
        // - <power-on-cost> is the electricity cost spent to power on FN i and is computer as follows:
        //    W^{max}_j*K^{on}_j*E
        //   with K^{on}_j is the percentage of an hour taken to power on FN j (e.g., if FN j takes 1 minute to power on, K^{on}_j = 60sec/3600sec)
        //
        costs.resize(nvms);
        for (std::size_t vm = 0; vm < nvms; ++vm)
        {
            DCS_ASSERT( vm < vm_categories.size(),
                        DCS_EXCEPTION_THROW( std::logic_error,
                                             "VM sequence number is out-of-bound" ) );

            auto const vm_cat = vm_categories[vm];
            auto const svc = vm_services[vm];

            costs[vm].resize(nfns);
            for (std::size_t fn = 0; fn < nfns; ++fn)
            {
                auto const fn_cat = fn_categories[fn];

                // Add energy consumption costs
                if (fixed_fns.size() > 0 && fixed_fns.count(fn) == 0)
                {
                    // Assign to this FN a very high cost so that it cannot be selected
                    costs[vm][fn] = std::numeric_limits<double>::max();
                }
                else
                {
                    costs[vm][fn] = (fn_cat_min_powers[fn_cat] + (fn_cat_max_powers[fn_cat]-fn_cat_min_powers[fn_cat])*vm_cat_fn_cat_cpu_specs[vm_cat][fn_cat])*fp_electricity_cost;
                }

                // Add VM allocation costs
                if (fn_vm_allocations[fn].count(svc) == 0 || fn_vm_allocations[fn].at(svc).first != vm_cat)
                {
DCS_DEBUG_TRACE("FN: " << fn << ", SVC: " << svc << ", VM: " << vm << " -> ADDING ALLOCATION COSTS: " << (vm_cat_alloc_costs[vm_cat]/deltat) << " to " << costs[vm][fn]);//XXX
                    // The VM has not already been allocated to this FN, so add allocation cost
                    costs[vm][fn] += vm_cat_alloc_costs[vm_cat]/deltat;
                }

                // Check if FN must be powered on and if so add the related costs
                if (!fn_power_states[fn])
                {
DCS_DEBUG_TRACE("FN: " << fn << ", SVC: " << svc << ", VM: " << vm << " -> ADDING AWAKE COSTS: " << (fp_fn_cat_awake_costs[fn_cat]/deltat) << " to " << costs[vm][fn]);//XXX
                    costs[vm][fn] += fp_fn_cat_awake_costs[fn_cat]/deltat;
                }
            }
        }

DCS_DEBUG_TRACE("COST MATRIX (nvms x nfns : " << nvms << " x " << nfns << "): " << costs);//XXX
        operations_research::MinimizeLinearAssignment(costs, &direct_assignment, &reverse_assignment);

DCS_DEBUG_TRACE("SOLVED");
#ifdef DCS_DEBUG
        DCS_DEBUG_TRACE( "-------------------------------------------------------------------------------[" );
        DCS_DEBUG_TRACE("- Direct assignments:" );
        for (auto const& vm_fn : direct_assignment)
        {
            auto const vm = vm_fn.first;
            auto const fn = vm_fn.second;

            DCS_DEBUG_STREAM << "VM = " << vm << " -> FN = " << fn << " (Cost = " << costs[vm][fn] << ")" << std::endl;
        }
        DCS_DEBUG_TRACE("- Reverse assignments:" );
        for (auto const& fn_vm : reverse_assignment)
        {
            auto const fn = fn_vm.first;
            auto const vm = fn_vm.second;

            DCS_DEBUG_STREAM << "FN = " << fn << " -> VM = " << vm << " (Cost = " << costs[vm][fn] << ")" << std::endl;
        }
        DCS_DEBUG_TRACE( "]-------------------------------------------------------------------------------" );
#endif // DCS_DEBUG

        vm_allocation_t<RealT> solution;

        solution.objective_value = 0;
        solution.solved = true;
        solution.optimal = false;
        solution.fn_vm_allocations.resize(nfns);
        solution.fn_cpu_allocations.resize(nfns, 0);
        //solution.fn_power_states.resize(nfns, false);
        solution.fn_power_states.assign(fn_power_states.begin(), fn_power_states.end());
        solution.revenue = 0;
        solution.cost = 0;
        for (auto const& vm_fn : direct_assignment)
        {
            auto const vm = vm_fn.first;
            auto const fn = vm_fn.second;

            DCS_ASSERT( static_cast<std::size_t>(fn) < fn_categories.size(),
                        DCS_EXCEPTION_THROW( std::logic_error,
                                             "FN sequence number is out-of-bound" ) );
            DCS_ASSERT( static_cast<std::size_t>(vm) < vm_categories.size() && static_cast<std::size_t>(vm) < vm_services.size(),
                        DCS_EXCEPTION_THROW( std::logic_error,
                                             "VM sequence number is out-of-bound" ) );

            if (fixed_fns.size() > 0 && fixed_fns.count(fn) == 0)
            {
                // Skip this assigment because the involved FN was not selected
                continue;
            }

            auto const fn_cat = fn_categories[fn];
            auto const vm_cat = vm_categories[vm];
            auto const svc = vm_services[vm];
            //auto const svc_cat = svc_categories[svc];

            solution.fn_vm_allocations[fn][svc] = std::make_pair(vm_cat, 1);
            solution.fn_power_states[fn] = true;
            solution.fn_cpu_allocations[fn] = vm_cat_fn_cat_cpu_specs[vm_cat][fn_cat];
            solution.objective_value += - costs[vm][fn];
        }
        for (std::size_t fn = 0; fn < nfns; ++fn)
        {
            auto const fn_cat = fn_categories[fn];

            if (fixed_fns.size() > 0 && fixed_fns.count(fn) > 0)
            {
                // Make sure the FN is powered on regardless its assignment
                solution.fn_power_states[fn] = true;
            }
            else if (solution.fn_vm_allocations[fn].size() == 0)
            {
                // Power off this FN because it has not been selected in the optimal assigment
                solution.fn_power_states[fn] = false;
            }

DCS_DEBUG_TRACE("FN: " << fn << " - old power state: " << fn_power_states[fn] << " - new power state: " << solution.fn_power_states[fn]);//XXX
            if (fn_power_states[fn] && !solution.fn_power_states[fn])
            {
                // Add power-off costs
DCS_DEBUG_TRACE("ADDING ASLEEP-COST: " << fp_fn_cat_asleep_costs[fn_cat]/deltat << " to " << solution.cost);
                solution.cost += fp_fn_cat_asleep_costs[fn_cat]/deltat;
            }
            //else if (!fn_power_states[fn] && solution.fn_power_states[fn])
            //{
            //    // Add power-on costs
            //    solution.cost += fp_fn_cat_awake_costs[fn_cat]/deltat;
            //}
            else if (solution.fn_power_states[fn])
            {
                //NO: already accounted in the costs matrix
                //if (!fn_power_states[fn])
                //{
                //    // Add power-on costs
//DCS_DEBUG_TRACE("Adding awake-cost: " << fp_fn_cat_awake_costs[fn_cat]/deltat << " to " << solution.cost);
                //    solution.cost += fp_fn_cat_awake_costs[fn_cat]/deltat;
                //}

                if (reverse_assignment.count(fn) > 0)
                {
                    auto const vm = reverse_assignment[fn];
                    auto const svc = vm_services[vm];
                    auto const svc_cat = svc_categories[svc];

                    solution.cost += costs[vm][fn];
                    solution.revenue += fp_svc_cat_revenues[svc_cat];
                }
            }
        }
        std::set<std::size_t> svc_with_penalty;
        for (std::size_t vm = 0; vm < nvms; ++vm)
        {
            // Add penalty cost if either a VM needed by a service has not been assignment or if it has been assigned to a FN that has not been selected
            if (direct_assignment.count(vm) == 0
                || (fixed_fns.size() > 0 && fixed_fns.count(direct_assignment.at(vm)) == 0))
            {
                auto const svc = vm_services[vm];

DCS_DEBUG_TRACE("VM: " << vm << " for SVC: " << svc << " should have been mapped but it's not");
                svc_with_penalty.insert(svc);
            }
        }
        for (auto const svc : svc_with_penalty)
        {
            auto const svc_cat = svc_categories[svc];

DCS_DEBUG_TRACE("SVC: " << svc << " - ADDING PENALTY COST: " << fp_svc_cat_penalties[svc_cat] << " to " << solution.cost);
            solution.cost += fp_svc_cat_penalties[svc_cat];
        }
        //solution.objective_value = solution.revenue - solution.cost;
        solution.revenue *= deltat;
        solution.cost *= deltat;
        solution.profit = solution.revenue-solution.cost;
DCS_DEBUG_TRACE("Final VM Allocation: " << solution.fn_vm_allocations);//XXX

        return solution;
    }
}; // bahreini2017_mcappim_vm_allocation_solver_t


template <typename RealT>
class bahreini2017_mcappim_alt_vm_allocation_solver_t: public base_vm_allocation_solver_t<RealT>
{
public:
    vm_allocation_t<RealT> solve(const std::vector<std::size_t>& fn_categories, // Maps every FN to its FN category
                                 const std::vector<bool>& fn_power_states, // The power status of each FN
                                 const std::vector<std::map<std::size_t,std::pair<std::size_t,std::size_t>>>& fn_vm_allocations, // Current VM allocations, by FN and service
                                 const std::vector<RealT>& fn_cat_min_powers, // The min power consumption of FNs by FN category
                                 const std::vector<RealT>& fn_cat_max_powers, // The max power consumption of FNs by FN category
                                 const std::vector<std::vector<RealT>>& vm_cat_fn_cat_cpu_specs, // The CPU requirement of VMs by VM category and FN category
                                 //const std::vector<std::vector<RealT>>& vm_cat_fn_cat_ram_specs, // The RAM requirement of VMs by VM category and FN category
                                 const std::vector<RealT>& vm_cat_alloc_costs, // The cost to allocate a VM on a FN (e.g., cost to boot a VM or to live-migrate its state), by VM category
                                 const std::vector<std::size_t>& svc_categories, // Service categories by service
                                 const std::vector<std::vector<std::size_t>>& svc_cat_vm_cat_min_num_vms, // The min number of VMs required to achieve QoS, by service category and VM category
                                 const std::vector<RealT>& fp_svc_cat_revenues, // Monetary revenues by service
                                 const std::vector<RealT>& fp_svc_cat_penalties, // Monetary penalties by service
                                 const RealT fp_electricity_cost, // Electricty cost (in $/Wh) of FP
                                 const std::vector<RealT>& fp_fn_cat_asleep_costs, // Cost to power-off a FN by FN category
                                 const std::vector<RealT>& fp_fn_cat_awake_costs, // Cost to power-on a FN by FN category
                                 RealT deltat = 1 // Length of the time interval
                            ) const
    {
        return do_solve(fn_categories,
                        fn_power_states,
                        fn_vm_allocations,
                        std::set<std::size_t>(), // Any FN can be selected
                        fn_cat_min_powers,
                        fn_cat_max_powers,
                        vm_cat_fn_cat_cpu_specs,
                        //vm_cat_fn_cat_ram_specs,
                        vm_cat_alloc_costs,
                        svc_categories,
                        svc_cat_vm_cat_min_num_vms,
                        fp_svc_cat_revenues,
                        fp_svc_cat_penalties,
                        fp_electricity_cost,
                        fp_fn_cat_asleep_costs,
                        fp_fn_cat_awake_costs,
                        deltat);
    }

    vm_allocation_t<RealT> solve_with_fixed_fns(const std::set<std::size_t>& fixed_fns,
                                                const std::vector<std::size_t>& fn_categories, // Maps every FN to its FN category
                                                const std::vector<bool>& fn_power_states, // The power status of each FN
                                                const std::vector<std::map<std::size_t,std::pair<std::size_t,std::size_t>>>& fn_vm_allocations, // Current VM allocations, by FN and service
                                                const std::vector<RealT>& fn_cat_min_powers, // The min power consumption of FNs by FN category
                                                const std::vector<RealT>& fn_cat_max_powers, // The max power consumption of FNs by FN category
                                                const std::vector<std::vector<RealT>>& vm_cat_fn_cat_cpu_specs, // The CPU requirement of VMs by VM category and FN category
                                                //const std::vector<std::vector<RealT>>& vm_cat_fn_cat_ram_specs, // The RAM requirement of VMs by VM category and FN category
                                                const std::vector<RealT>& vm_cat_alloc_costs, // The cost to allocate a VM on a FN (e.g., cost to boot a VM or to live-migrate its state), by VM category
                                                const std::vector<std::size_t>& svc_categories, // Service categories by service
                                                const std::vector<std::vector<std::size_t>>& svc_cat_vm_cat_min_num_vms, // The min number of VMs required to achieve QoS, by service category and VM category
                                                const std::vector<RealT>& fp_svc_cat_revenues, // Monetary revenues by service
                                                const std::vector<RealT>& fp_svc_cat_penalties, // Monetary penalties by service
                                                const RealT fp_electricity_cost, // Electricty cost (in $/Wh) of FP
                                                const std::vector<RealT>& fp_fn_cat_asleep_costs, // Cost to power-off a FN by FN category
                                                const std::vector<RealT>& fp_fn_cat_awake_costs, // Cost to power-on a FN by FN category
                                                RealT deltat = 1 // Length of the time interval
                            ) const
    {
        return do_solve(fn_categories,
                        fn_power_states,
                        fn_vm_allocations,
                        fixed_fns,
                        fn_cat_min_powers,
                        fn_cat_max_powers,
                        vm_cat_fn_cat_cpu_specs,
                        //vm_cat_fn_cat_ram_specs,
                        vm_cat_alloc_costs,
                        svc_categories,
                        svc_cat_vm_cat_min_num_vms,
                        fp_svc_cat_revenues,
                        fp_svc_cat_penalties,
                        fp_electricity_cost,
                        fp_fn_cat_asleep_costs,
                        fp_fn_cat_awake_costs,
                        deltat);
    }


private:
    vm_allocation_t<RealT> do_solve(const std::vector<std::size_t>& fn_categories, // Maps every FN to its FN category
                                    const std::vector<bool>& fn_power_states, // The power status of each FN
                                    const std::vector<std::map<std::size_t,std::pair<std::size_t,std::size_t>>>& fn_vm_allocations, // Current VM allocations, by FN and service
                                    const std::set<std::size_t>& fixed_fns, // The set of selected FNs to use for the VM allocation (if empty, any FN can be used)
                                    const std::vector<RealT>& fn_cat_min_powers, // The min power consumption of FNs by FN category
                                    const std::vector<RealT>& fn_cat_max_powers, // The max power consumption of FNs by FN category
                                    const std::vector<std::vector<RealT>>& vm_cat_fn_cat_cpu_specs, // The CPU requirement of VMs by VM category and FN category
                                    //const std::vector<std::vector<RealT>>& vm_cat_fn_cat_ram_specs, // The RAM requirement of VMs by VM category and FN category
                                    const std::vector<RealT>& vm_cat_alloc_costs, // The cost to allocate a VM on a FN (e.g., cost to boot a VM or to live-migrate its state), by VM category
                                    const std::vector<std::size_t>& svc_categories, // Service categories by service
                                    const std::vector<std::vector<std::size_t>>& svc_cat_vm_cat_min_num_vms, // The min number of VMs required to achieve QoS, by service category and VM category
                                    const std::vector<RealT>& fp_svc_cat_revenues, // Monetary revenues by service
                                    const std::vector<RealT>& fp_svc_cat_penalties, // Monetary penalties by service
                                    const RealT fp_electricity_cost, // Electricty cost (in $/Wh) of FP
                                    const std::vector<RealT>& fp_fn_cat_asleep_costs, // Cost to power-off a FN by FN category
                                    const std::vector<RealT>& fp_fn_cat_awake_costs, // Cost to power-on a FN by FN category
                                    RealT deltat = 1 // Length of the time interval
                                ) const
    {
        DCS_DEBUG_TRACE("Finding VM allocation by solving the assignment problem:");
        DCS_DEBUG_TRACE("- Number of FNs: " << fn_categories.size());
        DCS_DEBUG_TRACE("- FN Categories: " << fn_categories);
        DCS_DEBUG_TRACE("- FN Power States: " << fn_power_states);
        DCS_DEBUG_TRACE("- FN - VM Allocations: " << fn_vm_allocations);
        DCS_DEBUG_TRACE("- FN Fixed: " << fixed_fns);
        DCS_DEBUG_TRACE("- FN Mininimum Power Consumption by FN Category: " << fn_cat_min_powers);
        DCS_DEBUG_TRACE("- FN Maximum Power Consumption by FN Category: " << fn_cat_max_powers);
        DCS_DEBUG_TRACE("- VM CPU requirements by VM Category and FN Category: " << vm_cat_fn_cat_cpu_specs);
        //DCS_DEBUG_TRACE("- VM RAM requirements by VM Category and FN Category: " << vm_cat_fn_cat_ram_specs);
        DCS_DEBUG_TRACE("- VM Allocation Costs by VM Category: " << vm_cat_alloc_costs);
        DCS_DEBUG_TRACE("- Service of FNs: " << svc_categories.size());
        DCS_DEBUG_TRACE("- Service Categories: " << svc_categories);
        DCS_DEBUG_TRACE("- Service Minimum Number of VMs by Service Category and VM Category: " << svc_cat_vm_cat_min_num_vms);
        DCS_DEBUG_TRACE("- FP Service Revenues by Service Category: " << fp_svc_cat_revenues);
        DCS_DEBUG_TRACE("- FP Service Penalties by Service Category: " << fp_svc_cat_penalties);
        DCS_DEBUG_TRACE("- FP Energy Cost: " << fp_electricity_cost);
        DCS_DEBUG_TRACE("- FN On->Off Cost by FN Category: " << fp_fn_cat_asleep_costs);
        DCS_DEBUG_TRACE("- FN Off->On Cost by FN Category: " << fp_fn_cat_awake_costs);
        DCS_DEBUG_TRACE("- Length of the time interval: " << deltat);

        // Compute the cost of assigning a VM to an FN
        // In the assignment problem, a VM is considered a task and an FN is an agent (or server).
        // Thus the solution of the assignment problem is a mapping of 1 VM to a 1 FN and vice versa (no more than 1 VM is allowed on the same FN).

        auto const nfns = fn_categories.size();
        auto const nsvcs = svc_categories.size();

        DCS_ASSERT( nfns == fn_categories.size(),
                    DCS_EXCEPTION_THROW( std::logic_error,
                                         "FN categories container has a wrong size" ) );

        DCS_ASSERT( nfns == fn_power_states.size(),
                    DCS_EXCEPTION_THROW( std::logic_error,
                                         "FN power states container has a wrong size" ) );

        std::unordered_map<std::size_t, std::size_t> svc_cat_vm_categories; // Map service category to the best VM category to use (where "best" is defined in terms of less used CPU)
        std::vector<std::size_t> vm_categories; // Map a VM to its VM category
        std::vector<std::size_t> vm_services; // Map a VM to its service
        std::size_t nvms = 0;
        std::set<std::size_t> fncat_set;
        for (std::size_t fn = 0; fn < nfns; ++fn)
        {
            auto const fn_cat = fn_categories[fn];

            fncat_set.insert(fn_cat);
        }

        for (std::size_t svc = 0; svc < nsvcs; ++svc)
        {
            auto const svc_cat = svc_categories[svc];

            RealT best_tot_cpu_share = std::numeric_limits<RealT>::infinity();
            //std::size_t best_fn_cat = 0;
            std::size_t best_vm_cat = 0;
            //for (std::size_t fn = 0; fn < nfns; ++fn)
            //{
            //    auto const fn_cat = fn_categories[fn];
            for (auto const fn_cat : fncat_set)
            {

                // Choose the VM category that, to satisfy the service QoS, requires less CPU capacity
                //for (auto const vm_cat : svc_cat_vm_cat_min_num_vms[svc_cat])
                for (std::size_t vm_cat = 0; vm_cat < svc_cat_vm_cat_min_num_vms[svc_cat].size(); ++vm_cat)
                {
                    auto const tot_cpu_share = vm_cat_fn_cat_cpu_specs[vm_cat][fn_cat]*svc_cat_vm_cat_min_num_vms[svc_cat][vm_cat];

DCS_DEBUG_TRACE("Looking for VM CATEGORY - SVC: " << svc << " (cat: " << svc_cat << "), FN CAT: " << fn_cat << ", VM CAT: " << vm_cat << ", NUM: " << svc_cat_vm_cat_min_num_vms[svc_cat][vm_cat] << ", REQ: " << vm_cat_fn_cat_cpu_specs[vm_cat][fn_cat] << " -> TOT SHARE: " << tot_cpu_share << " vs. BEST SHARE: " << best_tot_cpu_share);//XXX
                    if (tot_cpu_share < best_tot_cpu_share)
                    {
                        best_tot_cpu_share = tot_cpu_share;
                        //best_fn_cat = fn_cat;
                        best_vm_cat = vm_cat;
                    }
                }
            }

            svc_cat_vm_categories[svc_cat] = best_vm_cat;

            auto svc_nvms = svc_cat_vm_cat_min_num_vms[svc_cat][best_vm_cat];

            vm_services.insert(vm_services.end(), svc_nvms, svc);
DCS_DEBUG_TRACE("Update VM_SERVICES: " << vm_services);//XXX
            vm_categories.insert(vm_categories.end(), svc_nvms, best_vm_cat);
DCS_DEBUG_TRACE("Update VM_CATEGORIES: " << vm_categories);//XXX
            nvms += svc_nvms;
        }

        std::size_t cur_vm = 0;
        std::vector<std::size_t> virtual_servers_fns;
        std::vector<std::map<std::size_t,RealT>> fn_vm_cat_weights(nfns);
        for (std::size_t fn = 0; fn < nfns; ++fn)
        {
            auto const fn_cat = fn_categories[fn];
            const RealT max_cpu_req = 1;
            RealT tot_cpu_req = 0;

            for (; tot_cpu_req < max_cpu_req && cur_vm < nvms; ++cur_vm)
            {
                DCS_ASSERT( cur_vm < vm_categories.size(),
                            DCS_EXCEPTION_THROW( std::logic_error,
                                                 "VM sequence number is out-of-bound" ) );

                auto const vm_cat = vm_categories[cur_vm];

                auto cpu_req = vm_cat_fn_cat_cpu_specs[vm_cat][fn_cat];
                if ((tot_cpu_req + cpu_req) > max_cpu_req)
                {
                    break;
                }
                tot_cpu_req += cpu_req;
                virtual_servers_fns.push_back(fn);
                fn_vm_cat_weights[fn][vm_cat] = cpu_req;
            }
            for (auto& vm_cat_weight : fn_vm_cat_weights[fn])
            {
                vm_cat_weight.second /= tot_cpu_req;
DCS_DEBUG_TRACE("FN: " << fn << ", TOT SHARE: " << tot_cpu_req << ", VM CAT: " << vm_cat_weight.first << " -> WEIGHT: " << vm_cat_weight.second);//XXX
            }
        }

        auto const nvss = virtual_servers_fns.size();
        std::vector<std::vector<double>> costs; // Cost matrix C, where C_{ij} is the cost of running VM i on FN j
        std::unordered_map<int, int> direct_assignment; // Map each VM to the FN where it has been allocated
        std::unordered_map<int, int> reverse_assignment; // Map each FN to the assigned VM

        // Compute the cost of assignment VM i to FN j as follows:
        //   C[i,j] = <VM-allocation-cost> + <power-on-cost>
        // where:
        // - <VM-allocation-cost> is the electricity cost spent to allocate and run VM i on FN i and is computed as follows:
        //    (W^{min}_j + (W^{max}_j-W^{min}_j)*U_i)*E
        //   with W^{min}_j and W^{max}_j are the min and max power consumption (in Wh) of FN j, U_i is the CPU requirement of VM i, and E is the electricity cost (in $/Wh).
        // - <power-on-cost> is the electricity cost spent to power on FN i and is computer as follows:
        //    W^{max}_j*K^{on}_j*E
        //   with K^{on}_j is the percentage of an hour taken to power on FN j (e.g., if FN j takes 1 minute to power on, K^{on}_j = 60sec/3600sec)
        //
        costs.resize(nvms);
        for (std::size_t vm = 0; vm < nvms; ++vm)
        {
            DCS_ASSERT( vm < vm_categories.size(),
                        DCS_EXCEPTION_THROW( std::logic_error,
                                             "VM sequence number is out-of-bound" ) );

            auto const vm_cat = vm_categories[vm];
            auto const svc = vm_services[vm];

            costs[vm].resize(nvss);
            for (std::size_t vs = 0; vs < nvss; ++vs)
            {
                auto const fn = virtual_servers_fns[vs];
                auto const fn_cat = fn_categories[fn];

                // Add energy consumption costs
                if (fixed_fns.size() > 0 && fixed_fns.count(fn) == 0)
                {
                    // Assign to this FN a very high cost so that it cannot be selected
DCS_DEBUG_TRACE("VS: " << vs << ", FN: " << fn << ", SVC: " << svc << ", VM: " << vm << " -> ADDING INFINITY COSTS: " << std::numeric_limits<double>::max() << " to " << costs[vm][vs]);//XXX
                    costs[vm][vs] = std::numeric_limits<double>::max();
                }
                else
                {
                    //costs[vm][vs] = (fn_cat_max_powers[fn_cat]-fn_cat_min_powers[fn_cat])*vm_cat_fn_cat_cpu_specs[vm_cat][fn_cat]*fp_electricity_cost; // Don't consider idle power consumption (which is added later)
                    //NOTE: to consider different idle power consumptions we add a contribution for each VS so that each VS contributes to idle power consumption according to the required CPU capacity. This contribution must be removed when computing real costs
DCS_DEBUG_TRACE("VS: " << vs << " (FN: " << fn << ", FN CAT: " << fn_cat << "), SVC: " << svc << ", VM: " << vm << " (VM CAT: " << vm_cat << ") -> ADDING POWER CONSUMPTION COSTS: " << ((fn_cat_min_powers[fn_cat]*vm_cat_fn_cat_cpu_specs[vm_cat][fn_cat] + (fn_cat_max_powers[fn_cat]-fn_cat_min_powers[fn_cat])*vm_cat_fn_cat_cpu_specs[vm_cat][fn_cat])*fp_electricity_cost) << " to " << costs[vm][vs]);//XXX
                    costs[vm][vs] = (fn_cat_min_powers[fn_cat]*vm_cat_fn_cat_cpu_specs[vm_cat][fn_cat] + (fn_cat_max_powers[fn_cat]-fn_cat_min_powers[fn_cat])*vm_cat_fn_cat_cpu_specs[vm_cat][fn_cat])*fp_electricity_cost;
                }

                // Add VM allocation costs
                if (fn_vm_allocations[fn].count(svc) == 0 || fn_vm_allocations[fn].at(svc).first != vm_cat)
                {
DCS_DEBUG_TRACE("VS: " << vs << " (FN: " << fn << ", FN CAT: " << fn_cat << "), SVC: " << svc << ", VM: " << vm << " (VM CAT: " << vm_cat << ") -> ADDING ALLOCATION COSTS: " << (vm_cat_alloc_costs[vm_cat]/deltat) << " to " << costs[vm][vs]);//XXX
                    // The VM has not already been allocated to this FN, so add allocation cost
                    costs[vm][vs] += vm_cat_alloc_costs[vm_cat]/deltat;
                }
                // Check if FN must be powered on and if so add the related costs
                if (!fn_power_states[fn])
                {
DCS_DEBUG_TRACE("VS: " << vs << " (FN: " << fn << ", FN CAT: " << fn_cat << "), SVC: " << svc << ", VM: " << vm << " (VM CAT: " << vm_cat << "), CPU REQ: " << vm_cat_fn_cat_cpu_specs[vm_cat][fn_cat] << " -> ADDING AWAKE COSTS: " << (vm_cat_fn_cat_cpu_specs[vm_cat][fn_cat]*fp_fn_cat_awake_costs[fn_cat]/deltat) << " to " << costs[vm][vs]);//XXX
                    costs[vm][vs] += vm_cat_fn_cat_cpu_specs[vm_cat][fn_cat]*fp_fn_cat_awake_costs[fn_cat]/deltat;
                }
            }
        }

DCS_DEBUG_TRACE("COST MATRIX (nvms x nvss : " << nvms << " x " << nvss << "): " << costs);//XXX
        operations_research::MinimizeLinearAssignment(costs, &direct_assignment, &reverse_assignment);

DCS_DEBUG_TRACE("SOLVED");
#ifdef DCS_DEBUG
        DCS_DEBUG_TRACE( "-------------------------------------------------------------------------------[" );
        DCS_DEBUG_TRACE("- Direct assignments:" );
        for (auto const& vm_vs : direct_assignment)
        {
            auto const vm = vm_vs.first;
            auto const vs = vm_vs.second;

            DCS_DEBUG_STREAM << "VM = " << vm << " -> VS = " << vs << " (Cost = " << costs[vm][vs] << ")" << std::endl;
        }
        DCS_DEBUG_TRACE("- Reverse assignments:" );
        for (auto const& vs_vm : reverse_assignment)
        {
            auto const vs = vs_vm.first;
            auto const vm = vs_vm.second;

            DCS_DEBUG_STREAM << "VS = " << vs << " -> VM = " << vm << " (Cost = " << costs[vm][vs] << ")" << std::endl;
        }
        DCS_DEBUG_TRACE( "]-------------------------------------------------------------------------------" );
#endif // DCS_DEBUG

        vm_allocation_t<RealT> solution;

        solution.objective_value = 0;
        solution.solved = true;
        solution.optimal = false;
        solution.fn_vm_allocations.resize(nfns);
        solution.fn_cpu_allocations.resize(nfns, 0);
        //solution.fn_power_states.resize(nfns, false);
        solution.fn_power_states.assign(fn_power_states.begin(), fn_power_states.end());
        solution.revenue = 0;
        solution.cost = 0;
        for (auto const& vm_vs : direct_assignment)
        {
            auto const vm = vm_vs.first;
            auto const vs = vm_vs.second;

            DCS_ASSERT( static_cast<std::size_t>(vm) < vm_categories.size() && static_cast<std::size_t>(vm) < vm_services.size(),
                        DCS_EXCEPTION_THROW( std::logic_error,
                                             "VM sequence number is out-of-bound" ) );
            DCS_ASSERT( static_cast<std::size_t>(vs) < virtual_servers_fns.size(),
                        DCS_EXCEPTION_THROW( std::logic_error,
                                             "Virtual server sequence number is out-of-bound" ) );

            auto const fn = virtual_servers_fns[vs];

            if (fixed_fns.size() > 0 && fixed_fns.count(fn) == 0)
            {
                // Skip this assigment because the involved FN was not selected
                continue;
            }

            auto const fn_cat = fn_categories[fn];
            auto const vm_cat = vm_categories[vm];
            auto const svc = vm_services[vm];
            auto const svc_cat = svc_categories[svc];

            if (solution.fn_vm_allocations[fn].count(svc) > 0)
            {
                solution.fn_vm_allocations[fn][svc].second += 1;
            }
            else
            {
                solution.fn_vm_allocations[fn][svc] = std::make_pair(vm_cat, 1);
            }
            solution.fn_power_states[fn] = true;
            solution.fn_cpu_allocations[fn] += vm_cat_fn_cat_cpu_specs[vm_cat][fn_cat];
            solution.objective_value += - costs[vm][vs];
            //solution.cost += costs[vm][vs];
            // Remove dummy contribution of the VS to idle power consumption (will be readded later)
            solution.cost -= vm_cat_fn_cat_cpu_specs[vm_cat][fn_cat]*fn_cat_min_powers[fn_cat]*fp_electricity_cost;
            // Remove dummy contribution of the VS to switch-on costs (will be readded later)
            solution.cost -= vm_cat_fn_cat_cpu_specs[vm_cat][fn_cat]*fp_fn_cat_awake_costs[fn_cat]/deltat;
            solution.revenue += fp_svc_cat_revenues[svc_cat];
        }
        for (std::size_t fn = 0; fn < nfns; ++fn)
        {
            auto const fn_cat = fn_categories[fn];

            if (fixed_fns.size() > 0 && fixed_fns.count(fn) > 0)
            {
                // Make sure the FN is powered on regardless its assignment
                solution.fn_power_states[fn] = true;
            }
            else if (solution.fn_vm_allocations[fn].size() == 0)
            {
                // Power off this FN because it has not been selected in the optimal assigment
                solution.fn_power_states[fn] = false;
            }

DCS_DEBUG_TRACE("FN: " << fn << " - old power state: " << fn_power_states[fn] << " - new power state: " << solution.fn_power_states[fn]);//XXX
            if (fn_power_states[fn] && !solution.fn_power_states[fn])
            {
                // Add power-off costs
DCS_DEBUG_TRACE("ADDING ASLEEP-COST: " << fp_fn_cat_asleep_costs[fn_cat]/deltat << " to " << solution.cost);
                solution.cost += fp_fn_cat_asleep_costs[fn_cat]/deltat;
            }
            else if (solution.fn_power_states[fn])
            {
                if (!fn_power_states[fn])
                {
                    // Add power-on costs
DCS_DEBUG_TRACE("ADDING AWAKE-COST: " << fp_fn_cat_awake_costs[fn_cat]/deltat << " to " << solution.cost);
                    solution.cost += fp_fn_cat_awake_costs[fn_cat]/deltat;
                }

                // Add idle power
                solution.cost += fn_cat_min_powers[fn_cat]*fp_electricity_cost;
            }
        }
        std::set<std::size_t> svc_with_penalty;
        for (std::size_t vm = 0; vm < nvms; ++vm)
        {
            // Add penalty cost if either a VM needed by a service has not been assignment or if it has been assigned to a FN that has not been selected
            if (direct_assignment.count(vm) == 0
                || (fixed_fns.size() > 0 && fixed_fns.count(virtual_servers_fns[direct_assignment.at(vm)]) == 0))
            {
                auto const svc = vm_services[vm];

DCS_DEBUG_TRACE("VM: " << vm << " for SVC: " << svc << " should have been mapped but it's not");
                svc_with_penalty.insert(svc);
            }
        }
        for (auto const svc : svc_with_penalty)
        {
            auto const svc_cat = svc_categories[svc];

DCS_DEBUG_TRACE("SVC: " << svc << " - ADDING PENALTY COST: " << fp_svc_cat_penalties[svc_cat] << " to " << solution.cost);
            solution.cost += fp_svc_cat_penalties[svc_cat];
        }
        //solution.objective_value = solution.revenue - solution.cost;
        solution.revenue *= deltat;
        solution.cost *= deltat;
        solution.profit = solution.revenue-solution.cost;
DCS_DEBUG_TRACE("Final VM Allocation: " << solution.fn_vm_allocations);//XXX

        return solution;
    }
}; // bahreini2017_mcappim_alt_vm_allocation_solver_t


}} // Namespace dcs::fog


#endif // DCS_FOG_VM_ALLOCATION_BAHREINE2017_MCAPP_HPP
