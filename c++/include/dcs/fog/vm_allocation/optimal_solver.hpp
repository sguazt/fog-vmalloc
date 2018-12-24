/* vim: set tabstop=4 expandtab shiftwidth=4 softtabstop=4: */

/**
 * \file dcs/fog/vm_allocation_solvers.hpp
 *
 * \brief Solvers for the VM allocation problem
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

#ifndef DCS_FOG_VM_ALLOCATION_OPTIMAL_SOLVER_HPP
#define DCS_FOG_VM_ALLOCATION_OPTIMAL_SOLVER_HPP


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
#include <dcs/math/function/round.hpp>
#include <dcs/math/traits/float.hpp>
#include <ilconcert/iloalg.h>
#include <ilconcert/iloenv.h>
#include <ilconcert/iloexpression.h>
#include <ilconcert/ilomodel.h>
#include <ilcp/cp.h>
#include <ilcplex/ilocplex.h>
#include <iostream>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <vector>


/*
 * Control the way we define s_j and s_{t,j} in the CP optimal solver
 * - true: use decision variables
 * - false: use decision expressions
 * .
 * NOTE: CP documentation claims that using a variable instead of an expression
 *       is in general more efficient.
 */
#define DCS_FOG_VM_ALLOC_CP_USE_S_AS_DECISION_VARIABLE true

#if !defined(DCS_FOG_VM_ALLOC_USE_CPLEX_SOLVER) && !defined(DCS_FOG_VM_ALLOC_USE_CP_SOLVER)
# define DCS_FOG_VM_ALLOC_USE_CPLEX_SOLVER true
#endif // DCS_FOG_VM_ALLOC_USE_..._SOLVER

/*
 * This macro controls the way we must react to an anomaly in the optimizer's results.
 * If defined, an anomaly will cause the program to abort its execution.
 * If not defined, an anomaly will just print a warning message.
 */ 
//#define DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY

namespace dcs { namespace fog {

namespace detail {

void dump_cplex_settings(const IloCplex& cplex)
{
#ifdef DCS_DEBUG
    // Dump optimizer parameters
    DCS_DEBUG_STREAM << "--- Cplex Optimizer Settings:" << std::endl;
    DCS_DEBUG_STREAM << "- Advance: " << cplex.getParam(IloCplex::Param::Advance) << " (default: " << cplex.getDefault(IloCplex::Param::Advance) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- Barrier::Algorithm: " << cplex.getParam(IloCplex::Param::Barrier::Algorithm) << " (default: " << cplex.getDefault(IloCplex::Param::Barrier::Algorithm) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- Barrier::ColNonzeros: " << cplex.getParam(IloCplex::Param::Barrier::ColNonzeros) << " (default: " << cplex.getDefault(IloCplex::Param::Barrier::ColNonzeros) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- Barrier::ConvergeTol: " << cplex.getParam(IloCplex::Param::Barrier::ConvergeTol) << " (default: " << cplex.getDefault(IloCplex::Param::Barrier::ConvergeTol) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- Barrier::Crossover: " << cplex.getParam(IloCplex::Param::Barrier::Crossover) << " (default: " << cplex.getDefault(IloCplex::Param::Barrier::Crossover) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- Barrier::Display: " << cplex.getParam(IloCplex::Param::Barrier::Display) << " (default: " << cplex.getDefault(IloCplex::Param::Barrier::Display) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- Barrier::Limits::Corrections: " << cplex.getParam(IloCplex::Param::Barrier::Limits::Corrections) << " (default: " << cplex.getDefault(IloCplex::Param::Barrier::Limits::Corrections) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- Barrier::Limits::Growth: " << cplex.getParam(IloCplex::Param::Barrier::Limits::Growth) << " (default: " << cplex.getDefault(IloCplex::Param::Barrier::Limits::Growth) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- Barrier::Limits::Iteration: " << cplex.getParam(IloCplex::Param::Barrier::Limits::Iteration) << " (default: " << cplex.getDefault(IloCplex::Param::Barrier::Limits::Iteration) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- Barrier::Limits::ObjRange: " << cplex.getParam(IloCplex::Param::Barrier::Limits::ObjRange) << " (default: " << cplex.getDefault(IloCplex::Param::Barrier::Limits::ObjRange) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- Barrier::Ordering: " << cplex.getParam(IloCplex::Param::Barrier::Ordering) << " (default: " << cplex.getDefault(IloCplex::Param::Barrier::Ordering) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- Barrier::QCPConvergeTol: " << cplex.getParam(IloCplex::Param::Barrier::QCPConvergeTol) << " (default: " << cplex.getDefault(IloCplex::Param::Barrier::QCPConvergeTol) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- Barrier::StartAlg: " << cplex.getParam(IloCplex::Param::Barrier::StartAlg) << " (default: " << cplex.getDefault(IloCplex::Param::Barrier::StartAlg) << ")" << std::endl;
//TODO: https://www.ibm.com/support/knowledgecenter/SSSA5P_12.8.0/ilog.odms.cplex.help/CPLEX/Parameters/topics/BendersStrategy.html and subsequent params
    DCS_DEBUG_STREAM << "- Emphasis::Memory: " << cplex.getParam(IloCplex::Param::Emphasis::Memory) << " (default: " << cplex.getDefault(IloCplex::Param::Emphasis::Memory) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- MIP::Limits::AggForCut: " << cplex.getParam(IloCplex::Param::MIP::Limits::AggForCut) << " (default: " << cplex.getDefault(IloCplex::Param::MIP::Limits::AggForCut) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- Preprocessing::Aggregator: " << cplex.getParam(IloCplex::Param::Preprocessing::Aggregator) << " (default: " << cplex.getDefault(IloCplex::Param::Preprocessing::Aggregator) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- Preprocessing::Fill: " << cplex.getParam(IloCplex::Param::Preprocessing::Fill) << " (default: " << cplex.getDefault(IloCplex::Param::Preprocessing::Fill) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- Read::APIEncoding: " << cplex.getParam(IloCplex::Param::Read::APIEncoding) << " (default: " << cplex.getDefault(IloCplex::Param::Read::APIEncoding) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- WorkDir: " << cplex.getParam(IloCplex::Param::WorkDir) << " (default: " << cplex.getDefault(IloCplex::Param::WorkDir) << ")" << std::endl;
#else
    DCS_MACRO_SUPPRESS_UNUSED_VARIABLE_WARNING( cplex );
#endif // DCS_DEBUG
}

void dump_cp_settings(const IloCP& cp)
{
#ifdef DCS_DEBUG
    // Dump optimizer parameters
    DCS_DEBUG_STREAM << "--- CP Optimizer Settings:" << std::endl;
    DCS_DEBUG_STREAM << "- AllDiffInferenceLevel: " << cp.getParameter(IloCP::AllDiffInferenceLevel) << " (default: " << cp.getParameterDefault(IloCP::AllDiffInferenceLevel) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- AllMinDistanceInferenceLevel: " << cp.getParameter(IloCP::AllMinDistanceInferenceLevel) << " (default: " << cp.getParameterDefault(IloCP::AllMinDistanceInferenceLevel) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- AutomaticReplay: " << cp.getParameter(IloCP::AutomaticReplay) << " (default: " << cp.getParameterDefault(IloCP::AutomaticReplay) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- BranchLimit: " << cp.getParameter(IloCP::BranchLimit) << " (default: " << cp.getParameterDefault(IloCP::BranchLimit) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- ChoicePointLimit: " << cp.getParameter(IloCP::ChoicePointLimit) << " (default: " << cp.getParameterDefault(IloCP::ChoicePointLimit) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- ConflictRefinerBranchLimit: " << cp.getParameter(IloCP::ConflictRefinerBranchLimit) << " (default: " << cp.getParameterDefault(IloCP::ConflictRefinerBranchLimit) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- ConflictRefinerFailLimit: " << cp.getParameter(IloCP::ConflictRefinerFailLimit) << " (default: " << cp.getParameterDefault(IloCP::ConflictRefinerFailLimit) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- ConflictRefinerIterationLimit: " << cp.getParameter(IloCP::ConflictRefinerIterationLimit) << " (default: " << cp.getParameterDefault(IloCP::ConflictRefinerIterationLimit) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- ConflictRefinerOnVariables: " << cp.getParameter(IloCP::ConflictRefinerOnVariables) << " (default: " << cp.getParameterDefault(IloCP::ConflictRefinerOnVariables) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- ConflictRefinerTimeLimit: " << cp.getParameter(IloCP::ConflictRefinerTimeLimit) << " (default: " << cp.getParameterDefault(IloCP::ConflictRefinerTimeLimit) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- CountDifferentInferenceLevel: " << cp.getParameter(IloCP::CountDifferentInferenceLevel) << " (default: " << cp.getParameterDefault(IloCP::CountDifferentInferenceLevel) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- CountInferenceLevel: " << cp.getParameter(IloCP::CountInferenceLevel) << " (default: " << cp.getParameterDefault(IloCP::CountInferenceLevel) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- CumulFunctionInferenceLevel: " << cp.getParameter(IloCP::CumulFunctionInferenceLevel) << " (default: " << cp.getParameterDefault(IloCP::CumulFunctionInferenceLevel) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- DefaultInferenceLevel: " << cp.getParameter(IloCP::DefaultInferenceLevel) << " (default: " << cp.getParameterDefault(IloCP::DefaultInferenceLevel) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- DistributeInferenceLevel: " << cp.getParameter(IloCP::DistributeInferenceLevel) << " (default: " << cp.getParameterDefault(IloCP::DistributeInferenceLevel) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- DynamicProbing: " << cp.getParameter(IloCP::DynamicProbing) << " (default: " << cp.getParameterDefault(IloCP::DynamicProbing) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- DynamicProbingStrength: " << cp.getParameter(IloCP::DynamicProbingStrength) << " (default: " << cp.getParameterDefault(IloCP::DynamicProbingStrength) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- ElementInferenceLevel: " << cp.getParameter(IloCP::ElementInferenceLevel) << " (default: " << cp.getParameterDefault(IloCP::ElementInferenceLevel) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- FailLimit: " << cp.getParameter(IloCP::FailLimit) << " (default: " << cp.getParameterDefault(IloCP::FailLimit) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- FailureDirectedSearch: " << cp.getParameter(IloCP::FailureDirectedSearch) << " (default: " << cp.getParameterDefault(IloCP::FailureDirectedSearch) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- FailureDirectedSearchEmphasis: " << cp.getParameter(IloCP::FailureDirectedSearchEmphasis) << " (default: " << cp.getParameterDefault(IloCP::FailureDirectedSearchEmphasis) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- FailureDirectedSearchMaxMemory: " << cp.getParameter(IloCP::FailureDirectedSearchMaxMemory) << " (default: " << cp.getParameterDefault(IloCP::FailureDirectedSearchMaxMemory) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- IntervalSequenceInferenceLevel: " << cp.getParameter(IloCP::IntervalSequenceInferenceLevel) << " (default: " << cp.getParameterDefault(IloCP::IntervalSequenceInferenceLevel) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- LogPeriod: " << cp.getParameter(IloCP::LogPeriod) << " (default: " << cp.getParameterDefault(IloCP::LogPeriod) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- LogSearchTags: " << cp.getParameter(IloCP::LogSearchTags) << " (default: " << cp.getParameterDefault(IloCP::LogSearchTags) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- LogVerbosity: " << cp.getParameter(IloCP::LogVerbosity) << " (default: " << cp.getParameterDefault(IloCP::LogVerbosity) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- ModelAnonymizer: " << cp.getParameter(IloCP::ModelAnonymizer) << " (default: " << cp.getParameterDefault(IloCP::ModelAnonymizer) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- MultiPointNumberOfSearchPoints: " << cp.getParameter(IloCP::MultiPointNumberOfSearchPoints) << " (default: " << cp.getParameterDefault(IloCP::MultiPointNumberOfSearchPoints) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- NoOverlapInferenceLevel: " << cp.getParameter(IloCP::NoOverlapInferenceLevel) << " (default: " << cp.getParameterDefault(IloCP::NoOverlapInferenceLevel) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- OptimalityTolerance: " << cp.getParameter(IloCP::OptimalityTolerance) << " (default: " << cp.getParameterDefault(IloCP::OptimalityTolerance) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- PrecedenceInferenceLevel: " << cp.getParameter(IloCP::PrecedenceInferenceLevel) << " (default: " << cp.getParameterDefault(IloCP::PrecedenceInferenceLevel) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- Presolve: " << cp.getParameter(IloCP::Presolve) << " (default: " << cp.getParameterDefault(IloCP::Presolve) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- PrintModelDetailsInMessages: " << cp.getParameter(IloCP::PrintModelDetailsInMessages) << " (default: " << cp.getParameterDefault(IloCP::PrintModelDetailsInMessages) << ")" << std::endl;
#if 0 // Deprecated since 12.6.0 and removed in 12.8.0
    DCS_DEBUG_STREAM << "- PropagationLog: " << cp.getParameter(IloCP::PropagationLog) << " (default: " << cp.getParameterDefault(IloCP::PropagationLog) << ")" << std::endl;
#endif
    DCS_DEBUG_STREAM << "- RandomSeed: " << cp.getParameter(IloCP::RandomSeed) << " (default: " << cp.getParameterDefault(IloCP::RandomSeed) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- RelativeOptimalityTolerance: " << cp.getParameter(IloCP::RelativeOptimalityTolerance) << " (default: " << cp.getParameterDefault(IloCP::RelativeOptimalityTolerance) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- RestartFailLimit: " << cp.getParameter(IloCP::RestartFailLimit) << " (default: " << cp.getParameterDefault(IloCP::RestartFailLimit) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- RestartGrowthFactor: " << cp.getParameter(IloCP::RestartGrowthFactor) << " (default: " << cp.getParameterDefault(IloCP::RestartGrowthFactor) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- SearchType: " << cp.getParameter(IloCP::SearchType) << " (default: " << cp.getParameterDefault(IloCP::SearchType) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- SequenceInferenceLevel: " << cp.getParameter(IloCP::SequenceInferenceLevel) << " (default: " << cp.getParameterDefault(IloCP::SequenceInferenceLevel) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- SolutionLimit: " << cp.getParameter(IloCP::SolutionLimit) << " (default: " << cp.getParameterDefault(IloCP::SolutionLimit) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- StateFunctionInferenceLevel: " << cp.getParameter(IloCP::StateFunctionInferenceLevel) << " (default: " << cp.getParameterDefault(IloCP::StateFunctionInferenceLevel) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- TemporalRelaxation: " << cp.getParameter(IloCP::TemporalRelaxation) << " (default: " << cp.getParameterDefault(IloCP::TemporalRelaxation) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- TimeLimit: " << cp.getParameter(IloCP::TimeLimit) << " (default: " << cp.getParameterDefault(IloCP::TimeLimit) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- TimeMode: " << cp.getParameter(IloCP::TimeMode) << " (default: " << cp.getParameterDefault(IloCP::TimeMode) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- UseFileLocations: " << cp.getParameter(IloCP::UseFileLocations) << " (default: " << cp.getParameterDefault(IloCP::UseFileLocations) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- WarningLevel: " << cp.getParameter(IloCP::WarningLevel) << " (default: " << cp.getParameterDefault(IloCP::WarningLevel) << ")" << std::endl;
    DCS_DEBUG_STREAM << "- Workers: " << cp.getParameter(IloCP::Workers) << " (default: " << cp.getParameterDefault(IloCP::Workers) << ")" << std::endl;
    DCS_DEBUG_STREAM << "---------------------------------------------------------" << std::endl;
#else
    DCS_MACRO_SUPPRESS_UNUSED_VARIABLE_WARNING( cp );
#endif // DCS_DEBUG
}

IloBool to_IloBool(const IloNum& x)
{
    return static_cast<IloBool>(std::round(x));
}

IloInt to_IloInt(const IloNum& x)
{
    return static_cast<IloInt>(std::round(x));
}

//bool cplex_int_eq(const IloCplex& cplex, const IloNum& x, const IloNum& y)
//{
//    return dcs::math::float_traits<IloNum>::approximately_equal(x, y, cplex.getParam(IloCplex::Param::MIP::Tolerances::Integrality));
//}

} // Namespace detail


/**
 * \brief Optimal solver for the VM allocation problem.
 */
template <typename RealT>
class optimal_vm_allocation_solver_t: public base_vm_allocation_solver_t<RealT>
{
public:
    explicit optimal_vm_allocation_solver_t(RealT relative_tolerance = 0,
                                            RealT time_limit = -1)
    : rel_tol_(relative_tolerance),
      time_lim_(time_limit)
    {
    }

    void relative_tolerance(RealT value)
    {
        rel_tol_ = value;
    }

    RealT relative_tolerance() const
    {
        return rel_tol_;
    }

    void time_limit(RealT value)
    {
        time_lim_ = value;
    }

    RealT time_limit() const
    {
        return time_lim_;
    }

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
                                 RealT deltat = 1) const // Length of the time interval
    {
        DCS_DEBUG_TRACE("Finding optimal VM allocation:");
        DCS_DEBUG_TRACE("- Number of FNs: " << fn_categories.size());
        DCS_DEBUG_TRACE("- FN Categories: " << fn_categories);
        DCS_DEBUG_TRACE("- FN Power States: " << fn_power_states);
        DCS_DEBUG_TRACE("- FN - VM Allocations: " << fn_vm_allocations);
        DCS_DEBUG_TRACE("- FN Mininimum Power Consumption by FN Category: " << fn_cat_min_powers);
        DCS_DEBUG_TRACE("- FN Maximum Power Consumption by FN Category: " << fn_cat_max_powers);
        DCS_DEBUG_TRACE("- VM CPU requirements by VM Category and FN Category: " << vm_cat_fn_cat_cpu_specs);
        //DCS_DEBUG_TRACE("- VM RAM requirements by VM Category and FN Category: " << vm_cat_fn_cat_ram_specs);
        DCS_DEBUG_TRACE("- VM Allocation Costs by VM Category: " << vm_cat_alloc_costs);
        DCS_DEBUG_TRACE("- Number of Services: " << svc_categories.size());
        DCS_DEBUG_TRACE("- Service Categories: " << svc_categories);
        DCS_DEBUG_TRACE("- Service Minimum Number of VMs by Service Category and VM Category: " << svc_cat_vm_cat_min_num_vms);
        DCS_DEBUG_TRACE("- FP Service Revenues by Service Category: " << fp_svc_cat_revenues);
        DCS_DEBUG_TRACE("- FP Service Penalties by Service Category: " << fp_svc_cat_penalties);
        DCS_DEBUG_TRACE("- FP Energy Cost: " << fp_electricity_cost);
        DCS_DEBUG_TRACE("- FN On->Off Cost by FN Category: " << fp_fn_cat_asleep_costs);
        DCS_DEBUG_TRACE("- FN Off->On Cost by FN Category: " << fp_fn_cat_awake_costs);
        DCS_DEBUG_TRACE("- Length of the time interval: " << deltat);
        DCS_DEBUG_TRACE("- Relative Tolerance: " << rel_tol_);
        DCS_DEBUG_TRACE("- Time Limit: " << time_lim_);

#if defined(DCS_FOG_VM_ALLOC_USE_CPLEX_SOLVER)
        return by_native_cplex(fn_categories,
                               fn_power_states,
                               fn_vm_allocations,
                               std::set<std::size_t>(), // Use any FN
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
#elif defined(DCS_FOG_VM_ALLOC_USE_CP_SOLVER)
        return by_native_cp(fn_categories,
                            fn_power_states,
                            fn_vm_allocations,
                            std::set<std::size_t>(), // Use any FN
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
#else
# error Unable to find a suitable solver for the VM allocation problem
#endif // DCS_FOG_VM_ALLOC_USE_..._SOLVER
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
                                                RealT deltat = 1) const // Length of the time interval
    {
        DCS_DEBUG_TRACE("Finding optimal VM allocation:");
        DCS_DEBUG_TRACE("- Number of FNs: " << fn_categories.size());
        DCS_DEBUG_TRACE("- FN Categories: " << fn_categories);
        DCS_DEBUG_TRACE("- FN Power States: " << fn_power_states);
        DCS_DEBUG_TRACE("- FN - VM Allocations: " << fn_vm_allocations);
        DCS_DEBUG_TRACE("- FN Mininimum Power Consumption by FN Category: " << fn_cat_min_powers);
        DCS_DEBUG_TRACE("- FN Maximum Power Consumption by FN Category: " << fn_cat_max_powers);
        DCS_DEBUG_TRACE("- VM CPU requirements by VM Category and FN Category: " << vm_cat_fn_cat_cpu_specs);
        //DCS_DEBUG_TRACE("- VM RAM requirements by VM Category and FN Category: " << vm_cat_fn_cat_ram_specs);
        DCS_DEBUG_TRACE("- VM Allocation Costs by VM Category: " << vm_cat_alloc_costs);
        DCS_DEBUG_TRACE("- Number of Services: " << svc_categories.size());
        DCS_DEBUG_TRACE("- Service Categories: " << svc_categories);
        DCS_DEBUG_TRACE("- Service Minimum Number of VMs by Service Category and VM Category: " << svc_cat_vm_cat_min_num_vms);
        DCS_DEBUG_TRACE("- FP Service Revenues by Service Category: " << fp_svc_cat_revenues);
        DCS_DEBUG_TRACE("- FP Service Penalties by Service Category: " << fp_svc_cat_penalties);
        DCS_DEBUG_TRACE("- FP Energy Cost: " << fp_electricity_cost);
        DCS_DEBUG_TRACE("- FN On->Off Cost by FN Category: " << fp_fn_cat_asleep_costs);
        DCS_DEBUG_TRACE("- FN Off->On Cost by FN Category: " << fp_fn_cat_awake_costs);
        DCS_DEBUG_TRACE("- Length of the time interval: " << deltat);
        DCS_DEBUG_TRACE("- Relative Tolerance: " << rel_tol_);
        DCS_DEBUG_TRACE("- Time Limit: " << time_lim_);

#if defined(DCS_FOG_VM_ALLOC_USE_CPLEX_SOLVER)
        return by_native_cplex(fn_categories,
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
#elif defined(DCS_FOG_VM_ALLOC_USE_CP_SOLVER)
        return by_native_cp(fn_categories,
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
#else
# error Unable to find a suitable solver for the VM allocation problem
#endif // DCS_FOG_VM_ALLOC_USE_..._SOLVER
    }


private:
    vm_allocation_t<RealT> by_native_cp(const std::vector<std::size_t>& fn_categories, // Maps every FN to its FN category
                                        const std::vector<bool>& fn_power_states, // The power status of each FN
                                        const std::vector<std::map<std::size_t,std::pair<std::size_t,std::size_t>>>& fn_vm_allocations, // Current VM allocations, by FN and service
                                        const std::set<std::size_t>& fixed_fns, // The set of selected FNs to use in the VM allocation (if empty, any FN may be used)
                                        const std::vector<RealT>& fn_cat_min_powers, // The min power consumption of FNs by FN category
                                        const std::vector<RealT>& fn_cat_max_powers, // The max power consumption of FNs by FN category
                                        const std::vector<std::vector<RealT>>& vm_cat_fn_cat_cpu_specs, // The CPU requirement of VMs by VM category and FN category
                                        //const std::vector<std::vector<RealT>>& vm_cat_fn_cat_ram_specs, // The RAM requirement of VMs by VM category and FN category
                                        const std::vector<RealT>& vm_cat_alloc_costs, // The cost to allocate a VM on a FN (e.g., cost to boot a VM or to live-migrate its state), by VM category
                                        const std::vector<std::size_t>& svc_categories, // Maps every service to its service category
                                        const std::vector<std::vector<std::size_t>>& svc_cat_vm_cat_min_num_vms, // The min number of VMs required to achieve QoS, by service category and VM category
                                        const std::vector<RealT>& fp_svc_cat_revenues, // Monetary revenues by service category
                                        const std::vector<RealT>& fp_svc_cat_penalties, // Monetary penalties by service category
                                        const RealT fp_electricity_cost, // Electricty cost (in $/Wh) of FP
                                        const std::vector<RealT>& fp_fn_cat_asleep_costs, // Cost to power-off a FN by FN category
                                        const std::vector<RealT>& fp_fn_cat_awake_costs, // Cost to power-on a FN by FN category
                                        RealT deltat) const // Length of the time interval
    {
        vm_allocation_t<RealT> solution;

        const std::size_t nfns = fn_categories.size();
        const std::size_t nsvcs = svc_categories.size();
        const std::size_t nvmcats = vm_cat_fn_cat_cpu_specs.size();

        DCS_ASSERT( nfns == fn_categories.size(),
                    DCS_EXCEPTION_THROW( std::logic_error,
                                         "FN categories container has a wrong size" ) );

        DCS_ASSERT( nfns == fn_power_states.size(),
                    DCS_EXCEPTION_THROW( std::logic_error,
                                         "FN power states container has a wrong size" ) );

        DCS_ASSERT( nvmcats == vm_cat_fn_cat_cpu_specs.size(),
                    DCS_EXCEPTION_THROW( std::logic_error,
                                         "VM CPU specs has a wrong size" ) );

        //DCS_ASSERT( nvmcats == vm_cat_fn_cat_ram_specs.size(),
        //            DCS_EXCEPTION_THROW( std::logic_error,
        //                                 "VM RAM specs has a wrong size" ) );

        // Setting up the optimization model
        try
        {
            // Initialize the Concert Technology app
            IloEnv env;

            IloModel model(env);

            model.setName("Max-Profit Optimization");

            // Decision Variables

            // Variables x_i \in \{0,1\}: 1 if FN i is to be powered on, 0 otherwise.
            IloBoolVarArray x(env, nfns);
            for (std::size_t i = 0; i < nfns; ++i)
            {
                std::ostringstream oss;
                oss << "x[" << i << "]";
                x[i] = IloBoolVar(env, oss.str().c_str());
                model.add(x[i]);
            }

            // Variables y_{ijk} \in \mathbb{N}: denotes the number of class-k VMs are allocated on FN i for service j.
            IloArray<IloArray<IloIntVarArray>> y(env, nfns);
            for (std::size_t i = 0; i < nfns; ++i)
            {
                y[i] = IloArray<IloIntVarArray>(env, nsvcs);

                for (std::size_t j = 0 ; j < nsvcs ; ++j)
                {
                    y[i][j] = IloIntVarArray(env, nvmcats);

                    for (std::size_t k = 0 ; k < nvmcats ; ++k)
                    {
                        std::ostringstream oss;
                        oss << "y[" << i << "][" << j << "][" << k << "]";
                        y[i][j][k] = IloIntVar(env, oss.str().c_str());
                        model.add(y[i][j][k]);
                    }
                }
            }

#if DCS_FOG_VM_ALLOC_CP_USE_S_AS_DECISION_VARIABLE
            // Auxiliary variables s_j \in \{0,1\}: 1 if service j is offered, 0
            // otherwise:
            //   s_j = \exists i \in F, k \in C : y_{i,j,k} > 0
            // which can be defined in CP as:
            //   s_j = \sum_{i \in F} \sum_{k \in C} (y_{i,j,k} > 0)
            IloBoolVarArray s(env, nsvcs);
            for (std::size_t j = 0; j < nsvcs; ++j)
            {
                std::ostringstream oss;
                oss << "s[" << j << "]";
                s[j] = IloBoolVar(env, oss.str().c_str());
                model.add(s[j]);
            }
#endif // DCS_FOG_VM_ALLOC_CP_USE_S_AS_DECISION_VARIABLE

            // Decision expressions

            // Expression u_i \in [0,1]: total fraction of CPU of FN i allocated to VMs
            //   u_i = \sum_{j \in S} \sum_{k \in C} y_{i,j,k}*C_{i,k}, \forall i \in FN'
            IloNumExprArray u(env, nfns);
            for (std::size_t i = 0; i < nfns; ++i)
            {
                auto const fn_cat = fn_categories[i];

                u[i] = IloNumExpr(env);

                for (std::size_t j = 0; j < nsvcs; ++j)
                {
                    for (std::size_t k = 0; k < nvmcats; ++k)
                    {
                        u[i] += y[i][j][k]*vm_cat_fn_cat_cpu_specs[k][fn_cat];
                    }
                }

                std::ostringstream oss;
                oss << "u[" << i << "]";
                u[i].setName(oss.str().c_str());

                model.add(u[i]);
            }

#if !DCS_FOG_VM_ALLOC_CP_USE_S_AS_DECISION_VARIABLE
            // Expression s_i \in \{0,1\}:
            //   s_j = \exists i \in F, k \in C : y_{i,j,k} > 0
            // which can be defined in CP as:
            //   s_j = ( \sum_{i \in F} \sum_{k \in C} y_{i,j,k} ) > 0
            IloArray<IloIntExpr> s(env, nsvcs);
            for (std::size_t j = 0; j < nsvc; ++j)
            {
                //s[j] = IloIntExpr(env);
                IloIntExpr rhs_expr(env);

                for (std::size_t i = 0; i < nfns; ++i)
                {
/*
                    for (std::size_t k = 0; k < nvmcats; ++k)
                    {
                        //s[j] += (y[i][j][k] > 0);
                        s[j] += y[i][j][k];
                    }
*/
                    rhs_expr += IloSum(y[i][j]);
                }

                s[j] = rhs_expr > 0;

                std::ostringstream oss;
                oss << "s[" << j << "]";
                s[j].setName(oss.str().c_str());

                model.add(s[j]);
            }
#endif // !DCS_FOG_VM_ALLOC_CP_USE_S_AS_DECISION_VARIABLE

            // Constraints

            std::size_t cc = 0; // Constraint counter

#if DCS_FOG_VM_ALLOC_CP_USE_S_AS_DECISION_VARIABLE
            // Constraints the values of s_j:
            //   s_j = \exists i \in F, k \in C : y_{i,j,k} > 0
            // which can be defined in CP as:
            //  \forall j \in S: s_j = ( \sum_{i \in F} \sum_{k \in C} y_{i,j,k} ) > 0
            ++cc;
            for (std::size_t j = 0; j < nsvcs; ++j)
            {
                IloIntExpr rhs_expr(env);
                for (std::size_t i = 0; i < nfns; ++i)
                {
                    rhs_expr += IloSum(y[i][j]);
/*
                    for (std::size_t k = 0; k < nvmcats; ++k)
                    {
                        rhs_expr += y[i][j][k];
                    }
*/
                }
//                rhs_expr = rhs_expr > 0;

                std::ostringstream oss;
                oss << "C" << cc << "_{" << j << "}";

                IloConstraint cons(s[j] == (rhs_expr > 0));
                cons.setName(oss.str().c_str());
                model.add(cons);
            }
#endif // DCS_FOG_VM_ALLOC_CP_USE_S_AS_DECISION_VARIABLE

            // VMs allocated for a given service must belong to the same class:
            //  \forall j \in S, k_1 \in C : \nexists k2 \in C, k2 \ne k1 : ( \sum_{i \in F} y_{i,j,k_1} ) > 0 \wedge ( \sum_{i \in F} y_{i,j,k_2} ) > 0
            // which can be defined in CP as:
#if 0
            // Alternative #1:
            //  \forall j \in S: \sum_{i \in F} \sum_{k \in C} y_{i,j,k} = max_{k \in C} \sum_{i \in F} y_{i,j,k}
            ++cc;
            for (std::size_t j = 0; j < nsvcs; ++j)
            {
                IloIntExpr lhs_expr(env); // \sum_{i \in F} \sum_{k \in C} y_{i,j,k}
                for (std::size_t i = 0; i < nfns; ++i)
                {
/*
                    for (std::size_t k = 0; k < nvmcats; ++k)
                    {
                        lhs_expr += y[i][j][k];
                    }
*/
                    lhs_expr += IloSum(y[i][j]);
                }

                IloIntExprArray max_op_expr(env, nvmcats); // max_{k \in C} \sum_{i \in F} y_{i,j,k}
                for (std::size_t k = 0; k < nvmcats; ++k)
                {
                    max_op_expr[k] = IloIntExpr(env);
                    for (std::size_t i = 0; i < nfns; ++i)
                    {
                        max_op_expr[k] += y[i][j][k]; 
                    }
                }

                std::ostringstream oss;
                oss << "C" << cc << "_{" << j << "}";

                IloConstraint cons(lhs_expr == IloMax(max_op_expr));
                cons.setName(oss.str().c_str());
                model.add(cons);
            }
#else
            // Alternative #2:
            //  \forall j \in S, \forall k_1 \in C : ( ((\sum_{i \in F} y_{i,j,k_1}) > 0) \cdot \sum_{k_2 \in C, k_2 > k_1} ((\sum_{i \in F} y_{i,j,k_2}) > 0) == 0 )
            ++cc;
            for (std::size_t j = 0; j < nsvcs; ++j)
            {
                IloIntExpr lhs_expr(env);
                for (std::size_t k1 = 0; k1 < nvmcats; ++k1)
                {
                    IloIntExpr ysum1_expr(env);
                    for (std::size_t i = 0; i < nfns; ++i)
                    {
                        ysum1_expr += y[i][j][k1];
                    }
                    for (std::size_t k2 = k1+1; k2 < nvmcats; ++k2)
                    {
                        IloIntExpr ysum2_expr(env);
                        for (std::size_t i = 0; i < nfns; ++i)
                        {
                            ysum2_expr += y[i][j][k2];
                        }
                        lhs_expr += (ysum1_expr > 0)*(ysum2_expr > 0);
                    }
                }

                std::ostringstream oss;
                oss << "C" << cc << "_{" << j << "}";

                IloConstraint cons(lhs_expr == 0);
                cons.setName(oss.str().c_str());
                model.add(cons);
            }
#endif

            // The total allocated capacity on a powered-on FN must not exceed the max capacity
            //  \forall i \in F: u_i \le x_i
            ++cc;
            for (std::size_t i = 0; i < nfns; ++i)
            {
                std::ostringstream oss;
                oss << "C" << cc << "_{" << i << "}";

                IloConstraint cons(u[i] <= x[i]);
                cons.setName(oss.str().c_str());
                model.add(cons);
            }

            // Don't allocate useless VMs
            //  \forall j \in S, k \in C: \sum_{i \in F} y_{i,j,k} <= N_{j,k}
            ++cc;
            for (std::size_t j = 0; j < nsvcs; ++j)
            {
                auto const svc_cat = svc_categories[j];

                for (std::size_t k = 0; k < nvmcats; ++k)
                {
                    IloIntExpr lhs_expr(env);
                    for (std::size_t i = 0; i < nfns; ++i)
                    {
                        lhs_expr += y[i][j][k];
                    }

                    std::ostringstream oss;
                    oss << "C" << cc << "_{" << j << "," << k << "}";

                    IloConstraint cons(lhs_expr <= IloInt(svc_cat_vm_cat_min_num_vms[svc_cat][k]));
                    cons.setName(oss.str().c_str());
                    model.add(cons);
                }
            }

/*FIXME: this constraint is redundant because it is implied by the above two constraints
            // A VM cannot be allocated to a powered-off FN
            //  \forall i \in F, j \in S, k \in C: y_{i,j,k} \le x_i N_{j,k}
            ++cc;
            for (std::size_t i = 0; i < nfns; ++i)
            {
                for (std::size_t j = 0; j < nsvcs; ++j)
                {
                    auto const svc_cat = svc_categories[j];

                    for (std::size_t k = 0; k < nvmcats; ++k)
                    {
                        std::ostringstream oss;
                        oss << "C" << cc << "_{" << i << "," << j << "," << k << "}";

                        IloConstraint cons(y[i][j][k] <= x[i]*IloInt(svc_cat_vm_cat_min_num_vms[svc_cat][k]));
                        cons.setName(oss.str().c_str());
                        model.add(cons);
                    }
                }
            }
*/

            // Constraints the value of x_{i} to force using a selected set of FNs:
            //   x_{i} = 1 if fixed_fns.count(i) > 0
            //   x_{i} = 0 if fixed_fns.count(i) == 0
            if (fixed_fns.size() > 0)
            {
                ++cc;
                for (std::size_t i = 0; i < nfns; ++i)
                {
                    std::ostringstream oss;
                    oss << "C" << cc << "_{" << i << "}";

                    IloConstraint cons(x[i] == IloBool((fixed_fns.count(i) > 0) ? true : false));
                    cons.setName(oss.str().c_str());
                    model.add(cons);
                }
            }


            // Set objective
            IloObjective obj;
            IloNumExpr revenue_expr(env);
            IloNumExpr cost_expr(env);

            // Revenues
            for (std::size_t i = 0; i < nfns; ++i)
            {
                for (std::size_t j = 0; j < nsvcs; ++j)
                {
                    auto const svc_cat = svc_categories[j];

                    for (std::size_t k = 0; k < nvmcats; ++k)
                    {
                        revenue_expr += fp_svc_cat_revenues[svc_cat]*y[i][j][k];
                    }
                }
            }

            // Costs
            // - Add elecricity costs
            for (std::size_t i = 0; i < nfns; ++i)
            {
                auto const fn_cat = fn_categories[i];
                auto const fn_power_state = fn_power_states[i];
                auto const dC = fn_cat_max_powers[fn_cat]-fn_cat_min_powers[fn_cat];
                auto const wcost = fp_electricity_cost;

                // Add elecricity consumption cost
                cost_expr += (x[i]*fn_cat_min_powers[fn_cat]+dC*u[i])*wcost;

                // Add switch-on/off costs
                cost_expr += x[i]*IloInt(1-fn_power_state)*fp_fn_cat_awake_costs[fn_cat]/deltat
                          +  (1-x[i])*IloInt(fn_power_state)*fp_fn_cat_asleep_costs[fn_cat]/deltat;
            }
            // - Add VM (re)allocation costs
            for (std::size_t i = 0; i < nfns; ++i)
            {
                for (std::size_t j = 0; j < nsvcs; ++j)
                {
                    auto const svc_cat = svc_categories[j];

                    for (std::size_t k = 0; k < nvmcats; ++k)
                    {
                        IloInt old_y = 0;
                        if (fn_vm_allocations[i].count(j) > 0 && fn_vm_allocations[i].at(j).first == k)
                        {
                            // This FN already hosted some VMs for service j
                            old_y = fn_vm_allocations[i].at(j).second;
                        }
                        cost_expr += IloMax(0, y[i][j][k] - old_y)*vm_cat_alloc_costs[k]/deltat;
                    }
                }
            }
            // - Add service penalties
            for (std::size_t j = 0; j < nsvcs; ++j)
            {
                auto const svc_cat = svc_categories[j];

                bool need_vms = true;
                for (std::size_t k = 0; k < nvmcats && need_vms; ++k)
                {
                    if (svc_cat_vm_cat_min_num_vms[svc_cat][k] == 0)
                    {
                        need_vms = false;
                    }
                }
                if (!need_vms)
                {
                    continue;
                }

                IloIntExpr left_expr(env);

/*
                for (std::size_t i = 0; i < nfns; ++i)
                {
                    for (std::size_t k = 0; k < nvmcats; ++k)
                    {
                        //left_expr += ((y[i][j][k] > 0) && (y[i][j][k] < svc_cat_vm_cat_min_num_vms[svc_cat][k])); // FIXME: check if it works
                        left_expr += (y[i][j][k] > 0)*(y[i][j][k] < svc_cat_vm_cat_min_num_vms[svc_cat][k]);
                    }
                }
*/
                for (std::size_t k = 0; k < nvmcats; ++k)
                {
                    IloIntExpr ysum_expr(env);

                    for (std::size_t i = 0; i < nfns; ++i)
                    {
                        ysum_expr += y[i][j][k];
                    }

                    left_expr += (ysum_expr > 0)*(ysum_expr < svc_cat_vm_cat_min_num_vms[svc_cat][k]);
                }

                //cost_expr += ((s[j] == 0) || left_expr)*fp_svc_cat_penalties[svc_cat]; // Don't work
                cost_expr += ((s[j] == 0) + left_expr)*fp_svc_cat_penalties[svc_cat];
            }

            obj = IloMaximize(env, deltat*(revenue_expr-cost_expr));
            model.add(obj);


            // Create the CPLEX solver and make 'model' the active ("extracted") model
            IloCP solver(model);

            //write model
#ifndef DCS_DEBUG
            solver.setOut(env.getNullStream());
            solver.setWarning(env.getNullStream());
#else // DCS_DEBUG
            solver.exportModel("opt_vm_alloc-cp-model.cpo");
            solver.dumpModel("opt_vm_alloc-cp-model_dump.cpo");
#endif // DCS_DEBUG

            // Set Relative Optimality Tolerance to (rel_tol_*100)%: CP will stop as soon as it has found a feasible solution proved to be within (rel_tol_*100)% of optimal.
            if (math::float_traits<RealT>::definitely_greater(rel_tol_, 0))
            {
                //solver.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, rel_tol_);
                solver.setParameter(IloCP::RelativeOptimalityTolerance, rel_tol_);
            }
            // Set Time Limit to limit the execution time of the search
            if (math::float_traits<RealT>::definitely_greater(time_lim_, 0))
            {
                //solver.setParam(IloCplex::Param::TimeLimit, time_lim_);
                solver.setParameter(IloCP::TimeLimit, time_lim_);
            }
            // Set the search log verbosity to 'terse' (default is 'normal') to
            // limit the amount of data written into the log file in case the
            // search takes very long
            solver.setParameter(IloCP::LogVerbosity, IloCP::Terse);

#ifdef DCS_DEBUG
            detail::dump_cp_settings(solver);
#endif // DCS_DEBUG

            // Try different search types (default is the Restart types)
            // Restart search:
            //solver.setParameter(IloCP::SearchType, IloCP::Restart)); // default
            //solver.setParameter(IloCP::RestartFailLimit, 100); // default
            //solver.setParameter(IloCP::RestartGrowthFactor, 1.15); // default
            // Depth-first search:
            //solver.setParameter(IloCP::SearchType, IloCP::DepthFirst);
            // Multi-point search:
            //solver.setParameter(IloCP::SearchType, IloCP::MultiPoint);
            //solver.setParameter(IloCP::BranchLimit, 10000);

            solver.propagate();
            solution.solved = solver.solve();
            solution.optimal = false;

            IloAlgorithm::Status status = solver.getStatus();
            switch (status)
            {
                case IloAlgorithm::Optimal: // The algorithm found an optimal solution.
                    solution.objective_value = static_cast<RealT>(solver.getObjValue());
                    solution.optimal = true;
                    break;
                case IloAlgorithm::Feasible: // The algorithm found a feasible solution, though it may not necessarily be optimal.

                    solution.objective_value = static_cast<RealT>(solver.getObjValue());
                    dcs::log_warn(DCS_LOGGING_AT, "Optimization problem solved but non-optimal");
                    break;
                case IloAlgorithm::Infeasible: // The algorithm proved the model infeasible (i.e., it is not possible to find an assignment of values to variables satisfying all the constraints in the model).
                case IloAlgorithm::Unbounded: // The algorithm proved the model unbounded.
                case IloAlgorithm::InfeasibleOrUnbounded: // The model is infeasible or unbounded.
                case IloAlgorithm::Error: // An error occurred and, on platforms that support exceptions, that an exception has been thrown.
                case IloAlgorithm::Unknown: // The algorithm has no information about the solution of the model.
                {
                    // Possible CP status (obtained with getInfo):
                    // - IloCP::SearchHasNotFailed: indicates that the search has not failed.
                    // - IloCP::SearchHasFailedNormally: indicates that the search has failed because it has searched the entire search space.
                    // - IloCP::SearchStoppedByLimit: indicates that the search was stopped by a limit, such as a time limit (see IloCP::TimeLimit) or a fail limit (see IloCP::FailLimit ).
                    // - IloCP::SearchStoppedByLabel: indicates that the search was stopped via a fail label which did not exist on any choice point (advanced use).
                    // - IloCP::SearchStoppedByExit: indicates that the search was exited using IloCP::exitSearch.
                    // - IloCP::SearchStoppedByAbort: indicates that the search was stopped by calling IloCP::abortSearch.
                    // - IloCP::UnknownFailureStatus: indicates that the search failed for some other reason.
                    std::ostringstream oss;
                    oss << "Optimization was stopped with status = " << status << " (CP status = " << solver.getInfo(IloCP::FailStatus) << ")";
                    dcs::log_warn(DCS_LOGGING_AT, oss.str());

                    // If the problem was shown to be infeasible, find a minimal
                    // explanation for infeasibility
                    if (solver.refineConflict())
                    {
                        dcs::log_warn(DCS_LOGGING_AT, "Conflict refinement:");
                        solver.writeConflict(DCS_LOGGING_STREAM);
                    }

                    return solution;
                }
            }

#ifdef DCS_DEBUG
            DCS_DEBUG_TRACE( "-------------------------------------------------------------------------------[" );
            DCS_DEBUG_TRACE( "- Objective value: " << solution.objective_value << " (revenue: " << solver.getValue(revenue_expr) << ", cost: " << solver.getValue(cost_expr) << ")");

            DCS_DEBUG_TRACE( "- Decision variables: " );

            // Output x_i
            for (std::size_t i = 0; i < nfns; ++i)
            {
                DCS_DEBUG_STREAM << x[i].getName() << " = " << solver.getValue(x[i]) << " (" << detail::to_IloBool(solver.getValue(x[i])) << ")" << std::endl;
            }

            // Output y_{i,j,k}
            for (std::size_t i = 0; i < nfns; ++i)
            {
                for (std::size_t j = 0; j < nsvcs; ++j)
                {
                    for (std::size_t k = 0; k < nvmcats; ++k)
                    {
                        DCS_DEBUG_STREAM << y[i][j][k].getName() << " = " << solver.getValue(y[i][j][k]) << " (" << detail::to_IloInt(solver.getValue(y[i][j][k])) << ")" << std::endl;
                    }
                }
            }

            DCS_DEBUG_TRACE( "- Derived variables: " );

            // Output u_i
            for (std::size_t i = 0; i < nfns; ++i)
            {
                DCS_DEBUG_STREAM << u[i].getName() << " = " << solver.getValue(u[i]) << std::endl;
            }

            // Output s_j
            for (std::size_t j = 0; j < nsvcs; ++j)
            {
                DCS_DEBUG_STREAM << s[j].getName() << " = " << solver.getValue(s[j]) << " (" << detail::to_IloBool(solver.getValue(s[j])) << ")" << std::endl;
            }

            DCS_DEBUG_TRACE( "]-------------------------------------------------------------------------------" );
#endif // DCS_DEBUG

            auto const check_tol = solver.getParameter(IloCP::RelativeOptimalityTolerance);
            auto const check_int_tol = check_tol; //FIXME: what is the CP's counterpart of IloCplex::Param::MIP::Tolerances::Integrality?

            // - Check 'x' variable consistency
            for (std::size_t i = 0; i < nfns; ++i)
            {
#ifdef DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
                DCS_ASSERT( dcs::math::float_traits<IloNum>::approximately_equal(solver.getValue(x[i]), detail::to_IloBool(solver.getValue(x[i])), check_int_tol),
                            DCS_EXCEPTION_THROW( std::logic_error,
                                                 "Anomaly in the CP 'x' variable" ) );
#else // DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
                if (!dcs::math::float_traits<IloNum>::approximately_equal(solver.getValue(x[i]), detail::to_IloBool(solver.getValue(x[i])), check_int_tol))
                {
                    std::ostringstream oss;
                    oss << "Anomaly in the CP 'x[" << i << "]' variable - value: " << solver.getValue(x[i]) << ", converted-value: " << detail::to_IloBool(solver.getValue(x[i])) << " (tol: " << check_int_tol << ")";
                    dcs::log_warn(DCS_LOGGING_AT, oss.str());
                }
#endif // DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
            }
            // - Check 'y' variable consistency
            for (std::size_t i = 0; i < nfns; ++i)
            {
                for (std::size_t j = 0; j < nsvcs; ++j)
                {
                    for (std::size_t k = 0; k < nvmcats; ++k)
                    {
#ifdef DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
                        DCS_ASSERT( dcs::math::float_traits<IloNum>::approximately_equal(solver.getValue(y[i][j][k]), detail::to_IloInt(solver.getValue(y[i][j][k])), check_int_tol),
                                    DCS_EXCEPTION_THROW( std::logic_error,
                                                         "Anomaly in the CP 'y' variable" ) );
#else // DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
                        if (!dcs::math::float_traits<IloNum>::approximately_equal(solver.getValue(y[i][j][k]), detail::to_IloInt(solver.getValue(y[i][j][k])), check_int_tol))
                        {
                            std::ostringstream oss;
                            oss << "Anomaly in the CP 'y[" << i << "][" << j << "][" << k << "]' variable - value: " << solver.getValue(y[i][j][k]) << ", converted-value: " << detail::to_IloBool(solver.getValue(y[i][j][k])) << " (tol: " << check_int_tol << ")";
                            dcs::log_warn(DCS_LOGGING_AT, oss.str());
                        }
#endif // DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
                    }
                }
            }
            // - Check 's' variable consistency
            for (std::size_t j = 0; j < nsvcs; ++j)
            {
#ifdef DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
                DCS_ASSERT( dcs::math::float_traits<IloNum>::approximately_equal(solver.getValue(s[j]), detail::to_IloBool(solver.getValue(s[j])), check_int_tol),
                            DCS_EXCEPTION_THROW( std::logic_error,
                                                 "Anomaly in the CP 's' variable" ) );
#else // DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
                if (!dcs::math::float_traits<IloNum>::approximately_equal(solver.getValue(s[j]), detail::to_IloBool(solver.getValue(s[j])), check_int_tol))
                {
                    std::ostringstream oss;
                    oss << "Anomaly in the CP 's[" << j << "]' variable - value: " << solver.getValue(s[j]) << ", converted-value: " << detail::to_IloBool(solver.getValue(s[j])) << " (tol: " << check_int_tol << ")";
                    dcs::log_warn(DCS_LOGGING_AT, oss.str());
                }
#endif // DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
            }
            // - Check objective value consistency
#ifdef DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
            DCS_ASSERT( dcs::math::float_traits<IloNum>::approximately_equal(solver.getObjValue(), (solver.getValue(revenue_expr)-solver.getValue(cost_expr))*deltat, check_tol),
                        DCS_EXCEPTION_THROW( std::logic_error,
                                             "Anomaly in th CP solution: objective value does not match with the difference of revenues and costs" ) );
#else // DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
            if (!dcs::math::float_traits<IloNum>::approximately_equal(solver.getObjValue(), (solver.getValue(revenue_expr)-solver.getValue(cost_expr))*deltat, check_tol))
            {
                std::ostringstream oss;
                oss << "Anomaly in the CP solution - objective value and the difference between revenue and costs do not match - objective value: " << solver.getObjValue() << ", difference: " << (solver.getValue(revenue_expr)-solver.getValue(cost_expr))*deltat << " (tol: " << check_tol << ")";
                dcs::log_warn(DCS_LOGGING_AT, oss.str());
            }
#endif // DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY

            // Fill the solution object

            solution.fn_vm_allocations.resize(nfns);
            solution.fn_cpu_allocations.resize(nfns);
            solution.fn_power_states.resize(nfns, 0);
            solution.profit = solution.objective_value;
            solution.cost = solver.getValue(cost_expr)*deltat;
            solution.revenue = solver.getValue(revenue_expr)*deltat;
            //solution.watts = 0;
            for (std::size_t i = 0; i < nfns; ++i)
            {
                //const std::size_t fn = fns[i];

                solution.fn_power_states[i] = detail::to_IloBool(solver.getValue(x[i]));
//                solution.fn_vm_allocations[i].resize(nvms);
                for (std::size_t j = 0; j < nsvcs; ++j)
                {
                    //const std::size_t vm = vms[j];

                    for (std::size_t k = 0; k < nvmcats; ++k)
                    {
                        auto const num_vms = detail::to_IloInt(solver.getValue(y[i][j][k]));
                        if (num_vms > 0)
                        {
                            solution.fn_vm_allocations[i][j] = std::make_pair(k, static_cast<std::size_t>(num_vms));
                        }
                    }
                }
                //solution.fn_cpu_allocations[i] = solver.getValue(u[i]);
                solution.fn_cpu_allocations[i] = dcs::math::roundp(solver.getValue(u[i]), std::log10(1.0/solver.getParameter(IloCP::RelativeOptimalityTolerance)));
            }

            // Consistency check: <obj value> == (<revenue> - <cost>)*deltat
#ifdef DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
            DCS_ASSERT( dcs::math::float_traits<RealT>::approximately_equal(solution.objective_value, (solution.revenue-solution.cost)*deltat, check_tol),
                        DCS_EXCEPTION_THROW( std::logic_error,
                                             "Inconsistency in CP solution: objective value does not match with the difference of revenues and costs" ) );
#else // DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
            if (!dcs::math::float_traits<RealT>::approximately_equal(solution.objective_value, (solution.revenue-solution.cost)*deltat, check_tol))
            {
                std::ostringstream oss;
                oss << "Anomaly in the CP solution - objective value and the difference between revenue and costs do not match - objective value: " << solution.objective_value << ", difference: " << (solution.revenue-solution.cost)*deltat << " (tol: " << check_tol << ")";
                dcs::log_warn(DCS_LOGGING_AT, oss.str());
            }
#endif // DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY

            revenue_expr.end();
            cost_expr.end();
            obj.end();
            s.end();
            u.end();
            y.end();
            x.end();

            // Close the Concert Technology app
            env.end();
        }
        catch (const IloException& e)
        {
            std::ostringstream oss;
            oss << "Got exception from CP Optimizer: " << e.getMessage();
            DCS_EXCEPTION_THROW(std::runtime_error, oss.str());
        }
        catch (...)
        {
            DCS_EXCEPTION_THROW(std::runtime_error,
                                "Unexpected error during the optimization");
        }

        return solution;
    }

    vm_allocation_t<RealT> by_native_cplex(const std::vector<std::size_t>& fn_categories, // Maps every FN to its FN category
                                           const std::vector<bool>& fn_power_states, // The power status of each FN
                                           const std::vector<std::map<std::size_t,std::pair<std::size_t,std::size_t>>>& fn_vm_allocations, // Current VM allocations, by FN and service
                                           const std::set<std::size_t>& fixed_fns, // The set of selected FNs to use in the VM allocation (if empty, any FN can be used)
                                           const std::vector<RealT>& fn_cat_min_powers, // The min power consumption of FNs by FN category
                                           const std::vector<RealT>& fn_cat_max_powers, // The max power consumption of FNs by FN category
                                           const std::vector<std::vector<RealT>>& vm_cat_fn_cat_cpu_specs, // The CPU requirement of VMs by VM category and FN category
                                           //const std::vector<std::vector<RealT>>& vm_cat_fn_cat_ram_specs, // The RAM requirement of VMs by VM category and FN category
                                           const std::vector<RealT>& vm_cat_alloc_costs, // The cost to allocate a VM on a FN (e.g., cost to boot a VM or to live-migrate its state), by VM category
                                           const std::vector<std::size_t>& svc_categories, // Maps every service to its service category
                                           const std::vector<std::vector<std::size_t>>& svc_cat_vm_cat_min_num_vms, // The min number of VMs required to achieve QoS, by service category and VM category
                                           const std::vector<RealT>& fp_svc_cat_revenues, // Monetary revenues by service category
                                           const std::vector<RealT>& fp_svc_cat_penalties, // Monetary penalties by service category
                                           const RealT fp_electricity_cost, // Electricty cost (in $/Wh) of FP
                                           const std::vector<RealT>& fp_fn_cat_asleep_costs, // Cost to power-off a FN by FN category
                                           const std::vector<RealT>& fp_fn_cat_awake_costs, // Cost to power-on a FN by FN category
                                           RealT deltat) const // Length of the time interval
    {
        vm_allocation_t<RealT> solution;

        const std::size_t nfns = fn_categories.size();
        const std::size_t nsvcs = svc_categories.size();
        const std::size_t nvmcats = vm_cat_fn_cat_cpu_specs.size();

        DCS_ASSERT( nfns == fn_categories.size(),
                    DCS_EXCEPTION_THROW( std::logic_error,
                                         "FN categories container has a wrong size" ) );

        DCS_ASSERT( nfns == fn_power_states.size(),
                    DCS_EXCEPTION_THROW( std::logic_error,
                                         "FN power states container has a wrong size" ) );

        DCS_ASSERT( nvmcats == vm_cat_fn_cat_cpu_specs.size(),
                    DCS_EXCEPTION_THROW( std::logic_error,
                                         "VM CPU specs has a wrong size" ) );

        //DCS_ASSERT( nvmcats == vm_cat_fn_cat_ram_specs.size(),
        //            DCS_EXCEPTION_THROW( std::logic_error,
        //                                 "VM RAM specs has a wrong size" ) );


        // Setting up the optimization model
        try
        {
            // Initialize the Concert Technology app
            IloEnv env;

            IloModel model(env);

            model.setName("Max-Profit Optimization");

            // Decision Variables

            // Variables x_{i} \in \{0,1\}: 1 if FN i is to be powered on, 0 otherwise.
            IloBoolVarArray x(env, nfns);
            for (std::size_t i = 0; i < nfns; ++i)
            {
                std::ostringstream oss;
                oss << "x[" << i << "]";
                x[i] = IloBoolVar(env, oss.str().c_str());
                model.add(x[i]);
            }

            // Variables y_{i,j,k} \in \mathbb{N}: denotes the number of class-k VMs are allocated on FN i for service j.
            IloArray<IloArray<IloIntVarArray>> y(env, nfns);
            for (std::size_t i = 0; i < nfns; ++i)
            {
                y[i] = IloArray<IloIntVarArray>(env, nsvcs);

                for (std::size_t j = 0 ; j < nsvcs ; ++j)
                {
                    y[i][j] = IloIntVarArray(env, nvmcats);

                    for (std::size_t k = 0 ; k < nvmcats ; ++k)
                    {
                        std::ostringstream oss;
                        oss << "y[" << i << "][" << j << "][" << k << "]";
                        y[i][j][k] = IloIntVar(env, oss.str().c_str());
                        model.add(y[i][j][k]);
                    }
                }
            }

            // Auxiliary variables s_{j} \in \{0,1\}: 1 if service j is offered, 0
            // otherwise:
            //   s_{j} = \exists i \in F, k \in C : u_{i,j,k} > 0
            // which can be defined in CP as:
            //   s_{j} = ( \sum_{i \in F} \sum_{k \in C} y_{i,j,k} ) > 0
            IloBoolVarArray s(env, nsvcs);
            for (std::size_t j = 0; j < nsvcs; ++j)
            {
                std::ostringstream oss;
                oss << "s[" << j << "]";
                s[j] = IloBoolVar(env, oss.str().c_str());
                model.add(s[j]);
            }

            // Variable u_{i} \in [0,1]: total fraction of CPU of FN i allocated to VMs
            //   u_{i} = \sum_{j \in S} \sum_{k \in C} y_{i,j,k}*C_{i,k}, \forall i \in FN'
            IloNumVarArray u(env, nfns);
            for (std::size_t i = 0; i < nfns; ++i)
            {
                std::ostringstream oss;
                oss << "u[" << i << "]";
                u[i] = IloNumVar(env, oss.str().c_str());
                model.add(u[i]);
            }

            // Constraints

            std::size_t cc = 0; // Constraint counter

            // Constraints the values of s_{j}:
            //   s_{j} = \exists i \in F, k \in C : y_{i,j,k} > 0
            // which can be defined in CP as:
            //   \forall j \in S: s_{j} = ( \sum_{i \in F} \sum_{k \in C} y_{i,j,k} ) > 0)
            ++cc;
            for (std::size_t j = 0; j < nsvcs; ++j)
            {
                IloIntExpr rhs_expr(env);
                for (std::size_t i = 0; i < nfns; ++i)
                {
                    rhs_expr += IloSum(y[i][j]);
/*
                    for (std::size_t k = 0; k < nvmcats; ++k)
                    {
                        rhs_expr += y[i][j][k];
                    }
*/
                }
                //rhs_expr = rhs_expr > 0;

                std::ostringstream oss;
                oss << "C" << cc << "_{" << j << "}";

                IloConstraint cons(s[j] == (rhs_expr > 0));
                cons.setName(oss.str().c_str());
                model.add(cons);
            }

            // Constraints the value of u_{i}:
            //   u_{i} = \sum_{j \in S} \sum_{k \in C} y_{i,j,k}*C_{i,k}, \forall i \in FN'
            for (std::size_t i = 0; i < nfns; ++i)
            {
                auto const fn_cat = fn_categories[i];

                IloNumExpr rhs_expr(env);
                for (std::size_t j = 0; j < nsvcs; ++j)
                {
                    for (std::size_t k = 0; k < nvmcats; ++k)
                    {
                        rhs_expr += y[i][j][k]*vm_cat_fn_cat_cpu_specs[k][fn_cat];
                    }
                }

                std::ostringstream oss;
                oss << "C" << cc << "_{" << i << "}";

                IloConstraint cons(u[i] == rhs_expr);
                cons.setName(oss.str().c_str());
                model.add(cons);
            }


            // VMs allocated for a given service must belong to the same class
            //  \forall j \in S, k_1 \in C : \nexists k2 \in C, k2 \ne k1 : ( \sum_{i \in F} y_{i,j,k_1} ) > 0 \wedge ( \sum_{i \in F} y_{i,j,k_2} ) > 0
            // which can be defined in CP as:
#if 1
            // Alternative #1
            //  \forall j \in S: \sum_{i \in F} \sum_{k \in C} y_{i,j,k} = max_{k \in C} \sum_{i \in F} y_{i,j,k}
            ++cc;
            for (std::size_t j = 0; j < nsvcs; ++j)
            {
                IloIntExpr lhs_expr(env); // \sum_{i \in F} \sum_{k \in C} y_{i,j,k}
                for (std::size_t i = 0; i < nfns; ++i)
                {
/*
                    for (std::size_t k = 0; k < nvmcats; ++k)
                    {
                        lhs_expr += y[i][j][k];
                    }
*/
                    lhs_expr += IloSum(y[i][j]);
                }

                IloIntExprArray max_op_expr(env, nvmcats); // max_{k \in C} \sum_{i \in F} y_{i,j,k}
                for (std::size_t k = 0; k < nvmcats; ++k)
                {
                    max_op_expr[k] = IloIntExpr(env);
                    for (std::size_t i = 0; i < nfns; ++i)
                    {
                        max_op_expr[k] += y[i][j][k]; 
                    }
                }

                std::ostringstream oss;
                oss << "C" << cc << "_{" << j << "}";

                IloConstraint cons(lhs_expr == IloMax(max_op_expr));
                cons.setName(oss.str().c_str());
                model.add(cons);
            }
#else
            // Alternative #2
            //  \forall j \in S, \forall k_1 \in C : ( ((\sum_{i \in F} y_{i,j,k_1}) > 0) \cdot \sum_{k_2 \in C, k_2 > k_1} ((\sum_{i \in F} y_{i,j,k_2}) > 0) == 0 )
            ++cc;
            for (std::size_t j = 0; j < nsvcs; ++j)
            {
                IloIntExpr lhs_expr(env);
                for (std::size_t k1 = 0; k1 < nvmcats; ++k1)
                {
                    IloIntExpr ysum1_expr(env);
                    for (std::size_t i = 0; i < nfns; ++i)
                    {
                        ysum1_expr += y[i][j][k1];
                    }
                    for (std::size_t k2 = k1+1; k2 < nvmcats; ++k2)
                    {
                        IloIntExpr ysum2_expr(env);
                        for (std::size_t i = 0; i < nfns; ++i)
                        {
                            ysum2_expr += y[i][j][k2];
                        }
                        lhs_expr += ysum1_expr*ysum2_expr;
                    }
                }

                std::ostringstream oss;
                oss << "C" << cc << "_{" << j << "}";

                IloConstraint cons(lhs_expr == 0);
                cons.setName(oss.str().c_str());
                model.add(cons);
            }
#endif

            // The total allocated capacity on a powered-on FN must not exceed the max capacity
            //  \forall i \in F: u_{t,i} \le x_{i}
            ++cc;
            for (std::size_t i = 0; i < nfns; ++i)
            {
                std::ostringstream oss;
                oss << "C" << cc << "_{" << i << "}";

                IloConstraint cons(u[i] <= x[i]);
                cons.setName(oss.str().c_str());
                model.add(cons);
            }

            // Don't allocate useless VMs
            //  \forall j \in S, k \in C: \sum_{i \in F} y_{i,j,k} <= N_{j,k}
            ++cc;
            for (std::size_t j = 0; j < nsvcs; ++j)
            {
                auto const svc_cat = svc_categories[j];

                for (std::size_t k = 0; k < nvmcats; ++k)
                {
                    IloIntExpr lhs_expr(env);
                    for (std::size_t i = 0; i < nfns; ++i)
                    {
                        lhs_expr += y[i][j][k];
                    }

                    std::ostringstream oss;
                    oss << "C" << cc << "_{" << j << "," << k << "}";

                    IloConstraint cons(lhs_expr <= IloInt(svc_cat_vm_cat_min_num_vms[svc_cat][k]));
                    cons.setName(oss.str().c_str());
                    model.add(cons);
                }
            }

/*FIXME: this constraint is redundant because it is implied by the above two constraints
            // A VM cannot be allocated to a powered-off FN
            //  \forall i \in F, j \in S, k \in C: y_{i,j,k} \le x_{i} N_{j,k}
            ++cc;
            for (std::size_t i = 0; i < nfns; ++i)
            {
                for (std::size_t j = 0; j < nsvcs; ++j)
                {
                    auto const svc_cat = svc_categories[j];

                    for (std::size_t k = 0; k < nvmcats; ++k)
                    {
                        std::ostringstream oss;
                        oss << "C" << cc << "_{" << i << "," << j << "," << k << "}";

                        IloConstraint cons(y[i][j][k] <= x[t][i]*IloInt(slot_svc_cat_vm_cat_min_num_vms[svc_cat][k]));
                        cons.setName(oss.str().c_str());
                        model.add(cons);
                    }
                }
            }
*/

            // Constraints the value of x_{i} to force using a selected set of FNs:
            //   x_{i} = 1 if fixed_fns.count(i) > 0
            //   x_{i} = 0 if fixed_fns.count(i) == 0
            if (fixed_fns.size() > 0)
            {
                ++cc;
                for (std::size_t i = 0; i < nfns; ++i)
                {
                    std::ostringstream oss;
                    oss << "C" << cc << "_{" << i << "}";

                    IloConstraint cons(x[i] == IloBool((fixed_fns.count(i) > 0) ? true : false));
                    cons.setName(oss.str().c_str());
                    model.add(cons);
                }
            }

            // Set objective

            IloObjective obj;

            // Revenues
            IloNumExpr revenue_expr(env);
            for (std::size_t i = 0; i < nfns; ++i)
            {
                for (std::size_t j = 0; j < nsvcs; ++j)
                {
                    auto const svc_cat = svc_categories[j];

                    for (std::size_t k = 0; k < nvmcats; ++k)
                    {
                        revenue_expr += fp_svc_cat_revenues[svc_cat]*y[i][j][k];
                    }
                }
            }

            // Costs
            IloNumExpr cost_expr(env);
            // - Add elecricity costs
            for (std::size_t i = 0; i < nfns; ++i)
            {
                auto const fn_cat = fn_categories[i];
                auto const fn_power_state = fn_power_states[i];
                auto const dC = fn_cat_max_powers[fn_cat]-fn_cat_min_powers[fn_cat];
                auto const wcost = fp_electricity_cost;

                // Add elecricity consumption cost
                cost_expr += (x[i]*fn_cat_min_powers[fn_cat]+dC*u[i])*wcost;

                // Add switch-on/off costs
                cost_expr += x[i]*IloInt(1-fn_power_state)*fp_fn_cat_awake_costs[fn_cat]/deltat
                          +  (1-x[i])*IloInt(fn_power_state)*fp_fn_cat_asleep_costs[fn_cat]/deltat;
            }
            // - Add VM (re)allocation costs
            for (std::size_t i = 0; i < nfns; ++i)
            {
                for (std::size_t j = 0; j < nsvcs; ++j)
                {
                    for (std::size_t k = 0; k < nvmcats; ++k)
                    {
                        IloInt old_y = 0;
                        if (fn_vm_allocations[i].count(j) > 0 && fn_vm_allocations[i].at(j).first == k)
                        {
                            // This FN already hosted some VMs for service j
                            old_y = fn_vm_allocations[i].at(j).second;
                        }
                        cost_expr += IloMax(0, y[i][j][k] - old_y)*vm_cat_alloc_costs[k]/deltat;
                    }
                }
            }
            // - Add service penalties
            for (std::size_t j = 0; j < nsvcs; ++j)
            {
                auto const svc_cat = svc_categories[j];

                bool need_vms = true;
                for (std::size_t k = 0; k < nvmcats && need_vms; ++k)
                {
                    if (svc_cat_vm_cat_min_num_vms[svc_cat][k] == 0)
                    {
                        need_vms = false;
                    }
                }
                if (!need_vms)
                {
                    continue;
                }

                IloIntExpr left_expr(env);

                for (std::size_t k = 0; k < nvmcats; ++k)
                {
                    IloIntExpr ysum_expr(env);

                    for (std::size_t i = 0; i < nfns; ++i)
                    {
                        ysum_expr += y[i][j][k];
                    }

                    //left_expr += (ysum_expr > 0)*(ysum_expr < svc_cat_vm_cat_min_num_vms[svc_cat][k]);
                    left_expr += ((ysum_expr == 0) + (ysum_expr == svc_cat_vm_cat_min_num_vms[svc_cat][k])) == 0;
                }


                //cost_expr += ((s[j] == 0) || left_expr)*fp_svc_cat_penalties[svc_cat]; // Don't work
                cost_expr += ((s[j] == 0) + left_expr)*fp_svc_cat_penalties[svc_cat];
            }

            obj = IloMaximize(env, (revenue_expr-cost_expr)*deltat);
            model.add(obj);


            // Create the CPLEX solver and make 'model' the active ("extracted") model
            IloCplex solver(model);

            //write model
#ifndef DCS_DEBUG
            solver.setOut(env.getNullStream());
            solver.setWarning(env.getNullStream());
#else // DCS_DEBUG
            solver.exportModel("opt_vm_alloc-cplex-model.lp");
#endif // DCS_DEBUG

            // Set Relative Optimality Tolerance to (rel_tol_*100)%: CP will stop as soon as it has found a feasible solution proved to be within (rel_tol_*100)% of optimal.
            if (math::float_traits<RealT>::definitely_greater(rel_tol_, 0))
            {
                //solver.setParam(IloCplex::EpGap, relative_gap);
                solver.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, rel_tol_);
            }
            // Set Time Limit to limit the execution time of the search
            if (math::float_traits<RealT>::definitely_greater(time_lim_, 0))
            {
                solver.setParam(IloCplex::Param::TimeLimit, time_lim_);
            }
//            // Set the search log verbosity to 'terse' (default is 'normal') to
//            // limit the amount of data written into the log file in case the
//            // search takes very long
//            solver.setParameter(IloCplex::Param::WriteLevel, IloCP::Terse);

#ifdef DCS_DEBUG
            detail::dump_cplex_settings(solver);
#endif // DCS_DEBUG

#ifdef DCS_FOG_VM_ALLOC_CPLEX_MEMORY_EMPHASIS
            solver.setParam(IloCplex::Param::Emphasis::Memory, 1);
#endif // DCS_FOG_VM_ALLOC_CPLEX_MEMORY_EMPHASIS

            solution.solved = solver.solve();
            solution.optimal = false;

            IloAlgorithm::Status status = solver.getStatus();
            switch (status)
            {
                case IloAlgorithm::Optimal: // The algorithm found an optimal solution.
                    solution.objective_value = static_cast<RealT>(solver.getObjValue());
                    solution.optimal = true;
                    break;
                case IloAlgorithm::Feasible: // The algorithm found a feasible solution, though it may not necessarily be optimal.

                    solution.objective_value = static_cast<RealT>(solver.getObjValue());
                    dcs::log_warn(DCS_LOGGING_AT, "Optimization problem solved but non-optimal");
                    break;
                case IloAlgorithm::Infeasible: // The algorithm proved the model infeasible (i.e., it is not possible to find an assignment of values to variables satisfying all the constraints in the model).
                case IloAlgorithm::Unbounded: // The algorithm proved the model unbounded.
                case IloAlgorithm::InfeasibleOrUnbounded: // The model is infeasible or unbounded.
                case IloAlgorithm::Error: // An error occurred and, on platforms that support exceptions, that an exception has been thrown.
                case IloAlgorithm::Unknown: // The algorithm has no information about the solution of the model.
                {
                    std::ostringstream oss;
                    oss << "Optimization was stopped with status = " << status << " (CPLEX status = " << solver.getCplexStatus() << ", sub-status = " << solver.getCplexSubStatus() << ")";
                    dcs::log_warn(DCS_LOGGING_AT, oss.str());

                    return solution;
                }
            }

#ifdef DCS_DEBUG
            DCS_DEBUG_TRACE( "-------------------------------------------------------------------------------[" );
            DCS_DEBUG_TRACE( "- Objective value: " << solution.objective_value << " (revenue: " << solver.getValue(revenue_expr) << ", cost: " << solver.getValue(cost_expr) << ", deltat: " << deltat << ")");

            DCS_DEBUG_TRACE( "- Decision variables: " );

            // Output x_{i}
            for (std::size_t i = 0; i < nfns; ++i)
            {
                DCS_DEBUG_STREAM << x[i].getName() << " = " << solver.getValue(x[i]) << " (" << detail::to_IloBool(solver.getValue(x[i])) << ")" << std::endl;
            }

            // Output y_{i,j,k}
            for (std::size_t i = 0; i < nfns; ++i)
            {
                for (std::size_t j = 0; j < nsvcs; ++j)
                {
                    for (std::size_t k = 0; k < nvmcats; ++k)
                    {
                        DCS_DEBUG_STREAM << y[i][j][k].getName() << " = " << solver.getValue(y[i][j][k]) << " (" << detail::to_IloInt(solver.getValue(y[i][j][k])) << ")" << std::endl;
                    }
                }
            }

            DCS_DEBUG_TRACE( "- Derived variables: " );

            // Output u_{i}
            for (std::size_t i = 0; i < nfns; ++i)
            {
                DCS_DEBUG_STREAM << u[i].getName() << " = " << solver.getValue(u[i]) << std::endl;
            }

            // Output s_{j}
            for (std::size_t j = 0; j < nsvcs; ++j)
            {
                DCS_DEBUG_STREAM << s[j].getName() << " = " << solver.getValue(s[j]) << " (" << detail::to_IloBool(solver.getValue(s[j])) << ")" << std::endl;
            }

            // Output revenue
            //DCS_DEBUG_STREAM << revenue_expr.getName() << " = " << solver.getValue(revenue_expr) << std::endl;
            DCS_DEBUG_STREAM << "revenue = " << solver.getValue(revenue_expr) << std::endl;

            // Output cost
            //DCS_DEBUG_STREAM << cost_expr.getName() << " = " << solver.getValue(cost_expr) << std::endl;
            DCS_DEBUG_STREAM << "cost = " << solver.getValue(cost_expr) << std::endl;

            DCS_DEBUG_TRACE( "- Computed values: " );

            {
                // Output computed revenue (compare it with the value of the variable 'revenue')
                RealT comp_revenue = 0;
                for (std::size_t i = 0; i < nfns; ++i)
                {
                    for (std::size_t j = 0; j < nsvcs; ++j)
                    {
                        auto const svc_cat = svc_categories[j];

                        for (std::size_t k = 0; k < nvmcats; ++k)
                        {
                            comp_revenue += fp_svc_cat_revenues[svc_cat]*detail::to_IloInt(solver.getValue(y[i][j][k]));
                        }
                    }
                }
                DCS_DEBUG_STREAM << "Computed revenue = " << comp_revenue << std::endl;

                // Output computed cost (compare it with the value of the variable 'cost')
                RealT comp_cost = 0;
                // - Add elecricity costs
                for (std::size_t i = 0; i < nfns; ++i)
                {
                    auto const fn_cat = fn_categories[i];
                    auto const fn_power_state = fn_power_states[i];
                    auto const dC = fn_cat_max_powers[fn_cat]-fn_cat_min_powers[fn_cat];
                    auto const wcost = fp_electricity_cost;

                    // Add elecricity consumption cost
                    comp_cost += (detail::to_IloBool(solver.getValue(x[i]))*fn_cat_min_powers[fn_cat]+dC*solver.getValue(u[i]))*wcost;

                    // Add switch-on/off costs
                    comp_cost += detail::to_IloBool(solver.getValue(x[i]))*(1-fn_power_state)*fp_fn_cat_awake_costs[fn_cat]/deltat
                              +  (1-detail::to_IloBool(solver.getValue(x[i])))*fn_power_state*fp_fn_cat_asleep_costs[fn_cat]/deltat;
                }
                // - Add VM (re)allocation costs
                for (std::size_t i = 0; i < nfns; ++i)
                {
                    for (std::size_t j = 0; j < nsvcs; ++j)
                    {
                        for (std::size_t k = 0; k < nvmcats; ++k)
                        {
                            IloInt old_y = 0;
                            if (fn_vm_allocations[i].count(j) > 0 && fn_vm_allocations[i].at(j).first == k)
                            {
                                // This FN already hosted some VMs for service j
                                old_y = fn_vm_allocations[i].at(j).second;
                            }
                            comp_cost += std::max(IloInt(0), detail::to_IloInt(solver.getValue(y[i][j][k])) - old_y)*vm_cat_alloc_costs[k]/deltat;
                        }
                    }
                }
                // - Add service penalties
                for (std::size_t j = 0; j < nsvcs; ++j)
                {
                    auto const svc_cat = svc_categories[j];

                    bool need_vms = true;
                    for (std::size_t k = 0; k < nvmcats && need_vms; ++k)
                    {
                        if (svc_cat_vm_cat_min_num_vms[svc_cat][k] == 0)
                        {
                            need_vms = false;
                        }
                    }
                    if (!need_vms)
                    {
                        continue;
                    }

                    std::size_t left_term = 0;

                    for (std::size_t k = 0; k < nvmcats; ++k)
                    {
                        std::size_t ysum_term = 0;

                        for (std::size_t i = 0; i < nfns; ++i)
                        {
                            ysum_term += detail::to_IloInt(solver.getValue(y[i][j][k]));
                        }

                        //left_term += (ysum_term > 0)*(ysum_term < svc_cat_vm_cat_min_num_vms[svc_cat][k]);
                        left_term += ((ysum_term == 0) + (ysum_term == svc_cat_vm_cat_min_num_vms[svc_cat][k])) == 0;
                    }


                    //cost_cost += ((s[t][j] == 0) || left_term)*fp_svc_cat_penalties[svc_cat]; // Don't work
                    comp_cost += ((detail::to_IloBool(solver.getValue(s[j])) == 0) + left_term)*fp_svc_cat_penalties[svc_cat];
                }
                DCS_DEBUG_STREAM << "Computed cost = " << comp_cost << std::endl;
                DCS_DEBUG_STREAM << "Computed objective value = " << (comp_revenue-comp_cost)*deltat << std::endl;
            }

            DCS_DEBUG_TRACE( "]-------------------------------------------------------------------------------" );
#endif // DCS_DEBUG

            // Perform consistency checks

            auto const check_tol = solver.getParam(IloCplex::Param::MIP::Tolerances::MIPGap);
            auto const check_int_tol = solver.getParam(IloCplex::Param::MIP::Tolerances::Integrality);

            // - Check 'x' variable consistency
            for (std::size_t i = 0; i < nfns; ++i)
            {
DCS_DEBUG_TRACE( "x[" << i << "] - Value: " << solver.getValue(x[i]) << ", Converted: " << detail::to_IloBool(solver.getValue(x[i])) << ", tol: " << check_int_tol );
#ifdef DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
                DCS_ASSERT( dcs::math::float_traits<IloNum>::approximately_equal(solver.getValue(x[i]), detail::to_IloBool(solver.getValue(x[i])), check_int_tol),
                            DCS_EXCEPTION_THROW( std::logic_error,
                                                 "Anomaly in the CPLEX 'x' variable" ) );
#else // DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
                if (!dcs::math::float_traits<IloNum>::approximately_equal(solver.getValue(x[i]), detail::to_IloBool(solver.getValue(x[i])), check_int_tol))
                {
                    std::ostringstream oss;
                    oss << "Anomaly in the CPLEX 'x[" << i << "]' variable - value: " << solver.getValue(x[i]) << ", converted-value: " << detail::to_IloBool(solver.getValue(x[i])) << " (tol: " << check_int_tol << ")";
                    dcs::log_warn(DCS_LOGGING_AT, oss.str());
                }
#endif // DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
            }
            // - Check 'y' variable consistency
            for (std::size_t i = 0; i < nfns; ++i)
            {
                for (std::size_t j = 0; j < nsvcs; ++j)
                {
                    for (std::size_t k = 0; k < nvmcats; ++k)
                    {
#ifdef DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
                        DCS_ASSERT( dcs::math::float_traits<IloNum>::approximately_equal(solver.getValue(y[i][j][k]), detail::to_IloInt(solver.getValue(y[i][j][k])), check_int_tol),
                                    DCS_EXCEPTION_THROW( std::logic_error,
                                                         "Anomaly in the CPLEX 'y' variable" ) );
#else // DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
                        if (!dcs::math::float_traits<IloNum>::approximately_equal(solver.getValue(y[i][j][k]), detail::to_IloInt(solver.getValue(y[i][j][k])), check_int_tol))
                        {
                            std::ostringstream oss;
                            oss << "Anomaly in the CPLEX 'y[" << i << "][" << j << "][" << k << "]' variable - value: " << solver.getValue(y[i][j][k]) << ", converted-value: " << detail::to_IloBool(solver.getValue(y[i][j][k])) << " (tol: " << check_int_tol << ")";
                            dcs::log_warn(DCS_LOGGING_AT, oss.str());
                        }
#endif // DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
                    }
                }
            }
            // - Check 's' variable consistency
            for (std::size_t j = 0; j < nsvcs; ++j)
            {
#ifdef DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
                DCS_ASSERT( dcs::math::float_traits<IloNum>::approximately_equal(solver.getValue(s[j]), detail::to_IloBool(solver.getValue(s[j])), check_int_tol),
                            DCS_EXCEPTION_THROW( std::logic_error,
                                                 "Anomaly in the CPLEX 's' variable" ) );
#else // DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
                if (!dcs::math::float_traits<IloNum>::approximately_equal(solver.getValue(s[j]), detail::to_IloBool(solver.getValue(s[j])), check_int_tol))
                {
                    std::ostringstream oss;
                    oss << "Anomaly in the CPLEX 's[" << j << "]' variable - value: " << solver.getValue(s[j]) << ", converted-value: " << detail::to_IloBool(solver.getValue(s[j])) << " (tol: " << check_int_tol << ")";
                    dcs::log_warn(DCS_LOGGING_AT, oss.str());
                }
#endif // DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
            }
            // - Check objective value consistency
#ifdef DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
            DCS_ASSERT( dcs::math::float_traits<IloNum>::approximately_equal(solver.getObjValue(), (solver.getValue(revenue_expr)-solver.getValue(cost_expr))*deltat, check_tol),
                        DCS_EXCEPTION_THROW( std::logic_error,
                                             "Anomaly in th CPLEX solution: objective value does not match with the difference of revenues and costs" ) );
#else // DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
            if (!dcs::math::float_traits<IloNum>::approximately_equal(solver.getObjValue(), (solver.getValue(revenue_expr)-solver.getValue(cost_expr))*deltat, check_tol))
            {
                std::ostringstream oss;
                oss << "Anomaly in the CPLEX solution - objective value and the difference between revenue and costs do not match - objective value: " << solver.getObjValue() << ", difference: " << (solver.getValue(revenue_expr)-solver.getValue(cost_expr))*deltat << " (tol: " << check_tol << ")";
                dcs::log_warn(DCS_LOGGING_AT, oss.str());
            }
#endif // DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY

            // Fill the solution object

            solution.fn_vm_allocations.resize(nfns);
            solution.fn_cpu_allocations.resize(nfns);
            solution.fn_power_states.resize(nfns, 0);
            solution.profit = solution.objective_value;
#if 1
            solution.cost = solver.getValue(cost_expr)*deltat;
            solution.revenue = solver.getValue(revenue_expr)*deltat;
#else
            // Computed revenue and cost from decision variables instead of getting thei values from the related CPLEX expressions
            solution.revenue = 0;
            for (std::size_t i = 0; i < nfns; ++i)
            {
                for (std::size_t j = 0; j < nsvcs; ++j)
                {
                    auto const svc_cat = svc_categories[j];

                    for (std::size_t k = 0; k < nvmcats; ++k)
                    {
                        solution.revenue += fp_svc_cat_revenues[svc_cat]*detail::to_IloInt(solver.getValue(y[i][j][k]));
                    }
                }
            }
            solution.revenue *= deltat;
            solution.cost = 0;
            // - Add elecricity costs
            for (std::size_t i = 0; i < nfns; ++i)
            {
                auto const fn_cat = fn_categories[i];
                auto const fn_power_state = fn_power_states[i];
                auto const dC = fn_cat_max_powers[fn_cat]-fn_cat_min_powers[fn_cat];
                auto const wcost = fp_electricity_cost;

                // Add elecricity consumption cost
                solution.cost += (detail::to_IloBool(solver.getValue(x[i]))*fn_cat_min_powers[fn_cat]+dC*solver.getValue(u[i]))*wcost;

                // Add switch-on/off costs
                solution.cost += detail::to_IloBool(solver.getValue(x[i]))*(1-fn_power_state)*fp_fn_cat_awake_costs[fn_cat]/deltat
                              +  (1-detail::to_IloBool(solver.getValue(x[i])))*fn_power_state*fp_fn_cat_asleep_costs[fn_cat]/deltat;
            }
            // - Add VM (re)allocation costs
            for (std::size_t i = 0; i < nfns; ++i)
            {
                for (std::size_t j = 0; j < nsvcs; ++j)
                {
                    for (std::size_t k = 0; k < nvmcats; ++k)
                    {
                        IloInt old_y = 0;
                        if (fn_vm_allocations[i].count(j) > 0 && fn_vm_allocations[i].at(j).first == k)
                        {
                            // This FN already hosted some VMs for service j
                            old_y = fn_vm_allocations[i].at(j).second;
                        }
                        solution.cost += std::max(IloInt(0), detail::to_IloInt(solver.getValue(y[i][j][k])) - old_y)*vm_cat_alloc_costs[k]/deltat;
                    }
                }
            }
            // - Add service penalties
            for (std::size_t j = 0; j < nsvcs; ++j)
            {
                auto const svc_cat = svc_categories[j];

                std::size_t left_term = 0;

                for (std::size_t k = 0; k < nvmcats; ++k)
                {
                    std::size_t ysum_term = 0;

                    for (std::size_t i = 0; i < nfns; ++i)
                    {
                        ysum_term += detail::to_IloInt(solver.getValue(y[i][j][k]));
                    }

                    //left_term += (ysum_term > 0)*(ysum_term < svc_cat_vm_cat_min_num_vms[svc_cat][k]);
                    left_term += ((ysum_term == 0) + (ysum_term == svc_cat_vm_cat_min_num_vms[svc_cat][k])) == 0;
                }


                //solution.cost += ((s[t][j] == 0) || left_term)*fp_svc_cat_penalties[svc_cat]; // Don't work
                solution.cost += ((detail::to_IloBool(solver.getValue(s[j])) == 0) + left_term)*fp_svc_cat_penalties[svc_cat];
            }
            solution.cost *= deltat;
#endif
            //solution.watts = 0;
            for (std::size_t i = 0; i < nfns; ++i)
            {
                //const std::size_t fn = fns[i];

                solution.fn_power_states[i] = detail::to_IloBool(solver.getValue(x[i]));
//                solution.fn_vm_allocations[i].resize(nvms);
                for (std::size_t j = 0; j < nsvcs; ++j)
                {
                    //const std::size_t vm = vms[j];

                    for (std::size_t k = 0; k < nvmcats; ++k)
                    {
                        auto const num_vms = detail::to_IloInt(solver.getValue(y[i][j][k]));
                        if (num_vms > 0)
                        {
                            solution.fn_vm_allocations[i][j] = std::make_pair(k, static_cast<std::size_t>(num_vms));
                        }
                    }
                }
                //solution.fn_cpu_allocations[i] = solver.getValue(u[i]);
                solution.fn_cpu_allocations[i] = dcs::math::roundp(solver.getValue(u[i]), std::log10(1.0/solver.getParam(IloCplex::Param::MIP::Tolerances::MIPGap)));
            }

            // Consistency check: <obj value> == (<revenue> - <cost>)*deltat
#ifdef DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
            DCS_ASSERT( dcs::math::float_traits<RealT>::approximately_equal(solution.objective_value, (solution.revenue-solution.cost)*deltat, check_tol),
                        DCS_EXCEPTION_THROW( std::logic_error,
                                             "Anomaly in CPLEX solution: objective value does not match with the difference of revenues and costs" ) );
#else // DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
            if (!dcs::math::float_traits<RealT>::approximately_equal(solution.objective_value, (solution.revenue-solution.cost)*deltat, check_tol))
            {
                std::ostringstream oss;
                oss << "Anomaly in the CPLEX solution - objective value and the difference between revenue and costs do not match - objective value: " << solution.objective_value << ", difference: " << (solution.revenue-solution.cost)*deltat << " (tol: " << check_tol << ")";
                dcs::log_warn(DCS_LOGGING_AT, oss.str());
            }
#endif // DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY

            revenue_expr.end();
            cost_expr.end();
            obj.end();
            s.end();
            u.end();
            y.end();
            x.end();

            // Close the Concert Technology app
            env.end();
        }
        catch (const IloException& e)
        {
            std::ostringstream oss;
            oss << "Got exception from Cplex Optimizer: " << e.getMessage();
            DCS_EXCEPTION_THROW(std::runtime_error, oss.str());
        }
        catch (...)
        {
            DCS_EXCEPTION_THROW(std::runtime_error,
                                "Unexpected error during the optimization");
        }

        return solution;
    }


private:
    RealT rel_tol_; ///< Relative optimality tolerance used to define optimality (a solution is considered optimal if there does not exist a solution with a better objective function with respect to a relative optimality tolerance).
    RealT time_lim_; ///< Time limit (in seconds) used to set the maximum time the optimizare can spend in search for the best solution.
}; // optimal_vm_alllocation_solver


/**
 * \brief Optimal solver for the multi-slot VM allocation problem.
 */
template <typename RealT>
class optimal_multislot_vm_allocation_solver_t: public base_multislot_vm_allocation_solver_t<RealT>
{
public:
    explicit optimal_multislot_vm_allocation_solver_t(RealT relative_tolerance = 0,
                                                      RealT time_limit = -1)
    : rel_tol_(relative_tolerance),
      time_lim_(time_limit)
    {
    }

    void relative_tolerance(RealT value)
    {
        rel_tol_ = value;
    }

    RealT relative_tolerance() const
    {
        return rel_tol_;
    }

    void time_limit(RealT value)
    {
        time_lim_ = value;
    }

    RealT time_limit() const
    {
        return time_lim_;
    }


    multislot_vm_allocation_t<RealT> solve(const std::vector<std::size_t>& fn_categories, // Maps every FN to its FN category
                                           const std::vector<bool>& fn_power_states, // The power status of each FN
                                           const std::vector<std::map<std::size_t,std::pair<std::size_t,std::size_t>>>& fn_vm_allocations, // Current VM allocations, by FN and service
                                           const std::vector<RealT>& fn_cat_min_powers, // The min power consumption of FNs by FN category
                                           const std::vector<RealT>& fn_cat_max_powers, // The max power consumption of FNs by FN category
                                           const std::vector<std::vector<RealT>>& vm_cat_fn_cat_cpu_specs, // The CPU requirement of VMs by VM category and FN category
                                           //const std::vector<std::vector<RealT>>& vm_cat_fn_cat_ram_specs, // The RAM requirement of VMs by VM category and FN category
                                           const std::vector<RealT>& vm_cat_alloc_costs, // The cost to allocate a VM on a FN (e.g., cost to boot a VM or to live-migrate its state), by VM category
                                           const std::vector<std::size_t>& svc_categories, // Service categories by service
                                           const std::vector<std::vector<std::vector<std::size_t>>>& svc_cat_vm_cat_min_num_vms, // The min number of VMs required to achieve QoS, by time slot, service category and VM category
                                           const std::vector<RealT>& fp_svc_cat_revenues, // Monetary revenues by service
                                           const std::vector<RealT>& fp_svc_cat_penalties, // Monetary penalties by service
                                           const RealT fp_electricity_cost, // Electricty cost (in $/Wh) of FP
                                           const std::vector<RealT>& fp_fn_cat_asleep_costs, // Cost to power-off a FN by FN category
                                           const std::vector<RealT>& fp_fn_cat_awake_costs, // Cost to power-on a FN by FN category
                                           RealT deltat = 1) const // Length of the time interval
    {
        DCS_DEBUG_TRACE("Finding optimal VM allocation:");
        DCS_DEBUG_TRACE("- Number of FNs: " << fn_categories.size());
        DCS_DEBUG_TRACE("- FN Categories: " << fn_categories);
        DCS_DEBUG_TRACE("- FN Power States: " << fn_power_states);
        DCS_DEBUG_TRACE("- FN - VM Allocations: " << fn_vm_allocations);
        DCS_DEBUG_TRACE("- FN Mininimum Power Consumption by FN Category: " << fn_cat_min_powers);
        DCS_DEBUG_TRACE("- FN Maximum Power Consumption by FN Category: " << fn_cat_max_powers);
        DCS_DEBUG_TRACE("- VM CPU requirements by VM Category and FN Category: " << vm_cat_fn_cat_cpu_specs);
        //DCS_DEBUG_TRACE("- VM RAM requirements by VM Category and FN Category: " << vm_cat_fn_cat_ram_specs);
        DCS_DEBUG_TRACE("- VM Allocation Costs by VM Category: " << vm_cat_alloc_costs);
        DCS_DEBUG_TRACE("- Number of Services: " << svc_categories.size());
        DCS_DEBUG_TRACE("- Service Categories: " << svc_categories);
        DCS_DEBUG_TRACE("- Service Minimum Number of VMs by Service Category and VM Category: " << svc_cat_vm_cat_min_num_vms);
        DCS_DEBUG_TRACE("- FP Service Revenues by Service Category: " << fp_svc_cat_revenues);
        DCS_DEBUG_TRACE("- FP Service Penalties by Service Category: " << fp_svc_cat_penalties);
        DCS_DEBUG_TRACE("- FP Energy Cost: " << fp_electricity_cost);
        DCS_DEBUG_TRACE("- FN On->Off Cost by FN Category: " << fp_fn_cat_asleep_costs);
        DCS_DEBUG_TRACE("- FN Off->On Cost by FN Category: " << fp_fn_cat_awake_costs);
        DCS_DEBUG_TRACE("- Length of the time interval: " << deltat);
        DCS_DEBUG_TRACE("- Relative Tolerance: " << rel_tol_);
        DCS_DEBUG_TRACE("- Time Limit: " << time_lim_);

#if defined(DCS_FOG_VM_ALLOC_USE_CPLEX_SOLVER)
        return by_native_cplex(fn_categories,
                               fn_power_states,
                               fn_vm_allocations,
                               std::vector<std::set<std::size_t>>(), // any FN can be used in any time slot
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
#elif defined(DCS_FOG_VM_ALLOC_USE_CP_SOLVER)
        return by_native_cp(fn_categories,
                            fn_power_states,
                            fn_vm_allocations,
                            std::vector<std::set<std::size_t>>(), // any FN can be used in any time slot
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
#else
# error Unable to find a suitable solver for the multi-slot VM allocation problem
#endif // DCS_FOG_VM_ALLOC_USE_..._SOLVER
    }

    multislot_vm_allocation_t<RealT> solve_with_fixed_fns(const std::vector<std::set<std::size_t>>& fixed_fns, // For each time slot, the set of selected FNs to use for the VM allocation (if in a given time slot the set is empty, any FN can be used)
                                                          const std::vector<std::size_t>& fn_categories, // Maps every FN to its FN category
                                                          const std::vector<bool>& fn_power_states, // The power status of each FN
                                                          const std::vector<std::map<std::size_t,std::pair<std::size_t,std::size_t>>>& fn_vm_allocations, // Current VM allocations, by FN and service
                                                          const std::vector<RealT>& fn_cat_min_powers, // The min power consumption of FNs by FN category
                                                          const std::vector<RealT>& fn_cat_max_powers, // The max power consumption of FNs by FN category
                                                          const std::vector<std::vector<RealT>>& vm_cat_fn_cat_cpu_specs, // The CPU requirement of VMs by VM category and FN category
                                                          //const std::vector<std::vector<RealT>>& vm_cat_fn_cat_ram_specs, // The RAM requirement of VMs by VM category and FN category
                                                          const std::vector<RealT>& vm_cat_alloc_costs, // The cost to allocate a VM on a FN (e.g., cost to boot a VM or to live-migrate its state), by VM category
                                                          const std::vector<std::size_t>& svc_categories, // Service categories by service
                                                          const std::vector<std::vector<std::vector<std::size_t>>>& svc_cat_vm_cat_min_num_vms, // The min number of VMs required to achieve QoS, by time slot, service category and VM category
                                                          const std::vector<RealT>& fp_svc_cat_revenues, // Monetary revenues by service
                                                          const std::vector<RealT>& fp_svc_cat_penalties, // Monetary penalties by service
                                                          const RealT fp_electricity_cost, // Electricty cost (in $/Wh) of FP
                                                          const std::vector<RealT>& fp_fn_cat_asleep_costs, // Cost to power-off a FN by FN category
                                                          const std::vector<RealT>& fp_fn_cat_awake_costs, // Cost to power-on a FN by FN category
                                                          RealT deltat = 1) const // Length of the time interval
    {
        DCS_DEBUG_TRACE("Finding optimal VM allocation:");
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
        DCS_DEBUG_TRACE("- Number of Services: " << svc_categories.size());
        DCS_DEBUG_TRACE("- Service Categories: " << svc_categories);
        DCS_DEBUG_TRACE("- Service Minimum Number of VMs by Service Category and VM Category: " << svc_cat_vm_cat_min_num_vms);
        DCS_DEBUG_TRACE("- FP Service Revenues by Service Category: " << fp_svc_cat_revenues);
        DCS_DEBUG_TRACE("- FP Service Penalties by Service Category: " << fp_svc_cat_penalties);
        DCS_DEBUG_TRACE("- FP Energy Cost: " << fp_electricity_cost);
        DCS_DEBUG_TRACE("- FN On->Off Cost by FN Category: " << fp_fn_cat_asleep_costs);
        DCS_DEBUG_TRACE("- FN Off->On Cost by FN Category: " << fp_fn_cat_awake_costs);
        DCS_DEBUG_TRACE("- Length of the time interval: " << deltat);
        DCS_DEBUG_TRACE("- Relative Tolerance: " << rel_tol_);
        DCS_DEBUG_TRACE("- Time Limit: " << time_lim_);

#if defined(DCS_FOG_VM_ALLOC_USE_CPLEX_SOLVER)
        return by_native_cplex(fn_categories,
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
#elif defined(DCS_FOG_VM_ALLOC_USE_CP_SOLVER)
        return by_native_cp(fn_categories,
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
#else
# error Unable to find a suitable solver for the multi-slot VM allocation problem
#endif // DCS_FOG_VM_ALLOC_USE_..._SOLVER
    }


private:
    multislot_vm_allocation_t<RealT> by_native_cp(const std::vector<std::size_t>& fn_categories, // Maps every FN to its FN category
                                                  const std::vector<bool>& fn_power_states, // The power status of each FN
                                                  const std::vector<std::map<std::size_t,std::pair<std::size_t,std::size_t>>>& fn_vm_allocations, // Current VM allocations, by FN and service
                                                  const std::vector<std::set<std::size_t>>& fixed_fns, // For each time slot, the set of selected FNs to use for the VM allocation (if in a given time slot the set is empty, any FN can be used)
                                                  const std::vector<RealT>& fn_cat_min_powers, // The min power consumption of FNs by FN category
                                                  const std::vector<RealT>& fn_cat_max_powers, // The max power consumption of FNs by FN category
                                                  const std::vector<std::vector<RealT>>& vm_cat_fn_cat_cpu_specs, // The CPU requirement of VMs by VM category and FN category
                                                  //const std::vector<std::vector<RealT>>& vm_cat_fn_cat_ram_specs, // The RAM requirement of VMs by VM category and FN category
                                                  const std::vector<RealT>& vm_cat_alloc_costs, // The cost to allocate a VM on a FN (e.g., cost to boot a VM or to live-migrate its state), by VM category
                                                  const std::vector<std::size_t>& svc_categories, // Maps every service to its service category
                                                  const std::vector<std::vector<std::vector<std::size_t>>>& slot_svc_cat_vm_cat_min_num_vms, // The min number of VMs required to achieve QoS, by time slot, service category and VM category
                                                  const std::vector<RealT>& fp_svc_cat_revenues, // Monetary revenues by service category
                                                  const std::vector<RealT>& fp_svc_cat_penalties, // Monetary penalties by service category
                                                  const RealT fp_electricity_cost, // Electricty cost (in $/Wh) of FP
                                                  const std::vector<RealT>& fp_fn_cat_asleep_costs, // Cost to power-off a FN by FN category
                                                  const std::vector<RealT>& fp_fn_cat_awake_costs, // Cost to power-on a FN by FN category
                                                  RealT deltat) const // Length of the time interval
    {
        multislot_vm_allocation_t<RealT> solution;

        const std::size_t nfns = fn_categories.size();
        const std::size_t nsvcs = svc_categories.size();
        const std::size_t nvmcats = vm_cat_fn_cat_cpu_specs.size();
        const std::size_t nslots = slot_svc_cat_vm_cat_min_num_vms.size();

        DCS_ASSERT( nfns == fn_categories.size(),
                    DCS_EXCEPTION_THROW( std::logic_error,
                                         "FN categories container has a wrong size" ) );

        DCS_ASSERT( nfns == fn_power_states.size(),
                    DCS_EXCEPTION_THROW( std::logic_error,
                                         "FN power states container has a wrong size" ) );

        DCS_ASSERT( nvmcats == vm_cat_fn_cat_cpu_specs.size(),
                    DCS_EXCEPTION_THROW( std::logic_error,
                                         "VM CPU specs has a wrong size" ) );

        //DCS_ASSERT( nvmcats == vm_cat_fn_cat_ram_specs.size(),
        //            DCS_EXCEPTION_THROW( std::logic_error,
        //                                 "VM RAM specs has a wrong size" ) );

        // Setting up the optimization model
        try
        {
            // Initialize the Concert Technology app
            IloEnv env;

            IloModel model(env);

            model.setName("Max-Profit Optimization");

            // Decision Variables

            // Variables x_{t,i} \in \{0,1\}: 1 if, at time slot t, FN i is to be powered on, 0 otherwise.
            IloArray<IloBoolVarArray> x(env, nslots);
            for (std::size_t t = 0; t < nslots; ++t)
            {
                x[t] = IloBoolVarArray(env, nfns);
                for (std::size_t i = 0; i < nfns; ++i)
                {
                    std::ostringstream oss;
                    oss << "x[" << t << "][" << i << "]";
                    x[t][i] = IloBoolVar(env, oss.str().c_str());
                    model.add(x[t][i]);
                }
            }

            // Variables y_{t,i,j,k} \in \mathbb{N}: denotes the number of class-k VMs are allocated on FN i for service j at time slot t.
            IloArray<IloArray<IloArray<IloIntVarArray>>> y(env, nslots);
            for (std::size_t t = 0; t < nslots; ++t)
            {
                y[t] = IloArray<IloArray<IloIntVarArray>>(env, nfns);

                for (std::size_t i = 0; i < nfns; ++i)
                {
                    y[t][i] = IloArray<IloIntVarArray>(env, nsvcs);

                    for (std::size_t j = 0 ; j < nsvcs ; ++j)
                    {
                        y[t][i][j] = IloIntVarArray(env, nvmcats);

                        for (std::size_t k = 0 ; k < nvmcats ; ++k)
                        {
                            std::ostringstream oss;
                            oss << "y[" << t << "][" << i << "][" << j << "][" << k << "]";
                            y[t][i][j][k] = IloIntVar(env, oss.str().c_str());
                            model.add(y[t][i][j][k]);
                        }
                    }
                }
            }

#if DCS_FOG_VM_ALLOC_CP_USE_S_AS_DECISION_VARIABLE
            // Auxiliary variables s_{t,j} \in \{0,1\}: 1 if service j is offered at time slot t, 0
            // otherwise:
            //   s_{t,j} = \exists i \in F, k \in C : u_{t,i,j,k} > 0
            // which can be defined in CP as:
            //   s_{t,j} = ( \sum_{i \in F} \sum_{k \in C} y_{t,i,j,k} ) > 0
            IloArray<IloBoolVarArray> s(env, nslots);
            for (std::size_t t = 0; t < nslots; ++t)
            {
                s[t] = IloBoolVarArray(env, nsvcs);
                for (std::size_t j = 0; j < nsvcs; ++j)
                {
                    std::ostringstream oss;
                    oss << "s[" << t << "][" << j << "]";
                    s[t][j] = IloBoolVar(env, oss.str().c_str());
                    model.add(s[t][j]);
                }
            }
#endif // DCS_FOG_VM_ALLOC_CP_USE_S_AS_DECISION_VARIABLE

            // Decision expressions

            // Expression u_{t,i} \in [0,1]: total fraction of CPU of FN i allocated to VMs at time slot t
            //   u_{t,i} = \sum_{j \in S} \sum_{k \in C} y_{t,i,j,k}*C_{i,k}, \forall i \in FN'
            IloArray<IloNumExprArray> u(env, nslots);
            for (std::size_t t = 0; t < nslots; ++t)
            {
                u[t] = IloNumExprArray(env, nfns);
                for (std::size_t i = 0; i < nfns; ++i)
                {
                    auto const fn_cat = fn_categories[i];

                    u[t][i] = IloNumExpr(env);

                    for (std::size_t j = 0; j < nsvcs; ++j)
                    {
                        for (std::size_t k = 0; k < nvmcats; ++k)
                        {
                            u[t][i] += y[t][i][j][k]*vm_cat_fn_cat_cpu_specs[k][fn_cat];
                        }
                    }

                    std::ostringstream oss;
                    oss << "u[" << t << "][" << i << "]";
                    u[t][i].setName(oss.str().c_str());

                    model.add(u[t][i]);
                }
            }

#if !DCS_FOG_VM_ALLOC_CP_USE_S_AS_DECISION_VARIABLE
            // Expression s_{t,j} \in \{0,1\}:
            //   s_{t,j} = \exists i \in F, k \in C : u_{t,i,j,k} > 0
            // which can be defined in CP as:
            //   s_{t,j} = ( \sum_{i \in F} \sum_{k \in C} y_{t,i,j,k} ) > 0
            IloArray<IloArray<IloIntExpr>> s(env, nslots);
            for (std::size_t t = 0; t < nslots; ++t)
            {
                s[t] = IloArray<IloIntExpr>(env, nsvcs);
                for (std::size_t j = 0; j < nsvcs; ++j)
                {
                    //s[t][j] = IloIntExpr(env);
                    IloIntExpr rhs_expr(env);

                    for (std::size_t i = 0; i < nfns; ++i)
                    {
/*
                        for (std::size_t k = 0; k < nvmcats; ++k)
                        {
                            s[t][j] += y[t][i][j][k];
                        }
*/
                        rhs_expr += IloSum(y[t][i][j]);
                    }

                    //s[t][j] = s[t][j] > 0;
                    s[t][j] = rhs_expr > 0;

                    std::ostringstream oss;
                    oss << "s[" << t << "][" << j << "]";
                    s[t][j].setName(oss.str().c_str());

                    model.add(s[t][j]);
                }
            }
#endif // !DCS_FOG_VM_ALLOC_CP_USE_S_AS_DECISION_VARIABLE

            // Constraints

            std::size_t cc = 0; // Constraint counter

#if DCS_FOG_VM_ALLOC_CP_USE_S_AS_DECISION_VARIABLE
            // Constraints the values of s_{t,j}:
            //   s_{t,j} = \exists i \in F, k \in C : u_{t,i,j,k} > 0
            // which can be defined in CP as:
            //   \forall t \in T, j \in S: s_{t,j} = ( \sum_{i \in F} \sum_{k \in C} y_{t,i,j,k} ) > 0)
            ++cc;
            for (std::size_t t = 0; t < nslots; ++t)
            {
                for (std::size_t j = 0; j < nsvcs; ++j)
                {
                    IloIntExpr rhs_expr(env);
                    for (std::size_t i = 0; i < nfns; ++i)
                    {
                        rhs_expr += IloSum(y[t][i][j]);
/*
                        for (std::size_t k = 0; k < nvmcats; ++k)
                        {
                            rhs_expr += y[t][i][j][k];
                        }
*/
                    }
                    //rhs_expr = rhs_expr > 0;

                    std::ostringstream oss;
                    oss << "C" << cc << "_{" << t << "," << j << "}";

                    //IloConstraint cons(s[t][j] == rhs_expr);
                    IloConstraint cons(s[t][j] == (rhs_expr > 0));
                    cons.setName(oss.str().c_str());
                    model.add(cons);
                }
            }
#endif

            // VMs allocated for a given service must belong to the same class
            //  \forall t \in T, j \in S, k_1 \in C : \nexists k2 \in C, k2 \ne k1 : ( \sum_{i \in F} y_{t,i,j,k_1} ) > 0 \wedge ( \sum_{i \in F} y_{t,i,j,k_2} ) > 0
            // which can be defined in CP as:
#if 0
            // Alternative #1
            //  \forall t \in T, j \in S: \sum_{i \in F} \sum_{k \in C} y_{t,i,j,k} = max_{k \in C} \sum_{i \in F} y_{t,i,j,k}
            ++cc;
            for (std::size_t t = 0; t < nslots; ++t)
            {
                for (std::size_t j = 0; j < nsvcs; ++j)
                {
                    IloIntExpr lhs_expr(env); // \sum_{i \in F} \sum_{k \in C} y_{t,i,j,k}
                    for (std::size_t i = 0; i < nfns; ++i)
                    {
/*
                        for (std::size_t k = 0; k < nvmcats; ++k)
                        {
                            lhs_expr += y[t][i][j][k];
                        }
*/
                        lhs_expr += IloSum(y[t][i][j]);
                    }

                    IloIntExprArray max_op_expr(env, nvmcats); // max_{k \in C} \sum_{i \in F} y_{t,i,j,k}
                    for (std::size_t k = 0; k < nvmcats; ++k)
                    {
                        max_op_expr[k] = IloIntExpr(env);
                        for (std::size_t i = 0; i < nfns; ++i)
                        {
                            max_op_expr[k] += y[t][i][j][k]; 
                        }
                    }

                    std::ostringstream oss;
                    oss << "C" << cc << "_{" << t << "," << j << "}";

                    IloConstraint cons(lhs_expr == IloMax(max_op_expr));
                    cons.setName(oss.str().c_str());
                    model.add(cons);
                }
            }
#else
            // Alternative #2
            //  \forall t \in T, j \in S, \forall k_1 \in C : ( ((\sum_{i \in F} y_{t,i,j,k_1}) > 0) \cdot \sum_{k_2 \in C, k_2 > k_1} ((\sum_{i \in F} y_{t,i,j,k_2}) > 0) == 0 )
            ++cc;
            for (std::size_t t = 0; t < nslots; ++t)
            {
                for (std::size_t j = 0; j < nsvcs; ++j)
                {
                    IloIntExpr lhs_expr(env);
                    for (std::size_t k1 = 0; k1 < nvmcats; ++k1)
                    {
                        IloIntExpr ysum1_expr(env);
                        for (std::size_t i = 0; i < nfns; ++i)
                        {
                            ysum1_expr += y[t][i][j][k1];
                        }
                        for (std::size_t k2 = k1+1; k2 < nvmcats; ++k2)
                        {
                            IloIntExpr ysum2_expr(env);
                            for (std::size_t i = 0; i < nfns; ++i)
                            {
                                ysum2_expr += y[t][i][j][k2];
                            }
                            lhs_expr += (ysum1_expr > 0)*(ysum2_expr > 0);
                        }
                    }

                    std::ostringstream oss;
                    oss << "C" << cc << "_{" << t << "," << j << "}";

                    IloConstraint cons(lhs_expr == 0);
                    cons.setName(oss.str().c_str());
                    model.add(cons);
                }
            }
#endif

            // The total allocated capacity on a powered-on FN must not exceed the max capacity
            //  \forall t \in T, i \in F: u_{t,i} \le x_{t,i}
            ++cc;
            for (std::size_t t = 0; t < nslots; ++t)
            {
                for (std::size_t i = 0; i < nfns; ++i)
                {
                    std::ostringstream oss;
                    oss << "C" << cc << "_{" << t << "," << i << "}";

                    IloConstraint cons(u[t][i] <= x[t][i]);
                    cons.setName(oss.str().c_str());
                    model.add(cons);
                }
            }

            // Don't allocate useless VMs
            //  \forall t \in T, j \in S, k \in C: \sum_{i \in F} y_{t,i,j,k} <= N_{t,j,k}
            ++cc;
            for (std::size_t t = 0; t < nslots; ++t)
            {
                for (std::size_t j = 0; j < nsvcs; ++j)
                {
                    auto const svc_cat = svc_categories[j];

                    for (std::size_t k = 0; k < nvmcats; ++k)
                    {
                        IloIntExpr lhs_expr(env);
                        for (std::size_t i = 0; i < nfns; ++i)
                        {
                            lhs_expr += y[t][i][j][k];
                        }

                        std::ostringstream oss;
                        oss << "C" << cc << "_{" << t << "," << j << "," << k << "}";

                        IloConstraint cons(lhs_expr <= IloInt(slot_svc_cat_vm_cat_min_num_vms[t][svc_cat][k]));
                        cons.setName(oss.str().c_str());
                        model.add(cons);
                    }
                }
            }

/*FIXME: this constraint is redundant because it is implied by the above two constraints
            // A VM cannot be allocated to a powered-off FN
            //  \forall t \in T, i \in F, j \in S, k \in C: y_{t,i,j,k} \le x_{t,i} N_{t,j,k}
            ++cc;
            for (std::size_t t = 0; t < nslots; ++t)
            {
                for (std::size_t i = 0; i < nfns; ++i)
                {
                    for (std::size_t j = 0; j < nsvcs; ++j)
                    {
                        auto const svc_cat = svc_categories[j];

                        for (std::size_t k = 0; k < nvmcats; ++k)
                        {
                            std::ostringstream oss;
                            oss << "C" << cc << "_{" << t << "," << i << "," << j << "," << k << "}";

                            IloConstraint cons(y[t][i][j][k] <= x[t][i]*IloInt(slot_svc_cat_vm_cat_min_num_vms[t][svc_cat][k]));
                            cons.setName(oss.str().c_str());
                            model.add(cons);
                        }
                    }
                }
            }
*/

            // Constraints the value of x_{t,i} to force using a selected set of FNs:
            //   x_{t,i} = 1 if fixed_fns.count(i) > 0
            //   x_{t,i} = 0 if fixed_fns.count(i) == 0
            if (fixed_fns.size() > 0)
            {
                ++cc;
                for (std::size_t t = 0; t < nslots; ++t)
                {
                    if (fixed_fns[t].size() > 0)
                    {
                        for (std::size_t i = 0; i < nfns; ++i)
                        {
                            std::ostringstream oss;
                            oss << "C" << cc << "_{" << t << "," << i << "}";

                            IloConstraint cons(x[t][i] == IloBool((fixed_fns[t].count(i) > 0) ? true : false));
                            cons.setName(oss.str().c_str());
                            model.add(cons);
                        }
                    }
                }
            }


            // Set objective
            IloObjective obj;
            IloNumExpr revenue_expr(env);
            IloNumExpr cost_expr(env);

            // Revenues
            for (std::size_t t = 0; t < nslots; ++t)
            {
                for (std::size_t i = 0; i < nfns; ++i)
                {
                    for (std::size_t j = 0; j < nsvcs; ++j)
                    {
                        auto const svc_cat = svc_categories[j];

                        for (std::size_t k = 0; k < nvmcats; ++k)
                        {
                            revenue_expr += fp_svc_cat_revenues[svc_cat]*y[t][i][j][k];
                        }
                    }
                }
            }

            // Costs
            // - Add elecricity costs
            for (std::size_t t = 0; t < nslots; ++t)
            {
                for (std::size_t i = 0; i < nfns; ++i)
                {
                    auto const fn_cat = fn_categories[i];
                    auto const fn_power_state = fn_power_states[i];
                    auto const dC = fn_cat_max_powers[fn_cat]-fn_cat_min_powers[fn_cat];
                    auto const wcost = fp_electricity_cost;

                    // Add elecricity consumption cost
                    cost_expr += (x[t][i]*fn_cat_min_powers[fn_cat]+dC*u[t][i])*wcost;

                    // Add switch-on/off costs
                    if (t > 0)
                    {
                        cost_expr += x[t][i]*(1-x[t-1][i])*fp_fn_cat_awake_costs[fn_cat]/deltat
                                  +  (1-x[t][i])*x[t-1][i]*fp_fn_cat_asleep_costs[fn_cat]/deltat;
                    }
                    else
                    {
                        cost_expr += x[0][i]*IloInt(1-fn_power_state)*fp_fn_cat_awake_costs[fn_cat]/deltat
                                  +  (1-x[0][i])*IloInt(fn_power_state)*fp_fn_cat_asleep_costs[fn_cat]/deltat;
                    }
                }
                // - Add VM (re)allocation costs
                for (std::size_t i = 0; i < nfns; ++i)
                {
                    for (std::size_t j = 0; j < nsvcs; ++j)
                    {
                        for (std::size_t k = 0; k < nvmcats; ++k)
                        {
                            if (t == 0)
                            {
                                // Add VM allocation costs
                                if (fn_vm_allocations[i].count(j) > 0 && fn_vm_allocations[i].at(j).first == k)
                                {
                                    // This FN already hosted some VMs for service j
                                    const IloInt old_y = fn_vm_allocations[i].at(j).second;
                                    cost_expr += IloMax(0, y[0][i][j][k] - old_y)*vm_cat_alloc_costs[k]/deltat;
                                }
                                else
                                {
                                    // No VM (for service j) were already allocated on this host
                                    cost_expr += y[0][i][j][k]*vm_cat_alloc_costs[k]/deltat;
                                }
                            }
                            else
                            {
                                // Add VM reallocation costs
                                cost_expr += IloMax(0, y[t][i][j][k] - y[t-1][i][j][k])*vm_cat_alloc_costs[k]/deltat;
                            }
                        }
                    }
                }
                // - Add service penalties
                for (std::size_t j = 0; j < nsvcs; ++j)
                {
                    auto const svc_cat = svc_categories[j];

                    bool need_vms = true;
                    for (std::size_t k = 0; k < nvmcats && need_vms; ++k)
                    {
                        if (slot_svc_cat_vm_cat_min_num_vms[t][svc_cat][k] == 0)
                        {
                            need_vms = false;
                        }
                    }
                    if (!need_vms)
                    {
                        continue;
                    }

                    IloIntExpr left_expr(env);
/*
                    for (std::size_t i = 0; i < nfns; ++i)
                    {
                        for (std::size_t k = 0; k < nvmcats; ++k)
                        {
                            left_expr += ((y[t][i][j][k] > 0) && (y[t][i][j][k] < slot_svc_cat_vm_cat_min_num_vms[t][svc_cat][k])); // FIXME: check if it works
                            //left_expr += (y[t][i][j][k] > 0)*(y[t][i][j][k] < slot_svc_cat_vm_cat_min_num_vms[t][svc_cat][k]);
                        }
                    }
*/
                    for (std::size_t k = 0; k < nvmcats; ++k)
                    {
                        IloIntExpr ysum_expr(env);

                        for (std::size_t i = 0; i < nfns; ++i)
                        {
                            ysum_expr += y[t][i][j][k];
                        }

                        left_expr += (ysum_expr > 0)*(ysum_expr < slot_svc_cat_vm_cat_min_num_vms[t][svc_cat][k]);
                    }


                    //cost_expr += ((s[t][j] == 0) || left_expr)*fp_svc_cat_penalties[svc_cat]; // Don't work
                    cost_expr += ((s[t][j] == 0) + left_expr)*fp_svc_cat_penalties[svc_cat];
                }
            }

            obj = IloMaximize(env, (revenue_expr-cost_expr)*deltat);
            model.add(obj);


            // Create the CPLEX solver and make 'model' the active ("extracted") model
            IloCP solver(model);

            //write model
#ifndef DCS_DEBUG
            solver.setOut(env.getNullStream());
            solver.setWarning(env.getNullStream());
#else // DCS_DEBUG
            solver.exportModel("opt_multislot_vm_alloc-cp-model.cpo");
            solver.dumpModel("opt_multislot_vm_alloc-cp-model_dump.cpo");
#endif // DCS_DEBUG

            // Set Relative Optimality Tolerance to (rel_tol_*100)%: CP will stop as soon as it has found a feasible solution proved to be within (rel_tol_*100)% of optimal.
            if (math::float_traits<RealT>::definitely_greater(rel_tol_, 0))
            {
                //solver.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, rel_tol_);
                solver.setParameter(IloCP::RelativeOptimalityTolerance, rel_tol_);
            }
            // Set Time Limit to limit the execution time of the search
            if (math::float_traits<RealT>::definitely_greater(time_lim_, 0))
            {
                //solver.setParam(IloCplex::Param::TimeLimit, time_lim_);
                solver.setParameter(IloCP::TimeLimit, time_lim_);
            }
            // Set the search log verbosity to 'terse' (default is 'normal') to
            // limit the amount of data written into the log file in case the
            // search takes very long
            solver.setParameter(IloCP::LogVerbosity, IloCP::Terse);

#ifdef DCS_DEBUG
            detail::dump_cp_settings(solver);
#endif // DCS_DEBUG

            // Try different search types (default is the Restart types)
            // Restart search:
            //solver.setParameter(IloCP::SearchType, IloCP::Restart)); // default
            //solver.setParameter(IloCP::RestartFailLimit, 100); // default
            //solver.setParameter(IloCP::RestartGrowthFactor, 1.15); // default
            // Depth-first search:
            //solver.setParameter(IloCP::SearchType, IloCP::DepthFirst);
            // Multi-point search:
            //solver.setParameter(IloCP::SearchType, IloCP::MultiPoint);
            //solver.setParameter(IloCP::BranchLimit, 10000);

            solver.propagate();
            solution.solved = solver.solve();
            solution.optimal = false;

            IloAlgorithm::Status status = solver.getStatus();
            switch (status)
            {
                case IloAlgorithm::Optimal: // The algorithm found an optimal solution.
                    solution.objective_value = static_cast<RealT>(solver.getObjValue());
                    solution.optimal = true;
                    break;
                case IloAlgorithm::Feasible: // The algorithm found a feasible solution, though it may not necessarily be optimal.

                    solution.objective_value = static_cast<RealT>(solver.getObjValue());
                    dcs::log_warn(DCS_LOGGING_AT, "Optimization problem solved but non-optimal");
                    break;
                case IloAlgorithm::Infeasible: // The algorithm proved the model infeasible (i.e., it is not possible to find an assignment of values to variables satisfying all the constraints in the model).
                case IloAlgorithm::Unbounded: // The algorithm proved the model unbounded.
                case IloAlgorithm::InfeasibleOrUnbounded: // The model is infeasible or unbounded.
                case IloAlgorithm::Error: // An error occurred and, on platforms that support exceptions, that an exception has been thrown.
                case IloAlgorithm::Unknown: // The algorithm has no information about the solution of the model.
                {
                    // Possible CP status (obtained with getInfo):
                    // - IloCP::SearchHasNotFailed: indicates that the search has not failed.
                    // - IloCP::SearchHasFailedNormally: indicates that the search has failed because it has searched the entire search space.
                    // - IloCP::SearchStoppedByLimit: indicates that the search was stopped by a limit, such as a time limit (see IloCP::TimeLimit) or a fail limit (see IloCP::FailLimit ).
                    // - IloCP::SearchStoppedByLabel: indicates that the search was stopped via a fail label which did not exist on any choice point (advanced use).
                    // - IloCP::SearchStoppedByExit: indicates that the search was exited using IloCP::exitSearch.
                    // - IloCP::SearchStoppedByAbort: indicates that the search was stopped by calling IloCP::abortSearch.
                    // - IloCP::UnknownFailureStatus: indicates that the search failed for some other reason.
                    std::ostringstream oss;
                    oss << "Optimization was stopped with status = " << status << " (CP status = " << solver.getInfo(IloCP::FailStatus) << ")";
                    dcs::log_warn(DCS_LOGGING_AT, oss.str());

                    // If the problem was shown to be infeasible, find a minimal
                    // explanation for infeasibility
                    if (solver.refineConflict())
                    {
                        dcs::log_warn(DCS_LOGGING_AT, "Conflict refinement:");
                        solver.writeConflict(DCS_LOGGING_STREAM);
                    }

                    return solution;
                }
            }

#ifdef DCS_DEBUG
            DCS_DEBUG_TRACE( "-------------------------------------------------------------------------------[" );
            DCS_DEBUG_TRACE( "- Objective value: " << solution.objective_value << " (revenue: " << solver.getValue(revenue_expr) << ", cost: " << solver.getValue(cost_expr) << ")");

            DCS_DEBUG_TRACE( "- Decision variables: " );

            // Output x_{t,i}
            for (std::size_t t = 0; t < nslots; ++t)
            {
                for (std::size_t i = 0; i < nfns; ++i)
                {
                    DCS_DEBUG_STREAM << x[t][i].getName() << " = " << solver.getValue(x[t][i]) << " (" << detail::to_IloBool(solver.getValue(x[t][i])) << ")" << std::endl;
                }
            }

            // Output y_{t,i,j,k}
            for (std::size_t t = 0; t < nslots; ++t)
            {
                for (std::size_t i = 0; i < nfns; ++i)
                {
                    for (std::size_t j = 0; j < nsvcs; ++j)
                    {
                        for (std::size_t k = 0; k < nvmcats; ++k)
                        {
                            DCS_DEBUG_STREAM << y[t][i][j][k].getName() << " = " << solver.getValue(y[t][i][j][k]) << " (" << detail::to_IloInt(solver.getValue(y[t][i][j][k])) << ")" << std::endl;
                        }
                    }
                }
            }

            DCS_DEBUG_TRACE( "- Derived variables: " );

            // Output u_{t,i}
            for (std::size_t t = 0; t < nslots; ++t)
            {
                for (std::size_t i = 0; i < nfns; ++i)
                {
                    DCS_DEBUG_STREAM << u[t][i].getName() << " = " << solver.getValue(u[t][i]) << std::endl;
                }
            }

            // Output s_{t,j}
            for (std::size_t t = 0; t < nslots; ++t)
            {
                for (std::size_t j = 0; j < nsvcs; ++j)
                {
                    DCS_DEBUG_STREAM << s[t][j].getName() << " = " << solver.getValue(s[t][j]) << " (" << detail::to_IloBool(solver.getValue(s[t][j])) << ")" << std::endl;
                }
            }

            DCS_DEBUG_TRACE( "]-------------------------------------------------------------------------------" );
#endif // DCS_DEBUG

            // Perform consistency checks

            auto const check_tol = solver.getParameter(IloCP::RelativeOptimalityTolerance);
            auto const check_int_tol = check_tol; //FIXME: what is the CP's counterpart of IloCplex::Param::MIP::Tolerances::Integrality?

            // - Check 'x' variable consistency
            for (std::size_t t = 0; t < nslots; ++t)
            {
                for (std::size_t i = 0; i < nfns; ++i)
                {
#ifdef DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
                    DCS_ASSERT( dcs::math::float_traits<IloNum>::approximately_equal(solver.getValue(x[t][i]), detail::to_IloBool(solver.getValue(x[t][i])), check_int_tol),
                                DCS_EXCEPTION_THROW( std::logic_error,
                                                     "Anomaly in the CP 'x' variable" ) );
#else // DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
                    if (!dcs::math::float_traits<IloNum>::approximately_equal(solver.getValue(x[t][i]), detail::to_IloBool(solver.getValue(x[t][i])), check_int_tol))
                    {
                        std::ostringstream oss;
                        oss << "Anomaly in the CP 'x[" << t << "][" << i << "]' variable - value: " << solver.getValue(x[t][i]) << ", converted-value: " << detail::to_IloBool(solver.getValue(x[t][i])) << " (tol: " << check_int_tol << ")";
                        dcs::log_warn(DCS_LOGGING_AT, oss.str());
                    }
#endif // DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
                }
            }
            // - Check 'y' variable consistency
            for (std::size_t t = 0; t < nslots; ++t)
            {
                for (std::size_t i = 0; i < nfns; ++i)
                {
                    for (std::size_t j = 0; j < nsvcs; ++j)
                    {
                        for (std::size_t k = 0; k < nvmcats; ++k)
                        {
#ifdef DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
                            DCS_ASSERT( dcs::math::float_traits<IloNum>::approximately_equal(solver.getValue(y[t][i][j][k]), detail::to_IloInt(solver.getValue(y[t][i][j][k])), check_int_tol),
                                        DCS_EXCEPTION_THROW( std::logic_error,
                                                             "Anomaly in the CP 'y' variable" ) );
#else // DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
                            if (!dcs::math::float_traits<IloNum>::approximately_equal(solver.getValue(y[t][i][j][k]), detail::to_IloInt(solver.getValue(y[t][i][j][k])), check_int_tol))
                            {
                                std::ostringstream oss;
                                oss << "Anomaly in the CP 'y[" << t << "][" << i << "][" << j << "][" << k << "]' variable - value: " << solver.getValue(y[t][i][j][k]) << ", converted-value: " << detail::to_IloBool(solver.getValue(y[t][i][j][k])) << " (tol: " << check_int_tol << ")";
                                dcs::log_warn(DCS_LOGGING_AT, oss.str());
                            }
#endif // DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
                        }
                    }
                }
            }
            // - Check 's' variable consistency
            for (std::size_t t = 0; t < nslots; ++t)
            {
                for (std::size_t j = 0; j < nsvcs; ++j)
                {
#ifdef DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
                    DCS_ASSERT( dcs::math::float_traits<IloNum>::approximately_equal(solver.getValue(s[t][j]), detail::to_IloBool(solver.getValue(s[t][j])), check_int_tol),
                                DCS_EXCEPTION_THROW( std::logic_error,
                                                     "Anomaly in the CP 's' variable" ) );
#else // DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
                    if (!dcs::math::float_traits<IloNum>::approximately_equal(solver.getValue(s[t][j]), detail::to_IloBool(solver.getValue(s[t][j])), check_int_tol))
                    {
                        std::ostringstream oss;
                        oss << "Anomaly in the CP 's[" << t << "][" << j << "]' variable - value: " << solver.getValue(s[t][j]) << ", converted-value: " << detail::to_IloBool(solver.getValue(s[t][j])) << " (tol: " << check_int_tol << ")";
                        dcs::log_warn(DCS_LOGGING_AT, oss.str());
                    }
#endif // DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
                }
            }
            // - Check objective value consistency
#ifdef DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
            DCS_ASSERT( dcs::math::float_traits<IloNum>::approximately_equal(solver.getObjValue(), (solver.getValue(revenue_expr)-solver.getValue(cost_expr))*deltat, check_tol),
                        DCS_EXCEPTION_THROW( std::logic_error,
                                             "Anomaly in th CP solution: objective value does not match with the difference of revenues and costs" ) );
#else // DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
            if (!dcs::math::float_traits<IloNum>::approximately_equal(solver.getObjValue(), (solver.getValue(revenue_expr)-solver.getValue(cost_expr))*deltat, check_tol))
            {
                std::ostringstream oss;
                oss << "Anomaly in the CP solution - objective value and the difference between revenue and costs do not match - objective value: " << solver.getObjValue() << ", difference: " << (solver.getValue(revenue_expr)-solver.getValue(cost_expr))*deltat << " (tol: " << check_tol << ")";
                dcs::log_warn(DCS_LOGGING_AT, oss.str());
            }
#endif // DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY

            // Fill the solution object

            solution.fn_vm_allocations.resize(nslots);
            solution.fn_cpu_allocations.resize(nslots);
            solution.fn_power_states.resize(nslots);
            solution.profit = solution.objective_value;
            solution.cost = solver.getValue(cost_expr)*deltat;
            solution.revenue = solver.getValue(revenue_expr)*deltat;
            //solution.watts = 0;
            for (std::size_t t = 0; t < nslots; ++t)
            {
                solution.fn_vm_allocations[t].resize(nfns);
                solution.fn_cpu_allocations[t].resize(nfns);
                solution.fn_power_states[t].resize(nfns, 0);
                for (std::size_t i = 0; i < nfns; ++i)
                {
                    //const std::size_t fn = fns[i];

                    solution.fn_power_states[t][i] = detail::to_IloBool(solver.getValue(x[t][i]));
//                  solution.fn_vm_allocations[t][i].resize(nvms);
                    for (std::size_t j = 0; j < nsvcs; ++j)
                    {
                        //const std::size_t vm = vms[j];

                        for (std::size_t k = 0; k < nvmcats; ++k)
                        {
                            auto const num_vms = detail::to_IloInt(solver.getValue(y[t][i][j][k]));
                            if (num_vms > 0)
                            {
                                solution.fn_vm_allocations[t][i][j] = std::make_pair(k, static_cast<std::size_t>(num_vms));
                            }
                        }
                    }
                    //solution.fn_cpu_allocations[t][i] = solver.getValue(u[t][i]);
                    solution.fn_cpu_allocations[t][i] = dcs::math::roundp(solver.getValue(u[t][i]), std::log10(1.0/solver.getParameter(IloCP::RelativeOptimalityTolerance)));
                }
            }

            // Check objective value consistency
#ifdef DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
            DCS_ASSERT( dcs::math::float_traits<RealT>::approximately_equal(solution.objective_value, (solution.revenue-solution.cost)*deltat, check_tol),
                        DCS_EXCEPTION_THROW( std::logic_error,
                                             "Inconsistency in CP solution: objective value does not match with the difference of revenues and costs" ) );
#else // DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
            if (!dcs::math::float_traits<RealT>::approximately_equal(solution.objective_value, (solution.revenue-solution.cost)*deltat, check_tol))
            {
                std::ostringstream oss;
                oss << "Anomaly in the CP solution - objective value and the difference between revenue and costs do not match - objective value: " << solution.objective_value << ", difference: " << (solution.revenue-solution.cost)*deltat << " (tol: " << check_tol << ")";
                dcs::log_warn(DCS_LOGGING_AT, oss.str());
            }
#endif // DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY

            revenue_expr.end();
            cost_expr.end();
            obj.end();
            s.end();
            u.end();
            y.end();
            x.end();

            // Close the Concert Technology app
            env.end();
        }
        catch (const IloException& e)
        {
            std::ostringstream oss;
            oss << "Got exception from CP Optimizer: " << e.getMessage();
            DCS_EXCEPTION_THROW(std::runtime_error, oss.str());
        }
        catch (...)
        {
            DCS_EXCEPTION_THROW(std::runtime_error,
                                "Unexpected error during the optimization");
        }

        return solution;
    }

    multislot_vm_allocation_t<RealT> by_native_cplex(const std::vector<std::size_t>& fn_categories, // Maps every FN to its FN category
                                                     const std::vector<bool>& fn_power_states, // The power status of each FN
                                                     const std::vector<std::map<std::size_t,std::pair<std::size_t,std::size_t>>>& fn_vm_allocations, // Current VM allocations, by FN and service
                                                     const std::vector<std::set<std::size_t>>& fixed_fns, // For each time slot, the set of selected FNs to use for the VM allocation (if in a given time slot the set is empty, any FN can be used)
                                                     const std::vector<RealT>& fn_cat_min_powers, // The min power consumption of FNs by FN category
                                                     const std::vector<RealT>& fn_cat_max_powers, // The max power consumption of FNs by FN category
                                                     const std::vector<std::vector<RealT>>& vm_cat_fn_cat_cpu_specs, // The CPU requirement of VMs by VM category and FN category
                                                     //const std::vector<std::vector<RealT>>& vm_cat_fn_cat_ram_specs, // The RAM requirement of VMs by VM category and FN category
                                                     const std::vector<RealT>& vm_cat_alloc_costs, // The cost to allocate a VM on a FN (e.g., cost to boot a VM or to live-migrate its state), by VM category
                                                     const std::vector<std::size_t>& svc_categories, // Maps every service to its service category
                                                     const std::vector<std::vector<std::vector<std::size_t>>>& slot_svc_cat_vm_cat_min_num_vms, // The min number of VMs required to achieve QoS, by time slot, service category and VM category
                                                     const std::vector<RealT>& fp_svc_cat_revenues, // Monetary revenues by service category
                                                     const std::vector<RealT>& fp_svc_cat_penalties, // Monetary penalties by service category
                                                     const RealT fp_electricity_cost, // Electricty cost (in $/Wh) of FP
                                                     const std::vector<RealT>& fp_fn_cat_asleep_costs, // Cost to power-off a FN by FN category
                                                     const std::vector<RealT>& fp_fn_cat_awake_costs, // Cost to power-on a FN by FN category
                                                     RealT deltat) const // Length of the time interval
    {
        multislot_vm_allocation_t<RealT> solution;

        const std::size_t nfns = fn_categories.size();
        const std::size_t nsvcs = svc_categories.size();
        const std::size_t nvmcats = vm_cat_fn_cat_cpu_specs.size();
        const std::size_t nslots = slot_svc_cat_vm_cat_min_num_vms.size();

        DCS_ASSERT( nfns == fn_categories.size(),
                    DCS_EXCEPTION_THROW( std::logic_error,
                                         "FN categories container has a wrong size" ) );

        DCS_ASSERT( nfns == fn_power_states.size(),
                    DCS_EXCEPTION_THROW( std::logic_error,
                                         "FN power states container has a wrong size" ) );

        DCS_ASSERT( nvmcats == vm_cat_fn_cat_cpu_specs.size(),
                    DCS_EXCEPTION_THROW( std::logic_error,
                                         "VM CPU specs has a wrong size" ) );

        //DCS_ASSERT( nvmcats == vm_cat_fn_cat_ram_specs.size(),
        //            DCS_EXCEPTION_THROW( std::logic_error,
        //                                 "VM RAM specs has a wrong size" ) );

        // Setting up the optimization model
        try
        {
            // Initialize the Concert Technology app
            IloEnv env;

            IloModel model(env);

            model.setName("Max-Profit Optimization");

            // Decision Variables

            // Variables x_{t,i} \in \{0,1\}: 1 if, at time slot t, FN i is to be powered on, 0 otherwise.
            IloArray<IloBoolVarArray> x(env, nslots);
            for (std::size_t t = 0; t < nslots; ++t)
            {
                x[t] = IloBoolVarArray(env, nfns);
                for (std::size_t i = 0; i < nfns; ++i)
                {
                    std::ostringstream oss;
                    oss << "x[" << t << "][" << i << "]";
                    x[t][i] = IloBoolVar(env, oss.str().c_str());
                    model.add(x[t][i]);
                }
            }

            // Variables y_{t,i,j,k} \in \mathbb{N}: denotes the number of class-k VMs are allocated on FN i for service j at time slot t.
            IloArray<IloArray<IloArray<IloIntVarArray>>> y(env, nslots);
            for (std::size_t t = 0; t < nslots; ++t)
            {
                y[t] = IloArray<IloArray<IloIntVarArray>>(env, nfns);

                for (std::size_t i = 0; i < nfns; ++i)
                {
                    y[t][i] = IloArray<IloIntVarArray>(env, nsvcs);

                    for (std::size_t j = 0 ; j < nsvcs ; ++j)
                    {
                        y[t][i][j] = IloIntVarArray(env, nvmcats);

                        for (std::size_t k = 0 ; k < nvmcats ; ++k)
                        {
                            std::ostringstream oss;
                            oss << "y[" << t << "][" << i << "][" << j << "][" << k << "]";
                            y[t][i][j][k] = IloIntVar(env, oss.str().c_str());
                            model.add(y[t][i][j][k]);
                        }
                    }
                }
            }

            // Auxiliary variables s_{t,j} \in \{0,1\}: 1 if service j is offered at time slot t, 0
            // otherwise:
            //   s_{t,j} = \exists i \in F, k \in C : u_{t,i,j,k} > 0
            // which can be defined in CP as:
            //   s_{t,j} = ( \sum_{i \in F} \sum_{k \in C} y_{t,i,j,k} ) > 0
            IloArray<IloBoolVarArray> s(env, nslots);
            for (std::size_t t = 0; t < nslots; ++t)
            {
                s[t] = IloBoolVarArray(env, nsvcs);
                for (std::size_t j = 0; j < nsvcs; ++j)
                {
                    std::ostringstream oss;
                    oss << "s[" << t << "][" << j << "]";
                    s[t][j] = IloBoolVar(env, oss.str().c_str());
                    model.add(s[t][j]);
                }
            }

            // Variable u_{t,i} \in [0,1]: total fraction of CPU of FN i allocated to VMs at time slot t
            //   u_{t,i} = \sum_{j \in S} \sum_{k \in C} y_{t,i,j,k}*C_{i,k}, \forall i \in FN'
            IloArray<IloNumVarArray> u(env, nslots);
            for (std::size_t t = 0; t < nslots; ++t)
            {
                u[t] = IloNumVarArray(env, nfns);
                for (std::size_t i = 0; i < nfns; ++i)
                {
                    std::ostringstream oss;
                    oss << "u[" << t << "][" << i << "]";
                    u[t][i] = IloNumVar(env, oss.str().c_str());
                    model.add(u[t][i]);
                }
            }

            // Constraints

            std::size_t cc = 0; // Constraint counter

            // Constraints the values of s_{t,j}:
            //   s_{t,j} = \exists i \in F, k \in C : y_{t,i,j,k} > 0
            // which can be defined in CP as:
            //   \forall t \in T, j \in S: s_{t,j} = ( \sum_{i \in F} \sum_{k \in C} y_{t,i,j,k} ) > 0)
            ++cc;
            for (std::size_t t = 0; t < nslots; ++t)
            {
                for (std::size_t j = 0; j < nsvcs; ++j)
                {
                    IloIntExpr rhs_expr(env);
                    for (std::size_t i = 0; i < nfns; ++i)
                    {
                        rhs_expr += IloSum(y[t][i][j]);
/*
                        for (std::size_t k = 0; k < nvmcats; ++k)
                        {
                            rhs_expr += y[t][i][j][k];
                        }
*/
                    }
                    //rhs_expr = rhs_expr > 0;

                    std::ostringstream oss;
                    oss << "C" << cc << "_{" << t << "," << j << "}";

                    IloConstraint cons(s[t][j] == (rhs_expr > 0));
                    cons.setName(oss.str().c_str());
                    model.add(cons);
                }
            }

            // Constraints the value of u_{t,i}:
            //   u_{t,i} = \sum_{j \in S} \sum_{k \in C} y_{t,i,j,k}*C_{i,k}, \forall i \in FN'
            for (std::size_t t = 0; t < nslots; ++t)
            {
                for (std::size_t i = 0; i < nfns; ++i)
                {
                    auto const fn_cat = fn_categories[i];

                    IloNumExpr rhs_expr(env);
                    for (std::size_t j = 0; j < nsvcs; ++j)
                    {
                        for (std::size_t k = 0; k < nvmcats; ++k)
                        {
                            rhs_expr += y[t][i][j][k]*vm_cat_fn_cat_cpu_specs[k][fn_cat];
                        }
                    }

                    std::ostringstream oss;
                    oss << "C" << cc << "_{" << t << "," << i << "}";

                    IloConstraint cons(u[t][i] == rhs_expr);
                    cons.setName(oss.str().c_str());
                    model.add(cons);
                }
            }


            // VMs allocated for a given service must belong to the same class
            //  \forall t \in T, j \in S, k_1 \in C : \nexists k2 \in C, k2 \ne k1 : ( \sum_{i \in F} y_{t,i,j,k_1} ) > 0 \wedge ( \sum_{i \in F} y_{t,i,j,k_2} ) > 0
            // which can be defined in CP as:
#if 1
            // Alternative #1
            //  \forall t \in T, j \in S: \sum_{i \in F} \sum_{k \in C} y_{t,i,j,k} = max_{k \in C} \sum_{i \in F} y_{t,i,j,k}
            ++cc;
            for (std::size_t t = 0; t < nslots; ++t)
            {
                for (std::size_t j = 0; j < nsvcs; ++j)
                {
                    IloIntExpr lhs_expr(env); // \sum_{i \in F} \sum_{k \in C} y_{t,i,j,k}
                    for (std::size_t i = 0; i < nfns; ++i)
                    {
/*
                        for (std::size_t k = 0; k < nvmcats; ++k)
                        {
                            lhs_expr += y[t][i][j][k];
                        }
*/
                        lhs_expr += IloSum(y[t][i][j]);
                    }

                    IloIntExprArray max_op_expr(env, nvmcats); // max_{k \in C} \sum_{i \in F} y_{t,i,j,k}
                    for (std::size_t k = 0; k < nvmcats; ++k)
                    {
                        max_op_expr[k] = IloIntExpr(env);
                        for (std::size_t i = 0; i < nfns; ++i)
                        {
                            max_op_expr[k] += y[t][i][j][k]; 
                        }
                    }

                    std::ostringstream oss;
                    oss << "C" << cc << "_{" << t << "," << j << "}";

                    IloConstraint cons(lhs_expr == IloMax(max_op_expr));
                    cons.setName(oss.str().c_str());
                    model.add(cons);
                }
            }
#else
            // Alternative #2
            //  \forall t \in T, j \in S, \forall k_1 \in C : ( ((\sum_{i \in F} y_{t,i,j,k_1}) > 0) \cdot \sum_{k_2 \in C, k_2 > k_1} ((\sum_{i \in F} y_{t,i,j,k_2}) > 0) == 0 )
            ++cc;
            for (std::size_t t = 0; t < nslots; ++t)
            {
                for (std::size_t j = 0; j < nsvcs; ++j)
                {
                    IloIntExpr lhs_expr(env);
                    for (std::size_t k1 = 0; k1 < nvmcats; ++k1)
                    {
                        IloIntExpr ysum1_expr(env);
                        for (std::size_t i = 0; i < nfns; ++i)
                        {
                            ysum1_expr += y[t][i][j][k1];
                        }
                        for (std::size_t k2 = k1+1; k2 < nvmcats; ++k2)
                        {
                            IloIntExpr ysum2_expr(env);
                            for (std::size_t i = 0; i < nfns; ++i)
                            {
                                ysum2_expr += y[t][i][j][k2];
                            }
                            lhs_expr += ysum1_expr*ysum2_expr;
                        }
                    }

                    std::ostringstream oss;
                    oss << "C" << cc << "_{" << t << "," << j << "}";

                    IloConstraint cons(lhs_expr == 0);
                    cons.setName(oss.str().c_str());
                    model.add(cons);
                }
            }
#endif

            // The total allocated capacity on a powered-on FN must not exceed the max capacity
            //  \forall t \in T, i \in F: u_{t,i} \le x_{t,i}
            ++cc;
            for (std::size_t t = 0; t < nslots; ++t)
            {
                for (std::size_t i = 0; i < nfns; ++i)
                {
                    std::ostringstream oss;
                    oss << "C" << cc << "_{" << t << "," << i << "}";

                    IloConstraint cons(u[t][i] <= x[t][i]);
                    cons.setName(oss.str().c_str());
                    model.add(cons);
                }
            }

            // Don't allocate useless VMs
            //  \forall t \in T, j \in S, k \in C: \sum_{i \in F} y_{t,i,j,k} <= N_{t,j,k}
            ++cc;
            for (std::size_t t = 0; t < nslots; ++t)
            {
                for (std::size_t j = 0; j < nsvcs; ++j)
                {
                    auto const svc_cat = svc_categories[j];

                    for (std::size_t k = 0; k < nvmcats; ++k)
                    {
                        IloIntExpr lhs_expr(env);
                        for (std::size_t i = 0; i < nfns; ++i)
                        {
                            lhs_expr += y[t][i][j][k];
                        }

                        std::ostringstream oss;
                        oss << "C" << cc << "_{" << t << "," << j << "," << k << "}";

                        IloConstraint cons(lhs_expr <= IloInt(slot_svc_cat_vm_cat_min_num_vms[t][svc_cat][k]));
                        cons.setName(oss.str().c_str());
                        model.add(cons);
                    }
                }
            }

/*FIXME: this constraint is redundant because it is implied by the above two constraints
            // A VM cannot be allocated to a powered-off FN
            //  \forall t \in T, i \in F, j \in S, k \in C: y_{t,i,j,k} \le x_{t,i} N_{t,j,k}
            ++cc;
            for (std::size_t t = 0; t < nslots; ++t)
            {
                for (std::size_t i = 0; i < nfns; ++i)
                {
                    for (std::size_t j = 0; j < nsvcs; ++j)
                    {
                        auto const svc_cat = svc_categories[j];

                        for (std::size_t k = 0; k < nvmcats; ++k)
                        {
                            std::ostringstream oss;
                            oss << "C" << cc << "_{" << t << "," << i << "," << j << "," << k << "}";

                            IloConstraint cons(y[t][i][j][k] <= x[t][i]*IloInt(slot_svc_cat_vm_cat_min_num_vms[t][svc_cat][k]));
                            cons.setName(oss.str().c_str());
                            model.add(cons);
                        }
                    }
                }
            }
*/

            // Constraints the value of x_{t,i} to force using a selected set of FNs:
            //   x_{t,i} = 1 if fixed_fns.count(i) > 0
            //   x_{t,i} = 0 if fixed_fns.count(i) == 0
            if (fixed_fns.size() > 0)
            {
                ++cc;
                for (std::size_t t = 0; t < nslots; ++t)
                {
                    if (fixed_fns[t].size() > 0)
                    {
                        for (std::size_t i = 0; i < nfns; ++i)
                        {
                            std::ostringstream oss;
                            oss << "C" << cc << "_{" << t << "," << i << "}";

                            IloConstraint cons(x[t][i] == IloBool((fixed_fns[t].count(i) > 0) ? true : false));
                            cons.setName(oss.str().c_str());
                            model.add(cons);
                        }
                    }
                }
            }


            // Set objective
            IloObjective obj;

            // Revenues
            IloNumExpr revenue_expr(env);
            {
                for (std::size_t t = 0; t < nslots; ++t)
                {
                    for (std::size_t i = 0; i < nfns; ++i)
                    {
                        for (std::size_t j = 0; j < nsvcs; ++j)
                        {
                            auto const svc_cat = svc_categories[j];

                            for (std::size_t k = 0; k < nvmcats; ++k)
                            {
                                revenue_expr += fp_svc_cat_revenues[svc_cat]*y[t][i][j][k];
                            }
                        }
                    }
                }
            }

            // Costs
            IloNumExpr cost_expr(env);
            {
                // - Add elecricity costs
                for (std::size_t t = 0; t < nslots; ++t)
                {
                    for (std::size_t i = 0; i < nfns; ++i)
                    {
                        auto const fn_cat = fn_categories[i];
                        auto const fn_power_state = fn_power_states[i];
                        auto const dC = fn_cat_max_powers[fn_cat]-fn_cat_min_powers[fn_cat];
                        auto const wcost = fp_electricity_cost;

                        // Add elecricity consumption cost
                        cost_expr += (x[t][i]*fn_cat_min_powers[fn_cat]+dC*u[t][i])*wcost;

                        // Add switch-on/off costs
                        if (t > 0)
                        {
                            cost_expr += x[t][i]*(1-x[t-1][i])*fp_fn_cat_awake_costs[fn_cat]/deltat
                                      +  (1-x[t][i])*x[t-1][i]*fp_fn_cat_asleep_costs[fn_cat]/deltat;
                        }
                        else
                        {
                            cost_expr += x[0][i]*IloInt(1-fn_power_state)*fp_fn_cat_awake_costs[fn_cat]/deltat
                                      +  (1-x[0][i])*IloInt(fn_power_state)*fp_fn_cat_asleep_costs[fn_cat]/deltat;
                        }
                    }
                    // - Add VM (re)allocation costs
                    for (std::size_t i = 0; i < nfns; ++i)
                    {
                        for (std::size_t j = 0; j < nsvcs; ++j)
                        {
                            for (std::size_t k = 0; k < nvmcats; ++k)
                            {
                                if (t == 0)
                                {
                                    // Add VM allocation costs
                                    if (fn_vm_allocations[i].count(j) > 0 && fn_vm_allocations[i].at(j).first == k)
                                    {
                                        // This FN already hosted some VMs for service j
                                        const IloInt old_y = fn_vm_allocations[i].at(j).second;
                                        cost_expr += IloMax(0, y[0][i][j][k] - old_y)*vm_cat_alloc_costs[k]/deltat;
                                    }
                                    else
                                    {
                                        // No VM (for service j) were already allocated on this host
                                        cost_expr += y[0][i][j][k]*vm_cat_alloc_costs[k]/deltat;
                                    }
                                }
                                else
                                {
                                    // Add VM reallocation costs
                                    cost_expr += IloMax(0, y[t][i][j][k] - y[t-1][i][j][k])*vm_cat_alloc_costs[k]/deltat;
                                }
                            }
                        }
                    }
                    // - Add service penalties
                    for (std::size_t j = 0; j < nsvcs; ++j)
                    {
                        auto const svc_cat = svc_categories[j];

                        bool need_vms = true;
                        for (std::size_t k = 0; k < nvmcats && need_vms; ++k)
                        {
                            if (slot_svc_cat_vm_cat_min_num_vms[t][svc_cat][k] == 0)
                            {
                                need_vms = false;
                            }
                        }
                        if (!need_vms)
                        {
                            continue;
                        }

                        IloIntExpr left_expr(env);

                        for (std::size_t k = 0; k < nvmcats; ++k)
                        {
                            IloIntExpr ysum_expr(env);

                            for (std::size_t i = 0; i < nfns; ++i)
                            {
                                ysum_expr += y[t][i][j][k];
                            }

                            //left_expr += (ysum_expr > 0)*(ysum_expr < slot_svc_cat_vm_cat_min_num_vms[t][svc_cat][k]);
                            left_expr += ((ysum_expr == 0) + (ysum_expr == slot_svc_cat_vm_cat_min_num_vms[t][svc_cat][k])) == 0;
                        }


                        //cost_expr += ((s[t][j] == 0) || left_expr)*fp_svc_cat_penalties[svc_cat]; // Don't work
                        cost_expr += ((s[t][j] == 0) + left_expr)*fp_svc_cat_penalties[svc_cat];
                    }
                }
            }

            obj = IloMaximize(env, (revenue_expr-cost_expr)*deltat);
            model.add(obj);

            // Create the CPLEX solver and make 'model' the active ("extracted") model
            IloCplex solver(model);

            //write model
#ifndef DCS_DEBUG
            solver.setOut(env.getNullStream());
            solver.setWarning(env.getNullStream());
#else // DCS_DEBUG
            solver.exportModel("opt_multislot_vm_alloc-cplex-model.lp");
#endif // DCS_DEBUG

            // Set Relative Optimality Tolerance to (rel_tol_*100)%: CP will stop as soon as it has found a feasible solution proved to be within (rel_tol_*100)% of optimal.
            if (math::float_traits<RealT>::definitely_greater(rel_tol_, 0))
            {
                //solver.setParam(IloCplex::EpGap, relative_gap);
                solver.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, rel_tol_);
            }
            // Set Time Limit to limit the execution time of the search
            if (math::float_traits<RealT>::definitely_greater(time_lim_, 0))
            {
                solver.setParam(IloCplex::Param::TimeLimit, time_lim_);
            }
//            // Set the search log verbosity to 'terse' (default is 'normal') to
//            // limit the amount of data written into the log file in case the
//            // search takes very long
//            solver.setParameter(IloCplex::Param::WriteLevel, IloCP::Terse);

#ifdef DCS_DEBUG
            detail::dump_cplex_settings(solver);
#endif // DCS_DEBUG

#ifdef DCS_FOG_VM_ALLOC_CPLEX_MEMORY_EMPHASIS
            solver.setParam(IloCplex::Param::Emphasis::Memory, 1);
#endif // DCS_FOG_VM_ALLOC_CPLEX_MEMORY_EMPHASIS

            solution.solved = solver.solve();
            solution.optimal = false;

            IloAlgorithm::Status status = solver.getStatus();
            switch (status)
            {
                case IloAlgorithm::Optimal: // The algorithm found an optimal solution.
                    solution.objective_value = static_cast<RealT>(solver.getObjValue());
                    solution.optimal = true;
                    break;
                case IloAlgorithm::Feasible: // The algorithm found a feasible solution, though it may not necessarily be optimal.

                    solution.objective_value = static_cast<RealT>(solver.getObjValue());
                    dcs::log_warn(DCS_LOGGING_AT, "Optimization problem solved but non-optimal");
                    break;
                case IloAlgorithm::Infeasible: // The algorithm proved the model infeasible (i.e., it is not possible to find an assignment of values to variables satisfying all the constraints in the model).
                case IloAlgorithm::Unbounded: // The algorithm proved the model unbounded.
                case IloAlgorithm::InfeasibleOrUnbounded: // The model is infeasible or unbounded.
                case IloAlgorithm::Error: // An error occurred and, on platforms that support exceptions, that an exception has been thrown.
                case IloAlgorithm::Unknown: // The algorithm has no information about the solution of the model.
                {
                    std::ostringstream oss;
                    oss << "Optimization was stopped with status = " << status << " (CPLEX status = " << solver.getCplexStatus() << ", sub-status = " << solver.getCplexSubStatus() << ")";
                    dcs::log_warn(DCS_LOGGING_AT, oss.str());

                    return solution;
                }
            }

#ifdef DCS_DEBUG
            DCS_DEBUG_TRACE( "-------------------------------------------------------------------------------[" );
            DCS_DEBUG_TRACE( "- Objective value: " << solution.objective_value << " (revenue: " << solver.getValue(revenue_expr) << ", cost: " << solver.getValue(cost_expr) << ", deltat: " << deltat << ")");

            DCS_DEBUG_TRACE( "- Decision variables: " );

            // Output x_{t,i}
            for (std::size_t t = 0; t < nslots; ++t)
            {
                for (std::size_t i = 0; i < nfns; ++i)
                {
                    DCS_DEBUG_STREAM << x[t][i].getName() << " = " << solver.getValue(x[t][i]) << " (" << detail::to_IloBool(solver.getValue(x[t][i])) << ")" << std::endl;
                }
            }

            // Output y_{t,i,j,k}
            for (std::size_t t = 0; t < nslots; ++t)
            {
                for (std::size_t i = 0; i < nfns; ++i)
                {
                    for (std::size_t j = 0; j < nsvcs; ++j)
                    {
                        for (std::size_t k = 0; k < nvmcats; ++k)
                        {
                            DCS_DEBUG_STREAM << y[t][i][j][k].getName() << " = " << solver.getValue(y[t][i][j][k]) << " (" << detail::to_IloInt(solver.getValue(y[t][i][j][k])) << ")" << std::endl;
                        }
                    }
                }
            }

            DCS_DEBUG_TRACE( "- Derived variables: " );

            // Output u_{t,i}
            for (std::size_t t = 0; t < nslots; ++t)
            {
                for (std::size_t i = 0; i < nfns; ++i)
                {
                    DCS_DEBUG_STREAM << u[t][i].getName() << " = " << solver.getValue(u[t][i]) << std::endl;
                }
            }

            // Output s_{t,j}
            for (std::size_t t = 0; t < nslots; ++t)
            {
                for (std::size_t j = 0; j < nsvcs; ++j)
                {
                    DCS_DEBUG_STREAM << s[t][j].getName() << " = " << solver.getValue(s[t][j]) << " (" << detail::to_IloBool(solver.getValue(s[t][j])) << ")" << std::endl;
                }
            }

            // Output revenue
            //DCS_DEBUG_STREAM << revenue_expr.getName() << " = " << solver.getValue(revenue_expr) << std::endl;
            DCS_DEBUG_STREAM << "revenue = " << solver.getValue(revenue_expr) << std::endl;

            // Output cost
            //DCS_DEBUG_STREAM << cost_expr.getName() << " = " << solver.getValue(cost_expr) << std::endl;
            DCS_DEBUG_STREAM << "cost = " << solver.getValue(cost_expr) << std::endl;

            DCS_DEBUG_TRACE( "- Computed values: " );

            {
                // Output computed revenue (compare it with the value of the variable 'revenue')
                RealT comp_revenue = 0;
                for (std::size_t t = 0; t < nslots; ++t)
                {
                    for (std::size_t i = 0; i < nfns; ++i)
                    {
                        for (std::size_t j = 0; j < nsvcs; ++j)
                        {
                            auto const svc_cat = svc_categories[j];

                            for (std::size_t k = 0; k < nvmcats; ++k)
                            {
                                comp_revenue += fp_svc_cat_revenues[svc_cat]*detail::to_IloInt(solver.getValue(y[t][i][j][k]));
                            }
                        }
                    }
                }
                DCS_DEBUG_STREAM << "Computed revenue = " << comp_revenue << std::endl;

                // Output computed cost (compare it with the value of the variable 'cost')
                RealT comp_cost = 0;
                // - Add elecricity costs
                for (std::size_t t = 0; t < nslots; ++t)
                {
                    for (std::size_t i = 0; i < nfns; ++i)
                    {
                        auto const fn_cat = fn_categories[i];
                        auto const fn_power_state = fn_power_states[i];
                        auto const dC = fn_cat_max_powers[fn_cat]-fn_cat_min_powers[fn_cat];
                        auto const wcost = fp_electricity_cost;

                        // Add elecricity consumption cost
                        comp_cost += (detail::to_IloBool(solver.getValue(x[t][i]))*fn_cat_min_powers[fn_cat]+dC*solver.getValue(u[t][i]))*wcost;

                        // Add switch-on/off costs
                        if (t > 0)
                        {
                            comp_cost += detail::to_IloBool(solver.getValue(x[t][i]))*(1-detail::to_IloBool(solver.getValue(x[t-1][i])))*fp_fn_cat_awake_costs[fn_cat]/deltat
                                      +  (1-detail::to_IloBool(solver.getValue(x[t][i])))*detail::to_IloBool(solver.getValue(x[t-1][i]))*fp_fn_cat_asleep_costs[fn_cat]/deltat;
                        }
                        else
                        {
                            comp_cost += detail::to_IloBool(solver.getValue(x[0][i]))*(1-fn_power_state)*fp_fn_cat_awake_costs[fn_cat]/deltat
                                      +  (1-detail::to_IloBool(solver.getValue(x[0][i])))*(fn_power_state)*fp_fn_cat_asleep_costs[fn_cat]/deltat;
                        }
                    }
                    // - Add VM (re)allocation costs
                    for (std::size_t i = 0; i < nfns; ++i)
                    {
                        for (std::size_t j = 0; j < nsvcs; ++j)
                        {
                            for (std::size_t k = 0; k < nvmcats; ++k)
                            {
                                if (t == 0)
                                {
                                    // Add VM allocation costs
                                    if (fn_vm_allocations[i].count(j) > 0 && fn_vm_allocations[i].at(j).first == k)
                                    {
                                        // This FN already hosted some VMs for service j
                                        const IloInt old_y = fn_vm_allocations[i].at(j).second;
                                        comp_cost += std::max(IloInt(0), detail::to_IloInt(solver.getValue(y[0][i][j][k])) - old_y)*vm_cat_alloc_costs[k]/deltat;
                                    }
                                    else
                                    {
                                        // No VM (for service j) were already allocated on this host
                                        comp_cost += detail::to_IloInt(solver.getValue(y[0][i][j][k]))*vm_cat_alloc_costs[k]/deltat;
                                    }
                                }
                                else
                                {
                                    // Add VM reallocation costs
                                    comp_cost += std::max(IloInt(0), detail::to_IloInt(solver.getValue(y[t][i][j][k])) - detail::to_IloInt(solver.getValue(y[t-1][i][j][k])))*vm_cat_alloc_costs[k]/deltat;
                                }
                            }
                        }
                    }
                    // - Add service penalties
                    for (std::size_t j = 0; j < nsvcs; ++j)
                    {
                        auto const svc_cat = svc_categories[j];

                        bool need_vms = true;
                        for (std::size_t k = 0; k < nvmcats && need_vms; ++k)
                        {
                            if (slot_svc_cat_vm_cat_min_num_vms[t][svc_cat][k] == 0)
                            {
                                need_vms = false;
                            }
                        }
                        if (!need_vms)
                        {
                            continue;
                        }

                        std::size_t left_term = 0;

                        for (std::size_t k = 0; k < nvmcats; ++k)
                        {
                            std::size_t ysum_term = 0;

                            for (std::size_t i = 0; i < nfns; ++i)
                            {
                                ysum_term += detail::to_IloInt(solver.getValue(y[t][i][j][k]));
                            }

                            //left_term += (ysum_term > 0)*(ysum_term < slot_svc_cat_vm_cat_min_num_vms[t][svc_cat][k]);
                            left_term += ((ysum_term == 0) + (ysum_term == slot_svc_cat_vm_cat_min_num_vms[t][svc_cat][k])) == 0;
                        }


                        //cost_term += ((s[t][j] == 0) || left_term)*fp_svc_cat_penalties[svc_cat]; // Don't work
                        comp_cost += ((detail::to_IloBool(solver.getValue(s[t][j])) == 0) + left_term)*fp_svc_cat_penalties[svc_cat];
                    }
                }
                DCS_DEBUG_STREAM << "Computed cost = " << comp_cost << std::endl;
                DCS_DEBUG_STREAM << "Computed objective value = " << (comp_revenue-comp_cost)*deltat << std::endl;
            }

            DCS_DEBUG_TRACE( "]-------------------------------------------------------------------------------" );
#endif // DCS_DEBUG

            // Perform consistency checks

            auto const check_tol = solver.getParam(IloCplex::Param::MIP::Tolerances::MIPGap);
            auto const check_int_tol = solver.getParam(IloCplex::Param::MIP::Tolerances::Integrality);

            // - Check 'x' variable consistency
            for (std::size_t t = 0; t < nslots; ++t)
            {
                for (std::size_t i = 0; i < nfns; ++i)
                {
#ifdef DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
                    DCS_ASSERT( dcs::math::float_traits<IloNum>::approximately_equal(solver.getValue(x[t][i]), detail::to_IloBool(solver.getValue(x[t][i])), check_int_tol),
                                DCS_EXCEPTION_THROW( std::logic_error,
                                                     "Anomaly in the CPLEX 'x' variable" ) );
#else // DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
                    if (!dcs::math::float_traits<IloNum>::approximately_equal(solver.getValue(x[t][i]), detail::to_IloBool(solver.getValue(x[t][i])), check_int_tol))
                    {
                        std::ostringstream oss;
                        oss << "Anomaly in the CPLEX 'x[" << t << "][" << i << "]' variable - value: " << solver.getValue(x[t][i]) << ", converted-value: " << detail::to_IloBool(solver.getValue(x[t][i])) << " (tol: " << check_int_tol << ")";
                        dcs::log_warn(DCS_LOGGING_AT, oss.str());
                    }
#endif // DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
                }
            }
            // - Check 'y' variable consistency
            for (std::size_t t = 0; t < nslots; ++t)
            {
                for (std::size_t i = 0; i < nfns; ++i)
                {
                    for (std::size_t j = 0; j < nsvcs; ++j)
                    {
                        for (std::size_t k = 0; k < nvmcats; ++k)
                        {
#ifdef DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
                            DCS_ASSERT( dcs::math::float_traits<IloNum>::approximately_equal(solver.getValue(y[t][i][j][k]), detail::to_IloInt(solver.getValue(y[t][i][j][k])), check_int_tol),
                                        DCS_EXCEPTION_THROW( std::logic_error,
                                                             "Anomaly in the CPLEX 'y' variable" ) );
#else // DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
                            if (!dcs::math::float_traits<IloNum>::approximately_equal(solver.getValue(y[t][i][j][k]), detail::to_IloInt(solver.getValue(y[t][i][j][k])), check_int_tol))
                            {
                                std::ostringstream oss;
                                oss << "Anomaly in the CPLEX 'y[" << t << "][" << i << "][" << j << "][" << k << "]' variable - value: " << solver.getValue(y[t][i][j][k]) << ", converted-value: " << detail::to_IloBool(solver.getValue(y[t][i][j][k])) << " (tol: " << check_int_tol << ")";
                                dcs::log_warn(DCS_LOGGING_AT, oss.str());
                            }
#endif // DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
                        }
                    }
                }
            }
            // - Check 's' variable consistency
            for (std::size_t t = 0; t < nslots; ++t)
            {
                for (std::size_t j = 0; j < nsvcs; ++j)
                {
#ifdef DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
                    DCS_ASSERT( dcs::math::float_traits<IloNum>::approximately_equal(solver.getValue(s[t][j]), detail::to_IloBool(solver.getValue(s[t][j])), check_int_tol),
                                DCS_EXCEPTION_THROW( std::logic_error,
                                                     "Anomaly in the CPLEX 's' variable" ) );
#else // DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
                    if (!dcs::math::float_traits<IloNum>::approximately_equal(solver.getValue(s[t][j]), detail::to_IloBool(solver.getValue(s[t][j])), check_int_tol))
                    {
                        std::ostringstream oss;
                        oss << "Anomaly in the CPLEX 's[" << t << "][" << j << "]' variable - value: " << solver.getValue(s[t][j]) << ", converted-value: " << detail::to_IloBool(solver.getValue(s[t][j])) << " (tol: " << check_int_tol << ")";
                        dcs::log_warn(DCS_LOGGING_AT, oss.str());
                    }
#endif // DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
                }
            }
            // - Check objective value consistency
#ifdef DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
            DCS_ASSERT( dcs::math::float_traits<IloNum>::approximately_equal(solver.getObjValue(), (solver.getValue(revenue_expr)-solver.getValue(cost_expr))*deltat, check_tol),
                        DCS_EXCEPTION_THROW( std::logic_error,
                                             "Anomaly in th CPLEX solution: objective value does not match with the difference of revenues and costs" ) );
#else // DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
            if (!dcs::math::float_traits<IloNum>::approximately_equal(solver.getObjValue(), (solver.getValue(revenue_expr)-solver.getValue(cost_expr))*deltat, check_tol))
            {
                std::ostringstream oss;
                oss << "Anomaly in the CPLEX solution - objective value and the difference between revenue and costs do not match - objective value: " << solver.getObjValue() << ", difference: " << (solver.getValue(revenue_expr)-solver.getValue(cost_expr))*deltat << " (tol: " << check_tol << ")";
                dcs::log_warn(DCS_LOGGING_AT, oss.str());
            }
#endif // DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY

            // Fill the solution object

            solution.fn_vm_allocations.resize(nslots);
            solution.fn_cpu_allocations.resize(nslots);
            solution.fn_power_states.resize(nslots);
            solution.profit = solution.objective_value;
#if 1
            solution.cost = solver.getValue(cost_expr)*deltat;
            solution.revenue = solver.getValue(revenue_expr)*deltat;
#else
            solution.revenue = 0;
            for (std::size_t t = 0; t < nslots; ++t)
            {
                for (std::size_t i = 0; i < nfns; ++i)
                {
                    for (std::size_t j = 0; j < nsvcs; ++j)
                    {
                        auto const svc_cat = svc_categories[j];

                        for (std::size_t k = 0; k < nvmcats; ++k)
                        {
                            solution.revenue += fp_svc_cat_revenues[svc_cat]*detail::to_IloInt(solver.getValue(y[t][i][j][k]));
                        }
                    }
                }
            }
            solution.revenue *= deltat;
            solution.cost = 0;
            // - Add elecricity costs
            for (std::size_t t = 0; t < nslots; ++t)
            {
                for (std::size_t i = 0; i < nfns; ++i)
                {
                    auto const fn_cat = fn_categories[i];
                    auto const fn_power_state = fn_power_states[i];
                    auto const dC = fn_cat_max_powers[fn_cat]-fn_cat_min_powers[fn_cat];
                    auto const wcost = fp_electricity_cost;

                    // Add elecricity consumption cost
                    solution.cost += (detail::to_IloBool(solver.getValue(x[t][i]))*fn_cat_min_powers[fn_cat]+dC*solver.getValue(u[t][i]))*wcost;

                    // Add switch-on/off costs
                    if (t > 0)
                    {
                        solution.cost += detail::to_IloBool(solver.getValue(x[t][i]))*(1-detail::to_IloBool(solver.getValue(x[t-1][i])))*fp_fn_cat_awake_costs[fn_cat]/deltat
                                      +  (1-detail::to_IloBool(solver.getValue(x[t][i])))*detail::to_IloBool(solver.getValue(x[t-1][i]))*fp_fn_cat_asleep_costs[fn_cat]/deltat;
                    }
                    else
                    {
                        solution.cost += detail::to_IloBool(solver.getValue(x[0][i]))*(1-fn_power_state)*fp_fn_cat_awake_costs[fn_cat]/deltat
                                      +  (1-detail::to_IloBool(solver.getValue(x[0][i])))*(fn_power_state)*fp_fn_cat_asleep_costs[fn_cat]/deltat;
                    }
                }
                // - Add VM (re)allocation costs
                for (std::size_t i = 0; i < nfns; ++i)
                {
                    for (std::size_t j = 0; j < nsvcs; ++j)
                    {
                        for (std::size_t k = 0; k < nvmcats; ++k)
                        {
                            if (t == 0)
                            {
                                // Add VM allocation costs
                                if (fn_vm_allocations[i].count(j) > 0 && fn_vm_allocations[i].at(j).first == k)
                                {
                                    // This FN already hosted some VMs for service j
                                    const IloInt old_y = fn_vm_allocations[i].at(j).second;
                                    solution.cost += std::max(IloInt(0), detail::to_IloInt(solver.getValue(y[0][i][j][k])) - old_y)*vm_cat_alloc_costs[k]/deltat;
                                }
                                else
                                {
                                    // No VM (for service j) were already allocated on this host
                                    solution.cost += detail::to_IloInt(solver.getValue(y[0][i][j][k]))*vm_cat_alloc_costs[k]/deltat;
                                }
                            }
                            else
                            {
                                // Add VM reallocation costs
                                solution.cost += std::max(IloInt(0), detail::to_IloInt(solver.getValue(y[t][i][j][k])) - detail::to_IloInt(solver.getValue(y[t-1][i][j][k])))*vm_cat_alloc_costs[k]/deltat;
                            }
                        }
                    }
                }
                // - Add service penalties
                for (std::size_t j = 0; j < nsvcs; ++j)
                {
                    auto const svc_cat = svc_categories[j];

                    bool need_vms = true;
                    for (std::size_t k = 0; k < nvmcats && need_vms; ++k)
                    {
                        if (slot_svc_cat_vm_cat_min_num_vms[t][svc_cat][k] == 0)
                        {
                            need_vms = false;
                        }
                    }
                    if (!need_vms)
                    {
                        continue;
                    }

                    std::size_t left_term = 0;

                    for (std::size_t k = 0; k < nvmcats; ++k)
                    {
                        std::size_t ysum_term = 0;

                        for (std::size_t i = 0; i < nfns; ++i)
                        {
                            ysum_term += detail::to_IloInt(solver.getValue(y[t][i][j][k]));
                        }

                        //left_term += (ysum_term > 0)*(ysum_term < slot_svc_cat_vm_cat_min_num_vms[t][svc_cat][k]);
                        left_term += ((ysum_term == 0) + (ysum_term == slot_svc_cat_vm_cat_min_num_vms[t][svc_cat][k])) == 0;
                    }


                    //solution.cost += ((s[t][j] == 0) || left_term)*fp_svc_cat_penalties[svc_cat]; // Don't work
                    solution.cost += ((detail::to_IloBool(solver.getValue(s[t][j])) == 0) + left_term)*fp_svc_cat_penalties[svc_cat];
                }
            }
            solution.cost *= deltat;
#endif
            //solution.watts = 0;
            for (std::size_t t = 0; t < nslots; ++t)
            {
                solution.fn_vm_allocations[t].resize(nfns);
                solution.fn_cpu_allocations[t].resize(nfns);
                solution.fn_power_states[t].resize(nfns, 0);
                for (std::size_t i = 0; i < nfns; ++i)
                {
                    //const std::size_t fn = fns[i];

                    solution.fn_power_states[t][i] = detail::to_IloBool(solver.getValue(x[t][i]));
//                  solution.fn_vm_allocations[t][i].resize(nvms);
                    for (std::size_t j = 0; j < nsvcs; ++j)
                    {
                        //const std::size_t vm = vms[j];

                        for (std::size_t k = 0; k < nvmcats; ++k)
                        {
                            auto const num_vms = detail::to_IloInt(solver.getValue(y[t][i][j][k]));
                            if (num_vms > 0)
                            {
                                solution.fn_vm_allocations[t][i][j] = std::make_pair(k, static_cast<std::size_t>(num_vms));
                            }
                        }
                    }
                    //solution.fn_cpu_allocations[t][i] = solver.getValue(u[t][i]);
                    solution.fn_cpu_allocations[t][i] = dcs::math::roundp(solver.getValue(u[t][i]), std::log10(1.0/solver.getParam(IloCplex::Param::MIP::Tolerances::MIPGap)));
                }
            }

            // Consistency check: <obj value> == (<revenue> - <cost>)*deltat
#ifdef DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
            DCS_ASSERT( dcs::math::float_traits<RealT>::approximately_equal(solution.objective_value, (solution.revenue-solution.cost)*deltat, check_tol),
                        DCS_EXCEPTION_THROW( std::logic_error,
                                             "Inconsistency in CPLEX solution: objective value does not match with the difference of revenues and costs" ) );
#else // DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY
            if (!dcs::math::float_traits<RealT>::approximately_equal(solution.objective_value, (solution.revenue-solution.cost)*deltat, check_tol))
            {
                std::ostringstream oss;
                oss << "Anomaly in the CPLEX solution - objective value and the difference between revenue and costs do not match - objective value: " << solution.objective_value << ", difference: " << (solution.revenue-solution.cost)*deltat << " (tol: " << check_tol << ")";
                dcs::log_warn(DCS_LOGGING_AT, oss.str());
            }
#endif // DCS_FOG_VM_ALLOC_ABORT_ON_ANOMALY

            revenue_expr.end();
            cost_expr.end();
            obj.end();
            s.end();
            u.end();
            y.end();
            x.end();

            // Close the Concert Technology app
            env.end();
        }
#ifdef DCS_DEBUG
        // While debugging it is useful to catch this exception (e.g., to see if some expression is not linear)
        catch (IloAlgorithm::CannotExtractException& cee) 
        {
            std::ostringstream oss;
            oss << "Got exception from Cplex Optimizer - CannoExtractException: " << cee.getMessage() << " (" << cee << ")" << std::endl;

            IloExtractableArray failed = cee.getExtractables(); 

            for (IloInt i = 0; i < failed.getSize(); ++i)
            {
                oss << "\t i = " << i << " ==> Extractable: " << failed[i] << std::endl; 
            }
            DCS_EXCEPTION_THROW(std::runtime_error, oss.str());
        } 
#endif // DCS_DEBUG
        catch (const IloException& e)
        {
            std::ostringstream oss;
            oss << "Got exception from Cplex Optimizer: " << e.getMessage() << " (" << e << ")";
            DCS_EXCEPTION_THROW(std::runtime_error, oss.str());
        }
        catch (...)
        {
            DCS_EXCEPTION_THROW(std::runtime_error,
                                "Unexpected error during the optimization");
        }

        return solution;
    }

private:
    RealT rel_tol_; ///< Relative optimality tolerance used to define optimality (a solution is considered optimal if there does not exist a solution with a better objective function with respect to a relative optimality tolerance).
    RealT time_lim_; ///< Time limit (in seconds) used to set the maximum time the optimizare can spend in search for the best solution.
}; // optimal_multislot_vm_allocation_solver_t

}} // Namespace dcs::fog


#endif // DCS_FOG_VM_ALLOCATION_OPTIMAL_SOLVER_HPP
