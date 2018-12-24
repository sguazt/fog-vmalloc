/* vim: set tabstop=4 expandtab shiftwidth=4 softtabstop=4: */

/**
 * \file src/fog_vmalloc.cpp
 *
 * \brief Application entry point.
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


#include <cstddef>
#include <chrono>
#include <ctime>
#include <dcs/assert.hpp>
#include <dcs/debug.hpp>
#include <dcs/cli.hpp>
#include <dcs/debug.hpp>
#include <dcs/exception.hpp>
#include <dcs/fog/detail/version.hpp>
#include <dcs/fog/experiment.hpp>
#include <dcs/fog/user_mobility.hpp>
#include <dcs/fog/scenario.hpp>
#include <dcs/logging.hpp>
#include <exception>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>


namespace cli = dcs::cli;
namespace fog = dcs::fog;


namespace /*<unnamed>*/ { namespace detail {

class cli_options_t;

template <typename CharT, typename CharTraitsT>
std::basic_ostream<CharT,CharTraitsT>& operator<<(std::basic_ostream<CharT,CharTraitsT>& os, const cli_options_t& opts);

cli_options_t parse_cli_options(int argc, char* argv[]);

void usage(char const* progname);
void version(char const* progname);


struct cli_options_t
{
    static constexpr double default_optim_relative_tolerance = 0;
    static constexpr double default_optim_time_limit = -1;
    static const unsigned long default_rng_seed = 5489U;
    static constexpr double default_sim_ci_level = 0.95;
    static constexpr double default_sim_ci_rel_precision = 0.04;
    static const std::size_t default_sim_max_num_replications = 0;
    static constexpr double default_sim_max_replication_duration = 0;
    static const int default_verbosity = 0;


    cli_options_t()
    : help(false),
      optim_relative_tolerance(default_optim_relative_tolerance),
      optim_time_limit(default_optim_time_limit),
      rng_seed(default_rng_seed),
      sim_ci_level(default_sim_ci_level),
      sim_ci_rel_precision(default_sim_ci_rel_precision),
      sim_max_num_replications(default_sim_max_num_replications),
      sim_max_replication_duration(default_sim_max_replication_duration),
      test(false),
      verbosity(default_verbosity),
      version(false)
    {
    }


    bool help;
    double optim_relative_tolerance; ///< The relative tolerance option to set to the optimizer
    double optim_time_limit; ///< The time limit option to set to the optimizer
    std::string output_stats_data_file; ///< The path to the output stats data file
    std::string output_trace_data_file; ///< The path to the output trace data file
    unsigned long rng_seed; ///< The seed used for random number generation
    std::string scenario_file; ///< The path to the input scenario file
    double sim_ci_level; ///< Level for confidence intervals
    double sim_ci_rel_precision; ///< Relative precision for the half-width of the confidence intervals
    std::size_t sim_max_num_replications; ///< Maximum number of replications (0 means 'unlimited')
    double sim_max_replication_duration; ///< Length of each replication (in terms of simulated time)
    bool test; ///< Show experimental settings without running any experiment
    int verbosity; ///< The verbosity level: 0 for 'minimum' and 9 for 'maximum' verbosity level
    bool version; ///< Show version information
}; // cli_options_t


cli_options_t parse_cli_options(int argc, char* argv[])
{
    std::string opt_str;
    cli_options_t opt;

    DCS_DEBUG_TRACE("Parse CLI options...");//XXX

    // First parse options that make this program to display a message and then exit.

    opt.help = cli::simple::get_option(argv, argv+argc, "--help");
    if (opt.help)
    {
        return opt;
    }

    opt.version = cli::simple::get_option(argv, argv+argc, "--version");
    if (opt.version)
    {
        return opt;
    }

    // Then parse the reamining options

    opt.optim_relative_tolerance = cli::simple::get_option<double>(argv, argv+argc, "--optim-reltol", opt.default_optim_relative_tolerance);
    opt.optim_time_limit = cli::simple::get_option<double>(argv, argv+argc, "--optim-tilim", opt.default_optim_time_limit);
    opt.output_stats_data_file = cli::simple::get_option<std::string>(argv, argv+argc, "--out-stats-file");
    opt.output_trace_data_file = cli::simple::get_option<std::string>(argv, argv+argc, "--out-trace-file");
    opt.rng_seed = cli::simple::get_option<unsigned long>(argv, argv+argc, "--rng-seed", opt.default_rng_seed);
    opt.scenario_file = cli::simple::get_option<std::string>(argv, argv+argc, "--scenario");
    opt.sim_ci_level = cli::simple::get_option<double>(argv, argv+argc, "--sim-ci-level", opt.default_sim_ci_level);
    opt.sim_ci_rel_precision = cli::simple::get_option<double>(argv, argv+argc, "--sim-ci-rel-precision", opt.default_sim_ci_rel_precision);
    opt.sim_max_num_replications = cli::simple::get_option<std::size_t>(argv, argv+argc, "--sim-max-num-rep", opt.default_sim_max_num_replications);
    opt.sim_max_replication_duration = cli::simple::get_option<double>(argv, argv+argc, "--sim-max-rep-len", opt.default_sim_max_replication_duration);
    opt.test = cli::simple::get_option(argv, argv+argc, "--test");
    opt.verbosity = cli::simple::get_option<short>(argv, argv+argc, "--verbosity", opt.default_verbosity);
    if (opt.verbosity < 0)
    {
        opt.verbosity = 0;
    }
    else if (opt.verbosity > 9)
    {
        opt.verbosity = 9;
    }

    // Check CLI options
    if (opt.scenario_file.empty())
    {
        DCS_EXCEPTION_THROW( std::invalid_argument, "Scenario file not specified" );
    }

    return opt;
}

template <typename CharT, typename CharTraitsT>
std::basic_ostream<CharT,CharTraitsT>& operator<<(std::basic_ostream<CharT,CharTraitsT>& os, const cli_options_t& opts)
{
    os  << "help: " << opts.help
        << ", optim-relative-tolerance: " << opts.optim_relative_tolerance
        << ", optim-time-limit: " << opts.optim_time_limit
        << ", output-stats-data-file: " << opts.output_stats_data_file
        << ", output-trace-data-file: " << opts.output_trace_data_file
        << ", random-generator-seed: " << opts.rng_seed
        << ", scenario-file: " << opts.scenario_file
        << ", sim-ci-level: " << opts.sim_ci_level
        << ", sim-ci-relative-precision: " << opts.sim_ci_rel_precision
        << ", sim-max-num-replications: " << opts.sim_max_num_replications
        << ", sim-max-replication-duration: " << opts.sim_max_replication_duration
        << ", test: " << opts.test
        << ", verbosity: " << opts.verbosity
        << ", version: " << opts.version;

    return os;
}

void usage(char const* progname)
{
    std::cout << "Usage: " << progname << " [options]" << std::endl
              << "Options:" << std::endl
              << "--help" << std::endl
              << "  Show this message." << std::endl
              << "--optim-reltol <num>" << std::endl
              << "  Real number in [0,1] denoting the relative tolerance parameter in the optimizer." << std::endl
              << "--optim-tilim <num>" << std::endl
              << "  Real positive number denoting the maximum number of seconds to wait for the termination of the optimizer." << std::endl
              << "--out-stats-file <file>" << std::endl
              << "  The output file where writing statistics." << std::endl
              << "--out-trace-file <file>" << std::endl
              << "  The output file where writing run-trace information." << std::endl
              << "--rng-seed <num>" << std::endl
              << "  Set the seed to use for random number generation." << std::endl
              << "--scenario <file>" << std::endl
              << "  The path to the file describing the scenario to use for the experiment." << std::endl
              << "--sim-ci-level <num>" << std::endl
              << "  Level for the confidence intervals (must be a number in [0,1])." << std::endl
              << "--sim-ci-rel-precision <num>" << std::endl
              << "  Relative precision for the half-width of the confidence intervals (must be a number in [0,1])." << std::endl
              << "--sim-max-rep-len <num>" << std::endl
              << "  Real number >= 0 denoting the maximum duration of each independent replication." << std::endl
              << "--sim-max-num-rep <num>" << std::endl
              << "  Integer number >= 0 denoting the maximum number of independent replications. Use 0 for an unlimited number of replications." << std::endl
              << "--test" << std::endl
              << "  Show the experiment settings without running any experiment." << std::endl
              << "--verbosity <num>" << std::endl
              << "  An integer number in [0,9] representing the verbosity level (0 for 'minimum verbosity' and 9 for 'maximum verbosity)." << std::endl
              << "--version" << std::endl
              << "  Show a version message and exit." << std::endl
              << std::endl;
}

void version(char const* progname)
{
    std::cout << progname << " version " << DCS_FOG_VM_ALLOC_DETAIL_VERSION_STR << std::endl;
}

template <typename RealT, typename RNGT>
void run_experiment(const fog::scenario_t<RealT>& scen, const cli_options_t& opts, RNGT& rng)
{
    fog::experiment_t<RealT> exp;

    // Setup experiment
    // - Load scenario
    exp.num_fog_node_categories(scen.num_fn_categories);
    exp.num_service_categories(scen.num_svc_categories);
    exp.num_virtual_machine_categories(scen.num_vm_categories);
    exp.service_arrival_rates(scen.svc_arrival_rates.begin(), scen.svc_arrival_rates.end());
    exp.max_service_arrival_rates(scen.svc_max_arrival_rates.begin(), scen.svc_max_arrival_rates.end());
    exp.max_service_delays(scen.svc_max_delays.begin(), scen.svc_max_delays.end());
    //exp.virtual_machine_categories(scen.svc_vm_categories.begin(), scen.svc_vm_categories.end());
    exp.virtual_machine_service_rates(scen.svc_vm_service_rates.begin(), scen.svc_vm_service_rates.end());
    exp.num_services(scen.fp_num_svcs.begin(), scen.fp_num_svcs.end());
    exp.num_fog_nodes(scen.fp_num_fns.begin(), scen.fp_num_fns.end());
    exp.electricity_costs(scen.fp_electricity_costs);
    exp.service_revenues(scen.fp_svc_revenues.begin(), scen.fp_svc_revenues.end());
    exp.service_penalties(scen.fp_svc_penalties.begin(), scen.fp_svc_penalties.end());
    exp.fog_node_asleep_costs(scen.fp_fn_asleep_costs.begin(), scen.fp_fn_asleep_costs.end());
    exp.fog_node_awake_costs(scen.fp_fn_awake_costs.begin(), scen.fp_fn_awake_costs.end());
    exp.fog_node_min_power_consumptions(scen.fn_min_powers.begin(), scen.fn_min_powers.end());
    exp.fog_node_max_power_consumptions(scen.fn_max_powers.begin(), scen.fn_max_powers.end());
    exp.virtual_machine_cpu_requirements(scen.vm_cpu_requirements.begin(), scen.vm_cpu_requirements.end());
    exp.virtual_machine_ram_requirements(scen.vm_ram_requirements.begin(), scen.vm_ram_requirements.end());
    exp.virtual_machine_allocation_costs(scen.vm_allocation_costs.begin(), scen.vm_allocation_costs.end());

    exp.fp_vm_allocation_trigger_interval(scen.fp_vm_allocation_interval);
    exp.service_arrival_rate_estimation(scen.svc_arrival_rate_estimation);
    exp.service_arrival_rate_estimation_params(scen.svc_arrival_rate_estimation_params.begin(), scen.svc_arrival_rate_estimation_params.end());
    exp.service_delay_tolerance(scen.svc_delay_tolerance);

    // - Add options
    exp.max_num_replications(opts.sim_max_num_replications);
    exp.max_replication_duration(opts.sim_max_replication_duration);
    exp.confidence_interval_level(opts.sim_ci_level);
    exp.confidence_interval_relative_precision(opts.sim_ci_rel_precision);
    exp.output_stats_data_file(opts.output_stats_data_file);
    exp.output_trace_data_file(opts.output_trace_data_file);
    exp.verbosity_level(opts.verbosity);
    //exp.service_delay_tolerance(opts.service_delay_tolerance);
    exp.optimization_relative_tolerance(opts.optim_relative_tolerance);
    exp.optimization_max_duration(opts.optim_time_limit);
    //exp.service_arrival_rate_estimation(opts.service_arrival_rate_estimation);
    //exp.fp_penalty_policy(opts.fp_penalty_policy);
    //exp.fp_penalty_model(opts.fp_penalty_model);
    //exp.fp_revenue_policy(opts.fp_revenue_policy);
    //exp.fp_vm_allocation_policy(opts.vm_allocation_policy);

    exp.random_number_generator(rng);
    std::shared_ptr<fog::user_mobility_model_t> p_usr_mob_model;
    switch (scen.svc_user_mobility_model)
    {
        case fog::fixed_user_mobility_model:
            {
                std::size_t num_users = 0;

                if (scen.svc_user_mobility_model_params.count("n") == 0)
                {
                    DCS_EXCEPTION_THROW( std::invalid_argument, "Missing one or more mandatory parameters of the fixed user mobility model" );
                }

                if (scen.svc_user_mobility_model_params.count("n") > 0)
                {
                    std::istringstream iss(scen.svc_user_mobility_model_params.at("n").back());
                    iss >> num_users;
                }

                p_usr_mob_model = std::make_shared<fog::fixed_user_mobility_model_t>(num_users);
            }
            break;
        case fog::random_waypoint_user_mobility_model:
            {
                std::size_t num_nodes = 0;
                std::size_t max_x = 0;
                std::size_t max_y = 0;
                auto min_v = fog::random_waypoint_user_mobility_model_t::default_min_v;
                auto max_v = fog::random_waypoint_user_mobility_model_t::default_max_v;
                auto max_wt = fog::random_waypoint_user_mobility_model_t::default_max_wt;
                auto seed = fog::random_waypoint_user_mobility_model_t::default_seed;

                if (scen.svc_user_mobility_model_params.count("nr_nodes") == 0
                    || scen.svc_user_mobility_model_params.count("max_x") == 0
                    || scen.svc_user_mobility_model_params.count("max_y") == 0)
                {
                    DCS_EXCEPTION_THROW( std::invalid_argument, "Missing one or more mandatory parameters of the random waypoint user mobility model" );
                }

                if (scen.svc_user_mobility_model_params.count("nr_nodes") > 0)
                {
                    std::istringstream iss(scen.svc_user_mobility_model_params.at("nr_nodes").back());
                    iss >> num_nodes;
                }
                if (scen.svc_user_mobility_model_params.count("max_x") > 0)
                {
                    std::istringstream iss(scen.svc_user_mobility_model_params.at("max_x").back());
                    iss >> max_x;
                }
                if (scen.svc_user_mobility_model_params.count("max_y") > 0)
                {
                    std::istringstream iss(scen.svc_user_mobility_model_params.at("max_y").back());
                    iss >> max_y;
                }
                if (scen.svc_user_mobility_model_params.count("min_v") > 0)
                {
                    std::istringstream iss(scen.svc_user_mobility_model_params.at("min_v").back());
                    iss >> min_v;
                }
                if (scen.svc_user_mobility_model_params.count("max_v") > 0)
                {
                    std::istringstream iss(scen.svc_user_mobility_model_params.at("max_v").back());
                    iss >> max_v;
                }
                if (scen.svc_user_mobility_model_params.count("max_wt") > 0)
                {
                    std::istringstream iss(scen.svc_user_mobility_model_params.at("max_wt").back());
                    iss >> max_wt;
                }
                if (scen.svc_user_mobility_model_params.count("seed") > 0)
                {
                    std::istringstream iss(scen.svc_user_mobility_model_params.at("seed").back());
                    iss >> seed;
                }

                p_usr_mob_model = std::make_shared<fog::random_waypoint_user_mobility_model_t>(num_nodes, max_x, max_y, min_v, max_v, max_wt, seed);
            }
            break;
        case fog::step_user_mobility_model:
            {
                std::vector<std::size_t> num_users_seq;

                if (scen.svc_user_mobility_model_params.count("n") == 0)
                {
                    DCS_EXCEPTION_THROW( std::invalid_argument, "Missing one or more mandatory parameters of the fixed user mobility model" );
                }

                if (scen.svc_user_mobility_model_params.count("n") > 0)
                {
                    for (auto const& str_val : scen.svc_user_mobility_model_params.at("n"))
                    {
                        std::size_t num_users = 0;

                        std::istringstream iss(str_val);
                        iss >> num_users;
                        num_users_seq.push_back(num_users);
                    }
                }

                p_usr_mob_model = std::make_shared<fog::step_user_mobility_model_t>(num_users_seq.begin(), num_users_seq.end());
            }
            break;
    }
    //exp.user_mobility_model(std::make_shared<fog::random_waypoint_user_mobility_model_t>(300, 100, 100));
    exp.user_mobility_model(p_usr_mob_model);
    std::shared_ptr<fog::base_vm_allocation_solver_t<RealT>> p_vm_alloc_solver;
    std::shared_ptr<fog::base_multislot_vm_allocation_solver_t<RealT>> p_multislot_vm_alloc_solver;
    switch (scen.fp_vm_allocation_policy)
    {
        case fog::optimal_vm_allocation_policy:
            p_vm_alloc_solver = std::make_shared<fog::optimal_vm_allocation_solver_t<RealT>>(opts.optim_relative_tolerance, opts.optim_time_limit);
            p_multislot_vm_alloc_solver = std::make_shared<fog::optimal_multislot_vm_allocation_solver_t<RealT>>(opts.optim_relative_tolerance, opts.optim_time_limit);
            break;
        case fog::bahreini2017_match_vm_allocation_policy:
            p_vm_alloc_solver = std::make_shared<fog::bahreini2017_mcappim_vm_allocation_solver_t<RealT>>();
            p_multislot_vm_alloc_solver = std::make_shared<fog::optimal_multislot_vm_allocation_solver_t<RealT>>(opts.optim_relative_tolerance, opts.optim_time_limit); //FIXME: multislot VM allocation uses optimal VM allocation policy
            break;
        case fog::bahreini2017_match_alt_vm_allocation_policy:
            p_vm_alloc_solver = std::make_shared<fog::bahreini2017_mcappim_alt_vm_allocation_solver_t<RealT>>();
            p_multislot_vm_alloc_solver = std::make_shared<fog::optimal_multislot_vm_allocation_solver_t<RealT>>(opts.optim_relative_tolerance, opts.optim_time_limit); //FIXME: multislot VM allocation uses optimal VM allocation policy
            break;
    }
    exp.vm_allocation_solver(p_vm_alloc_solver);
    exp.multislot_vm_allocation_solver(p_multislot_vm_alloc_solver);
//        std::unique_ptr<base_multislot_vm_allocation_solver_t<RealT>> p_vm_alloc_solver;
//
//        p_vm_alloc_solver = std::make_unique<optimal_multislot_vm_allocation_solver_t<RealT>>(optim_relative_tolerance_, optim_time_limit_);


    // Run experiment

    auto start_clock = std::chrono::system_clock::now();
    auto start_time = std::chrono::system_clock::to_time_t(start_clock);

    DCS_LOGGING_STREAM << "****************************************************************" << std::endl;
    DCS_LOGGING_STREAM << "**** [" << std::put_time(std::localtime(&start_time), "%c %Z") << "]" << std::endl;
    DCS_LOGGING_STREAM << "**** SCENARIO: " << scen << std::endl;
    DCS_LOGGING_STREAM << "**** OPTIONS: " << opts << std::endl;
    DCS_LOGGING_STREAM << "****************************************************************" << std::endl;
    DCS_LOGGING_STREAM << "**** RUNNING EXPERIMENT: " << exp << std::endl;

    if (!opts.test)
    {
        exp.run();
    }
    else
    {
        DCS_LOGGING_STREAM << "Test only. Nothing to run." << std::endl;
    }

    auto stop_clock = std::chrono::system_clock::now();
    auto stop_time = std::chrono::system_clock::to_time_t(stop_clock);
    std::chrono::duration<double> elapsed_seconds = stop_clock-start_clock;

    DCS_LOGGING_STREAM << "**** ELAPSED TIME: " << elapsed_seconds.count() << "s" << std::endl;
    DCS_LOGGING_STREAM << "**** [" << std::put_time(std::localtime(&stop_time), "%c %Z") << "]" << std::endl;
    DCS_LOGGING_STREAM << "****************************************************************" << std::endl;
}

}} // Namespace <unnamed>::detail



int main(int argc, char* argv[])
{
    typedef double real_t;

    try
    {
        detail::cli_options_t cli_opts;
        cli_opts = detail::parse_cli_options(argc, argv);
        if (cli_opts.help)
        {
            detail::usage(argv[0]);
            return 0;
        }
        if (cli_opts.version)
        {
            detail::version(argv[0]);
            return 0;
        }
        DCS_DEBUG_TRACE("Options: " << cli_opts);

        // Prepare the experiment
        DCS_DEBUG_TRACE("Preparing the experiment...");//XXX
        fog::scenario_t<real_t> scenario;
        scenario = fog::make_scenario<real_t>(cli_opts.scenario_file);
        DCS_DEBUG_TRACE("Scenario: " << scenario);

        //std::default_random_engine rng(cli_opts.rng_seed);
        fog::random_number_engine_t rng(cli_opts.rng_seed);

        // Run the experiment
        DCS_DEBUG_TRACE("Runnig the experiment...");//XXX
        detail::run_experiment(scenario, cli_opts, rng);
    }
    catch (const std::invalid_argument& ia)
    {
        dcs::log_error(DCS_LOGGING_AT, ia.what());
        detail::usage(argv[0]);
        return 1;
    }
    catch (const std::exception& e)
    {
        dcs::log_error(DCS_LOGGING_AT, e.what());
        return 1;
    }
}
