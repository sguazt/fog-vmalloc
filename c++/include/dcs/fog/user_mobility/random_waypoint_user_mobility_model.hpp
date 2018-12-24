/* vim: set tabstop=4 expandtab shiftwidth=4 softtabstop=4: */

/**
 * \file dcs/fog/user_mobility/random_waypoint_user_mobility.hpp
 *
 * \brief Random waypoint user mobility model.
 *
 * The random waypoint model is a random model for the movement of mobile users,
 * and how their location, velocity and acceleration change over time.
 *
 * REFERENCES:
 * - Mao, Shiwen (2010). "Fundamentals of Communication Networks". Cognitive Radio Communications and Networks. pp. 201–234.
 *   [doi:10.1016/B978-0-12-374715-0.00008-3]
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

#ifndef DCS_FOG_USER_MOBILITY_RANDOM_WAYPOINT_USER_MOBILITY_MODEL_HPP
#define DCS_FOG_USER_MOBILITY_RANDOM_WAYPOINT_USER_MOBILITY_MODEL_HPP


#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <dcs/debug.hpp>
#include <dcs/fog/user_mobility/user_mobility_model.hpp>
#include <dcs/logging.hpp>
#include <fstream>
#include <rndwaypoint/RndWalkPnt.hpp>
#include <stdexcept>


namespace dcs { namespace fog {

/**
 * \brief Random waypoint model.
 *
 * The random waypoint model generates users according to the Random Waypoint
 * model.
 *
 * The random waypoint model is characterized by the following parameters:
 * - num_nodes: the number of nodes.
 * - max_x: the max x dimension of the simulation area.
 * - max_y: the max y dimension of the simulation area.
 * - min_v: the min value for node velocity.
 * - max_wt: the max waiting time for node pauses (0 means no pause time).
 * - seed: the seed for initializing the random number generator
 * .
 */
class random_waypoint_user_mobility_model_t: public user_mobility_model_t
{
private:
    static constexpr std::size_t rwp_num_args = 4;
    static constexpr const char* rwp_py_module = "rndWayPoint";
    static constexpr const char* rwp_py_function = "nextPlease";


public:
    //static constexpr std::size_t default_num_nodes = 300;
    //static constexpr std::size_t default_max_x = 100;
    //static constexpr std::size_t default_max_y = 100;
    static constexpr std::size_t default_min_v = 10;
    static constexpr std::size_t default_max_v = 100;
    static constexpr std::size_t default_max_wt = 0;
    static constexpr std::uint32_t default_seed = 0xffff;


public:
    random_waypoint_user_mobility_model_t(std::size_t num_nodes,
                                          std::size_t max_x,
                                          std::size_t max_y,
                                          std::size_t min_v = default_min_v,
                                          std::size_t max_v = default_max_v,
                                          std::size_t max_wt = default_max_wt,
                                          std::uint32_t seed = default_seed)
    : num_nodes_(num_nodes),
      max_x_(max_x),
      max_y_(max_y),
      min_v_(min_v),
      max_v_(max_v),
      max_wt_(max_wt),
      seed_(seed)
    {
        initialize_py_script();
    }

    ~random_waypoint_user_mobility_model_t()
    {
        finalize_py_script();
    }


private:
    std::size_t do_next()
    {
        auto ret = rwp_.nextPlease();
        if (ret < 0)
        {
            throw std::runtime_error("Error while calling Python script for random waypoint user mobility model");
        }

        return static_cast<std::size_t>(ret);
    }

    void initialize_py_script()
    {
        // Initialize Python script for the mobility model
        // - Write model parameters to file
//FIXME: std::tmpnam is not recommended, better use tempfile or mkstemp
        std::string cfg_fname = std::tmpnam(nullptr);
        std::ofstream ofs(cfg_fname);
        ofs << "[WayPoint]" << std::endl
            << "nr_nodes: " << num_nodes_ << std::endl
            << "MAX_X: " << max_x_ << std::endl
            << "MAX_Y: " << max_y_ << std::endl
            << "MIN_V: " << min_v_ << std::endl
            << "MAX_V: " << max_v_ << std::endl
            << "MAX_WT: " << max_wt_ << std::endl
            //<< "#seed: 0xffff" << std::endl; //FIXME: currently this param is not nused
            << "seed: " << seed_ << std::endl;
        ofs.close();

        // - Prepare the arguments (i.e., module and file names)
        std::size_t sz = 0;
        std::size_t arg = 0;
        // arg 0: not used (however, we can't pass a null pointer because Python doesn't like it)
        sz = 1;
        rwp_args_[arg] = new char[sz];
        std::strncpy(rwp_args_[arg], "", 1);
        // arg 1: module name
        ++arg;
        sz = std::strlen(rwp_py_module)+1;
        rwp_args_[arg] = new char[sz];
        std::strncpy(rwp_args_[arg], rwp_py_module, sz);
        // arg 2: function name
        ++arg;
        sz = std::strlen(rwp_py_function)+1;
        rwp_args_[arg] = new char[sz];
        std::strncpy(rwp_args_[arg], rwp_py_function, sz);
        // arg 3: config file name
        ++arg;
        sz = cfg_fname.size()+1;
        rwp_args_[arg] = new char[sz];
        std::strncpy(rwp_args_[arg], cfg_fname.c_str(), sz);
        // sanity check
        DCS_DEBUG_ASSERT( rwp_num_args == rwp_args_.size() );
        // - Setup
        auto ret = rwp_.setup(rwp_args_.size(), rwp_args_.data());
        if (ret < 0)
        {
            throw std::runtime_error("Unable to initialize the Python script");
        }
    }

    void finalize_py_script()
    {
        auto ret = rwp_.dismiss();
        if (ret < 0)
        {
            dcs::log_warn(DCS_LOGGING_AT, "Problems when finalizing the Python script");
        }

        for (std::size_t i = 0; i < rwp_args_.size(); ++i)
        {
            if (rwp_args_[i])
            {
                delete[] rwp_args_[i];
                rwp_args_[i] = 0;
            }
        }
    }


private:
    std::size_t num_nodes_;
    std::size_t max_x_;
    std::size_t max_y_;
    std::size_t min_v_;
    std::size_t max_v_;
    std::size_t max_wt_;
    std::uint32_t seed_;
#if 1 //FIXME: enable the code fragment below to use std:array instead of plain char**
    std::array<char*,rwp_num_args> rwp_args_; /// Arguments to pass to the Python script for the mobility model
#else
    char** rwp_args_; /// Arguments to pass to the Python script for the mobility model
#endif
    RndWalkPnt rwp_; ///< Interface to the Python script for the mobility model
}; // random_waypoint_user_mobility_model_t

}} // Namespace dcs::fog

#endif // DCS_FOG_USER_MOBILITY_RANDOM_WAYPOINT_USER_MOBILITY_MODEL_HPP
