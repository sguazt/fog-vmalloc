/* vim: set tabstop=4 expandtab shiftwidth=4 softtabstop=4: */

/**
 * \file dcs/fog/user_mobility/step_user_mobility.hpp
 *
 * \brief Fixed user mobility model.
 *
 * In this model, the user position never changes.
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

#ifndef DCS_FOG_USER_MOBILITY_STEP_USER_MOBILITY_MODEL_HPP
#define DCS_FOG_USER_MOBILITY_STEP_USER_MOBILITY_MODEL_HPP


#include <cstddef>
#include <dcs/fog/user_mobility/user_mobility_model.hpp>
#include <initializer_list>
#include <vector>


namespace dcs { namespace fog {

/**
 * \brief Step user mobility model.
 *
 * The step user mobility model is characterized by the following parameters:
 * - [num_nodes]: a sequence of number of nodes for each step.
 * .
 * For instance:
 *   [2 6 4]
 * the first call to this model returns 2, then returns 6, then returns 4, and then restarts from the begining, thus generating the following step function:
 *   #nodes
 *  /|\
 *  6|   *     *
 *  5|
 *  4|     *     *
 *  3|
 *  2| *     *     *
 *  1|
 *   |--------------> time
 *   0 1 2 3 4 5 6 7
 */
class step_user_mobility_model_t: public user_mobility_model_t
{
public:
    template <typename IterT>
    step_user_mobility_model_t()
    : next_idx_(0)
    {
    }

    template <typename IterT>
    step_user_mobility_model_t(IterT first, IterT last)
    : num_nodes_seq_(first, last),
      next_idx_(0)
    {
    }

    template <typename UIntT>
    step_user_mobility_model_t(std::initializer_list<UIntT> num_nodes_list)
    : num_nodes_seq_(num_nodes_list.begin(), num_nodes_list.end()),
      next_idx_(0)
    {
    }


private:
    std::size_t do_next()
    {
        return num_nodes_seq_[next_idx_++ % num_nodes_seq_.size()];
    }


private:
    std::vector<std::size_t> num_nodes_seq_;
    std::size_t next_idx_;
}; // step_user_mobility_model_t

}} // Namespace dcs::fog

#endif // DCS_FOG_USER_MOBILITY_STEP_USER_MOBILITY_MODEL_HPP
