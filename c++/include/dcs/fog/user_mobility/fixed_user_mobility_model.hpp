/* vim: set tabstop=4 expandtab shiftwidth=4 softtabstop=4: */

/**
 * \file dcs/fog/user_mobility/fixed_user_mobility.hpp
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

#ifndef DCS_FOG_USER_MOBILITY_FIXED_USER_MOBILITY_MODEL_HPP
#define DCS_FOG_USER_MOBILITY_FIXED_USER_MOBILITY_MODEL_HPP


#include <cstddef>
#include <dcs/fog/user_mobility/user_mobility_model.hpp>


namespace dcs { namespace fog {

/**
 * \brief Fixed user mobility model.
 *
 * The fixed user mobility model always generates the same number of nodes.
 *
 * This model is characterized by the following parameters:
 * - num_nodes: the number of nodes.
 * .
 */
class fixed_user_mobility_model_t: public user_mobility_model_t
{
public:
    fixed_user_mobility_model_t(std::size_t num_nodes)
    : num_nodes_(num_nodes)
    {
    }


private:
    std::size_t do_next()
    {
        return num_nodes_;
    }


private:
    std::size_t num_nodes_;
}; // fixed_user_mobility_model_t

}} // Namespace dcs::fog

#endif // DCS_FOG_USER_MOBILITY_FIXED_USER_MOBILITY_MODEL_HPP
