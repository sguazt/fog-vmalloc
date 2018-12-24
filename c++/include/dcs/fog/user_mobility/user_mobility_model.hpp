/* vim: set tabstop=4 expandtab shiftwidth=4 softtabstop=4: */

/**
 * \file dcs/fog/user_mobility/user_mobility_model.hpp
 *
 * \brief Base class for user mobility models
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

#ifndef DCS_FOG_USER_MOBILITY_USER_MOBILITY_MODEL_HPP
#define DCS_FOG_USER_MOBILITY_USER_MOBILITY_MODEL_HPP


#include <cstddef>


namespace dcs { namespace fog {

struct user_mobility_model_t
{
    std::size_t next()
    {
        return do_next();
    }

    virtual ~user_mobility_model_t() { }


private:
    virtual std::size_t do_next() = 0;
}; // user_mobility_model_t

}} // Namespace dcs::fog

#endif // DCS_FOG_USER_MOBILITY_USER_MOBILITY_MODEL_HPP
