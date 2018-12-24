/* vim: set tabstop=4 expandtab shiftwidth=4 softtabstop=4: */

/**
 * \file dcs/fog/io.hpp
 *
 * \brief Input/Output utilities.
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

#ifndef DCS_FOG_IO_HPP
#define DCS_FOG_IO_HPP


#include <iostream>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <utility>


namespace dcs { namespace fog {

template <typename CharT, typename CharTraitsT, typename T1, typename T2>
std::basic_ostream<CharT,CharTraitsT>& operator<<(std::basic_ostream<CharT,CharTraitsT>& os, const std::pair<T1,T2>& p)
{
    os << "<" << p.first << ", " << p.second << ">";

    return os;
}

template <typename CharT, typename CharTraitsT, typename T>
std::basic_ostream<CharT,CharTraitsT>& operator<<(std::basic_ostream<CharT,CharTraitsT>& os, const std::vector<T>& v)
{
    typedef typename std::vector<T>::const_iterator iterator;

    os << "[";
    for (iterator it = v.begin(), end_it = v.end();
         it != end_it;
         ++it)
    {
        if (it != v.begin())
        {
            os << ", ";
        }
        os << *it;
    }
    os << "]";

    return os;
}

template <typename CharT, typename CharTraitsT, typename T>
std::basic_ostream<CharT,CharTraitsT>& operator<<(std::basic_ostream<CharT,CharTraitsT>& os, const std::set<T>& s)
{
    typedef typename std::set<T>::const_iterator iterator;

    os << "{";
    for (iterator it = s.begin(), end_it = s.end();
         it != end_it;
         ++it)
    {
        if (it != s.begin())
        {
            os << ", ";
        }
        os << *it;
    }
    os << "}";

    return os;
}

template <typename CharT, typename CharTraitsT, typename T>
std::basic_ostream<CharT,CharTraitsT>& operator<<(std::basic_ostream<CharT,CharTraitsT>& os, const std::unordered_set<T>& s)
{
    typedef typename std::unordered_set<T>::const_iterator iterator;

    os << "{";
    for (iterator it = s.begin(), end_it = s.end();
         it != end_it;
         ++it)
    {
        if (it != s.begin())
        {
            os << ", ";
        }
        os << *it;
    }
    os << "}";

    return os;
}

template <typename CharT, typename CharTraitsT, typename KT, typename VT>
std::basic_ostream<CharT,CharTraitsT>& operator<<(std::basic_ostream<CharT,CharTraitsT>& os, const std::map<KT,VT>& m)
{
    typedef typename std::map<KT,VT>::const_iterator iterator;

    os << "{";
    for (iterator it = m.begin(), end_it = m.end();
         it != end_it;
         ++it)
    {
        if (it != m.begin())
        {
            os << ", ";
        }
        os << it->first << " => " << it->second;
    }
    os << "}";

    return os;
}

template <typename CharT, typename CharTraitsT, typename KT, typename VT>
std::basic_ostream<CharT,CharTraitsT>& operator<<(std::basic_ostream<CharT,CharTraitsT>& os, const std::unordered_map<KT,VT>& m)
{
    typedef typename std::unordered_map<KT,VT>::const_iterator iterator;

    os << "{";
    for (iterator it = m.begin(), end_it = m.end();
         it != end_it;
         ++it)
    {
        if (it != m.begin())
        {
            os << ", ";
        }
        os << it->first << " => " << it->secon;
    }
    os << "}";

    return os;
}

}} // Namespace dcs::fog

#endif // DCS_FOG_IO_HPP
