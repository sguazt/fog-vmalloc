#!/bin/sh

if [ "$#" -lt "1" ]; then
	echo "Usage: $0 <base dir>"
	exit 1
fi

base_dir=$1
if [ ! -d "$base_dir" ]; then
    echo "Wrong base directory '$base_dir'"
	exit 1
fi

base_inc_dir=$base_dir/c++/include
prj_subdir=dcs/fog
dtnow=$(date +'%F %T (%Z)')
year=$(date +'%Y')

## Get version
version=$(cat $base_dir/VERSION)
major_version=$((version / 100000))
minor_version=$((version / 100 % 1000))
patch_version=$((version % 100))

mkdir -p $base_inc_dir/$prj_subdir/detail

cat > $base_inc_dir/$prj_subdir/detail/version.hpp <<EOT
/** \file $prj_subdir/detail/version.hpp
 *
 * \brief Version of this application.
 *
 * [$dtnow]
 * This is an autogenerated file. Do not edit.
 *
 * \author Marco Guazzone (marco.guazzone@gmail.com)
 *
 * <hr/>
 *  
 * Copyright $year Marco Guazzone (marco.guazzone@gmail.com)
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

#ifndef DCS_FOG_VM_ALLOC_DETAIL_VERSION_HPP
#define DCS_FOG_VM_ALLOC_DETAIL_VERSION_HPP


#define DCS_FOG_VM_ALLOC_DETAIL_VERSION $version
#define DCS_FOG_VM_ALLOC_DETAIL_VERSION_STR "$major_version.$minor_version.$patch_version"
#define DCS_FOG_VM_ALLOC_DETAIL_MAJOR_VERSION $major_version // (DCS_FOG_VM_ALLOC_DETAIL_VERSION / 100000)
#define DCS_FOG_VM_ALLOC_DETAIL_MINOR_VERSION $minor_version // (DCS_FOG_VM_ALLOC_DETAIL_VERSION / 100 % 1000)
#define DCS_FOG_VM_ALLOC_DETAIL_PATCH_VERSION $patch_version // (DCS_FOG_VM_ALLOC_DETAIL_VERSION % 100)


#endif // DCS_FOG_VM_ALLOC_DETAIL_VERSION_HPP
EOT

