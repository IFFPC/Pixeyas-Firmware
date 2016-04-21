############################################################################
#
# Copyright (c) 2015 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

#=============================================================================
# CMAKE CODING STANDARD FOR PX4
#
# Structure
# ---------------------------------------------------------------------------
#
# * Common functions should be included in px_base.cmake.
#
# * OS/ board specific fucntions should be include in
#	px_impl_${OS}.cmake or px4_impl_${OS}_${BOARD}.cmake.
#
# Formatting
# ---------------------------------------------------------------------------
#
# * Use hard indents to match the px4 source code.
#
# * All function and script arguments are upper case.
#
# * All local variables are lower case.
#
# * All cmake functions are lowercase.
#
# * For else, endif, endfunction, etc, never put the name of the statement
#
#	Instead of the very confusing:
#		if (${var} STREQUAL "1") <-- condition now becomes if name
#			# do somthing
#		elseif (${var} STREQUAL "2") <-- another condition
#			# do somthing
#		else (${var} STREQUAL "1") <-- tag is referring to name of if
#			# do somthing
#		endif (${var} STREQUAL "1") <-- tag is referring to name of if
#
#	Do this:
#		if (${var} STREQUAL "1") <-- condition now becomes if name
#			# do somthing
#		elseif (${var} STREQUAL "2") <-- another condition
#			# do somthing
#		else () <-- leave blank
#			# do somthing
#		endif () <-- leave blank
#
# Functions/Macros
# ---------------------------------------------------------------------------
#
# * Use px4_parse_function_args to parse functions and check for required
#   arguments. Unless there is only one argument in the function and it is clear.
#
# * Never use macros. They allow overwriting global variables and this
#	makes variable declarations hard to locate.
#
# * If a target from add_custom_* is set in a function, explicitly pass it
#	as an output argument so that the target name is clear to the user.
#
# * Avoid use of global variables in functions. Functions in a nested 
#	scope may use global variables, but this makes it difficult to
#	resuse functions.
#
# Included CMake Files
# ---------------------------------------------------------------------------
#
# * All variables in config files must have the prefix "config_".
#
# * Never set global variables in an included cmake file,
#	you may only define functions. This excludes config and Toolchain files.
#	This makes it clear to the user when variables are being set or targets
#	are being created.
#
# * Setting a global variable in a CMakeLists.txt file is ok, because
#	each CMakeLists.txt file has scope in the current directory and all
#	subdirecties, so it is not truly global.
#
# * All toolchain files should be included in the cmake
#	directory and named Toolchain-"name".cmake.
#
# Misc
# ---------------------------------------------------------------------------
#
# * If referencing a string variable, don't put it in quotes.
#	Don't do "${OS}" STREQUAL "posix",
#	instead type ${OS} STREQUAL "posix". This will throw an
#	error when ${OS} is not defined instead of silently
#	evaluating to false.
#
#=============================================================================

# Warning: Changing this modifies CMake's internal workings
# and leads to wrong toolchain detection
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

#=============================================================================
# parameters
#
# set parameters i.e. CMAKE_BUILD_TYPE = ""
# If CACHE is present, then the <variable> is put in the cache instead 
# STRING is the type, "build type" is the docstring.
set(CMAKE_BUILD_TYPE "" CACHE STRING "build type")

# set a property in the CACHE scope
# i.e. CMAKE_BUILD_TYPE.STRINGS = ";Debug;Release;RelWithDebInfo;MinSizeRel"
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY
	STRINGS ";Debug;Release;RelWithDebInfo;MinSizeRel")

# CONFIG = "nuttx_px4fmu-v2_default" 
set(CONFIG "nuttx_px4fmu-v2_default" CACHE STRING "desired configuration")

# file: file manipulation, GLOB_RECURSE, generate a list of all files that
#   match the globbing expression recursively.
# configs = "cmake/configs/*.cmake"
file(GLOB_RECURSE configs RELATIVE cmake/configs "cmake/configs/*.cmake")

# CONFIG.STRINGS = ${configs}
set_property(CACHE CONFIG PROPERTY STRINGS ${configs})

# THREADS = "4"
set(THREADS "4" CACHE STRING
	"number of threads to use for external build processes")

# DEBUG_PORT = "/dev/ttyACM0"
set(DEBUG_PORT "/dev/ttyACM0" CACHE STRING "debugging port")

#=============================================================================
# configuration
#
# must come before project to set toolchain

# string(): string operations, REPLACE match_string replace_string 
# i.e. in ${CONFIG} replace "_" with ";" store the result in config_args
# config_args = "nuttx;px4fmu-v2;default"
string(REPLACE "_" ";" config_args ${CONFIG})

# list operations. GET return elements specified by the index
# i.e. OS = config_args[0] = nuttx
list(GET config_args 0 OS)

# BOARD = config_args[1] = px4fmu-v2
list(GET config_args 1 BOARD)

# LABEL = config_args[2] = default
list(GET config_args 2 LABEL)

# target_name = "${OS}-${BOARD}-${LABEL}" = nuttx-px4fmu-v2-default
set(target_name "${OS}-${BOARD}-${LABEL}")

# message(): display a message to the user. STATUS: incidental message
message(STATUS "${target_name}")

# switch to ros CMake file if building ros
# include(): load and run CMake code from a file
if (${OS} STREQUAL "ros")
	include("cmake/ros-CMakeLists.txt")
else()  # otherwise use the rest of this file

# list APPEND add elements to the list
# CMAKE_SOURCE_DIR: the path to the top level of the source tree
# CMAKE_MODULE_PATH = CMAKE_MODULE_PATH;CMAKE_SOURCE_DIR/cmake
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

# config_module = "configs/nuttx_px4fmu-v2_default"
set(config_module "configs/${CONFIG}")

# load and run the files in config_module
include(${config_module})

# cmake modules

# load and run the CMake ExternalProject module
# create custom targets to build projects in external trees
include(ExternalProject)

# require px4 module interface

# px4_required_interface = px4_os_prebuild_targets;px4_os_add_flags
set(px4_required_interface
	px4_os_prebuild_targets
	px4_os_add_flags
	)
# cmd is the loop_var, i.e. cmd = px4_required_interface[1:end] 
foreach(cmd ${px4_required_interface})
# if(COMMAND command-name) True if the given name is a command, macro or 
# function that can be invoked.
# px4_os_prebuild_targets and px4_os_add_flags are functions defined in
# cmake/nuttx/px4_impl_nuttx.cmake
	if(NOT COMMAND ${cmd})
		message(FATAL_ERROR "${config_module} must implement ${cmd}")
	endif()
endforeach()

# px4_required_config = config_module_list
# config_module_list is defined in cmake/configs/nuttx_px4fmu-v2_default.cmake
# and includes modules that will be built.
set(px4_required_config
	config_module_list
	)
foreach(conf ${px4_required_config})
# test whether the variable is defined or not
	if(NOT DEFINED ${conf})
		message(FATAL_ERROR "cmake/${config_module} must define ${conf}")
	endif()
endforeach()

#=============================================================================
# project definition
#

# name px4, languages used CXX C ASM
project(px4 CXX C ASM)
if (NOT ${CMAKE_VERSION} VERSION_LESS 3.0.0)
	cmake_policy(SET CMP0045 NEW) # error on non-existent target in get prop
	cmake_policy(SET CMP0046 NEW) # no non-existent targets as dependencies
	cmake_policy(SET CMP0025 OLD) # still report AppleClang as Clang
endif()
if (NOT ${CMAKE_VERSION} VERSION_LESS 3.1.0)
	cmake_policy(SET CMP0054 NEW) # don't dereference quoted variables
endif()

# version_major = 1
set(version_major 1)

# version_minor = 0
set(version_minor 0)

# version_patch = 1
set(version_patch 1)

# version = 1.0.1
set(version "${version_major}.${version_minor}.${version_patch}")

# package-contact = "px4users@googlegroups.com"
set(package-contact "px4users@googlegroups.com")

#=============================================================================
# programs
#
# find and load settings from an external project
find_package(PythonInterp REQUIRED)

#=============================================================================
# cmake modules
#
# enable testing for current directory and below.
enable_testing()

#=============================================================================
# check required toolchain variables
#

# required_variables = CMAKE_C_COMPILER_ID
set(required_variables
	CMAKE_C_COMPILER_ID
	)


foreach(var ${required_variables})
# check whether ${var} is defined or not
	if (NOT ${var})
		message(FATAL_ERROR "Toolchain/config must define ${var}")
	endif()
endforeach()

#=============================================================================
# git
#

# px4_add_git_submodule is defined in cmake/common/px4_base.cmake
# this function adds a git submodule target.
# target = git_genmsg, PATH = "Tools/genmsg"
px4_add_git_submodule(TARGET git_genmsg PATH "Tools/genmsg")
px4_add_git_submodule(TARGET git_gencpp PATH "Tools/gencpp")
px4_add_git_submodule(TARGET git_mavlink PATH "mavlink/include/mavlink/v1.0")
px4_add_git_submodule(TARGET git_gtest PATH "unittets/gtest")
px4_add_git_submodule(TARGET git_uavcan PATH "src/modules/uavcan/libuavcan")
px4_add_git_submodule(TARGET git_nuttx PATH "NuttX")
px4_add_git_submodule(TARGET git_dspal PATH "src/lib/dspal")
px4_add_git_submodule(TARGET git_jmavsim PATH "Tools/jMAVSim")
px4_add_git_submodule(TARGET git_gazebo PATH "Tools/sitl_gazebo")
px4_add_git_submodule(TARGET git_matrix PATH "src/lib/matrix")

# add a target with the given name that executes the given commands. 
# NAME = submodule_clean; command1 = git submodule deinit -f .
# command2 = rm -rf .git/modules/* 
add_custom_target(submodule_clean
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
	COMMAND git submodule deinit -f .
	COMMAND rm -rf .git/modules/*
	)

#=============================================================================
# misc targets
#

# another custom target, the target can be used in make, e.g. make check_format
# NAME = check_format, COMMAND = Tools/check_code_style.sh
add_custom_target(check_format
	COMMAND Tools/check_code_style.sh
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
	)

# another custom target
# NAME = config, COMMAND = cmake-gui . 
add_custom_target(config
	COMMAND cmake-gui .
	WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
	)

#=============================================================================
# external libraries
#

# execute function px4_os_prebuild_targets defined in 
# cmake/nuttx/px4_impl_nuttx.cmake. This function generates os dependent targets
# prebuild_targets is a list of targets generated by the function
px4_os_prebuild_targets(OUT prebuild_targets
	BOARD ${BOARD}
	THREADS ${THREADS})

#=============================================================================
# build flags
#
# execute function px4_os_add_flags defined in 
# cmake/nuttx/px4_impl_nuttx.cmake. This function sets the nuttx build flags
px4_os_add_flags(
	BOARD ${BOARD}
	C_FLAGS c_flags
	CXX_FLAGS cxx_flags
	EXE_LINKER_FLAGS exe_linker_flags
	INCLUDE_DIRS include_dirs
	LINK_DIRS link_dirs
	DEFINITIONS definitions)

# execute function px4_join define in 
# cmake/common/px4_base.cmake. This function joins a list with a 
# separator specified by GLUE. The output is stored in CMAKE_EXE_LINKER_FLAGS
# ${exe_linker_flags} are returned by the function px4_os_add_flags. 
px4_join(OUT CMAKE_EXE_LINKER_FLAGS LIST "${exe_linker_flags}" GLUE " ")
px4_join(OUT CMAKE_C_FLAGS LIST "${c_flags}" GLUE " ")
px4_join(OUT CMAKE_CXX_FLAGS LIST "${cxx_flags}" GLUE " ")

# directories for the compiler to search for include files
include_directories(${include_dirs})

# directories for the linker to look for libraries
link_directories(${link_dirs})

# adds -D define flags to the compiler command line 
add_definitions(${definitions})

#=============================================================================
# source code generation

# msg_files = msg/*.msg
file(GLOB_RECURSE msg_files msg/*.msg)

# px4_generate_messages is defined in cmake/common/px4_base.cmake and
# generates source code from msg definitions
px4_generate_messages(TARGET msg_gen
	MSG_FILES ${msg_files}
	OS ${OS}
	DEPENDS git_genmsg git_gencpp
	)

# px4_generate_parameters_xml is defined in cmake/common/px4_base.cmake and
# set a custom command to genrate a parameters.xml file. 
# The custom command is stored in parameters.xml variable
px4_generate_parameters_xml(OUT parameters.xml BOARD ${BOARD})

# px4_generate_airframes_xml is defined in cmake/common/px4_base.cmake and
# set a custom command to generates an airframes.xml file
px4_generate_airframes_xml(OUT airframes.xml BOARD ${BOARD})

# add a target named xml_gen
# the keyword DEPENDS list dependencies of the command. NOTE the parameters.xml
# and airframes.xml are two custom commands, rather than xml files here.
add_custom_target(xml_gen
	DEPENDS parameters.xml airframes.xml)

#=============================================================================
# external projects
#

# Don't know what it is doing here.

set(ep_base ${CMAKE_BINARY_DIR}/external)
set_property(DIRECTORY PROPERTY EP_BASE ${ep_base})

# add external project install folders to build
link_directories(${ep_base}/Install/lib)
include_directories(${ep_base}/Install/include)
# add the directories so cmake won't warn
execute_process(COMMAND cmake -E make_directory ${ep_base}/Install/lib)
execute_process(COMMAND cmake -E make_directory ${ep_base}/Install/include)

#=============================================================================
# subdirectories
#

# module_libraries = empty
set(module_libraries)
foreach(module ${config_module_list})

# add a subdirectory to the build. The source_dir specifies the directory in which the source CMakeLists.txt and code files are located. 
	add_subdirectory(src/${module})

# px4_mangle_name is defined in cmake/common/px4_base.cmake and 
# converts a path name to a module name
	px4_mangle_name(${module} mangled_name)

# module_libraries = module_libraries;${mangled_name}
	list(APPEND module_libraries ${mangled_name})
	#message(STATUS "adding module: ${module}")
endforeach()

add_subdirectory(src/firmware/${OS})

# config_io_board is defined in cmake/configs/nuttx_px4fmu-v2_default.cmake
if (config_io_board)
	add_subdirectory(src/modules/px4iofirmware)
endif()

#=============================================================================
# generate git version
#
# px4_create_git_hash_header is defined in cmake/common/px4_base.cmake and
# creates a header file containing the git hash of the current tree.
px4_create_git_hash_header(HEADER ${CMAKE_BINARY_DIR}/build_git_version.h)

#=============================================================================
# packaging
#
# Important to having packaging at end of cmake file.
#
set(CPACK_PACKAGE_VERSION ${version})
set(CPACK_PACKAGE_CONTACT ${package_contact})
set(CPACK_GENERATOR "ZIP")
set(CPACK_SOURCE_GENERATOR "ZIP")

# CPack is a CMake package to build binary and source package installers
include(CPack)

endif() # ros alternative endif

# vim: set noet fenc=utf-8 ff=unix ft=cmake :
