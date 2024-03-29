#-------------------------------------------------------------------------------
# Copyright (c) 2019, Arm Limited. All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
#
#-------------------------------------------------------------------------------

#Sphinx needs all document files to be under a single directory. This script
#copies all document files to a temporary directory while keeping the original
#directory tree (relative location of files) except "docs/index.rst" which is
#moved to the top level of the new tree.
#
#  i.e.:
#    <DST_DIR>
#    |   All documents from <TFM_ROOT_DIR> plus <TFM_ROOT_DIR>/docs/index.rst
#    |
#    +---docs
#    |   |
#        \- all documents from <TFM_ROOT_DIR>/docs except index.rst
#    |
#    +---lib
#    |   |
#    |   \- All document from <TFM_ROOT_DIR>/lib keeping reativle location
#    ...
#    |
#
#Usage:
#   cmake -DDST_DIR=<path to destination> -DTFM_ROOT_DIR=<path to tf-m root> \
#          -P SphinxCopyDoc.cmake

#Check input parameters
foreach(_PARAM IN ITEMS TFM_ROOT_DIR DST_DIR)
	if (NOT DEFINED ${_PARAM})
		message(FATAL_ERROR "Variable ${_PARAM} is undefined. Please add -D${_PARAM}=<...> when calling this script.")
	endif()
endforeach()

message(STATUS "Creating document tree for Sphinx under ${DST_DIR}")

#List all document files.
file(GLOB_RECURSE _COPY_FILES
		LIST_DIRECTORIES false
		RELATIVE "${TFM_ROOT_DIR}"
		"${TFM_ROOT_DIR}/*.md"
		"${TFM_ROOT_DIR}/*.rst"
		"${TFM_ROOT_DIR}/*.png"
		"${TFM_ROOT_DIR}/*.jpg"
		"${TFM_ROOT_DIR}/dco.txt")

#Subtract exluded files from copy files
list(REMOVE_ITEM _COPY_FILES "docs/index.rst")

#Copy files with directory tree.
foreach(_FILE ${_COPY_FILES})
	get_filename_component(_DIR ${_FILE} DIRECTORY)
	file(COPY ${_FILE} DESTINATION "${DST_DIR}/${_DIR}")
endforeach()

#Copy index.rst to the top level
file(COPY "${TFM_ROOT_DIR}/docs/index.rst" DESTINATION "${DST_DIR}")
