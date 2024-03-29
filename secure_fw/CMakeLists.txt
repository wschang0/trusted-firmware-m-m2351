#-------------------------------------------------------------------------------
# Copyright (c) 2017-2019, Arm Limited. All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
#
#-------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.7)

#Tell cmake where our modules can be found
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/../cmake)

#Include common stuff to control cmake.
include("Common/BuildSys")

#Include functionality to enable building the documentation.
include("Common/BuildDoxygenDoc")
include("Common/BuildSphinxDoc")

#Start an embedded project.
embedded_project_start(CONFIG "${CMAKE_CURRENT_LIST_DIR}/../ConfigDefault.cmake")
project(tfm_s LANGUAGES ASM C)
embedded_project_fixup()

set(SECURE_FW_DIR "${CMAKE_CURRENT_LIST_DIR}")
set(TFM_ROOT_DIR  "${SECURE_FW_DIR}/..")
set(TEST_DIR      "${TFM_ROOT_DIR}/test")
set(INTERFACE_DIR "${TFM_ROOT_DIR}/interface")

if (NOT DEFINED TFM_LVL)
	message(FATAL_ERROR "Incomplete build configuration: TFM_LVL is undefined. ")
endif()

if (NOT DEFINED TFM_PARTITION_AUDIT_LOG)
	message(FATAL_ERROR "Incomplete build configuration: TFM_PARTITION_AUDIT_LOG is undefined.")
endif()

if (NOT DEFINED TFM_PARTITION_PLATFORM)
	message(FATAL_ERROR "Incomplete build configuration: TFM_PARTITION_PLATFORM is undefined.")
endif()

include(${SECURE_FW_DIR}/spm/CMakeLists.inc)
include(${SECURE_FW_DIR}/ns_callable/CMakeLists.inc)
#Involve all IPC related sources in ipc's CMakeLists.inc, and switch core between IPC and Library.
if(TFM_PSA_API)
	include(${SECURE_FW_DIR}/core/ipc/CMakeLists.inc)
else()
	include(${SECURE_FW_DIR}/core/CMakeLists.inc)
endif()

set(BUILD_CMSIS_CORE On)
set(BUILD_RETARGET On)
set(BUILD_NATIVE_DRIVERS On)
set(BUILD_STARTUP On)
set(BUILD_TARGET_CFG On)
# FIXME: The following TARGET flags are platform dependent.
#        It is required to add a mechanism to expose the
#        target capabilities and, based on them, set the
#        flags properly.
set(BUILD_TARGET_HARDWARE_KEYS On)
set(BUILD_TARGET_NV_COUNTERS On)
set(BUILD_CMSIS_DRIVERS On)
set(BUILD_TIME Off)
set(BUILD_UART_STDOUT On)
set(BUILD_FLASH On)
set(BUILD_BOOT_SEED On)
set(BUILD_DEVICE_ID On)
if(NOT DEFINED PLATFORM_CMAKE_FILE)
	message (FATAL_ERROR "Platform specific CMake is not defined. Please set PLATFORM_CMAKE_FILE.")
elseif(NOT EXISTS ${PLATFORM_CMAKE_FILE})
	message (FATAL_ERROR "Platform specific CMake \"${PLATFORM_CMAKE_FILE}\" file does not exist. Please fix value of PLATFORM_CMAKE_FILE.")
else()
	include(${PLATFORM_CMAKE_FILE})
endif()

if(NOT DEFINED S_SCATTER_FILE_NAME)
	message(FATAL_ERROR "ERROR: Incomplete Configuration: S_SCATTER_FILE_NAME not defined, Include this file from a Config*.cmake")
endif()
embedded_set_target_linker_file(TARGET ${PROJECT_NAME} PATH "${S_SCATTER_FILE_NAME}")

embedded_target_include_directories(TARGET ${PROJECT_NAME} PATH ${TFM_ROOT_DIR} ABSOLUTE APPEND)
#Create an object library to avoid compiling all source files twice, when two executables
#with different memory map need to be linked(BL2 non-swapping)
set(PROJECT_OBJ_LIB ${PROJECT_NAME}_obj_lib)
add_library(${PROJECT_OBJ_LIB} OBJECT ${ALL_SRC_C} ${ALL_SRC_C_S} ${ALL_SRC_ASM_S})

#Set common compiler flags
config_setting_shared_compiler_flags(${PROJECT_OBJ_LIB})

if(NOT DEFINED TARGET_NV_COUNTERS_ENABLE)
	set(TARGET_NV_COUNTERS_ENABLE OFF)
endif()

if(TARGET_NV_COUNTERS_ENABLE)
	embedded_set_target_compile_defines(TARGET ${PROJECT_OBJ_LIB} LANGUAGE C DEFINES TFM_NVCOUNTERS_ENABLE APPEND)
endif()

if (NOT DEFINED CORE_TEST)
	message(FATAL_ERROR "Incomplete build configuration: CORE_TEST is undefined.")
elseif(CORE_TEST)
	embedded_set_target_compile_defines(TARGET ${PROJECT_OBJ_LIB} LANGUAGE C DEFINES TFM_CORE_DEBUG APPEND)
endif()

if (NOT DEFINED TFM_NS_CLIENT_IDENTIFICATION)
	message(FATAL_ERROR "Incomplete build configuration: TFM_NS_CLIENT_IDENTIFICATION is undefined.")
elseif (TFM_NS_CLIENT_IDENTIFICATION)
	target_compile_definitions(${PROJECT_OBJ_LIB} PRIVATE TFM_NS_CLIENT_IDENTIFICATION)
endif()

if (NOT DEFINED DEBUG_AUTHENTICATION)
	set(DEBUG_AUTHENTICATION "DAUTH_CHIP_DEFAULT")
endif()

#Set include directories
embedded_target_include_directories(TARGET ${PROJECT_OBJ_LIB} PATH ${TFM_ROOT_DIR} ABSOLUTE APPEND)

# For the non-swapping BL2 configuration two executables need to be built.
# One can be executed from flash partition slot_0 and other from slot_1.
# Only the linking phase is different. This function captures common settings
# and eliminates copy-paste.
function(set_up_secure_fw_build)
	set( _OPTIONS_ARGS)                                #Option (on/off) arguments (e.g. IGNORE_CASE)
	set( _ONE_VALUE_ARGS S_TARGET VENEER_NAME POSTFIX) #Single option arguments (e.g. PATH "./foo/bar")
	set( _MULTI_VALUE_ARGS LINK_DEFINES)               #List arguments (e.g. LANGUAGES C ASM CXX)
	cmake_parse_arguments(_MY_PARAMS "${_OPTIONS_ARGS}" "${_ONE_VALUE_ARGS}" "${_MULTI_VALUE_ARGS}" ${ARGN})

	if (NOT DEFINED _MY_PARAMS_S_TARGET)
		message(FATAL_ERROR "set_up_secure_fw_build(): mandatory parameter 'S_TARGET' missing.")
	endif()

	if (NOT DEFINED _MY_PARAMS_VENEER_NAME)
		message(FATAL_ERROR "set_up_secure_fw_build(): mandatory parameter 'VENEER_NAME' missing.")
	endif()

	set(EXE_NAME ${_MY_PARAMS_S_TARGET}${_MY_PARAMS_POSTFIX})
	set(VENEER_NAME ${_MY_PARAMS_VENEER_NAME}${_MY_PARAMS_POSTFIX}.o)

	#Create linker target: add object library to executable
	add_executable(${EXE_NAME} $<TARGET_OBJECTS:${PROJECT_OBJ_LIB}>)

	#Set common linker flags
	config_setting_shared_linker_flags(${EXE_NAME})

	#Indicates to secure target(s) already created
	set(TARGET_TFM_S_EXISTED True PARENT_SCOPE)

	#Set individual linker flags per linker target/executable
	foreach(flag ${_MY_PARAMS_LINK_DEFINES})
		embedded_set_target_link_defines(TARGET ${EXE_NAME} DEFINES "${flag}")
	endforeach(flag)


	embedded_set_target_linker_file(TARGET ${EXE_NAME} PATH "${S_SCATTER_FILE_NAME}")

	add_dependencies(${EXE_NAME} tfm_crypto)
	add_dependencies(${EXE_NAME} tfm_storage)
	add_dependencies(${EXE_NAME} tfm_secure_tests)
	add_dependencies(${EXE_NAME} tfm_attest)
	if (TFM_PARTITION_AUDIT_LOG)
		add_dependencies(${EXE_NAME} tfm_audit)
	endif()
	if (TFM_PARTITION_PLATFORM)
		add_dependencies(${EXE_NAME} tfm_platform)
	endif()

	#Set macro definitions for the project.
	embedded_set_target_compile_defines(TARGET ${PROJECT_OBJ_LIB} LANGUAGE C DEFINES __thumb2__ __ARM_FEATURE_CMSE=3 TFM_LVL=${TFM_LVL} ${DEBUG_AUTHENTICATION} APPEND)

	if (REGRESSION OR CORE_TEST)
		if (DEFINED TFM_PARTITION_TEST_SECURE_SERVICES AND TFM_PARTITION_TEST_SECURE_SERVICES)
			#The test service veneers in the tfm_secure_tests library may not be
			#referenced in the secure binary so the veneer objects are explicitly loaded
			#from the secure tests library. However by generating the veneer files from
			#the manifests, all the iovec interfaced veneers are in a single file in the
			#secure_fw directory. The core test partitions use the veneers with the
			#iovec API, so we only need the explicit load in case the secure client test
			#partition is present.
			#FIXME Remove the explicit load and the above comment once the secure client
			#test partition uses the generated veneers.
			if((${COMPILER} STREQUAL "ARMCLANG") AND (NOT TFM_PSA_API))
				target_link_libraries(${EXE_NAME} tfm_attest tfm_secure_tests tfm_attest tfm_crypto tfm_storage $<TARGET_LINKER_FILE:tfm_secure_tests>\(*veneers.o\) tfm_attest)
			else()
				target_link_libraries(${EXE_NAME} tfm_attest tfm_secure_tests tfm_attest tfm_crypto tfm_storage tfm_attest)
			endif()
		else()
			target_link_libraries(${EXE_NAME} tfm_attest tfm_crypto tfm_storage tfm_secure_tests tfm_attest)
		endif()
	else()
		target_link_libraries(${EXE_NAME} tfm_attest tfm_crypto tfm_storage tfm_attest)
	endif()

	if (TFM_PARTITION_AUDIT_LOG)
		target_link_libraries(${EXE_NAME} tfm_audit)
	endif()

	if (TFM_PARTITION_PLATFORM)
		target_link_libraries(${EXE_NAME} tfm_platform)
	endif()

	embedded_set_target_link_defines(TARGET ${EXE_NAME} DEFINES "TFM_LVL=${TFM_LVL}")

	if (TFM_PARTITION_AUDIT_LOG)
		embedded_set_target_link_defines(TARGET ${EXE_NAME} DEFINES "TFM_PARTITION_AUDIT_LOG")
	endif()

	if (TFM_PARTITION_PLATFORM)
		embedded_set_target_link_defines(TARGET ${EXE_NAME} DEFINES "TFM_PARTITION_PLATFORM")
	endif()

	if (NOT DEFINED TFM_PARTITION_TEST_CORE)
		message(FATAL_ERROR "Incomplete build configuration: TFM_PARTITION_TEST_CORE is undefined. ")
	elseif (TFM_PARTITION_TEST_CORE)
		embedded_set_target_link_defines(TARGET ${EXE_NAME} DEFINES "TFM_PARTITION_TEST_CORE")
	endif()

	if (NOT DEFINED TFM_PARTITION_TEST_CORE_IPC)
		message(FATAL_ERROR "Incomplete build configuration: TFM_PARTITION_TEST_CORE_IPC is undefined.")
	elseif (TFM_PARTITION_TEST_CORE_IPC)
		embedded_set_target_link_defines(TARGET ${EXE_NAME} DEFINES "TFM_PARTITION_TEST_CORE_IPC")
	endif()

	if (NOT DEFINED TFM_PARTITION_TEST_SECURE_SERVICES)
		message(FATAL_ERROR "Incomplete build configuration: TFM_PARTITION_TEST_SECURE_SERVICES is undefined. ")
	elseif (TFM_PARTITION_TEST_SECURE_SERVICES)
		embedded_set_target_link_defines(TARGET ${EXE_NAME} DEFINES "TFM_PARTITION_TEST_SECURE_SERVICES")
	endif()

	if (NOT DEFINED TEST_FRAMEWORK_S)
		message(FATAL_ERROR "Incomplete build configuration: TEST_FRAMEWORK_S is undefined.")
	elseif (TEST_FRAMEWORK_S)
		embedded_set_target_link_defines(TARGET ${EXE_NAME} DEFINES "TEST_FRAMEWORK_S")
	endif()

	if (NOT DEFINED TEST_FRAMEWORK_NS)
		message(FATAL_ERROR "Incomplete build configuration: TEST_FRAMEWORK_NS is undefined.")
	elseif (TEST_FRAMEWORK_NS)
		embedded_set_target_link_defines(TARGET ${EXE_NAME} DEFINES "TEST_FRAMEWORK_NS")
	endif()

	if (NOT DEFINED BL2)
		message(FATAL_ERROR "Incomplete build configuration: BL2 is undefined. ")
	elseif (BL2)
		embedded_set_target_link_defines(TARGET ${EXE_NAME} DEFINES "BL2")
	endif()

	if (NOT DEFINED TFM_PSA_API)
		message(FATAL_ERROR "Incomplete build configuration: TFM_PSA_API is undefined. ")
	elseif (TFM_PSA_API)
		embedded_set_target_link_defines(TARGET ${EXE_NAME} DEFINES "TFM_PSA_API")
	endif()

	if(CORE_TEST)
		set(SECURE_AXF_DIR_PREFIX "${CMAKE_BINARY_DIR}/unit_test/")
		set_target_properties(${EXE_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${SECURE_AXF_DIR_PREFIX})
		embedded_set_target_link_defines(TARGET ${EXE_NAME} DEFINES "TFM_PARTITION_TEST_CORE")
	endif()

	if(NOT DEFINED PLATFORM_LINK_INCLUDES)
		message(FATAL_ERROR "ERROR: Incomplete Configuration: PLATFORM_LINK_INCLUDES is not defined.")
	endif()
	embedded_set_target_link_includes(TARGET ${EXE_NAME} INCLUDES "${PLATFORM_LINK_INCLUDES}")

	#Generate binary file from executable
	compiler_generate_binary_output(${EXE_NAME})

	#Configure where we put the CMSE veneers generated by the compiler.
	if (DEFINED S_VENEER_FILE_LOCATION)
		set(S_VENEER_FILE "${S_VENEER_FILE_LOCATION}/${VENEER_NAME}")
	else()
		set(S_VENEER_FILE "${CMAKE_CURRENT_BINARY_DIR}/${VENEER_NAME}")
	endif()
	compiler_set_cmse_output(${EXE_NAME} "${S_VENEER_FILE}")

	#Configure what file shall be installed.
	#Set install location. Keep original value to avoid overriding command line settings.
	if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
		set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "Default install location for secure_fw." FORCE)
	endif()

	#Export files needed to interface external applications at: <build_dir>/install/export/tfm/
	install(DIRECTORY ${TFM_ROOT_DIR}/interface/include/
			DESTINATION export/tfm/inc)

	install(DIRECTORY ${TFM_ROOT_DIR}/interface/src/
			DESTINATION export/tfm/src)

	install(FILES ${S_VENEER_FILE} DESTINATION export/tfm/veneers)

	#Collect executables to common location: <build_dir>/install/outputs/
	if (DEFINED SECURE_AXF_DIR_PREFIX)
		set(MY_BINARY_DIR ${SECURE_AXF_DIR_PREFIX})
	else()
		set(MY_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})
	endif()

	install(FILES ${MY_BINARY_DIR}/${EXE_NAME}.axf
				  ${MY_BINARY_DIR}/${EXE_NAME}.bin
			DESTINATION outputs/${TARGET_PLATFORM}/)

	install(FILES ${MY_BINARY_DIR}/${EXE_NAME}.axf
				  ${MY_BINARY_DIR}/${EXE_NAME}.bin
			DESTINATION outputs/fvp/)
endfunction()

#Adds the test directory
add_subdirectory(${TFM_ROOT_DIR}/test ${CMAKE_BINARY_DIR}/test)

#Add the crypto library target
add_subdirectory(${SECURE_FW_DIR}/services/crypto)

#Add the secure storage library target
add_subdirectory(${SECURE_FW_DIR}/services/secure_storage)

#Add the platform service library target
if (TFM_PARTITION_PLATFORM)
	add_subdirectory(${SECURE_FW_DIR}/services/platform)
endif()

#Add the initial attestation service library target
add_subdirectory(${SECURE_FW_DIR}/services/initial_attestation)

#Add the audit logging library target
if (TFM_PARTITION_AUDIT_LOG)
	add_subdirectory(${SECURE_FW_DIR}/services/audit_logging)
endif()

if (LINK_TO_BOTH_MEMORY_REGION)
	#Link to primary memory region
	set_up_secure_fw_build(S_TARGET      ${PROJECT_NAME}
						   VENEER_NAME   s_veneers
						   POSTFIX       "_0")

	#Link to secondary memory region(add extra linker flag)
	set_up_secure_fw_build(S_TARGET      ${PROJECT_NAME}
						   LINK_DEFINES  "LINK_TO_SECONDARY_PARTITION"
						   VENEER_NAME   s_veneers
						   POSTFIX       "_1")
else()
	#Link to primary memory region only
	set_up_secure_fw_build(S_TARGET      ${PROJECT_NAME}
						   VENEER_NAME   s_veneers)
endif()

#Finally let CMake system apply changes after the whole project is defined.
if (TARGET ${PROJECT_NAME})
	embedded_project_end(${PROJECT_NAME})
endif()

if (TARGET ${PROJECT_NAME}_0)
	embedded_project_end(${PROJECT_NAME}_0)
endif()

if (TARGET ${PROJECT_NAME}_1)
	embedded_project_end(${PROJECT_NAME}_1)
endif()

embedded_project_end(${PROJECT_OBJ_LIB})
