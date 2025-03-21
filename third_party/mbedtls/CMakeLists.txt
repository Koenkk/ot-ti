#
#  Copyright (c) 2020, The OpenThread Authors.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#  1. Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#  2. Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#  3. Neither the name of the copyright holder nor the
#     names of its contributors may be used to endorse or promote products
#     derived from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#

set(ENABLE_TESTING OFF CACHE BOOL "Disable mbedtls test" FORCE)
set(ENABLE_PROGRAMS OFF CACHE BOOL "Disable mbedtls program" FORCE)

set(MBEDTLS_TARGET_PREFIX "ti-" CACHE STRING "The mbedTLS prefix" FORCE)

find_program(UNIFDEFALL_EXE unifdefall)
find_program(UNIFDEF_EXE unifdef)
if(UNIFDEF_EXE)
    execute_process(COMMAND ${UNIFDEF_EXE} -V ERROR_VARIABLE VERSION_OUTPUT)
    string(REGEX MATCH "Version: unifdef-([0-9]+\\.[0-9]+)" VERSION_MATCH "${VERSION_OUTPUT}")
    set(UNIFDEF_VERSION ${CMAKE_MATCH_1})
endif()
find_program(SED_EXE sed)

string(REPLACE "-Wconversion" "" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
string(REPLACE "-Wconversion" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

if(TI_SIMPLELINK_ISA STREQUAL "m4f")
    string(REPLACE "-Os" "-Ofast" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
    string(REPLACE "-Os" "-Ofast" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
endif()

add_library(ti-mbedtls-hardware-alt
    alt/aes_alt.c
    alt/ecdh_alt.c
    alt/ecdsa_alt.c
    alt/ecjpake_alt.c
    alt/sha256_alt.c
)

target_link_libraries(ti-mbedtls-hardware-alt
    PRIVATE
        ${OT_PLATFORM_LIB}
)

target_compile_definitions(ti-mbedtls-hardware-alt
    PUBLIC
        ${OT_PLATFORM_DEFINES}
)

target_include_directories(ti-mbedtls-hardware-alt
    PRIVATE
        ${OT_PLATFORM_INCLUDES}
        ${OT_PUBLIC_INCLUDES}
)

set(thirdparty_lib ti-mbedtls-hardware-alt)
add_subdirectory(repo)


target_compile_definitions(ti-mbedtls-hardware-alt
    PUBLIC
        "MBEDTLS_CONFIG_FILE=\"mbedtls-config-${TI_SIMPLELINK_FAMILY}.h\""
    PRIVATE
        $<TARGET_PROPERTY:ot-config,INTERFACE_COMPILE_DEFINITIONS>
)
target_include_directories(ti-mbedtls-hardware-alt
    PUBLIC
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/alt
        ${CMAKE_CURRENT_SOURCE_DIR}/repo/include
    PRIVATE
        ${OT_PUBLIC_INCLUDES}
        $<TARGET_PROPERTY:ot-config,INTERFACE_INCLUDE_DIRECTORIES>
)

target_compile_definitions(ti-mbedtls
    PUBLIC
        "MBEDTLS_CONFIG_FILE=\"mbedtls-config-${TI_SIMPLELINK_FAMILY}.h\""
    PRIVATE
        $<TARGET_PROPERTY:ot-config,INTERFACE_COMPILE_DEFINITIONS>
)
target_include_directories(ti-mbedtls
    PUBLIC
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/alt
        ${CMAKE_CURRENT_SOURCE_DIR}/repo/include
    PRIVATE
        ${OT_PUBLIC_INCLUDES}
        $<TARGET_PROPERTY:ot-config,INTERFACE_INCLUDE_DIRECTORIES>
)

target_compile_definitions(ti-mbedx509
    PUBLIC
        "MBEDTLS_CONFIG_FILE=\"mbedtls-config-${TI_SIMPLELINK_FAMILY}.h\""
    PRIVATE
        $<TARGET_PROPERTY:ot-config,INTERFACE_COMPILE_DEFINITIONS>
)
target_include_directories(ti-mbedx509
    PUBLIC
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/alt
        ${CMAKE_CURRENT_SOURCE_DIR}/repo/include
    PRIVATE
        ${OT_PUBLIC_INCLUDES}
        $<TARGET_PROPERTY:ot-config,INTERFACE_INCLUDE_DIRECTORIES>
)

target_compile_definitions(ti-mbedcrypto
    PUBLIC
        "MBEDTLS_CONFIG_FILE=\"mbedtls-config-${TI_SIMPLELINK_FAMILY}.h\""
    PRIVATE
        $<TARGET_PROPERTY:ot-config,INTERFACE_COMPILE_DEFINITIONS>
)
target_include_directories(ti-mbedcrypto
    PUBLIC
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/alt
        ${CMAKE_CURRENT_SOURCE_DIR}/repo/include
    PRIVATE
        ${OT_PUBLIC_INCLUDES}
        $<TARGET_PROPERTY:ot-config,INTERFACE_INCLUDE_DIRECTORIES>
)
