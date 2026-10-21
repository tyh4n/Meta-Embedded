# This file includes configurations for ARM embedded development and toolchain.
# liuzikai 2018-02-09

# CMake system configs. These configs affects the way to pass flags.
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# ---------- Toolchain Configurations ----------

# Prefer an explicit toolchain directory from CMake presets or the environment.
# Falling back to PATH keeps direct CMake invocations working as before.
if(DEFINED ENV{ARM_NONE_EABI_TOOLCHAIN_DIR} AND NOT "$ENV{ARM_NONE_EABI_TOOLCHAIN_DIR}" STREQUAL "")
    file(TO_CMAKE_PATH "$ENV{ARM_NONE_EABI_TOOLCHAIN_DIR}" TOOLCHAIN_DIR)
    string(REGEX REPLACE "/+$" "" TOOLCHAIN_DIR "${TOOLCHAIN_DIR}")
    set(TOOLCHAIN_PREFIX "${TOOLCHAIN_DIR}/arm-none-eabi-")
    message("[Notice] ARM GNU Toolchain: ${TOOLCHAIN_DIR}")
else()
    set(TOOLCHAIN_PREFIX arm-none-eabi-)
endif()

set(CMAKE_C_COMPILER   ${TOOLCHAIN_PREFIX}gcc    )
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}gcc    )
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}g++    )
set(CMAKE_OBJCOPY      ${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_SIZE_UTILITY ${TOOLCHAIN_PREFIX}size)
set(CMAKE_EXE_LINKER_FLAGS "--specs=nosys.specs")


# The following lines create target to let CMake call Make directly. Now they are deprecated.

#if (CMAKE_HOST_SYSTEM_NAME MATCHES "Darwin")
#    message("CMAKE_HOST_SYSTEM_NAME = ${CMAKE_HOST_SYSTEM_NAME}, use `make` to compile")
#    set(MAKE_EXECUTABLE make)
#else()
#    message("CMAKE_HOST_SYSTEM_NAME = ${CMAKE_HOST_SYSTEM_NAME}, use `mingw32-make` to compile")
#    set(MAKE_EXECUTABLE mingw32-make)
#endif()
#add_custom_target(Main_Make COMMAND ${MAKE_EXECUTABLE} -j -C ${Meta_Infantry_SOURCE_DIR}/dev UNIT_TEST=NONE)
