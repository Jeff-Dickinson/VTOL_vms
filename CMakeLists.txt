cmake_minimum_required(VERSION 3.16)
project(vms C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_compile_options(-Wall -Wextra -O2)

# ── HAL shared library ──────────────────────────────────────────────
add_library(vms_hal SHARED
    src/hal/src/bno085.c
    src/hal/src/pwm_output.c
    src/hal/src/rc_input.c
    src/hal/src/hal_init.c
)
target_include_directories(vms_hal PUBLIC
    src/hal/include
    src/bindings
)
target_link_libraries(vms_hal PRIVATE pigpio rt)

# ── Control shared library ──────────────────────────────────────────
add_library(vms_control SHARED
    src/control/src/control_loop.c
    src/control/src/pid.c
    src/control/src/mixer.c
    src/control/src/attitude.c
    src/control/src/ipc_shm.c
)
target_include_directories(vms_control PUBLIC
    src/control/include
    src/hal/include
    src/bindings
)
target_link_libraries(vms_control PRIVATE vms_hal pthread rt)

# ── Unit tests ──────────────────────────────────────────────────────
enable_testing()

add_executable(test_pid tests/test_pid.c src/control/src/pid.c)
target_include_directories(test_pid PRIVATE src/control/include src/bindings)
target_link_libraries(test_pid m)
add_test(NAME test_pid COMMAND test_pid)

add_executable(test_mixer
    tests/test_mixer.c
    src/control/src/mixer.c
    src/control/src/pid.c
)
target_include_directories(test_mixer PRIVATE
    src/control/include
    src/hal/include
    src/bindings
)
target_link_libraries(test_mixer m)
add_test(NAME test_mixer COMMAND test_mixer)
