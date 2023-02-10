project(LSM6DSOX C CXX)
add_library(LSM6DSOX INTERFACE)
target_sources(LSM6DSOX INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/LSM6DSOX.cpp
        )
target_include_directories(LSM6DSOX INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/src)
#target_compile_definitions(LSM6DSOX INTERFACE PLATFORM_ALLOW_UNKNOWN_OS)

list(APPEND DEPS foundation-logging)


if(NOT ADMORPH_PLATFORM STREQUAL "posix")
    # we have a RP2040 platform
    list(APPEND DEPS pico_stdlib pico_multicore hardware_i2c hardware_irq i2c_slave)

    if(PICO_BOARD STREQUAL "pico_w")
        # we have pico_w
        list(APPEND DEPS pico_cyw43_arch_none)
    endif()
endif()

# optionally disable logging
if(ADMORPH_DISABLE_LOGGING)
    message("Disabling logging!")
    target_compile_definitions(LSM6DSOX PRIVATE FOUNDATION_LOGGING_DISABLED)
endif()

target_link_libraries(LSM6DSOX INTERFACE ${DEPS})

