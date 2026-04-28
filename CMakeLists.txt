add_library(devices_ina228 STATIC
    src/ina228.cpp
    src/ina228_driver.cpp
)

target_include_directories(devices_ina228
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(devices_ina228
    PRIVATE
        stm32cubemx       
        fsw_data
        fsw::devices::board_config 
        fsw::devices::device_interface
)

add_library(fsw::devices::ina228 ALIAS devices_ina228)