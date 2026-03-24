add_library(devices_ina228 STATIC
    ina228.cpp
)

target_include_directories(devices_ina228
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(devices_ina228
    PRIVATE
        stm32cubemx       
        fsw_data
        fsw::devices::board_config 
)

add_library(fsw::devices::ina228 ALIAS devices_ina228)