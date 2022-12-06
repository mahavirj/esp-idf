# Main component CMakeLists.txt
idf_component_register(
        SRC_DIRS "."
        INCLUDE_DIRS "."
    )

target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=calib_get_pubkey")
target_link_libraries(${COMPONENT_LIB} INTERFACE "-u __wrap_calib_get_pubkey")
