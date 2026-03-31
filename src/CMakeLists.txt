add_executable(${ProjectName}
    main.c
)

target_include_directories(${ProjectName} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(${ProjectName}
    pico_stdlib
    pico_cyw43_arch_none
    FreeRTOS-Kernel-Heap4
    )

pico_add_extra_outputs(${ProjectName})