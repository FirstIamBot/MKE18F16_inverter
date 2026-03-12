if(NOT DEFINED FPU)
	set(FPU "-mfpu=fpv4-sp-d16 -mfloat-abi=hard")
endif()

if(${LIBRARY_TYPE} STREQUAL "REDLIB")
	set(SPECS "-specs=redlib.specs")
elseif(${LIBRARY_TYPE} STREQUAL "NEWLIB_NANO")
	set(SPECS "--specs=nano.specs")
endif()

if(NOT DEFINED DEBUG_CONSOLE_CONFIG)
	set(DEBUG_CONSOLE_CONFIG "-DSDK_DEBUGCONSOLE=0")
endif()

set(CMAKE_ASM_FLAGS_DEBUG " \
    ${CMAKE_ASM_FLAGS_DEBUG} \
    ${FPU} \
    -mcpu=cortex-m4 \
    -mthumb \
")

set(CMAKE_C_FLAGS_DEBUG " \
    ${CMAKE_C_FLAGS_DEBUG} \
    ${FPU} \
    ${DEBUG_CONSOLE_CONFIG} \
    -DCPU_MKE18F512VLH16 \
    -DCPU_MKE18F512VLH16_cm4 \
    -DFSL_RTOS_BM \
    -DSDK_OS_BAREMETAL \
    -DSERIAL_PORT_TYPE_UART=1 \
    -DCR_INTEGER_PRINTF \
    -DPRINTF_FLOAT_ENABLE=0 \
    -DSERIAL_PORT_TYPE_SWO=1 \
    -D__MCUXPRESSO \
    -D__USE_CMSIS \
    -DDEBUG \
    -DFSL_RTOS_FREE_RTOS \
    -DSDK_OS_FREE_RTOS \
    -O0 \
    -fno-common \
    -fmerge-constants \
    -g3 \
     -ffunction-sections -fdata-sections -fno-builtin \
    -fstack-usage \
    -mcpu=cortex-m4 \
    -mthumb \
")

set(CMAKE_CXX_FLAGS_DEBUG " \
    ${CMAKE_CXX_FLAGS_DEBUG} \
    ${FPU} \
    ${DEBUG_CONSOLE_CONFIG} \
    -O0 \
    -fno-common \
    -fmerge-constants \
    -g3 \
    -Wall \
    -fstack-usage \
    -mcpu=cortex-m4 \
    -mthumb \
")

set(CMAKE_EXE_LINKER_FLAGS_DEBUG " \
    ${CMAKE_EXE_LINKER_FLAGS_DEBUG} \
    ${FPU} \
    ${SPECS} \
    -nostdlib \
    -Xlinker \
    -Map=output.map \
    -Xlinker \
    --gc-sections \
    -Xlinker \
    -print-memory-usage \
    -Xlinker \
    --sort-section=alignment \
    -Xlinker \
    --cref \
    -mcpu=cortex-m4 \
    -mthumb \
    -T\"${ProjDirPath}/MKE18F16_inverter_Debug.ld\" \
")
