include_directories(../stm32_common)
include_directories(../stm32f4_disco-metal)

add_library(freertps_system_stm32f4_disco-metal console.c 
                                led.c 
                                systime.c
                                flash.c
                                enet_init_pins.c
                                ../stm32_common/stm32f4_vectors.c)

add_library(freertos_portable ../../FreeRTOS/FreeRTOS_Plus_UDP/portable/BufferManagement/BufferAllocation_2.c
                              ../../FreeRTOS/FreeRTOS_Plus_UDP/portable/NetworkInterface/STM32/NetworkInterface.c
                              ../../FreeRTOS/FreeRTOS/portable/GCC/ARM_CM4F/port.c
                              ../../FreeRTOS/FreeRTOS/portable/MemMang/heap_3.c
                              ../../FreeRTOS/FreeRTOS/portable/Common/mpu_wrappers.c)

set(SYSTEM_EXTRA_LIBS  "stm32_common metal_common" CACHE STRING "extra system libs")
set(SYSTEM_APPS  "talker listener pid" CACHE STRING "applications for this system")

#set(SYSTEM_NO_ROSIDL_APPS "talker_no_rosidl listener_no_rosidl" CACHE STRING "")
if (NOT freertps_standalone)
  ament_export_libraries(freertps_system_${SYSTEM})
endif()
install(
  TARGETS freertps_system_${SYSTEM}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
