##############################################################################
# Binaries
##############################################################################

add_executable(xbot_ftdi_scan ftdi_scan.cpp)
target_link_libraries(xbot_ftdi_scan ftdi)

add_executable(ftdi_xbot ftdi_xbot.cpp)
target_link_libraries(ftdi_xbot ftdi)

add_executable(xbot_ftdi_read_eeprom ftdi_read_eeprom.cpp)
target_link_libraries(xbot_ftdi_read_eeprom ftdi)

add_executable(xbot_ftdi_write_eeprom ftdi_write_eeprom.cpp)
target_link_libraries(xbot_ftdi_write_eeprom ftdi)

add_executable(xbot_find_devices find_devices.cpp)
target_link_libraries(xbot_find_devices usb)

add_executable(xbot_get_serial_number get_serial_number.cpp)
target_link_libraries(xbot_get_serial_number usb)

add_executable(xbot_flasher flasher.cpp)
target_link_libraries(xbot_flasher usb ftdi)

add_executable(xbot_reset_device reset_device.cpp)
target_link_libraries(xbot_reset_device usb)

add_executable(xbot_overwrite_serial_number overwrite_serial_number.cpp)
target_link_libraries(xbot_overwrite_serial_number usb ftdi)

add_executable(xbot_unflasher unflasher.cpp)
target_link_libraries(xbot_unflasher usb ftdi)

install(TARGETS xbot_ftdi_scan ftdi_xbot xbot_ftdi_read_eeprom xbot_ftdi_write_eeprom xbot_find_devices
                xbot_get_serial_number xbot_flasher xbot_reset_device xbot_overwrite_serial_number xbot_unflasher
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
