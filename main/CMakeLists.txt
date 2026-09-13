idf_component_register(SRCS "can_tasks.c" "main.c" "mcp2517fd.c" "can_tracker.c" "usb_comm.c" "usb_protocol.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES driver esp_timer esp_hw_support esp_driver_usb_serial_jtag esp_driver_uart)
