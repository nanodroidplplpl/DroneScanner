idf_component_register(SRCS "scan.c"
                    PRIV_REQUIRES esp_wifi nvs_flash esp_netif lwip esp_driver_usb_serial_jtag
                    INCLUDE_DIRS ".")
