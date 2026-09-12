idf_component_register(SRCS "main.c" "mcp2517fd.c" "can_tracker.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES driver esp_timer esp_hw_support)
