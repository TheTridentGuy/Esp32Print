idf_component_register(
  SRCS "ble_client.c"
  INCLUDE_DIRS "."
  REQUIRES bt nvs_flash
)