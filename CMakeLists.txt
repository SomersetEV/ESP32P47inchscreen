cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# esp_hosted (both 1.4.x and 2.0.x) includes driver/sdmmc_host.h and
# esp_dma_utils.h without listing esp_driver_sdmmc / esp_driver_dma in its
# REQUIRES. Both headers exist in IDF 6.0.1, they are just not on the include
# path for that component. Adding them to the common requirements fixes the
# build without patching anything under managed_components/.
foreach(dep esp_driver_sdmmc esp_driver_dma esp_driver_gpio esp_driver_spi esp_driver_sdio esp_driver_uart)
    idf_build_set_property(__COMPONENT_REQUIRES_COMMON ${dep} APPEND)
endforeach()

project(tractor_dash)
