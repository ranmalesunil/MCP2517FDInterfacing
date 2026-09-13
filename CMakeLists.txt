cmake_minimum_required(VERSION 3.16)

# Build only main and its direct dependencies to speed up compilation
set(COMPONENTS main)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(can_fd_logger)
