cmake_minimum_required(VERSION 3.10)

project(LikeVcom)

include_directories("inc")
aux_source_directory("src" SRC_DIR)

add_compile_options(-lpthread)

add_executable(LikeVcom ${SRC_DIR})
