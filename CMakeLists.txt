cmake_minimum_required(VERSION 3.10)

# 项目名称
project(mpc_pos_lcm)

# 指定C标准
set(CMAKE_C_STANDARD 11)

# 包含目录
include_directories(${CMAKE_SOURCE_DIR} /usr/local/etherlab/include)

# 编译器标志
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")

# 定义源文件列表
set(T_SRCS main_0803.c ecat_data_buffer.c)
set(CSP_SRCS main_0710.c ecat_data_buffer.c)

# 添加可执行文件 T
add_executable(T ${T_SRCS})
target_include_directories(T PRIVATE /usr/local/etherlab/include)
target_link_directories(T PRIVATE /usr/local/etherlab/lib)
target_link_libraries(T PRIVATE pthread rt ethercat)

# 添加可执行文件 CSP
add_executable(CSP ${CSP_SRCS})
target_include_directories(CSP PRIVATE /usr/local/etherlab/include)
target_link_directories(CSP PRIVATE /usr/local/etherlab/lib)
target_link_libraries(CSP PRIVATE pthread rt ethercat)

# 设置清理目标
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${CMAKE_BINARY_DIR})

