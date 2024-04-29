cmake_minimum_required(VERSION 3.8)
project(CodeCraftSDK)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

add_compile_definitions(DISABLE_PTHREAD_HOOK)

if (NOT WIN32)
    link_libraries(pthread rt m)
endif (NOT WIN32)

find_package(CURL REQUIRED)

AUX_SOURCE_DIRECTORY(. src)
list(REMOVE_ITEM src "./load_pthread.cpp")

add_executable(main ${src})
target_link_libraries(main CURL::libcurl)