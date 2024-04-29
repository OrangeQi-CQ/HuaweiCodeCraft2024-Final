cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_COMPILER g++)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (WIN32)
    MATH(EXPR stack_size "500*1024*1024")
    set(CMAKE_EXE_LINKER_FLAGS "-Wl,--stack,${stack_size}")
endif()

if(CMAKE_BINARY_DIR STREQUAL CMAKE_SOURCE_DIR)
    message(FATAL_ERROR "Please run cmake in directory 'build'!")
endif()


project(HuaweiCodeCraft2024
    LANGUAGES CXX
)

add_compile_definitions(DISABLE_PTHREAD_HOOK)

if (NOT WIN32)
    if (APPLE)
        link_libraries(pthread m)
        find_package(CURL REQUIRED)
    else ()
        link_libraries(pthread rt m)
        find_package(CURL REQUIRED)
    endif ()
endif (NOT WIN32)

file(COPY "judge" DESTINATION .)
file(COPY "scripts" DESTINATION .)
    
aux_source_directory(src src)
list(REMOVE_ITEM src "./load_pthread.cpp")
add_executable(main ${src})
if(NOT WIN32)
    target_link_libraries(main CURL::libcurl)
endif()
target_compile_options(main PRIVATE -O2 -DLOCAL)

if(WIN32)
    find_path(CURL_INCLUDE_DIR NAMES curl/curl.h PATHS
    ${CURL_DIR}/include)
    find_library(CURL_LIBRARY NAMES libcurl PATHS ${CURL_DIR}/lib)
    include_directories(${CURL_INCLUDE_DIR})
    target_link_libraries(main ${CURL_LIBRARY})
endif()




add_executable(main_debug ${src})
if(NOT WIN32)
    target_link_libraries(main_debug CURL::libcurl)
endif()
target_compile_options(main_debug PRIVATE -g -DDEBUG -DLOCAL -O2)

set(PYTHON python3)
if(WIN32)
    set(PYTHON python)
endif()

enable_testing()

set(CTEST_FULL_OUTPUT TRUE)
set(TEST_NAMES 1)

foreach(name IN LISTS TEST_NAMES)
    add_custom_target(${name} COMMAND ${PYTHON} scripts/run.py --map ${name})
    add_dependencies(${name}  main)
    add_custom_target(debug-${name} COMMAND ${PYTHON} scripts/run.py --map ${name} --debug True)
    add_dependencies(debug-${name} main_debug)
endforeach()
    
add_custom_target(run COMMAND ${PYTHON} scripts/run.py --map 0)
add_dependencies(run  main)

add_custom_target(zip
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/output
    COMMAND ${CMAKE_COMMAND} -E env current_time=$ENV{DATE} zip -9 -r ${CMAKE_BINARY_DIR}/output/\${current_time}.zip ${CMAKE_SOURCE_DIR}/src/*.cpp ${CMAKE_SOURCE_DIR}/src/*.hpp
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)


if(WIN32)
    set(REPLAY_COMMANDS
        COMMAND ${CMAKE_COMMAND} -E chdir ${CMAKE_BINARY_DIR}/judge/windows mkdir -p replay
        COMMAND ${CMAKE_COMMAND} -E chdir ${CMAKE_BINARY_DIR}/judge/windows/replay ../CodeCraft_2024_Replayer_v2.exe
    )
elseif(APPLE)
    set(REPLAY_COMMANDS
        COMMAND ${CMAKE_COMMAND} -E chdir ${CMAKE_BINARY_DIR}/judge/mac mkdir -p replay
        COMMAND ${CMAKE_COMMAND} -E chdir ${CMAKE_BINARY_DIR}/judge/mac chmod +x CodeCraft_2024_Replayer_mac.app/Contents/MacOS/CodeCraft_2024_Replayer_mac
        COMMAND ${CMAKE_COMMAND} -E chdir ${CMAKE_BINARY_DIR}/judge/mac/replay ../CodeCraft_2024_Replayer_mac.app/Contents/MacOS/CodeCraft_2024_Replayer_mac
    )
else()
    set(REPLAY_COMMANDS
        COMMAND ${CMAKE_COMMAND} -E chdir ${CMAKE_BINARY_DIR}/judge/linux mkdir -p replay
        COMMAND ${CMAKE_COMMAND} -E chdir ${CMAKE_BINARY_DIR}/judge/linux/replay ../CodeCraft_2024_Replayer_final.x86_64
    )
endif()

add_custom_target(replay ${REPLAY_COMMANDS})

