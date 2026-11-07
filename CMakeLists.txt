cmake_minimum_required(VERSION 3.20)
project(keel_mobile VERSION 0.1.0 LANGUAGES CXX)

option(KEEL_MOBILE_NO_EXCEPTIONS_RTTI "Compile keel_mobile without C++ exceptions and RTTI" OFF)
option(KEEL_MOBILE_BUILD_TESTS "Build keel_mobile unit tests and header purity checks" ${PROJECT_IS_TOP_LEVEL})

if( NOT TARGET keel::keel )
    if( EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/keel/CMakeLists.txt" )
        add_subdirectory(third_party/keel EXCLUDE_FROM_ALL)
    elseif( EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../keel/CMakeLists.txt" )
        add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../keel" "${CMAKE_CURRENT_BINARY_DIR}/keel" EXCLUDE_FROM_ALL)
    else()
        find_package(keel REQUIRED)
    endif()
endif()

add_library(keel_mobile STATIC
    src/sensors.cpp
    src/location.cpp
    src/audio.cpp
    src/camera.cpp
    src/system.cpp
)
add_library(keel::mobile ALIAS keel_mobile)

target_compile_features(keel_mobile PUBLIC cxx_std_20)
set_target_properties(keel_mobile PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

target_include_directories(keel_mobile PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(keel_mobile PUBLIC keel::keel)

if( ANDROID )
    find_library(ANDROID_LIB android REQUIRED)
    find_library(ANDROID_LOG_LIB log REQUIRED)
    find_library(ANDROID_CAMERA2NDK_LIB camera2ndk REQUIRED)
    find_library(ANDROID_MEDIANDK_LIB mediandk REQUIRED)
    find_library(ANDROID_AAUDIO_LIB aaudio REQUIRED)
    target_link_libraries(keel_mobile PUBLIC
        ${ANDROID_LIB}
        ${ANDROID_LOG_LIB}
        ${ANDROID_CAMERA2NDK_LIB}
        ${ANDROID_MEDIANDK_LIB}
        ${ANDROID_AAUDIO_LIB}
    )
elseif( WIN32 )
    target_link_libraries(keel_mobile PUBLIC winmm)
endif()

if( MSVC )
    target_compile_options(keel_mobile PRIVATE /W4 /utf-8)
    if( KEEL_MOBILE_NO_EXCEPTIONS_RTTI )
        string(REGEX REPLACE "/EH[a-z]+" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
        string(REGEX REPLACE "/GR" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
        target_compile_options(keel_mobile PUBLIC /EHs-c- /GR-)
        target_compile_definitions(keel_mobile PUBLIC _HAS_EXCEPTIONS=0)
    endif()
else()
    target_compile_options(keel_mobile PRIVATE -Wall -Wextra -Wpedantic)
    if( KEEL_MOBILE_NO_EXCEPTIONS_RTTI )
        target_compile_options(keel_mobile PUBLIC -fno-exceptions -fno-rtti)
    endif()
endif()

# Install & package support (`find_package(keel_mobile)`)
include(CMakePackageConfigHelpers)
install(TARGETS keel_mobile EXPORT keel_mobileTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
install(DIRECTORY include/keel DESTINATION include)
install(EXPORT keel_mobileTargets
    FILE keel_mobileTargets.cmake
    NAMESPACE keel::
    DESTINATION lib/cmake/keel_mobile
)
configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/keel_mobileConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/keel_mobileConfig.cmake"
    INSTALL_DESTINATION lib/cmake/keel_mobile
)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/keel_mobileConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMinorVersion
)
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/keel_mobileConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/keel_mobileConfigVersion.cmake"
    DESTINATION lib/cmake/keel_mobile
)

if( KEEL_MOBILE_BUILD_TESTS AND NOT ANDROID )
    enable_testing()

    add_executable(test_mobile tests/test_mobile.cpp)
    target_link_libraries(test_mobile PRIVATE keel::mobile)
    if( MSVC )
        target_compile_options(test_mobile PRIVATE /W4 /utf-8)
    else()
        target_compile_options(test_mobile PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    add_test(NAME test_mobile COMMAND test_mobile)

    get_target_property(KEEL_CORE_INC_DIRS keel::keel INTERFACE_INCLUDE_DIRECTORIES)
    list(GET KEEL_CORE_INC_DIRS 0 KEEL_CORE_INC_DIR)
    string(REGEX REPLACE "^\\$<BUILD_INTERFACE:([^>]+)>$" "\\1" KEEL_CORE_INC_DIR "${KEEL_CORE_INC_DIR}")

    string(REPLACE ";" "|" KEEL_STD_INC_JOINED "${CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES}")
    add_test(
        NAME test_header_purity
        COMMAND ${CMAKE_COMMAND}
            "-DKEEL_MOBILE_INCLUDE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/include"
            "-DKEEL_CORE_INCLUDE_DIR=${KEEL_CORE_INC_DIR}"
            "-DCOMPILER_PATH=${CMAKE_CXX_COMPILER}"
            "-DCOMPILER_ID=${CMAKE_CXX_COMPILER_ID}"
            "-DSTD_INC_DIRS=${KEEL_STD_INC_JOINED}"
            -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/CheckHeaderPurity.cmake"
    )
endif()
