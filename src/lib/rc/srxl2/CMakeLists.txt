# Native protocol code has no PX4 or RTOS dependencies so the same source is
# exercised on the host and linked into the receiver driver.
cmake_minimum_required(VERSION 3.16)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
	project(srxl2_protocol_tests LANGUAGES CXX)
	enable_testing()
	set(SRXL2_STANDALONE ON)
endif()

add_library(srxl2_protocol Srxl2.cpp)
target_include_directories(srxl2_protocol PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(srxl2_protocol PUBLIC cxx_std_14)
if(TARGET prebuild_targets)
	target_link_libraries(srxl2_protocol PRIVATE prebuild_targets)
endif()

if(SRXL2_STANDALONE OR (BUILD_TESTING AND PX4_PLATFORM STREQUAL "posix"))
	add_executable(srxl2_protocol_test tests/Srxl2Test.cpp)
	target_link_libraries(srxl2_protocol_test PRIVATE srxl2_protocol)
	target_compile_options(srxl2_protocol_test PRIVATE -UNDEBUG)
	add_test(NAME srxl2_protocol_test COMMAND srxl2_protocol_test)
	if(TARGET test_results)
		add_filtered_test_dependencies(srxl2_protocol_test)
	endif()
	option(SRXL2_SANITIZERS "Enable address and undefined behavior sanitizers" OFF)
	if(SRXL2_SANITIZERS)
		target_compile_options(srxl2_protocol PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
		target_compile_options(srxl2_protocol_test PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
		target_link_options(srxl2_protocol_test PRIVATE -fsanitize=address,undefined)
	endif()
endif()

if(NOT SRXL2_STANDALONE AND COMMAND px4_add_functional_gtest)
	px4_add_functional_gtest(
		SRC tests/Srxl2RCUpdateTest.cpp
		LINKLIBS srxl2_protocol modules__rc_update modules__manual_control
	)
endif()
