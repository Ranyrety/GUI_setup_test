cmake_minimum_required(VERSION 3.7)

project( M6800_UnitTests )
# GoogleTest requires at least C++14
set(CMAKE_CXX_STANDARD 14)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)



enable_testing()

set( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DM6800_DEBUG" )	#so we can add the DEBUG preprocessor define and other flags to stay in debug mode - see https://cmake.org/Wiki/CMake_Useful_Variables#Compilers_and_Tools
set( CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -DM6800_DEBUG" )

if(MSVC)
	add_compile_options(/MP)				#Use multiple processors when building
	add_compile_options(/W4 /wd4201 /WX)	#Warning level 4, all warnings are errors
else()
	add_compile_options(-W -Wall -Werror) #All Warnings, all warnings are errors
endif()

# Prevent overriding the parent project's compiler/linker
# settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# source for the test executable
set  (TEST_SOURCES
		"src/main.cpp"
		"src/testAccumulators.cpp"
        "src/testLogicOperations.cpp")
		
source_group("src" FILES ${TEST_SOURCES})
		
add_executable( M6800_TestMain ${TEST_SOURCES} 	)
add_dependencies( M6800_TestMain emulatorLib )
target_link_libraries(M6800_TestMain GTest::gtest_main)
target_link_libraries(M6800_TestMain emulatorLib)
include(GoogleTest)
gtest_discover_tests(M6800_TestMain)