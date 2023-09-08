# Set the library name
project( emulatorLib )


# Add the source files
file(GLOB EMULATOR_SRC
    "src/*.cpp"
    "include/*.h"
)


# Create the library
add_library(${PROJECT_NAME} ${EMULATOR_SRC})
target_include_directories(${PROJECT_NAME} PUBLIC "${PROJECT_SOURCE_DIR}/include")

# Add additional executable target
add_executable(emulator_main
    main.cpp
)
target_include_directories(emulator_main PUBLIC "${PROJECT_SOURCE_DIR}/include")
add_dependencies( emulator_main emulatorLib )

target_link_libraries(emulator_main emulatorLib )
target_link_libraries(emulator_main UTILITIES_LIB )
