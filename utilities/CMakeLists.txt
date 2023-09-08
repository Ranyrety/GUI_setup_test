# Set the library name
project(UTILITIES_LIB)

# Add the source files using GLOB
file(GLOB UTILITIES_SRC
    "src/*.cpp"
    "include/*.h"
)

# Add the include directories
include_directories(include)

# Create the library
add_library(${PROJECT_NAME} STATIC ${UTILITIES_SRC})
target_include_directories(${PROJECT_NAME} PUBLIC "${PROJECT_SOURCE_DIR}/include")