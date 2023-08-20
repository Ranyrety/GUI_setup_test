cmake_minimum_required(VERSION 3.12)
project(gui_with_helloimgui)
set(CMAKE_CXX_STANDARD 17)
set(emuLib "emulatorLib")
##########################################################
# Prepare hello_imgui during configure time
##########################################################
include(FetchContent)
FetchContent_Declare(
    hello_imgui
    GIT_REPOSITORY https://github.com/pthom/hello_imgui.git
    # Enter the desired git tag below
    # GIT_TAG
)
FetchContent_MakeAvailable(hello_imgui)
# Make cmake function `hello_imgui_add_app` available
list(APPEND CMAKE_MODULE_PATH ${HELLOIMGUI_CMAKE_PATH})
include(hello_imgui_add_app)

##########################################################
# Build your app
##########################################################
hello_imgui_add_app(hello_world main.cpp)
add_dependencies( hello_world emulatorLib )
target_link_libraries(hello_world PRIVATE ${emuLib})
