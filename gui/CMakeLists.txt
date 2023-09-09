cmake_minimum_required(VERSION 3.12)
project(gui_with_helloimgui)
set(CMAKE_CXX_STANDARD 17)
set(emuLib "emulatorLib")
set(utils "UTILITIES_LIB")
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
file(GLOB TEXTEDITOR_SRC
    ImGuiColorTextEditor/*.cpp
    ImGuiColorTextEditor/*.h
)
hello_imgui_add_app(test_gui main.cpp ImGuiColorTextEdit/TextEditor.cpp )
target_include_directories(test_gui PUBLIC ImGuiColorTextEdit imgui_club/imgui_memory_editor)
add_dependencies( test_gui emulatorLib )
add_dependencies( test_gui UTILITIES_LIB)

target_link_libraries(test_gui PRIVATE ${emuLib})
target_link_libraries(test_gui PRIVATE ${utils})
