#include "hello_imgui/hello_imgui.h"

int main(int , char *[])
{

    bool showDemo = true;
    HelloImGui::RunnerParams runnerParams;
    runnerParams.appWindowParams.windowTitle = "M6800 - emulator gui demo";
    runnerParams.appWindowParams.windowGeometry.size = { 800,600 };

    // ImGui window params
    runnerParams.imGuiWindowParams.defaultImGuiWindowType =
        HelloImGui::DefaultImGuiWindowType::ProvideFullScreenDockSpace;
    runnerParams.imGuiWindowParams.showMenuBar = true;
    runnerParams.imGuiWindowParams.showStatusBar = true;

    runnerParams.dockingParams.dockingSplits = {
        {"Code", "Status", ImGuiDir_Left, 0.60f},
    };
    HelloImGui::DockableWindow dock_imguiDemoWindow;
    {
        dock_imguiDemoWindow.label = "Dear ImGui Demo";
        dock_imguiDemoWindow.dockSpaceName = "Code";// This window goes into "MainDockSpace"
        dock_imguiDemoWindow.GuiFunction = [&dock_imguiDemoWindow] {
            if (dock_imguiDemoWindow.isVisible)
                ImGui::ShowDemoWindow(nullptr);
        };
        dock_imguiDemoWindow.callBeginEnd = false;
        dock_imguiDemoWindow.isVisible = true;
    };

    HelloImGui::DockableWindow dock_imguiDemoCode;
    {
        dock_imguiDemoCode.label = "CPU status";
        dock_imguiDemoCode.dockSpaceName = "Status";// This window goes into "CodeSpace"
        dock_imguiDemoCode.isVisible = true;
        dock_imguiDemoCode.GuiFunction = [] { ImGui::Text("Here will be all status registers and memory"); };
        dock_imguiDemoCode.imGuiWindowFlags = ImGuiWindowFlags_HorizontalScrollbar;
        dock_imguiDemoCode.callBeginEnd = true;
        dock_imguiDemoCode.windowSize = ImVec2(400, 600);
    };

    runnerParams.dockingParams.dockableWindows = {
        dock_imguiDemoWindow,
        dock_imguiDemoCode
    };
    HelloImGui::Run(runnerParams);
    return 0;
}
