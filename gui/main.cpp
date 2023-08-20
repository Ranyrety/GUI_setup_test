#include "hello_imgui/hello_imgui.h"

int main(int , char *[])
{

    bool showDemo = true;
    HelloImGui::RunnerParams runnerParams;
    runnerParams.appWindowParams.windowTitle = "M6800 - emulator gui demo";
    runnerParams.appWindowParams.windowGeometry.size = { 800,600 };

    runnerParams.callbacks.ShowGui =  [] {
        // Left
            static int selected = 0;
        {
            ImGui::BeginChild("left pane", ImVec2(ImGui::GetWindowWidth() * 0.40, -ImGui::GetFrameHeightWithSpacing()), true);
            for (int i = 0; i < 100; i++)
            {
                // FIXME: Good candidate to use ImGuiSelectableFlags_SelectOnNav
                char label[128];
                sprintf(label, "MyObject %d", i);
                if (ImGui::Selectable(label, selected == i))
                    selected = i;
            }
            ImGui::EndChild();
        }
        ImGui::SameLine();

        // Right
        {
            ImGui::BeginGroup();
            ImGui::BeginChild("item view", ImVec2(0, -ImGui::GetFrameHeightWithSpacing())); // Leave room for 1 line below us
            ImGui::Text("MyObject: %d", selected);
            ImGui::Separator();
            if (ImGui::BeginTabBar("##Tabs", ImGuiTabBarFlags_None))
            {
                if (ImGui::BeginTabItem("Description"))
                {
                    ImGui::TextWrapped("Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. ");
                    ImGui::EndTabItem();
                }
                if (ImGui::BeginTabItem("Details"))
                {
                    ImGui::Text("ID: 0123456789");
                    ImGui::EndTabItem();
                }
                ImGui::EndTabBar();
            }
            ImGui::EndChild();
            if (ImGui::Button("Revert")) {}
            ImGui::SameLine();
            if (ImGui::Button("Save")) {}
            ImGui::EndGroup();
        }
        };
    HelloImGui::Run(runnerParams);
    return 0;
}
