#include "hello_imgui/hello_imgui.h"
#include "TextEditor.h"
#include "imgui_memory_editor.h"

int main(int , char *[])
{
    bool showDemo = true;
    HelloImGui::RunnerParams runnerParams;
    runnerParams.appWindowParams.windowTitle = "M6800 - emulator gui demo";
    runnerParams.appWindowParams.windowGeometry.size = { 800,600 };

            MemoryEditor memEditor;
            memEditor.Cols = 8;
            char buff[0xFFFF];
            char* codeBuf = (char*)calloc(1024, sizeof(uint8_t));
            TextEditor editor;
            
    runnerParams.callbacks.ShowGui =  [&editor, codeBuf, &memEditor, buff] {
        // Left
            static int selected = 0;
            
        {
            ImGui::BeginChild("left pane", ImVec2(ImGui::GetWindowWidth() * 0.40, -ImGui::GetFrameHeightWithSpacing()), true);
            //ImGui::ShowDemoWindow(nullptr);
            editor.Render("Dissasembly");
            ImGui::EndChild();
        }
        ImGui::SameLine();

        // Right
        {
            ImGui::BeginGroup();
            ImGui::BeginChild("item view", ImVec2(0, 150)); // Leave room for 1 line below us
            ImGui::Text("MyObject: %d", selected);
            ImGui::EndChild();
            ImGui::Separator();
            ImGui::BeginChild("Memory", ImVec2(0, -ImGui::GetFrameHeightWithSpacing())); // Leave room for 1 line below us
            memEditor.DrawContents((void*)buff, sizeof(buff), sizeof(char));
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
