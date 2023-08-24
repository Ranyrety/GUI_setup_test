#include "hello_imgui/hello_imgui.h"
#include "TextEditor.h"
#include "imgui_memory_editor.h"
#include "utilities.h"
#include "mpu.h"

int main(int , char *[])
{
    bool showDemo = true;
    HelloImGui::RunnerParams runnerParams;
    runnerParams.appWindowParams.windowTitle = "M6800 - emulator gui demo";
    runnerParams.appWindowParams.windowGeometry.size = { 800,600 };

            MemoryEditor memEditor;
            memEditor.Cols = 8;
            char* codeBuf = (char*)calloc(1024, sizeof(uint8_t));
            TextEditor editor;
                Emulator::Mpu mpu;
            bool dataLoaded = false;
            std::vector<HexRecord> hexRecords = parseHex("test01.hex");

            if (!hexRecords.empty())

                dataLoaded = true;

            if (dataLoaded) {
                std::cout << "Hello m6800 emulator is greeting you" << std::endl;
                mpu.init(hexRecords[0].data, hexRecords[0].address);
                mpu.setAccA(0x00);
                mpu.setAccB(0x01);
                //act
            }
                int cycles = mpu.execute();
                auto buff = mpu.getMemory().getData();
    runnerParams.callbacks.ShowGui =  [&editor, codeBuf, &memEditor,  buff] {
        {
            ImGui::BeginChild("left pane", ImVec2(ImGui::GetWindowWidth() * 0.40, -ImGui::GetFrameHeightWithSpacing()), true);
            
            if (ImGui::BeginTabBar("##Tabs", ImGuiTabBarFlags_None))
            {
                if (ImGui::BeginTabItem("Operations"))
                {
                    for (int i = 0; i < 1024; i++) {
                        char label[128];
                        sprintf(label, "%04d :: %02X ", i, (uint8_t)buff[i]);
                        ImGui::Text(label);
                    }
                    ImGui::EndTabItem();
                }

                if (ImGui::BeginTabItem("Disassembly"))
                {
                    ImGui::TextWrapped("Here is going to be shown disassembled code if it will be possible to achieve");
                    editor.Render("Dissasembly");
                    ImGui::EndTabItem();
                }
                ImGui::EndTabBar();
            }
            ImGui::EndChild();
        }
        ImGui::SameLine();

        // Right
        {
            ImGui::BeginGroup();
            ImGui::BeginChild("item view", ImVec2(0, 150)); // Leave room for 1 line below us
            ImGui::Text("MyObject: 0002");
            ImGui::EndChild();
            ImGui::Separator();
            ImGui::BeginChild("Memory", ImVec2(0, -ImGui::GetFrameHeightWithSpacing())); // Leave room for 1 line below us
            memEditor.DrawContents((void*)buff, 0xFFFF, sizeof(char));
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
