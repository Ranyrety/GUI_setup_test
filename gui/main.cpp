#include "hello_imgui/hello_imgui.h"
#include "TextEditor.h"
#include "imgui_memory_editor.h"
#include "mpu.h"
#include "utilities.h"

void itoab(uint8_t val, char* buff) {
    if (val != 0) {
        int i = 8;
        while (i) {
            if (val & 1)
                buff[i-1] = '1';
            else
                buff[i-1] = '0';
            val >>= 1;
            i--;
        }
    }
    else {
        buff[0] = buff[1] = buff[2] = buff[3] = buff[4] = buff[5] = buff[6] = buff[7] = '0';
    }
    buff[8] = '\0';
}

ImFont * gCustomFont = nullptr;
void MyLoadFonts()
{

    HelloImGui::ImGuiDefaultSettings::LoadDefaultFont_WithFontAwesomeIcons(); // The font that is loaded first is the default font
    gCustomFont = HelloImGui::LoadFontTTF("DroidSans.ttf", 20.f); // will be loaded from the assets folder
}

struct DissasemblyInfo {
    uint16_t operationAddres;
    char label[6];
    union {
        uint8_t data8;
        uint16_t data16;
    };
        uint8_t type;
};

uint16_t getWord(std::vector<uint8_t>::iterator it) {
    uint16_t word = 0;
    uint8_t msb = *it;
    uint8_t lsb = *(it+1);
    word = msb << 8;
    word = word | lsb;
    return word;
};

std::vector<DissasemblyInfo> disassemble(HexRecord &record) {
    auto result = std::vector<DissasemblyInfo>();
    uint16_t address = record.address;
    auto input = record.data;
    for (std::vector<uint8_t>::iterator it = input.begin(); it != input.end(); ++it)
    {
        DissasemblyInfo info = DissasemblyInfo();
        switch (*it)
        {
        case Emulator::ABA:
            strcpy(info.label, "ABA");
            info.operationAddres = address;
            info.type = 0;
            break;
        case Emulator::ADC_A_data8:
            strcpy(info.label, "ADCA");
            info.operationAddres = address;
            info.data8 = *++it;
            info.type = 1;
            address += 2;
            break;
        case Emulator::ADC_A_addr8:
            strcpy(info.label, "ADCA");
            info.operationAddres = address;
            info.data8 = *++it;
            info.type = 2;
            address += 2;
            break;
        case Emulator::ADC_A_addr16:
            strcpy(info.label, "ADCA");
            info.operationAddres = address;
            info.data16 = getWord(it);
            info.type = 3;
            address += 3;
            break;
        case Emulator::ADC_A_data8_X:
            strcpy(info.label, "ADCA");
            info.operationAddres = address;
            info.data8 = *++it;
            info.type = 4;
            address += 2;
            break;
        case Emulator::ADC_B_data8:
            strcpy(info.label, "ADCB");
            info.operationAddres = address;
            info.data8 = *++it;
            info.type = 1;
            address += 2;
            break;
        case Emulator::ADC_B_addr8:
            strcpy(info.label, "ADCB");
            info.operationAddres = address;
            info.data8 = *++it;
            info.type = 2;
            address += 2;
            break;
        case Emulator::ADC_B_addr16:
            strcpy(info.label, "ADCB");
            info.operationAddres = address;
            info.data16 = getWord(it);
            info.type = 3;
            address += 3;
            break;
        case Emulator::ADC_B_data8_X:
            strcpy(info.label, "ADCB");
            info.operationAddres = address;
            info.data8 = *++it;
            info.type = 3;
            address += 2;
            break;
        case Emulator::ADD_A_data8:
            strcpy(info.label, "ADDA");
            info.operationAddres = address;
            info.data8 = *++it;
            info.type = 1;
            address += 2;
            break;
        case Emulator::ADD_A_addr8:
            strcpy(info.label, "ADDA");
            info.operationAddres = address;
            info.data8 = *++it;
            info.type = 2;
            address += 2;
            break;
        case Emulator::ADD_A_addr16:
            break;
        case Emulator::ADD_A_data8_X:
            strcpy(info.label, "ADDA");
            info.operationAddres = address;
            info.data8 = *++it;
            info.type = 3;
            address += 1;
            break;
        case Emulator::ADD_B_data8:
            strcpy(info.label, "ADDB");
            info.operationAddres = address;
            info.data8 = *++it;
            info.type = 1;
            address += 2;
            break;
        case Emulator::ADD_B_addr8:
            strcpy(info.label, "ADDB");
            info.operationAddres = address;
            info.data8 = *++it;
            info.type = 2;
            address += 2;
            break;
        case Emulator::ADD_B_addr16:
            break;
        case Emulator::ADD_B_data8_X:
            strcpy(info.label, "ADDB");
            info.operationAddres = address;
            info.data8 = *++it;
            info.type = 3;
            address += 2;
            break;
        case Emulator::AND_A_data8:
            strcpy(info.label, "ANDA");
            info.operationAddres = address;
            info.data8 = *++it;
            info.type = 1;
            address += 2;
            break;
        case Emulator::AND_A_addr8:
            strcpy(info.label, "ANDA");
            info.operationAddres = address;
            info.data8 = *++it;
            info.type = 2;
            address += 2;
            break;
        case Emulator::AND_A_addr16:
            break;
        case Emulator::AND_A_data8_X:
            strcpy(info.label, "ANDA");
            info.operationAddres = address;
            info.data8 = *++it;
            info.type = 3;
            address += 2;
            break;
        case Emulator::AND_B_data8:
            break;
        case Emulator::AND_B_addr8:
            break;
        case Emulator::AND_B_addr16:
            break;
        case Emulator::AND_B_data8_X:
            break;
        case Emulator::ASL_A:
            break;
        case Emulator::ASL_B:
            break;
        case Emulator::ASL_addr16:
            break;
        case Emulator::ASL_data8_X:
            break;
        case Emulator::ASR_A:
            break;
        case Emulator::ASR_B:
            break;
        case Emulator::ASR_addr16:
            break;
        case Emulator::ASR_data8_X:
            break;
        case Emulator::BCC_rel8:
            break;
        case Emulator::BCS_rel8:
            break;
        case Emulator::BEQ_rel8:
            break;
        case Emulator::BGE_rel8:
            break;
        case Emulator::BGT_rel8:
            break;
        case Emulator::BHI_rel8:
            break;
        case Emulator::BIT_A_data8:
            break;
        case Emulator::BIT_A_addr8:
            break;
        case Emulator::BIT_A_addr16:
            break;
        case Emulator::BIT_A_data8_X:
            break;
        case Emulator::BIT_B_data8:
            break;
        case Emulator::BIT_B_addr8:
            break;
        case Emulator::BIT_B_addr16:
            break;
        case Emulator::BIT_B_data8_X:
            break;
        case Emulator::BLE_rel8:
            break;
        case Emulator::BLS_rel8:
            break;
        case Emulator::BLT_rel8:
            break;
        case Emulator::BMI_rel8:
            break;
        case Emulator::BNE_rel8:
            break;
        case Emulator::BPL_rel8:
            break;
        case Emulator::BRA_rel8:
            break;
        case Emulator::BSR_rel8:
            break;
        case Emulator::BVC_rel8:
            break;
        case Emulator::BVS_rel8:
            break;
        case Emulator::CBA:
            break;
        case Emulator::CLC:
            break;
        case Emulator::CLI:
            break;
        case Emulator::CLR_A:
            break;
        case Emulator::CLR_B:
            break;
        case Emulator::CLR_data8_X:
            break;
        case Emulator::CLR_addr16:
            break;
        case Emulator::CLV:
            break;
        case Emulator::CMP_A_data8:
            break;
        case Emulator::CMP_A_addr8:
            break;
        case Emulator::CMP_A_data8_X:
            break;
        case Emulator::CMP_A_addr16:
            break;
        case Emulator::CMP_B_data8:
            break;
        case Emulator::CMP_B_addr8:
            break;
        case Emulator::CMP_B_data8_X:
            break;
        case Emulator::CMP_B_addr16:
            break;
        case Emulator::COM_A:
            break;
        case Emulator::COM_B:
            break;
        case Emulator::COM_data8_X:
            break;
        case Emulator::COM_addr16:
            break;
        case Emulator::DAA:
            break;
        case Emulator::DEC_A:
            break;
        case Emulator::DEC_B:
            break;
        case Emulator::DEC_data8_X:
            break;
        case Emulator::DEC_addr16:
            break;
        case Emulator::DES:
            break;
        case Emulator::DEX:
            break;
        case Emulator::EOR_A_data8:
            break;
        case Emulator::EOR_A_addr8:
            break;
        case Emulator::EOR_A_data8_X:
            break;
        case Emulator::EOR_A_addr16:
            break;
        case Emulator::EOR_B_data8:
            break;
        case Emulator::EOR_B_addr8:
            break;
        case Emulator::EOR_B_data8_X:
            break;
        case Emulator::EOR_B_addr16:
            break;
        case Emulator::INC_A:
            strcpy(info.label, "INCA");
            info.type = 0;
            info.operationAddres = address;
            address += 1;
            break;
        case Emulator::INC_B:
            break;
        case Emulator::INC_data8_X:
            break;
        case Emulator::INC_addr16:
            break;
        case Emulator::INS:
            break;
        case Emulator::INX:
            break;
        case Emulator::JMP_data8_X:
            break;
        case Emulator::JMP_addr16:
            break;
        case Emulator::JSR_data8_X:
            break;
        case Emulator::JSR_addr16:
            break;
        case Emulator::LDA_A_data8:
            strcpy(info.label, "LDAA");
            info.operationAddres = address;
            info.data8 = *++it;
            info.type = 3;
            address += 2;
            break;
        case Emulator::LDA_A_addr8:
            strcpy(info.label, "LDAA");
            info.operationAddres = address;
            info.data8 = *++it;
            info.type = 3;
            address += 2;
            break;
        case Emulator::LDA_A_data8_X:
            strcpy(info.label, "LDAA");
            info.operationAddres = address;
            info.data8 = *++it;
            info.type = 3;
            address += 2;
            break;
        case Emulator::LDA_A_addr16:
            strcpy(info.label, "LDAA");
            info.operationAddres = address;
            info.data16 = getWord(it);
            info.type = 3;
            address += 2;
            break;
        case Emulator::LDA_B_data8:
            strcpy(info.label, "LDAB");
            info.operationAddres = address;
            info.data8 = *++it;
            info.type = 3;
            address += 2;
            break;
        case Emulator::LDA_B_addr8:
            strcpy(info.label, "LDAB");
            info.operationAddres = address;
            info.data8 = *++it;
            info.type = 3;
            address += 2;
            break;
        case Emulator::LDA_B_data8_X:
            strcpy(info.label, "LDAB");
            info.operationAddres = address;
            info.data8 = *++it;
            info.type = 3;
            address += 2;
            break;
        case Emulator::LDA_B_addr16:
            strcpy(info.label, "LDAB");
            info.operationAddres = address;
            info.data16 =getWord(it);
            info.type = 4;
            address += 3;
            break;
        case Emulator::LDS_data16:
            break;
        case Emulator::LDS_addr8:
            break;
        case Emulator::LDS_data8_X:
            break;
        case Emulator::LDS_addr16:
            break;
        case Emulator::LDX_data16:
            break;
        case Emulator::LDX_addr8:
            break;
        case Emulator::LDX_data8_X:
            break;
        case Emulator::LDX_addr16:
            break;
        case Emulator::LSR_A:
            break;
        case Emulator::LSR_B:
            break;
        case Emulator::LSR_data8_X:
            break;
        case Emulator::LSR_addr16:
            break;
        case Emulator::NEG_A:
            break;
        case Emulator::NEG_B:
            break;
        case Emulator::NEG_data8_X:
            break;
        case Emulator::NEG_addr16:
            break;
        case Emulator::NOP:
            strcpy(info.label, "NOP");
            info.operationAddres = address;
            info.type = 4;
            address += 1;
            break;
        case Emulator::ORA_A_data8:
            break;
        case Emulator::ORA_A_addr8:
            break;
        case Emulator::ORA_A_data8_X:
            break;
        case Emulator::ORA_A_addr16:
            break;
        case Emulator::ORA_B_data8:
            break;
        case Emulator::ORA_B_addr8:
            break;
        case Emulator::ORA_B_data8_X:
            break;
        case Emulator::ORA_B_addr16:
            break;
        case Emulator::PSH_A:
            break;
        case Emulator::PSH_B:
            break;
        case Emulator::PUL_A:
            break;
        case Emulator::PUL_B:
            break;
        case Emulator::ROL_A:
            break;
        case Emulator::ROL_B:
            break;
        case Emulator::ROL_data8_X:
            break;
        case Emulator::ROL_addr16:
            break;
        case Emulator::ROR_A:
            break;
        case Emulator::ROR_B:
            break;
        case Emulator::ROR_data8_X:
            break;
        case Emulator::ROR_addr16:
            break;
        case Emulator::RTI:
            break;
        case Emulator::RTS:
            break;
        case Emulator::SBA:
            break;
        case Emulator::SBC_A_data8:
            break;
        case Emulator::SBC_A_addr8:
            break;
        case Emulator::SBC_A_data8_X:
            break;
        case Emulator::SBC_A_addr16:
            break;
        case Emulator::SBC_B_data8:
            break;
        case Emulator::SBC_B_addr8:
            break;
        case Emulator::SBC_B_data8_X:
            break;
        case Emulator::SBC_B_addr16:
            break;
        case Emulator::SEC:
            break;
        case Emulator::SEI:
            break;
        case Emulator::SEV:
            break;
        case Emulator::STA_A_addr8:
            break;
        case Emulator::STA_A_data8_X:
            break;
        case Emulator::STA_A_addr16:
            break;
        case Emulator::STA_B_addr8:
            break;
        case Emulator::STA_B_data8_X:
            break;
        case Emulator::STA_B_addr16:
            break;
        case Emulator::STS_addr8:
            break;
        case Emulator::STS_data8_X:
            break;
        case Emulator::STS_addr16:
            break;
        case Emulator::STX_addr8:
            break;
        case Emulator::STX_data8_X:
            break;
        case Emulator::STX_addr16:
            break;
        case Emulator::SUB_A_data8:
            break;
        case Emulator::SUB_A_addr8:
            break;
        case Emulator::SUB_A_data8_X:
            break;
        case Emulator::SUB_A_addr16:
            break;
        case Emulator::SUB_B_data8:
            break;
        case Emulator::SUB_B_addr8:
            break;
        case Emulator::SUB_B_data8_X:
            break;
        case Emulator::SUB_B_addr16:
            break;
        case Emulator::SWI:
            break;
        case Emulator::TAB:
            break;
        case Emulator::TAP:
            break;
        case Emulator::TBA:
            break;
        case Emulator::TPA:
            break;
        case Emulator::TST_A:
            break;
        case Emulator::TST_B:
            break;
        case Emulator::TST_data8_X:
            break;
        case Emulator::TST_addr16:
            break;
        case Emulator::TSX:
            break;
        case Emulator::TXS:
            break;
        case Emulator::WAI:
            break;
        default:
            strcpy(info.label, "ERROR");
            ++it;
            break;
        }
        result.emplace_back(info);
    }
    return result;
}

int main(int , char *[])
{
    bool showDemo = true;
    HelloImGui::RunnerParams runnerParams;
    runnerParams.appWindowParams.windowTitle = "M6800 - emulator gui demo";
    runnerParams.appWindowParams.windowGeometry.size = { 800, 600};
    runnerParams.callbacks.LoadAdditionalFonts = MyLoadFonts;
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
                mpu.init(hexRecords[0].data, hexRecords[0].address);
                mpu.setAccA(0x00);
                mpu.setAccB(0x01);
            }

                auto buff = mpu.getMemory().getData();
                bool dirty = true;
                std::vector<DissasemblyInfo> dissasemblyInfo = std::vector<DissasemblyInfo>();
            
                for (auto it = hexRecords.begin(); it != hexRecords.end(); ++it) {
                    if (it->type == 0x01) {
                        continue;
                    }
                    else {
                        dissasemblyInfo = disassemble(*it);
                    }
                }

                auto dissasemblyOutputBuffer = std::vector<std::string>();
                for (int i = 0; i < 1024; i++) {
                    char label[20];
                            sprintf(label, "%04X :: %s ", i, "ERROR");
                            dissasemblyOutputBuffer.emplace_back(std::string(label));
                }

                memEditor.OptShowDataPreview = true;
                memEditor.OptShowOptions = true;
                memEditor.OptUpperCaseHex = true;
                memEditor.Cols = 8;

    runnerParams.callbacks.ShowGui =  [&mpu, &editor, codeBuf, &memEditor,  buff, dirty, &dissasemblyInfo, &dissasemblyOutputBuffer] () {
        {
            ImGui::BeginChild("left pane", ImVec2(ImGui::GetWindowWidth() * 0.30, -ImGui::GetFrameHeightWithSpacing()), true);
            ImGui::PushFont(gCustomFont);
            if (ImGui::BeginTabBar("##Tabs", ImGuiTabBarFlags_None))
            {
                if (ImGui::BeginTabItem("Disassembly"))
                {
                    char descTxt[128];
                    sprintf(descTxt, "Address:: Label  Operands");
                    ImGui::Text(descTxt);
                    for (int i = 0; i < dissasemblyInfo.size(); i++) {
                        char str[20];
                        sprintf(str, "%04X :: %s %04X", 
                            dissasemblyInfo[i].operationAddres, dissasemblyInfo[i].label, dissasemblyInfo[i].type != 0 ? dissasemblyInfo[i].data8 : dissasemblyInfo[i].type);
                        dissasemblyOutputBuffer[dissasemblyInfo[i].operationAddres] = std::string(str);
                    }
                    for (int i = 0; i < 1024; i++) {
                        ImGui::Text(dissasemblyOutputBuffer[i].c_str());
                    }
                    ImGui::EndTabItem();
                }

                if (ImGui::BeginTabItem("Source"))
                {
                    ImGui::TextWrapped("Here is going to be shown disassembled code if it will be possible to achieve");
                    editor.Render("Disassembly");
                    ImGui::EndTabItem();
                }
                ImGui::EndTabBar();
            }
            ImGui::PopFont();
            ImGui::EndChild();
        }
        ImGui::SameLine();

        // Right
        {
            char accAVal[9];
            char accBVal[9];
            ImGui::BeginGroup();
            ImGui::BeginChild("item view", ImVec2(0, ImGui::GetWindowHeight() * 0.40)); // Leave room for 1 line below us
            ImGui::PushFont(gCustomFont);
            ImGui::Text("Condition Register:");
            ImGui::Text("H");
            ImGui::SameLine();
            ImGui::Text("V");
            ImGui::SameLine();
            ImGui::Text("Z");
            ImGui::SameLine();
            ImGui::Text("N");
            ImGui::SameLine();
            ImGui::Text("I");
            ImGui::Text("%c",(char)mpu.getCondCodeReg()->H + '0');
            ImGui::SameLine();
            ImGui::Text("%c", (char)mpu.getCondCodeReg()->V + '0');
            ImGui::SameLine();
            ImGui::Text("%c", (char)mpu.getCondCodeReg()->Z + '0');
            ImGui::SameLine();
            ImGui::Text("%c", (char)mpu.getCondCodeReg()->N + '0');
            ImGui::SameLine();
            ImGui::Text("%c", (char)mpu.getCondCodeReg()->I + '0');
            ImGui::Text("C");
            ImGui::SameLine();
            ImGui::Text("Accumulator A:");
            ImGui::Text("%c", (char)mpu.getCondCodeReg()->C + '0');
            ImGui::SameLine();
            itoab(mpu.getAccA(), accAVal);
            ImGui::Text("%s",accAVal);
            ImGui::SameLine();
            ImGui::Text(": %02X", mpu.getAccA());
            ImGui::Text(" ");
            ImGui::SameLine();
            ImGui::Text("Accumulator B:");
            ImGui::Text(" ");
            ImGui::SameLine();
            itoab(mpu.getAccB(), accBVal);
            ImGui::Text("%s", accBVal);
            ImGui::SameLine();
            ImGui::Text(": %02X", mpu.getAccB());
            ImGui::Text("Program Counter:");
            ImGui::SameLine();
            ImGui::Text(": %04X", mpu.getRegPC());
            ImGui::Text("Stackpointer Register:");
            ImGui::SameLine();
            ImGui::Text(": %04X", mpu.getRegSP());
            ImGui::Text("Indexed Register:");
            ImGui::SameLine();
            ImGui::Text(": %04X", mpu.getRegX());
            ImGui::PopFont();
            ImGui::EndChild();
            ImGui::Separator();
            ImGui::BeginChild("Memory", ImVec2(0, -ImGui::GetFrameHeightWithSpacing())); // Leave room for 1 line below us
            ImGui::PushFont(gCustomFont);
            memEditor.DrawContents((void*)buff, 0xFFFF, sizeof(char));
            ImGui::PopFont();
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

