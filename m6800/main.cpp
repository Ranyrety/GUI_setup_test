#include <iostream>
#include "mpu.h"
#include "utilities.h"
using namespace std;
int main(int argc, char* argv[]){
    bool dataLoaded = false;
    if (argc != 2) {
        cout << "Usage: " << argv[0] << " <hex_file_path>" << endl;
        return 1;
    }

    string hexFilePath = argv[1];
    vector<HexRecord> hexRecords = parseHex(hexFilePath.c_str());

    if (!hexRecords.empty())

        dataLoaded = true;

    if (dataLoaded) {
        std::cout << "Hellow m6800 emulator is greeting you" << std::endl;
        Emulator::Mpu mpu;
        mpu.init(hexRecords[0].data,hexRecords[0].address);
        mpu.setAccA(0x00);
        mpu.setAccB(0x01);
        //act
        int cycles = mpu.execute();
    }
    return 0;
}