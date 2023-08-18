// function that will open hex file, parse it and load into memory
#ifndef UTILITIES_H
#define UTILITIES_H

#include <limits.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string.h>

struct HexRecord{
    uint32_t length;
    uint16_t address;
    uint8_t type;
    std::vector<uint8_t> data;
};

std::vector<HexRecord> parseHex(const char* path);

/*
std::vector<hexRecord> parseHex(const char* path){
    ifstream file(path);
    string line;
    vector<hexRecord> records;
    while(getline(file,line)){
        if(line[0] == ":"){
            hexRecord record;
            record.length = stoi(line.substr(1, 2), 0, 16);
            record.address = stoi(line.substr(3, 4), 0, 16);
            record.type = stoi(line.substr(7, 2), 0, 16);
            for (int i = 0; i < record.length; i++) {
                record.data.push_back(stoi(line.substr(9 + i * 2, 2), 0, 16));
            }
            records.push_back(record);
        }
    }
    return records;
}
*/
#endif // UTILITIES.H