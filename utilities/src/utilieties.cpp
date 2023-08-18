#include "utilities.h"
#include <string>
std::vector<HexRecord> parseHex(const char* path){
    std::ifstream file(path);
    std::string line;
    std::vector<HexRecord> records;
    while(std::getline(file,line)){
        if(line[0] == ':'){
            HexRecord record;
            record.length = std::stoi(line.substr(1, 2), 0, 16);
            record.address = std::stoi(line.substr(3, 4), 0, 16);
            record.type = std::stoi(line.substr(7, 2), 0, 16);
            for (int i = 0; i < record.length; i++) {
                record.data.push_back(std::stoi(line.substr(9 + i * 2, 2), 0, 16));
            }
            records.push_back(record);
        }
    }
    return records;
}