#include "utilities.h"
#include <string>
std::vector<HexRecord> parseHex(const char* path){
    std::vector<HexRecord> records;

    std::string line;
    #ifndef __EMSCRIPTEN__
    std::ifstream file(path);

''    while(std::getline(file,line)){
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
    #else
    printf("DEBUG: %s\n", "About to open file");
    FILE* file = fopen("/hexFiles/test01.hex", "r");
    printf("DEBUG: File path=%s\n", path);
    printf("DEBUG: FD=%p\n", file);

    char cLine[512];
    while (fgets((char*)cLine, sizeof(cLine), file)) {
        printf("DEBUG: Parsing content=%s\n", cLine);
        /* note that fgets don't strip the terminating \n, checking its
           presence would allow to handle lines longer that sizeof(line) */
           line = std::string((const char*)cLine);
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
    #endif // __EMSCRIPTEN__
    return records;
}