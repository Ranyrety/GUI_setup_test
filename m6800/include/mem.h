//Copyright Michal Ruminski  
// Definitly need to chose licence
#ifndef MEM_H
#define MEM_H
#include <stdint.h>
#include <vector>
namespace Emulator{

/**
 * @brief this class will handle how we operate on memory 
 * it should be possible to configure at least in two ways
 *  1. All 65k is RAM
 *   
 */
struct Memory{
    public:
    void load(std::vector<uint8_t>& data);
    void load(std::vector<uint8_t>& data, uint16_t address);
    void reset();
    uint8_t fetchByte(uint16_t addr);
    uint16_t fetchWord(uint16_t addr);

    void writeByte(uint16_t addr, uint8_t data);
    void writeWord(uint16_t addr, uint16_t data);
    uint8_t* getData() { return data; };
    private:
    uint8_t data[0xFFFF];
    
};

}

#endif  // MEM_H