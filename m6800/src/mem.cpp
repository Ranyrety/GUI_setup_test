#include "mem.h"

namespace Emulator
{
    void Memory::load(std::vector<uint8_t>& data)
    {
        for (int i = 0; i < data.size(); i++)
        {
            this->data[i] = data[i];
        }
    }

    void Memory::load(std::vector<uint8_t>& data, uint16_t address)
    {
        for (int i = 0; i < data.size(); i++)
        {
            this->data[address+i] = data[i];
        }
    }

    void Memory::reset()
    {
        for (int i = 0; i < 0xFFFF; i++)
        {
            this->data[i] = 0;
        }
    }

    uint8_t Memory::fetchByte(uint16_t addr)
    {
        return this->data[addr];
    }

    uint16_t Memory::fetchWord(uint16_t addr) {
        uint16_t word = 0;
        uint8_t msb = this->data[addr]; 
        uint8_t lsb = this->data[addr + 1];
        word = msb << 8;
        word = word | lsb;
        return word;
    }

    void Memory::writeByte(uint16_t addr, uint8_t data) {
        this->data[addr] = data;
    }

    void Memory::writeWord(uint16_t addr, uint16_t data) {
        uint8_t bytes[2];
        bytes[0] = (data >> 0) & 0xFF;
        bytes[1] = (data >> 8) & 0xFF;
        this->data[addr] = bytes[1];
        this->data[addr + 1] = bytes[0];
    }

} // namespace Emulator
