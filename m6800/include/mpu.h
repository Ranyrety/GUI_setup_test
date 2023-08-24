// Copyright Michal Ruminski
#ifndef MPU_H
#define MPU_H
#include <stdint.h>
#include <climits>
#ifdef linux
#include <inttypes.h>
#endif // linux

#include <vector>
#include "mem.h"
// Emulator thats whole system designed split in 3 parts
// SR - SystemRepresentation - representation of microprocessor system MPU, DataBus,
//       ROM, RAM and PIA/ACIA other signals if needed like VMA and clocks input
// EC - emulator control responsible for creating desired system configuration
//       (ie type of memory, I/O etc), exposing data to user, handling accessing compiled data etc
// GUI - this part is still biggest unknown , it should just be place to present data to user
//       and allow him to manipulate and perform needed tasks so disassembly should be in EC
//
namespace Emulator {
    //(TODO: Michal) Decide for documentation generation tools  from comments
    /**
     * @brief this is example of doxygen comment but it can be to much to configure and maintain
     * and we can use "///" comment in visual studio this may be easier to generate documentation
     *
     */

    /*** Adressing mode***/
    enum AddresMode {
        immidiate,
        direct,
        inherent,
        indexed,
        extended,
        accuA,
        accuB,
        relative
    };

    struct CondCodeReg
    {
        uint8_t C : 1; // carry flag
        uint8_t V : 1; // overflow  arithmetic 2's component overflow from bit6 to bit7
        uint8_t Z : 1; // zero flag, set if result = 0
        uint8_t N : 1; // negative, set if bit7 of result is set
        uint8_t I : 1; // interrupt mask, set by SEI clear by CLI
        uint8_t H : 1; // carry from b3 to b4, set by ADD, ABA and ADC cleared if no carry. not affected by other operations
        uint8_t AUXC : 1; // auxiliary carry according to 8bit-era page, probably need to look this up
        uint8_t EMPTY : 1;
        void setCFlag() {  C = 1; }
        void resetCFlag() { C = 0; }
        void setVFlag() {  V = 1; }
        void resetVFlag() { V = 0; }
        void setZFlag() {  Z = 1; }
        void resetZFlag() { Z = 0; }
        void setNFlag() {  N = 1; }
        void resetNFlag() { N = 0; }
        void setIFlag() { I = 1; }
        void resetIFlag() { I = 0; }
        void setHFlag() {  H = 1; }
        void resetHFlag() { H = 0; }
    };

    class Mpu
    {
    public:
        // method that will execute procedure pointing by mPCReg and return nr of cycles it took to finish
        //  number of cycles is taken from M6800 Microprocessor Applications Manual
        // algo goes like this, read data pointing by program counter register, check if opcode valid
        //  if valid  if  need to get operand inc mPCReg, get operands, go to proper procedure
        uint8_t execute();
        void init(std::vector<uint8_t>& data);
        void init(std::vector<uint8_t>& data, uint16_t address);
        void reset();
        uint8_t getAccA(){return mAccA;};
        uint8_t getAccB(){return mAccB;};
        uint16_t getRegX(){return mXReg;};
        uint16_t getRegPC(){return mPCReg;};
        uint16_t getRegSP(){return mSPReg;};
        void setAccA(uint8_t data){mAccA = data;};
        void setAccB(uint8_t data){mAccB = data;};
        void setRegX(uint16_t data){mXReg = data;};
        void setRegPC(uint16_t data){mPCReg = data;};
        void setPC(uint16_t addr);
        CondCodeReg *getCondCodeReg(){return &mCondCodeReg;};
        void writeData(uint16_t addr, uint8_t data);
        Memory& getMemory() { return mMem; }
    private:
        CondCodeReg mCondCodeReg;
        uint8_t mAccA;
        uint8_t mAccB;
        uint16_t mXReg;
        uint16_t mPCReg;
        uint16_t mSPReg;
        Memory mMem;
        uint8_t readData(uint16_t addr);
        int decode(uint8_t opCode);
        /*Depending on mode adds to accumulator acc either data directly or data pointed by address
        address and data is taken from memory don't increment pc after dispatching 
        return number of cycles
        */
        int aba();
        int adc(AddresMode mode, uint8_t &dest);
        int add(AddresMode mode, uint8_t &dest);
        int and_(AddresMode mode, uint8_t &dest);
        int asl(AddresMode mode);
        int asr(AddresMode mode);
        int bcc();
        int bcs();
        int beq();
        int bge();
        int bgt();
        int bhi();
        int bit(AddresMode mode, uint8_t &dest);
        int ble();
        int bls();
        int blt();
        int bmi();
        int bne();
        int bpl();
        int bra();
        int bsr();
        int bvc();
        int bvs();
        int cba();
        int clc();
        int cli();
        int clr(AddresMode mode);
        int clv();
        int cmp(AddresMode mode, uint8_t &dest);
        int com(AddresMode mode);
        int cpx(AddresMode mode);
        int daa();
        int dec(AddresMode mode);
        int des();
        int dex();
        int eor(AddresMode mode, uint8_t &dest);
        int inc(AddresMode mode);
        int ins();
        int inx();
        int jmp(AddresMode mode);
        int jsr(AddresMode mode);
        int lda(AddresMode mode, uint8_t& dest);
        int lds(AddresMode mode);
        int ldx(AddresMode mode);
        int lsr(AddresMode mode);
        int neg(AddresMode mode);
        int nop();
        int ora(AddresMode mode, uint8_t& dest);
        int psh(AddresMode mode);
        int pul(AddresMode mode);
        int rol(AddresMode mode);
        int ror(AddresMode mode);
        int rti();
        int rts();
        int sba();
        int sbc(AddresMode mode, uint8_t& dest);
        int sec();
        int sei();
        int sev();
        int sta(AddresMode mode, uint8_t& source);
        int sts(AddresMode mode);
        int stx(AddresMode mode);
        int sub(AddresMode mode, uint8_t &dest);
        int swi();
        int tab();
        int tap();
        int tba();
        int tpa();
        int tst(AddresMode mode);
        int tsx();
        int txs();
        int wai();
    };

    static constexpr uint8_t
        ABA = 0x1B,
        ADC_A_data8 = 0x89,
        ADC_A_addr8 = 0x99,
        ADC_A_data8_X = 0xA9,
        ADC_A_addr16 = 0xB9,
        ADC_B_data8 = 0xC9,
        ADC_B_addr8 = 0xD9,
        ADC_B_data8_X = 0xE9,
        ADC_B_addr16 = 0xF9,
        ADD_A_data8 = 0x8B,
        ADD_A_addr8 = 0x9B,
        ADD_A_data8_X = 0xAB,
        ADD_A_addr16 = 0xBB,
        ADD_B_data8 = 0xCB,
        ADD_B_addr8 = 0xDB,
        ADD_B_data8_X = 0xEB,
        ADD_B_addr16 = 0xFB,
        AND_A_data8 = 0x84,
        AND_A_addr8 = 0x94,
        AND_A_data8_X = 0xA4,
        AND_A_addr16 = 0xB4,
        AND_B_data8 = 0xC4,
        AND_B_addr8 = 0xD4,
        AND_B_data8_X = 0xE4,
        AND_B_addr16 = 0xF4,
        ASL_A = 0x48,
        ASL_B = 0x58,
        ASL_data8_X = 0x68,
        ASL_addr16 = 0x78,
        ASR_A = 0x47,
        ASR_B = 0x57,
        ASR_data8_X = 0x67,
        ASR_addr16 = 0x77,
        BCC_rel8 = 0x24,
        BCS_rel8 = 0x25,
        BEQ_rel8 = 0x27,
        BGE_rel8 = 0x2C,
        BGT_rel8 = 0x2E,
        BHI_rel8 = 0x22,
        BIT_A_data8 = 0x85,
        BIT_A_addr8 = 0x95,
        BIT_A_data8_X = 0xA5,
        BIT_A_addr16 = 0xB5,
        BIT_B_data8 = 0xC5,
        BIT_B_addr8 = 0xD5,
        BIT_B_data8_X = 0xE5,
        BIT_B_addr16 = 0xF5,
        BLE_rel8 = 0x2F,
        BLS_rel8 = 0x23,
        BLT_rel8 = 0x2D,
        BMI_rel8 = 0x2B,
        BNE_rel8 = 0x26,
        BPL_rel8 = 0x2A,
        BRA_rel8 = 0x20,
        BSR_rel8 = 0x8D,
        BVC_rel8 = 0x28,
        BVS_rel8 = 0x29,
        CBA = 0x11,
        CLC = 0x0C,
        CLI = 0x0E,
        CLR_A = 0x4F,
        CLR_B = 0x5F,
        CLR_data8_X = 0x6F,
        CLR_addr16 = 0x7F,
        CLV = 0x0A,
        CMP_A_data8 = 0x81,
        CMP_A_addr8 = 0x91,
        CMP_A_data8_X = 0xA1,
        CMP_A_addr16 = 0xB1,
        CMP_B_data8 = 0xC1,
        CMP_B_addr8 = 0xD1,
        CMP_B_data8_X = 0xE1,
        CMP_B_addr16 = 0xF1,
        COM_A = 0x43,
        COM_B = 0x53,
        COM_data8_X = 0x63,
        COM_addr16 = 0x73,
        CPX_addr8 = 0x9C,
        CPX_data8_X = 0xAC,
        CPX_data16 = 0x8C,
        CPX_addr16 = 0xBC,
        DAA = 0x19,
        DEC_A = 0x4A,
        DEC_B = 0x5A,
        DEC_data8_X = 0x6A,
        DEC_addr16 = 0x7A,
        DES = 0x34,
        DEX = 0x09,
        EOR_A_data8 = 0x88,
        EOR_A_addr8 = 0x98,
        EOR_A_data8_X = 0xA8,
        EOR_A_addr16 = 0xB8,
        EOR_B_data8 = 0xC8,
        EOR_B_addr8 = 0xD8,
        EOR_B_data8_X = 0xE8,
        EOR_B_addr16 = 0xF8,
        INC_A = 0x4C,
        INC_B = 0x5C,
        INC_data8_X = 0x6C,
        INC_addr16 = 0x7C,
        INS = 0x31,
        INX = 0x08,
        JMP_data8_X = 0x6E,
        JMP_addr16 = 0x7E,
        JSR_data8_X = 0xAD,
        JSR_addr16 = 0xBD,
        LDA_A_data8 = 0x86,
        LDA_A_addr8 = 0x96,
        LDA_A_data8_X = 0xA6,
        LDA_A_addr16 = 0xB6,
        LDA_B_data8 = 0xC6,
        LDA_B_addr8 = 0xD6,
        LDA_B_data8_X = 0xE6,
        LDA_B_addr16 = 0xF6,
        LDS_addr8 = 0x9E,
        LDS_data8_X = 0xAE,
        LDS_data16 = 0x8E,
        LDS_addr16 = 0xBE,
        LDX_addr8 = 0xDE,
        LDX_data8_X = 0xEE,
        LDX_data16 = 0xCE,
        LDX_addr16 = 0xFE,
        LSR_A = 0x44,
        LSR_B = 0x54,
        LSR_data8_X = 0x64,
        LSR_addr16 = 0x74,
        NEG_A = 0x40,
        NEG_B = 0x50,
        NEG_data8_X = 0x60,
        NEG_addr16 = 0x70,
        NOP = 0x01,
        ORA_A_data8 = 0x8A,
        ORA_A_addr8 = 0x9A,
        ORA_A_data8_X = 0xAA,
        ORA_A_addr16 = 0xBA,
        ORA_B_data8 = 0xCA,
        ORA_B_addr8 = 0xDA,
        ORA_B_data8_X = 0xEA,
        ORA_B_addr16 = 0xFA,
        PSH_A = 0x36,
        PSH_B = 0x37,
        PUL_A = 0x32,
        PUL_B = 0x33,
        ROL_A = 0x49,
        ROL_B = 0x59,
        ROL_data8_X = 0x69,
        ROL_addr16 = 0x79,
        ROR_A = 0x46,
        ROR_B = 0x56,
        ROR_data8_X = 0x66,
        ROR_addr16 = 0x76,
        RTI = 0x3B,
        RTS = 0x39,
        SBA = 0x10,
        SBC_A_data8 = 0x82,
        SBC_A_addr8 = 0x92,
        SBC_A_data8_X = 0xA2,
        SBC_A_addr16 = 0xB2,
        SBC_B_data8 = 0xC2,
        SBC_B_addr8 = 0xD2,
        SBC_B_data8_X = 0xE2,
        SBC_B_addr16 = 0xF2,
        SEC = 0x0D,
        SEI = 0x0F,
        SEV = 0x0B,
        STA_A_addr8 = 0x97,
        STA_A_data8_X = 0xA7,
        STA_A_addr16 = 0xB7,
        STA_B_addr8 = 0xD7,
        STA_B_data8_X = 0xE7,
        STA_B_addr16 = 0xF7,
        STS_addr8 = 0x9F,
        STS_data8_X = 0xAF,
        STS_addr16 = 0xBF,
        STX_addr8 = 0xDF,
        STX_data8_X = 0xEF,
        STX_addr16 = 0xFF,
        SUB_A_data8 = 0x80,
        SUB_A_addr8 = 0x90,
        SUB_A_data8_X = 0xA0,
        SUB_A_addr16 = 0xB0,
        SUB_B_data8 = 0xC0,
        SUB_B_addr8 = 0xD0,
        SUB_B_data8_X = 0xE0,
        SUB_B_addr16 = 0xF0,
        SWI = 0x3F,
        TAB = 0x16,
        TAP = 0x06,
        TBA = 0x17,
        TPA = 0x07,
        TST_A = 0x4D,
        TST_B = 0x5D,
        TST_data8_X = 0x6D,
        TST_addr16 = 0x7D,
        TSX = 0x30,
        TXS = 0x35,
        WAI = 0x3E;

} //namespace Emulator
#endif // MPU.H