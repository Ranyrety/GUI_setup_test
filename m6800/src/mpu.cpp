#include "mpu.h"

namespace Emulator{

    void Mpu::reset(){
        mCondCodeReg.C = 0;
        mCondCodeReg.V = 0;
        mCondCodeReg.Z = 0;
        mCondCodeReg.N = 0;
        mCondCodeReg.I = 0;
        mCondCodeReg.H = 0;
        mAccA = 0;
        mAccB = 0;
        mXReg = 0;
        mPCReg = 0;
        mSPReg = 0;
        mMem.reset();
    }

    void Mpu::init(std::vector<uint8_t>& data){
        reset();
        mMem.load(data);
    }

    void Mpu::init(std::vector<uint8_t>& data, uint16_t address) {
        reset();
        mMem.load(data, address);
        mPCReg = address;
    }


    void Mpu::setPC(uint16_t addr){
        mPCReg = addr;
    }

    uint8_t Mpu::execute(){
            auto opcode = mMem.fetchByte(mPCReg);
            switch (opcode) {
            case Emulator::ABA:
                return aba();
            case Emulator::ADC_A_data8:
                return adc(Emulator::AddresMode::immidiate, mAccA);
            case Emulator::ADC_A_addr8:
                return adc(Emulator::AddresMode::direct, mAccA);
            case Emulator::ADC_A_addr16:
                return adc(Emulator::AddresMode::extended, mAccA);
            case Emulator::ADC_A_data8_X:
                return adc(Emulator::AddresMode::indexed, mAccA);
            case Emulator::ADC_B_data8:
                return adc(Emulator::AddresMode::immidiate, mAccB);
            case Emulator::ADC_B_addr8:
                return adc(Emulator::AddresMode::direct, mAccB);
            case Emulator::ADC_B_addr16:
                return adc(Emulator::AddresMode::extended, mAccB);
            case Emulator::ADC_B_data8_X:
                return adc(Emulator::AddresMode::indexed, mAccB);
            case Emulator::ADD_A_data8:
                return add(Emulator::AddresMode::immidiate, mAccA);
            case Emulator::ADD_A_addr8:
                return add(Emulator::AddresMode::direct, mAccA);
            case Emulator::ADD_A_addr16:
                return add(Emulator::AddresMode::extended, mAccA);
            case Emulator::ADD_A_data8_X:
                return add(Emulator::AddresMode::indexed, mAccA);
            case Emulator::ADD_B_data8:
                return add(Emulator::AddresMode::immidiate, mAccB);
            case Emulator::ADD_B_addr8:
                return add(Emulator::AddresMode::direct, mAccB);
            case Emulator::ADD_B_addr16:
                return add(Emulator::AddresMode::extended, mAccB);
            case Emulator::ADD_B_data8_X:
                return add(Emulator::AddresMode::indexed, mAccB);
            case Emulator::AND_A_data8:
                return and_(Emulator::AddresMode::direct, mAccA);
            case Emulator::AND_A_addr8:
                return and_(Emulator::AddresMode::immidiate, mAccA);
            case Emulator::AND_A_addr16:
                return and_(Emulator::AddresMode::extended, mAccA);
            case Emulator::AND_A_data8_X:
                return and_(Emulator::AddresMode::indexed, mAccA);
            case Emulator::AND_B_data8:
                return and_(Emulator::AddresMode::direct, mAccB);
            case Emulator::AND_B_addr8:
                return and_(Emulator::AddresMode::immidiate, mAccB);
            case Emulator::AND_B_addr16:
                return and_(Emulator::AddresMode::extended, mAccB);
            case Emulator::AND_B_data8_X:
                return and_(Emulator::AddresMode::indexed, mAccB);
            case Emulator::ASL_A:
                return asl(Emulator::AddresMode::accuA);
            case Emulator::ASL_B:
                return asl(Emulator::AddresMode::accuB);
            case Emulator::ASL_addr16:
                return asl(Emulator::AddresMode::extended);
            case Emulator::ASL_data8_X:
                return asl(Emulator::AddresMode::indexed);
            case Emulator::ASR_A:
                return asl(Emulator::AddresMode::accuA);
            case Emulator::ASR_B:
                return asl(Emulator::AddresMode::accuB);
            case Emulator::ASR_addr16:
                return asl(Emulator::AddresMode::extended);
            case Emulator::ASR_data8_X:
                return asl(Emulator::AddresMode::indexed);
            case Emulator::BCC_rel8:
                return bcc();
            case Emulator::BCS_rel8:
                return bcs();
            case Emulator::BEQ_rel8:
                return beq();
            case Emulator::BGE_rel8:
                return bge();
            case Emulator::BGT_rel8:
                return bgt();
            case Emulator::BHI_rel8:
                return bhi();
            case Emulator::BIT_A_data8:
                return bit(Emulator::AddresMode::immidiate, mAccA);
            case Emulator::BIT_A_addr8:
                return bit(Emulator::AddresMode::direct, mAccA);
            case Emulator::BIT_A_addr16:
                return bit(Emulator::AddresMode::extended, mAccA);
            case Emulator::BIT_A_data8_X:
                return bit(Emulator::AddresMode::indexed, mAccA);
            case Emulator::BIT_B_data8:
                return bit(Emulator::AddresMode::immidiate, mAccB);
            case Emulator::BIT_B_addr8:
                return bit(Emulator::AddresMode::direct, mAccB);
            case Emulator::BIT_B_addr16:
                return bit(Emulator::AddresMode::extended, mAccB);
            case Emulator::BIT_B_data8_X:
                return bit(Emulator::AddresMode::indexed, mAccB);
            case Emulator::BLE_rel8:
                return ble();
            case Emulator::BLS_rel8:
                return bls();
            case Emulator::BLT_rel8:
                return blt();
            case Emulator::BMI_rel8:
                return bmi();
            case Emulator::BNE_rel8:
                return bne();
            case Emulator::BPL_rel8:
                return bpl();
            case Emulator::BRA_rel8:
                return bra();
            case Emulator::BSR_rel8:
                return bsr();
            case Emulator::BVC_rel8:
                return bvc();
            case Emulator::BVS_rel8:
                return bvs();
            case Emulator::CBA:
                return cba();
            case Emulator::CLC:
                return clc();
            case Emulator::CLI:
                return cli();
            case Emulator::CLR_A:
                return clr(Emulator::AddresMode::accuA);
            case Emulator::CLR_B:
                return clr(Emulator::AddresMode::accuB);
            case Emulator::CLR_data8_X:
                return clr(Emulator::AddresMode::indexed);
            case Emulator::CLR_addr16:
                return clr(Emulator::AddresMode::extended);
            case Emulator::CLV:
                return clv();
            case Emulator::CMP_A_data8:
                return cmp(Emulator::AddresMode::immidiate, mAccA);
            case Emulator::CMP_A_addr8:
                return cmp(Emulator::AddresMode::direct, mAccA);
            case Emulator::CMP_A_data8_X:
                return cmp(Emulator::AddresMode::indexed, mAccA);
            case Emulator::CMP_A_addr16:
                return cmp(Emulator::AddresMode::extended, mAccA);
            case Emulator::CMP_B_data8:
                return cmp(Emulator::AddresMode::immidiate, mAccB);
            case Emulator::CMP_B_addr8:
                return cmp(Emulator::AddresMode::direct, mAccB);
            case Emulator::CMP_B_data8_X:
                return cmp(Emulator::AddresMode::indexed, mAccB);
            case Emulator::CMP_B_addr16:
                return cmp(Emulator::AddresMode::extended, mAccB);
            case Emulator::COM_A:
                return com(Emulator::AddresMode::accuA);
            case Emulator::COM_B:
                return com(Emulator::AddresMode::accuB);
            case Emulator::COM_data8_X:
                return com(Emulator::AddresMode::indexed);
            case Emulator::COM_addr16:
                return com(Emulator::AddresMode::extended);
            case Emulator::DAA:
                return daa();
            case Emulator::DEC_A:
                return dec(Emulator::AddresMode::accuA);
            case Emulator::DEC_B:
                return dec(Emulator::AddresMode::accuB);
            case Emulator::DEC_data8_X:
                return dec(Emulator::AddresMode::indexed);
            case Emulator::DEC_addr16:
                return dec(Emulator::AddresMode::extended);
            case Emulator::DES:
                return des();
            case Emulator::DEX:
                return dex();
            case Emulator::EOR_A_data8:
                return eor(Emulator::AddresMode::immidiate, mAccA);
            case Emulator::EOR_A_addr8:
                return eor(Emulator::AddresMode::direct, mAccA);
            case Emulator::EOR_A_data8_X:
                return eor(Emulator::AddresMode::indexed, mAccA);
            case Emulator::EOR_A_addr16:
                return eor(Emulator::AddresMode::extended, mAccA);
            case Emulator::EOR_B_data8:
                return eor(Emulator::AddresMode::immidiate, mAccB);
            case Emulator::EOR_B_addr8:
                return eor(Emulator::AddresMode::direct, mAccB);
            case Emulator::EOR_B_data8_X:
                return eor(Emulator::AddresMode::indexed, mAccB);
            case Emulator::EOR_B_addr16:
                return eor(Emulator::AddresMode::extended, mAccB);
            case Emulator::INC_A:
                return inc(Emulator::AddresMode::accuA);
            case Emulator::INC_B:
                return inc(Emulator::AddresMode::accuB);
            case Emulator::INC_data8_X:
                return inc(Emulator::AddresMode::indexed);
            case Emulator::INC_addr16:
                return inc(Emulator::AddresMode::extended);
            case Emulator::INS:
                return ins();
            case Emulator::INX:
                return inx();
            case Emulator::JMP_data8_X:
                return jmp(Emulator::AddresMode::indexed);
            case Emulator::JMP_addr16:
                return jmp(Emulator::AddresMode::extended);
            case Emulator::JSR_data8_X:
                return jsr(Emulator::AddresMode::indexed);
            case Emulator::JSR_addr16:
                return jsr(Emulator::AddresMode::extended);
            case Emulator::LDA_A_data8:
                return lda(Emulator::AddresMode::immidiate, mAccA);
            case Emulator::LDA_A_addr8:
                return lda(Emulator::AddresMode::direct, mAccA);
            case Emulator::LDA_A_data8_X:
                return lda(Emulator::AddresMode::indexed, mAccA);
            case Emulator::LDA_A_addr16:
                return lda(Emulator::AddresMode::extended, mAccA);
            case Emulator::LDA_B_data8:
                return lda(Emulator::AddresMode::immidiate, mAccB);
            case Emulator::LDA_B_addr8:
                return lda(Emulator::AddresMode::direct, mAccB);
            case Emulator::LDA_B_data8_X:
                return lda(Emulator::AddresMode::indexed, mAccB);
            case Emulator::LDA_B_addr16:
                return lda(Emulator::AddresMode::extended, mAccB);
            case Emulator::LDS_data16:
                return lds(Emulator::AddresMode::immidiate);
            case Emulator::LDS_addr8:
                return lds(Emulator::AddresMode::direct);
            case Emulator::LDS_data8_X:
                return lds(Emulator::AddresMode::indexed);
            case Emulator::LDS_addr16:
                return lds(Emulator::AddresMode::extended);
            case Emulator::LDX_data16:
                return ldx(Emulator::AddresMode::immidiate);
            case Emulator::LDX_addr8:
                return ldx(Emulator::AddresMode::direct);
            case Emulator::LDX_data8_X:
                return ldx(Emulator::AddresMode::indexed);
            case Emulator::LDX_addr16:
                return ldx(Emulator::AddresMode::extended);
            case Emulator::LSR_A:
                return lsr(Emulator::AddresMode::accuA);
            case Emulator::LSR_B:
                return lsr(Emulator::AddresMode::accuB);
            case Emulator::LSR_data8_X:
                return lsr(Emulator::AddresMode::indexed);
            case Emulator::LSR_addr16:
                return lsr(Emulator::AddresMode::extended);
            case Emulator::NEG_A:
                return neg(AddresMode::accuA);
            case Emulator::NEG_B:
                return neg(Emulator::AddresMode::accuB);
            case Emulator::NEG_data8_X:
                return neg(Emulator::AddresMode::indexed);
            case Emulator::NEG_addr16:
                return neg(Emulator::AddresMode::extended);
            case Emulator::NOP:
                return nop();
            case Emulator::ORA_A_data8:
                return ora(Emulator::AddresMode::immidiate, mAccA);
            case Emulator::ORA_A_addr8:
                return ora(Emulator::AddresMode::direct, mAccA);
            case Emulator::ORA_A_data8_X:
                return ora(Emulator::AddresMode::indexed, mAccA);
            case Emulator::ORA_A_addr16:
                return ora(Emulator::AddresMode::extended, mAccA);
            case Emulator::ORA_B_data8:
                return ora(Emulator::AddresMode::immidiate, mAccB);
            case Emulator::ORA_B_addr8:
                return ora(Emulator::AddresMode::direct, mAccB);
            case Emulator::ORA_B_data8_X:
                return ora(Emulator::AddresMode::indexed, mAccB);
            case Emulator::ORA_B_addr16:
                return ora(Emulator::AddresMode::extended, mAccB);
            case Emulator::PSH_A:
                return psh(Emulator::AddresMode::accuA);
            case Emulator::PSH_B:
                return psh(Emulator::AddresMode::accuB);
            case Emulator::PUL_A:
                return pul(Emulator::AddresMode::accuA);
            case Emulator::PUL_B:
                return pul(Emulator::AddresMode::accuB);
            case Emulator::ROL_A:
                return rol(Emulator::AddresMode::accuA);
            case Emulator::ROL_B:
                return rol(Emulator::AddresMode::accuB);
            case Emulator::ROL_data8_X:
                return rol(Emulator::AddresMode::indexed);
            case Emulator::ROL_addr16:
                return rol(Emulator::AddresMode::extended);
            case Emulator::ROR_A:
                return ror(Emulator::AddresMode::accuA);
            case Emulator::ROR_B:
                return ror(Emulator::AddresMode::accuB);
            case Emulator::ROR_data8_X:
                return ror(Emulator::AddresMode::indexed);
            case Emulator::ROR_addr16:
                return ror(Emulator::AddresMode::extended);
            case Emulator::RTI:
                return rti();
            case Emulator::RTS:
                return rts();
            case Emulator::SBA:
                return sba();
            case Emulator::SBC_A_data8:
                return sbc(Emulator::AddresMode::immidiate, mAccA);
            case Emulator::SBC_A_addr8:
                return sbc(Emulator::AddresMode::direct, mAccA);
            case Emulator::SBC_A_data8_X:
                return sbc(Emulator::AddresMode::indexed, mAccA);
            case Emulator::SBC_A_addr16:
                return sbc(Emulator::AddresMode::extended, mAccA);
            case Emulator::SBC_B_data8:
                return sbc(Emulator::AddresMode::immidiate, mAccB);
            case Emulator::SBC_B_addr8:
                return sbc(Emulator::AddresMode::direct, mAccB);
            case Emulator::SBC_B_data8_X:
                return sbc(Emulator::AddresMode::indexed, mAccB);
            case Emulator::SBC_B_addr16:
                return sbc(Emulator::AddresMode::extended, mAccB);
            case Emulator::SEC:
                return sec();
            case Emulator::SEI:
                return sei();
            case Emulator::SEV:
                return sev();
            case Emulator::STA_A_addr8:
                return sbc(Emulator::AddresMode::direct, mAccA);
            case Emulator::STA_A_data8_X:
                return sbc(Emulator::AddresMode::indexed, mAccA);
            case Emulator::STA_A_addr16:
                return sbc(Emulator::AddresMode::extended, mAccA);
            case Emulator::STA_B_addr8:
                return sbc(Emulator::AddresMode::direct, mAccB);
            case Emulator::STA_B_data8_X:
                return sbc(Emulator::AddresMode::indexed, mAccB);
            case Emulator::STA_B_addr16:
                return sbc(Emulator::AddresMode::extended, mAccB);
            case Emulator::STS_addr8:
                return sts(Emulator::AddresMode::direct);
            case Emulator::STS_data8_X:
                return sts(Emulator::AddresMode::indexed);
            case Emulator::STS_addr16:
                return sts(Emulator::AddresMode::extended);
            case Emulator::STX_addr8:
                return stx(Emulator::AddresMode::direct);
            case Emulator::STX_data8_X:
                return stx(Emulator::AddresMode::indexed);
            case Emulator::STX_addr16:
                return stx(Emulator::AddresMode::extended);
            case Emulator::SUB_A_data8:
                return sub(Emulator::AddresMode::immidiate, mAccA);
            case Emulator::SUB_A_addr8:
                return sub(Emulator::AddresMode::direct, mAccA);
            case Emulator::SUB_A_data8_X:
                return sub(Emulator::AddresMode::indexed, mAccA);
            case Emulator::SUB_A_addr16:
                return sub(Emulator::AddresMode::extended, mAccA);
            case Emulator::SUB_B_data8:
                return sub(Emulator::AddresMode::immidiate, mAccB);
            case Emulator::SUB_B_addr8:
                return sub(Emulator::AddresMode::direct, mAccB);
            case Emulator::SUB_B_data8_X:
                return sub(Emulator::AddresMode::indexed, mAccB);
            case Emulator::SUB_B_addr16:
                return sub(Emulator::AddresMode::extended, mAccB);
            case Emulator::SWI:
                return swi();
            case Emulator::TAB:
                return tab();
            case Emulator::TAP:
                return tap();
            case Emulator::TBA:
                return tba();
            case Emulator::TPA:
                return tpa();
            case Emulator::TST_A:
                return tst(Emulator::AddresMode::accuA);
            case Emulator::TST_B:
                return tst(Emulator::AddresMode::accuB);
            case Emulator::TST_data8_X:
                return tst(Emulator::AddresMode::indexed);
            case Emulator::TST_addr16:
                return tst(Emulator::AddresMode::extended);
            case Emulator::TSX:
                return tsx();
            case Emulator::TXS:
                return txs();
            case Emulator::WAI:
                return wai();
            default:
                return -1;
            }
    return 0;
    }

    void Mpu::writeData(uint16_t addr, uint8_t data)
    {
        mMem.writeByte(addr, data);
    }

    int Mpu::adc(AddresMode mode, uint8_t& dest)
    {
        int cycles = 0;
        uint8_t result = 0;
        uint8_t addr8 = 0;
        uint16_t addr16 = 0;
        uint8_t data = 0;
        uint8_t offset = 0;
        mPCReg++;
        switch (mode)
        {
        case Emulator::immidiate:
            data = mMem.fetchByte(mPCReg);
            result = dest + data + mCondCodeReg.C;
            //set flags
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            mCondCodeReg.V = (result ^ dest) ? 1 : 0;
            mCondCodeReg.C = dest + data + mCondCodeReg.C > 0xFF ? 1 : 0;
            cycles = 2;
            mPCReg++;
            break;
        case Emulator::direct:
            addr8 = mMem.fetchByte(mPCReg);
            result = dest + mMem.fetchByte(addr8) + mCondCodeReg.C;
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            mCondCodeReg.V = (result ^ dest) ? 1 : 0;
            mCondCodeReg.C = dest + mMem.fetchByte(addr8) + mCondCodeReg.C > 0xFF ? 1 : 0;
            cycles = 2;
            mPCReg++;
            break;
        case Emulator::indexed:
            offset = mMem.fetchByte(mPCReg);
            data = mMem.fetchByte(mXReg + offset);
            result = dest + data + mCondCodeReg.C;
            //set flags
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            mCondCodeReg.V = (result ^ dest) ? 1 : 0;
            mCondCodeReg.C = dest + data + mCondCodeReg.C > 0xFF ? 1 : 0;
            cycles = 2;
            mPCReg++;
            break;
        case Emulator::extended:
            addr16 = mMem.fetchWord(mPCReg);
            result = dest + mMem.fetchByte(addr16) + mCondCodeReg.C;
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            mCondCodeReg.V = (result ^ dest) ? 1 : 0;
            mCondCodeReg.C = dest + mMem.fetchByte(addr16) + mCondCodeReg.C > 0xFF ? 1 : 0;
            cycles = 3;
            mPCReg += 2;
            break;
        default:
            result =  0;
            break;
        }
        dest = result;
        return cycles;
    }

    int Mpu::add(AddresMode mode, uint8_t& dest)
    {
        int cycles = 0;
        uint8_t result = 0;
        uint8_t addr8 = 0;
        uint16_t addr16 = 0;
        uint8_t offset = 0;
        uint8_t data = 0;
        mPCReg++;
        switch (mode)
        {
        case Emulator::immidiate:
            data = mMem.fetchByte(mPCReg);
            result = dest + data;
            //set flags
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            mCondCodeReg.V = (result ^ dest) ? 1 : 0;
            mCondCodeReg.C = dest + data > 0xFF ? 1 : 0;
            cycles = 2;
            mPCReg++;
            break;
        case Emulator::direct:
            addr8 = mMem.fetchByte(mPCReg);
            result = dest + mMem.fetchByte(addr8);
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            mCondCodeReg.V = (result ^ dest) ? 1 : 0;
            mCondCodeReg.C = dest + mMem.fetchByte(addr8) > 0xFF ? 1 : 0;
            cycles = 2;
            mPCReg++;
            break;
        case Emulator::indexed:
            offset = mMem.fetchByte(mPCReg);
            data = mMem.fetchByte(mXReg + offset);
            result = dest + data;
            //set flags
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            mCondCodeReg.V = (result ^ dest) ? 1 : 0;
            mCondCodeReg.C = dest + data> 0xFF ? 1 : 0;
            cycles = 2;
            mPCReg++;
            break;
        case Emulator::extended:
            addr16 = mMem.fetchWord(mPCReg);
            result = dest + mMem.fetchByte(addr16);
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            mCondCodeReg.V = (result ^ dest) ? 1 : 0;
            mCondCodeReg.C = dest + mMem.fetchByte(addr16) > 0xFF ? 1 : 0;
            cycles = 3;
            mPCReg += 2;
            break;
        default:
            result = 0;
            break;
        }
        dest = result;
        return cycles;
    }
    /*** Adds content of accumulator B into accumulator A*/
    int Mpu::aba() {
        uint8_t result = mAccA + mAccB;
        //set flags
        mCondCodeReg.Z = result ? 0 : 1;
        mCondCodeReg.N = result & 0x80 ? 1 : 0;
        mCondCodeReg.V = (result ^ mAccA) & (result ^ mAccB) ? 1 : 0;
        mCondCodeReg.C = mAccA + mAccB > 0xFF ? 1 : 0;
        mAccA = result;
        return 2;
}
    
    int Mpu::and_(Emulator::AddresMode mode, uint8_t& dest) {
        int cycles = 0;
        uint8_t result = 0;
        uint8_t addr8 = 0;
        uint16_t addr16 = 0;
        uint8_t offset = 0;
        uint8_t data = 0;
        mPCReg++;
        switch (mode)
        {
        case Emulator::immidiate:
            data = mMem.fetchByte(mPCReg);
            result = dest & data;
            //set flags
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            mCondCodeReg.V = (result ^ dest) ? 1 : 0;
            mCondCodeReg.C = dest - data > 0xFF ? 1 : 0;
            cycles = 2;
            break;
        case Emulator::direct:
            addr8 = mMem.fetchByte(mPCReg);
            result = dest & mMem.fetchByte(addr8);
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            mCondCodeReg.V = (result ^ dest) ? 1 : 0;
            mCondCodeReg.C = dest - mMem.fetchByte(addr8) > 0xFF ? 1 : 0;
            cycles = 2;
            break;
        case Emulator::indexed:
            offset = mMem.fetchByte(mPCReg);
            data = mMem.fetchByte(mXReg + offset);
            result = dest & data;
            //set flags
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            mCondCodeReg.V = (result ^ dest) ? 1 : 0;
            mCondCodeReg.C = dest - data > 0xFF ? 1 : 0;
            break;
        case Emulator::extended:
            addr16 = mMem.fetchWord(mPCReg);
            result = dest & mMem.fetchByte(addr16);
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            mCondCodeReg.V = (result ^ dest) ? 1 : 0;
            mCondCodeReg.C = dest - mMem.fetchByte(addr16) > 0xFF ? 1 : 0;
            cycles = 3;
            break;
        default:
            break;
        }
        return cycles;
    }

    int Mpu::asl(Emulator::AddresMode mode) {
        mPCReg++;
        int offset = 0;
        int adres = 0;
        int data = 0;
        int prevData = 0;
        int result = 0;
        switch (mode)
        {
        case Emulator::indexed:
            adres = mXReg + mMem.fetchByte(mPCReg);
            data = mMem.fetchByte(adres);
            prevData = data;
            data = data << 1;
            mMem.writeByte(adres, data);
            mCondCodeReg.Z = data ? 0 : 1;
            mCondCodeReg.N = data & 0x80 ? 1 : 0;
            mCondCodeReg.V = (data ^ prevData) ? 1 : 0;
            mCondCodeReg.C = prevData - data > 0xFF ? 1 : 0;
            result = 2;
            break;
        case Emulator::extended:
            adres = mMem.fetchWord(mPCReg);
            data = mMem.fetchByte(adres);
            prevData = data;
            data = data << 1;
            mMem.writeByte(adres, data);
            mCondCodeReg.Z = data ? 0 : 1;
            mCondCodeReg.N = data & 0x80 ? 1 : 0;
            mCondCodeReg.V = (data ^ prevData) ? 1 : 0;
            mCondCodeReg.C = prevData - data > 0xFF ? 1 : 0;
            result = 3;
            break;
        case Emulator::accuA:
            prevData = mAccA;
            mAccA = mAccA << 1;
            mCondCodeReg.Z = mAccA ? 0 : 1;
            mCondCodeReg.N = mAccA & 0x80 ? 1 : 0;
            mCondCodeReg.V = (mAccA ^ prevData) ? 1 : 0;
            mCondCodeReg.C = prevData - mAccA > 0xFF ? 1 : 0;
            result = 2;
            break;
        case Emulator::accuB:
            prevData = mAccB;
            mAccB = mAccB << 1;
            mCondCodeReg.Z = mAccB ? 0 : 1;
            mCondCodeReg.N = mAccB & 0x80 ? 1 : 0;
            mCondCodeReg.V = (mAccB ^ prevData) ? 1 : 0;
            mCondCodeReg.C = prevData - mAccB > 0xFF ? 1 : 0;
            result = 2;
            break;
        default:
            break;
        }
        return result;
    }

    int Mpu::asr(Emulator::AddresMode mode) {
        mPCReg++;
        uint8_t bit7 = 0;
        int adres = 0;
        int data = 0;
        int prevData = 0;
        int result = 0;
        switch (mode)
        {
        case Emulator::indexed:
            adres = mXReg + mMem.fetchByte(mPCReg);
            data = mMem.fetchByte(adres);
            prevData = data;
            bit7 = data & 0x80;
            data = data >> 1;
            data |= bit7;
            mMem.writeByte(adres, data);
            mCondCodeReg.Z = data ? 0 : 1;
            mCondCodeReg.N = data & 0x80 ? 1 : 0;
            mCondCodeReg.V = (data ^ prevData) ? 1 : 0;
            mCondCodeReg.C = prevData - data > 0xFF ? 1 : 0;
            result = 2;
            break;
        case Emulator::extended:
            adres = mMem.fetchWord(mPCReg);
            data = mMem.fetchByte(adres);
            prevData = data;
            bit7 = data & 0x80;
            data = data >> 1;
            data |= bit7;
            mMem.writeByte(adres, data);
            mCondCodeReg.Z = data ? 0 : 1;
            mCondCodeReg.N = data & 0x80 ? 1 : 0;
            mCondCodeReg.V = (data ^ prevData) ? 1 : 0;
            mCondCodeReg.C = prevData - data > 0xFF ? 1 : 0;
            result = 3;
            break;
        case Emulator::accuA:
            prevData = mAccA;
            bit7 = prevData & 0x80;
            mAccA = mAccA << 1;
            mAccA |= bit7;
            mCondCodeReg.Z = mAccA ? 0 : 1;
            mCondCodeReg.N = mAccA & 0x80 ? 1 : 0;
            mCondCodeReg.V = (mAccA ^ prevData) ? 1 : 0;
            mCondCodeReg.C = prevData - mAccA > 0xFF ? 1 : 0;
            result = 2;
            break;
        case Emulator::accuB:
            prevData = mAccB;
            bit7 = prevData & 0x80;
            mAccB = mAccB << 1;
            mAccB |= bit7;
            mCondCodeReg.Z = mAccB ? 0 : 1;
            mCondCodeReg.N = mAccB & 0x80 ? 1 : 0;
            mCondCodeReg.V = (mAccB ^ prevData) ? 1 : 0;
            mCondCodeReg.C = prevData - mAccB > 0xFF ? 1 : 0;
            result = 2;
            break;
        default:
            break;
        }
        return result;
    }

    int Mpu::bcc() {    
        mPCReg++;
        int8_t offset = mMem.fetchByte(mPCReg);
        if (mCondCodeReg.C == 0) {
            mPCReg = mPCReg + offset + 2;
        }
        return 4;
    }
    
    int Mpu::bcs() {
        mPCReg++;
        int8_t offset = mMem.fetchByte(mPCReg);
        if (mCondCodeReg.C == 1) {
            mPCReg = mPCReg + offset + 2;
        }
        return 4; }
    int Mpu::beq(){ 
        mPCReg++;
        int8_t offset = mMem.fetchByte(mPCReg);
        if (mCondCodeReg.Z == 0) {
            mPCReg = mPCReg + offset + 2;
        }
        return 4;
    }
    int Mpu::bge(){ 
        mPCReg++;
        int8_t offset = mMem.fetchByte(mPCReg);
        if ((mCondCodeReg.N ^ mCondCodeReg.V) == 0) {
            mPCReg = mPCReg + offset + 2;
        }
        return 4;
    }
    int Mpu::bgt(){
        mPCReg++;
        int8_t offset = mMem.fetchByte(mPCReg);
        if ((mCondCodeReg.Z ^ (mCondCodeReg.N ^ mCondCodeReg.V)) == 0) {
            mPCReg = mPCReg + offset + 2;
        }
        return 4;
    }
    int Mpu::bhi(){
        mPCReg++;
        int8_t offset = mMem.fetchByte(mPCReg);
        if ((mCondCodeReg.C ^ mCondCodeReg.Z ) == 0) {
            mPCReg = mPCReg + offset + 2;
        }
        return 4;
    }
    int Mpu::bit(AddresMode mode, uint8_t& dest) { 
        mPCReg++;
        int cycles = 0;
        int data = 0;
        int offset = 0;
        int addres = 0;
        switch (mode)
        {
        case Emulator::immidiate:
            data = mMem.fetchByte(mPCReg);
            data = data & dest;
            mCondCodeReg.Z = data ? 0 : 1;
            mCondCodeReg.N = data & 0x80 ? 1 : 0;
            mCondCodeReg.V = (data ^ dest) ? 1 : 0;
            mCondCodeReg.C = dest - data > 0xFF ? 1 : 0;
            cycles = 2;
            break;
        case Emulator::direct:
            addres = mMem.fetchByte(mPCReg);
            data = mMem.fetchByte(addres);
            data = data & dest;
            mCondCodeReg.Z = data ? 0 : 1;
            mCondCodeReg.N = data & 0x80 ? 1 : 0;
            mCondCodeReg.V = (data ^ dest) ? 1 : 0;
            mCondCodeReg.C = dest - data > 0xFF ? 1 : 0;
            cycles = 3;
            break;
        case Emulator::indexed:
            addres = mXReg + mMem.fetchByte(mPCReg);
            data = mMem.fetchByte(addres);
            data = data & dest;
            mCondCodeReg.Z = data ? 0 : 1;
            mCondCodeReg.N = data & 0x80 ? 1 : 0;
            mCondCodeReg.V = (data ^ dest) ? 1 : 0;
            mCondCodeReg.C = dest - data > 0xFF ? 1 : 0;
            cycles = 5;
            break;
        case Emulator::extended:
            addres = mMem.fetchWord(mPCReg);
            data = dest & mMem.fetchByte(addres);
            mCondCodeReg.Z = data ? 0 : 1;
            mCondCodeReg.N = data & 0x80 ? 1 : 0;
            mCondCodeReg.V = (data ^ dest) ? 1 : 0;
            mCondCodeReg.C = dest - data > 0xFF ? 1 : 0;
            cycles = 4;
            break;
            break;
        default:
            break;
        }
        return cycles;
    }

    int Mpu::ble(){
        mPCReg++;
        int8_t offset = mMem.fetchByte(mPCReg);
        if ((mCondCodeReg.Z ^(mCondCodeReg.N ^ mCondCodeReg.V)) == 1) {
            mPCReg = mPCReg + offset + 2;
        }
        return 4;
    }
    int Mpu::bls(){
        mPCReg++;
        int8_t offset = mMem.fetchByte(mPCReg);
        if ((mCondCodeReg.C ^ mCondCodeReg.Z) == 1) {
            mPCReg = mPCReg + offset + 2;
        }
        return 4;
    }
    int Mpu::blt(){
        mPCReg++;
        int8_t offset = mMem.fetchByte(mPCReg);
        if ((mCondCodeReg.N ^ mCondCodeReg.V) == 0) {
            mPCReg = mPCReg + offset + 2;
        }
        return 4;
    }
    int Mpu::bmi(){
        mPCReg++;
        int8_t offset = mMem.fetchByte(mPCReg);
        if (mCondCodeReg.N == 0) {
            mPCReg = mPCReg + offset + 2;
        }
        return 4;
    }
    int Mpu::bne(){
        mPCReg++;
        int8_t offset = mMem.fetchByte(mPCReg);
        if (mCondCodeReg.Z == 0) {
            mPCReg = mPCReg + offset + 2;
        }
        return 4;
    }
    int Mpu::bpl(){
        mPCReg++;
        int8_t offset = mMem.fetchByte(mPCReg);
        if (mCondCodeReg.N == 0) {
            mPCReg = mPCReg + offset + 2;
        }
        return 4;
    }
    int Mpu::bra(){
        mPCReg++;
        int8_t offset = mMem.fetchByte(mPCReg);
        mPCReg = mPCReg + offset + 2;
        return 4;
    }
    int Mpu::bsr(){
        return -1;
    }
    int Mpu::bvc(){
        mPCReg++;
        int8_t offset = mMem.fetchByte(mPCReg);
        if (mCondCodeReg.V == 0) {
            mPCReg = mPCReg + offset + 2;
        }
        return 4;
    }
    int Mpu::bvs(){
        mPCReg++;
        int8_t offset = mMem.fetchByte(mPCReg);
        if (mCondCodeReg.V == 1) {
            mPCReg = mPCReg + offset + 2;
        }
        return 4;
    }
    int Mpu::cba(){ 
        int result = mAccA - mAccB;
        mCondCodeReg.Z = result ? 0 : 1;
        mCondCodeReg.N = result & 0x80 ? 1 : 0;
        mCondCodeReg.V = (result ^ mAccA) & (result ^ mAccB) ? 1 : 0;
        mCondCodeReg.C = mAccA + mAccB > 0xFF ? 1 : 0;
        return 2;
    }

    int Mpu::clc()
    {
        mPCReg++;
        mCondCodeReg.resetCFlag();
        return 2;
    }

    int Mpu::cli()
    {
        mPCReg++;
        mCondCodeReg.resetIFlag();
        return 2;
    }

    int Mpu::clr(AddresMode mode)
    {
        mPCReg++;
        int result = 0;
        switch (mode)
        {
        case Emulator::indexed:
            mMem.writeByte(mXReg + mMem.fetchByte(mPCReg),0);
            break;
        case Emulator::extended:
            mMem.writeByte(mMem.fetchWord(mPCReg), 0);
            result = 6;
            break;
        case Emulator::accuA:
            mAccA = 0;
            result = 2;
            break;
        case Emulator::accuB:
            mAccB = 0;
            result = 2;
            break;
        default:
            break;
        }
        return result;
    }

    int Mpu::clv()
    {
        mPCReg++;
        mCondCodeReg.resetVFlag();
        return 2;
    }

    int Mpu::cmp(AddresMode mode, uint8_t& dest)
    {
        int cycles = 0;
        uint8_t result = 0;
        uint8_t addr8 = 0;
        uint16_t addr16 = 0;
        uint8_t offset = 0;
        uint8_t data = 0;
        mPCReg++;
        switch (mode)
        {
        case Emulator::immidiate:
            data = mMem.fetchByte(mPCReg);
            result = dest - data;
            //set flags
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            mCondCodeReg.V = (result ^ dest) ? 1 : 0;
            mCondCodeReg.C = dest - data > 0xFF ? 1 : 0;
            cycles = 2;
            break;
        case Emulator::direct:
            addr8 = mMem.fetchByte(mPCReg);
            result = dest + mMem.fetchByte(addr8);
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            mCondCodeReg.V = (result ^ dest) ? 1 : 0;
            mCondCodeReg.C = dest - mMem.fetchByte(addr8) > 0xFF ? 1 : 0;
            cycles = 3;
            break;
        case Emulator::indexed:
            offset = mMem.fetchByte(mPCReg);
            data = mMem.fetchByte(mXReg + offset);
            result = dest - data;
            //set flags
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            mCondCodeReg.V = (result ^ dest) ? 1 : 0;
            mCondCodeReg.C = dest - data > 0xFF ? 1 : 0;
            cycles = 5;
            break;
        case Emulator::extended:
            addr16 = mMem.fetchWord(mPCReg);
            result = dest - mMem.fetchByte(addr16);
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            mCondCodeReg.V = (result ^ dest) ? 1 : 0;
            mCondCodeReg.C = dest - mMem.fetchByte(addr16) > 0xFF ? 1 : 0;
            cycles = 4;
            break;
        default:
            result = 0;
            break;
        }
        return cycles;
    }
        
    int Mpu::com(AddresMode mode)
    {
        mPCReg++;
        uint8_t result = 0;
        int adres = 0;
        uint8_t data = 0;
        int cycles = 0;
        switch (mode)
        {
        case Emulator::indexed:
            adres = mXReg + mMem.fetchByte(mPCReg);
            data = mMem.fetchByte(adres);
            result = 0xFF - data;
            mMem.writeByte(adres, data);
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            result = 7;
            break;
        case Emulator::extended:
            adres = mMem.fetchWord(mPCReg);
            data = mMem.fetchByte(adres);
            result = 0xFF - data;
            mMem.writeByte(adres, data);
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            result = 6;
            break;
        case Emulator::accuA:
            result = 0xFF - mAccA;
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            mAccA = result;
            result = 2;
            break;
        case Emulator::accuB:
            result = 0xFF - mAccB;
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            mAccB = result;
            result = 2;
            break;
        default:
            break;
        }
        mCondCodeReg.setCFlag();
        mCondCodeReg.resetVFlag();
        return cycles;
    }

    int Mpu::cpx(AddresMode mode)
    {
        int cycles = 0;
        uint8_t result = 0;
        uint8_t addr8 = 0;
        uint16_t addr16 = 0;
        uint8_t offset = 0;
        uint8_t data = 0;
        mPCReg++;
        switch (mode)
        {
        case Emulator::immidiate:
            data = mMem.fetchByte(mPCReg);
            result = mXReg - data;
            //set flags
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            mCondCodeReg.V = (result ^ mXReg) ? 1 : 0;
            cycles = 2;
            break;
        case Emulator::direct:
            addr8 = mMem.fetchByte(mPCReg);
            result = mXReg + mMem.fetchByte(addr8);
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            mCondCodeReg.V = (result ^ mXReg) ? 1 : 0;
            cycles = 3;
            break;
        case Emulator::indexed:
            offset = mMem.fetchByte(mPCReg);
            data = mMem.fetchByte(mXReg + offset);
            result = mXReg - data;
            //set flags
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            mCondCodeReg.V = (result ^ mXReg) ? 1 : 0;
            cycles = 5;
            break;
        case Emulator::extended:
            addr16 = mMem.fetchWord(mPCReg);
            result = mXReg - mMem.fetchByte(addr16);
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            mCondCodeReg.V = (result ^ mXReg) ? 1 : 0;
            cycles = 4;
            break;
        default:
            result = 0;
            break;
        }
        return cycles;
    }

    int Mpu::daa()
    {
        return 0;
    }

    int Mpu::dec(AddresMode mode)
    {
        mPCReg++;
        int cycles = 0;
        uint8_t result = 0;
        uint16_t addr = 0;
        switch (mode)
        {
        case Emulator::indexed:
            result = mMem.fetchByte(mXReg + mPCReg) - 1;
            mMem.writeByte(mXReg + mMem.fetchByte(mPCReg), result);
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            mCondCodeReg.V = (result ^ mXReg) ? 1 : 0;
            cycles = 7;
            break;
        case Emulator::extended:
            addr = mMem.fetchWord(mPCReg);
            result = mMem.fetchByte(mPCReg) - 1;
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            mCondCodeReg.V = (result ^ mMem.fetchByte(mPCReg)) ? 1 : 0;
            mMem.writeByte(mMem.fetchWord(addr), result);
            cycles = 6;
            break;
        case Emulator::accuA:
            mCondCodeReg.Z = mAccA - 1 ? 0 : 1;
            mCondCodeReg.N = mAccA - 1 & 0x80 ? 1 : 0;
            mCondCodeReg.V = ((mAccA - 1) ^ mAccA) ? 1 : 0;
            mAccA = mAccA - 1;
            cycles = 2;
            break;
        case Emulator::accuB:
            mCondCodeReg.Z = mAccB - 1 ? 0 : 1;
            mCondCodeReg.N = mAccB - 1 & 0x80 ? 1 : 0;
            mCondCodeReg.V = ((mAccB - 1) ^ mAccB) ? 1 : 0;
            mAccB = mAccB - 1;
            cycles = 2;
            break;
        default:
            break;
        }
        return cycles;
    }

    int Mpu::des()
    {
        mSPReg = mSPReg - 1;
        return 4;
    }

    int Mpu::dex()
    {
        mXReg = mXReg - 1;
        mCondCodeReg.Z = mXReg ? 0 : 1;
        return 4;
    }

    int Mpu::eor(AddresMode mode, uint8_t& dest)
    {
        int cycles = 0;
        uint8_t result = 0;
        uint8_t addr8 = 0;
        uint16_t addr16 = 0;
        uint8_t offset = 0;
        uint8_t data = 0;
        mPCReg++;
        switch (mode)
        {
        case Emulator::immidiate:
            data = mMem.fetchByte(mPCReg);
            result = dest ^ data;
            //set flags
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            mCondCodeReg.V = 0;
            cycles = 2;
            break;
        case Emulator::direct:
            addr8 = mMem.fetchByte(mPCReg);
            result = dest ^ mMem.fetchByte(addr8);
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            mCondCodeReg.V = 0;
            cycles = 3;
            break;
        case Emulator::indexed:
            offset = mMem.fetchByte(mPCReg);
            data = mMem.fetchByte(mXReg + offset);
            result = dest & data;
            //set flags
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            mCondCodeReg.V = 0;
            cycles = 5;
            break;
        case Emulator::extended:
            addr16 = mMem.fetchWord(mPCReg);
            result = dest ^ mMem.fetchByte(addr16);
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = 0;
            mCondCodeReg.V = (result ^ dest) ? 1 : 0;
            cycles = 4;
            break;
        default:
            break;
        }
        return cycles;
    }

    int Mpu::inc(AddresMode mode)
    {
        mPCReg++;
        int cycles = 0;
        uint8_t result = 0;
        uint16_t addr = 0;
        switch (mode)
        {
        case Emulator::indexed:
            result = mMem.fetchByte(mXReg + mPCReg) + 1;
            mMem.writeByte(mXReg + mMem.fetchByte(mPCReg), result);
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            mCondCodeReg.V = (result ^ mXReg) ? 1 : 0;
            cycles = 7;
            break;
        case Emulator::extended:
            addr = mMem.fetchWord(mPCReg);
            result = mMem.fetchByte(mPCReg) + 1;
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            mCondCodeReg.V = (result ^ mMem.fetchByte(mPCReg)) ? 1 : 0;
            mMem.writeByte(mMem.fetchWord(addr), result);
            cycles = 6;
            break;
        case Emulator::accuA:
            mCondCodeReg.Z = mAccA + 1 ? 0 : 1;
            mCondCodeReg.N = mAccA + 1 & 0x80 ? 1 : 0;
            mCondCodeReg.V = ((mAccA + 1) ^ mAccA) ? 1 : 0;
            mAccA = mAccA + 1;
            cycles = 2;
            break;
        case Emulator::accuB:
            mCondCodeReg.Z = mAccB + 1 ? 0 : 1;
            mCondCodeReg.N = mAccB + 1 & 0x80 ? 1 : 0;
            mCondCodeReg.V = ((mAccB + 1) ^ mAccB) ? 1 : 0;
            mAccB = mAccB + 1;
            cycles = 2;
            break;
        default:
            break;
        }
        return cycles;
    }

    int Mpu::ins()
    {
        mSPReg = mSPReg + 1;
        return 4;
    }

    int Mpu::inx()
    {
        mXReg = mXReg + 1;
        mCondCodeReg.Z = mXReg ? 0 : 1;
        return 4;
    }

    int Mpu::jmp(AddresMode mode)
    {
        int cycles = 0;
        uint8_t addr8 = 0;
        uint16_t addr16 = 0;
        uint8_t offset = 0;
        uint8_t data = 0;
        mPCReg++;
        switch (mode) {
        case Emulator::indexed:
            offset = mMem.fetchByte(mPCReg);
            data = mMem.fetchByte(mXReg + offset);
            cycles = 4;
            break;
        case Emulator::extended:
            addr16 = mMem.fetchWord(mPCReg);
            data = mMem.fetchByte(addr16);
            cycles = 3;
            break;
        }
        mPCReg = data;
        return cycles;
    }

    int Mpu::jsr(AddresMode mode)
    {
        return 0;
    }

    int Mpu::lda(AddresMode mode, uint8_t& dest)
    {
        int cycles = 0;
        uint8_t addr8 = 0;
        uint16_t addr16 = 0;
        uint8_t offset = 0;
        uint8_t data = 0;
        mPCReg++;
        switch (mode)
        {
        case Emulator::immidiate:
            data = mMem.fetchByte(mPCReg);
            //set flags
            mCondCodeReg.Z = data ? 0 : 1;
            mCondCodeReg.N = data & 0x80 ? 1 : 0;
            mCondCodeReg.V = 0;
            cycles = 2;
            break;
        case Emulator::direct:
            addr8 = mMem.fetchByte(mPCReg);
            data = mMem.fetchByte(addr8);
            mCondCodeReg.Z = data ? 0 : 1;
            mCondCodeReg.N = data & 0x80 ? 1 : 0;
            mCondCodeReg.V = 0;
            cycles = 3;
            break;
        case Emulator::indexed:
            offset = mMem.fetchByte(mPCReg);
            data = mMem.fetchByte(mXReg + offset);
            //set flags
            mCondCodeReg.Z = data ? 0 : 1;
            mCondCodeReg.N = data & 0x80 ? 1 : 0;
            mCondCodeReg.V = 0;
            cycles = 5;
            break;
        case Emulator::extended:
            addr16 = mMem.fetchWord(mPCReg);
            data = mMem.fetchByte(addr16);
            mCondCodeReg.Z = data ? 0 : 1;
            mCondCodeReg.N = data & 0x80 ? 1 : 0;;
            mCondCodeReg.V = 0;
            cycles = 4;
            break;
        default:
            break;
        }
        dest = data;
        return cycles;
    }

    int Mpu::lds(AddresMode mode)
    {
        int cycles = 0;
        uint8_t addr8 = 0;
        uint16_t addr16 = 0;
        uint8_t offset = 0;
        uint16_t data = 0;
        mPCReg++;
        switch (mode)
        {
        case Emulator::immidiate:
            data = mMem.fetchWord(mPCReg);
            //set flags
            mCondCodeReg.Z = data ? 0 : 1;
            mCondCodeReg.N = data & 0x80 ? 1 : 0;
            mCondCodeReg.V = 0;
            cycles = 3;
            break;
        case Emulator::direct:
            addr8 = mMem.fetchByte(mPCReg);
            data = mMem.fetchWord(addr8);
            mCondCodeReg.Z = data ? 0 : 1;
            mCondCodeReg.N = data & 0x80 ? 1 : 0;
            mCondCodeReg.V = 0;
            cycles = 4;
            break;
        case Emulator::indexed:
            offset = mMem.fetchByte(mPCReg);
            data = mMem.fetchWord(mXReg + offset);
            //set flags
            mCondCodeReg.Z = data ? 0 : 1;
            mCondCodeReg.N = data & 0x80 ? 1 : 0;
            mCondCodeReg.V = 0;
            cycles = 6;
            break;
        case Emulator::extended:
            addr16 = mMem.fetchWord(mPCReg);
            data = mMem.fetchWord(addr16);
            mCondCodeReg.Z = data ? 0 : 1;
            mCondCodeReg.N = data & 0x80 ? 1 : 0;;
            mCondCodeReg.V = 0;
            cycles = 5;
            break;
        default:
            break;
        }
        mSPReg = data;
        return cycles;
    }

    int Mpu::ldx(AddresMode mode)
    {
        int cycles = 0;
        uint8_t addr8 = 0;
        uint16_t addr16 = 0;
        uint8_t offset = 0;
        uint16_t data = 0;
        mPCReg++;
        switch (mode)
        {
        case Emulator::immidiate:
            data = mMem.fetchWord(mPCReg);
            //set flags
            mCondCodeReg.Z = data ? 0 : 1;
            mCondCodeReg.N = data & 0x80 ? 1 : 0;
            mCondCodeReg.V = 0;
            cycles = 3;
            break;
        case Emulator::direct:
            addr8 = mMem.fetchByte(mPCReg);
            data = mMem.fetchWord(addr8);
            mCondCodeReg.Z = data ? 0 : 1;
            mCondCodeReg.N = data & 0x80 ? 1 : 0;
            mCondCodeReg.V = 0;
            cycles = 4;
            break;
        case Emulator::indexed:
            offset = mMem.fetchByte(mPCReg);
            data = mMem.fetchWord(mXReg + offset);
            //set flags
            mCondCodeReg.Z = data ? 0 : 1;
            mCondCodeReg.N = data & 0x80 ? 1 : 0;
            mCondCodeReg.V = 0;
            cycles = 6;
            break;
        case Emulator::extended:
            addr16 = mMem.fetchWord(mPCReg);
            data = mMem.fetchWord(addr16);
            mCondCodeReg.Z = data ? 0 : 1;
            mCondCodeReg.N = data & 0x80 ? 1 : 0;;
            mCondCodeReg.V = 0;
            cycles = 5;
            break;
        default:
            break;
        }
        mSPReg = data;
        return cycles;
    }
    /// <summary>
    /// Logical bit shift right. Sign bit(7) is set to 0;
    /// </summary>
    /// <param name="mode">Addressing mode</param>
    /// <returns>number of cycles</returns>
    /// <sideefect>Value in accumulator or memory location is changed by this operation<sideefect>
    int Mpu::lsr(AddresMode mode)
    {
        mPCReg++;
        int offset = 0;
        int adres = 0;
        int data = 0;
        int prevData = 0;
        int cycles = 0;
        switch (mode)
        {
        case Emulator::indexed:
            adres = mXReg + mMem.fetchByte(mPCReg);
            data = mMem.fetchByte(adres);
            prevData = data;
            data = data >> 1;
            mMem.writeByte(adres, data);
            mCondCodeReg.Z = data ? 0 : 1;
            mCondCodeReg.N = 0;
            mCondCodeReg.V = (data ^ prevData) ? 1 : 0;
            mCondCodeReg.C = prevData - data > 0xFF ? 1 : 0;
            cycles = 7;
            break;
        case Emulator::extended:
            adres = mMem.fetchWord(mPCReg);
            data = mMem.fetchByte(adres);
            prevData = data;
            data = data >> 1;
            mMem.writeByte(adres, data);
            mCondCodeReg.Z = data ? 0 : 1;
            mCondCodeReg.N = 0;
            mCondCodeReg.V = (data ^ prevData) ? 1 : 0;
            mCondCodeReg.C = prevData - data > 0xFF ? 1 : 0;
            cycles = 6;
            break;
        case Emulator::accuA:
            prevData = mAccA;
            mAccA = mAccA << 1;
            mCondCodeReg.Z = mAccA ? 0 : 1;
            mCondCodeReg.N = 0;
            mCondCodeReg.V = (mAccA ^ prevData) ? 1 : 0;
            mCondCodeReg.C = prevData - mAccA > 0xFF ? 1 : 0;
            cycles = 2;
            break;
        case Emulator::accuB:
            prevData = mAccB;
            mAccB = mAccB << 1;
            mCondCodeReg.Z = mAccB ? 0 : 1;
            mCondCodeReg.N = 0;
            mCondCodeReg.V = (mAccB ^ prevData) ? 1 : 0;
            mCondCodeReg.C = prevData - mAccB > 0xFF ? 1 : 0;
            cycles = 2;
            break;
        default:
            break;
        }
        return cycles;
    }

    int Mpu::neg(AddresMode mode)
    {
        mPCReg++;
        int offset = 0;
        int adres = 0;
        uint8_t result = 0;
        int data = 0;
        int cycles = 0;
        switch (mode)
        {
        case Emulator::indexed:
            adres = mXReg + mMem.fetchByte(mPCReg);
            data = mMem.fetchByte(adres);
            result = 0 - data;
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            mCondCodeReg.V = (result ^ data) ? 1 : 0;
            mCondCodeReg.C = data - result > 0xFF ? 1 : 0;
            mMem.writeByte(adres, result);
            cycles = 7;
            break;
        case Emulator::extended:
            adres = mMem.fetchWord(mPCReg);
            data = mMem.fetchByte(adres);
            result = 0 - data;
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            mCondCodeReg.V = (result ^ data) ? 1 : 0;
            mCondCodeReg.C = data - result > 0xFF ? 1 : 0;
            mMem.writeByte(adres, result);
            cycles = 6;
            break;
        case Emulator::accuA:
            result = 0 - mAccA;
            mCondCodeReg.Z = mAccA ? 0 : 1;
            mCondCodeReg.N = mAccA & 0x80 ? 1 : 0;
            mCondCodeReg.V = (mAccA ^ result) ? 1 : 0;
            mCondCodeReg.C = result - mAccA > 0xFF ? 1 : 0;
            mAccA = result;
            cycles = 2;
            break;
        case Emulator::accuB:
            result = 0 - mAccB;
            mCondCodeReg.Z = mAccB ? 0 : 1;
            mCondCodeReg.N = mAccB & 0x80 ? 1 : 0;
            mCondCodeReg.V = (mAccB ^ result) ? 1 : 0;
            mCondCodeReg.C = result - mAccB > 0xFF ? 1 : 0;
            mAccB = result;
            cycles = 2;
            break;
        default:
            break;
        }
        return cycles;
    }
    int Mpu::nop()
    {
        mPCReg++;
        return 1;
    }

    int Mpu::ora(AddresMode mode, uint8_t& dest)
    {
        int cycles = 0;
        uint8_t result = 0;
        uint8_t addr8 = 0;
        uint16_t addr16 = 0;
        uint8_t offset = 0;
        uint8_t data = 0;
        mPCReg++;
        switch (mode)
        {
        case Emulator::immidiate:
            data = mMem.fetchByte(mPCReg);
            result = dest | data;
            //set flags
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            mCondCodeReg.V = 0;
            cycles = 2;
            break;
        case Emulator::direct:
            addr8 = mMem.fetchByte(mPCReg);
            result = dest & mMem.fetchByte(addr8);
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            mCondCodeReg.V = 0;
            cycles = 3;
            break;
        case Emulator::indexed:
            offset = mMem.fetchByte(mPCReg);
            data = mMem.fetchByte(mXReg + offset);
            result = dest | data;
            //set flags
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            mCondCodeReg.V = 0;
            cycles = 5;
            break;
        case Emulator::extended:
            addr16 = mMem.fetchWord(mPCReg);
            result = dest | mMem.fetchByte(addr16);
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            mCondCodeReg.V = 0;
            cycles = 4;
            break;
        default:
            break;
        }
        dest = result;
        return cycles;
    }

    int Mpu::psh(AddresMode mode)
    {
        mPCReg++;
        switch (mode) {
        case Emulator::AddresMode::accuA:
            mMem.writeByte(mSPReg, mAccA);
            mSPReg--;
            break;
        case Emulator::AddresMode::accuB:
            mMem.writeByte(mSPReg, mAccB);
            mSPReg--;
            break;
        }
        return 4;
    }

    int Mpu::pul(AddresMode mode)
    {
        mPCReg++;
        switch (mode) {
        case Emulator::AddresMode::accuA:
            ++mSPReg;
            mAccA = mMem.fetchByte(mSPReg);
            break;
        case Emulator::AddresMode::accuB:
            ++mSPReg;
            mAccB = mMem.fetchByte(mSPReg);
            break;
        }
        return 4;
    }

    int Mpu::rol(AddresMode mode)
    {
        return 0;
    }

    int Mpu::ror(AddresMode mode)
    {
        return 0;
    }

    int Mpu::rti()
    {
        return 0;
    }

    int Mpu::rts()
    {
        return 0;
    }

    int Mpu::sba()
    {
        uint8_t result = mAccA - mAccB;
        //set flags
        mCondCodeReg.Z = result ? 0 : 1;
        mCondCodeReg.N = result & 0x80 ? 1 : 0;
        mCondCodeReg.V = (result ^ mAccA) & (result ^ mAccB) ? 1 : 0;
        mCondCodeReg.C = mAccA - mAccB < 0 ? 1 : 0;
        mAccA = result;
        return 2;
    }

    int Mpu::sbc(AddresMode mode, uint8_t& dest)
    {
        int cycles = 0;
        uint8_t result = 0;
        uint8_t addr8 = 0;
        uint16_t addr16 = 0;
        uint8_t offset = 0;
        uint8_t data = 0;
        mPCReg++;
        switch (mode)
        {
        case Emulator::immidiate:
            data = mMem.fetchByte(mPCReg);
            result = dest - data - mCondCodeReg.C;
            //set flags
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            mCondCodeReg.V = (result ^ dest) ? 1 : 0;
            mCondCodeReg.C = dest - data > 0xFF ? 1 : 0;
            cycles = 2;
            break;
        case Emulator::direct:
            addr8 = mMem.fetchByte(mPCReg);
            result = dest + mMem.fetchByte(addr8) - mCondCodeReg.C;
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            mCondCodeReg.V = (result ^ dest) ? 1 : 0;
            mCondCodeReg.C = dest - mMem.fetchByte(addr8) > 0xFF ? 1 : 0;
            cycles = 3;
            break;
        case Emulator::inherent:
            addr8 = mMem.fetchByte(addr8);
            break;
        case Emulator::indexed:
            offset = mMem.fetchByte(mPCReg);
            data = mMem.fetchByte(mXReg + offset);
            result = dest - data - mCondCodeReg.C;
            //set flags
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            mCondCodeReg.V = (result ^ dest) ? 1 : 0;
            mCondCodeReg.C = dest - data > 0xFF ? 1 : 0;
            cycles = 5;
            break;
        case Emulator::extended:
            addr16 = mMem.fetchWord(mPCReg);
            result = dest - mMem.fetchByte(addr16) - mCondCodeReg.C;
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            mCondCodeReg.V = (result ^ dest) ? 1 : 0;
            mCondCodeReg.C = dest - mMem.fetchByte(addr16) > 0xFF ? 1 : 0;
            cycles = 4;
            break;
        default:
            result = 0;
            break;
        }
        dest = result;
        return cycles;
    }

    int Mpu::sec()
    {
        mPCReg++;
        mCondCodeReg.setCFlag();
        return 2;
    }

    int Mpu::sei()
    {
        mPCReg++;
        mCondCodeReg.setIFlag();
        return 2;
    }

    int Mpu::sev()
    {
        mPCReg++;
        mCondCodeReg.setVFlag();
        return 2;
    }

    int Mpu::sta(AddresMode mode, uint8_t& source)
    {
        int cycles = 0;
        uint8_t addr8 = 0;
        uint16_t addr16 = 0;
        uint8_t offset = 0;
        mPCReg++;
        switch (mode)
        {
        case Emulator::direct:
            addr8 = mMem.fetchByte(mPCReg);
            mCondCodeReg.Z = source ? 0 : 1;
            mCondCodeReg.N = source & 0x80 ? 1 : 0;
            mCondCodeReg.V = 0;
            mMem.writeByte(addr8, source);
            cycles = 4;
            break;
        case Emulator::indexed:
            offset = mMem.fetchByte(mPCReg);
            addr16 = mXReg + offset;
            //set flags
            mCondCodeReg.Z = source ? 0 : 1;
            mCondCodeReg.N = source & 0x80 ? 1 : 0;
            mCondCodeReg.V = 0;
            mMem.writeByte(addr16, source);
            cycles = 5;
            break;
        case Emulator::extended:
            addr16 = mMem.fetchWord(mPCReg);
            //set flags
            mCondCodeReg.Z = source ? 0 : 1;
            mCondCodeReg.N = source & 0x80 ? 1 : 0;
            mCondCodeReg.V = 0;
            mMem.writeByte(addr16, source);
            cycles = 4;
            break;
        default:
            break;
        }
        return cycles;
    }

    int Mpu::sts(AddresMode mode)
    {
        int cycles = 0;
        uint8_t addr8 = 0;
        uint16_t addr16 = 0;
        uint8_t offset = 0;
        mPCReg++;
        switch (mode)
        {
        case Emulator::direct:
            addr8 = mMem.fetchByte(mPCReg);
            mCondCodeReg.Z = mSPReg ? 0 : 1;
            mCondCodeReg.N = mSPReg & 0x80 ? 1 : 0;
            mCondCodeReg.V = 0;
            mMem.writeWord(addr8, mSPReg);
            cycles = 5;
            break;
        case Emulator::indexed:
            offset = mMem.fetchByte(mPCReg);
            addr16 = mXReg + offset;
            //set flags
            mCondCodeReg.Z = mSPReg ? 0 : 1;
            mCondCodeReg.N = mSPReg & 0x80 ? 1 : 0;
            mCondCodeReg.V = 0;
            mMem.writeWord(addr16, mSPReg);
            cycles = 7;
            break;
        case Emulator::extended:
            addr16 = mMem.fetchWord(mPCReg);
            mCondCodeReg.Z = mSPReg ? 0 : 1;
            mCondCodeReg.N = mSPReg & 0x80 ? 1 : 0;;
            mCondCodeReg.V = 0;
            mMem.writeWord(addr16, mSPReg);
            cycles = 6;
            break;
        default:
            break;
        }
        return cycles;
    }

    int Mpu::stx(AddresMode mode)
    {
        int cycles = 0;
        uint8_t addr8 = 0;
        uint16_t addr16 = 0;
        uint8_t offset = 0;
        mPCReg++;
        switch (mode)
        {
        case Emulator::direct:
            addr8 = mMem.fetchByte(mPCReg);
            mCondCodeReg.Z = mXReg ? 0 : 1;
            mCondCodeReg.N = mXReg & 0x80 ? 1 : 0;
            mCondCodeReg.V = 0;
            mMem.writeWord(addr8, mXReg);
            cycles = 5;
            break;
        case Emulator::indexed:
            offset = mMem.fetchByte(mPCReg);
            addr16 = mXReg + offset;
            //set flags
            mCondCodeReg.Z = mXReg ? 0 : 1;
            mCondCodeReg.N = mXReg & 0x80 ? 1 : 0;
            mCondCodeReg.V = 0;
            mMem.writeWord(addr16, mXReg);
            cycles = 7;
            break;
        case Emulator::extended:
            addr16 = mMem.fetchWord(mPCReg);
            mCondCodeReg.Z = mXReg ? 0 : 1;
            mCondCodeReg.N = mXReg & 0x80 ? 1 : 0;;
            mCondCodeReg.V = 0;
            mMem.writeWord(addr16, mXReg);
            cycles = 6;
            break;
        default:
            break;
        }
        return cycles;
    }
    int Mpu::sub(AddresMode mode, uint8_t& dest)
    {
        int cycles = 0;
        uint8_t result = 0;
        uint8_t addr8 = 0;
        uint16_t addr16 = 0;
        uint8_t offset = 0;
        uint8_t data = 0;
        mPCReg++;
        switch (mode)
        {
        case Emulator::immidiate:
            data = mMem.fetchByte(mPCReg);
            result = dest - data;
            //set flags
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            mCondCodeReg.V = (result ^ dest) ? 1 : 0;
            mCondCodeReg.C = dest - data > 0xFF ? 1 : 0;
            cycles = 2;
            break;
        case Emulator::direct:
            addr8 = mMem.fetchByte(mPCReg);
            result = dest + mMem.fetchByte(addr8);
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            mCondCodeReg.V = (result ^ dest) ? 1 : 0;
            mCondCodeReg.C = dest - mMem.fetchByte(addr8) > 0xFF ? 1 : 0;
            cycles = 2;
            break;
        case Emulator::inherent:
            addr8 = mMem.fetchByte(addr8);
            break;
        case Emulator::indexed:
            offset = mMem.fetchByte(mPCReg);
            data = mMem.fetchByte(mXReg + offset);
            result = dest - data;
            //set flags
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            mCondCodeReg.V = (result ^ dest) ? 1 : 0;
            mCondCodeReg.C = dest - data > 0xFF ? 1 : 0;
            break;
        case Emulator::extended:
            addr16 = mMem.fetchWord(mPCReg);
            result = dest - mMem.fetchByte(addr16);
            mCondCodeReg.Z = result ? 0 : 1;
            mCondCodeReg.N = result & 0x80 ? 1 : 0;
            mCondCodeReg.V = (result ^ dest) ? 1 : 0;
            mCondCodeReg.C = dest - mMem.fetchByte(addr16) > 0xFF ? 1 : 0;
            cycles = 3;
            break;
        default:
            result = 0;
            break;
        }
        dest = result;
        return cycles;
    }

    int Mpu::swi()
    {
        return 0;
    }

    int Mpu::tab()
    {
        mAccB = mAccA;
        mCondCodeReg.Z = mAccB ? 0 : 1;
        mCondCodeReg.N = mAccB & 0x80 ? 1 : 0;
        mCondCodeReg.V = 0;
        return 2;
    }

    int Mpu::tap()
    {
        mCondCodeReg.Z = mAccA & 0b00000001;
        mCondCodeReg.Z = mAccA & 0b00000010;
        mCondCodeReg.Z = mAccA & 0b00000100;
        mCondCodeReg.Z = mAccA & 0b00001000;
        mCondCodeReg.Z = mAccA & 0b00010000;
        return 2;
    }

    int Mpu::tba()
    {
        mAccA = mAccB;
        mCondCodeReg.Z = mAccA ? 0 : 1;
        mCondCodeReg.N = mAccA & 0x80 ? 1 : 0;
        mCondCodeReg.V = 0;
        return 2;
    }

    int Mpu::tpa()
    {
        mCondCodeReg.Z = mAccB & 0b00000001;
        mCondCodeReg.Z = mAccB & 0b00000010;
        mCondCodeReg.Z = mAccB & 0b00000100;
        mCondCodeReg.Z = mAccB & 0b00001000;
        mCondCodeReg.Z = mAccB & 0b00010000;
        return 2;
    }

    int Mpu::tst(AddresMode mode)
    {
        mPCReg++;
        int cycles = 0;
        uint8_t result;
        uint8_t offset;
        uint8_t data = 0;
        switch (mode) {
            case Emulator::AddresMode::accuA:
                result = mAccA - 0;
                mCondCodeReg.Z = result ? 0 : 1;
                mCondCodeReg.N = result & 0x80 ? 1 : 0;
                mCondCodeReg.V = 0;
                mCondCodeReg.C = 0;
                cycles = 2;
                break;
            case Emulator::AddresMode::accuB:
                result = mAccB - 0;
                mCondCodeReg.Z = result ? 0 : 1;
                mCondCodeReg.N = result & 0x80 ? 1 : 0;
                mCondCodeReg.V = 0;
                mCondCodeReg.C = 0;
                cycles = 2;
                break;
            case Emulator::AddresMode::indexed:
                offset = mMem.fetchByte(mPCReg);
                data = mMem.fetchByte(mXReg + offset);
                result = data - 0;
                mCondCodeReg.Z = result ? 0 : 1;
                mCondCodeReg.N = result & 0x80 ? 1 : 0;
                mCondCodeReg.V = 0;
                mCondCodeReg.C = 0;
                cycles = 7;
                break;
            case Emulator::AddresMode::extended:
                data = mMem.fetchWord(mPCReg);
                result = data - 0;
                mCondCodeReg.Z = result ? 0 : 1;
                mCondCodeReg.N = result & 0x80 ? 1 : 0;
                mCondCodeReg.V = 0;
                mCondCodeReg.C = 0;
                cycles = 6;
                break;
        }
        return cycles;
    }

    int Mpu::tsx()
    {
        mXReg = mSPReg;
        mSPReg++;
        return 4;
    }

    int Mpu::txs()
    {
        mSPReg = mXReg;
        mXReg--;
        return 0;
    }

    int Mpu::wai()
    {
        return 0;
    }
}