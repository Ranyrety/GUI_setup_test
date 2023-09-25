#include <gtest/gtest.h>
#include "mpu.h"

class testAccumulators : public ::testing::Test {
    public:
    Emulator::Mpu mpu;
    
    void SetUp() override {
        //run initiaziation of mpu
    }

    void TearDown() override {
        
    }
};
TEST_F(testAccumulators, abaCorrectFlagsNZ) {
        //assign
    auto v = std::vector<uint8_t>{ 0x1B, 0x01 };
            mpu.init(v,0);
            mpu.setAccA(0x00);
            mpu.setAccB(0x01);
        //act
        int cycles = mpu.execute();
        //assert
        EXPECT_EQ(cycles, 2);
        EXPECT_EQ(mpu.getAccA(), 0x01);
}

TEST_F(testAccumulators, abaSetVFlag) {
        //assign
    auto v = std::vector<uint8_t>{ 0x1B, 0x01 };
            mpu.init(v,0);
            mpu.setAccA(0xFF);
            mpu.setAccB(0x01);
        //act
        int cycles = mpu.execute();
        //assert
        EXPECT_EQ(cycles, 2);
        EXPECT_EQ(mpu.getAccA(), 0x00);
        EXPECT_EQ(mpu.getCondCodeReg()->Z, 1);
        EXPECT_EQ(mpu.getCondCodeReg()->V, 1);
        EXPECT_EQ(mpu.getCondCodeReg()->N, 0);
}


TEST_F(testAccumulators, abaSetNVFlag) {
        //assign
    auto v = std::vector<uint8_t>{ 0x1B, 0x01 };
            mpu.init(v,0);
            mpu.setAccA(0x22);
            mpu.setAccB(0x68);
        //act
        int cycles = mpu.execute();
        //assert
        EXPECT_EQ(cycles, 2);
        EXPECT_EQ(mpu.getAccA(), 0x8A);
        EXPECT_EQ(mpu.getCondCodeReg()->Z, 0);
        EXPECT_EQ(mpu.getCondCodeReg()->V, 1);
        EXPECT_EQ(mpu.getCondCodeReg()->N, 1);
}

TEST_F(testAccumulators, abaCorrectOverflowHandling) {
        //assign
    auto v = std::vector<uint8_t>{ 0x1B, 0x01 };
            mpu.init(v,0);
            mpu.setAccA(0x79);
            mpu.setAccB(0x68);
        //act
        int cycles = mpu.execute();
        //assert
        EXPECT_EQ(cycles, 2);
        EXPECT_EQ(mpu.getAccA(), 0xE1);
        EXPECT_EQ(mpu.getCondCodeReg()->Z, 0);
        EXPECT_EQ(mpu.getCondCodeReg()->V, 1);
        EXPECT_EQ(mpu.getCondCodeReg()->N, 1);
        EXPECT_EQ(mpu.getCondCodeReg()->C, 0);
}

TEST_F(testAccumulators, abaCorrectOverflowHandling02) {
        //assign
    auto v = std::vector<uint8_t>{ 0x1B, 0x01 };
            mpu.init(v,0);
            mpu.setAccA(0x79);
            mpu.setAccB(0x98);
        //act
        int cycles = mpu.execute();
        //assert
        EXPECT_EQ(cycles, 2);
        EXPECT_EQ(mpu.getAccA(), 0x11);
        EXPECT_EQ(mpu.getCondCodeReg()->Z, 0);
        EXPECT_EQ(mpu.getCondCodeReg()->V, 1);
        EXPECT_EQ(mpu.getCondCodeReg()->N, 0);
        EXPECT_EQ(mpu.getCondCodeReg()->C, 1);
}

TEST_F(testAccumulators, adcaImmidiateMode) {
    //assign
    auto v = std::vector<uint8_t>{ 0x89, 0x01 };
    mpu.init(v,0);
    mpu.setAccA(0x79);
    mpu.getCondCodeReg()->resetCFlag();
    //act
    int cycles = mpu.execute();
    //assert
    EXPECT_EQ(cycles, 2);
    EXPECT_EQ(mpu.getAccA(), 0x7A);
    EXPECT_EQ(mpu.getCondCodeReg()->Z, 0);
    EXPECT_EQ(mpu.getCondCodeReg()->V, 1);
    EXPECT_EQ(mpu.getCondCodeReg()->N, 0);
    EXPECT_EQ(mpu.getCondCodeReg()->C, 0);
    
    mpu.init(v,0);
    mpu.setAccA(0x79);
    mpu.getCondCodeReg()->setCFlag();
    //act
    cycles = mpu.execute();
    //assert
    EXPECT_EQ(cycles, 2);
    EXPECT_EQ(mpu.getAccA(), 0x7B);
    EXPECT_EQ(mpu.getCondCodeReg()->Z, 0);
    EXPECT_EQ(mpu.getCondCodeReg()->V, 1);
    EXPECT_EQ(mpu.getCondCodeReg()->N, 0);
    EXPECT_EQ(mpu.getCondCodeReg()->C, 0);
}

TEST_F(testAccumulators, adcaDirectMode) {
    //assign
    auto v = std::vector<uint8_t>{ 0x99, 0xF1 };
    mpu.init(v,0);
    mpu.setAccA(0x79);
    mpu.getCondCodeReg()->resetCFlag();
    mpu.writeData(0xF1, 0x01);
    //act
    int cycles = mpu.execute();
    //assert
    EXPECT_EQ(cycles, 2);
    EXPECT_EQ(mpu.getAccA(), 0x7A);
    EXPECT_EQ(mpu.getCondCodeReg()->Z, 0);
    EXPECT_EQ(mpu.getCondCodeReg()->V, 1);
    EXPECT_EQ(mpu.getCondCodeReg()->N, 0);
    EXPECT_EQ(mpu.getCondCodeReg()->C, 0);

    mpu.init(v,0);
    mpu.setAccA(0x79);
    mpu.getCondCodeReg()->setCFlag();
    mpu.writeData(0xF1, 0x01);
    //act
    cycles = mpu.execute();
    //assert
    EXPECT_EQ(cycles, 2);
    EXPECT_EQ(mpu.getAccA(), 0x7B);
    EXPECT_EQ(mpu.getCondCodeReg()->Z, 0);
    EXPECT_EQ(mpu.getCondCodeReg()->V, 1);
    EXPECT_EQ(mpu.getCondCodeReg()->N, 0);
    EXPECT_EQ(mpu.getCondCodeReg()->C, 0);
}

TEST_F(testAccumulators, adcbImmMode) {
    //assign
    auto v = std::vector<uint8_t>{ 0xC9, 0x01 };

    mpu.init(v,0);
    mpu.setAccB(0x79);
    mpu.getCondCodeReg()->resetCFlag();
    //act
    int cycles = mpu.execute();
    //assert
    EXPECT_EQ(cycles, 2);
    EXPECT_EQ(mpu.getAccB(), 0x7A);
    EXPECT_EQ(mpu.getCondCodeReg()->Z, 0);
    EXPECT_EQ(mpu.getCondCodeReg()->V, 1);
    EXPECT_EQ(mpu.getCondCodeReg()->N, 0);
    EXPECT_EQ(mpu.getCondCodeReg()->C, 0);

    mpu.init(v,0);
    mpu.setAccB(0x79);
    mpu.getCondCodeReg()->setCFlag();
    //act
    cycles = mpu.execute();
    //assert
    EXPECT_EQ(cycles, 2);
    EXPECT_EQ(mpu.getAccB(), 0x7B);
    EXPECT_EQ(mpu.getCondCodeReg()->Z, 0);
    EXPECT_EQ(mpu.getCondCodeReg()->V, 1);
    EXPECT_EQ(mpu.getCondCodeReg()->N, 0);
    EXPECT_EQ(mpu.getCondCodeReg()->C, 0);
}
TEST_F(testAccumulators, adcbDirectMode) {
    //assign
    auto v = std::vector<uint8_t>{ 0xD9, 0xF1 };
    mpu.init(v,0);
    mpu.setAccB(0x79);
    mpu.getCondCodeReg()->resetCFlag();
    mpu.writeData(0xF1, 0x01);
    //act
    int cycles = mpu.execute();
    //assert
    EXPECT_EQ(cycles, 2);
    EXPECT_EQ(mpu.getAccB(), 0x7A);
    EXPECT_EQ(mpu.getCondCodeReg()->Z, 0);
    EXPECT_EQ(mpu.getCondCodeReg()->V, 1);
    EXPECT_EQ(mpu.getCondCodeReg()->N, 0);
    EXPECT_EQ(mpu.getCondCodeReg()->C, 0);

    mpu.init(v,0);
    mpu.setAccB(0x79);
    mpu.getCondCodeReg()->setCFlag();
    mpu.writeData(0xF1, 0x01);
    //act
    cycles = mpu.execute();
    //assert
    EXPECT_EQ(cycles, 2);
    EXPECT_EQ(mpu.getAccB(), 0x7B);
    EXPECT_EQ(mpu.getCondCodeReg()->Z, 0);
    EXPECT_EQ(mpu.getCondCodeReg()->V, 1);
    EXPECT_EQ(mpu.getCondCodeReg()->N, 0);
    EXPECT_EQ(mpu.getCondCodeReg()->C, 0);
}

