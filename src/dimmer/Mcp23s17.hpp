#pragma once

#include <driver/spi_master.h>

#include <array>

class Mcp23s17 {
 public:
    Mcp23s17(int spi2MosiPin = 13, int spi2SclkPin = 14, int spi2CsPin = 15,
             unsigned int clockFrequency = SPI_MASTER_FREQ_10M);

    void stageChannel(uint16_t channel, bool turnOn);
    void commitStagedChannels();

 private:
    static const int MAX_CLOCK_FREQUENCY_HZ = 10 * 1000 * 1000;
    static const int DEVICE_BASE_ADDRESS = 0b00100000;

    static const uint8_t IODIR_REGISTER = 0x00;
    static const uint8_t GPIOA_REGISTER = 0x12;
    static const uint8_t IOCON_REGISTER = 0x0A;

    static const uint8_t IOCON_HAEN_BIT = 3;

    uint16_t stagedChannels_;

    spi_device_handle_t spi2DeviceHandle_;

    uint8_t getDeviceAddress(uint8_t deviceId);

    void writeRegister8Bit(spi_device_handle_t spiDeviceHandle, uint8_t deviceId, uint8_t registerAddress,
                           uint8_t value);

    void writeRegister16Bit(spi_device_handle_t spiDeviceHandle, uint8_t deviceId, uint8_t registerAddress,
                            uint16_t value);

    void prepareWriteTransaction16Bit(spi_transaction_t &transaction, uint8_t deviceId, uint8_t registerAddress,
                                      uint16_t value);

    void configureAllPinsAsOutputs(spi_device_handle_t spiDeviceHandle, uint8_t deviceId);

    void enableHardwareAddressing(spi_device_handle_t spiDeviceHandle);
};