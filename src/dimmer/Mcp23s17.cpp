#include "Mcp23s17.hpp"

#include <string.h>

Mcp23s17::Mcp23s17(int spi2MosiPin, int spi2SclkPin, int spi2CsPin, unsigned int clockFrequency) {
    assert(clockFrequency <= MAX_CLOCK_FREQUENCY_HZ && "MCP23S17 clock frequency must not exceed 10 MHz");

    spi_bus_config_t spi2BusConfig = {.mosi_io_num = spi2MosiPin,
                                      .miso_io_num = -1,  // MISO is not needed for write operations
                                      .sclk_io_num = spi2SclkPin,
                                      .quadwp_io_num = -1,
                                      .quadhd_io_num = -1,
                                      .max_transfer_sz = 0,
                                      .flags = 0};

    // Disable DMA (direct memory access) of transfer data for better performance.
    // This limits the maximum transfer size to 64 bytes which is sufficient for the MCP23S17.
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &spi2BusConfig, SPI_DMA_DISABLED));

    spi_device_interface_config_t spi2DeviceConfig;
    memset(&spi2DeviceConfig, 0, sizeof(spi2DeviceConfig));
    spi2DeviceConfig.spics_io_num = spi2CsPin;
    spi2DeviceConfig.clock_speed_hz = clockFrequency;
    spi2DeviceConfig.mode = 0;
    spi2DeviceConfig.queue_size = 1;
    spi2DeviceConfig.flags = SPI_DEVICE_NO_DUMMY;

    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &spi2DeviceConfig, &spi2DeviceHandle_));

    // Acquiring the bus once here avoids the bus acquisition overhead for individual transmissions.
    // However, wile the bus is acquired, no other device can use it.
    ESP_ERROR_CHECK(spi_device_acquire_bus(spi2DeviceHandle_, portMAX_DELAY));

    enableHardwareAddressing(spi2DeviceHandle_);
    configureAllPinsAsOutputs(spi2DeviceHandle_, 0);
}

void Mcp23s17::enableHardwareAddressing(spi_device_handle_t spiDeviceHandle) {
    // Enable hardware addressing for all MCP23S17 on this bus.
    writeRegister8Bit(spiDeviceHandle, 0, IOCON_REGISTER, 1 << IOCON_HAEN_BIT);

    // Due to a silicon errata, devices with the A2 pin pulled high need to be addressed with
    // the A2 bit set even if hardware addressing is disabled.
    // Since this is the case initially, the dynamic address bits are set to 0b111 (7) for this transmission
    writeRegister8Bit(spiDeviceHandle, 7, IOCON_REGISTER, 1 << IOCON_HAEN_BIT);
}

void Mcp23s17::configureAllPinsAsOutputs(spi_device_handle_t spiDeviceHandle, uint8_t deviceId) {
    // Configure all 16 pins as outputs
    // 0 -> output
    // 1 -> input
    writeRegister16Bit(spiDeviceHandle, deviceId, IODIR_REGISTER, 0x0000);
}

void IRAM_ATTR Mcp23s17::stageChannel(uint16_t channel, bool turnOn) {
    assert(channel < 16 && "Channel must be in the range [0, 15]");

    int channelIndex = channel % 16;

    if (turnOn) {
        stagedChannels_ |= 1 << channelIndex;
    } else {
        stagedChannels_ &= ~(1 << channelIndex);
    }
}

void IRAM_ATTR Mcp23s17::commitStagedChannels() {
    spi_transaction_t spi2Transaction;

    // Write channels 0-15 (MCP23S17 with A0=0 at SPI2)
    if (stagedChannels_ != 0) {
        prepareWriteTransaction16Bit(spi2Transaction, 0, GPIOA_REGISTER, stagedChannels_);
        ESP_ERROR_CHECK(spi_device_polling_transmit(spi2DeviceHandle_, &spi2Transaction));
    }

    stagedChannels_ = 0;
}

uint8_t Mcp23s17::getDeviceAddress(uint8_t deviceId) {
    assert(deviceId <= 7 && "Device ID must not exceed 7 (0b111)");
    return DEVICE_BASE_ADDRESS | deviceId;
}

void Mcp23s17::prepareWriteTransaction16Bit(spi_transaction_t &transaction, uint8_t deviceId, uint8_t registerAddress,
                                            uint16_t value) {
    uint8_t deviceAddress = getDeviceAddress(deviceId);

    memset(&transaction, 0, sizeof(spi_transaction_t));
    // | 0 | 1 | 0 | 0 | A2 | A1 | A0 | R/W |
    // Most significant seven bits are the shifted slave address, least significant bit is R/W select.
    // R/W == 0 -> writing, R/W == 1 -> reading
    transaction.tx_data[0] = deviceAddress << 1;
    transaction.tx_data[1] = registerAddress;
    transaction.tx_data[2] = value;
    transaction.tx_data[3] = value >> 8;
    transaction.length = 32;  // bits
    transaction.flags = SPI_TRANS_USE_TXDATA;
}

void Mcp23s17::writeRegister8Bit(spi_device_handle_t spiDeviceHandle, uint8_t deviceId, uint8_t registerAddress,
                                 uint8_t value) {
    uint8_t deviceAddress = getDeviceAddress(deviceId);

    spi_transaction_t transaction;
    memset(&transaction, 0, sizeof(spi_transaction_t));
    // | 0 | 1 | 0 | 0 | A2 | A1 | A0 | R/W |
    // Most significant seven bits are the shifted slave address, least significant bit is R/W select.
    // R/W == 0 -> writing, R/W == 1 -> reading
    transaction.tx_data[0] = deviceAddress << 1;
    transaction.tx_data[1] = registerAddress;
    transaction.tx_data[2] = value;
    transaction.length = 24;  // bits
    transaction.flags = SPI_TRANS_USE_TXDATA;

    // Use polling instead of interrupt transmission for better performance
    ESP_ERROR_CHECK(spi_device_polling_transmit(spiDeviceHandle, &transaction));
}

void Mcp23s17::writeRegister16Bit(spi_device_handle_t spiDeviceHandle, uint8_t deviceId, uint8_t registerAddress,
                                  uint16_t value) {
    spi_transaction_t transaction;
    prepareWriteTransaction16Bit(transaction, deviceId, registerAddress, value);

    // Use polling instead of interrupt transmission for better performance
    ESP_ERROR_CHECK(spi_device_polling_transmit(spiDeviceHandle, &transaction));
}
