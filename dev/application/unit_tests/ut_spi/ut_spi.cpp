//
// Created by Tianyi Han on 06/16/26.
// ut_spi.cpp
//

#include "ch.hpp"
#include "hal.h"
#include "led.h"
#include "shell.h"

using namespace chibios_rt;

// Union to reconstruct the 32-bit floats from the 16-bit SPI words
// sent by the TI Launchpad.
union float_int {
    uint16_t parts[2];
    float value;
};

// SPI2 Configuration (Slave Mode)
// Matches TI Launchpad: CPOL=0, CPHA=0, 16-bit data frame
static const SPIConfig spi2_slave_cfg = {
    false,                  // circular buffer mode
    nullptr,                // end callback function pointer
    nullptr,                // chip select line port (Handled by Hardware NSS)
    0,                      // chip select line pad number
    SPI_CR1_DFF,            // CR1: 16-bit data frame (DFF=1). CPOL=0, CPHA=0, MSTR=0 (Slave)
    0                       // CR2
};

void forceSpiSlave() {
    // Override ChibiOS's hardcoded Master settings
    SPID2.spi->CR1 &= ~SPI_CR1_SPE;  // Disable SPI
    SPID2.spi->CR1 &= ~SPI_CR1_MSTR; // Clear Master bit (Enable Hardware Slave)
    SPID2.spi->CR1 &= ~SPI_CR1_SSI;  // Clear Internal Slave Select
    SPID2.spi->CR1 &= ~SPI_CR1_SSM;  // Disable Software Slave Select (Force hardware PB12 NSS)
    SPID2.spi->CR1 |= SPI_CR1_SPE;   // Re-enable SPI
}

class SPIRecvThread : public BaseStaticThread<1024> {
    void main() final {
        setName("SPIRecvThd");

        uint16_t rx_buffer[5];

        while(!shouldTerminate()) {
            // Wait to receive 5 words
            spiReceive(&SPID2, 5, rx_buffer);

            // Check for the Sync Word (0xAAAA)
            if (rx_buffer[0] == 0xAAAA) {
                float_int target_x;
                float_int target_y;

                target_x.parts[0] = rx_buffer[1];
                target_x.parts[1] = rx_buffer[2];
                target_y.parts[0] = rx_buffer[3];
                target_y.parts[1] = rx_buffer[4];

                Shell::printf("SPI RX Success | X: %.2f, Y: %.2f" SHELL_NEWLINE_STR,
                              target_x.value, target_y.value);
            } else {
                Shell::printf("SPI Sync Lost! RX[0]=0x%04X" SHELL_NEWLINE_STR, rx_buffer[0]);
                spiStop(&SPID2);
                spiStart(&SPID2, &spi2_slave_cfg);
                forceSpiSlave();
            }

            sleep(TIME_MS2I(5));
        }
    }
} spiRecvThread;

int main(void) {
    halInit();
    System::init();

    Shell::start(HIGHPRIO);

    LED::all_off();
    LED::red_on();

    // Start SPI2 Driver
    spiStart(&SPID2, &spi2_slave_cfg);
    forceSpiSlave();

    // Start the reception thread
    spiRecvThread.start(NORMALPRIO + 5);

#if CH_CFG_NO_IDLE_THREAD
    while (true) {}
#else
    BaseThread::setPriority(1);
#endif
    return 0;
}