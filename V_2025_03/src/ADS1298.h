#ifndef _ADS1298_H
#define _ADS1298_H

#include <stdint.h>
#include <stdbool.h>


#define SPI_INSTANCE 2


#define PIN_SCK NRF_GPIO_PIN_MAP(0, 8)
#define PIN_MOSI NRF_GPIO_PIN_MAP(0, 9)
#define PIN_MISO NRF_GPIO_PIN_MAP(0, 24)
#define PIN_CS NRF_GPIO_PIN_MAP(0, 2)

#define PIN_DRDY NRF_GPIO_PIN_MAP(0, 3)

#define COMMAND_WAKEUP 0x02
#define COMMAND_STANDBY 0x04
#define COMMAND_RESET 0x06
#define COMMAND_START 0x08
#define COMMAND_STOP 0x0A
#define COMMAND_RDATA 0x12
#define COMMAND_RDATAC 0x10
#define COMMAND_SDATAC 0x11
#define COMMAND_RREG 0b00100000
#define COMMAND_WREG 0b01000000


// SPI Command Definition Byte Assignments (Datasheet, pg. 35)
#define ADS129X_CMD_WAKEUP  0x02 // Wake-up from standby mode
#define ADS129X_CMD_STANDBY 0x04 // Enter Standby mode
#define ADS129X_CMD_RESET   0x06 // Reset the device
#define ADS129X_CMD_START   0x08 // Start and restart (synchronize) conversions
#define ADS129X_CMD_STOP    0x0A // Stop conversion
#define ADS129X_CMD_RDATAC  0x10 // Enable Read Data Continuous mode (default mode at power-up)
#define ADS129X_CMD_SDATAC  0x11 // Stop Read Data Continuous mode
#define ADS129X_CMD_RDATA   0x12 // Read data by command; supports multiple read back
#define ADS129X_CMD_RREG    0x20 // (also = 00100000) is the first opcode that the address must be added to for RREG communication
#define ADS129X_CMD_WREG    0x40 // 01000000 in binary (Datasheet, pg. 35)



#define ADS129X_REG_ID         0x00 // ID Control Register
#define ADS129X_REG_CONFIG1    0x01 // Configuration Register 1
#define ADS129X_REG_CONFIG2    0x02 // Configuration Register 2
#define ADS129X_REG_CONFIG3    0x03 // Configuration Register 3
#define ADS129X_REG_LOFF       0x04 // Lead-Off Control Register
#define ADS129X_REG_CH1SET     0x05 // Individual Channel Settings 1-8
#define ADS129X_REG_CH2SET     0x06 // ---
#define ADS129X_REG_CH3SET     0x07 // ---
#define ADS129X_REG_CH4SET     0x08 // ---
#define ADS129X_REG_CH5SET     0x09 // ---
#define ADS129X_REG_CH6SET     0x0A // ---
#define ADS129X_REG_CH7SET     0x0B // ---
#define ADS129X_REG_CH8SET     0x0C // ---
#define ADS129X_REG_RLD_SENSP  0x0D // Right Leg Drive, positive side
#define ADS129X_REG_RLD_SENSN  0x0E // Right Leg Drive, negative side
#define ADS129X_REG_LOFF_SENSP 0x0F // Lead-Off Detection, positive side
#define ADS129X_REG_LOFF_SENSN 0x10 // Lead-Off Detection, negative side
#define ADS129X_REG_LOFF_FLIP  0x11 // Lead-Off Detection, current direction
#define ADS129X_REG_LOFF_STATP 0x12 // Electrode Status, positive (read-only)
#define ADS129X_REG_LOFF_STATN 0x13 // Electrode Status, negative (read-only)
#define ADS129X_REG_GPIO       0x14 // General-Purpose I/O Register
#define ADS129X_REG_PACE       0x15 // PACE Detect Register
#define ADS129X_REG_RESP       0x16 // Respiration Control Register
#define ADS129X_REG_CONFIG4    0x17 // Configuration Register 4
#define ADS129X_REG_WCT1       0x18 // Wilson Central Terminal and Augmented Lead Control Register
#define ADS129X_REG_WCT2       0x19 // Wilson Central Terminal Control Register

// IDs
#define ADS129X_ID_ADS1294  0x90
#define ADS129X_ID_ADS1296  0x91
#define ADS129X_ID_ADS1298  0x92
#define ADS129X_ID_ADS1294R 0xD0
#define ADS129X_ID_ADS1296R 0xD1
#define ADS129X_ID_ADS1298R 0xD2

// Configuration Register 1
#define ADS129X_BIT_HR       0x7
#define ADS129X_BIT_DAISY_EN 0x6
#define ADS129X_BIT_CLK_EN   0x5
// always 0
// always 0
#define ADS129X_BIT_DR2      0x2
#define ADS129X_BIT_DR1      0x1
#define ADS129X_BIT_DR0      0x0

// Configuration Register 2
// always 0
// always 0
#define ADS129X_BIT_WCT_CHOP   0x5
#define ADS129X_BIT_INT_TEST   0x4
// always 0
#define ADS129X_BIT_TEST_AMP   0x2
#define ADS129X_BIT_TEST_FREQ1 0x1
#define ADS129X_BIT_TEST_FREQ0 0x0
#define ADS129X_TEST_FREQ_1HZ  0x0
#define ADS129X_TEST_FREQ_2HZ  0x1
#define ADS129X_TEST_FREQ_DC   0x3

// Configuration Register 3
#define ADS129X_BIT_PD_REFBUF     0x7
// always 1
#define ADS129X_BIT_VREF_4V       0x5
#define ADS129X_BIT_RLD_MEAS      0x4
#define ADS129X_BIT_RLDREF_INT    0x3
#define ADS129X_BIT_PD_RLD        0x2
#define ADS129X_BIT_RLD_LOFF_SENS 0x1
#define ADS129X_BIT_RLD_STAT      0x0

// Lead-Off Control Register
#define ADS129X_BIT_COMP_TH2     0x7
#define ADS129X_BIT_COMP_TH1     0x6
#define ADS129X_BIT_COMP_TH0     0x5
#define ADS129X_BIT_VLEAD_OFF_EN 0x4
#define ADS129X_BIT_ILEAD_OFF1   0x3
#define ADS129X_BIT_ILEAD_OFF0   0x2
#define ADS129X_BIT_FLEAD_OFF1   0x1
#define ADS129X_BIT_FLEAD_OFF0   0x0

// Individual Channel Settings
#define ADS129X_BIT_PD    0x7
#define ADS129X_BIT_GAIN2 0x6
#define ADS129X_BIT_GAIN1 0x5
#define ADS129X_BIT_GAIN0 0x4
// always 0
#define ADS129X_BIT_MUX2  0x2
#define ADS129X_BIT_MUX1  0x1
#define ADS129X_BIT_MUX0  0x0

// Channel Select
#define ADS129X_BIT_CH8 0x7
#define ADS129X_BIT_CH7 0x6
#define ADS129X_BIT_CH6 0x5
#define ADS129X_BIT_CH5 0x4
#define ADS129X_BIT_CH4 0x3
#define ADS129X_BIT_CH3 0x2
#define ADS129X_BIT_CH2 0x1
#define ADS129X_BIT_CH1 0x0

// General-Purpose I/O Register
#define ADS129X_BIT_GPIOD4 0x7
#define ADS129X_BIT_GPIOD3 0x6
#define ADS129X_BIT_GPIOD2 0x5
#define ADS129X_BIT_GPIOD1 0x4
#define ADS129X_BIT_GPIOC4 0x3
#define ADS129X_BIT_GPIOC3 0x2
#define ADS129X_BIT_GPIOC2 0x1
#define ADS129X_BIT_GPIOC1 0x0

// PACE Detect Register
// always 0
// always 0
// always 0
#define ADS129X_BIT_PACEE1  0x4
#define ADS129X_BIT_PACEE0  0x3
#define ADS129X_BIT_PACEO1  0x2
#define ADS129X_BIT_PACEO0  0x1
#define ADS129X_BIT_PD_PACE 0x0

// Respiration Control Register
#define ADS129X_BIT_RESP_DEMOD_EN1 0x7
#define ADS129X_BIT_RESP_MOD_EN1   0x6
// always 1
#define ADS129X_BIT_RESP_PH2       0x4
#define ADS129X_BIT_RESP_PH1       0x3
#define ADS129X_BIT_RESP_PH0       0x2
#define ADS129X_BIT_RESP_CTRL1     0x1
#define ADS129X_BIT_RESP_CTRL0     0x0

// Configuration Register 4
#define ADS129X_BIT_RESP_FREQ2   0x7
#define ADS129X_BIT_RESP_FREQ1   0x6
#define ADS129X_BIT_RESP_FREQ0   0x5
// always 0
#define ADS129X_BIT_SINGLE_SHOT  0x3
#define ADS129X_BIT_WCT_TO_RLD   0x2
#define ADS129X_BIT_PD_LOFF_COMP 0x1
// always 0

// Wilson Central Terminal and Augmented Lead Control Register
#define ADS129X_BIT_aVF_CH6 0x7
#define ADS129X_BIT_aVF_CH5 0x6
#define ADS129X_BIT_aVF_CH7 0x5
#define ADS129X_BIT_aVF_CH4 0x4
#define ADS129X_BIT_PD_WCTA 0x3
#define ADS129X_BIT_WCTA2   0x2
#define ADS129X_BIT_WCTA1   0x1
#define ADS129X_BIT_WCTA0   0x0

// Wilson Central Terminal Control Register
#define ADS129X_BIT_PD_WCTC 0x7
#define ADS129X_BIT_PD_WCTB 0x6
#define ADS129X_BIT_WCTB2   0x5
#define ADS129X_BIT_WCTB1   0x4
#define ADS129X_BIT_WCTB0   0x3
#define ADS129X_BIT_WCTC2   0x2
#define ADS129X_BIT_WCTC1   0x1
#define ADS129X_BIT_WCTC0   0x0

// Gain Configuration
#define ADS129X_GAIN_6X     0x0
#define ADS129X_GAIN_1X     0x1
#define ADS129X_GAIN_2X     0x2
#define ADS129X_GAIN_3X     0x3
#define ADS129X_GAIN_4X     0x4
#define ADS129X_GAIN_8X     0x5
#define ADS129X_GAIN_12X    0x6

// Mux Configuration
#define ADS129X_MUX_NORMAL      0x0 // Normal electrode input (default)
#define ADS129X_MUX_SHORT       0x1 // Input shorted (for offset or noise measurements)
#define ADS129X_MUX_RLD_MEAS    0x2 // Used in conjunction with RLD_MEAS bit for RLD measurements
#define ADS129X_MUX_MVDD        0x3 // MVDD for supply measurement
#define ADS129X_MUX_TEMP        0x4 // Temperature sensor
#define ADS129X_MUX_TEST        0x5 // Test signal
#define ADS129X_MUX_RLD_DRP     0x6 // RLD_DRP (positive electrode is the driver)
#define ADS129X_MUX_RLD_DRN     0x7 // RLD_DRN (negative electrode is the driver)

// Sample-rate Configuration
#define ADS129X_SAMPLERATE_LP_250 	0x46
#define ADS129X_SAMPLERATE_LP_500  	0x45
#define ADS129X_SAMPLERATE_LP_1K  	0x44
#define ADS129X_SAMPLERATE_LP_2K  	0x43
#define ADS129X_SAMPLERATE_LP_4K   	0x42
#define ADS129X_SAMPLERATE_LP_8K  	0x41
#define ADS129X_SAMPLERATE_LP_16K       0x40

// channel Info
#define GAIN_ONE                                          0x10
#define GAIN_TWO                                          0x20
#define GAIN_THREE                                        0x30
#define GAIN_FOUR                                         0x40
#define GAIN_SIX                                          0x00
#define GAIN_EIGHT                                        0x50
#define GAIN_TWELVE                                       0x60
#define TURN_OFF_CHANNEL                                  0x80
#define INPUT_SHORT                                       0x01



bool ads1298_write_multiple_register(uint8_t reg, uint8_t* val, uint8_t num);
bool ads1298_write_register(uint8_t reg, uint8_t val);


uint8_t ads1298_read_register_kar(uint8_t reg);
bool ADS1298_send_command(uint8_t command);
bool ADS1298_init();
bool ADS1298_send_start();
bool ADS1298_send_stop();
bool ADS1298_send_reset();
bool ADS1298_send_wakeup();
bool ADS1298_send_read_continuous();
bool ADS1298_send_stop_read_continuous();
bool ADS1298_poll_new_data();
bool get_spi_busy();



typedef struct ADS1298_data
{
    uint8_t lead_off_positive;
    uint8_t lead_off_negative;
    bool gpio_0;
    bool gpio_1;
    bool gpio_2;
    bool gpio_3;
    int32_t channgel_0;
    int32_t channgel_1;
    int32_t channgel_2;
    int32_t channgel_3;
    int32_t channgel_4;
    int32_t channgel_5;
    int32_t channgel_6;
    int32_t channgel_7;
} ADS1298_data_t;


typedef struct data_8chs
{
    int32_t channel_1;
    int32_t channel_2;
    int32_t channel_3;
    int32_t channel_4;
    int32_t channel_5;
    int32_t channel_6;
    int32_t channel_7;
    int32_t channel_8;

} data_8chs_t;


#endif /* _ADS1298_H */