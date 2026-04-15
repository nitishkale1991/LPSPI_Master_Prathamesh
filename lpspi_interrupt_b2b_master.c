/*
 * RT1060 EVKB J17 LPI2C interrupt-based validation transfer
 *
 * Step 1 packet-emulation build:
 *   Replay an ILI9341-style init + red-fill packet script over LPI2C using the
 *   interrupt-driven API, purely so we can observe the command/data framing on
 *   the Saleae.
 *
 * Wiring to probe on the EVKB:
 *   J17 pin 7  -> GND
 *   J17 pin 9  -> D14 / I2C_SDA -> GPIO_AD_B1_01 -> LPI2C1_SDA
 *   J17 pin 10 -> D15 / I2C_SCL -> GPIO_AD_B1_00 -> LPI2C1_SCL
 *   J16 pin 6  -> D5 / DC       -> GPIO_AD_B0_10 -> GPIO1_IO10
 *
 * Helpful EVKB schematic note:
 *   J17 pin 9 reaches the MCU through R54.
 *   J17 pin 10 reaches the MCU through R95.
 *
 * Important note:
 *   ILI9341 is a SPI display, not an I2C display. This file does NOT drive the
 *   panel directly. It only re-creates the packet sequence we would want on the
 *   SPI side, while sending it out through interrupt-driven LPI2C so the bus
 *   activity is easy to validate on accessible EVKB header pins.
 *
 * Transport note:
 *   We intentionally set ignoreAck = true for this validation step so the
 *   master clocks out the full write frame even when no real I2C slave is
 *   attached yet. That makes the Saleae capture much easier to verify.
 */

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "fsl_iomuxc.h"
#include "fsl_lpi2c.h"
#include "fsl_clock.h"
#include "board.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define TEST_LPI2C_MASTER_BASEADDR LPI2C1
#define TEST_LPI2C_BAUDRATE        100000U
#define TEST_LPI2C_TARGET_ADDR     0x3CU
#define TFT_WIDTH                  240U
#define TFT_HEIGHT                 320U
#define TFT_RED_BURST_LINES        8U

/* Select USB1 PLL (480 MHz) as LPI2C clock source, divided as in the SDK example. */
#define TEST_LPI2C_CLOCK_SOURCE_SELECT  (0U)
#define TEST_LPI2C_CLOCK_SOURCE_DIVIDER (5U)
#define TEST_LPI2C_CLOCK_FREQUENCY \
    ((CLOCK_GetFreq(kCLOCK_Usb1PllClk) / 8U) / (TEST_LPI2C_CLOCK_SOURCE_DIVIDER + 1U))

#define J16_DC_GPIO GPIO1
#define J16_DC_PIN  10U

typedef struct _display_packet
{
    const char *name;
    uint8_t dcLevel;
    const uint8_t *data;
    size_t dataSize;
    uint32_t postDelayUs;
} display_packet_t;

static lpi2c_master_handle_t g_lpi2cHandle;
static volatile bool g_lpi2cTransferComplete = false;
static volatile status_t g_lpi2cTransferStatus = kStatus_Success;
static volatile uint32_t g_transferCount = 0U;

static const uint8_t g_cmdSoftwareReset[] = {0x01U};
static const uint8_t g_cmdSleepOut[]      = {0x11U};
static const uint8_t g_cmdPixelFormat[]   = {0x3AU};
static const uint8_t g_dataPixelFormat[]  = {0x55U};
static const uint8_t g_cmdMadctl[]        = {0x36U};
static const uint8_t g_dataMadctl[]       = {0x48U};
static const uint8_t g_cmdDisplayOn[]     = {0x29U};
static const uint8_t g_cmdColumnAddr[]    = {0x2AU};
static const uint8_t g_dataColumnAddr[]   = {0x00U, 0x00U, 0x00U, 0xEFU};
static const uint8_t g_cmdPageAddr[]      = {0x2BU};
static const uint8_t g_dataPageAddr[]     = {0x00U, 0x00U, 0x01U, 0x3FU};
static const uint8_t g_cmdMemoryWrite[]   = {0x2CU};
static uint8_t g_redLineData[TFT_WIDTH * 2U];

static const display_packet_t g_displayInitPackets[] = {
    {"SWRESET", 0U, g_cmdSoftwareReset, sizeof(g_cmdSoftwareReset), 5000U},
    {"SLPOUT", 0U, g_cmdSleepOut, sizeof(g_cmdSleepOut), 120000U},
    {"COLMOD", 0U, g_cmdPixelFormat, sizeof(g_cmdPixelFormat), 0U},
    {"COLMOD_DATA", 1U, g_dataPixelFormat, sizeof(g_dataPixelFormat), 1000U},
    {"MADCTL", 0U, g_cmdMadctl, sizeof(g_cmdMadctl), 0U},
    {"MADCTL_DATA", 1U, g_dataMadctl, sizeof(g_dataMadctl), 1000U},
    {"DISPON", 0U, g_cmdDisplayOn, sizeof(g_cmdDisplayOn), 10000U},
    {"CASET", 0U, g_cmdColumnAddr, sizeof(g_cmdColumnAddr), 0U},
    {"CASET_DATA", 1U, g_dataColumnAddr, sizeof(g_dataColumnAddr), 0U},
    {"PASET", 0U, g_cmdPageAddr, sizeof(g_cmdPageAddr), 0U},
    {"PASET_DATA", 1U, g_dataPageAddr, sizeof(g_dataPageAddr), 0U},
    {"RAMWR", 0U, g_cmdMemoryWrite, sizeof(g_cmdMemoryWrite), 0U},
};

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void BOARD_InitHardware(void);
static void DelayUs(uint32_t us);
static void Board_InitValidationPins(void);
static void Board_InitValidationLpi2c(void);
static void PrepareDisplayPacketData(void);
static status_t StartValidationTransfer(const uint8_t *data, size_t dataSize);
static status_t SendDisplayPacket(const display_packet_t *packet);
static void lpi2c_master_callback(LPI2C_Type *base,
                                  lpi2c_master_handle_t *handle,
                                  status_t completionStatus,
                                  void *userData);

/*******************************************************************************
 * Helpers
 ******************************************************************************/
static void DelayUs(uint32_t us)
{
    SDK_DelayAtLeastUs(us, SystemCoreClock);
}

static void Board_InitValidationPins(void)
{
    gpio_pin_config_t outCfg = {
        .direction = kGPIO_DigitalOutput,
        .outputLogic = 0U,
        .interruptMode = kGPIO_NoIntmode,
    };

    CLOCK_EnableClock(kCLOCK_Iomuxc);

    /* J17 pin 10 = D15 / I2C_SCL = GPIO_AD_B1_00 = LPI2C1_SCL */
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_00_LPI2C1_SCL, 1U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_00_LPI2C1_SCL, 0xD8B0U);

    /* J17 pin 9 = D14 / I2C_SDA = GPIO_AD_B1_01 = LPI2C1_SDA */
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_01_LPI2C1_SDA, 1U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_01_LPI2C1_SDA, 0xD8B0U);

    /* J16 pin 6 = D5 / DC = GPIO_AD_B0_10 = GPIO1_IO10 */
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_10_GPIO1_IO10, 0U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_10_GPIO1_IO10, 0x10B0U);

    GPIO_PinInit(J16_DC_GPIO, J16_DC_PIN, &outCfg);
}

static void Board_InitValidationLpi2c(void)
{
    lpi2c_master_config_t masterConfig;

    /* Match the EVKB SDK examples for LPI2C1 clocking. */
    CLOCK_SetMux(kCLOCK_Lpi2cMux, TEST_LPI2C_CLOCK_SOURCE_SELECT);
    CLOCK_SetDiv(kCLOCK_Lpi2cDiv, TEST_LPI2C_CLOCK_SOURCE_DIVIDER);

    LPI2C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Hz = TEST_LPI2C_BAUDRATE;
    masterConfig.pinConfig = kLPI2C_2PinOpenDrain;
    masterConfig.ignoreAck = true;

    LPI2C_MasterInit(TEST_LPI2C_MASTER_BASEADDR, &masterConfig, TEST_LPI2C_CLOCK_FREQUENCY);
    LPI2C_MasterTransferCreateHandle(TEST_LPI2C_MASTER_BASEADDR, &g_lpi2cHandle, lpi2c_master_callback, NULL);
}

static void PrepareDisplayPacketData(void)
{
    for (uint32_t x = 0U; x < TFT_WIDTH; x++)
    {
        g_redLineData[2U * x] = 0xF8U;
        g_redLineData[(2U * x) + 1U] = 0x00U;
    }
}

static void lpi2c_master_callback(LPI2C_Type *base,
                                  lpi2c_master_handle_t *handle,
                                  status_t completionStatus,
                                  void *userData)
{
    (void)base;
    (void)handle;
    (void)userData;

    g_lpi2cTransferStatus = completionStatus;
    g_lpi2cTransferComplete = true;
}

static status_t StartValidationTransfer(const uint8_t *data, size_t dataSize)
{
    lpi2c_master_transfer_t masterXfer = {0};

    g_lpi2cTransferComplete = false;
    g_lpi2cTransferStatus = kStatus_Success;

    masterXfer.slaveAddress = TEST_LPI2C_TARGET_ADDR;
    masterXfer.direction = kLPI2C_Write;
    masterXfer.subaddress = 0U;
    masterXfer.subaddressSize = 0U;
    masterXfer.data = (void *)data;
    masterXfer.dataSize = dataSize;
    masterXfer.flags = kLPI2C_TransferDefaultFlag;

    return LPI2C_MasterTransferNonBlocking(TEST_LPI2C_MASTER_BASEADDR, &g_lpi2cHandle, &masterXfer);
}

static status_t SendDisplayPacket(const display_packet_t *packet)
{
    status_t status;

    GPIO_WritePinOutput(J16_DC_GPIO, J16_DC_PIN, packet->dcLevel);

    status = StartValidationTransfer(packet->data, packet->dataSize);
    if (status != kStatus_Success)
    {
        return status;
    }

    while (!g_lpi2cTransferComplete)
    {
    }

    if (packet->postDelayUs != 0U)
    {
        DelayUs(packet->postDelayUs);
    }

    return g_lpi2cTransferStatus;
}

/*******************************************************************************
 * Main
 ******************************************************************************/
int main(void)
{
    status_t status;

    BOARD_InitHardware();

    Board_InitValidationPins();
    Board_InitValidationLpi2c();
    PrepareDisplayPacketData();

    PRINTF("\r\n=== Step 1: LPI2C Interrupt TFT Packet Emulation ===\r\n");
    PRINTF("Probe J17 pin 9 = SDA, J17 pin 10 = SCL, J16 pin 6 = DC, J17 pin 7 = GND.\r\n");
    PRINTF("LPI2C1 writes repeatedly at %u Hz to 0x%02X using interrupt-based transfers.\r\n",
           TEST_LPI2C_BAUDRATE,
           TEST_LPI2C_TARGET_ADDR);
    PRINTF("This replays ILI9341-style command/data frames only; it does not drive the SPI TFT directly.\r\n");
    PRINTF("ignoreAck=true is enabled so Saleae sees full frames without a real I2C slave.\r\n");

    while (1)
    {
        for (uint32_t i = 0U; i < (sizeof(g_displayInitPackets) / sizeof(g_displayInitPackets[0])); i++)
        {
            status = SendDisplayPacket(&g_displayInitPackets[i]);
            if (status != kStatus_Success)
            {
                PRINTF("Packet %s failed with status %d\r\n", g_displayInitPackets[i].name, (int)status);
                break;
            }
            g_transferCount++;
        }

        if (status != kStatus_Success)
        {
            DelayUs(500000U);
            continue;
        }

        for (uint32_t line = 0U; line < TFT_RED_BURST_LINES; line++)
        {
            display_packet_t redPacket = {
                .name = "RED_LINE_DATA",
                .dcLevel = 1U,
                .data = g_redLineData,
                .dataSize = sizeof(g_redLineData),
                .postDelayUs = 0U,
            };

            status = SendDisplayPacket(&redPacket);
            if (status != kStatus_Success)
            {
                PRINTF("Red line %lu failed with status %d\r\n", (unsigned long)line, (int)status);
                break;
            }
            g_transferCount++;
        }

        DelayUs(500000U);
    }
}
