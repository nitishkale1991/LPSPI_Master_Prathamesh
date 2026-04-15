/*
 * RT1060 EVKB simple bit-banged TFT bring-up
 *
 * This version stops using LPI2C entirely. Instead, it reuses the easy-to-reach
 * J17/J16 header pins as plain GPIO and bit-bangs a SPI-like write stream to an
 * ILI9341 display module.
 *
 * EVKB -> TFT wiring
 *   J17 pin 1  -> TFT CS
 *   J17 pin 2  -> TFT RESET
 *   J17 pin 7  -> TFT GND
 *   J17 pin 9  -> TFT SDI(MOSI)
 *   J17 pin 10 -> TFT SCK
 *   J16 pin 6  -> TFT DC
 *   3.3V       -> TFT VCC
 *   3.3V       -> TFT LED
 *
 * Leave these unconnected for now
 *   TFT SDO(MISO)
 *   TFT touch pins
 *   TFT SD card pins
 *
 * Header signal map on EVKB
 *   J17 pin 1  = D8  = GPIO_AD_B0_03 = GPIO1_IO03
 *   J17 pin 2  = D9  = GPIO_AD_B0_02 = GPIO1_IO02
 *   J17 pin 9  = D14 = GPIO_AD_B1_01 = GPIO1_IO17
 *   J17 pin 10 = D15 = GPIO_AD_B1_00 = GPIO1_IO16
 *   J16 pin 6  = D5  = GPIO_AD_B0_10 = GPIO1_IO10
 */

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "fsl_iomuxc.h"
#include "board.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define TFT_WIDTH         240U
#define TFT_HEIGHT        320U
#define TFT_SPI_DELAY_US  1U
#define TFT_RESET_DELAY_US 20000U

#define TFT_CS_GPIO    GPIO1
#define TFT_CS_PIN     3U
#define TFT_RESET_GPIO GPIO1
#define TFT_RESET_PIN  2U
#define TFT_DC_GPIO    GPIO1
#define TFT_DC_PIN     10U
#define TFT_MOSI_GPIO  GPIO1
#define TFT_MOSI_PIN   17U
#define TFT_SCK_GPIO   GPIO1
#define TFT_SCK_PIN    16U

static const uint8_t g_colmodData[] = {0x55U};
static const uint8_t g_madctlData[] = {0x48U};
static const uint8_t g_columnAddr[] = {0x00U, 0x00U, 0x00U, 0xEFU};
static const uint8_t g_pageAddr[]   = {0x00U, 0x00U, 0x01U, 0x3FU};

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void BOARD_InitHardware(void);
static void DelayUs(uint32_t us);
static void TftGpioInit(void);
static void TftReset(void);
static void TftWriteByte(uint8_t value);
static void TftWriteCommand(uint8_t command);
static void TftWriteDataBuffer(const uint8_t *data, size_t size);
static void TftWriteCommandWithData(uint8_t command, const uint8_t *data, size_t size);
static void TftSetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
static void TftInitMinimal(void);
static void TftFillScreenGreen(void);

/*******************************************************************************
 * Helpers
 ******************************************************************************/
static void DelayUs(uint32_t us)
{
    SDK_DelayAtLeastUs(us, SystemCoreClock);
}

static void TftCsSet(uint8_t level)
{
    GPIO_WritePinOutput(TFT_CS_GPIO, TFT_CS_PIN, level);
}

static void TftDcSet(uint8_t level)
{
    GPIO_WritePinOutput(TFT_DC_GPIO, TFT_DC_PIN, level);
}

static void TftResetSet(uint8_t level)
{
    GPIO_WritePinOutput(TFT_RESET_GPIO, TFT_RESET_PIN, level);
}

static void TftMosiSet(uint8_t level)
{
    GPIO_WritePinOutput(TFT_MOSI_GPIO, TFT_MOSI_PIN, level);
}

static void TftSckSet(uint8_t level)
{
    GPIO_WritePinOutput(TFT_SCK_GPIO, TFT_SCK_PIN, level);
}

static void TftGpioInit(void)
{
    gpio_pin_config_t outCfg = {
        .direction = kGPIO_DigitalOutput,
        .outputLogic = 1U,
        .interruptMode = kGPIO_NoIntmode,
    };

    CLOCK_EnableClock(kCLOCK_Iomuxc);

    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_03_GPIO1_IO03, 0U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_03_GPIO1_IO03, 0x10B0U);

    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_02_GPIO1_IO02, 0U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_02_GPIO1_IO02, 0x10B0U);

    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_10_GPIO1_IO10, 0U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_10_GPIO1_IO10, 0x10B0U);

    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_01_GPIO1_IO17, 0U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_01_GPIO1_IO17, 0x10B0U);

    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_00_GPIO1_IO16, 0U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_00_GPIO1_IO16, 0x10B0U);

    GPIO_PinInit(TFT_CS_GPIO, TFT_CS_PIN, &outCfg);
    GPIO_PinInit(TFT_RESET_GPIO, TFT_RESET_PIN, &outCfg);
    GPIO_PinInit(TFT_DC_GPIO, TFT_DC_PIN, &outCfg);
    GPIO_PinInit(TFT_MOSI_GPIO, TFT_MOSI_PIN, &outCfg);
    GPIO_PinInit(TFT_SCK_GPIO, TFT_SCK_PIN, &outCfg);

    TftCsSet(1U);
    TftResetSet(1U);
    TftDcSet(1U);
    TftMosiSet(0U);
    TftSckSet(0U);
}

static void TftReset(void)
{
    TftResetSet(1U);
    DelayUs(5000U);
    TftResetSet(0U);
    DelayUs(TFT_RESET_DELAY_US);
    TftResetSet(1U);
    DelayUs(120000U);
}

static void TftWriteByte(uint8_t value)
{
    for (uint8_t mask = 0x80U; mask != 0U; mask >>= 1U)
    {
        TftMosiSet((value & mask) != 0U ? 1U : 0U);
        DelayUs(TFT_SPI_DELAY_US);
        TftSckSet(1U);
        DelayUs(TFT_SPI_DELAY_US);
        TftSckSet(0U);
    }
}

static void TftWriteCommand(uint8_t command)
{
    TftCsSet(0U);
    TftDcSet(0U);
    TftWriteByte(command);
    TftCsSet(1U);
}

static void TftWriteDataBuffer(const uint8_t *data, size_t size)
{
    TftCsSet(0U);
    TftDcSet(1U);

    for (size_t i = 0U; i < size; i++)
    {
        TftWriteByte(data[i]);
    }

    TftCsSet(1U);
}

static void TftWriteCommandWithData(uint8_t command, const uint8_t *data, size_t size)
{
    TftWriteCommand(command);
    if ((data != NULL) && (size != 0U))
    {
        TftWriteDataBuffer(data, size);
    }
}

static void TftSetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t columnData[4];
    uint8_t pageData[4];

    columnData[0] = (uint8_t)(x0 >> 8);
    columnData[1] = (uint8_t)(x0 & 0xFFU);
    columnData[2] = (uint8_t)(x1 >> 8);
    columnData[3] = (uint8_t)(x1 & 0xFFU);

    pageData[0] = (uint8_t)(y0 >> 8);
    pageData[1] = (uint8_t)(y0 & 0xFFU);
    pageData[2] = (uint8_t)(y1 >> 8);
    pageData[3] = (uint8_t)(y1 & 0xFFU);

    TftWriteCommandWithData(0x2AU, columnData, sizeof(columnData));
    TftWriteCommandWithData(0x2BU, pageData, sizeof(pageData));
}

static void TftInitMinimal(void)
{
    TftReset();

    TftWriteCommand(0x01U);
    DelayUs(5000U);

    TftWriteCommand(0x11U);
    DelayUs(120000U);

    TftWriteCommandWithData(0x3AU, g_colmodData, sizeof(g_colmodData));
    DelayUs(1000U);

    TftWriteCommandWithData(0x36U, g_madctlData, sizeof(g_madctlData));
    DelayUs(1000U);

    TftWriteCommandWithData(0x2AU, g_columnAddr, sizeof(g_columnAddr));
    TftWriteCommandWithData(0x2BU, g_pageAddr, sizeof(g_pageAddr));

    TftWriteCommand(0x29U);
    DelayUs(10000U);
}

static void TftFillScreenGreen(void)
{
    //const uint8_t greenHi = 0x07U;
    //const uint8_t greenLo = 0xE0U;
    /*For red color use these values*/
    uint8_t redHi = 0xF8;
    uint8_t redLo = 0x00;

    TftSetAddressWindow(0U, 0U, TFT_WIDTH - 1U, TFT_HEIGHT - 1U);

    TftCsSet(0U);
    TftDcSet(0U);
    TftWriteByte(0x2CU);
    TftDcSet(1U);

    for (uint32_t y = 0U; y < TFT_HEIGHT; y++)
    {
        for (uint32_t x = 0U; x < TFT_WIDTH; x++)
        {
            TftWriteByte(redHi);
            TftWriteByte(redLo);
        }
    }

    TftCsSet(1U);
}

/*******************************************************************************
 * Main
 ******************************************************************************/
int main(void)
{
    BOARD_InitHardware();
    TftGpioInit();

    PRINTF("\r\n=== EVKB Bit-Banged TFT Bring-Up ===\r\n");
    PRINTF("J17.1=CS, J17.2=RESET, J17.9=MOSI, J17.10=SCK, J16.6=DC\r\n");
    PRINTF("Power TFT from 3.3V and GND, tie LED to 3.3V for now.\r\n");
    PRINTF("Initializing ILI9341...\r\n");

    TftInitMinimal();

    PRINTF("Continuously blasting red frames...\r\n");

    while (1)
    {
        TftFillScreenGreen();
    }
}
