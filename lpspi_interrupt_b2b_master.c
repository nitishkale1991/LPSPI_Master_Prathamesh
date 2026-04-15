/*
 * RT1060 EVKB J17 I2C-header GPIO wiggle test
 *
 * This is a simple physical-connectivity test for the Arduino header pins
 * that are easier to probe than the SPI-routed J17 pins used earlier.
 *
 * Wiring to probe on the EVKB:
 *   J17 pin 7  -> GND
 *   J17 pin 9  -> D14 / I2C_SDA -> GPIO_AD_B1_01 -> GPIO1_IO17
 *   J17 pin 10 -> D15 / I2C_SCL -> GPIO_AD_B1_00 -> GPIO1_IO16
 *   J16 pin 6  -> D5 / DC       -> GPIO_AD_B0_10 -> GPIO1_IO10
 *
 * Helpful EVKB schematic note:
 *   J17 pin 9 reaches the MCU through R54.
 *   J17 pin 10 reaches the MCU through R95.
 */

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "fsl_iomuxc.h"
#include "board.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define J17_SDA_GPIO GPIO1
#define J17_SDA_PIN  17U

#define J17_SCL_GPIO GPIO1
#define J17_SCL_PIN  16U

#define J16_DC_GPIO GPIO1
#define J16_DC_PIN  10U

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void BOARD_InitHardware(void);
static void DelayUs(uint32_t us);
static void Board_InitJ17I2cWigglePins(void);
static void RunJ17I2cGpioWiggleTest(void);

/*******************************************************************************
 * Helpers
 ******************************************************************************/
static void DelayUs(uint32_t us)
{
    SDK_DelayAtLeastUs(us, SystemCoreClock);
}

static void Board_InitJ17I2cWigglePins(void)
{
    gpio_pin_config_t outCfg = {
        .direction = kGPIO_DigitalOutput,
        .outputLogic = 0U,
        .interruptMode = kGPIO_NoIntmode,
    };

    CLOCK_EnableClock(kCLOCK_Iomuxc);

    /* J17 pin 10 = D15 / I2C_SCL = GPIO_AD_B1_00 = GPIO1_IO16 */
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_00_GPIO1_IO16, 0U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_00_GPIO1_IO16, 0x10B0U);

    /* J17 pin 9 = D14 / I2C_SDA = GPIO_AD_B1_01 = GPIO1_IO17 */
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_01_GPIO1_IO17, 0U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_01_GPIO1_IO17, 0x10B0U);

    /* J16 pin 6 = D5 / DC = GPIO_AD_B0_10 = GPIO1_IO10 */
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_10_GPIO1_IO10, 0U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_10_GPIO1_IO10, 0x10B0U);

    GPIO_PinInit(J17_SCL_GPIO, J17_SCL_PIN, &outCfg);
    GPIO_PinInit(J17_SDA_GPIO, J17_SDA_PIN, &outCfg);
    GPIO_PinInit(J16_DC_GPIO, J16_DC_PIN, &outCfg);
}

static void RunJ17I2cGpioWiggleTest(void)
{
    PRINTF("\r\n=== J17/J16 GPIO Wiggle Test ===\r\n");
    PRINTF("Probe J17 pin 9 = SDA, J17 pin 10 = SCL, J16 pin 6 = DC, J17 pin 7 = GND.\r\n");
    PRINTF("This is a GPIO wiggle test, not an I2C peripheral transfer.\r\n");
    PRINTF("If J17 stays flat, check EVKB continuity through R54 and R95.\r\n");

    while (1)
    {
        /* Baseline low window. */
        GPIO_WritePinOutput(J17_SDA_GPIO, J17_SDA_PIN, 0U);
        GPIO_WritePinOutput(J17_SCL_GPIO, J17_SCL_PIN, 0U);
        GPIO_WritePinOutput(J16_DC_GPIO, J16_DC_PIN, 0U);
        DelayUs(300000U);

        /* Distinct SDA pulse so it is easy to identify. */
        GPIO_WritePinOutput(J17_SDA_GPIO, J17_SDA_PIN, 1U);
        DelayUs(500000U);
        GPIO_WritePinOutput(J17_SDA_GPIO, J17_SDA_PIN, 0U);
        DelayUs(250000U);

        /* Distinct SCL pulse burst so it looks different from SDA. */
        for (uint32_t i = 0U; i < 16U; i++)
        {
            GPIO_WritePinOutput(J17_SCL_GPIO, J17_SCL_PIN, 1U);
            DelayUs(50000U);
            GPIO_WritePinOutput(J17_SCL_GPIO, J17_SCL_PIN, 0U);
            DelayUs(50000U);
        }

        /* Distinct DC pulse so J16 can be validated independently. */
        GPIO_WritePinOutput(J16_DC_GPIO, J16_DC_PIN, 1U);
        DelayUs(350000U);
        GPIO_WritePinOutput(J16_DC_GPIO, J16_DC_PIN, 0U);
        DelayUs(200000U);

        /* Combined high window to prove all lines can drive high. */
        GPIO_WritePinOutput(J17_SDA_GPIO, J17_SDA_PIN, 1U);
        GPIO_WritePinOutput(J17_SCL_GPIO, J17_SCL_PIN, 1U);
        GPIO_WritePinOutput(J16_DC_GPIO, J16_DC_PIN, 1U);
        DelayUs(200000U);
        GPIO_WritePinOutput(J17_SDA_GPIO, J17_SDA_PIN, 0U);
        GPIO_WritePinOutput(J17_SCL_GPIO, J17_SCL_PIN, 0U);
        GPIO_WritePinOutput(J16_DC_GPIO, J16_DC_PIN, 0U);

        DelayUs(500000U);
    }
}

/*******************************************************************************
 * Main
 ******************************************************************************/
int main(void)
{
    BOARD_InitHardware();

    Board_InitJ17I2cWigglePins();
    RunJ17I2cGpioWiggleTest();

    while (1)
    {
    }
}
