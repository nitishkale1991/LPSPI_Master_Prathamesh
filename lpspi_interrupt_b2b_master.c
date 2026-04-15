/*
 * Bit-Banged TFT Green Screen (ILI9341) - RT1060 EVKB
 *
 * ============================================================================
 * WHAT THIS PROGRAM DOES
 * ============================================================================
 * Fills an ILI9341 TFT display (240x320 pixels) with solid green using
 * bit-banged SPI over GPIO pins. No SPI or I2C peripheral hardware is used —
 * we toggle the pins manually in software to simulate the SPI protocol.
 *
 * ============================================================================
 * WHY BIT-BANG INSTEAD OF USING THE SPI PERIPHERAL?
 * ============================================================================
 * The I2C header pins (J17) on the EVKB are the most accessible for wiring.
 * These pins are NOT connected to an SPI peripheral, so we can't use hardware
 * SPI. Instead, we configure them as plain GPIO outputs and "bit-bang" the SPI
 * protocol by toggling the pins in the correct sequence with software.
 *
 * ============================================================================
 * SPI PROTOCOL BASICS (what we're bit-banging)
 * ============================================================================
 * SPI uses 4 signals:
 *   CS   (Chip Select)  - LOW to talk to the display, HIGH when idle
 *   SCK  (Clock)        - We toggle this to clock data in/out
 *   MOSI (Master Out)   - Data line from us to the display
 *   DC   (Data/Command) - LOW = sending a command, HIGH = sending pixel data
 *
 * To send one byte, we shift out 8 bits MSB-first:
 *   For each bit: set MOSI to the bit value, then pulse SCK high->low.
 *   The display reads MOSI on the rising edge of SCK.
 *
 * ============================================================================
 * WIRING (EVKB -> TFT module)
 * ============================================================================
 *   EVKB Pin             RT1060 Pad          GPIO        TFT Signal
 *   -------------------------------------------------------------------
 *   J17 pin 1  (D8)      GPIO_AD_B0_03       GPIO1_IO03  CS
 *   J17 pin 2  (D9)      GPIO_AD_B0_02       GPIO1_IO02  RESET
 *   J17 pin 9  (D14)     GPIO_AD_B1_01       GPIO1_IO17  MOSI (SDI)
 *   J17 pin 10 (D15)     GPIO_AD_B1_00       GPIO1_IO16  SCK
 *   J16 pin 6  (D5)      GPIO_AD_B0_10       GPIO1_IO10  DC
 *   3.3V                 —                   —           VCC + LED
 *   GND (J17 pin 7)      —                   —           GND
 *
 *   Leave unconnected: TFT SDO(MISO), touch pins, SD card pins.
 */

#include "fsl_device_registers.h"   /* RT1060 register definitions        */
#include "fsl_debug_console.h"      /* PRINTF for debug output over UART  */
#include "fsl_gpio.h"               /* GPIO_WritePinOutput, GPIO_PinInit  */
#include "fsl_iomuxc.h"             /* Pin mux and pad configuration      */
#include "board.h"                  /* BOARD_InitHardware (clocks, UART)  */

/*******************************************************************************
 * Screen dimensions (ILI9341 native resolution)
 ******************************************************************************/
#define TFT_WIDTH   240U
#define TFT_HEIGHT  320U

/*******************************************************************************
 * GPIO pin numbers — all five signals are on GPIO1
 ******************************************************************************/
#define PIN_CS    3U    /* J17.1  - Chip Select: LOW = display selected      */
#define PIN_RESET 2U    /* J17.2  - Hardware Reset: pulse LOW to reset       */
#define PIN_DC    10U   /* J16.6  - Data/Command: LOW = command, HIGH = data */
#define PIN_MOSI  17U   /* J17.9  - Master Out Slave In: serial data to TFT  */
#define PIN_SCK   16U   /* J17.10 - Serial Clock: display reads on rising edge */

/*******************************************************************************
 * ILI9341 command codes
 *
 * The ILI9341 accepts commands as single bytes sent with DC=LOW.
 * After a command, optional parameter/data bytes are sent with DC=HIGH.
 * Full command reference: ILI9341 datasheet, Section 8.
 ******************************************************************************/
#define ILI9341_SOFT_RESET     0x01U  /* Reset all registers to defaults     */
#define ILI9341_SLEEP_OUT      0x11U  /* Exit sleep mode (enables display)   */
#define ILI9341_DISPLAY_ON     0x29U  /* Turn on the display output          */
#define ILI9341_COL_ADDR_SET   0x2AU  /* Set column (X) start and end        */
#define ILI9341_PAGE_ADDR_SET  0x2BU  /* Set page (Y) start and end          */
#define ILI9341_MEMORY_WRITE   0x2CU  /* Begin writing pixel data to RAM     */
#define ILI9341_MEM_ACCESS_CTL 0x36U  /* Set row/col scan order and RGB/BGR  */
#define ILI9341_PIXEL_FORMAT   0x3AU  /* Set bits-per-pixel for color data   */

/*******************************************************************************
 * RGB565 color format
 *
 * Each pixel is 16 bits (2 bytes), packed as: RRRRR GGGGGG BBBBB
 *   - 5 bits red   (0-31)
 *   - 6 bits green (0-63)
 *   - 5 bits blue  (0-31)
 *
 * Pure green: R=0, G=63(max), B=0
 *   Binary:  00000 111111 00000  =  0000 0111 1110 0000  =  0x07E0
 *   Sent as: high byte 0x07, then low byte 0xE0
 *
 * Other common colors for reference:
 *   Red:   0xF800 (hi=0xF8, lo=0x00)
 *   Blue:  0x001F (hi=0x00, lo=0x1F)
 *   White: 0xFFFF (hi=0xFF, lo=0xFF)
 *   Black: 0x0000 (hi=0x00, lo=0x00)
 ******************************************************************************/
#define GREEN_BYTE_HI  0x07U
#define GREEN_BYTE_LO  0xE0U

/* Delay between SPI clock edges (microseconds). The ILI9341 supports up to
 * 10 MHz SPI, so 1us per half-clock (~500 KHz) is well within spec. */
#define SPI_BIT_DELAY_US  1U

/*******************************************************************************
 * Forward declaration — defined in board.c, sets up clocks and debug UART
 ******************************************************************************/
void BOARD_InitHardware(void);

/*******************************************************************************
 * DelayUs — blocking microsecond delay using the SDK's cycle-counting helper
 ******************************************************************************/
static void DelayUs(uint32_t us)
{
    SDK_DelayAtLeastUs(us, SystemCoreClock);
}

/*******************************************************************************
 * PinWrite — write a 0 or 1 to one of the GPIO1 pins
 *
 * All five TFT signals (CS, RESET, DC, MOSI, SCK) are on GPIO1,
 * so this single helper covers every pin we need.
 ******************************************************************************/
static void PinWrite(uint32_t pin, uint8_t level)
{
    GPIO_WritePinOutput(GPIO1, pin, level);
}

/*******************************************************************************
 * SpiSendByte — bit-bang one byte over SPI, MSB first
 *
 * SPI Mode 0 (CPOL=0, CPHA=0):
 *   - Clock idles LOW
 *   - Data is sampled on the RISING edge of SCK
 *
 * For each of the 8 bits, starting from bit 7 (MSB) down to bit 0 (LSB):
 *   1. Set MOSI to the current bit value (1 or 0)
 *   2. Wait a short delay for the signal to stabilize
 *   3. Drive SCK HIGH  -> display latches the MOSI bit on this rising edge
 *   4. Wait a short delay
 *   5. Drive SCK LOW   -> back to idle, ready for the next bit
 ******************************************************************************/
static void SpiSendByte(uint8_t byte)
{
    for (uint8_t bit = 0x80U; bit != 0U; bit >>= 1U)
    {
        /* Set MOSI to the current bit value */
        if ((byte & bit) != 0U)
        {
            PinWrite(PIN_MOSI, 1U);
        }
        else
        {
            PinWrite(PIN_MOSI, 0U);
        }

        /* Pulse the clock: rising edge latches data, falling edge resets */
        DelayUs(SPI_BIT_DELAY_US);
        PinWrite(PIN_SCK, 1U);
        DelayUs(SPI_BIT_DELAY_US);
        PinWrite(PIN_SCK, 0U);
    }
}

/*******************************************************************************
 * TftSendCommand — send a command byte to the display
 *
 * The ILI9341 uses the DC (Data/Command) pin to distinguish between
 * commands and data. DC=LOW tells the display "this byte is a command."
 *
 * Sequence: select display -> set DC low -> send byte -> deselect display
 ******************************************************************************/
static void TftSendCommand(uint8_t cmd)
{
    PinWrite(PIN_CS, 0U);    /* Select display (active low) */
    PinWrite(PIN_DC, 0U);    /* DC=0 -> command mode */
    SpiSendByte(cmd);
    PinWrite(PIN_CS, 1U);    /* Deselect display */
}

/*******************************************************************************
 * TftSendData — send one or more data/parameter bytes to the display
 *
 * DC=HIGH tells the display "these bytes are data" (pixel data or command
 * parameters). CS stays low for the entire transfer so the display knows
 * all the bytes belong to the same transaction.
 ******************************************************************************/
static void TftSendData(const uint8_t *data, uint32_t len)
{
    PinWrite(PIN_CS, 0U);    /* Select display */
    PinWrite(PIN_DC, 1U);    /* DC=1 -> data mode */

    for (uint32_t i = 0U; i < len; i++)
    {
        SpiSendByte(data[i]);
    }

    PinWrite(PIN_CS, 1U);    /* Deselect display */
}

/*******************************************************************************
 * TftGpioInit — configure all 5 TFT pins as GPIO outputs
 *
 * Each pin needs two setup steps:
 *   1. IOMUXC: route the physical pad to GPIO mode (mux) and set electrical
 *      properties like drive strength and slew rate (config = 0x10B0).
 *   2. GPIO: initialize the pin as a digital output.
 *
 * Pin config 0x10B0 means:
 *   - Slow slew rate, 100MHz drive, 100k pull-up, pull enabled, open drain off
 *   - This is a safe general-purpose output configuration for GPIO bit-banging
 ******************************************************************************/
static void TftGpioInit(void)
{
    gpio_pin_config_t outputCfg = {
        .direction = kGPIO_DigitalOutput,
        .outputLogic = 1U,
        .interruptMode = kGPIO_NoIntmode,
    };

    /* Enable the IOMUXC clock so we can configure pin muxing */
    CLOCK_EnableClock(kCLOCK_Iomuxc);

    /* Configure pin mux (select GPIO function) and pad settings for each pin */
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_03_GPIO1_IO03, 0U);   /* CS    */
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_03_GPIO1_IO03, 0x10B0U);

    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_02_GPIO1_IO02, 0U);   /* RESET */
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_02_GPIO1_IO02, 0x10B0U);

    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_10_GPIO1_IO10, 0U);   /* DC    */
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_10_GPIO1_IO10, 0x10B0U);

    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_01_GPIO1_IO17, 0U);   /* MOSI  */
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_01_GPIO1_IO17, 0x10B0U);

    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_00_GPIO1_IO16, 0U);   /* SCK   */
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_00_GPIO1_IO16, 0x10B0U);

    /* Initialize all 5 pins as GPIO digital outputs */
    GPIO_PinInit(GPIO1, PIN_CS, &outputCfg);
    GPIO_PinInit(GPIO1, PIN_RESET, &outputCfg);
    GPIO_PinInit(GPIO1, PIN_DC, &outputCfg);
    GPIO_PinInit(GPIO1, PIN_MOSI, &outputCfg);
    GPIO_PinInit(GPIO1, PIN_SCK, &outputCfg);

    /* Set idle state:
     *   CS    = HIGH (display deselected — nothing listens until CS goes low)
     *   RESET = HIGH (not in reset — low would hold display in reset)
     *   DC    = HIGH (data mode — doesn't matter until we start a transfer)
     *   MOSI  = LOW  (no data on the line)
     *   SCK   = LOW  (clock idle low, per SPI Mode 0)
     */
    PinWrite(PIN_CS, 1U);
    PinWrite(PIN_RESET, 1U);
    PinWrite(PIN_DC, 1U);
    PinWrite(PIN_MOSI, 0U);
    PinWrite(PIN_SCK, 0U);
}

/*******************************************************************************
 * TftReset — perform a hardware reset of the display
 *
 * The ILI9341 RESET pin is active-low. Pulling it LOW for at least 10us
 * resets all internal registers. After releasing (HIGH), the datasheet
 * requires 120ms before sending any commands.
 ******************************************************************************/
static void TftReset(void)
{
    PinWrite(PIN_RESET, 1U);    /* Start with RESET high (not in reset) */
    DelayUs(5000U);             /* Brief settle time */

    PinWrite(PIN_RESET, 0U);    /* Assert reset (pull low) */
    DelayUs(20000U);            /* Hold reset for 20ms (datasheet min: 10us) */

    PinWrite(PIN_RESET, 1U);    /* Release reset */
    DelayUs(120000U);           /* Wait 120ms for display to initialize */
}

/*******************************************************************************
 * TftInit — initialize the ILI9341 display for 16-bit color output
 *
 * After reset, the display is in sleep mode with all default settings.
 * We need to configure it step-by-step before we can write pixel data.
 ******************************************************************************/
static void TftInit(void)
{
    /* Step 1: Hardware reset — puts all display registers back to defaults */
    TftReset();

    /* Step 2: Software reset — redundant after hardware reset, but ensures
     *         a clean state even if the hardware reset pin was glitchy */
    TftSendCommand(ILI9341_SOFT_RESET);
    DelayUs(5000U);

    /* Step 3: Exit sleep mode — the display wakes up in sleep mode after reset.
     *         Must wait 120ms after this command before sending more commands. */
    TftSendCommand(ILI9341_SLEEP_OUT);
    DelayUs(120000U);

    /* Step 4: Set pixel format to 16-bit RGB565
     *
     * Parameter 0x55 means:
     *   High nibble (0x5) = 16-bit RGB interface (unused here, but required)
     *   Low nibble  (0x5) = 16-bit MCU interface (this is what SPI uses)
     */
    uint8_t pixelFormat = 0x55U;
    TftSendCommand(ILI9341_PIXEL_FORMAT);
    TftSendData(&pixelFormat, 1U);
    DelayUs(1000U);

    /* Step 5: Set memory access control (scan direction and color order)
     *
     * Parameter 0x48 means:
     *   Bit 6 (0x40) = Column address order: left to right
     *   Bit 3 (0x08) = BGR color order (the ILI9341 panel uses BGR internally)
     *   Result: normal landscape-like scan, colors display correctly
     */
    uint8_t scanDirection = 0x48U;
    TftSendCommand(ILI9341_MEM_ACCESS_CTL);
    TftSendData(&scanDirection, 1U);
    DelayUs(1000U);

    /* Step 6: Define the drawable area (full screen)
     *
     * Column address: 4 bytes = [X_start_hi, X_start_lo, X_end_hi, X_end_lo]
     * Page address:   4 bytes = [Y_start_hi, Y_start_lo, Y_end_hi, Y_end_lo]
     *
     * X range: 0x0000 to 0x00EF = 0 to 239 (240 columns)
     * Y range: 0x0000 to 0x013F = 0 to 319 (320 rows)
     */
    uint8_t colRange[] = {0x00U, 0x00U, 0x00U, 0xEFU};
    TftSendCommand(ILI9341_COL_ADDR_SET);
    TftSendData(colRange, sizeof(colRange));

    uint8_t pageRange[] = {0x00U, 0x00U, 0x01U, 0x3FU};
    TftSendCommand(ILI9341_PAGE_ADDR_SET);
    TftSendData(pageRange, sizeof(pageRange));

    /* Step 7: Turn on the display — pixels in display RAM become visible */
    TftSendCommand(ILI9341_DISPLAY_ON);
    DelayUs(10000U);
}

/*******************************************************************************
 * TftFillScreenGreen — write green to every pixel on the screen
 *
 * The MEMORY_WRITE command (0x2C) tells the display: "the next bytes I send
 * are pixel data." The display auto-increments its internal address after
 * each pixel, so we just stream 76,800 pixels (240 * 320) worth of data
 * without needing to set the address for each one.
 *
 * Important: we keep CS LOW for the entire pixel stream. If we toggled CS
 * between pixels, the display would reset its internal write pointer.
 ******************************************************************************/
static void TftFillScreenGreen(void)
{
    /* Set drawing window to full screen (same as during init) */
    uint8_t colRange[] = {0x00U, 0x00U, 0x00U, 0xEFU};     /* x: 0 to 239 */
    TftSendCommand(ILI9341_COL_ADDR_SET);
    TftSendData(colRange, sizeof(colRange));

    uint8_t pageRange[] = {0x00U, 0x00U, 0x01U, 0x3FU};    /* y: 0 to 319 */
    TftSendCommand(ILI9341_PAGE_ADDR_SET);
    TftSendData(pageRange, sizeof(pageRange));

    /* Send the MEMORY_WRITE command, then switch to data mode and keep
     * CS held low for the entire pixel stream */
    PinWrite(PIN_CS, 0U);                   /* Select display */
    PinWrite(PIN_DC, 0U);                   /* Command mode */
    SpiSendByte(ILI9341_MEMORY_WRITE);      /* "I'm about to send pixels" */
    PinWrite(PIN_DC, 1U);                   /* Switch to data mode */

    /* Stream green to every pixel: 240 x 320 = 76,800 pixels, 2 bytes each */
    for (uint32_t pixel = 0U; pixel < (TFT_WIDTH * TFT_HEIGHT); pixel++)
    {
        SpiSendByte(GREEN_BYTE_HI);         /* High byte of 0x07E0 */
        SpiSendByte(GREEN_BYTE_LO);         /* Low byte of 0x07E0 */
    }

    PinWrite(PIN_CS, 1U);                   /* Deselect display — transfer done */
}

/*******************************************************************************
 * Main — entry point
 *
 * Program flow:
 *   1. Initialize board hardware (clocks, debug UART)
 *   2. Configure GPIO pins for TFT signals
 *   3. Initialize the ILI9341 display
 *   4. Continuously fill the screen with green (infinite loop)
 ******************************************************************************/
int main(void)
{
    /* Set up system clocks and debug console (UART) for PRINTF */
    BOARD_InitHardware();

    /* Configure the 5 GPIO pins we use to talk to the TFT */
    TftGpioInit();

    PRINTF("\r\n=== Bit-Banged TFT Green Screen ===\r\n");
    PRINTF("Wiring: J17.1=CS, J17.2=RESET, J17.9=MOSI, J17.10=SCK, J16.6=DC\r\n");
    PRINTF("Initializing ILI9341...\r\n");

    /* Run the 7-step display initialization sequence */
    TftInit();

    PRINTF("Filling screen with green...\r\n");

    /* Continuously repaint the screen green */
    while (1)
    {
        TftFillScreenGreen();
    }
}
