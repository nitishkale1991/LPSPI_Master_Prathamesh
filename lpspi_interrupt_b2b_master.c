/*
 * RT1060 EVKB -> ILI9341 TFT Display (SPI, Interrupt-Driven)
 *
 * Wiring (EVKB Arduino headers):
 *   TFT VCC   -> J32 pin 4  (3.3V)
 *   TFT GND   -> J32 pin 7  (GND)
 *   TFT CS    -> J17 pin 3  (LPSPI1_PCS0 / GPIO_SD_B0_01)
 *   TFT MOSI  -> J17 pin 4  (LPSPI1_SDO  / GPIO_SD_B0_02)
 *   TFT SCK   -> J17 pin 6  (LPSPI1_SCK  / GPIO_SD_B0_00)
 *   TFT DC    -> J16 pin 6  (GPIO1_IO10  / GPIO_AD_B0_10)
 *   TFT RESET -> J16 pin 7  (GPIO1_IO18  / GPIO_AD_B1_02)
 *   TFT LED   -> J32 pin 4  (3.3V)
 *
 * Transfer model:
 *   Main context kicks off a transfer by enabling the TX interrupt.
 *   The ISR fills the FIFO in chunks each time it fires.  After the
 *   last byte is queued the ISR writes PCS-uncontinuous into the FIFO
 *   and disables itself.  Main polls the Transfer Complete flag to
 *   confirm the last byte has physically shifted out.
 *
 *   NOSTALL = 1 so we never need to drain the RX FIFO.  The master
 *   ignores RX overrun — safe because this is a TX-only display link.
 */

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_lpspi.h"
#include "fsl_gpio.h"
#include "fsl_iomuxc.h"
#include "board.h"
#include "app.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define TFT_WIDTH          240U
#define TFT_HEIGHT         320U
#define TFT_SPI_BAUDRATE   2000000U  /* 2 MHz */

/* DC pin: J16 pin 6 = GPIO_AD_B0_10 = GPIO1_IO10 */
#define DC_GPIO    GPIO1
#define DC_PIN     10U

/* RESET pin: J16 pin 7 = GPIO_AD_B1_02 = GPIO1_IO18 */
#define RESET_GPIO GPIO1
#define RESET_PIN  18U

/*******************************************************************************
 * ISR shared state
 ******************************************************************************/
static const uint8_t *g_txBuffer;
static volatile uint32_t g_txSize;
static volatile uint32_t g_txIndex;
static volatile bool     g_transferDone;
static uint8_t           g_fifoSize;  /* set once in init, read by ISR */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void DelayUs(uint32_t us);
static void Board_InitDcResetPins(void);
static void Board_InitLpspi(void);
static void LPSPI_SendBuffer(const uint8_t *data, uint32_t size);
static void Display_Reset(void);
static void Display_SendCommand(uint8_t cmd);
static void Display_SendData(const uint8_t *data, uint32_t len);
static void Display_SetAddressWindow(uint16_t x0, uint16_t y0,
                                     uint16_t x1, uint16_t y1);
static void Display_Init(void);
static void Display_FillScreen(uint16_t color);

/*******************************************************************************
 * Delay helper
 ******************************************************************************/
static void DelayUs(uint32_t us)
{
    SDK_DelayAtLeastUs(us, SystemCoreClock);
}

/*******************************************************************************
 * ISR: TX Data Request — fills the FIFO in chunks
 *
 * Fires when the TX FIFO is empty (default watermark = 0).
 * Each entry: push up to g_fifoSize bytes.
 * After the last byte: write PCS-uncontinuous into the FIFO so
 * hardware de-asserts CS, disable the interrupt, signal main.
 ******************************************************************************/
void EXAMPLE_LPSPI_MASTER_IRQHandler(void)
{
    /* Fill FIFO until full or all bytes queued */
    while ((LPSPI_GetTxFifoCount(EXAMPLE_LPSPI_MASTER_BASEADDR) < g_fifoSize) &&
           (g_txIndex < g_txSize))
    {
        LPSPI_WriteData(EXAMPLE_LPSPI_MASTER_BASEADDR, g_txBuffer[g_txIndex]);
        g_txIndex++;
    }

    /* All bytes queued — tell hardware to release CS after last byte */
    if (g_txIndex >= g_txSize)
    {
        /* TCR shares the FIFO — wait for one slot */
        while (LPSPI_GetTxFifoCount(EXAMPLE_LPSPI_MASTER_BASEADDR) ==
               g_fifoSize)
        {
        }
        LPSPI_SetPCSContinous(EXAMPLE_LPSPI_MASTER_BASEADDR, false);
        LPSPI_DisableInterrupts(EXAMPLE_LPSPI_MASTER_BASEADDR,
                                kLPSPI_TxInterruptEnable);
        g_transferDone = true;
    }

    SDK_ISR_EXIT_BARRIER;
}

/*******************************************************************************
 * SPI send — main context kicks off, ISR does the work
 *
 *   1. Set up ISR state (buffer pointer, size, index = 0)
 *   2. Flush FIFO, clear flags, set PCS continuous
 *   3. Enable TX interrupt — ISR fires immediately (FIFO is empty)
 *   4. Spin until ISR signals all bytes queued
 *   5. Poll Transfer Complete flag — last byte physically shifted out
 ******************************************************************************/
static void LPSPI_SendBuffer(const uint8_t *data, uint32_t size)
{
    if ((data == NULL) || (size == 0U))
    {
        return;
    }

    /* Set up ISR state */
    g_txBuffer     = data;
    g_txSize       = size;
    g_txIndex      = 0U;
    g_transferDone = false;

    /* Clean slate */
    LPSPI_FlushFifo(EXAMPLE_LPSPI_MASTER_BASEADDR, true, true);
    LPSPI_ClearStatusFlags(EXAMPLE_LPSPI_MASTER_BASEADDR, kLPSPI_AllStatusFlag);
    LPSPI_DisableInterrupts(EXAMPLE_LPSPI_MASTER_BASEADDR,
                            kLPSPI_AllInterruptEnable);

    /* PCS0 held low across all bytes in this transfer */
    LPSPI_SelectTransferPCS(EXAMPLE_LPSPI_MASTER_BASEADDR,
                            EXAMPLE_LPSPI_MASTER_PCS_FOR_INIT);
    LPSPI_SetPCSContinous(EXAMPLE_LPSPI_MASTER_BASEADDR, true);

    /* Wait for TCR written (TCR shares the TX FIFO) */
    while (LPSPI_GetTxFifoCount(EXAMPLE_LPSPI_MASTER_BASEADDR) != 0U)
    {
    }

    /* Kick off — ISR fires immediately because FIFO is empty */
    LPSPI_EnableInterrupts(EXAMPLE_LPSPI_MASTER_BASEADDR,
                           kLPSPI_TxInterruptEnable);

    /* Wait for ISR to queue every byte and write PCS-uncontinuous */
    while (!g_transferDone)
    {
    }

    /* Wait for last byte to physically shift out on the wire */
    while ((LPSPI_GetStatusFlags(EXAMPLE_LPSPI_MASTER_BASEADDR) &
            kLPSPI_TransferCompleteFlag) == 0U)
    {
    }
    LPSPI_ClearStatusFlags(EXAMPLE_LPSPI_MASTER_BASEADDR,
                           kLPSPI_TransferCompleteFlag);
}

/*******************************************************************************
 * Board pin init: DC and RESET GPIOs only
 *
 * LPSPI1 SPI pins are already configured by BOARD_InitHardware().
 ******************************************************************************/
static void Board_InitDcResetPins(void)
{
    gpio_pin_config_t outCfg = {
        .direction     = kGPIO_DigitalOutput,
        .outputLogic   = 1U,
        .interruptMode = kGPIO_NoIntmode
    };

    /* DC: GPIO_AD_B0_10 -> GPIO1_IO10 */
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_10_GPIO1_IO10, 0U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_10_GPIO1_IO10, 0x10B0U);
    GPIO_PinInit(DC_GPIO, DC_PIN, &outCfg);

    /* RESET: GPIO_AD_B1_02 -> GPIO1_IO18 */
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_02_GPIO1_IO18, 0U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_02_GPIO1_IO18, 0x10B0U);
    GPIO_PinInit(RESET_GPIO, RESET_PIN, &outCfg);
}

/*******************************************************************************
 * LPSPI1 master config for TFT
 *
 * Mode 0 (CPOL=0, CPHA=0), 8-bit, MSB-first, PCS0 active-low.
 * NOSTALL = 1 so we never stall on RX FIFO full (TX-only link).
 ******************************************************************************/
static void Board_InitLpspi(void)
{
    lpspi_master_config_t cfg;

    LPSPI_MasterGetDefaultConfig(&cfg);

    cfg.baudRate                      = TFT_SPI_BAUDRATE;
    cfg.bitsPerFrame                  = 8U;
    cfg.cpol                          = kLPSPI_ClockPolarityActiveHigh;
    cfg.cpha                          = kLPSPI_ClockPhaseFirstEdge;
    cfg.direction                     = kLPSPI_MsbFirst;
    cfg.whichPcs                      = EXAMPLE_LPSPI_MASTER_PCS_FOR_INIT;
    cfg.pcsActiveHighOrLow            = kLPSPI_PcsActiveLow;
    cfg.pinCfg                        = kLPSPI_SdiInSdoOut;
    cfg.dataOutConfig                 = kLpspiDataOutRetained;
    cfg.pcsToSckDelayInNanoSec        = 1000000000U / (cfg.baudRate * 2U);
    cfg.lastSckToPcsDelayInNanoSec    = 1000000000U / (cfg.baudRate * 2U);
    cfg.betweenTransferDelayInNanoSec = 1000000000U / (cfg.baudRate * 2U);

    LPSPI_MasterInit(EXAMPLE_LPSPI_MASTER_BASEADDR, &cfg, LPSPI_MASTER_CLK_FREQ);

    /* NOSTALL = 1: master ignores RX FIFO overrun (TX-only, no RX drain) */
    LPSPI_Enable(EXAMPLE_LPSPI_MASTER_BASEADDR, false);
    EXAMPLE_LPSPI_MASTER_BASEADDR->CFGR1 |= LPSPI_CFGR1_NOSTALL_MASK;
    LPSPI_Enable(EXAMPLE_LPSPI_MASTER_BASEADDR, true);

    /* Cache FIFO depth for the ISR */
    g_fifoSize = LPSPI_GetRxFifoSize(EXAMPLE_LPSPI_MASTER_BASEADDR);

    NVIC_SetPriority(EXAMPLE_LPSPI_MASTER_IRQN, 3U);
    EnableIRQ(EXAMPLE_LPSPI_MASTER_IRQN);
}

/*******************************************************************************
 * Display primitives
 ******************************************************************************/
static void Display_Reset(void)
{
    GPIO_WritePinOutput(RESET_GPIO, RESET_PIN, 0U);
    DelayUs(10000U);
    GPIO_WritePinOutput(RESET_GPIO, RESET_PIN, 1U);
    DelayUs(120000U);
}

static void Display_SendCommand(uint8_t cmd)
{
    GPIO_WritePinOutput(DC_GPIO, DC_PIN, 0U);
    LPSPI_SendBuffer(&cmd, 1U);
}

static void Display_SendData(const uint8_t *data, uint32_t len)
{
    GPIO_WritePinOutput(DC_GPIO, DC_PIN, 1U);
    LPSPI_SendBuffer(data, len);
}

static void Display_SetAddressWindow(uint16_t x0, uint16_t y0,
                                     uint16_t x1, uint16_t y1)
{
    uint8_t col[4];
    uint8_t row[4];

    col[0] = (uint8_t)(x0 >> 8);
    col[1] = (uint8_t)(x0 & 0xFFU);
    col[2] = (uint8_t)(x1 >> 8);
    col[3] = (uint8_t)(x1 & 0xFFU);

    Display_SendCommand(0x2AU);  /* Column Address Set */
    Display_SendData(col, 4U);

    row[0] = (uint8_t)(y0 >> 8);
    row[1] = (uint8_t)(y0 & 0xFFU);
    row[2] = (uint8_t)(y1 >> 8);
    row[3] = (uint8_t)(y1 & 0xFFU);

    Display_SendCommand(0x2BU);  /* Row Address Set */
    Display_SendData(row, 4U);

    Display_SendCommand(0x2CU);  /* Memory Write */
}

/*******************************************************************************
 * ILI9341 init sequence (minimum viable)
 ******************************************************************************/
static void Display_Init(void)
{
    uint8_t param;

    Display_Reset();

    /* Software reset */
    Display_SendCommand(0x01U);
    DelayUs(5000U);

    /* Sleep out */
    Display_SendCommand(0x11U);
    DelayUs(120000U);

    /* Pixel format: 16-bit RGB565 */
    Display_SendCommand(0x3AU);
    param = 0x55U;
    Display_SendData(&param, 1U);

    /* Memory access control */
    Display_SendCommand(0x36U);
    param = 0x48U;
    Display_SendData(&param, 1U);

    /* Display ON */
    Display_SendCommand(0x29U);
    DelayUs(10000U);
}

/*******************************************************************************
 * Fill entire screen with a single RGB565 color
 ******************************************************************************/
static void Display_FillScreen(uint16_t color)
{
    static uint8_t lineBuf[TFT_WIDTH * 2U];
    uint32_t x;
    uint32_t y;

    for (x = 0U; x < TFT_WIDTH; x++)
    {
        lineBuf[2U * x]        = (uint8_t)(color >> 8);
        lineBuf[(2U * x) + 1U] = (uint8_t)(color & 0xFFU);
    }

    Display_SetAddressWindow(0U, 0U, TFT_WIDTH - 1U, TFT_HEIGHT - 1U);

    for (y = 0U; y < TFT_HEIGHT; y++)
    {
        Display_SendData(lineBuf, sizeof(lineBuf));
    }
}

/*******************************************************************************
 * Main
 ******************************************************************************/
int main(void)
{
    BOARD_InitHardware();

    PRINTF("\r\n=== SPI TFT (ILI9341) Bring-Up ===\r\n");
    PRINTF("SPI baud: %u Hz\r\n", TFT_SPI_BAUDRATE);

    Board_InitDcResetPins();
    Board_InitLpspi();

    PRINTF("Initializing display...\r\n");
    Display_Init();
    PRINTF("Display init done.\r\n");

    PRINTF("Filling screen red (0xF800)...\r\n");
    Display_FillScreen(0xF800U);
    PRINTF("Fill complete.\r\n");

    while (1)
    {
    }
}
