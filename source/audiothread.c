/*
 * audiothread.c
 *
 *  Created on: 9 cze 2021
 *      Author: Jakub
 */

#include "audioThread.h"

#include "fsl_sai_edma.h"
#include "fsl_codec_common.h"
#include "fsl_wm8960.h"
#include "fsl_codec_adapter.h"
#include "fsl_sai.h"
#include "fsl_dmamux.h"

#include "fsl_debug_console.h"

#include "board.h"
#include <cr_section_macros.h>

/* SAI instance and clock */
#define CODEC_WM8960
#define SAI              SAI1
#define SAI_CHANNEL      (0)
#define SAI_IRQ          SAI1_IRQn
#define SAITxIRQHandler  SAI1_IRQHandler
#define SAI_TX_SYNC_MODE kSAI_ModeAsync
#define SAI_RX_SYNC_MODE kSAI_ModeSync
#define SAI_MCLK_OUTPUT  true
#define SAI_MASTER_SLAVE kSAI_Master

#define AUDIO_DATA_CHANNEL (2U)
#define AUDIO_BIT_WIDTH    kSAI_WordWidth16bits
#define AUDIO_SAMPLE_RATE  (kSAI_SampleRate8KHz)
#define AUDIO_MASTER_CLOCK SAI_CLK_FREQ

/* IRQ */
#define SAI_TX_IRQ SAI1_IRQn
#define SAI_RX_IRQ SAI1_IRQn

/* DMA */
#define DMA             DMA0
#define TX_EDMA_CHANNEL (0U)
#define RX_EDMA_CHANNEL (1U)
#define SAI_TX_SOURCE   kDmaRequestMuxSai1Tx
#define SAI_RX_SOURCE   kDmaRequestMuxSai1Rx

/* Select Audio/Video PLL (786.48 MHz) as sai1 clock source */
#define SAI1_CLOCK_SOURCE_SELECT (2U)
/* Clock pre divider for sai1 clock source */
#define SAI1_CLOCK_SOURCE_PRE_DIVIDER (0U)
/* Clock divider for sai1 clock source */
#define SAI1_CLOCK_SOURCE_DIVIDER (63U)
/* Get frequency of sai1 clock */
#define SAI_CLK_FREQ                                                        \
    (CLOCK_GetFreq(kCLOCK_AudioPllClk) / (SAI1_CLOCK_SOURCE_DIVIDER + 1U) / \
     (SAI1_CLOCK_SOURCE_PRE_DIVIDER + 1U))

/* I2C instance and clock */
#define I2C LPI2C1

/* Select USB1 PLL (480 MHz) as master lpi2c clock source */
#define LPI2C_CLOCK_SOURCE_SELECT (0U)
/* Clock divider for master lpi2c clock source */
#define LPI2C_CLOCK_SOURCE_DIVIDER (5U)
/* Get frequency of lpi2c clock */
#define I2C_CLK_FREQ ((CLOCK_GetFreq(kCLOCK_Usb1PllClk) / 8) / (LPI2C_CLOCK_SOURCE_DIVIDER + 1U))

#define BOARD_MASTER_CLOCK_CONFIG()
#define BUFFER_SIZE   (1024U)
#define BUFFER_SIZE_IN_BYTES  (BUFFER_SIZE * 2U)
#define BUFFER_NUMBER (2U)

wm8960_config_t wm8960Config = {
    .i2cConfig = {.codecI2CInstance = BOARD_CODEC_I2C_INSTANCE, .codecI2CSourceClock = BOARD_CODEC_I2C_CLOCK_FREQ},
    .route     = kWM8960_RoutePlaybackandRecord,
    .rightInputSource = kWM8960_InputDifferentialMicInput2,
    .playSource       = kWM8960_PlaySourceDAC,
    .slaveAddress     = WM8960_I2C_ADDR,
    .bus              = kWM8960_BusI2S,
    .format = {.mclk_HZ = 6144000U, .sampleRate = kWM8960_AudioSampleRate8KHz, .bitWidth = kWM8960_AudioBitWidth16bit},
    .master_slave = false,
};
codec_config_t boardCodecConfig = {.codecDevType = kCODEC_WM8960, .codecDevConfig = &wm8960Config};
/*
 * AUDIO PLL setting: Frequency = Fref * (DIV_SELECT + NUM / DENOM)
 *                              = 24 * (32 + 77/100)
 *                              = 786.48 MHz
 */
const clock_audio_pll_config_t audioPllConfig = {
    .loopDivider = 32,  /* PLL loop divider. Valid range for DIV_SELECT divider value: 27~54. */
    .postDivider = 1,   /* Divider after the PLL, should only be 1, 2, 4, 8, 16. */
    .numerator   = 77,  /* 30 bit numerator of fractional loop divider. */
    .denominator = 100, /* 30 bit denominator of fractional loop divider */
};
#define WORD_SIZE 2U



__NOINIT (BOARD_SDRAM)static int8_t Buffer[BUFFER_NUMBER * BUFFER_SIZE * WORD_SIZE];
AT_NONCACHEABLE_SECTION_INIT(sai_edma_handle_t txHandle);
AT_NONCACHEABLE_SECTION_INIT(sai_edma_handle_t rxHandle);
static uint32_t tx_index = 0U, rx_index = 0U;
volatile uint32_t emptyBlock = BUFFER_NUMBER;
edma_handle_t dmaTxHandle = {0}, dmaRxHandle = {0};
extern codec_config_t boardCodecConfig;
codec_handle_t codecHandle;




static void BOARD_EnableSaiMclkOutput(bool enable)
{
    if (enable)
    {
        IOMUXC_GPR->GPR1 |= IOMUXC_GPR_GPR1_SAI1_MCLK_DIR_MASK;
    }
    else
    {
        IOMUXC_GPR->GPR1 &= (~IOMUXC_GPR_GPR1_SAI1_MCLK_DIR_MASK);
    }
}

static void rx_callback(I2S_Type *base, sai_edma_handle_t *handle, status_t status, void *userData)
{
    if (kStatus_SAI_RxError == status)
    {
        /* Handle the error. */
    }
    else
    {
        emptyBlock--;
    }
}

static void tx_callback(I2S_Type *base, sai_edma_handle_t *handle, status_t status, void *userData)
{
    if (kStatus_SAI_TxError == status)
    {
        /* Handle the error. */
    }
    else
    {
        emptyBlock++;
    }
}

void setupAudioThread(void)
{
	CLOCK_InitAudioPll(&audioPllConfig);
	/*Clock setting for LPI2C*/
	    CLOCK_SetMux(kCLOCK_Lpi2cMux, LPI2C_CLOCK_SOURCE_SELECT);
	    CLOCK_SetDiv(kCLOCK_Lpi2cDiv, LPI2C_CLOCK_SOURCE_DIVIDER);

	    /*Clock setting for SAI1*/
	    CLOCK_SetMux(kCLOCK_Sai1Mux, SAI1_CLOCK_SOURCE_SELECT);
	    CLOCK_SetDiv(kCLOCK_Sai1PreDiv, SAI1_CLOCK_SOURCE_PRE_DIVIDER);
	    CLOCK_SetDiv(kCLOCK_Sai1Div, SAI1_CLOCK_SOURCE_DIVIDER);

	    /*Enable MCLK clock*/
	    BOARD_EnableSaiMclkOutput(true);

	    /* Init DMAMUX */
	    DMAMUX_Init(DMAMUX);
	    DMAMUX_SetSource(DMAMUX, TX_EDMA_CHANNEL, (uint8_t)SAI_TX_SOURCE);
	    DMAMUX_EnableChannel(DMAMUX, TX_EDMA_CHANNEL);
	    DMAMUX_SetSource(DMAMUX, RX_EDMA_CHANNEL, (uint8_t)SAI_RX_SOURCE);
	    DMAMUX_EnableChannel(DMAMUX, RX_EDMA_CHANNEL);

	    PRINTF("SAI example started!\n\r");

	    edma_config_t dmaConfig = {0};

	    /* Init DMA and create handle for DMA */
	    EDMA_GetDefaultConfig(&dmaConfig);
	    EDMA_Init(DMA, &dmaConfig);
	    EDMA_CreateHandle(&dmaTxHandle, DMA, TX_EDMA_CHANNEL);
	    EDMA_CreateHandle(&dmaRxHandle, DMA, RX_EDMA_CHANNEL);
	#if defined(FSL_FEATURE_EDMA_HAS_CHANNEL_MUX) && FSL_FEATURE_EDMA_HAS_CHANNEL_MUX
	    EDMA_SetChannelMux(DMA, TX_EDMA_CHANNEL, SAI_TX_EDMA_CHANNEL);
	    EDMA_SetChannelMux(DMA, RX_EDMA_CHANNEL, SAI_RX_EDMA_CHANNEL);
	#endif


	    sai_transceiver_t saiConfig;
	    /* SAI init */
	    SAI_Init(SAI);

	    SAI_TransferTxCreateHandleEDMA(SAI, &txHandle, tx_callback, NULL, &dmaTxHandle);
	    SAI_TransferRxCreateHandleEDMA(SAI, &rxHandle, rx_callback, NULL, &dmaRxHandle);

	    /* I2S mode configurations */
	    SAI_GetClassicI2SConfig(&saiConfig, AUDIO_BIT_WIDTH, kSAI_MonoRight, 1U << SAI_CHANNEL);
	    saiConfig.syncMode    = SAI_TX_SYNC_MODE;
	    saiConfig.masterSlave = SAI_MASTER_SLAVE;
	    SAI_TransferTxSetConfigEDMA(SAI, &txHandle, &saiConfig);
	    saiConfig.syncMode = SAI_RX_SYNC_MODE;
	    SAI_TransferRxSetConfigEDMA(SAI, &rxHandle, &saiConfig);

	    /* set bit clock divider */
	    SAI_TxSetBitClockRate(SAI, AUDIO_MASTER_CLOCK, AUDIO_SAMPLE_RATE, AUDIO_BIT_WIDTH,
	                          AUDIO_DATA_CHANNEL);
	    SAI_RxSetBitClockRate(SAI, AUDIO_MASTER_CLOCK, AUDIO_SAMPLE_RATE, AUDIO_BIT_WIDTH,
	                          AUDIO_DATA_CHANNEL);

	    /* master clock configurations */
	    BOARD_MASTER_CLOCK_CONFIG();

	    /* Use default setting to init codec */
	    if (CODEC_Init(&codecHandle, &boardCodecConfig) != kStatus_Success)
	    {
	        assert(false);
	    }

	    HAL_CODEC_SetMicGain(&codecHandle, 99);
}


void audioThread(void *pvParameters)
{
	uint8_t state=  0;

	uint16_t y_k =0;
	while (1)
	{

		sai_transfer_t xfer;

		if (emptyBlock > 0)
		{
			xfer.data     = (uint8_t*)(Buffer + rx_index * BUFFER_SIZE);
			xfer.dataSize = BUFFER_SIZE;
			if (kStatus_Success == SAI_TransferReceiveEDMA(SAI, &rxHandle, &xfer))
			{
				rx_index++;
			}
			if (rx_index == BUFFER_NUMBER  *2)
			{
				rx_index = 0U;
			}
		}

		uint8_t OnThreshold = 130;
		uint8_t OffThreshold = 70;



		if (emptyBlock < BUFFER_NUMBER)
		{

			xfer.data     = Buffer + tx_index * BUFFER_SIZE;
			xfer.dataSize = BUFFER_SIZE;
			if (kStatus_Success == SAI_TransferSendEDMA(SAI, &txHandle, &xfer))
			{
				tx_index++;
			}
			if (tx_index == BUFFER_NUMBER * 2)
			{
				tx_index = 0U;
			}


			int16_t* workingBuffer = (int16_t*)(Buffer + tx_index * BUFFER_SIZE);

			for (int i = 0; i < BUFFER_SIZE ; i++)
			{
				int16_t val = abs(workingBuffer[i]);

//        		uint32_t av = (filtTaps[0] + filtTaps[1] + filtTaps[2] + filtTaps[3] + filtTaps[4] )/ 5;
				float av = val * 0.03 + y_k * 0.97;

				y_k  = (uint32_t) av;
//        		filtTaps[0] = filtTaps[1];
//        		filtTaps[1] = filtTaps[2];
//        		filtTaps[2] = filtTaps[3];
//        		filtTaps[3] = filtTaps[4];
//        		filtTaps[4] = val;

				if (state == 0 && av > OnThreshold)
				{
					state = 1;
					USER_LED_ON();
				}
//        		else if (state == 1 && y_k < OffThreshold)
//				{
//        			state = 0;
//        			USER_LED_OFF();
//				}

			}



		}
	}
}
