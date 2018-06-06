#include <math.h>
#include <stdio.h>
#include "arm_math.h"
#include "main.h"
#include "pdm_filter.h"
#include "stm32f4xx.h"

uint8_t PDM_Input_Buffer[PDM_Input_Buffer_SIZE];
uint16_t PCM_Output_Buffer[PCM_Output_Buffer_SIZE];

float32_t buffer_input[4096];
float32_t buffer_output[4096];
float32_t buffer_output_mag[4096];

float32_t maxvalue;
uint32_t maxvalueindex;

float32_t peak = 0;
uint8_t mode = 1;

arm_rfft_instance_f32 S;
arm_cfft_radix4_instance_f32 S_CFFT;
PDMFilter_InitStruct Filter;

static void GPIO_Configure(void);
static void I2S_Configure(void);
static void NVIC_Configure(void);
static void RCC_Configure(void);

void compare(int a, int b) { // Compare the peak with given frequencies
	if (peak > a && peak < b) {
		GPIO_WriteBit(GPIOD, GPIO_Pin_12, Bit_SET);
		GPIO_WriteBit(GPIOD, GPIO_Pin_14, Bit_SET);
	} else if (peak < a)
		GPIO_WriteBit(GPIOD, GPIO_Pin_12, Bit_SET);
	else if (peak > b)
		GPIO_WriteBit(GPIOD, GPIO_Pin_14, Bit_SET);
}

int main(void) {
	extern uint32_t Data_Status;
	unsigned int i, z;

	RCC_Configure();
	NVIC_Configure();
	GPIO_Configure();
	I2S_Configure();

	// Initialize PDM filter
	Filter.Fs = OUT_FREQ;
	Filter.HP_HZ = 70;
	Filter.LP_HZ = 1000;
	Filter.In_MicChannels = 1;
	Filter.Out_MicChannels = 1;
	PDM_Filter_Init(&Filter);

	arm_rfft_init_f32(&S, &S_CFFT, 2048, 0, 1); // Initialize FFT
	z = 0;

	I2S_Cmd(SPI2, ENABLE);

	while (1) {

		if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)) {
			while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0))
				;
			if (mode == 1) {
				mode = 2;
				GPIO_WriteBit(GPIOD, GPIO_Pin_13, Bit_SET);
			} else if (mode == 2) {
				mode = 1;
				GPIO_WriteBit(GPIOD, GPIO_Pin_13, Bit_RESET);

			}
		}

		if (Data_Status) {
			for (i = 0; i < (OUT_FREQ / 1000); i++) { //Fill FFT buffer with samples
				buffer_input[i + (OUT_FREQ / 1000) * z] =
						(float32_t) PCM_Output_Buffer[i];
			}

			++z;
			if (z > 2048 / (OUT_FREQ / 1000)) {
				z = 0;
				// FFT
				arm_rfft_f32(&S, buffer_input, buffer_output);
				// Calculate magnitude
				arm_cmplx_mag_f32(buffer_output, buffer_output_mag, 2048);
				// Get maximum value of magnitude
				arm_max_f32(&(buffer_output_mag[1]), 2048, &maxvalue,
						&maxvalueindex);
			}
			Data_Status = 0;
		}

		if (maxvalue < 5000000) { // Threshold
			peak = 0;
			GPIO_WriteBit(GPIOD, GPIO_Pin_12, Bit_RESET);
			GPIO_WriteBit(GPIOD, GPIO_Pin_14, Bit_RESET);
		} else {
			peak = (maxvalueindex + 1) * 62.5 / 4 / 4; // Calculate peak frequency
			GPIO_WriteBit(GPIOD, GPIO_Pin_12, Bit_RESET);
			GPIO_WriteBit(GPIOD, GPIO_Pin_14, Bit_RESET);

			if (mode == 2){
				peak /= 2;
			}
			if (peak >= 280 && peak < 400) { //E
				compare(328, 330);
			} else if (peak >= 220 && peak < 280) { //B
				compare(244, 248);
			} else if (peak >= 165 && peak < 220) { //G
				compare(195, 199);
			} else if (peak >= 130 && peak < 165) { //D
				compare(144, 148);
			} else if (peak >= 95 && peak < 130) { //A
				compare(109, 114);
			} else if (peak >= 70 && peak < 95) { //E
				compare(81, 83);
			}
		}
	}
	I2S_Cmd(SPI2, DISABLE);
}

static void GPIO_Configure(void) {
	GPIO_InitTypeDef GPIO_InitStructure;

	// Configure MP45DT02's CLK / I2S2_CLK (PB10) line
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// Configure MP45DT02's DOUT / I2S2_DATA (PC3) line
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_SPI2); // Connect pin 10 of port B to the SPI2 peripheral
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource3, GPIO_AF_SPI2); // Connect pin 3 of port C to the SPI2 peripheral

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure); // Enable LED indicators

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStructure); // Enable LED indicators

}

static void I2S_Configure(void) {
	I2S_InitTypeDef I2S_InitStructure;

	SPI_I2S_DeInit(SPI2);
	I2S_InitStructure.I2S_AudioFreq = OUT_FREQ * 2;
	I2S_InitStructure.I2S_Standard = I2S_Standard_LSB;
	I2S_InitStructure.I2S_DataFormat = I2S_DataFormat_16b;
	I2S_InitStructure.I2S_CPOL = I2S_CPOL_High;
	I2S_InitStructure.I2S_Mode = I2S_Mode_MasterRx;
	I2S_InitStructure.I2S_MCLKOutput = I2S_MCLKOutput_Disable;
	I2S_Init(SPI2, &I2S_InitStructure);

	SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);
}

static void NVIC_Configure(void) {
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
}

static void RCC_Configure(void) {
	RCC_AHB1PeriphClockCmd(
			RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_CRC,
			ENABLE);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	RCC_PLLI2SCmd(ENABLE);
}
