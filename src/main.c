#include "mm32_device.h"
#include <string.h>

#include "stdlib.h"

#define SYSCLOCK (72000000)

#define IN_START (0x55)
#define IN_STOP (0xAA)

#define OUT_START (0x55)
#define OUT_STOP (0xAA)

uint32_t millis = 0;

void SysTick_Handler()
{
	millis++;
}

void display_bit_delay(uint32_t us)
{
	while (us--)
	{
		__nop();
	}
}

// Define port
#define clk(state) (GPIOB->ODR = (GPIOB->ODR & ~(1 << 13)) | ((state ? 1 : 0) << 13))
#define dio(state) (GPIOB->ODR = (GPIOB->ODR & ~(1 << 14)) | ((state ? 1 : 0) << 14))

void I2CStart(void) // 1637 start
{
	clk(1);
	dio(1);
	display_bit_delay(2);
	dio(0);
}
/// =============================================
void I2Cask(void) // 1637 Answer
{
	clk(0);
	display_bit_delay(5); // After the falling edge of the eighth clock delay 5us, ACK signals the beginning of judgment while (dio);
	clk(1);
	display_bit_delay(2);
	clk(0);
}
/// ========================================
void I2CStop(void) // 1637 Stop
{
	clk(0);
	display_bit_delay(2);
	dio(0);
	display_bit_delay(2);
	clk(1);
	display_bit_delay(2);
	dio(1);
}
/// =========================================
void I2CWrByte(unsigned char oneByte) // write a byte
{
	unsigned char i;
	for (i = 0; i < 8; i++)
	{
		clk(0);
		if (oneByte & 0x01) // low front
		{
			dio(1);
		}
		else
		{
			dio(0);
		}
		display_bit_delay(3);
		oneByte = oneByte >> 1;
		clk(1);
		display_bit_delay(3);
	}
}

void SmgDisplay(uint8_t *data, uint8_t size, uint8_t br) // Write display register
{
	unsigned char i;
	I2CStart();
	I2CWrByte(0x40); // 40H address is automatically incremented by 1 mode, 44H fixed address mode
	I2Cask();
	I2CStop();
	I2CStart();
	I2CWrByte(0xc0); // Set the first address
	I2Cask();
	size = size > 6 ? 6 : size;
	for (i = 0; i < size; i++) // Addresses from Canada, do not always write address
	{
		I2CWrByte(data[i]); // Send data
		I2Cask();
	}
	I2CStop();
	I2CStart();
	I2CWrByte(0x80 | (br & 0xF)); // Open display, maximum brightness
	I2Cask();
	I2CStop();
}

const uint8_t num_to_seven_seg[10] = {
	0xFC,
	0x60,
	0xDA,
	0xF2,
	0x66,
	0xB6,
	0xBe,
	0xE0,
	0xFE,
	0xF6,
};

typedef struct __attribute__((__packed__))
{
	uint8_t start;

	uint8_t mode;
	uint8_t LED;

	uint16_t current;
	uint16_t set_current;

	uint8_t freq;
	uint8_t batt_voltage;

	uint8_t err;

	uint8_t stop;
} in_data_t;

typedef struct __attribute__((__packed__))
{
	uint8_t start;
	uint16_t accel;
	uint16_t brake;
	uint8_t stop;
} out_data_t;

out_data_t out_data_current;
out_data_t out_data;
int out_data_sended_size = 0;

in_data_t in_data_current;
in_data_t in_data;
int in_data_readed = 0;

int no_data_timeout = 0;

void tx_task()
{
	static uint32_t delay_ms = 0;
	if (delay_ms < millis)
	{
		delay_ms = millis + 50;
		memcpy(&out_data, &out_data_current, sizeof(out_data_t));

		out_data.start = OUT_START;
		out_data.stop = OUT_STOP;

		out_data_sended_size = 0;
	}

	if (out_data_sended_size < sizeof(out_data_t) && (UART2->CSR & UART_CSR_TXC) > 0)
	{
		UART2->TDR = *(((uint8_t *)&out_data) + (out_data_sended_size++));
	}
}

void rx_task()
{
	uint8_t *read_data = (void *)&in_data_current;

	if (UART2->CSR & UART_CSR_RXAVL)
	{
		uint8_t byte = UART2->RDR;
		if (byte == IN_START)
			in_data_readed = 0;

		read_data[in_data_readed++] = byte;

		if (in_data_readed == sizeof(in_data_t))
		{
			if (byte == IN_STOP)
			{
				memcpy(&in_data, &in_data_current, sizeof(in_data_t));
				no_data_timeout = millis + 2000;
			}
			in_data_readed = 0;
		}
	}
}

void display_task()
{
	static uint32_t delay_ms = 0;

	static uint8_t data[6];
	if (delay_ms < millis)
	{
		delay_ms = millis + 200;
		if (no_data_timeout > millis)
		{
			if (in_data.start == IN_START && in_data.stop == IN_STOP)
			{
				for (int i = 0; i < 6; i++)
					data[i] = 0;
				// speed
				data[3] = num_to_seven_seg[in_data.freq % 10];
				data[5] = num_to_seven_seg[(in_data.freq / 10) % 10];

				// errors
				data[1] |= in_data.err;

				// batt
				if (in_data.batt_voltage > 30)
					data[1] |= 1 << 7;
				if (in_data.batt_voltage > 32)
					data[1] |= 1 << 6;
				if (in_data.batt_voltage > 34)
					data[1] |= 1 << 5;
				if (in_data.batt_voltage > 36)
					data[1] |= 1 << 4;
				if (in_data.batt_voltage > 38)
					data[1] |= 1 << 3;

				// mode & info
				data[0] |= 1 << (3 + in_data.mode);
				data[0] |= in_data.LED << 2;
				data[0] |= 1 << 7;

				SmgDisplay(data, 6, 0xf);
			}
		}
		else
		{
			for (int i = 0; i < 6; i++)
				data[i] = 0;
			SmgDisplay(data, 6, 0xf);
		}
	}
}

int uint16_t_qsort(const void *a, const void *b)
{
	return *(uint16_t *)a - *(uint16_t *)b;
}

void adc_task()
{
	static uint32_t delay_ms = 0;
	static uint16_t accel_median_table[5];
	static uint16_t brake_median_table[5];

	static uint8_t median_id = 0;

	if (delay_ms < millis)
	{
		delay_ms = millis + 5;

		if ((ADC1->SR & ADC_SR_BUSY) == 0)
		{
			brake_median_table[median_id] = ADC1->CH0DR & 0xFFFF;
			accel_median_table[median_id++] = ADC1->CH4DR & 0xFFFF;

			if (median_id > (sizeof(accel_median_table) / sizeof(accel_median_table[0])))
			{
				median_id = 0;
				qsort(brake_median_table, (sizeof(accel_median_table) / sizeof(accel_median_table[0])),
					  sizeof(uint16_t), uint16_t_qsort);
				qsort(accel_median_table, (sizeof(accel_median_table) / sizeof(accel_median_table[0])),
					  sizeof(uint16_t), uint16_t_qsort);

				static int accel;
				static int brake;

				accel += (100 * accel_median_table[(sizeof(accel_median_table) / sizeof(accel_median_table[0])) / 2] - accel) / 2;
				brake += (100 * brake_median_table[(sizeof(brake_median_table) / sizeof(brake_median_table[0])) / 2] - brake) / 2;
				out_data_current.accel = accel / 100;
				out_data_current.brake = brake / 100;
			}
			ADC1->CR |= ADC_CR_ADST;
		}
	}
}

int main()
{

	SysTick->LOAD = (SYSCLOCK / 1000 - 1) & SysTick_LOAD_RELOAD_Msk;
	SysTick->VAL &= ~SysTick_VAL_CURRENT_Msk;

	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk;

	RCC->AHBENR |= RCC_AHBENR_GPIOB | RCC_AHBENR_GPIOA;

	RCC->APB2ENR |= RCC_APB2RSTR_ADC1;
	RCC->APB1ENR |= RCC_APB1ENR_UART2;

	GPIOB->CRH = (GPIO_CNF_MODE_50MHZ_OUT_PP << GPIO_CRH_CNF_MODE_13_Pos) |
				 (GPIO_CNF_MODE_50MHZ_OUT_PP << GPIO_CRH_CNF_MODE_14_Pos);
	GPIOB->CRL = (GPIO_CNF_MODE_AF_PP << GPIO_CRL_CNF_MODE_7_Pos);
	GPIOB->AFRL = (GPIO_AF_MODE4 << GPIO_AFRL_AFR7_Pos);

	GPIOA->CRL = (GPIO_CNF_MODE_AIN << GPIO_CRL_CNF_MODE_0_Pos) |
				 (GPIO_CNF_MODE_AIN << GPIO_CRL_CNF_MODE_4_Pos) |
				 (GPIO_CNF_MODE_INPUPD << GPIO_CRL_CNF_MODE_6_Pos) |
				 (GPIO_CNF_MODE_50MHZ_OUT_PP << GPIO_CRL_CNF_MODE_5_Pos);

	GPIOA->AFRL = (GPIO_AF_MODE3 << GPIO_AFRL_AFR6_Pos);

	ADC1->CFGR = ADC_CFGR_ADEN;
	ADC1->CR = 0x01 << ADC_CR_MODE_Pos;
	ADC1->CHSR = ADC_CHSR_CH0 | ADC_CHSR_CH4;

	UART2->GCR = UART_GCR_TX | UART_GCR_RX | UART_GCR_UART;
	UART2->CCR = UART_CCR_CHAR_8b;
	UART2->BRR = 19;

	// UART2->IER |= UART_IER_RX;

	// NVIC_SetPriority(UART2_IRQn, 1);
	// NVIC_EnableIRQ(UART2_IRQn);

	out_data_current.start = OUT_START;
	out_data_current.stop = OUT_STOP;

	for (;;)
	{
		display_task();
		adc_task();
		tx_task();
		rx_task();

		if (in_data.LED)
			GPIOA->ODR |= 1 << 5;
		else
			GPIOA->ODR &= ~(1 << 5);
	}
}
