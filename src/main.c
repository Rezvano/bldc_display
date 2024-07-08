#include "mm32_device.h"

#define SYSCLOCK (72000000)

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

enum
{
	NUM0 = 0xFC,
	NUM1 = 0x60,
	NUM2 = 0xDA,
	NUM3 = 0xF2,
	NUM4 = 0x66,
	NUM5 = 0xB6,
	NUM6 = 0xBe,
	NUM7 = 0xE0,
	NUM8 = 0xFE,
	NUM9 = 0xF6,

	PROGGRESS0 = 0,
	PROGGRESS1 = 0x80,
	PROGGRESS2 = 0xC0,
	PROGGRESS3 = 0xE0,
	PROGGRESS4 = 0xF0,
	PROGGRESS5 = 0xF8,
	PROGGRESS6 = 0xFC,
} seven_seg_char_t;

const uint8_t num_to_seven_seg[10] = {
	NUM0,
	NUM1,
	NUM2,
	NUM3,
	NUM4,
	NUM5,
	NUM6,
	NUM7,
	NUM8,
	NUM9,
};

const uint8_t num_to_proggress[7] = {
	PROGGRESS0,
	PROGGRESS1,
	PROGGRESS2,
	PROGGRESS3,
	PROGGRESS4,
	PROGGRESS5,
	PROGGRESS6,
};

struct
{
	uint8_t data[6];
	uint8_t br;

	enum
	{
		MODE_SPEED,
		MODE_CURRENT,
	} mode;

	uint8_t low_red;
	uint8_t high_red;

	uint8_t num;

	uint8_t ECO;
	uint8_t D;
	uint8_t S;

	uint8_t LED;
	uint8_t BLE;

	uint8_t overheat;
	uint8_t check_engine;

	uint8_t batt_level;
	uint8_t batt_red_point;
} display;

void display_number()
{
	uint8_t nn = display.num % 10;
	uint8_t nN = display.num / 10;

	uint8_t *data_low_N = &display.data[3];
	uint8_t *data_high_N = &display.data[5];

	uint8_t *seven_seg_table = (uint8_t *)num_to_seven_seg;

	if (display.mode == MODE_CURRENT)
		seven_seg_table = (uint8_t *)num_to_proggress;

	if (display.low_red)
		data_low_N = &display.data[2];
	if (display.high_red)
		data_high_N = &display.data[4];

	*data_low_N = seven_seg_table[nn];
	if (nN > 0)
		*data_high_N = seven_seg_table[nN];
	else
		*data_high_N = 0;
}

int main()
{
	SysTick->LOAD = (SYSCLOCK / 1000 - 1) & SysTick_LOAD_RELOAD_Msk;
	SysTick->VAL &= ~SysTick_VAL_CURRENT_Msk;

	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk;

	RCC->AHBENR |= RCC_AHBENR_GPIOB;

	GPIOB->CRH = (GPIO_CNF_MODE_50MHZ_OUT_PP << GPIO_CRH_CNF_MODE_13_Pos) |
				 (GPIO_CNF_MODE_50MHZ_OUT_PP << GPIO_CRH_CNF_MODE_14_Pos);

	display.br = 0xF;

	while (1)
	{
		display_number();
		SmgDisplay(display.data, 6, display.br); // Write register and open display
	}
}
