/*
	Copyright 2016 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef HW_moxie_mini_H_
#define HW_moxie_mini_H_

#define HW_NAME					"moxie_mini_v1"

// HW properties
#define HW_HAS_DRV8301
#define HW_HAS_3_SHUNTS
#define HW_HAS_PERMANENT_NRF

/*
* hw info
*


ports:
- servo, also can act as adc, maybe ws2812 as well w tim5?

- serial1. serial, i2c, and ws2812 timer def supported. either pin.
- serial2, can only act as serial. connect to bluetooth module
- i2c1, can also be used as serial port if needed.

- hall, connected to timer, as well as spi 1/3 port pins.
- analog in / digital. throttle, brake, estop, cruize. 
	also has dac outputs for debug/dac output.
	adc's protected.
*/




// Macros
#define LED_GREEN_GPIO			GPIOD
#define LED_GREEN_PIN			12
#define LED_RED_GPIO			GPIOD
#define LED_RED_PIN				14

#define LED_GREEN_ON()			palSetPad(LED_GREEN_GPIO, LED_GREEN_PIN)
#define LED_GREEN_OFF()			palClearPad(LED_GREEN_GPIO, LED_GREEN_PIN)
#define LED_RED_ON()			palSetPad(LED_RED_GPIO, LED_RED_PIN)
#define LED_RED_OFF()			palClearPad(LED_RED_GPIO, LED_RED_PIN)

#define GATE_ENABLE_GPIO		GPIOB
#define GATE_ENABLE_PIN			5
#define ENABLE_GATE()			palSetrPad(GATE_ENABLE_GPIO, GATE_ENABLE_PIN)
#define DISABLE_GATE()			palClearPad(GATE_ENABLE_GPIO, GATE_ENABLE_PIN)

#define DCCAL_ON() // drv spi fcn
#define DCCAL_OFF()
#define IS_DRV_FAULT()			(!palReadPad(GPIOB, 7))


#ifdef HW_HAS_SHUTDOWN
// Shutdown pin
#define HW_SHUTDOWN_GPIO		GPIOC
#define HW_SHUTDOWN_PIN			5
#define HW_SHUTDOWN_HOLD_ON()	palSetPad(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN)
#define HW_SHUTDOWN_HOLD_OFF()	palClearPad(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN)
#define HW_SAMPLE_SHUTDOWN()	hw_sample_shutdown_button()

// Hold shutdown pin early to wake up on short pulses
#define HW_EARLY_INIT()			palSetPadMode(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN, PAL_MODE_OUTPUT_PUSHPULL); \
								HW_SHUTDOWN_HOLD_ON(); \
								palSetPadMode(GPIOD, 2, \
								PAL_MODE_OUTPUT_PUSHPULL | \
								PAL_STM32_OSPEED_HIGHEST); \
								CURRENT_FILTER_ON()

//functions
bool hw_sample_shutdown_button(void);							
#else


/*
 * ADC Vector
 *
 * 0:	IN0		SENS1
 * 1:	IN1		SENS2
 * 2:	IN2		SENS3
 * 3:	IN10	CURR1
 * 4:	IN11	CURR2
 * 5:	IN12	CURR3
 * 6:	IN5		ADC_EXT1
 * 7:	IN6		ADC_EXT2
 * 8:	IN3		TEMP_PCB
 * 9:	IN14	TEMP_MOTOR
 * 10:	IN15	ADC_EXT3, Shutdown on MK3
 * 11:	IN13	AN_IN
 * 12:	Vrefint
 * 13:	IN0		SENS1
 * 14:	IN1		SENS2
 */

/*
 * ADC Vector
 *
 * 0  (1):	IN0		SENS1
 * 1  (2):	IN1		SENS2
 * 2  (3):	IN2		SENS3
 * 3  (1):	IN10	CURR1
 * 4  (2):	IN11	CURR2
 * 5  (3):	IN12	CURR3
 * 
 * 6  (1):	IN5		ADC_EXT1
 * 7  (2):	IN6		ADC_EXT2
 * 8  (3):	IN3		TEMP_MOS
 * 9  (1):	IN14	TEMP_MOTOR
 * 10 (2):	IN15	ADC_EXT3
 * 11 (3):	IN13	AN_IN
 * 12 (1):	Vrefint
 * 13 (2):	IN0		SENS1
 * 14 (3):	IN1		SENS2
 * 15 (1):  IN8		TEMP_MOS_2
 * 16 (2):  IN9		TEMP_MOS_3
 * 17 (3):  IN3		SENS3
 */

#define HW_ADC_CHANNELS			15
#define HW_ADC_INJ_CHANNELS		3
#define HW_ADC_NBR_CONV			5

// ADC Indexes
#define ADC_IND_SENS1			0
#define ADC_IND_SENS2			1
#define ADC_IND_SENS3			2
#define ADC_IND_CURR1			3
#define ADC_IND_CURR2			4
#define ADC_IND_CURR3			5
#define ADC_IND_VIN_SENS		11
#define ADC_IND_EXT				6
#define ADC_IND_EXT2			7
#define ADC_IND_TEMP_MOS		8
#define ADC_IND_TEMP_MOTOR		9
#define ADC_IND_VREFINT			12
#ifdef HW60_IS_MK3
#define ADC_IND_SHUTDOWN		10
#endif

// ADC macros and settings

// Component parameters (can be overridden)
#ifndef V_REG
#define V_REG					3.3
#endif
#ifndef VIN_R1
#define VIN_R1					39000.0
#endif
#ifndef VIN_R2
#define VIN_R2					2200.0
#endif
#ifndef CURRENT_AMP_GAIN
#define CURRENT_AMP_GAIN		20.0
#endif
#ifndef CURRENT_SHUNT_RES
#define CURRENT_SHUNT_RES		0.0005
#endif

// Input voltage
#define GET_INPUT_VOLTAGE()		((V_REG / 4095.0) * (float)ADC_Value[ADC_IND_VIN_SENS] * ((VIN_R1 + VIN_R2) / VIN_R2))

// NTC Termistors
#define NTC_RES(adc_val)		((4095.0 * 10000.0) / adc_val - 10000.0)
#define NTC_TEMP(adc_ind)		(1.0 / ((logf(NTC_RES(ADC_Value[adc_ind]) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15)

#define NTC_RES_MOTOR(adc_val)	(10000.0 / ((4095.0 / (float)adc_val) - 1.0)) // Motor temp sensor on low side
#define NTC_TEMP_MOTOR(beta)	(1.0 / ((logf(NTC_RES_MOTOR(ADC_Value[ADC_IND_TEMP_MOTOR]) / 10000.0) / beta) + (1.0 / 298.15)) - 273.15)

// Voltage on ADC channel
#define ADC_VOLTS(ch)			((float)ADC_Value[ch] / 4096.0 * V_REG)

// // Double samples in beginning and end for positive current measurement.
// // Useful when the shunt sense traces have noise that causes offset.
// #ifndef CURR1_DOUBLE_SAMPLE
// #define CURR1_DOUBLE_SAMPLE		0
// #endif
// #ifndef CURR2_DOUBLE_SAMPLE
// #define CURR2_DOUBLE_SAMPLE		0
// #endif
// #ifndef CURR3_DOUBLE_SAMPLE
// #define CURR3_DOUBLE_SAMPLE		0
// #endif

// EXT ADC GPIOs
// fix
#define HW_ADC_EXT_GPIO			GPIOA
#define HW_ADC_EXT_PIN			5
#define HW_ADC_EXT2_GPIO		GPIOA
#define HW_ADC_EXT2_PIN			6

// UART Peripheral
// *good to moxie?
// test
#define HW_UART_DEV				SD1
#define HW_UART_GPIO_AF			GPIO_AF_USART1
#define HW_UART_TX_PORT			GPIOB
#define HW_UART_TX_PIN			6
#define HW_UART_RX_PORT			GPIOB
#define HW_UART_RX_PIN			7

// Permanent UART Peripheral (for NRF51)
// *good to moxie
#define HW_UART_P_BAUD			115200
#define HW_UART_P_DEV			SD4
#define HW_UART_P_GPIO_AF		GPIO_AF_UART4
#define HW_UART_P_TX_PORT		GPIOC
#define HW_UART_P_TX_PIN		10
#define HW_UART_P_RX_PORT		GPIOC
#define HW_UART_P_RX_PIN		11

// !!!!!!!!!! test
// ICU Peripheral for servo decoding (from unity conf, test.)
// servo actually on PA3, tim9/ch2, or tim5/ch4
#define HW_ICU_TIMER            TIM9
#define HW_ICU_DEV              ICUD9
//HW_ICU_TIM_CLK_EN() need to bring this over?
#define HW_ICU_CHANNEL          ICU_CHANNEL_2
#define HW_ICU_GPIO_AF          GPIO_AF_TIM9
#define HW_ICU_GPIO             GPIOA
#define HW_ICU_PIN 	            3

// // ICU Peripheral for servo decoding
// #define HW_USE_SERVO_TIM4
// #define HW_ICU_TIMER			TIM4
// #define HW_ICU_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE)
// #define HW_ICU_DEV				ICUD4
// #define HW_ICU_CHANNEL			ICU_CHANNEL_1
// #define HW_ICU_GPIO_AF			GPIO_AF_TIM4
// #define HW_ICU_GPIO				GPIOB
// #define HW_ICU_PIN				6

// I2C Peripheral
// *good to moxie
#define HW_I2C_DEV				I2CD2
#define HW_I2C_GPIO_AF			GPIO_AF_I2C2
#define HW_I2C_SCL_PORT			GPIOB
#define HW_I2C_SCL_PIN			10
#define HW_I2C_SDA_PORT			GPIOB
#define HW_I2C_SDA_PIN			11

// Hall/encoder pins
// *good to moxie
#define HW_HALL_ENC_GPIO1		GPIOC
#define HW_HALL_ENC_PIN1		6
#define HW_HALL_ENC_GPIO2		GPIOC
#define HW_HALL_ENC_PIN2		7
#define HW_HALL_ENC_GPIO3		GPIOC
#define HW_HALL_ENC_PIN3		8
#define HW_ENC_TIM				TIM3
#define HW_ENC_TIM_AF			GPIO_AF_TIM3
#define HW_ENC_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE)
#define HW_ENC_EXTI_PORTSRC		EXTI_PortSourceGPIOC
#define HW_ENC_EXTI_PINSRC		EXTI_PinSource8
#define HW_ENC_EXTI_CH			EXTI9_5_IRQn
#define HW_ENC_EXTI_LINE		EXTI_Line8
#define HW_ENC_EXTI_ISR_VEC		EXTI9_5_IRQHandler
#define HW_ENC_TIM_ISR_CH		TIM3_IRQn
#define HW_ENC_TIM_ISR_VEC		TIM3_IRQHandler

// // SPI pins
// // connected to hall port for direct spi encoder
// #define HW_SPI_DEV				SPID1
// #define HW_SPI_GPIO_AF			GPIO_AF_SPI1
// #define HW_SPI_PORT_NSS			GPIOA
// #define HW_SPI_PIN_NSS			4
// #define HW_SPI_PORT_SCK			GPIOA
// #define HW_SPI_PIN_SCK			5
// #define HW_SPI_PORT_MOSI		GPIOA
// #define HW_SPI_PIN_MOSI			7
// #define HW_SPI_PORT_MISO		GPIOA
// #define HW_SPI_PIN_MISO			6

// SPI for DRV8353
// *good to moxie? no
// todo: change to drv8353
// mosi = pb1
// miso = pb12
// sck = pc13
// cs =  pc14
#define DRV8301_MOSI_GPIO		GPIOB
#define DRV8301_MOSI_PIN		1
#define DRV8301_MISO_GPIO		GPIOB
#define DRV8301_MISO_PIN		12
#define DRV8301_SCK_GPIO		GPIOC
#define DRV8301_SCK_PIN			13
#define DRV8301_CS_GPIO			GPIOC
#define DRV8301_CS_PIN			14


// // MPU9250
// // fix to i2c
// #ifdef HW_HAS_MPU9250
// #define MPU9X50_SDA_GPIO		GPIOB
// #define MPU9X50_SDA_PIN			2
// #define MPU9X50_SCL_GPIO		GPIOA
// #define MPU9X50_SCL_PIN			15
// //#define IMU_FLIP
// #endif

// #ifdef HW_HAS_NRF_SWD
// // NRF SWD
// #define NRF5x_SWDIO_GPIO		GPIOB
// #define NRF5x_SWDIO_PIN			12
// #define NRF5x_SWCLK_GPIO		GPIOA
// #define NRF5x_SWCLK_PIN			4
// #endif

// Measurement macros
#define ADC_V_L1				ADC_Value[ADC_IND_SENS1]
#define ADC_V_L2				ADC_Value[ADC_IND_SENS2]
#define ADC_V_L3				ADC_Value[ADC_IND_SENS3]
#define ADC_V_ZERO				(ADC_Value[ADC_IND_VIN_SENS] / 2)

// Macros
#define READ_HALL1()			palReadPad(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1)
#define READ_HALL2()			palReadPad(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2)
#define READ_HALL3()			palReadPad(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3)

// Default setting overrides
#ifndef MCCONF_DEFAULT_MOTOR_TYPE
#define MCCONF_DEFAULT_MOTOR_TYPE		MOTOR_TYPE_FOC
#endif
#ifndef MCCONF_L_MAX_ABS_CURRENT
#define MCCONF_L_MAX_ABS_CURRENT		100.0	// The maximum absolute current above which a fault is generated
#endif
#ifndef MCCONF_FOC_SAMPLE_V0_V7
#define MCCONF_FOC_SAMPLE_V0_V7			false	// Run control loop in both v0 and v7 (requires phase shunts)
#endif

// Setting limits
#define HW_LIM_CURRENT			-110.0, 110.0
#define HW_LIM_CURRENT_IN		-40.0, 60.0
#define HW_LIM_CURRENT_ABS		0.0, 160.0
#define HW_LIM_VIN				11.0, 70.0
#define HW_LIM_ERPM				-100e3, 100e3
#define HW_LIM_DUTY_MIN			0.0, 0.5
#define HW_LIM_DUTY_MAX			0.0, 0.95
#define HW_LIM_TEMP_FET			-40.0, 90.0

#endif /* HW_60_H_ */
