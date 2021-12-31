
#ifndef SETUP_H_
#define SETUP_H_
/*
 * Setup.h file
*/
// This is left here just to have it handy for copying when debugging a specific function
// Don't uncomment!!
//               __attribute__((optimize("O0")))

/* Private defines -----------------------------------------------------------*/
#define EE_SCL_Pin GPIO_PIN_0
#define EE_SCL_GPIO_Port GPIOA
#define EE_SDA_Pin GPIO_PIN_1
#define EE_SDA_GPIO_Port GPIOA
#define TIP_Pin GPIO_PIN_2
#define TIP_GPIO_Port GPIOA
#define BUZ0_Pin GPIO_PIN_3
#define BUZ0_GPIO_Port GPIOA
#define BUZ1_Pin GPIO_PIN_4
#define BUZ1_GPIO_Port GPIOA
#define BUZ2_Pin GPIO_PIN_5
#define BUZ2_GPIO_Port GPIOA
#define WAKE_Pin GPIO_PIN_6
#define WAKE_GPIO_Port GPIOA
#define NTC_Pin GPIO_PIN_7
#define NTC_GPIO_Port GPIOA
#define PWM_Pin GPIO_PIN_0
#define PWM_GPIO_Port GPIOB
#define VIN_Pin GPIO_PIN_1
#define VIN_GPIO_Port GPIOB
#define HW_SCL_Pin GPIO_PIN_10
#define HW_SCL_GPIO_Port GPIOB
#define HW_SDA_Pin GPIO_PIN_11
#define HW_SDA_GPIO_Port GPIOB
#define SW_SDA_Pin GPIO_PIN_12
#define SW_SDA_GPIO_Port GPIOB
#define SW_SCL_Pin GPIO_PIN_13
#define SW_SCL_GPIO_Port GPIOB
#define ENC_SW_Pin GPIO_PIN_15
#define ENC_SW_GPIO_Port GPIOA
#define ENC_R_Pin GPIO_PIN_3
#define ENC_R_GPIO_Port GPIOB
#define ENC_L_Pin GPIO_PIN_4
#define ENC_L_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define HWSTRING "HW: KSGER21F410"
/********************************
 * 			PWM Settings        *
 ********************************/
#define READ_TIMER			  htim5			                    // Timer for the dead time
#define PWM_TIMER 			  htim1			                    // PWM Timer
#define PWM_CHANNEL 		  TIM_CHANNEL_2	                // PWM Timer Channel
#define PWM_CHxN							                        // Using CHxN Output type
//#define PWM_CHx								                          // Using CHx Output type

/********************************
 *       Display Settings    *
 ********************************/
//#define OLED_SPI                                            // SPI display
#define OLED_I2C                                              // I2C display
//#define I2C_TRY_HW                                          // Try I2C HW first, use I2C SW if not detected
#define OLED_ADDRESS        (0x3c<<1)                         // Only used for I2C
//#define OLED_DEVICE         hi2c2                           // SPI / I2C handler if used. Enables HW mode, otherwise SW mode is used
#define FILL_DMA            hdma_memtomem_dma2_stream0       // DMA mem2mem for filling
//#define USE_RST                                             // Reset pin is used
//#define USE_DC                                              // DC pin is used
//#define USE_CS                                              // CS pin is used
#define OLED_OFFSET         2                                 // Display offset


/********************************
 *       ADC Settings        *
 ********************************/
#define ADC_DEVICE          hadc1                             // ADC device
#define ADC_BFSIZ           (16+2)                            // ADC DMA buffer size Buffer[ADC_BFSIZ][Adc_Buffer_Elements](+2 to compensate min/max value discard in filtering)

// Order for secondary measurements, ADC channels not requiring sampling in the PWM low period. Order as ADC rank order (usually ch0-ch18)
#define ADC_1st             TIP                               // ADC 1st used channel (CH2)
#define ADC_2nd             NTC                               // ADC 2nd used channel (CH7)
#define ADC_3rd             VIN                               // ADC 3nd used channel (CH9)
#define ADC_4th             INT_TMP                           // ADC 4th used channel (CH16)
//#define ADC_5th           VREF                              // ADC 5th used channel
#define ADC_Num             4                                 // Number of channels

// Channel assignment
#define ADC_CH_1ST          ADC_CHANNEL_2                     // CH2 = IRON TIP
#define ADC_CH_2ND          ADC_CHANNEL_7                     // CH7 = NTC
#define ADC_CH_3RD          ADC_CHANNEL_9                     // CH9 = VIN
#define ADC_CH_4TH          ADC_CHANNEL_TEMPSENSOR            // CH16 = INT TEMP
//#define ADC_CH_5TH        ADC_CHANNEL__                     // CH? = ?

// To enable specific functions in code
//#define USE_VREF
#define USE_VIN
#define USE_NTC
#define ENABLE_INT_TEMP                                       // Enable internal temperature if NTC fails or disabled in options (Depends on USE_NTC)

/********************************
 *       TIP calibration    *
 ********************************/
#define PROFILE_VALUES                                        // Enable profile values

#ifdef PROFILE_VALUES
#define T12_Cal250        1900
#define T12_Cal400        2800

#define C210_Cal250       450
#define C210_Cal400       700

#define C245_Cal250       900
#define C245_Cal400       1000
#endif

/********************************
 * 			Buzzer				*
 ********************************/
#define BUZZER_OFF 			  HAL_GPIO_WritePin(BUZ0_GPIO_Port, BUZ0_Pin, GPIO_PIN_SET);  \
							            HAL_GPIO_WritePin(BUZ1_GPIO_Port, BUZ1_Pin, GPIO_PIN_SET);  \
							            HAL_GPIO_WritePin(BUZ2_GPIO_Port, BUZ2_Pin, GPIO_PIN_SET);

#define BUZZER_ON 			  HAL_GPIO_WritePin(BUZ0_GPIO_Port, BUZ0_Pin, GPIO_PIN_RESET);  \
                          HAL_GPIO_WritePin(BUZ1_GPIO_Port, BUZ1_Pin, GPIO_PIN_RESET);  \
                          HAL_GPIO_WritePin(BUZ2_GPIO_Port, BUZ2_Pin, GPIO_PIN_RESET);

#define BUZZER_TOGGLE 	  HAL_GPIO_TogglePin(BUZ0_GPIO_Port, BUZ0_Pin); \
							            HAL_GPIO_TogglePin(BUZ1_GPIO_Port, BUZ1_Pin); \
							            HAL_GPIO_TogglePin(BUZ2_GPIO_Port, BUZ2_Pin);

/********************************
 *       Misc    *
 ********************************/
//#define NOSAVESETTINGS                                      // Don't use flash to save or load settings. Always use defaults (for debugging purposes)
//#define SWO_PRINT                                           // To enable printing through SWO


#ifdef USE_NTC

#define NTC_RES       10000
#define NTC_BETA      3450
#define PULL_RES      4700
#define PULLUP

#endif

// To stop peripherals when debugging
#define DebugOpts()         __HAL_DBGMCU_FREEZE_IWDG(); \
                            __HAL_DBGMCU_FREEZE_TIM1(); \
                            __HAL_DBGMCU_FREEZE_TIM5()

#endif