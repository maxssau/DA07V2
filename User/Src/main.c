
#include "usb_audio.h"
#include "audio_configuration.h"
#include "usbd_audio_if.h"
#include "usbd_desc.h"
#include "stm32f4xx_hal_i2c.h"


#define __debug 1

#define DSP_TIMEOUT 40

#ifdef __debug

int32_t __debug_gpiob_moder;
int32_t __debug_gpiob_otyper;
int32_t __debug_gpiob_ospeedr;
int32_t __debug_gpiob_pupdr;
int32_t __debug_gpiob_afr0;
int32_t __debug_gpiob_afr1;

int32_t __debug_gpioa_moder;
int32_t __debug_gpioa_otyper;
int32_t __debug_gpioa_ospeedr;
int32_t __debug_gpioa_pupdr;
int32_t __debug_gpioa_afr0;
int32_t __debug_gpioa_afr1;

int32_t __debug_rcc_ahb1enr;
int32_t __debug_rcc_ahb2enr;
int32_t __debug_rcc_apb1enr;
int32_t __debug_rcc_cr;
int32_t __debug_rcc_cfgr;
int32_t __debug_rcc_pllcfgr;
int32_t __debug_i2c3_cr1;
int32_t __debug_i2c3_cr2;
int32_t __debug_i2c3_ccr;
int32_t __debug_i2c3_trise;
int32_t __debug_i2c3_dr;
int32_t __debug_i2c3_oar1;
int32_t __debug_i2c3_oar2;
int32_t __debug_i2c3_sr1;
int32_t __debug_i2c3_sr2;

int32_t __debug_time3_ccer;
int32_t __debug_time3_ccmr1;
int32_t __debug_time3_smcr;
int32_t __debug_time3_cr1;
int32_t __debug_time3_cr2;
int32_t __debug_time3_sr;

#endif


int LCD_NeedUpdate=1;
uint32_t LCD_I2S_AudioFrequency;
uint8_t LCD_I2S_AudioResolution;

int ERR_I2C=0;

USBD_HandleTypeDef USBD_Device;
extern USBD_AUDIO_InterfaceCallbacksfTypeDef audio_class_interface;

// i2c
I2C_HandleTypeDef hi2c3;
uint32_t __encoder_data,__encoder_data_last=1;

void SystemClock_Config(void);
void Init_Encoder(void);
void LCD_Update(void);
void GPIO_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM3_Init(void);
void SIGMA_WRITE_REGISTER_BLOCK(int devAddress, int address, int length, uint8_t *pData);
void SIGMA_WRITE_REGISTER_BLOCK1(int devAddress, int address, int length, uint8_t *pData);
void DSP_Update(void);
void default_download_IC96(void);
void default_download_IC48(void);

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_Encoder_InitTypeDef encoder;
TIM_HandleTypeDef timer;

int err_count;

#include "lcd_1306.h"
#include "da07v2_96_IC96.h"
#include "da07v2_48_IC48.h"

void SIGMA_WRITE_REGISTER_BLOCK1(int devAddress, int address, int length, uint8_t *pData)
{
	uint8_t __buff[6144];
        int i=0;
	while(i<length)
	{
                i++;
		__buff[i+2]=pData[i];
	}
	__buff[0]=(address & 0xFF00)>>8;
	__buff[1]=address & 0x00FF;

	while(HAL_I2C_Master_Transmit(&hi2c3, devAddress,__buff,length+2, DSP_TIMEOUT)!=HAL_OK)
	{
			err_count++;
			if(err_count>3)
			{
				break;
			}
	}
}

void SIGMA_WRITE_REGISTER_BLOCK(int devAddress, int address, int length, uint8_t *pData)
{
        uint32_t tickstart = 0x00U;
        tickstart = HAL_GetTick();
        
        err_count=0;
        int result=0;
        
        I2C3->CR1 |= I2C_CR1_START;
        I2C3->CR1 |= I2C_CR1_ACK;
        while (!(I2C3->SR1 & I2C_SR1_SB))
        {
          if(HAL_GetTick()-tickstart>DSP_TIMEOUT)
          { 
            err_count++;
            if(err_count>10)
            {
              I2C3->CR1 |= I2C_CR1_STOP;
              return;
            }
          }
        };
        
        result=I2C3->SR1;
        
        /*if(result && 1<<14)
        {
          I2C3->CR1 |= I2C_CR1_STOP;
          return;
        }
        
        if(result && 1<<10)
        {
          I2C3->CR1 |= I2C_CR1_STOP;
          return;
        }*/
        
        result=I2C3->SR2;
  
        I2C3->DR = devAddress;
        err_count=0;
        while (!(I2C3->SR1 & I2C_SR1_ADDR))
        {
          if(HAL_GetTick()-tickstart>DSP_TIMEOUT)
          {
            err_count++;
            if(err_count>10)
            {
              I2C3->CR1 |= I2C_CR1_STOP;
              return;
            }
          }
        };
        (void) I2C3->SR1;
        (void) I2C3->SR2;
        
        err_count=0;
        I2C3->DR = (address & 0xFF00)>>8;	
        while(!(I2C3->SR1 & I2C_SR1_BTF))
        {
          if(HAL_GetTick()-tickstart>DSP_TIMEOUT)
          {
            err_count++;
            ERR_I2C++;
            if(err_count>10)
            {
              I2C3->CR1 |= I2C_CR1_STOP;
              return;
            }
          }
        };
        err_count=0;
        I2C3->DR = address & 0x00FF;	
        while(!(I2C3->SR1 & I2C_SR1_BTF))
        {
          if(HAL_GetTick()-tickstart>DSP_TIMEOUT)
          {
            err_count++;
            ERR_I2C++;
            if(err_count>10)
            {
              I2C3->CR1 |= I2C_CR1_STOP;
              return;
            }
          }
        };
        
        for(uint32_t i=0;i<length;i++)
        {
          I2C3->DR = pData[i];
          err_count=0;
          while(!(I2C3->SR1 & I2C_SR1_BTF))
          {
            if(HAL_GetTick()-tickstart>DSP_TIMEOUT)
            {
              err_count++;
              ERR_I2C++;
              if(err_count>10)
              {
                I2C3->CR1 |= I2C_CR1_STOP;
                return;
              }
            }
          };
	}
	I2C3->CR1 |= I2C_CR1_STOP;

}

int main(void)
{
  HAL_Init();
  //SystemClock_Config();
  USB_I2S_Init();
  
  //GPIO_Init();
  MX_I2C3_Init();
  __HAL_RCC_I2C3_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();
  
  Init_Encoder();
  
  //MX_TIM3_Init();
  
  /*HAL_Delay(100);*/
  //default_download_IC48();
  
  OLED_init();
  LCD_Clear();
  LCD_Goto(0,1);
  OLED_string("PCM Mode");
  USBD_Init(&USBD_Device, &AUDIO_Desc, 0);
  //InitDelay();
  //USB
  //аудио конфигурация
  AudioConfig_Init();
  OUTClk_Init();
  //USB
  
  USBD_RegisterClass(&USBD_Device, USBD_AUDIO_CLASS);
  USBD_AUDIO_RegisterInterface(&USBD_Device, &audio_class_interface);
  USBD_Start(&USBD_Device);
  
  while (1)
  {
    __WFI();
    if(LCD_NeedUpdate==1)
    {
      LCD_NeedUpdate=0;
      LCD_Update();
      DSP_Update();
    };
    
    // encoder
    
    __encoder_data = TIM3->CNT;
    if(__encoder_data!=__encoder_data_last)
    {
      __encoder_data_last=__encoder_data;
      char __buff[128];
      sprintf(__buff,"Enc: %i    ",__encoder_data);
      LCD_Goto(0,3);
      OLED_string(__buff);
    }
    
  }
}

void Init_Encoder()
{
  //timer 3 setup
  //PC6

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
  TIM3->CR1=0;
  TIM3->CCMR1=0;
  TIM3->CCER=0;
  TIM3->SMCR=0;
  
  TIM3->CR1|=(10<<8);
  
  TIM3->CCMR1 |= TIM_CCMR1_IC1F | TIM_CCMR1_IC2F;
  
  TIM3->CCER |= TIM_CCER_CC1P;
  TIM3->CCER |= TIM_CCER_CC2P;

  TIM3->CCMR1 |= TIM_CCMR1_CC1S_0;
  TIM3->CCMR1 |= TIM_CCMR1_CC2S_0;
  TIM3->SMCR |= 3;

  TIM3->ARR = 65535;
  TIM3->EGR=1;
  TIM3->CR1 |= TIM_CR1_CEN;

  
  
#ifdef __debug
  __debug_time3_ccer=TIM3->CCER;
  __debug_time3_ccmr1=TIM3->CCMR1;
  __debug_time3_smcr=TIM3->SMCR;
  __debug_time3_cr1=TIM3->CR1;
  __debug_time3_cr2=TIM3->CR2;
  __debug_time3_sr=TIM3->SR;
#endif  
  
}

static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  //htim3.Init.AutoReloadPreload = 0;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

//------------------------------------------------------------------------------
void USB_I2S_Init(void)
{
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  HAL_InitTick(TICK_INT_PRIORITY);
  
  //_HAL_RCC_PWR_CLK_ENABLE();
  
  //HSE, PLL 168 MHz
  FLASH->ACR = FLASH_ACR_LATENCY_5WS | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;
  //RCC->APB1ENR|=RCC_APB1ENR_PWREN;
  RCC->CR |= RCC_CR_HSEON;
  while((RCC->CR & RCC_CR_HSERDY) == 0);
    
  RCC->PLLCFGR = RCC_PLLCFGR_PLLR_1 | RCC_PLLCFGR_PLLQ_0 | RCC_PLLCFGR_PLLQ_1 | RCC_PLLCFGR_PLLQ_2 | \
                 RCC_PLLCFGR_PLLSRC | RCC_PLLCFGR_PLLN_4 | RCC_PLLCFGR_PLLN_6 | RCC_PLLCFGR_PLLN_8 | RCC_PLLCFGR_PLLM_3;
    
  RCC->CFGR = RCC_CFGR_PPRE2_DIV2 | RCC_CFGR_PPRE1_DIV4;
  
  RCC->CFGR|=(1<<7);
  //RCC->CFGR = RCC_CFGR_PPRE2_DIV4 | RCC_CFGR_PPRE1_DIV8;
    
  RCC->CR |= RCC_CR_PLLON;
  while((RCC->CR & RCC_CR_PLLRDY) == 0);
  
  RCC->CFGR |= RCC_CFGR_SW_PLL;
  while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)RCC_CFGR_SWS_PLL);
  
  //добавить для всех используемых выводов
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;
  while(((RCC->AHB1ENR & RCC_AHB1ENR_GPIOAEN) != RCC_AHB1ENR_GPIOAEN)
     || ((RCC->AHB1ENR & RCC_AHB1ENR_GPIOBEN) != RCC_AHB1ENR_GPIOBEN)
     || ((RCC->AHB1ENR & RCC_AHB1ENR_GPIOCEN) != RCC_AHB1ENR_GPIOCEN));
  
  RCC->APB1ENR|=(1<<RCC_APB1RSTR_I2C3RST_Pos);
  RCC->APB1ENR|=(1<<RCC_APB1ENR_TIM3EN);
  
  
#ifdef __debug
  
  __debug_rcc_ahb1enr=RCC->AHB1ENR;
  __debug_rcc_ahb2enr=RCC->AHB2ENR;
  __debug_rcc_apb1enr=RCC->APB1ENR;
  __debug_rcc_cr=RCC->CR;
  __debug_rcc_cfgr=RCC->CFGR;
  __debug_rcc_pllcfgr=RCC->PLLCFGR;
  
#endif  
  
  //RCC->APB1LPENR|=(1<<RCC_APB1RSTR_I2C3RST_Pos);
  
  //GPIOB->MODER|=GPIO_MODER_MODER_8_1;
  
  //SystemClock_Config();
    
  //выводы конфигурации
  ConfigGPIOs_Init();
  //задержка, если нужна
  
}

//------------------------------------------------------------------------------
void Error_Handler(void)
{
   
}
//------------------------------------------------------------------------------
void USBD_error_handler(void)
{
  Error_Handler();
}
static void MX_I2C3_Init(void)
{
  
  RCC->AHB1ENR |= 3; //set GPIOB clock enable bit
  RCC->AHB1ENR |= 2; //set GPIOB clock enable bit
  RCC->APB1ENR |= 1 << (23); //set I2C3 clock enable bit
  
  // PB4 - SDA
  
  GPIOB->MODER &= ~(3 << (4 * 2));     // clear bit field using 2 bit mask
  GPIOB->MODER |= 2 << (4 * 2); //set bit filds to 0b10 to enable alternate function mode
  GPIOB->OTYPER |= (1 << (4));
  GPIOB->OSPEEDR &= ~(3 << (4 * 2)); //clear
  GPIOB->OSPEEDR |= (2 << (4 * 2)); //fast speed - 50MHz
  GPIOB->PUPDR &= ~(3 << (4 * 2)); //clear
  GPIOB->PUPDR |= (1 << (4 * 2)); // pull up
  GPIOB->AFR[0] &= ~(15 << (4*4)); //clear field
  GPIOB->AFR[0] |= (4 << (4*4)); //set for alternate function 4 (I2C 1)
  
  
  // PA8 - SDA
  
  
  GPIOA->MODER &= ~(3 << (8 * 2));     // clear bit field using 2 bit mask
  GPIOA->MODER |= 2 << (8 * 2); //set bit filds to 0b10 to enable alternate function mode
  GPIOA->OTYPER |= (1 << (8));
  GPIOA->OSPEEDR &= ~(3 << (8 * 2)); //clear
  GPIOA->OSPEEDR |= (2 << (8 * 2)); //fast speed - 50MHz
  GPIOA->PUPDR &= ~(3 << (8 * 2)); //clear
  GPIOA->PUPDR |= (1 << (8 * 2)); // pull up
  GPIOA->AFR[1] &= ~(15 << (0*8)); //clear field
  GPIOA->AFR[1] |= (4 << (0*8)); //set for alternate function 4 (I2C 1)
  
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 50000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
    ERR_I2C=1;
  }

#ifdef __debug
  
  __debug_i2c3_cr1=I2C3->CR1;
  __debug_i2c3_cr2=I2C3->CR2;
  __debug_i2c3_ccr=I2C3->CCR;
  __debug_i2c3_trise=I2C3->TRISE;
  __debug_i2c3_dr=I2C3->DR;
  __debug_i2c3_oar1=I2C3->OAR1;
  __debug_i2c3_oar2=I2C3->OAR2;
  __debug_i2c3_sr1=I2C3->SR1;
  __debug_i2c3_sr2=I2C3->SR2;

  __debug_gpiob_moder=GPIOB->MODER;
  __debug_gpiob_otyper=GPIOB->OTYPER;
  __debug_gpiob_ospeedr=GPIOB->OSPEEDR;
  __debug_gpiob_pupdr=GPIOB->PUPDR;
  __debug_gpiob_afr0=GPIOB->AFR[0];
  __debug_gpiob_afr1=GPIOB->AFR[1];

  __debug_gpioa_moder=GPIOA->MODER;
  __debug_gpioa_otyper=GPIOA->OTYPER;
  __debug_gpioa_ospeedr=GPIOA->OSPEEDR;
  __debug_gpioa_pupdr=GPIOA->PUPDR;
  __debug_gpioa_afr0=GPIOA->AFR[0];
  __debug_gpioa_afr1=GPIOA->AFR[1];
  
#endif

}

void LCD_Update(void)
{
  LCD_Goto(0,0);
  char __buff[128];
  sprintf(__buff,"%i bit %i kHz",LCD_I2S_AudioResolution,LCD_I2S_AudioFrequency);
  OLED_string(__buff);
}

void GPIO_Init(void)
{
  /*__HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();*/
  
  RCC->AHB1ENR |= 3; //set GPIOB clock enable bit
  RCC->APB1ENR |= 1 << (23); //set I2C3 clock enable bit
  
  // PB4 - SDA
  
  GPIOB->MODER &= ~(3 << (4 * 2));     // clear bit field using 2 bit mask
  GPIOB->MODER |= 2 << (4 * 2); //set bit filds to 0b10 to enable alternate function mode
  GPIOB->OTYPER |= (1 << (4));
  GPIOB->OSPEEDR &= ~(3 << (4 * 2)); //clear
  GPIOB->OSPEEDR |= (2 << (4 * 2)); //fast speed - 50MHz
  GPIOB->PUPDR &= ~(3 << (4 * 2)); //clear
  GPIOB->PUPDR |= (1 << (4 * 2)); // pull up
  GPIOB->AFR[0] &= ~(15 << (4*4)); //clear field
  GPIOB->AFR[0] |= (4 << (4*4)); //set for alternate function 4 (I2C 1)
  
  
  // PA8 - SDA
  
  
  GPIOA->MODER &= ~(3 << (8 * 2));     // clear bit field using 2 bit mask
  GPIOA->MODER |= 2 << (8 * 2); //set bit filds to 0b10 to enable alternate function mode
  GPIOA->OTYPER |= (1 << (8));
  GPIOA->OSPEEDR &= ~(3 << (8 * 2)); //clear
  GPIOA->OSPEEDR |= (2 << (8 * 2)); //fast speed - 50MHz
  GPIOA->PUPDR &= ~(3 << (8 * 2)); //clear
  GPIOA->PUPDR |= (1 << (8 * 2)); // pull up
  GPIOA->AFR[1] &= ~(15 << (0*8)); //clear field
  GPIOA->AFR[1] |= (4 << (0*8)); //set for alternate function 4 (I2C 1)
  
  I2C3->CR1 &=~ (1 << 0); //Peripheral enable bit
  //set clock frequency to 100kHz
  I2C3->CR2 &=~ (63);
  I2C3->CR2 |= (8); //set I2C peripheral frequency to 8MHz
  I2C3->CCR &=~ (1 < 15); //clear the 15th bit to set it to Sm mode

  I2C3->CCR &=~ (4095); //clear the lowest 12 bits
  I2C3->CCR |= (0x28); //clear the lowest 12 bits

  //Set TRISE
  //TRise is the maximum rise time divided by the peripheral clock period + 1
  //TRISE = floor(Trise_max/Tpclk)+1
  //For Sm mode the max rise time is 1000ns
  //thus we have ((1000E-9/(1/8E6))+1) = 9
  I2C3->TRISE = 9; //set TRISE to 9

  //Program the I2C_CR1 register to enable the peripheral
  I2C3->CR1 |= (1 << 0); //Peripheral enable bit
  /*
  GPIO_InitTypeDef GPIO_InitStruct_sda = {0};
  GPIO_InitStruct_sda.Pin = GPIO_PIN_4;
  GPIO_InitStruct_sda.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct_sda.Pull = GPIO_PULLUP;
  GPIO_InitStruct_sda.Speed = GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct_sda.Alternate = GPIO_AF4_I2C3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct_sda);
  
  GPIO_InitTypeDef GPIO_InitStruct_scl = {0};
  GPIO_InitStruct_scl.Pin = GPIO_PIN_8;
  GPIO_InitStruct_scl.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct_scl.Pull = GPIO_PULLUP;
  GPIO_InitStruct_scl.Speed = GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct_scl.Alternate = GPIO_AF4_I2C3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct_scl);*/
  
  /*GPIO_InitTypeDef GPIO_InitStruct_ckin = {0};
  GPIO_InitStruct_ckin.Pin = GPIO_PIN_9;
  GPIO_InitStruct_ckin.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct_ckin.Pull = GPIO_NOPULL;
  GPIO_InitStruct_ckin.Speed = GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct_ckin.Alternate = 0x05;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct_ckin);*/
  
}

void DSP_Update()
{
  /*
    int LCD_NeedUpdate=0;
    uint32_t LCD_I2S_AudioFrequency;
    uint8_t LCD_I2S_AudioResolution;
  */
  switch(LCD_I2S_AudioFrequency)
  {
    case 48000:
    {
      default_download_IC48();
    }
    case 96000:
    {
      //default_download_IC96();
    }
    break;
  }
  
  
  
}