/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include "MPU6050.h"
#include "HMC5883.h"
#include "I2C.h"
#include "util.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM11_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
#define SBI(address, ab) ( address |= (0x01<<ab) )
#define CBI(address, ab) ( address &= ~(0x01<<ab) )

int _write(int file,uint8_t* p, int len){
	for(int i=0;i<len;i++){
		LL_USART_TransmitData8(USART1,p[i]);
		while(!LL_USART_IsActiveFlag_TXE(USART1));
	}
	return len;
}



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



I2C_DMA_struct DMA;


uint8_t i2c_dma_buf[14];
uint8_t uart4_dma_buf[33];

uint8_t condition_1ms=0;
//uint8_t condition_2ms=0;
uint8_t condition_angle=0;
uint8_t condition_uart4=0;
uint8_t condition_pid=0;
uint8_t condition_mag_baro=0;


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM11_Init();
  MX_UART4_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_SPI1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  //----------------------------- test counter
  uint16_t counter=0;
  uint16_t uart_counter=0;
  uint16_t mag_counter=0;
  uint16_t motor_counter=0;
  uint16_t battery_counter=0;

  //----------------------------- timer
  uint8_t counter_2ms=0;

  //----------------------------- unarmed condition
  uint8_t cali_condition=0;
  uint8_t is_mag_cali=0;
  uint8_t first_data=1;


  uint32_t battery_sum=0;
  uint16_t adc_val=0;
  uint16_t battery_arr[200]={0,},battery_val=0;
  uint8_t battery_count=0;
  uint8_t battery_adc_led_off=0;

  //----------------------------- mpu6050
  uint8_t mpu_reg_address=0;

  //----------------------------- mag baro
  uint8_t mag_baro=0;

  //-----------------------------



  /*hmc5883.offset_x=70.31;
  hmc5883.offset_y=36.49;
  hmc5883.offset_z=16.47;

  hmc5883.scale_x=0.99;
  hmc5883.scale_y=1.0;
  hmc5883.scale_z=1.01;
  save_mag_para();*/

  LL_GPIO_SetOutputPin(GPIOE,LL_GPIO_PIN_7);
  LL_GPIO_SetOutputPin(GPIOE,LL_GPIO_PIN_8);
  LL_GPIO_SetOutputPin(GPIOE,LL_GPIO_PIN_9);
  LL_mDelay(100);

////////////////////////////
  printf("\r\nstart\r\n");
////////////////////////////


//----------------------------------------------------------------------------------- usart1 command
  init_COMMAND(&command,USART1);

//----------------------------------------------------------------------------------- adc battery
  LL_ADC_Enable(ADC1);
  LL_ADC_REG_StartConversionSWStart(ADC1);

//----------------------------------------------------------------------------------- ibus receiver
  LL_DMA_SetMemoryAddress(DMA1,LL_DMA_STREAM_2,(uint32_t)uart4_dma_buf);
  LL_DMA_SetPeriphAddress(DMA1,LL_DMA_STREAM_2,(uint32_t)(&UART4->DR));
  LL_DMA_SetDataLength(DMA1,LL_DMA_STREAM_2,33);
  DMA1->LIFCR=0x003D0000;	//DMA1 STREAM2 CLEAR ALL FLAG
  /*LL_DMA_ClearFlag_TE2(DMA1);
  LL_DMA_ClearFlag_HT2(DMA1);
  LL_DMA_ClearFlag_TC2(DMA1);
  LL_DMA_ClearFlag_DME2(DMA1);
  LL_DMA_ClearFlag_FE2(DMA1);*/
  LL_DMA_EnableStream(DMA1,LL_DMA_STREAM_2);
  LL_USART_EnableDMAReq_RX(UART4);
  LL_USART_EnableIT_IDLE(UART4);

//----------------------------------------------------------------------------------- HMC5883
  init_HMC(&hmc5883,I2C3);
  HMC_Default_Reg(&hmc5883);

//----------------------------------------------------------------------------------- mpu6050 I2C
  I2C_Receive_DMA_init(&(mpu6050.I2C),&DMA,I2C2,DMA1,LL_DMA_STREAM_3,(uint32_t)i2c_dma_buf,LL_I2C_DMA_GetRegAddr(I2C2),14);
  init_MPU6050(&mpu6050,I2C2);
  MPU_Gyrocali(&mpu6050,&angle);
  MPU_Angcali(&mpu6050,&angle);

  mpu_reg_address=MPU6050_RA_ACCEL_XOUT_H;
  I2C_Transmit(&(mpu6050.I2C),mpu6050.gyro_address,&mpu_reg_address,1);
  I2C_Receive_DMA(&(mpu6050.I2C),&DMA, mpu6050.gyro_address);

//----------------------------------------------------------------------------------- load para
  load_pid_para();
  load_mag_para();

//----------------------------------------------------------------------------------- pwm timer
  LL_TIM_EnableIT_UPDATE(TIM2);
  LL_TIM_CC_EnableChannel(TIM2,LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM2,LL_TIM_CHANNEL_CH2);
  LL_TIM_CC_EnableChannel(TIM2,LL_TIM_CHANNEL_CH3);

  LL_TIM_CC_EnableChannel(TIM4,LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM4,LL_TIM_CHANNEL_CH2);
  LL_TIM_CC_EnableChannel(TIM4,LL_TIM_CHANNEL_CH3);



  LL_TIM_EnableCounter(TIM2);
  LL_TIM_EnableCounter(TIM4);

//----------------------------------------------------------------------------------- 1ms timer
  LL_TIM_EnableIT_UPDATE(TIM11);
  LL_TIM_EnableCounter(TIM11);

//-----------------------------------------------------------------------------------
  LL_GPIO_ResetOutputPin(GPIOE,LL_GPIO_PIN_7);
  LL_GPIO_ResetOutputPin(GPIOE,LL_GPIO_PIN_8);
  LL_GPIO_ResetOutputPin(GPIOE,LL_GPIO_PIN_9);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//=========================================================================================================================	condition
	  if(condition_1ms==1){
		  counter_2ms++;
		  //---------------------------------------------------------------------- 1ms condition
		  condition_angle=1;

		  //---------------------------------------------------------------------- 2ms condition
		  if(counter_2ms>1){
			  condition_mag_baro=1;

			  counter_2ms=0;
		  }

		  //----------------------------------------------------------------------
		  condition_1ms=0;
	  }

//========================================================================================================================= unarmed
	  else if(timer[4]<1400){
		  motor[0]=1000;
		  motor[1]=1000;
		  motor[2]=1000;
		  motor[3]=1000;
		  motor[4]=1000;
		  motor[5]=1000;
		  pitch_para.stabilize_i_mem=0;
		  pitch_para.dual_rate_i_mem=0;
		  roll_para.stabilize_i_mem=0;
		  roll_para.dual_rate_i_mem=0;
		  yaw_para.rate_i_mem=0;
		  //printf("aa\r\n");

		  //--------------------------------------------------------------------- gyro cali
		  if(LL_GPIO_IsInputPinSet(SWITCH_1_GPIO_Port,SWITCH_1_Pin) && cali_condition==0){
			  //printf("aa\r\n");
			  LL_GPIO_ResetOutputPin(GPIOE,LL_GPIO_PIN_7);
			  LL_GPIO_ResetOutputPin(GPIOE,LL_GPIO_PIN_8);
			  LL_GPIO_ResetOutputPin(GPIOE,LL_GPIO_PIN_9);
			  cali_condition=1;
			  battery_adc_led_off=1;
			  LL_mDelay(500);
			  if(LL_GPIO_IsInputPinSet(SWITCH_2_GPIO_Port,SWITCH_2_Pin)){
				  LL_GPIO_SetOutputPin(GPIOE,LL_GPIO_PIN_7);
				  LL_GPIO_SetOutputPin(GPIOE,LL_GPIO_PIN_8);
				  is_mag_cali=1;
			  }
			  else{
				  LL_GPIO_SetOutputPin(GPIOE,LL_GPIO_PIN_9);
				  MPU_Gyrocali(&mpu6050,&angle);
				  MPU_Angcali(&mpu6050,&angle);
				  LL_mDelay(500);
				  LL_GPIO_ResetOutputPin(GPIOE,LL_GPIO_PIN_9);
			  }
		  }
		  else if(LL_GPIO_IsInputPinSet(SWITCH_1_GPIO_Port,SWITCH_1_Pin)==0 && LL_GPIO_IsInputPinSet(SWITCH_2_GPIO_Port,SWITCH_2_Pin)==0 && cali_condition==1){
			  //printf("bb\r\n");
			  if(first_data==0){	//mag save
				  LL_mDelay(500);
				  save_mag_para();
				  LL_GPIO_ResetOutputPin(GPIOE,LL_GPIO_PIN_7);
				  LL_GPIO_ResetOutputPin(GPIOE,LL_GPIO_PIN_8);
			  }
			  battery_adc_led_off=0;
			  cali_condition=0;
			  is_mag_cali=0;
			  first_data=1;
		  }

		  if(is_mag_cali){
			  //printf("cc\r\n");
			  if(1){
				  float mag_max[3]={0,};
				  float mag_min[3]={0,};
				  float avg_delta[4]={0,};

				  HMC_Mag_Read(&hmc5883);
				  if(first_data){
					  mag_max[0]=mag_min[0]=hmc5883.mx;
					  mag_max[1]=mag_min[1]=hmc5883.my;
					  mag_max[2]=mag_min[2]=hmc5883.mz;
					  first_data=0;
				  }
				  else{
					  if(mag_max[0]<hmc5883.mx){
						  mag_max[0]=hmc5883.mx;
					  }
					  else if(mag_min[0]>hmc5883.mx){
						  mag_min[0]=hmc5883.mx;
					  }

					  if(mag_max[1]<hmc5883.my){
						  mag_max[1]=hmc5883.my;
					  }
					  else if(mag_min[1]>hmc5883.my){
						  mag_min[1]=hmc5883.my;
					  }

					  if(mag_max[2]<hmc5883.mz){
						  mag_max[2]=hmc5883.mz;
					  }
					  else if(mag_min[2]>hmc5883.mz){
						  mag_min[2]=hmc5883.mz;
					  }
					  hmc5883.offset_x=(mag_max[0]+mag_min[0])/2;
					  hmc5883.offset_y=(mag_max[1]+mag_min[1])/2;
					  hmc5883.offset_z=(mag_max[2]+mag_min[2])/2;

					  avg_delta[0]=(mag_max[0]-mag_min[0])/2;
					  avg_delta[1]=(mag_max[1]-mag_min[1])/2;
					  avg_delta[2]=(mag_max[2]-mag_min[2])/2;

					  avg_delta[3]=(avg_delta[0]+avg_delta[1]+avg_delta[2])/3;

					  hmc5883.scale_x=avg_delta[3]/avg_delta[0];
					  hmc5883.scale_y=avg_delta[3]/avg_delta[1];
					  hmc5883.scale_z=avg_delta[3]/avg_delta[2];
				  }
			  }
		  }

		  //--------------------------------------------------------------------- command decode
		  else if(command.command_receive){
			  //printf("echo:%s\r\n",command.command_buf);
			  COMMAND_Decode(&command);
			  command.command_receive=0;
		  }

		  // battery adc
		  if(LL_ADC_IsActiveFlag_EOCS(ADC1)){
			 LL_ADC_ClearFlag_EOCS(ADC1);
			 adc_val=LL_ADC_REG_ReadConversionData12(ADC1);

			 battery_sum+=adc_val;
			 battery_sum-=battery_arr[battery_count];
			 battery_arr[battery_count]=adc_val;
			 battery_count++;
			 if(battery_count==200)
				 battery_count=0;

			 battery_val=battery_sum/200;

			 //battery_counter++;
			 if(battery_counter>50000){
				 //printf("%d\t%d\t%d\t%d\t%d\r\n",battery_arr[0],battery_arr[1],battery_arr[2],battery_arr[3],battery_arr[4]);
				 printf("%d\t%d\t%d\n\n\r",battery_val,battery_sum,adc_val);
				 battery_counter=0;
			 }
			 LL_ADC_REG_StartConversionSWStart(ADC1);
		  }

		  if(battery_adc_led_off==0){
			  if(battery_val>2246){
				  LL_GPIO_ResetOutputPin(GPIOE,LL_GPIO_PIN_7);
				  LL_GPIO_ResetOutputPin(GPIOE,LL_GPIO_PIN_8);
			  }
			  else if(battery_val>2235){	//green
				  LL_GPIO_ResetOutputPin(GPIOE,LL_GPIO_PIN_7);
				  LL_GPIO_SetOutputPin(GPIOE,LL_GPIO_PIN_8);
			  }
			  else{		//red
				  LL_GPIO_SetOutputPin(GPIOE,LL_GPIO_PIN_7);
				  LL_GPIO_ResetOutputPin(GPIOE,LL_GPIO_PIN_8);
			  }
		  }



	  }

//========================================================================================================================= ANGLE
	  else if(condition_angle && DMA.i2c_receive_dma){
		  //LL_GPIO_TogglePin(GPIOE,LL_GPIO_PIN_10);
		  //LL_GPIO_SetOutputPin(GPIOE,LL_GPIO_PIN_2);
		  MPU_Complementary_Filter(&angle,i2c_dma_buf);

		  status_data.pitch=angle.pitch-angle.pitch_offset;
		  status_data.roll=angle.roll-angle.roll_offset;

		  status_data.pitch = (status_data.pitch * 0.9) + (status_data.pre_pitch * 0.1);
		  status_data.roll = (status_data.roll * 0.9) + (status_data.pre_roll * 0.1);
		  status_data.pre_pitch=status_data.pitch;
		  status_data.pre_roll=status_data.roll;

		  //counter++;
		  if(counter>200){
			  /*printf("%d\t%d\t%d\r\n",angle.getmpuaccx,angle.getmpuaccy,angle.getmpuaccz);
			  printf("%.2f\t%.2f\t%.2f\n\n\r",angle.f_gyx,angle.f_gyy,angle.f_gyz);*/
			  printf("pitch:%.2f\r\nroll:%.2f\n\n\r",status_data.pitch,status_data.roll);
			  counter=0;
		  }
		  mpu_reg_address=MPU6050_RA_ACCEL_XOUT_H;
		  I2C_Transmit(&mpu6050.I2C,mpu6050.gyro_address,&mpu_reg_address,1);
		  I2C_Receive_DMA(&mpu6050.I2C,&DMA,mpu6050.gyro_address);
		  condition_angle=0;
		  //LL_GPIO_ResetOutputPin(GPIOE,LL_GPIO_PIN_2);
	  }

//========================================================================================================================= PID
	  else if(condition_pid){
		  float input_yaw;
		  float yaw_limit;
		  //LL_GPIO_TogglePin(GPIOE,LL_GPIO_PIN_10);

		  //LL_GPIO_SetOutputPin(GPIOB,LL_GPIO_PIN_1);

		  //---------------------------------------------------------------------------------------------------------------------- pitch
		  DUAL_PID(&pitch_para, -status_data.set_pitch, status_data.pitch, -angle.f_gyy);

		  //---------------------------------------------------------------------------------------------------------------------- roll
		  DUAL_PID(&roll_para, status_data.set_roll, status_data.roll, angle.f_gyx);

		  //---------------------------------------------------------------------------------------------------------------------- yaw
		  if(status_data.set_yaw>0){
			  if(status_data.yaw>status_data.set_yaw || (status_data.set_yaw-180)>status_data.yaw){		//yaw_angle ccw(positive value)
				  if(status_data.yaw>0){
					  input_yaw=status_data.yaw;
				  }
				  else{
					  input_yaw=status_data.yaw+360.0;
				  }
			  }
			  else{		//yaw_angle cw(negative value)
				  input_yaw=status_data.yaw;
			  }
		  }
		  else{
			  if(status_data.set_yaw>status_data.yaw || status_data.yaw>(status_data.set_yaw+180)){		//yaw_angle cw(negative value)
				  if(status_data.yaw>0){
					  input_yaw=status_data.yaw-360.0;
				  }
				  else{
					  input_yaw=status_data.yaw;
				  }
			  }
			  else{		//yaw_angle ccw(positive value)
				  input_yaw=status_data.yaw;
			  }
		  }


		  NORMAL_PID(&yaw_para,status_data.set_yaw,input_yaw);
		  yaw_limit=((float)(status_data.throttle-1000)*0.7)+50.0;
		  if(yaw_para.pid_result>yaw_limit)
			  yaw_para.pid_result=yaw_limit;
		  else if(yaw_para.pid_result<-yaw_limit)
			  yaw_para.pid_result=-yaw_limit;

		  //---------------------------------------------------------------------------------------------------------------------- motor update

		  if(status_data.throttle>1000){
			  motor[0]=status_data.throttle+(int32_t)(pitch_para.dual_pid_result+roll_para.dual_pid_result-yaw_para.pid_result);	//ccw
			  motor[1]=status_data.throttle+(int32_t)(pitch_para.dual_pid_result-roll_para.dual_pid_result+yaw_para.pid_result);	//cw
			  motor[2]=status_data.throttle+(int32_t)(roll_para.dual_pid_result+yaw_para.pid_result);	//cw
			  motor[3]=status_data.throttle+(int32_t)(-roll_para.dual_pid_result-yaw_para.pid_result);	//ccw
			  motor[4]=status_data.throttle+(int32_t)(-pitch_para.dual_pid_result+roll_para.dual_pid_result-yaw_para.pid_result);	//ccw
			  motor[5]=status_data.throttle+(int32_t)(-pitch_para.dual_pid_result-roll_para.dual_pid_result+yaw_para.pid_result);	//cw

			  for(uint8_t i=0;i<6;i++){
				  if(motor[i]<1000)
					  motor[i]=1000;
				  else if(motor[i]>1900)
					  motor[i]=1900;
			  }
		  }
		  else{
			  motor[0]=motor[1]=motor[2]=motor[3]=motor[4]=motor[5]=1000;
		  }


		  //motor_counter++;
		  if(motor_counter>80){
			  /*printf("0:%d\t1:%d\t2:%d\r\n",motor[0],motor[1],motor[2]);
			  printf("3:%d\t4:%d\t5:%d\n\n\r",motor[3],motor[4],motor[5]);*/
			  printf("%.2f\r\n",yaw_limit);
			  printf("%.2f\r\n",status_data.set_yaw-input_yaw);
			  //printf("pitch:%.2f\troll:%.2f\r\n",status_data.pitch,status_data.roll);
			  printf("set:%.2f\t%.2f\t%.2f\n\n\r",status_data.set_pitch,status_data.set_roll,status_data.set_yaw);
			  motor_counter=0;
		  }

		  condition_pid=0;
		  //LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_1);
	  }

//========================================================================================================================= mag, baro
	  else if(condition_mag_baro){
		  if(mag_baro==0){
			  float radpitch, radroll;
			  float xh,yh;

			  HMC_Mag_Read(&hmc5883);
			  hmc5883.mx=(hmc5883.mx-hmc5883.offset_x)*hmc5883.scale_x;
			  hmc5883.my=(hmc5883.my-hmc5883.offset_y)*hmc5883.scale_y;
			  hmc5883.mz=(hmc5883.mz-hmc5883.offset_z)*hmc5883.scale_z;

			  radroll=-status_data.pitch*0.01745329252;
			  radpitch=-status_data.roll*0.01745329252;

			  xh=hmc5883.mx*cosf(radpitch) + hmc5883.my*sinf(radroll)*sinf(radpitch) + hmc5883.mz*sinf(radpitch)*cosf(radroll);
			  yh=hmc5883.my*cosf(radroll) + hmc5883.mz*sinf(radroll);
			  status_data.yaw=atan2f(-yh,-xh)*(57.29577951);

			  //mag_counter++;
			  if(mag_counter>50){
				  printf("%.2f\n\n\r",status_data.yaw);
				  //printf("%.2f\t%.2f\t%.2f\r\n",hmc5883.mx,hmc5883.my,hmc5883.mz);
				  //printf("%.2f\t%.2f\t%.2f\r\n",hmc5883.offset_x,hmc5883.offset_y,hmc5883.offset_z);
				  //printf("%.2f\t%.2f\t%.2f\n\n\r",hmc5883.scale_x,hmc5883.scale_y,hmc5883.scale_z);
				  mag_counter=0;
			  }

			  /*if(first_data){
				  mag_max[0]=mag_min[0]=hmc5883.mx;
				  mag_max[1]=mag_min[1]=hmc5883.my;
				  mag_max[2]=mag_min[2]=hmc5883.mz;
				  first_data=0;
			  }

			  else{
				  if(mag_max[0]<hmc5883.mx){
					  mag_max[0]=hmc5883.mx;
				  }
				  else if(mag_min[0]>hmc5883.mx){
					  mag_min[0]=hmc5883.mx;
				  }

				  if(mag_max[1]<hmc5883.my){
					  mag_max[1]=hmc5883.my;
				  }
				  else if(mag_min[1]>hmc5883.my){
					  mag_min[1]=hmc5883.my;
				  }

				  if(mag_max[2]<hmc5883.mz){
					  mag_max[2]=hmc5883.mz;
				  }
				  else if(mag_min[2]>hmc5883.mz){
					  mag_min[2]=hmc5883.mz;
				  }
				  hmc5883.offset_x=(mag_max[0]+mag_min[0])/2;
				  hmc5883.offset_y=(mag_max[1]+mag_min[1])/2;
				  hmc5883.offset_z=(mag_max[2]+mag_min[2])/2;

				  avg_delta[0]=(mag_max[0]-mag_min[0])/2;
				  avg_delta[1]=(mag_max[1]-mag_min[1])/2;
				  avg_delta[2]=(mag_max[2]-mag_min[2])/2;

				  avg_delta[3]=(avg_delta[0]+avg_delta[1]+avg_delta[2])/3;

				  hmc5883.scale_x=avg_delta[3]/avg_delta[0];
				  hmc5883.scale_y=avg_delta[3]/avg_delta[1];
				  hmc5883.scale_z=avg_delta[3]/avg_delta[2];
			  }*/

			  mag_baro=1;
		  }
		  else{


			  mag_baro=0;
		  }
		  condition_mag_baro=0;
	  }

//========================================================================================================================= ibus receiver
	  if(condition_uart4){
		  LL_DMA_DisableStream(DMA1,LL_DMA_STREAM_2);
		  DMA1->LIFCR=0x003D0000;	//DMA1 STREAM2 CLEAR ALL FLAG
		  /*LL_DMA_ClearFlag_TE2(DMA1);
		  LL_DMA_ClearFlag_HT2(DMA1);
		  LL_DMA_ClearFlag_TC2(DMA1);
		  LL_DMA_ClearFlag_DME2(DMA1);
		  LL_DMA_ClearFlag_FE2(DMA1);*/
		  LL_DMA_EnableStream(DMA1,LL_DMA_STREAM_2);

		  if(uart4_dma_buf[0]==0x20 && uart4_dma_buf[1]==0x40){
			  timer[0]=(uint32_t)(uart4_dma_buf[7]<<8) + uart4_dma_buf[6];
			  timer[1]=(uint32_t)(uart4_dma_buf[5]<<8) + uart4_dma_buf[4];
			  timer[2]=(uint32_t)(uart4_dma_buf[9]<<8) + uart4_dma_buf[8];
			  timer[3]=(uint32_t)(uart4_dma_buf[3]<<8) + uart4_dma_buf[2];
			  timer[4]=(uint32_t)(uart4_dma_buf[11]<<8) + uart4_dma_buf[10];
			  timer[5]=(uint32_t)(uart4_dma_buf[13]<<8) + uart4_dma_buf[12];
		  }
		  //---------------------------------------------------------------------- throttle
		  if(timer[0]<1015)
			  status_data.throttle=1000;
		  else
			  status_data.throttle=(((timer[0]-1000)*70)/100)+1000;

		  //---------------------------------------------------------------------- pitch
		  if(timer[1]>1489 && timer[1]<1511)
			  timer[1]=1500;
		  else if(timer[1]>2000)
			  timer[1]=2000;
		  else if(timer[1]<1000)
			  timer[1]=1000;

		  status_data.set_pitch=((float)timer[1]-1500)/30;	//	max: 500/30

		  //---------------------------------------------------------------------- roll
		  if(timer[3]>1489 && timer[3]<1511)
			  timer[3]=1500;
		  else if(timer[3]>2000)
			  timer[3]=2000;
		  else if(timer[3]<1000)
			  timer[3]=1000;

		  status_data.set_roll=((float)timer[3]-1500)/30;	//	max: 500/30

		  //---------------------------------------------------------------------- yaw
		  if(timer[2]>1489 && timer[2]<1511)
			  timer[2]=1500;
		  else if(timer[2]>2000)
			  timer[2]=2000;
		  else if(timer[2]<1000)
			  timer[2]=1000;

		  if(timer[0]>1030){
			  status_data.set_yaw-=(float)((float)timer[2]-1500.0)/(5000.0);
			  if(status_data.set_yaw>179.9999){
				  status_data.set_yaw-=360.0;
			  }
			  else if(status_data.set_yaw<-179.9999){
				  status_data.set_yaw+=360.0;
			  }
		  }
		  else{
			  status_data.set_yaw=status_data.yaw;
		  }

		  //----------------------------------------------------------------------
		  //uart_counter++;
		  if(uart_counter>100){
			  printf("%d\t%d\t%d\r\n%d\t%d\t%d\n\n\r",timer[0],timer[1],timer[2],timer[3],timer[4],timer[5]);
			  uart_counter=0;
		  }
		  condition_uart4=0;
	  }

//=========================================================================================================================


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);

   if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
  {
  Error_Handler();  
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {
    
  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_8, 336, LL_RCC_PLLP_DIV_4);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
    
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  
  }
  LL_Init1msTick(84000000);
  LL_SetSystemCoreClock(84000000);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  /**ADC1 GPIO Configuration  
  PC0   ------> ADC1_IN10 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.SequencersScanMode = LL_ADC_SEQ_SCAN_DISABLE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  LL_ADC_REG_SetFlagEndOfConversion(ADC1, LL_ADC_REG_FLAG_EOC_UNITARY_CONV);
  LL_ADC_DisableIT_EOCS(ADC1);
  ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;
  ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);
  /** Configure Regular Channel 
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_10);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_10, LL_ADC_SAMPLINGTIME_3CYCLES);
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**I2C2 GPIO Configuration  
  PB10   ------> I2C2_SCL
  PB11   ------> I2C2_SDA 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10|LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C2);

  /* I2C2 DMA Init */
  
  /* I2C2_RX Init */
  LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_3, LL_DMA_CHANNEL_7);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_3, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_3, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_3, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_3, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_3, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_3, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_3, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_3);

  /* I2C2 interrupt Init */
  NVIC_SetPriority(I2C2_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(I2C2_EV_IRQn);

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  /** I2C Initialization 
  */
  LL_I2C_DisableOwnAddress2(I2C2);
  LL_I2C_DisableGeneralCall(I2C2);
  LL_I2C_EnableClockStretching(I2C2);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.ClockSpeed = 400000;
  I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C2, &I2C_InitStruct);
  LL_I2C_SetOwnAddress2(I2C2, 0);
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**I2C3 GPIO Configuration  
  PC9   ------> I2C3_SDA
  PA8   ------> I2C3_SCL 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C3);

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  /** I2C Initialization 
  */
  LL_I2C_DisableOwnAddress2(I2C3);
  LL_I2C_DisableGeneralCall(I2C3);
  LL_I2C_EnableClockStretching(I2C3);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.ClockSpeed = 400000;
  I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C3, &I2C_InitStruct);
  LL_I2C_SetOwnAddress2(I2C3, 0);
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**SPI1 GPIO Configuration  
  PB3   ------> SPI1_SCK
  PB4   ------> SPI1_MISO
  PB5   ------> SPI1_MOSI 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_3|LL_GPIO_PIN_4|LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 10;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

  /* TIM2 interrupt Init */
  NVIC_SetPriority(TIM2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM2_IRQn);

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  TIM_InitStruct.Prescaler = 83;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 2000;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM2, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM2);
  LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM2, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH2);
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM2, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH3);
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM2, LL_TIM_CHANNEL_CH3);
  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM2);
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**TIM2 GPIO Configuration  
  PA0-WKUP   ------> TIM2_CH1
  PA1   ------> TIM2_CH2
  PA2   ------> TIM2_CH3 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1|LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  TIM_InitStruct.Prescaler = 83;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 2000;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM4, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM4);
  LL_TIM_SetClockSource(TIM4, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM4, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH2);
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM4, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH3);
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM4, LL_TIM_CHANNEL_CH3);
  LL_TIM_SetTriggerOutput(TIM4, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM4);
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
  /**TIM4 GPIO Configuration  
  PD12   ------> TIM4_CH1
  PD13   ------> TIM4_CH2
  PD14   ------> TIM4_CH3 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_12|LL_GPIO_PIN_13|LL_GPIO_PIN_14;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM11);

  /* TIM11 interrupt Init */
  NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  TIM_InitStruct.Prescaler = 83;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 1000;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM11, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM11);
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART4);
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  /**UART4 GPIO Configuration  
  PC10   ------> UART4_TX
  PC11   ------> UART4_RX 
  */
  GPIO_InitStruct.Pin = RECEIVER_TX_Pin|RECEIVER_RX_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* UART4 DMA Init */
  
  /* UART4_RX Init */
  LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_2, LL_DMA_CHANNEL_4);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_2, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_2, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_2, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_2, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_2, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_2, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_2, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_2);

  /* UART4 interrupt Init */
  NVIC_SetPriority(UART4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(UART4_IRQn);

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(UART4, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(UART4);
  LL_USART_Enable(UART4);
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration  
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9|LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
  /**USART2 GPIO Configuration  
  PD5   ------> USART2_TX
  PD6   ------> USART2_RX 
  */
  GPIO_InitStruct.Pin = GPS_TX_Pin|GPS_RX_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART2);
  LL_USART_Enable(USART2);
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);

  /**/
  LL_GPIO_ResetOutputPin(GPIOE, LL_GPIO_PIN_2|TEST_OUTPUT_1_Pin|LL_GPIO_PIN_4|LL_GPIO_PIN_5 
                          |LED1_Pin|LED2_Pin|LED3_Pin|LL_GPIO_PIN_10);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, TEST_OUTPUT1_Pin|TEST_OUTPUT2_Pin);

  /**/
  LL_GPIO_ResetOutputPin(FLASH_CS_GPIO_Port, FLASH_CS_Pin);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2|TEST_OUTPUT_1_Pin|LL_GPIO_PIN_4|LL_GPIO_PIN_5 
                          |LED1_Pin|LED2_Pin|LED3_Pin|LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = TEST_OUTPUT1_Pin|TEST_OUTPUT2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = SWITCH_2_Pin|SWITCH_1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = FLASH_CS_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(FLASH_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
