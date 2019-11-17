/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ON  1
#define OFF 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint32_t adcResult = 0;   //���������� ��� ���������� ���������� ���������

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t Look_button(){  //������� ������ ������ in1
	uint8_t state_button=0; //������ ������� ������
	                         //0 - �� ������
	                         //1- �������� �������
	                        //2-������ �������
	uint32_t buttonstart_count2=0;
	 if(HAL_GPIO_ReadPin(GPIOA, in1_Pin)==GPIO_PIN_SET){ //������� ������� ������
		  		  while(HAL_GPIO_ReadPin(GPIOA, in1_Pin)==GPIO_PIN_SET){
		  			  buttonstart_count2=buttonstart_count2+1;
		  		  }
		  	  }else{
		  		  buttonstart_count2=0;
		  	  }



		  	  if((400000>buttonstart_count2)&(buttonstart_count2>50)){//������� ������������� �� ���� ������ ������
		  		   state_button=1;  //���� �������� �������
		  		  //flag_start=1;   //��������� ������ �������
		  		  buttonstart_count2=0;
		  	  }else{
		  		  if(buttonstart_count2>600000){
		  			// if(buttonstart_count2<800000){
		  				state_button=2;   //���� ������ �������
		  				buttonstart_count2=0;
		  			// }
		  			  //flag_stop=1;
		  	        // flag_start=0;
		  			//  buttonstart_count2=0;
		  		  }else {
		  		  state_button=0;   //�� ���� �������
		  		buttonstart_count2=0;
		  		  }
		  	  }

	return state_button;
}
 uint8_t See_ignation(){// ������� ��������� ��������� ��������� in2
 uint8_t Zajiganie=0;
           //�������� ��������� ���������
	  	  if(HAL_GPIO_ReadPin(GPIOA, in2_Pin)==GPIO_PIN_SET){ //������� �������� �� ���������
	  	 	        HAL_Delay(1);
	  	 	       HAL_GPIO_WritePin(GPIOA, led2_Pin, GPIO_PIN_SET);
	  	 	       	 		    HAL_Delay(25);
	  	 	        if(HAL_GPIO_ReadPin(GPIOA, in2_Pin)==GPIO_PIN_SET){
	  	 	        	Zajiganie=1;   //������ ���� ��� ��������� ��������
	  	 		     HAL_GPIO_WritePin(GPIOA, led2_Pin, GPIO_PIN_SET);
	  	 		     //HAL_Delay(1000);
	  	 	                                                     }
	  	 	}else{
	  	 		Zajiganie=0;   //����� ��������� ���������
	  	 		 HAL_GPIO_WritePin(GPIOA, led2_Pin, GPIO_PIN_RESET);
	  	 	  }

return	Zajiganie;
}
 void Webasto(int status){    //������� ��������� ����������� ���� ����� 2 ��������� �������
	 if(status==1){
		 HAL_GPIO_WritePin(GPIOA, rele2_Pin, GPIO_PIN_SET);
	 }else{
		 HAL_GPIO_WritePin(GPIOA, rele2_Pin, GPIO_PIN_RESET);
	 }
 }
 void Rele1(int status2){    //������� ��������� ����������� ���� ����� 1
 	 if(status2==1){
 		 HAL_GPIO_WritePin(GPIOB, rele1_Pin, GPIO_PIN_SET);
 	 }else{
 		 HAL_GPIO_WritePin(GPIOB, rele1_Pin, GPIO_PIN_RESET);
 	 }
  }
 void Blink(){
		  		  HAL_GPIO_TogglePin(GPIOA, led1_Pin);
 }
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
   // uint16_t timereboot=86400;
	uint8_t short_actionflag=0;         ///���� ��� ������� ��������� �� �������� ������ 25 ���
	uint8_t long_actionflag=0;  //���� �������� ������� webasto 0-�������� �� 25��� 1 -�������� �� ���������� ���������
	uint8_t statusZajiganiya=0;  //0- ��������� ��������� 1- ��������� ��������
	uint8_t statuspressbutton=0;  //0 ������ �� ������ 1- �������� ������� 2  ������ �������
	uint32_t buttonstart_count; //���������� ��� ��������� ������� ������������ ������� ������
    uint8_t flag_start=0;  //���� ���������� �������
    uint16_t time_progrev=1500;  // ����� �������� ������� 1500 ���
    //uint16_t time_progrev=600;  // ����� �������� ������� 1500 ���
    uint16_t time_posle_raboty=1200; //����� ������ ���������� ����� ���������� ��������
    uint8_t action=0; //���� �������� ������� ������ ��� ��� 0- ��� 1- ��
    uint32_t old_time_counter=0;  //������� ��� �������� �����������
    uint8_t good=0;   // ���� �������� ��������� ������� 0-������� ������ ����� 1- ������� ���������� 2- ������� ����������
    uint8_t flag_stop=0;   // ���� �������������� ���������  0- �� �������������  1- ���������� �������
    uint8_t led_ok_set=0; // ���� ��������� ���������� ���� ��� ���������� ��
    uint16_t buttonIgnition_count=0;
    uint8_t ig_Off=0,ig_On=0;  // ���� ��������� ���������

   // uint8_t str[]="USART Transmit\r\n";// ����� ���������� ��� ADC
    uint8_t mistr[6];
    uint8_t ubseti[8];
    uint8_t uvoltage[6];
    //uint8_t uvoltage[6],blabla[6];
    //uint8_t uvoltage[6],ubseti[8];
    //uint8_t usdecimal[8];
    uint8_t miadc[4];
    //uint8_t ch=1234;
    float Upit=3.26,UR2,I2;
    volatile  float Utek;
    double U4,U3,I1;
    float Ubortseti;
    float U,x;
    uint16_t MiConstRazr=3850,R1=15000,R2=4300;
    //  uint16_t MiConstRazr=3940,R1=16610,R2=4690;
    uint8_t  Mimassiv[4];
    char *p;

    volatile uint16_t Ubsint;
     uint16_t i,U2,y;
     uint8_t sel_akb=0;//  ���� ��� ���   0 - �� ��� 1 ���
     uint8_t dvigatel_zapushen=0;   //���� ����������� ���������  0-�� ������� 1 -�������
     uint8_t dvigstopweb=0;   //���� ��� ���� ����� ������� ��������� ���� ��� �� ����� ������ ���������
     uint8_t alarm_akbstopweb=0;  //���� ��������� ������� �� ���������� ������� ������������ 0-�� ���� ������� 1- ��������� � ������
  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_USART1_UART_Init();
  MX_TIM16_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  //HAL_TIM_Base_Start(&htim16);
  //HAL_TIM_Base_Start_IT(&htim16);
  tim16_counter=0;            // �������������� ������� �������
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /////////////////////////////////////////////////
	  //������ ��������� //
	  //�������� WDT ��� ������
	  //�������� ��������� ��������� ������
	  //� ��������� ��������� ������� ��������� Webasto ��� ��������� ���������
	  //��������//
	  //�������� ������ in1    �������� ������� ����� ������ ������� ����
	        //������� �������� �� ��������� ig_On==1
	       //���� ������� �� ��������� ������� �� ������� ���������� ��������� ig_On==0
	       //���� ��������� �� ��������� �� 25�����
	             //���������� ����� rele2_Pin -������� ���� ������
	             //������� 25�����
	            //������� ������� �����������
	            //�� ����������� 25 ��� �������� ����
	            //������� ������ ��������� �� 20��� -��� ���������� ������ ��� ��
	            //������ �� �����������
	                     //���� ���������� ������� �� 10� �� ��� ���
	                           //��������� ���������� Webasto
	                           //������� ������� ����������� 30 ��� ��� �� ��� ���������



	  /////////////////////////////////////////////
	  //������ ��������� �������//
	  //�������� ������   in1  ������ 12�����
	  	    //��������� ������� � ����� �������� ����2
	  	    //������� 25 �����   ������� ������� ���������
	  	    //��������� ����2  ���� ��� ������ ����������  �� ������� ����� ��������� 20��� ���  �� ��������� ���������
	  	    //�� ����� ������ ������ ����������
	  	    //���� ���������� ��������� 12,9 ������ �� ��������� ������  ���� ����2
	  	    //���� ���������� ����� ���� 10� ��������� ������� - ��� ����� ������� ������� ��������� 30 ��� ��� �� ��� ���������
	  	    //��������� �� ���� in2
	  	    //���������� ��� �����
	  	    //��������� � �.1
//////////////////������ ��������� �������/////////////////////////
	  statuspressbutton=Look_button();//���������� ���� �������� �� ������ �������
      statusZajiganiya=See_ignation();//���������� �������� �� ���������
          if(statuspressbutton==0){  //���� ������ �� ���� ������
        	  flag_start=0;
        	  statuspressbutton=0;
          }else if(statuspressbutton==1){  //���� ������ ������� ������
        	  flag_start=1;
        	  statuspressbutton=0;
          }else if(statuspressbutton==2){  //���� ������ ������� ������

        	  statuspressbutton=0;
			  flag_stop=1;
          }

          if(statusZajiganiya==1){
                  // long_actionflag=1;  //��� ��� �������� ��������� �� ��������� Webasto �� �������

          //���� ������ ���� �� ������ � �� ����� ��� �� ���� -������� �������
          	  	  if((flag_start==1)&(action==0)){
          	  		  flag_start=0;
          	  		  Webasto(ON);  //��������  ���� �������
                      HAL_TIM_Base_Start(&htim16);    //��������� ������
          	  		  HAL_TIM_Base_Start_IT(&htim16);  //��������� ����������
          	  		  action=1;  // ������� ��������
          	  		  flag_start=0;// ������ ������ �� ���������
          	  		  long_actionflag=1;  //��� ��� �������� ��������� �� ��������� Webasto �� �������


          	  	  }
          	  	  //������� �������� ���� �������� ������ ������� �������
          	  	  if((old_time_counter<tim16_counter)&(action==1)){
          	  		  Blink();
          	  	      old_time_counter=tim16_counter;
          	  	  }

                 if(action==1){//���� ������� ��������
                	 if(flag_stop==1){  //������ ������ �� ��������� webasto
                		 action=0;  // ������� ���������
						  long_actionflag=0; //���� �������� ������� �������������
						  Webasto(OFF);  //������  ���� �������
						  HAL_GPIO_WritePin(GPIOA, led1_Pin,RESET);  //��������� ���������
						  flag_stop=0;  //���������� ���� �������������� ���������
						  HAL_TIM_Base_Stop(&htim16);    //��������� ������
						  HAL_TIM_Base_Stop_IT(&htim16);  //��������� ����������
                     }
                 }
        	//�� if statusZajiganiya
          }else if(long_actionflag==1){  //���� ������� ���� �������� � ������ ��������� ���������
        	                              //� ��������� ���������
        	  action=0;  // ������� ���������
        	  long_actionflag=0; //���� �������� ������� �������������
        	  Webasto(OFF);  //������  ���� �������
        	  HAL_GPIO_WritePin(GPIOA, led1_Pin,RESET);  //��������� ���������
        	  HAL_TIM_Base_Stop(&htim16);    //��������� ������
        	  HAL_TIM_Base_Stop_IT(&htim16);  //��������� ����������
          }else if((flag_start==1)&(action==0)){ //��������� ��������� � ������ ������������ �� ������
					  flag_start=0;       //��������� ���������� �� ������
					  Webasto(ON);  //��������  ���� �������
					  HAL_TIM_Base_Start(&htim16);    //��������� ������
					  HAL_TIM_Base_Start_IT(&htim16);  //��������� ����������
					  action=1;  // ������� ��������
					  flag_start=0;// ������ ������ �� ���������
					  HAL_TIM_Base_Start(&htim16);    //��������� ������
					  HAL_TIM_Base_Start_IT(&htim16);  //��������� ����������
					  short_actionflag=1;   //���� ��� webasto �������� �� ���������
          }

          //������� �������� ���� �������� ������ ������� �������
                   	  	if(short_actionflag==1){
                       if((old_time_counter<tim16_counter)&(action==1)){
                   	  		  Blink();
                   	  	      old_time_counter=tim16_counter;
                   	  	  }
                       //���� ������ 25��� �� ��� ���������
                       	  	  if((tim16_counter>time_progrev)&((action==1))){
                       	  		           short_actionflag=0;
                                           Webasto(OFF);
                       	  		           action=0;  // ������� �������
                       	  		  		   tim16_counter=0;   //���������� ��������
                       	  		  		   old_time_counter=0;  //���������� ��������
                       	  		  		   good=1;  //���� ��������� ���������� ��������
                       	  		  		   flag_start=0;
                       	  		  	       HAL_GPIO_WritePin(GPIOA, led1_Pin,RESET);  //��������� ���������
                       	  	  }
						//���� ���� �������������� ��������� ��
						  if(flag_stop==1){
          	  		           short_actionflag=0;
							   Webasto(OFF);
							   HAL_GPIO_WritePin(GPIOA, led2_Pin, GPIO_PIN_SET);
							   HAL_Delay(2000);
							   HAL_GPIO_WritePin(GPIOA, led2_Pin, GPIO_PIN_RESET);
							   HAL_GPIO_WritePin(GPIOA, led1_Pin, GPIO_PIN_RESET);
							   tim16_counter=0;
							   action=0;  // ������� �������
							   old_time_counter=0;  //���������� ��������
							   good=0;  //���� ��������� ���������� ��������
							   flag_start=0;
							   flag_stop=0;
							   HAL_TIM_Base_Stop(&htim16);    //��������� ������
							   HAL_TIM_Base_Stop_IT(&htim16);  //��������� ����������
						  }

                   	  	}/////�� if short_actionflag////
             //�������� ������� ��������� �� 20 ��� ���� ��� ���������� ��
			  if((good==1)&(action==0)&(flag_stop==0)){
					 if((tim16_counter<time_posle_raboty)&(led_ok_set==0)){
					      HAL_GPIO_WritePin(GPIOA, led1_Pin, GPIO_PIN_SET);
					      led_ok_set=1;
					      tim16_counter=0;//����������� ���������
						 }
				 //���� ��������� �������� �� ��������� ���������
				 //if(ig_On==1){
				 //	 tim16_counter=time_posle_raboty+1;
				 //}
				    if(tim16_counter>time_posle_raboty){
					   HAL_GPIO_WritePin(GPIOA, led1_Pin, GPIO_PIN_RESET);
				       good=0;
				       tim16_counter=0;
				       HAL_TIM_Base_Stop(&htim16);    //��������� ������
				       HAL_TIM_Base_Stop_IT(&htim16);  //��������� ����������
					   flag_start=0;
					   led_ok_set=0;           //���������� ���� ��������� ����������
				    }
				  }//�� ��������� ���������� �� 20���
         //////////////////////////////////
			 if(short_actionflag==1){
			  //����� ������ ����������
			  	  	  HAL_ADC_Start(&hadc);      //��������� ��������� ����������� �����
			  	        HAL_ADC_PollForConversion(&hadc, 100);    //���������� ��������� ��������
			  	        adcResult = HAL_ADC_GetValue(&hadc);     //�������� ��������
			  	        HAL_ADC_Stop(&hadc);          //������������� ���������

			  	        Utek=adcResult*Upit/MiConstRazr; //��������� ������� �����
			  	        U=Utek*100;        //�������� ������ �������� ���� ���������
			  	        U3=(double)U;      //������ ��� ������� ��� ������� floor
			  	        U4=floor(U3+0.5);    //floor ���������� �� ������ �� ��� ��� double
			  	        U2=(int)U4;           // ������ ��� double ��  int ��� ���������� ����� � ��������

			  	   //����� ��������� ���������� �� ����� � ����� �� ��������
			      //2///////////////////////////////////////////////////////////////////////////////
			  	         UR2=Utek;      //������ ��� ������
			  	         I2=UR2/R2;             //������� ��� ����� �������� R2 -4700
			  	         Ubortseti=I2*(R1+R2);  //��������� ������� ����������
						 Ubortseti=Ubortseti*100;// �������� ������ ������ ����� ���������� ���� ����
						 Ubortseti=floor(Ubortseti+0.5); //���������
						 x=Ubortseti;
						 Ubsint=(int)Ubortseti;
			     /////////////////////////����� ������ ����������
			  	          //����� ���������� Ubsint  1236   ���������� � int
			  	          //���� Ubsint  ������  1290  �� ��������� �������
			  	          //���� Ubsint  ������ 1000   �� ��� ���
			  	          //����� ������������� ������ �����������
			  	  //3///////////////////////////////////////////////////////////////////////////////
			             if(Ubsint<1000){
			  	      	   sel_akb=1;
			  	         }else{
			  	      	   sel_akb=0;
			  	      	   alarm_akbstopweb=0;   //��������� ���� ���������� ����� ������� �� ������� ���
			  	         }
			  	              if((sel_akb==1)&(alarm_akbstopweb==0)){    //���� ��� ��� � ������� �������� � ��� �� ��������� �� ����� ��
			  	              	if(action==1){
			  	              		flag_stop=1;
			  	              		alarm_akbstopweb=1;
			  	              	}
			  	              }

			  	            ///����� ������������� �������� ���������� ������� ��� ������� ���
			  	         if(Ubsint>1290){

			  	      	   dvigatel_zapushen=1;
			  	         }
			  	         else{
			  	      	   dvigatel_zapushen=0;
			  	      	   dvigstopweb=0;          //����� ��������� ����� �������� �� ����� ������� �������
			  	         }


			  	         if((dvigatel_zapushen==1)&(dvigstopweb==0)){ //���� ��������� ������� � �������  ��� �� ��������� �� ������ �������
			  	      	  if(action==1){
			  	      		//HAL_TIM_Base_Stop(&htim16);    //��������� ������
						    //HAL_TIM_Base_Stop_IT(&htim16);  //��������� ����������
						    flag_start=0;
						    tim16_counter=0;
        	  		  	    HAL_GPIO_WritePin(GPIOA, led1_Pin,RESET);  //��������� ���������
        	  		  	    short_actionflag=0;   //��������� �� �������� ������� ������� � ������
        	  		  	   // Rele1(ON);
        	  		  	    long_actionflag=1;
        	  		  	    dvigatel_zapushen=0;

			  	      	             }
			  	         }

			 }//�� short action flag








			  //////����� ������� ���������� ���������
////////////////����� ������ ���������� �������///////////////////////////
	  ////////////////////////////////////////������ ��������� �������////////////
          //1//////////////////////////////////////////////////////////////
          /*


	  if(HAL_GPIO_ReadPin(GPIOA, in1_Pin)==GPIO_PIN_SET){ //������� ������� ������
	  		  while(HAL_GPIO_ReadPin(GPIOA, in1_Pin)==GPIO_PIN_SET){
	  			  buttonstart_count=buttonstart_count+1;
	  		  }
	  	  }else{
	  		  buttonstart_count=0;
	  	  }



	  	  if((200000>buttonstart_count)&(buttonstart_count>3000)){  //������� ������������� �� ���� ������ ������


	  		  flag_start=1;   //��������� ������ �������

	  		  buttonstart_count=0;

	  	  }else{
	  		  if(buttonstart_count>200000){
	  			  flag_stop=1;
	  	          flag_start=0;
	  			  buttonstart_count=0;
	  		  }
	  	  }

	  	  //���� ������ ���� �� ������ -������� �������
	  	  if((flag_start==1)&(action==0)){
	  		  flag_start=0;
	  		  HAL_GPIO_WritePin(GPIOA, rele2_Pin, GPIO_PIN_SET);  //�������� ����� � �������
	  		//  HAL_GPIO_WritePin(GPIOA, rele2_Pin, GPIO_PIN_SET);/
	  		  HAL_TIM_Base_Start(&htim16);    //��������� ������
	  		  HAL_TIM_Base_Start_IT(&htim16);  //��������� ����������
	  		  action=1;  // ������� ��������
	  		  flag_start=0;// ������ ������ �� ���������
	  	  }
	  	  //������� �������� ���� �������� ������ ������� �������

	  	  if((old_time_counter<tim16_counter)&(action==1)){
	  		  HAL_GPIO_TogglePin(GPIOA, led1_Pin);
	  	      old_time_counter=tim16_counter;
	  	  }

	  	  //���� ������ 25��� �� ��� ���������
	  	  if((tim16_counter>time_progrev)&((action==1))){

	  		          HAL_GPIO_WritePin(GPIOA, rele2_Pin, GPIO_PIN_RESET);  //�������� ����� � �������
	  		  		 action=0;  // ������� �������
	  		  		tim16_counter=0;   //���������� ��������
	  		  		old_time_counter=0;  //���������� ��������
	  		  		good=1;  //���� ��������� ���������� ��������
	  		  		flag_start=0;
	  	  }
	  	  //���� ���� �������������� ��������� ��
	  	  if(flag_stop==1){

	  		  HAL_GPIO_WritePin(GPIOA, rele2_Pin, GPIO_PIN_RESET);  //��������� ����� � �������

	  		  			   HAL_GPIO_WritePin(GPIOA, led2_Pin, GPIO_PIN_SET);
	  		  			   HAL_Delay(2000);

	  		  HAL_GPIO_WritePin(GPIOA, led2_Pin, GPIO_PIN_RESET);
	  		  HAL_GPIO_WritePin(GPIOA, led1_Pin, GPIO_PIN_RESET);

	  		  tim16_counter=0;
	  		  action=0;  // ������� �������
	  		  old_time_counter=0;  //���������� ��������
	  		   good=0;  //���� ��������� ���������� ��������


	  		   flag_start=0;
	  		   flag_stop=0;
	  		   HAL_TIM_Base_Stop(&htim16);    //��������� ������
	  		   HAL_TIM_Base_Stop_IT(&htim16);  //��������� ����������
	  		   buttonstart_count=0;
	  	 	  }
	  	  //�������� ������� ��������� �� 20 ��� ���� ��� ���������� ��
	  	  if((good==1)&(action==0)&(flag_stop==0)){

	  		//  HAL_GPIO_WritePin(GPIOA, led2_Pin, GPIO_PIN_SET);

	  		 if((tim16_counter<time_posle_raboty)&(led_ok_set==0)){
	  		  HAL_GPIO_WritePin(GPIOA, led1_Pin, GPIO_PIN_SET);
	  		 // HAL_Delay(3000);
	  		  led_ok_set=1;
	  	             }
	  		 //���� ��������� �������� �� ��������� ���������
	  		 if(ig_On==1){
	  			 tim16_counter=time_posle_raboty+1;
	  		 }
	  		  if(tim16_counter>time_posle_raboty){
	  			 // HAL_GPIO_WritePin(GPIOA, led2_Pin, GPIO_PIN_SET);
	  		  HAL_GPIO_WritePin(GPIOA, led1_Pin, GPIO_PIN_RESET);
	  		  good=0;
	  		  tim16_counter=0;
	  		   HAL_TIM_Base_Stop(&htim16);    //��������� ������
	  		   HAL_TIM_Base_Stop_IT(&htim16);  //��������� ����������
	            flag_start=0;
	            buttonstart_count=0;
	            led_ok_set=0;           //���������� ���� ��������� ����������
	  		  }
	  		  }//�� ��������� ���������� �� 25���
	        //�������� ��������� ���������
	  	  if(HAL_GPIO_ReadPin(GPIOA, in2_Pin)==GPIO_PIN_SET){ //������� �������� �� ���������
	  	 	        HAL_Delay(1);
	  	 	       HAL_GPIO_WritePin(GPIOA, led2_Pin, GPIO_PIN_SET);
	  	 	       	 		//     HAL_Delay(1000);
	  	 	        if(HAL_GPIO_ReadPin(GPIOA, in2_Pin)==GPIO_PIN_SET){
	  	 		     ig_On=1;   //������ ���� ��� ��������� ��������
	  	 		     HAL_GPIO_WritePin(GPIOA, led2_Pin, GPIO_PIN_SET);
	  	 		     //HAL_Delay(1000);
	  	 	                                                     }
	  	 	}else{
	  	 		  ig_On=0;   //����� ��������� ���������
	  	 		 HAL_GPIO_WritePin(GPIOA, led2_Pin, GPIO_PIN_RESET);
	  	 	  }
	        //����� ������� ��������� ���������
	  ////////////////////////////////////////////////////////////////////////
	  	  //����� ������ ����������
	  	  HAL_ADC_Start(&hadc);      //��������� ��������� ����������� �����
	        HAL_ADC_PollForConversion(&hadc, 100);    //���������� ��������� ��������
	        adcResult = HAL_ADC_GetValue(&hadc);     //�������� ��������
	        HAL_ADC_Stop(&hadc);          //������������� ���������

	        Utek=adcResult*Upit/MiConstRazr; //��������� ������� �����
	        U=Utek*100;        //�������� ������ �������� ���� ���������
	        U3=(double)U;      //������ ��� ������� ��� ������� floor
	        U4=floor(U3+0.5);    //floor ���������� �� ������ �� ��� ��� double
	        U2=(int)U4;           // ������ ��� double ��  int ��� ���������� ����� � ��������
	        //����������� ������� ������� �����  ��������� � ADC �� �������� �������
	              /*     if(adcResult>999){
	        	                    miadc[0]=(adcResult%10000-adcResult%1000)/1000;
	        	                	miadc[1]=(adcResult%1000-adcResult%100)/100;
	        	                	miadc[2]=(adcResult%100-adcResult%10)/10;
	        	      	            miadc[3]=adcResult%10;

	                             }else if(adcResult>99) {
	                           	 miadc[0]=0;
	                          	 miadc[1]=(adcResult%1000-adcResult%100-adcResult%10)/100;
	                	             miadc[2]=(adcResult%100-adcResult%10)/10;
	                	             miadc[3]=adcResult%10;
	                            }else if(adcResult>9){
	                	             miadc[0]=0;
	                	             miadc[1]=0;
	                	             miadc[2]=(adcResult%100-adcResult%10)/10;
	                	             miadc[3]=adcResult%10;
	                             }else {
	                	             miadc[0]=0;
	                	             miadc[1]=0;
	                	             miadc[2]=0;
	                	             miadc[3]=adcResult%10;
	                                   }
	                   // ������� ��� �������� ������� miadc (uint) � mistr (string)
	                             sprintf(mistr,"%d%d%d%d\r\n", miadc[0],miadc[1],miadc[2],miadc[3]);
	                             //���������� ������ mistr � ����
	                            HAL_UART_Transmit(&huart1,mistr,6,0xFFFF);
	                              	  HAL_Delay(200);
	                	  //��������� ������ Mimassiv ���� string �� ��������� ������� Mimassiv
	                                  Mimassiv[0]=(U2%1000-U2%100)/100;
	                                  Mimassiv[1]=(U2%100-U2%10)/10;
	                                  Mimassiv[2]=U2%10;
	                            sprintf(uvoltage,"%d%c%d%d\r\n", Mimassiv[0],'.',Mimassiv[1],Mimassiv[2]);
	                   //���������� �������� ���������� �� �������� ����� � ����
	                             HAL_UART_Transmit(&huart1,uvoltage,6,0xFFFF);
	                              HAL_Delay(200);
	                                     */
	   //����� ��������� ���������� �� ����� � ����� �� ��������
    //2///////////////////////////////////////////////////////////////////////////////
	/*        UR2=Utek;      //������ ��� ������
	        I2=UR2/R2;             //������� ��� ����� �������� R2 -4700
	       Ubortseti=I2*(R1+R2);  //��������� ������� ����������
	       // ����� ���� ������ �� �������� � ������ ��� �������� � ����


	                     Ubortseti=Ubortseti*100;// �������� ������ ������ ����� ���������� ���� ����
	                     Ubortseti=floor(Ubortseti+0.5); //���������
	                     x=Ubortseti;
	                     Ubsint=(int)Ubortseti;
	             //������ ��������� ��� ������� ���� ���
	                /*        if(Ubsint>999){
	                        usdecimal[0]=(Ubsint%10000-Ubsint%1000)/1000;
	                        usdecimal[1]=(Ubsint%1000-Ubsint%100)/100;
	                        usdecimal[2]=(Ubsint%100-Ubsint%10)/10;
	                        usdecimal[3]=Ubsint%10;
	                        }else if(Ubsint>99){
	                     	   usdecimal[0]=0;
	                     	   usdecimal[1]=(Ubsint%1000-Ubsint%100)/100;
	                     	   usdecimal[2]=(Ubsint%100-Ubsint%10)/10;
	                     	   usdecimal[3]=Ubsint%10;
	                        }else if(Ubsint>9){
	                     	   usdecimal[0]=0;
	                     	   usdecimal[1]=0;
	                     	   usdecimal[2]=(Ubsint%100-Ubsint%10)/10;
	                     	   usdecimal[3]=Ubsint%10;
	                        }else {
	                     	   usdecimal[0]=0;
	                     	   usdecimal[1]=0;
	                     	   usdecimal[2]=0;
	                     	   usdecimal[3]=Ubsint%10;
	                        }     */
	              /*         if(Ubsint>999){
	                      	 miadc[0]=(Ubsint%10000-Ubsint%1000)/1000;
	                      	 miadc[1]=(Ubsint%1000-Ubsint%100)/100;
	                      	 miadc[2]=(Ubsint%100-Ubsint%10)/10;
	                      	 miadc[3]=Ubsint%10;
	                       }             */
	                       //��������� ������ ������
	                   //���������� ������� ���������� � USART
	                       // sprintf(ubseti,"%c\r\n",usdecimal[0]);
	               /*        sprintf(ubseti,"%d%d%d%d\r\n", miadc[0],miadc[1],miadc[2],miadc[3]);
	                     if(Ubsint>999) {
	                       HAL_UART_Transmit(&huart1,ubseti,7,0xFFFF);
	                       HAL_Delay(200);
	                     }          */
	        /////////////////////////����� ������ ����������
	              //����� ���������� Ubsint  1236   ���������� � int
	          //���� Ubsint  ������  1290  �� ��������� �������
	          //���� Ubsint  ������ 1000   �� ��� ���
	         //����� ������������� ������ �����������
	  //3///////////////////////////////////////////////////////////////////////////////
        /*  if(Ubsint<1000){
	      	   sel_akb=1;
	         }else{
	      	   sel_akb=0;
	      	   alarm_akbstopweb=0;   //��������� ���� ���������� ����� ������� �� ������� ���
	         }
	              if((sel_akb==1)&(alarm_akbstopweb==0)){    //���� ��� ��� � ������� �������� � ��� �� ��������� �� ����� ��
	              	if(action==1){
	              		flag_stop=1;
	              		alarm_akbstopweb=1;
	              	}
	              }

	            ///����� ������������� �������� ���������� ������� ��� ������� ���
	         if(Ubsint>1290){
	      	   dvigatel_zapushen=1;
	         }
	         else{
	      	   dvigatel_zapushen=0;
	      	   dvigstopweb=0;          //����� ��������� ����� �������� �� ����� ������� �������
	         }


	         if((dvigatel_zapushen==1)&(dvigstopweb==0)){ //���� ��������� ������� � �������  ��� �� ��������� �� ������ �������
	      	  if(action==1){
	      		  flag_stop=1;
	      	   dvigstopweb=1;      //������� �������� � ��� �� ������ �������
	      	              }
	      	                                         }
	         */
////////////////////////////////////////////////////////////////////////////////////////////////////

	         ////////����� ������ ���������///////////
	         //////////////////////////////////////////////
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 7999;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 999;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 7999;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, rele2_Pin|led2_Pin|led1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(rele1_GPIO_Port, rele1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : in2_Pin in1_Pin */
  GPIO_InitStruct.Pin = in2_Pin|in1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : rele2_Pin led2_Pin led1_Pin */
  GPIO_InitStruct.Pin = rele2_Pin|led2_Pin|led1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : rele1_Pin */
  GPIO_InitStruct.Pin = rele1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(rele1_GPIO_Port, &GPIO_InitStruct);

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
