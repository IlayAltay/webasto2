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
uint32_t adcResult = 0;   //переменная для считывания реультатов измерения

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
uint8_t Look_button(){  //функция опроса кнопки in1
	uint8_t state_button=0; //статус нажатия кнопки
	                         //0 - не нажата
	                         //1- короткое нажатие
	                        //2-длиное нажатие
	uint32_t buttonstart_count2=0;
	 if(HAL_GPIO_ReadPin(GPIOA, in1_Pin)==GPIO_PIN_SET){ //смотрим нажатие кнопки
		  		  while(HAL_GPIO_ReadPin(GPIOA, in1_Pin)==GPIO_PIN_SET){
		  			  buttonstart_count2=buttonstart_count2+1;
		  		  }
		  	  }else{
		  		  buttonstart_count2=0;
		  	  }



		  	  if((400000>buttonstart_count2)&(buttonstart_count2>50)){//смотрим действительно ли была нажата кнопка
		  		   state_button=1;  //было короткое нажатие
		  		  //flag_start=1;   //разрешаем запуск вебасто
		  		  buttonstart_count2=0;
		  	  }else{
		  		  if(buttonstart_count2>600000){
		  			// if(buttonstart_count2<800000){
		  				state_button=2;   //было длиное нажатие
		  				buttonstart_count2=0;
		  			// }
		  			  //flag_stop=1;
		  	        // flag_start=0;
		  			//  buttonstart_count2=0;
		  		  }else {
		  		  state_button=0;   //не было нажатие
		  		buttonstart_count2=0;
		  		  }
		  	  }

	return state_button;
}
 uint8_t See_ignation(){// функция просмотра состояния зажигания in2
 uint8_t Zajiganie=0;
           //Проверка включения зажигания
	  	  if(HAL_GPIO_ReadPin(GPIOA, in2_Pin)==GPIO_PIN_SET){ //смотрим включено ли зажигание
	  	 	        HAL_Delay(1);
	  	 	       HAL_GPIO_WritePin(GPIOA, led2_Pin, GPIO_PIN_SET);
	  	 	       	 		    HAL_Delay(25);
	  	 	        if(HAL_GPIO_ReadPin(GPIOA, in2_Pin)==GPIO_PIN_SET){
	  	 	        	Zajiganie=1;   //ставим флаг что зажигание включено
	  	 		     HAL_GPIO_WritePin(GPIOA, led2_Pin, GPIO_PIN_SET);
	  	 		     //HAL_Delay(1000);
	  	 	                                                     }
	  	 	}else{
	  	 		Zajiganie=0;   //иначе зажигание выключено
	  	 		 HAL_GPIO_WritePin(GPIOA, led2_Pin, GPIO_PIN_RESET);
	  	 	  }

return	Zajiganie;
}
 void Webasto(int status){    //функция активации деактивации реле номер 2 включение вебасто
	 if(status==1){
		 HAL_GPIO_WritePin(GPIOA, rele2_Pin, GPIO_PIN_SET);
	 }else{
		 HAL_GPIO_WritePin(GPIOA, rele2_Pin, GPIO_PIN_RESET);
	 }
 }
 void Rele1(int status2){    //функция активации деактивации реле номер 1
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
	uint8_t short_actionflag=0;         ///флаг что вебасто запсущена по короткой версии 25 мин
	uint8_t long_actionflag=0;  //флаг длинного запуска webasto 0-включаем на 25мин 1 -включаем до выключения зажигания
	uint8_t statusZajiganiya=0;  //0- зажигание выключено 1- зажигание включено
	uint8_t statuspressbutton=0;  //0 кнопка не нажата 1- короткое нажатие 2  длиное нажатие
	uint32_t buttonstart_count; //переменная для счеткчика ложного срабатывания нажатия кнопки
    uint8_t flag_start=0;  //флаг разрешения запуска
    uint16_t time_progrev=1500;  // время прогрева вебасто 1500 сек
    //uint16_t time_progrev=600;  // время прогрева вебасто 1500 сек
    uint16_t time_posle_raboty=1200; //время работы светодиода после завершения прогрева
    uint8_t action=0; //флаг работает система сейчас или нет 0- нет 1- да
    uint32_t old_time_counter=0;  //счетчик для моргания светодиодом
    uint8_t good=0;   // флаг успешной отработки вебасто 0-система только зашла 1- успешно отработала 2- хреново отработала
    uint8_t flag_stop=0;   // флаг принудительной остановки  0- не отсанавливать  1- останавить вебасто
    uint8_t led_ok_set=0; // флаг включения светодиода если все отработало ок
    uint16_t buttonIgnition_count=0;
    uint8_t ig_Off=0,ig_On=0;  // флаг включения зажигания

   // uint8_t str[]="USART Transmit\r\n";// далее переменные для ADC
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
     uint8_t sel_akb=0;//  флаг сел акб   0 - не сел 1 сел
     uint8_t dvigatel_zapushen=0;   //флаг запущенного двигателя  0-не заведен 1 -заведен
     uint8_t dvigstopweb=0;   //флаг для того чтобы вебасто глушилась один раз во время работы двигателя
     uint8_t alarm_akbstopweb=0;  //флаг остановки вебасто по требованию севшего аккумулятора 0-не надо глушить 1- заглушить к ебеням
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
  tim16_counter=0;            // инициализируем счетчик времени
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /////////////////////////////////////////////////
	  //Вторая генерация //
	  //Добавлен WDT для сброса
	  //Изменена концепция алгоритма работы
	  //в частности добавлена функция включения Webasto при включеном зажигании
	  //Алгоритм//
	  //Нажимаем кнопку in1    короткое нажатие старт длиное нажатие стоп
	        //смотрим включено ли зажигание ig_On==1
	       //если вклчено то запускаем вебасто до момента выключения зажигания ig_On==0
	       //если выключено то запускаем на 25минут
	             //активируем выход rele2_Pin -вклчаем реле веасто
	             //считаем 25минут
	            //моргаем зеленым светодиодом
	            //по прошестивии 25 мин отклчаем реле
	            //вклчаем еленый светодиод на 20мин -все отработало штатно все ок
	            //следим за напряжением
	                     //если напряжение просело до 10в то акб сел
	                           //аварийное отключение Webasto
	                           //моргаем красным светодиодом 30 мин или до вкл зажигания



	  /////////////////////////////////////////////
	  //Первая генерация проекта//
	  //нажимаем кнокпу   in1  подаем 12вольт
	  	    //запускаем вебасто и помпу включаем реле2
	  	    //считаем 25 минут   моргает зеленый светодиод
	  	    //выключаем реле2  если все штатно отработало  то зеленый горит постоянно 20мин или  до включения зажигания
	  	    //во время работы меряем напряжение
	  	    //если напряжение превысило 12,9 вольта то произошел запуск  выкл реле2
	  	    //если напряжение упало ниже 10в выключаем вебасту - сел аккум моргает красный светодиод 30 мин или до вкл зажигания
	  	    //зажигание на вход in2
	  	    //сбрасываем все флаги
	  	    //переходим в П.1
//////////////////Вторая генерация проекта/////////////////////////
	  statuspressbutton=Look_button();//спрашиваем чего творится на кнопке запуска
      statusZajiganiya=See_ignation();//спрашиваем включено ли зажигание
          if(statuspressbutton==0){  //если кнопка не была нажата
        	  flag_start=0;
        	  statuspressbutton=0;
          }else if(statuspressbutton==1){  //если кнопку коротко нажали
        	  flag_start=1;
        	  statuspressbutton=0;
          }else if(statuspressbutton==2){  //если кнопку длинное нажали

        	  statuspressbutton=0;
			  flag_stop=1;
          }

          if(statusZajiganiya==1){
                  // long_actionflag=1;  //так как включено зажигание то запускаем Webasto по долгому

          //если пришел флаг на запуск и до этого она не была -заводим вебасто
          	  	  if((flag_start==1)&(action==0)){
          	  		  flag_start=0;
          	  		  Webasto(ON);  //включаем  реле вебасто
                      HAL_TIM_Base_Start(&htim16);    //запускаем таймер
          	  		  HAL_TIM_Base_Start_IT(&htim16);  //запускаем прерывания
          	  		  action=1;  // система завелась
          	  		  flag_start=0;// запуск больше не требуется
          	  		  long_actionflag=1;  //так как включено зажигание то запускаем Webasto по долгому


          	  	  }
          	  	  //моргаем светиком пока работаем прошла секунда моргаем
          	  	  if((old_time_counter<tim16_counter)&(action==1)){
          	  		  Blink();
          	  	      old_time_counter=tim16_counter;
          	  	  }

                 if(action==1){//елси вебасто запущена
                	 if(flag_stop==1){  //пришел запрос на остановку webasto
                		 action=0;  // система заглушена
						  long_actionflag=0; //флаг длинного запуска деактивирован
						  Webasto(OFF);  //глушим  реле вебасто
						  HAL_GPIO_WritePin(GPIOA, led1_Pin,RESET);  //отключаем светодиод
						  flag_stop=0;  //сбрасываем флаг принудительной оттановки
						  HAL_TIM_Base_Stop(&htim16);    //выключаем таймер
						  HAL_TIM_Base_Stop_IT(&htim16);  //выключаем прерывания
                     }
                 }
        	//от if statusZajiganiya
          }else if(long_actionflag==1){  //если вебасто была запущена в режиме вклченого зажигания
        	                              //и зажигание выключено
        	  action=0;  // система заглушена
        	  long_actionflag=0; //флаг длинного запуска деактивирован
        	  Webasto(OFF);  //глушим  реле вебасто
        	  HAL_GPIO_WritePin(GPIOA, led1_Pin,RESET);  //отключаем светодиод
        	  HAL_TIM_Base_Stop(&htim16);    //выключаем таймер
        	  HAL_TIM_Base_Stop_IT(&htim16);  //выключаем прерывания
          }else if((flag_start==1)&(action==0)){ //зажигание выключено и пришло распрояжение на запуск
					  flag_start=0;       //скидываем разрешение на запуск
					  Webasto(ON);  //включаем  реле вебасто
					  HAL_TIM_Base_Start(&htim16);    //запускаем таймер
					  HAL_TIM_Base_Start_IT(&htim16);  //запускаем прерывания
					  action=1;  // система завелась
					  flag_start=0;// запуск больше не требуется
					  HAL_TIM_Base_Start(&htim16);    //запускаем таймер
					  HAL_TIM_Base_Start_IT(&htim16);  //запускаем прерывания
					  short_actionflag=1;   //флаг что webasto запущена по короткому
          }

          //моргаем светиком пока работаем прошла секунда моргаем
                   	  	if(short_actionflag==1){
                       if((old_time_counter<tim16_counter)&(action==1)){
                   	  		  Blink();
                   	  	      old_time_counter=tim16_counter;
                   	  	  }
                       //если прошло 25мин то все выключаем
                       	  	  if((tim16_counter>time_progrev)&((action==1))){
                       	  		           short_actionflag=0;
                                           Webasto(OFF);
                       	  		           action=0;  // система потухла
                       	  		  		   tim16_counter=0;   //сбрасываем счетчики
                       	  		  		   old_time_counter=0;  //сбрасываем счетчики
                       	  		  		   good=1;  //флаг успешного завершения прогрева
                       	  		  		   flag_start=0;
                       	  		  	       HAL_GPIO_WritePin(GPIOA, led1_Pin,RESET);  //отключаем светодиод
                       	  	  }
						//если была принудительная остановка то
						  if(flag_stop==1){
          	  		           short_actionflag=0;
							   Webasto(OFF);
							   HAL_GPIO_WritePin(GPIOA, led2_Pin, GPIO_PIN_SET);
							   HAL_Delay(2000);
							   HAL_GPIO_WritePin(GPIOA, led2_Pin, GPIO_PIN_RESET);
							   HAL_GPIO_WritePin(GPIOA, led1_Pin, GPIO_PIN_RESET);
							   tim16_counter=0;
							   action=0;  // система потухла
							   old_time_counter=0;  //сбрасываем счетчики
							   good=0;  //флаг успешного завершения прогрева
							   flag_start=0;
							   flag_stop=0;
							   HAL_TIM_Base_Stop(&htim16);    //выключаем таймер
							   HAL_TIM_Base_Stop_IT(&htim16);  //выключаем прерывания
						  }

                   	  	}/////от if short_actionflag////
             //включаем зеленый светодиод на 20 мин типа все отрабоатло ок
			  if((good==1)&(action==0)&(flag_stop==0)){
					 if((tim16_counter<time_posle_raboty)&(led_ok_set==0)){
					      HAL_GPIO_WritePin(GPIOA, led1_Pin, GPIO_PIN_SET);
					      led_ok_set=1;
					      tim16_counter=0;//скиываемдля светоиода
						 }
				 //если зажигание включили то выключаем светодиод
				 //if(ig_On==1){
				 //	 tim16_counter=time_posle_raboty+1;
				 //}
				    if(tim16_counter>time_posle_raboty){
					   HAL_GPIO_WritePin(GPIOA, led1_Pin, GPIO_PIN_RESET);
				       good=0;
				       tim16_counter=0;
				       HAL_TIM_Base_Stop(&htim16);    //выключаем таймер
				       HAL_TIM_Base_Stop_IT(&htim16);  //выключаем прерывания
					   flag_start=0;
					   led_ok_set=0;           //сбрасываем флаг включения светодиода
				    }
				  }//от включения светодиода на 20мин
         //////////////////////////////////
			 if(short_actionflag==1){
			  //здесь меряем напряжение
			  	  	  HAL_ADC_Start(&hadc);      //запускаем измерения аналогового входа
			  	        HAL_ADC_PollForConversion(&hadc, 100);    //дожидаемся окончания имерения
			  	        adcResult = HAL_ADC_GetValue(&hadc);     //забираем реультат
			  	        HAL_ADC_Stop(&hadc);          //останавливаем измерения

			  	        Utek=adcResult*Upit/MiConstRazr; //вычисляем сколько вольт
			  	        U=Utek*100;        //получаем первые трицифры чтоы округлить
			  	        U3=(double)U;      //меняем тип начения для функции floor
			  	        U4=floor(U3+0.5);    //floor округление до целого но оно еще double
			  	        U2=(int)U4;           // меняем тип double на  int для дальнейшей раоты с массивом

			  	   //Далее вычисляем напряжение на входе в схему до делителя
			      //2///////////////////////////////////////////////////////////////////////////////
			  	         UR2=Utek;      //меняем тип данных
			  	         I2=UR2/R2;             //считаем ток через резистор R2 -4700
			  	         Ubortseti=I2*(R1+R2);  //ВЫчисляем входное напряжение
						 Ubortseti=Ubortseti*100;// получаем первые четыре цифры напряжения борт сети
						 Ubortseti=floor(Ubortseti+0.5); //округляем
						 x=Ubortseti;
						 Ubsint=(int)Ubortseti;
			     /////////////////////////конец замера напряжения
			  	          //Имеем переменную Ubsint  1236   напряжение в int
			  	          //если Ubsint  больше  1290  то двигатель запущен
			  	          //если Ubsint  меньше 1000   то сел акб
			  	          //здесь рассматриваем севший аккумулятор
			  	  //3///////////////////////////////////////////////////////////////////////////////
			             if(Ubsint<1000){
			  	      	   sel_akb=1;
			  	         }else{
			  	      	   sel_akb=0;
			  	      	   alarm_akbstopweb=0;   //скидываем флаг аварийного стопа вебасто по севшему акб
			  	         }
			  	              if((sel_akb==1)&(alarm_akbstopweb==0)){    //если сел акб и вебасто работает и еще не глушилась то тушим ее
			  	              	if(action==1){
			  	              		flag_stop=1;
			  	              		alarm_akbstopweb=1;
			  	              	}
			  	              }

			  	            ///здесь рассматриваем ситуацию выключения вебасто при запуске двс
			  	         if(Ubsint>1290){

			  	      	   dvigatel_zapushen=1;
			  	         }
			  	         else{
			  	      	   dvigatel_zapushen=0;
			  	      	   dvigstopweb=0;          //когда двигатель будет заглушен то можно глушить вебасто
			  	         }


			  	         if((dvigatel_zapushen==1)&(dvigstopweb==0)){ //если двигатель запущен и вебасто  еще не глушилась то глушим вебасто
			  	      	  if(action==1){
			  	      		//HAL_TIM_Base_Stop(&htim16);    //выключаем таймер
						    //HAL_TIM_Base_Stop_IT(&htim16);  //выключаем прерывания
						    flag_start=0;
						    tim16_counter=0;
        	  		  	    HAL_GPIO_WritePin(GPIOA, led1_Pin,RESET);  //отключаем светодиод
        	  		  	    short_actionflag=0;   //переводим из короткой работый вебасто в длиную
        	  		  	   // Rele1(ON);
        	  		  	    long_actionflag=1;
        	  		  	    dvigatel_zapushen=0;

			  	      	             }
			  	         }

			 }//от short action flag








			  //////конец вставки напряжения измерения
////////////////конец Второй генерациии проекта///////////////////////////
	  ////////////////////////////////////////первая генерация проекта////////////
          //1//////////////////////////////////////////////////////////////
          /*


	  if(HAL_GPIO_ReadPin(GPIOA, in1_Pin)==GPIO_PIN_SET){ //смотрим нажатие кнопки
	  		  while(HAL_GPIO_ReadPin(GPIOA, in1_Pin)==GPIO_PIN_SET){
	  			  buttonstart_count=buttonstart_count+1;
	  		  }
	  	  }else{
	  		  buttonstart_count=0;
	  	  }



	  	  if((200000>buttonstart_count)&(buttonstart_count>3000)){  //смотрим действительно ли была нажата кнопка


	  		  flag_start=1;   //разрешаем запуск вебасто

	  		  buttonstart_count=0;

	  	  }else{
	  		  if(buttonstart_count>200000){
	  			  flag_stop=1;
	  	          flag_start=0;
	  			  buttonstart_count=0;
	  		  }
	  	  }

	  	  //если пришел флаг на запуск -заводим вебасто
	  	  if((flag_start==1)&(action==0)){
	  		  flag_start=0;
	  		  HAL_GPIO_WritePin(GPIOA, rele2_Pin, GPIO_PIN_SET);  //включаем помпу и вебасту
	  		//  HAL_GPIO_WritePin(GPIOA, rele2_Pin, GPIO_PIN_SET);/
	  		  HAL_TIM_Base_Start(&htim16);    //запускаем таймер
	  		  HAL_TIM_Base_Start_IT(&htim16);  //запускаем прерывания
	  		  action=1;  // система завелась
	  		  flag_start=0;// запуск больше не требуется
	  	  }
	  	  //моргаем светиком пока работаем прошла секунда моргаем

	  	  if((old_time_counter<tim16_counter)&(action==1)){
	  		  HAL_GPIO_TogglePin(GPIOA, led1_Pin);
	  	      old_time_counter=tim16_counter;
	  	  }

	  	  //если прошло 25мин то все выключаем
	  	  if((tim16_counter>time_progrev)&((action==1))){

	  		          HAL_GPIO_WritePin(GPIOA, rele2_Pin, GPIO_PIN_RESET);  //включаем помпу и вебасту
	  		  		 action=0;  // система потухла
	  		  		tim16_counter=0;   //сбрасываем счетчики
	  		  		old_time_counter=0;  //сбрасываем счетчики
	  		  		good=1;  //флаг успешного завершения прогрева
	  		  		flag_start=0;
	  	  }
	  	  //если была принудительная остановка то
	  	  if(flag_stop==1){

	  		  HAL_GPIO_WritePin(GPIOA, rele2_Pin, GPIO_PIN_RESET);  //выключаем помпу и вебасту

	  		  			   HAL_GPIO_WritePin(GPIOA, led2_Pin, GPIO_PIN_SET);
	  		  			   HAL_Delay(2000);

	  		  HAL_GPIO_WritePin(GPIOA, led2_Pin, GPIO_PIN_RESET);
	  		  HAL_GPIO_WritePin(GPIOA, led1_Pin, GPIO_PIN_RESET);

	  		  tim16_counter=0;
	  		  action=0;  // система потухла
	  		  old_time_counter=0;  //сбрасываем счетчики
	  		   good=0;  //флаг успешного завершения прогрева


	  		   flag_start=0;
	  		   flag_stop=0;
	  		   HAL_TIM_Base_Stop(&htim16);    //выключаем таймер
	  		   HAL_TIM_Base_Stop_IT(&htim16);  //выключаем прерывания
	  		   buttonstart_count=0;
	  	 	  }
	  	  //включаем зеленый светодиод на 20 мин типа все отрабоатло ок
	  	  if((good==1)&(action==0)&(flag_stop==0)){

	  		//  HAL_GPIO_WritePin(GPIOA, led2_Pin, GPIO_PIN_SET);

	  		 if((tim16_counter<time_posle_raboty)&(led_ok_set==0)){
	  		  HAL_GPIO_WritePin(GPIOA, led1_Pin, GPIO_PIN_SET);
	  		 // HAL_Delay(3000);
	  		  led_ok_set=1;
	  	             }
	  		 //если зажигание включили то выключаем светодиод
	  		 if(ig_On==1){
	  			 tim16_counter=time_posle_raboty+1;
	  		 }
	  		  if(tim16_counter>time_posle_raboty){
	  			 // HAL_GPIO_WritePin(GPIOA, led2_Pin, GPIO_PIN_SET);
	  		  HAL_GPIO_WritePin(GPIOA, led1_Pin, GPIO_PIN_RESET);
	  		  good=0;
	  		  tim16_counter=0;
	  		   HAL_TIM_Base_Stop(&htim16);    //выключаем таймер
	  		   HAL_TIM_Base_Stop_IT(&htim16);  //выключаем прерывания
	            flag_start=0;
	            buttonstart_count=0;
	            led_ok_set=0;           //сбрасываем флаг включения светодиода
	  		  }
	  		  }//от включения светодиода на 25мин
	        //Проверка включения зажигания
	  	  if(HAL_GPIO_ReadPin(GPIOA, in2_Pin)==GPIO_PIN_SET){ //смотрим включено ли зажигание
	  	 	        HAL_Delay(1);
	  	 	       HAL_GPIO_WritePin(GPIOA, led2_Pin, GPIO_PIN_SET);
	  	 	       	 		//     HAL_Delay(1000);
	  	 	        if(HAL_GPIO_ReadPin(GPIOA, in2_Pin)==GPIO_PIN_SET){
	  	 		     ig_On=1;   //ставим флаг что зажигание включено
	  	 		     HAL_GPIO_WritePin(GPIOA, led2_Pin, GPIO_PIN_SET);
	  	 		     //HAL_Delay(1000);
	  	 	                                                     }
	  	 	}else{
	  	 		  ig_On=0;   //иначе зажигание выключено
	  	 		 HAL_GPIO_WritePin(GPIOA, led2_Pin, GPIO_PIN_RESET);
	  	 	  }
	        //конец анализа включения зажигания
	  ////////////////////////////////////////////////////////////////////////
	  	  //здесь меряем напряжение
	  	  HAL_ADC_Start(&hadc);      //запускаем измерения аналогового входа
	        HAL_ADC_PollForConversion(&hadc, 100);    //дожидаемся окончания имерения
	        adcResult = HAL_ADC_GetValue(&hadc);     //забираем реультат
	        HAL_ADC_Stop(&hadc);          //останавливаем измерения

	        Utek=adcResult*Upit/MiConstRazr; //вычисляем сколько вольт
	        U=Utek*100;        //получаем первые трицифры чтоы округлить
	        U3=(double)U;      //меняем тип начения для функции floor
	        U4=floor(U3+0.5);    //floor округление до целого но оно еще double
	        U2=(int)U4;           // меняем тип double на  int для дальнейшей раоты с массивом
	        //расклаываем четырех значное число  полученое и ADC по клеточка массива
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
	                   // сливаем все значения массива miadc (uint) в mistr (string)
	                             sprintf(mistr,"%d%d%d%d\r\n", miadc[0],miadc[1],miadc[2],miadc[3]);
	                             //отправляем массив mistr в порт
	                            HAL_UART_Transmit(&huart1,mistr,6,0xFFFF);
	                              	  HAL_Delay(200);
	                	  //Склеиваем массив Mimassiv типа string из цифрового массива Mimassiv
	                                  Mimassiv[0]=(U2%1000-U2%100)/100;
	                                  Mimassiv[1]=(U2%100-U2%10)/10;
	                                  Mimassiv[2]=U2%10;
	                            sprintf(uvoltage,"%d%c%d%d\r\n", Mimassiv[0],'.',Mimassiv[1],Mimassiv[2]);
	                   //отправляем значение напряжения на алоговом входе в порт
	                             HAL_UART_Transmit(&huart1,uvoltage,6,0xFFFF);
	                              HAL_Delay(200);
	                                     */
	   //Далее вычисляем напряжение на входе в схему до делителя
    //2///////////////////////////////////////////////////////////////////////////////
	/*        UR2=Utek;      //меняем тип данных
	        I2=UR2/R2;             //считаем ток через резистор R2 -4700
	       Ubortseti=I2*(R1+R2);  //ВЫчисляем входное напряжение
	       // далее таже эпопея по переводу в массив для отправки в порт


	                     Ubortseti=Ubortseti*100;// получаем первые четыре цифры напряжения борт сети
	                     Ubortseti=floor(Ubortseti+0.5); //округляем
	                     x=Ubortseti;
	                     Ubsint=(int)Ubortseti;
	             //далеее расчленем для массива типа инт
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
	                       //склеиваем массив строку
	                   //отправляем входное напряжение в USART
	                       // sprintf(ubseti,"%c\r\n",usdecimal[0]);
	               /*        sprintf(ubseti,"%d%d%d%d\r\n", miadc[0],miadc[1],miadc[2],miadc[3]);
	                     if(Ubsint>999) {
	                       HAL_UART_Transmit(&huart1,ubseti,7,0xFFFF);
	                       HAL_Delay(200);
	                     }          */
	        /////////////////////////конец замера напряжения
	              //Имеем переменную Ubsint  1236   напряжение в int
	          //если Ubsint  больше  1290  то двигатель запущен
	          //если Ubsint  меньше 1000   то сел акб
	         //здесь рассматриваем севший аккумулятор
	  //3///////////////////////////////////////////////////////////////////////////////
        /*  if(Ubsint<1000){
	      	   sel_akb=1;
	         }else{
	      	   sel_akb=0;
	      	   alarm_akbstopweb=0;   //скидываем флаг аварийного стопа вебасто по севшему акб
	         }
	              if((sel_akb==1)&(alarm_akbstopweb==0)){    //если сел акб и вебасто работает и еще не глушилась то тушим ее
	              	if(action==1){
	              		flag_stop=1;
	              		alarm_akbstopweb=1;
	              	}
	              }

	            ///здесь рассматриваем ситуацию выключения вебасто при запуске двс
	         if(Ubsint>1290){
	      	   dvigatel_zapushen=1;
	         }
	         else{
	      	   dvigatel_zapushen=0;
	      	   dvigstopweb=0;          //когда двигатель будет заглушен то можно глушить вебасто
	         }


	         if((dvigatel_zapushen==1)&(dvigstopweb==0)){ //если двигатель запущен и вебасто  еще не глушилась то глушим вебасто
	      	  if(action==1){
	      		  flag_stop=1;
	      	   dvigstopweb=1;      //вебасто потушена и нех ее больше тыркать
	      	              }
	      	                                         }
	         */
////////////////////////////////////////////////////////////////////////////////////////////////////

	         ////////конец первой генерации///////////
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
