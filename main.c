/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//adres czujnika
#define MPU9255_ADDR				0x68 << 1//adres "LOW"? ("HIGH" 0x69 albo 0x77)
#define MPU9255_WHO_AM_I 			0x75
#define MPU9255_ADDR_ACCELCONFIG    0x1C
#define MPU9255_ADDR_ACCELCONFIG_2  0x1D
#define MPU9255_ADDR_ACCEL_XOUT_H	0x3B
#define MPU9255_ADDR_ACCEL_XOUT_L	0x3C
#define MPU9255_ADDR_ACCEL_YOUT_H	0x3D
#define MPU9255_ADDR_ACCEL_YOUT_L	0x3E
#define MPU9255_ADDR_ACCEL_ZOUT_H	0x3F
#define MPU9255_ADDR_ACCEL_ZOUT_L	0x40
#define MPU9255_ADDR_PWR_MGMT		0x6B
#define MPU9255_ADDR_PWR_MGMT_2		0x6C
#define MPU9255_ADDR_TEMP_OUT_H		0x41
#define MPU9255_ADDR_TEMP_OUT_L		0x42
#define MPU9255_ADDR_GYRO_CONFIG	0x1B
#define MPU9255_ADDR_GYRO_XOUT_H	0x43
#define MPU9255_ADDR_GYRO_XOUT_L	0x44
#define MPU9255_ADDR_GYRO_YOUT_H	0x45
#define MPU9255_ADDR_GYRO_YOUT_L	0x46
#define MPU9255_ADDR_GYRO_ZOUT_H	0x47
#define MPU9255_ADDR_GYRO_ZOUT_L	0x48
#define MPU9255_ADDR_USER_CTRL		0x6A
#define MPU9255_ADDR_INT_BYPASS		0x37

//ponizsze adresy odnosza sie do magnetometru
#define MPU9255_ADDR_AK8963			0x0C << 1//moze zamiast tego to 0x77? wedle dokumentacji 0x0C
#define MPU9255_ADDR_INT_PIN_CFG    0x37
#define MPU9255_ADDR_MAG_CNTL_1		0x0A
#define MPU9255_ADDR_ASAX			0x10
#define MPU9255_ADDR_ASAY			0x11
#define MPU9255_ADDR_ASAZ			0x12
#define MPU9255_MAG_STATUS_1		0x02
#define MPU9255_DRDY_MASK 			0x01
#define MPU9255_DRDY				0x01

#define MPU9255_ADDR_MAG_HXL		0x03
#define MPU9255_ADDR_MAG_HXH		0x04
#define MPU9255_ADDR_MAG_HYL		0x05
#define MPU9255_ADDR_MAG_HYH		0x06
#define MPU9255_ADDR_MAG_HZL		0x07
#define MPU9255_ADDR_MAG_HZH		0x08

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//funkcja __io_putchar przekierowywuje wiadomosci pojawiajace sie na I2C i przekierowywuje je na UART
//wiadomosci w UART mozna odczytac programem zewnetrznym (PuTTY albo Tera Term)
//UWAGA bardzo ladne ctrl+c ctrl+v z forbot, trzeba przepisac

int __io_putchar(int ch)
{
  if (ch == '\n') {
    __io_putchar('\r');
  }

  HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);

  return 1;
}
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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf("Szukanie czujnika...\n");
  uint8_t who_am_i = 0;
  HAL_I2C_Mem_Read(&hi2c1, MPU9255_ADDR, MPU9255_WHO_AM_I, 1, &who_am_i, sizeof(who_am_i), HAL_MAX_DELAY);

  if (who_am_i == 0x71) {
   printf("Znaleziono: MPU9255\n");
  } else {
   printf("Blad: (0x%02X)\n", who_am_i);
  }

  //inicjalizacja podstawowa czujnika oraz akcelelometru i zyroskopu
  uint8_t pwr_mgmt = 0x00; //00000000, RS-MPU strona 40
  if (HAL_I2C_Mem_Write(&hi2c1, MPU9255_ADDR, MPU9255_ADDR_PWR_MGMT, 1, &pwr_mgmt, sizeof(pwr_mgmt), HAL_MAX_DELAY) != HAL_OK) {
	  printf("Blad przy zapisaniu MPU9255_ADDR_PWR_MGMT\n");
  }
  HAL_Delay(10);
  uint8_t pwr_mgmt_2 = 0x00; //00000000, RS-MPU strona 41
  if (HAL_I2C_Mem_Write(&hi2c1, MPU9255_ADDR, MPU9255_ADDR_PWR_MGMT_2, 1, &pwr_mgmt_2, sizeof(pwr_mgmt_2), HAL_MAX_DELAY) != HAL_OK) {
	  printf("Blad przy zapisaniu MPU9255_ADDR_PWR_MGMT_2\n");
  }
  uint8_t accelconfig = 0x00; //odpowiada to wlaczeniu akcelerometru +-2g, RS-MPU strona 14
  if (HAL_I2C_Mem_Write(&hi2c1, MPU9255_ADDR, MPU9255_ADDR_ACCELCONFIG, 1, &accelconfig, sizeof(accelconfig), HAL_MAX_DELAY) != HAL_OK) {
	  printf("Blad przy zapisaniu MPU9255_ADDR_ACCELCONFIG\n");
  }
  uint8_t accelconfig_2 = 0x00; //RS-MPU strona 15
  if (HAL_I2C_Mem_Write(&hi2c1, MPU9255_ADDR, MPU9255_ADDR_ACCELCONFIG_2, 1, &accelconfig_2, sizeof(accelconfig_2), HAL_MAX_DELAY) != HAL_OK) {
	  printf("Blad przy zapisaniu MPU9255_ADDR_ACCELCONFIG_2\n");
  }
  uint8_t gyroconfig = 0x00; //RS-MPU strona 13
  if (HAL_I2C_Mem_Write(&hi2c1, MPU9255_ADDR, MPU9255_ADDR_GYRO_CONFIG, 1, &gyroconfig, sizeof(gyroconfig), HAL_MAX_DELAY) != HAL_OK) {
	  printf("Blad przy zapisaniu MPU9255_ADDR_GYRO_CONFIG\n");
  }





  //magnetometr, inicjalizacja

  if (HAL_I2C_Mem_Write(&hi2c1, MPU9255_ADDR, MPU9255_ADDR_PWR_MGMT, 1, &pwr_mgmt, sizeof(pwr_mgmt), HAL_MAX_DELAY) != HAL_OK) {
	  printf("Blad przy zapisaniu PWR_MGMT\n");
  }
  HAL_Delay(10);

  uint8_t int_pin_cfg;
  if (HAL_I2C_Mem_Read(&hi2c1, MPU9255_ADDR, MPU9255_ADDR_INT_PIN_CFG, 1, &int_pin_cfg, sizeof(int_pin_cfg), HAL_MAX_DELAY) != HAL_OK) {
      printf("Blad przy odczycie INT_PIN_CFG\n");
  }
  int_pin_cfg |= 0x02;
  if (HAL_I2C_Mem_Write(&hi2c1, MPU9255_ADDR, MPU9255_ADDR_INT_PIN_CFG, 1, &int_pin_cfg, sizeof(int_pin_cfg), HAL_MAX_DELAY) != HAL_OK) {
      printf("Blad przy zapisaniu INT_PIN_CFG\n");
  }
  HAL_Delay(50);

  uint8_t powerdown = 0x00;
  if (HAL_I2C_Mem_Write(&hi2c1, MPU9255_ADDR_AK8963, MPU9255_ADDR_MAG_CNTL_1, 1, &powerdown, sizeof(powerdown), HAL_MAX_DELAY) != HAL_OK) {
      printf("Blad przy zapisaniu MAG_CNTL_1 (powerdown)\n");
  }
  HAL_Delay(50);

  uint8_t fuserom = 0x0F;
  if (HAL_I2C_Mem_Write(&hi2c1, MPU9255_ADDR_AK8963, MPU9255_ADDR_MAG_CNTL_1, 1, &fuserom, sizeof(fuserom), HAL_MAX_DELAY) != HAL_OK) {
      printf("Blad przy zapisaniu MAG_CNTL_1 (fuse ROM)\n");
  }
  HAL_Delay(50);

  uint8_t ASAX, ASAY, ASAZ;
  if (HAL_I2C_Mem_Read(&hi2c1, MPU9255_ADDR_AK8963, MPU9255_ADDR_ASAX, 1, &ASAX, sizeof(ASAX), HAL_MAX_DELAY) != HAL_OK) {
      printf("Blad przy odczycie ASAX\n");
  }
  if (HAL_I2C_Mem_Read(&hi2c1, MPU9255_ADDR_AK8963, MPU9255_ADDR_ASAY, 1, &ASAY, sizeof(ASAY), HAL_MAX_DELAY) != HAL_OK) {
      printf("Blad przy odczycie ASAY\n");
  }
  if (HAL_I2C_Mem_Read(&hi2c1, MPU9255_ADDR_AK8963, MPU9255_ADDR_ASAZ, 1, &ASAZ, sizeof(ASAZ), HAL_MAX_DELAY) != HAL_OK) {
      printf("Blad przy odczycie ASAZ\n");
  }

  if (HAL_I2C_Mem_Write(&hi2c1, MPU9255_ADDR_AK8963, MPU9255_ADDR_MAG_CNTL_1, 1, &powerdown, sizeof(powerdown), HAL_MAX_DELAY) != HAL_OK) {
      printf("Blad przy zapisaniu MAG_CNTL_1 (powerdown after fuse ROM)\n");
  }
  HAL_Delay(50);

  uint8_t continous_8hz = 0x12;
  if (HAL_I2C_Mem_Write(&hi2c1, MPU9255_ADDR_AK8963, MPU9255_ADDR_MAG_CNTL_1, 1, &continous_8hz, sizeof(continous_8hz), HAL_MAX_DELAY) != HAL_OK) {
      printf("Blad przy zapisaniu MAG_CNTL_1 (continuous 8Hz)\n");
  }
  HAL_Delay(50);

  HAL_Delay(1000);





  //definiowanie zmiennych do kalibracji
  int measured_gyro_count = 0;
  int c_gyro_x = 0;
  int c_gyro_y = 0;
  int c_gyro_z = 0;
  int measured_mag_count = 0;
  float mX_max = -10.0, mX_min = 10.0;
  float mY_max = -10.0, mY_min = 10.0;
  float mZ_max = -10.0, mZ_min = 10.0;


  while (1)
  {

	  //AKCELELOMETR

	  int16_t accelX;
	  uint8_t accelX_h, accelX_l;
	  if (HAL_I2C_Mem_Read(&hi2c1, MPU9255_ADDR, MPU9255_ADDR_ACCEL_XOUT_H, 1, &accelX_h, sizeof(accelX_h), HAL_MAX_DELAY) != HAL_OK) {
		  printf("Blad przy odczycie accelX_h\n");
	  }
	  if (HAL_I2C_Mem_Read(&hi2c1, MPU9255_ADDR, MPU9255_ADDR_ACCEL_XOUT_L, 1, &accelX_l, sizeof(accelX_l), HAL_MAX_DELAY) != HAL_OK) {
		  printf("Blad przy odczycie accelX_l\n");
	  }
	  accelX = (int16_t)((accelX_h << 8) | accelX_l);
	  float aX = (float)accelX / 16384.0f;
	  //printf("accelX = %d\n", accelX);
	  //printf("aX = %f\n", aX);

	  int16_t accelY;
	  uint8_t accelY_h, accelY_l;
	  if (HAL_I2C_Mem_Read(&hi2c1, MPU9255_ADDR, MPU9255_ADDR_ACCEL_YOUT_H, 1, &accelY_h, sizeof(accelY_h), HAL_MAX_DELAY) != HAL_OK) {
		  printf("Blad przy odczycie accelY_h\n");
	  }
	  if (HAL_I2C_Mem_Read(&hi2c1, MPU9255_ADDR, MPU9255_ADDR_ACCEL_YOUT_L, 1, &accelY_l, sizeof(accelY_l), HAL_MAX_DELAY) != HAL_OK) {
		  printf("Blad przy odczycie accelY_l\n");
	  }
	  accelY = (int16_t)((accelY_h << 8) | accelY_l);
	  float aY = (float)accelY / 16384.0f;
	  //printf("accelY = %d\n", accelY);
	  //printf("aY = %f\n", aY);

	  int16_t accelZ;
	  uint8_t accelZ_h, accelZ_l;
	  if (HAL_I2C_Mem_Read(&hi2c1, MPU9255_ADDR, MPU9255_ADDR_ACCEL_ZOUT_H, 1, &accelZ_h, sizeof(accelZ_h), HAL_MAX_DELAY) != HAL_OK) {
		  printf("Blad przy odczycie accelZ_h\n");
	  }
	  if (HAL_I2C_Mem_Read(&hi2c1, MPU9255_ADDR, MPU9255_ADDR_ACCEL_ZOUT_L, 1, &accelZ_l, sizeof(accelZ_l), HAL_MAX_DELAY) != HAL_OK) {
		  printf("Blad przy odczycie accelZ_l\n");
	  }
	  accelZ = (int16_t)((accelZ_h << 8) | accelZ_l);
	  float aZ = (float)accelZ / 16384.0f;
	  //printf("accelZ = %d\n", accelZ);
	  //printf("aZ = %f\n", aZ);

	  float accelSqrt = sqrt(pow(aX, 2) + pow(aY, 2) + pow(aZ, 2));
	  //printf("accelSqrt = %f\n", accelSqrt);

	  printf("accelX = %f   accelY = %f   accelZ = %f   accelSqrt = %f\n", aX, aY, aZ, accelSqrt);

	  //AKCELELOMETR KALIBRACJA
	  //przewiduje ze kod bedzie polegal na planowanym kolejnym polozeniu ramienia NACHI
	  //oczekiwac sie bedzie przez HAL_DELAY az znajdzie sie odpowiednie polozenie (w stosunku do polozenia od ziemii)
	  //i zostanie wykonany pomiar, poniewaz producent przewiduje aby wynik pokazywany byl w jednostce "g" to niech
	  //ta jednostka bedzie odpowiednia dla Gdańska, a nie dla średniej światowej
	  //
	  //zamiast HAL_DELAY mozna uzyc przerwania systemowe (doczytac)
	  //w pierwszej wersji programu lepiej najpierw wykonac zyroskop (przeciez wykonywuje sie on w pozycji lezacej)




	  //TERMOMETR
	  //niepotrzebny, do usuniecia w przyszlosci, na razie daje pewnosc dzialania urzadzenia (byl dobry do nauki)

	  int16_t temp;
	  uint8_t temp_out_h, temp_out_l;
	  HAL_I2C_Mem_Read(&hi2c1, MPU9255_ADDR, MPU9255_ADDR_TEMP_OUT_H, 1, &temp_out_h, sizeof(temp_out_h), HAL_MAX_DELAY);
	  HAL_I2C_Mem_Read(&hi2c1, MPU9255_ADDR, MPU9255_ADDR_TEMP_OUT_L, 1, &temp_out_l, sizeof(temp_out_l), HAL_MAX_DELAY);

	  temp = (int16_t)((temp_out_h << 8) | temp_out_l);

	  //printf("Raw T = %d\n", temp);
	  printf("T = %.1f*C\n", ((temp)/333.87f)+21.0f);




	  //ZYROSKOP

	  int16_t gyroX;
	  uint8_t gyroX_h, gyroX_l;
	  if (HAL_I2C_Mem_Read(&hi2c1, MPU9255_ADDR, MPU9255_ADDR_GYRO_XOUT_H, 1, &gyroX_h, sizeof(gyroX_h), HAL_MAX_DELAY) != HAL_OK) {
		  printf("Blad przy odczycie gyroX_h\n");
	  }
	  if (HAL_I2C_Mem_Read(&hi2c1, MPU9255_ADDR, MPU9255_ADDR_GYRO_XOUT_L, 1, &gyroX_l, sizeof(gyroX_l), HAL_MAX_DELAY) != HAL_OK) {
		  printf("Blad przy odczycie gyroX_l\n");
	  }
	  gyroX = (int16_t)((gyroX_h << 8) | gyroX_l);
	  //printf("gyroX = %d\n", gyroX);

	  int16_t gyroY;
	  uint8_t gyroY_h, gyroY_l;
	  if (HAL_I2C_Mem_Read(&hi2c1, MPU9255_ADDR, MPU9255_ADDR_GYRO_YOUT_H, 1, &gyroY_h, sizeof(gyroY_h), HAL_MAX_DELAY) != HAL_OK) {
		  printf("Error reading gyroY_h\n");
	  }
	  if (HAL_I2C_Mem_Read(&hi2c1, MPU9255_ADDR, MPU9255_ADDR_GYRO_YOUT_L, 1, &gyroY_l, sizeof(gyroY_l), HAL_MAX_DELAY) != HAL_OK) {
		  printf("Error reading gyroY_l\n");
	  }
	  gyroY = (int16_t)((gyroY_h << 8) | gyroY_l);
	  //printf("gyroY = %d\n", gyroY);

	  int16_t gyroZ;
	  uint8_t gyroZ_h, gyroZ_l;
	  if (HAL_I2C_Mem_Read(&hi2c1, MPU9255_ADDR, MPU9255_ADDR_GYRO_ZOUT_H, 1, &gyroZ_h, sizeof(gyroZ_h), HAL_MAX_DELAY) != HAL_OK) {
		  printf("Error reading gyroZ_h\n");
	  }
	  if (HAL_I2C_Mem_Read(&hi2c1, MPU9255_ADDR, MPU9255_ADDR_GYRO_ZOUT_L, 1, &gyroZ_l, sizeof(gyroZ_l), HAL_MAX_DELAY) != HAL_OK) {
		  printf("Error reading gyroZ_l\n");
	  }
	  gyroZ = (int16_t)((gyroZ_h << 8) | gyroZ_l);
	  //printf("gyroZ = %d\n", gyroZ);

	  float gX = (float)gyroX / 131.0f;
	  float gY = (float)gyroY / 131.0f;
	  float gZ = (float)gyroZ / 131.0f;
	  printf("gyroX = %f   gyroY = %f   gyroZ = %f\n", gX, gY, gZ);

	  //ZYROSKOP KALIBRACJA

	  if(measured_gyro_count < 2001) {
		  c_gyro_x += gX;
		  c_gyro_y += gY;
		  c_gyro_z += gZ;
		  measured_gyro_count += 1;
	  }
	  if (measured_gyro_count == 2001) {
		  int cgX = c_gyro_x / 2000;
		  int cgY = c_gyro_y / 2000;
		  int cgZ = c_gyro_z / 2000;
		  measured_gyro_count += 1;
	  }


	  //MAGNETOMETR

	  int16_t magX, magY, magZ;
	  uint8_t magX_h, magX_l, magY_h, magY_l, magZ_h, magZ_l;

	  if (HAL_I2C_Mem_Read(&hi2c1, MPU9255_ADDR_AK8963, MPU9255_ADDR_MAG_HXL, 1, &magX_l, sizeof(magX_l), HAL_MAX_DELAY) != HAL_OK) {
		  printf("Blad przy odczycie MAG_HXL\n");
	  }
	  if (HAL_I2C_Mem_Read(&hi2c1, MPU9255_ADDR_AK8963, MPU9255_ADDR_MAG_HXH, 1, &magX_h, sizeof(magX_h), HAL_MAX_DELAY) != HAL_OK) {
		  printf("Blad przy odczycie MAG_HXH\n");
	  }
	  magX = (int16_t)((magX_h << 8) | magX_l);

	  if (HAL_I2C_Mem_Read(&hi2c1, MPU9255_ADDR_AK8963, MPU9255_ADDR_MAG_HYL, 1, &magY_l, sizeof(magY_l), HAL_MAX_DELAY) != HAL_OK) {
		  printf("Blad przy odczycie MAG_HYL\n");
	  }
	  if (HAL_I2C_Mem_Read(&hi2c1, MPU9255_ADDR_AK8963, MPU9255_ADDR_MAG_HYH, 1, &magY_h, sizeof(magY_h), HAL_MAX_DELAY) != HAL_OK) {
		  printf("Blad przy odczycie MAG_HYH\n");
	  }
	  magY = (int16_t)((magY_h << 8) | magY_l);

	  if (HAL_I2C_Mem_Read(&hi2c1, MPU9255_ADDR_AK8963, MPU9255_ADDR_MAG_HZL, 1, &magZ_l, sizeof(magZ_l), HAL_MAX_DELAY) != HAL_OK) {
		  printf("Blad przy odczycie MAG_HZL\n");
	  }
	  if (HAL_I2C_Mem_Read(&hi2c1, MPU9255_ADDR_AK8963, MPU9255_ADDR_MAG_HZH, 1, &magZ_h, sizeof(magZ_h), HAL_MAX_DELAY) != HAL_OK) {
		  printf("Blad przy odczycie MAG_HZH\n");
	  }
	  magZ = (int16_t)((magZ_h << 8) | magZ_l);

	  // Offset wartosci odczytanych z czujnika
	  float mX = ((float)magX * ((((float)ASAX - 128.0f) / 256.0f) + 1.0f));
	  float mY = ((float)magY * ((((float)ASAY - 128.0f) / 256.0f) + 1.0f));
	  float mZ = ((float)magZ * ((((float)ASAZ - 128.0f) / 256.0f) + 1.0f));

	  // Horyzont
	  double mHorizon = atan2((double)magX, (double)magY) * 180.0 / 3.14159;

	  printf("magX = %f   magY = %f   magZ = %f   Horizon = %f\n", mX, mY, mZ, mHorizon);



	  //MAGNETOMETR KALIBRACJA


	  if(measured_mag_count < 100001) {
		  if (mX > mX_max) mX_max = magX;
		  if (mX < mX_min) mX_min = magX;

		  if (mY > mY_max) mY_max = magY;
		  if (mY < mY_min) mY_min = magY;

		  if (mZ > mZ_max) mZ_max = magZ;
		  if (mZ < mZ_min) mZ_min = magZ;
	  		  measured_mag_count += 1;
	  	  }
	  	  if (measured_mag_count == 100001) {
	  		float biasX = (mX_max + mX_min) / 2.0;
	  		float biasY = (mY_max + mY_min) / 2.0;
	  		float biasZ = (mZ_max + mZ_min) / 2.0;

	  		float scaleX = (magX_max - magX_min) / 2.0;
	  		float scaleY = (magY_max - magY_min) / 2.0;
	  		float scaleZ = (magZ_max - magZ_min) / 2.0;

	  		float avgScale = (scaleX + scaleY + scaleZ) / 3.0;

	  		float scaleFactorX = avgScale / scaleX;
	  		float scaleFactorY = avgScale / scaleY;
	  		float scaleFactorZ = avgScale / scaleZ;


	  		//hard iron
	  		float HI[3] = {biasX, biasY, biasZ};

	  		//soft iron
	  		float SI[3][3] = {
	  		    {scaleFactorX, 0, 0},
	  		    {0, scaleFactorY, 0},
	  		    {0, 0, scaleFactorZ}
	  		};

	  		measured_mag_count += 1;
	  	  }




	  //Przygotowanie magnetometru do kolejnego pomiaru
	  uint8_t singlemeasurement = 0x01;
	  if (HAL_I2C_Mem_Write(&hi2c1, MPU9255_ADDR_AK8963, MPU9255_ADDR_MAG_CNTL_1, 1, &singlemeasurement, sizeof(singlemeasurement), HAL_MAX_DELAY) != HAL_OK) {
		  printf("Blad przy zapisaniu MPU9255_ADDR_MAG_CNTL_1\n");
	  }

	  HAL_Delay(750);
	  printf("\n");
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
