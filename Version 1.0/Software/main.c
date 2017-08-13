#include "stm32l0xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define gpsRstOn GPIOA->BSRR = (1 << 1)
#define BuzzerOn GPIOA->BSRR = (1 << 7)
#define GSMPowerKeyOn GPIOA->BSRR = (1 << 11)
#define Out1On GPIOB->BSRR = (1 << 8)
#define Out2On GPIOB->BSRR = (1 << 9)
#define Out3On GPIOB->BSRR = (1 << 4)
#define Out4On GPIOB->BSRR = (1 << 5)

#define gpsRstOff GPIOA->BRR = (1 << 1)
#define BuzzerOff GPIOA->BRR = (1 << 7)
#define GSMPowerKeyOff GPIOA->BRR = (1 << 11)
#define Out1Off GPIOB->BRR = (1 << 8)
#define Out2Off GPIOB->BRR = (1 << 9)
#define Out3Off GPIOB->BRR = (1 << 4)
#define Out4Off GPIOB->BRR = (1 << 5)

#define ContactInput (GPIOA->IDR & (1 << 12))
#define input2 (GPIOA->IDR & (1 << 15))
#define input3 (GPIOB->IDR & (1 << 14))
#define ButtonInput GPIOA->IDR & (1 << 0)
#define ChargeStatusInput GPIOA->IDR & (1 << 15)

unsigned char port[]="18001";
unsigned char checksum[]="*0F";
unsigned char domain[]="92.42.39.110";
unsigned char imei[]="352432060288395";
unsigned char swVersion[]= {"V1.6"};
unsigned char gps[100],contactStatus=0;
double speedKm, distance;
unsigned const char maxSpeed=120;

char  *date, *time, *gpsValidation, *latitude, *northorSouth, *longitude, *eastorWest, positionFix[5]="1", 
	satilliteNumber[5]="12", altitude[5]="0", *speedKnots;

char *token, delim[] = ",";
char nmeadata[100] = "$GPRMC,225446,A,4916.45,N,12311.12,W,000.5,054.7,191194,020.3,E*68";



unsigned int dly=0,i=0,smsNumber=0,forKick=0;
char printBuffer[100]="";
volatile int gpsDataReceivedFlag=0;
volatile char gpsReceived[80],usart2=1,gpsDataReceivedForSpeedFlag=0;
volatile char gsmReceived[80],usart1=1,sendCheckFlag=0,immoStateFlag=0;
volatile char rfidReceived[80],usart3=1,rfidFlag=0;

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

//USART1
void GsmSendChar(char ch)
{
    while (!(USART1->ISR & (1UL << 7)));
    USART1->TDR = (ch & 0x1FF);
}

void GsmSendString(char * str)
{
    while(*str != 0)
    {
        GsmSendChar(*str);
        str++;
    }
}

//USART2
void GPS_SendChar(char ch)
{
    while (!(USART2->ISR & (1UL << 7)));
    USART2->TDR = (ch & 0x1FF);
}

void GPS_SendStr(char * str)
{
    while(*str != 0)
    {
        GPS_SendChar(*str);
        str++;
    }
}

void WatchdogInit(){

//  RCC->CSR |= (1<<0);                                           // LSI enable, necessary for IWDG
//  while ((RCC->CSR & (1<<1)) == 0);                             // wait till LSI is ready

//  IWDG->KR  = 0x5555;                                           // enable write to PR, RLR
//  IWDG->PR  = __IWDG_PR;                                        // Init prescaler
//  IWDG->RLR = __IWDG_RLR;                                       // Init RLR
//  IWDG->KR  = 0xAAAA;                                           // Reload the watchdog
//  IWDG->KR  = 0xCCCC;                                           // Start the watchdog	
	
//RCC->CSR |=RCC_CSR_LSION; //Internal Low Speed oscillator enable
IWDG->KR= 0x5555; //Writing the key value 5555h to enable access to the IWDG_PR and IWDG_RLR registers
IWDG->PR=0xDD;	 // Init prescaler
IWDG->RLR=0xFFF; //Reload register
IWDG->KR=0xAAAA; // Reload the watchdog
IWDG->KR=0xCCCC; //Writing the key value CCCCh starts the watchdog (except if the hardware watchdog option is selected)
}
void Kick(){
	IWDG->KR= 0xAAAA; //These bits must be written by software at regular intervals with the key value AAAAh,otherwise the watchdog generates a reset when the counter reaches 0.
}

void GpsInit()
{
    //GPS_SendStr("$PMTK314,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0*30\r\n"); //RMC Data every 5 sec
		GPS_SendStr("$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"); //RMC Data every 1 sec
   //for(dly = 0; dly < 200000; dly++);
   // GPS_SendStr("$PQTXT,W,0,1*23\r\n");    //disable GPTXT output and save it in flash

    //GPS_SendStr("$PMTK314,0,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*2D\r\n"); //GGA Datasi
    //GPS_SendStr("$PMTK353,1,1,1,1,1*2A\r\n"); //Full GPS
    //GPS_SendStr("$PMTK251,38400*27");//38400 Baud
}

void GsmInit()
{
    //for(dly = 0; dly < 20000000; dly++);
    GsmSendString("AT\r\n");
        while( i < 500000 ){
					i++;
					if( gsmReceived[1]=='O' ){
						gsmReceived[1]='\0';break;
					}
				}
				i=0;
    GsmSendString("AT+QIFGCNT=0\r\n");  //Set the context 0 as FGCNT.
        while( i < 500000 ){
					i++;
					if( gsmReceived[1]=='O' ){
						gsmReceived[1]='\0';break;
					}
				}
				i=0;
    GsmSendString("AT+QICSGP=1,\"INTERNET\"\r\n");  //Set the context 0 as FGCNT.
        while( i < 500000 ){
					i++;
					if( gsmReceived[1]=='O' ){
						gsmReceived[1]='\0';break;
					}
				}
				i=0;
    GsmSendString("AT+QIMUX=0\r\n"); // 0 disables the function of MUXIP.
        while( i < 500000 ){
					i++;
					if( gsmReceived[1]=='O' ){
						gsmReceived[1]='\0';break;
					}
				}
				i=0;
    GsmSendString("AT+QIMODE=1\r\n"); // Set the session mode as transparent.
        while( i < 500000 ){
					i++;
					if( gsmReceived[1]=='O' ){
						gsmReceived[1]='\0';break;
					}
				}
				i=0;
    GsmSendString("AT+QITCFG=3,2,512,1\r\n");
        while( i < 500000 ){
					i++;
					if( gsmReceived[1]=='O' ){
						gsmReceived[1]='\0';break;
					}
				}
				i=0;
    GsmSendString("AT+QIDNSIP=1\r\n");  //Use IP address 0 or use Domain with 1.
        while( i < 500000 ){
					i++;
					if( gsmReceived[1]=='O' ){
						gsmReceived[1]='\0';break;
					}
				}
				i=0;
    GsmSendString("AT+QIREGAPP\r\n"); // Register the TCP/IP stack.
        while( i < 500000 ){
					i++;
					if( gsmReceived[1]=='O' ){
						gsmReceived[1]='\0';break;
					}
				}
				i=0;
    GsmSendString("AT+QIACT\r\n"); // Activate FGCNT.
        while( i < 500000 ){
					i++;
					if( gsmReceived[1]=='O' ){
						gsmReceived[1]='\0';break;
					}
				}
				i=0;
    GsmSendString("AT+QILOCIP\r\n"); //Get Local IP address.
        while( i < 500000 ){
					i++;
					if( gsmReceived[1]=='1' ){
						gsmReceived[1]='\0';break;
					}
				}
				i=0;
    sprintf(printBuffer,"AT+QIOPEN=\"TCP\",\"%s\",%s\r\n", domain, port);
    GsmSendString(printBuffer); //Connect to the server
          while( i < 20000000 ){
					i++;
					if( gsmReceived[1]=='C' ){
						gsmReceived[1]='\0';break;
					}
				}
					
}

void IntroductionMessage(){
		GsmSendChar(0x02); // Start Text
    sprintf(printBuffer,"|RGS,%s%s", imei, checksum);
    GsmSendString(printBuffer);
		GsmSendChar(0x03); // End of Text
		GsmSendString("\r\n");
}

void GpsParse(){
	
		for(i=1; gpsReceived[i]!='*'; i++)
		nmeadata[i]=gpsReceived[i]; // Firstly, copy data to the array, because strtok function changes the data

		token = strtok(nmeadata, delim);
		time = strtok(NULL, ",");				 
		gpsValidation = strtok(NULL, ","); // GPS Valid or Invalid
		latitude = strtok(NULL, ",");
		northorSouth = strtok(NULL, ",");
    longitude = strtok(NULL, ",");
		eastorWest = strtok(NULL, ",");
		speedKnots = strtok(NULL, ",");
		token = strtok(NULL, ","); // True Course
		date = strtok(NULL, ",");
		token = strtok(NULL, ","); // Variation
		
if( strcmp(gpsValidation,"V") ){ //Eger gps datasi Valid ise distance hesaplansin
					speedKm=strtod(speedKnots, NULL); //String to float
					speedKm=1.852*speedKm; //Knots to KM
					distance+=speedKm/3600;
//	sprintf(printBuffer,"distance=%.4lf speedKm=%.1lf\r\n", distance, speedKm);
//	GsmSendString(printBuffer);
}

}

void GetSms(){
	//+CMTI: "SM",4  gelen sms uyari ornegi
	GsmSendString("AT+CMGF=1\r\n"); /* Use AT+CMGF=1 to set text mode */
	        while( i < 500000 ){
					i++;
					if( gsmReceived[1]=='O' ){
						gsmReceived[1]='\0';break;
					}
				}
				i=0;	
	GsmSendString("AT+CSDH=1\r\n"); /* Use AT+CSDH=1 to show header values only in text mode */
	        while( i < 500000 ){
					i++;
					if( gsmReceived[1]=='O' ){
						gsmReceived[1]='\0';break;
					}
				}
				i=0;
	GsmSendString("AT+CSCS=\"GSM\"\r\n"); /* Use AT+CSCS to set character type (default setting:AT+CSCS="GSM") */
	        while( i < 500000 ){
					i++;
					if( gsmReceived[1]=='O' ){
						gsmReceived[1]='\0';break;
					}
				}
				i=0;	
	sprintf(printBuffer, "AT+CMGR=%d\r\n", smsNumber); /* Read the message which will change the status of the message */
	GsmSendString(printBuffer); 
	        while( i < 500000 ){
					i++;
					if( gsmReceived[1]=='O' ){
						gsmReceived[1]='\0';break;
					}
				}
				i=0;	
	GsmSendString("AT+CSDH=0\r\n"); /* Use AT+CSDH=0 to hide header values only in text mode */
	        while( i < 500000 ){
					i++;
					if( gsmReceived[1]=='O' ){
						gsmReceived[1]='\0';break;
					}
				}
				i=0;	
	sprintf(printBuffer, "AT+CMGD=%d\r\n", smsNumber); /* Use AT+CMGD=<index> to delete a message */
	GsmSendString(printBuffer); 
	        while( i < 500000 ){
					i++;
					if( gsmReceived[1]=='O' ){
						gsmReceived[1]='\0';break;
					}
				}
				i=0;	
}

void SendSms(char *sendText){
//GsmSendString("AT+GSN\r\n"); /* Use AT+GSN to query the IMEI of module */
	/* Use AT+CSQ to query current signal quality */
	/* Use AT+COPS? to query current Network Operator */
	/* use AT+CREG? /AT+CGREG? to query the network registration status, if the return value is [0,1] or [0,5], it is successfully registered, other value is fail to register */
	GsmSendString("AT+CMGF=1\r\n"); /* Use AT+CMGF=1 to set text mode */
	        while( i < 500000 ){
					i++;
					if( gsmReceived[1]=='O' ){
						gsmReceived[1]='\0';break;
					}
				}
				i=0;
	GsmSendString("AT+CSMP=17,167,0,0\r\n"); /* Use AT+CSMP to set SMS parameter for text mode (default setting: AT+CSMP=17,167,0,0) */
	        while( i < 500000 ){
					i++;
					if( gsmReceived[1]=='O' ){
						gsmReceived[1]='\0';break;
					}
				}
				i=0;				
	GsmSendString("AT+CSCS=\"GSM\"\r\n"); /* Use AT+CSCS to set character type (default setting:AT+CSCS="GSM") */
	        while( i < 500000 ){
					i++;
					if( gsmReceived[1]=='O' ){
						gsmReceived[1]='\0';break;
					}
				}
				i=0;		
	GsmSendString("AT+CMGS=\"05321791881\"\r\n"); /* AT+CMGS="05321791881", and then wait for the">" appears, input your message after the ">", use <CTRL+Z> or 1A (HEX String) to send a message,when receive +CMGS:<index> and OK , means the message has been sent successfully. */
	        while( i < 500000 ){
					i++;
					if( gsmReceived[1]=='>' ){
						gsmReceived[1]='\0';break;
					}
				}
				i=0;		
	GsmSendString(sendText);		
	GsmSendChar(0x1A);	
	        while( i < 500000 ){ /* If OK mesage gets the message has been sent succesfully */
					i++;
					if( gsmReceived[1]=='O' ){
						gsmReceived[1]='\0';break;
					}
				}
				i=0;					
}

void SendTrackingData(){
	//<STX>|GPS,359394058270633,040416,104600,3825.52420,N,02710.29471,E,2,09,0.1,130,1,00001,15*AA<ETX><CR><LF>
	
  sendCheckFlag=0; //OK Onayi flag'i sifirlaniyor
  gpsDataReceivedFlag=0; //GPS data geldi flag'i sifirlaniyor
		
	GpsParse();

	GsmSendChar(0x02); // Start Text		 
	sprintf(printBuffer,"|GPS,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%.1lf,0000%d,%.4lf%s", 
	imei, date, time, latitude, 
	northorSouth, longitude, eastorWest, positionFix, satilliteNumber, 
	altitude, speedKm, contactStatus, distance, checksum);
	GsmSendString(printBuffer);
	GsmSendChar(0x03); // End of Text
	GsmSendString("\r\n");
	
}

void SentDataControl()
{
    for(dly = 0; dly < 3000000; dly++);
		Kick();
    if(sendCheckFlag==0)
    {
        GsmSendString("+++");
        for(dly = 0; dly < 1500000; dly++);
        GsmSendString("AT+QICLOSE\r\n");
        for(dly = 0; dly < 500000; dly++);
        GsmSendString("AT+QILOCIP\r\n"); //Get IP
        for(dly = 0; dly < 500000; dly++);
        sprintf(printBuffer,"AT+QIOPEN=\"TCP\",\"%s\",%s\r\n", domain, port);
        GsmSendString(printBuffer); //Connect to the server
        for(dly = 0; dly < 3000000; dly++);
			
				Kick();
        if(gsmReceived[1]=='A')  //Eger ALREADY CONNECT ise
        {
            GsmSendString("ATO\r\n");//Return last connection status
            for(dly = 0; dly < 500000; dly++);
        }
        IntroductionMessage();
    }
}

void Disconnect(){
	      GsmSendString("+++");
        for(dly = 0; dly < 1500000; dly++);
        GsmSendString("AT+QICLOSE\r\n");
        for(dly = 0; dly < 500000; dly++);
				Kick();
}


void USART1_IRQHandler(void)
{
    gsmReceived[usart1]=USART1->RDR;
    if( (gsmReceived[1]=='B') && (gpsReceived[2]=='Z') )BuzzerOn;
    if(gsmReceived[usart1]=='\n')
    {
        usart1=0;
        
        if( (gsmReceived[23]=='0') && (gsmReceived[24])=='1' )immoStateFlag=1;
        if( (gsmReceived[23]=='0') && (gsmReceived[24])=='0' )immoStateFlag=0;
        if( (gsmReceived[1]=='O') && (gsmReceived[2])=='K' )sendCheckFlag=1;
    }
    usart1++;
}

void USART2_IRQHandler(void)
{
    //USART1->TDR=USART2->RDR;

    gpsReceived[usart2]=USART2->RDR;
    if(gpsReceived[usart2]=='\n')
    {
        usart2=0;
        gpsDataReceivedFlag++;//GPS data geldi flag'i
				gpsDataReceivedForSpeedFlag=1; //GPS data geldi flag'i hiz hesaplamasi için her saniye
    }
    usart2++;
}


int main(void)
{

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_ADC_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();

    USART1->CR1  |= USART_CR1_RXNEIE;	//USART1 Receive Interrupt Enable
    USART2->CR1  |= USART_CR1_RXNEIE; //USART2 Receive Interrupt Enable
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_EnableIRQ(USART2_IRQn);

	//	WatchdogInit();
		Kick();
	//    gpsRstOn;
    for(dly = 0; dly < 1000000; dly++);

		GpsInit();
//    GSMPowerKeyOn;
//		BuzzerOn;
//    for(dly = 0; dly < 200000; dly++);
//    BuzzerOff;
//    for(dly = 0; dly < 400000; dly++);
//		GSMPowerKeyOff;
    
//  while (1){
//		
//		            if(gpsDataReceivedFlag>=1)  //Send data every 15 mins
//            {
//							gpsDataReceivedFlag=0;
//		sprintf(printBuffer, "%d\r\n", i);
//		GsmSendString(printBuffer);
//		i++;
//							if(i>=9){
//								i=0;
//								Kick();
//								GsmSendString("Kicked");
//							}
//						}
//	}



	
    for(dly = 0; dly < 5000000; dly++);
		
		Kick();
  //  GsmInit();

    IntroductionMessage();
    BuzzerOn;
    for(dly = 0; dly < 200000; dly++);
    BuzzerOff;
		
Kick();
    while (1){
//*************Kontak kapali***********
        if( !((gpsReceived[2]=='G') && (gpsReceived[3]=='P') && (gpsReceived[4]=='R') && (gpsReceived[5]=='M') && (gpsReceived[6]=='C')) )GpsInit();
       contactStatus=0;// Sisteme kontak durumunu göndermek için, yeni sistemde contactInput degiskeni ile gönderilecek
				while(ContactInput)
        {
					if(forKick>=7){
					Kick();
					forKick=0;
					}
					forKick++;
					Kick();
					
					
            for(dly = 0; dly < 500000; dly++);
            if(gpsDataReceivedFlag>=60)  //Send data every 15 mins
            {
                //send_string1("$GPRMC,225446,A,4916.45,N,12311.12,W,000.5,054.7,191194,020.3,E*68\r\n");
                //$PGID,12345609*OF
                SendTrackingData();
                SentDataControl();
							Disconnect();
							GetSms();
            }
        }

//*************Arac Calisiyor**********
				if(immoStateFlag==1)
					Out3On;
        else
					Out3Off;
        if( !((gpsReceived[2]=='G') && (gpsReceived[3]=='P') && (gpsReceived[4]=='R') && (gpsReceived[5]=='M') && (gpsReceived[6]=='C')) )GpsInit();
       contactStatus=1; // Sisteme kontak durumunu göndermek için, yeni sistemde contactInput degiskeni ile gönderilecek
				while( !(ContactInput) )
        {
          if(forKick==70){
					Kick();
					forKick=0;
					}
					forKick++;
					
					if(gpsDataReceivedForSpeedFlag>=1){
					gpsDataReceivedForSpeedFlag=0;
  				GpsParse();
					
					}

            for(dly = 0; dly < 50000; dly++);
            if(gpsDataReceivedFlag>=15){
                SendTrackingData();
                SentDataControl();
            }
        }


    }
    /* USER CODE END 3 */

}




/** System Clock Configuration
*/
void SystemClock_Config(void)
{

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    __PWR_CLK_ENABLE();

    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                                         |RCC_PERIPHCLK_I2C1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
void MX_ADC_Init(void)
{

    ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
    hadc.Instance = ADC1;
    hadc.Init.OversamplingMode = DISABLE;
    hadc.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV1;
    hadc.Init.Resolution = ADC_RESOLUTION12b;
    hadc.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
    hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc.Init.ContinuousConvMode = DISABLE;
    hadc.Init.DiscontinuousConvMode = DISABLE;
    hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIG_EDGE_NONE;
    hadc.Init.DMAContinuousRequests = DISABLE;
    hadc.Init.EOCSelection = EOC_SINGLE_CONV;
    hadc.Init.Overrun = OVR_DATA_PRESERVED;
    hadc.Init.LowPowerAutoWait = DISABLE;
    hadc.Init.LowPowerFrequencyMode = DISABLE;
    hadc.Init.LowPowerAutoPowerOff = DISABLE;
    HAL_ADC_Init(&hadc);

    /**Configure for the selected ADC regular channel to be converted.
    */
    sConfig.Channel = ADC_CHANNEL_4;
    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
    HAL_ADC_ConfigChannel(&hadc, &sConfig);

    /**Configure for the selected ADC regular channel to be converted.
    */
    sConfig.Channel = ADC_CHANNEL_5;
    HAL_ADC_ConfigChannel(&hadc, &sConfig);

}

/* I2C1 init function */
void MX_I2C1_Init(void)
{

    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x00000708;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
    HAL_I2C_Init(&hi2c1);

    /**Configure Analogue filter
    */
    HAL_I2CEx_AnalogFilter_Config(&hi2c1, I2C_ANALOGFILTER_ENABLED);

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 9600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    HAL_UART_Init(&huart1);

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

    huart2.Instance = USART2;
    huart2.Init.BaudRate = 9600;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    HAL_UART_Init(&huart2);

}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __GPIOH_CLK_ENABLE();
    __GPIOA_CLK_ENABLE();
    __GPIOB_CLK_ENABLE();

    /*Configure GPIO pins : PA0 PA15 PA12 */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_15|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : PA1 PA11 PA7 */
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_11|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : PB3 PB15 */
    GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


    /*Configure GPIO pins : PB4 PB5 PB9 PB8 */
    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_11|GPIO_PIN_15, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_9, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
      ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
