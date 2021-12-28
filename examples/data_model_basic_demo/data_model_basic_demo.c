#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <unistd.h>
#include <pthread.h>  

#include "cJSON.h"
#include "tuya_cacert.h"
#include "tuya_log.h"
#include "tuya_error_code.h"
#include "system_interface.h"
#include "mqtt_client_interface.h"
#include "tuyalink_core.h"

#include <wiringPi.h>
#include "sx126x_v01.h"

const char productId[] = "t1tlm6p13aouheta";
const char deviceId[] = "6cf918e90b12f7b1ffwiuz";
const char deviceSecret[] = "a5f23a3fb341edbd";


tuya_mqtt_context_t client_instance;
tuya_mqtt_context_t context_cmd;

typedef union
{   
        double  f_type;
        uint8_t val[8];
}FL_TYPE;
FL_TYPE hh;

#define LORA_MODE	1
#define FSK_MODE    0

#define TRANSMITTER     0
#define RECEIVER        1

#define RX_CONTINOUS    1//连续接收

#if (TRANSMITTER == RECEIVER)
    #error "Please define only Transmitter or receiver."
#endif

#define TEST_MODE	3  	//0-infinite preamble TX mode（暂时只是做了lora）
					            //1-continous CW TX 
					           	//其他值才能进入到收发分离模式
						
#if (LORA_MODE == FSK_MODE)
    #error "Please define only LoRa or FSK."
#endif


#define TX_OUTPUT_POWER                             22        // dBm  //测出来是18.536
#define RF_FREQUENCY                                490000000//480000000//915000000//470000000 // Hz


#if (FSK_MODE==1)

#define FSK_FDEV                                    10000//38400//25e3      // Hz 
#define FSK_DATARATE                                19200//40e3      // bps
#define FSK_BANDWIDTH                               93800//93800////58600  140e3     // Hz >> DSB in sx126x
#define FSK_AFC_BANDWIDTH                           100000     // Hz
#define FSK_PREAMBLE_LENGTH                         5         // Same for Tx and Rx
#define FSK_FIX_LENGTH_PAYLOAD_ON                   true
#define FSK_FIX_LENGTH_PAYLOAD 						          10
#define FSK_CRC										true
 
#elif (LORA_MODE==1)

#define LORA_BANDWIDTH                             1     // [0: 125 kHz,
															//	1: 250 kHz, 														 
															//	2: 500k
															//	3 :20.83kHz
															//	4:31.25kHz
															//	5:62.5kHz4
															//6:41.67
#define LORA_SPREADING_FACTOR                       10        // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx  SF5&6 will automatilly change to 12
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#endif


#define HAL_MAX_DELAY      0xFFFFFFFFU

#define RX_TIMEOUT_VALUE                            1000
#define TX_TIMEOUT                                  65535 
#define BUFFER_SIZE                                 250//10//250 // Define the payload size here

#define CADTIMEOUT_MS								2000   //CAD timeout 时间  用ms表示

uint8_t dat;
uint8_t cnt=0x55;
uint8_t recdat=0;
uint8_t version=0;

uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE]={0};

int8_t RssiValue = 0;
int8_t SnrValue = 0;

PacketStatus_t RadioPktStatus;
uint8_t RadioRxPayload[255];
uint8_t RadioRxPacketSize;
uint8_t SendCnt=0;

volatile bool TXDone=false;
volatile bool RXDoneFlag=false;
volatile bool TimeOutFlag=false;
volatile bool CRCFail=false;

volatile int Cnt1=0;

const RadioLoRaBandwidths_t Bandwidths_copy[] = { LORA_BW_125, LORA_BW_250, LORA_BW_500,LORA_BW_020,LORA_BW_031,LORA_BW_062,LORA_BW_041 };

void LoRa_GPIO_Init(void);

int initSPI()
{
  int spiFd; //spi的文件描述符
  //初始化SPI通道0，并设置为速度500k
  spiFd=wiringPiSPISetup(0,500000); //默认模式0：SCLK空闲时为低电平，第一个时间沿采样
  if(spiFd==-1)
  {
    printf("init spi failed!\n");
  }
}

//写文件
int write_file(char str[])
{
  FILE *fp = NULL; 
  fp = fopen("/tmp/from_platform.txt", "w");
  fprintf(fp, "%s", str);
  fclose(fp);
  printf("deviceid_action_time: %s\n", str); 
  return 0;
}
//读文件
int read_file(int num,char return_data[])
{
  FILE *fp = NULL;
  switch (num)
    {
    case 1:
        fp = fopen("/tmp/1.txt", "r");
        if(fp == NULL)
        {
           printf("open /tmp/1.txt error!\n");
           return 0;
        }
        fgets(return_data, 255, (FILE*)fp);
        fclose(fp);
        break;
    case 2:
        fp = fopen("/tmp/2.txt", "r");
        if(fp == NULL)
        {
           printf("open /tmp/2.txt error!\n");
           return 0;
        }
        fgets(return_data, 255, (FILE*)fp);
        fclose(fp);
        break;
    case 3:
        fp = fopen("/tmp/3.txt", "r");
        if(fp == NULL)
        {
           printf("open /tmp/3.txt error!\n");
           return 0;
        }
        fgets(return_data, 255, (FILE*)fp);  
        fclose(fp); 
        break;
    default:
        break;
    }
    printf("return_data:%s\n",return_data);
    return 0;
}
//写一个线程函数 void *函数名（void *arg）
void *thread_worker1(void *arg) 
{  
    char json_temp_hum[255];
    char json_door_state[255];
    char json_data[255];
    char json_data1[255];    
    while(1)
    {  
        if(read_file(2,json_data) == 0)
        {
            sprintf(json_temp_hum,"%s",json_data);
            printf("json_temp_hum:%s\r\n",json_temp_hum);
            tuyalink_thing_property_report_with_ack(arg, NULL, json_temp_hum);
            memset(json_data, 0, sizeof(json_data));  
            
        }
        else
        {
            printf("read temp_hum error\r\n");

        }
        if(read_file(3,json_data1) == 0)
        {
            sprintf(json_door_state,"%s",json_data1);
            printf("json_door_state:%s\r\n",json_door_state);
            tuyalink_thing_event_trigger(arg, NULL, json_door_state); 
            memset(json_data1, 0, sizeof(json_data1));  
        }
        else
        {
            printf("read door_state error\r\n");

        }
        sleep(2); 
    }  
}
void *thread_worker2(void *arg) 
{

  
  uint8_t i=0;
  bool DetectTruetable[100]={0};//CAD成功的分布
  bool RXTruetable[100]={0};//CAD后能接收正确的分布
  uint8_t CadDetectedTime=0;//检测到的cad的次数
  uint8_t RxCorrectTime=0;  //RX 接收正确次数
  uint8_t TxTime=0;		      //TX 次数
  
 //连续发送的时候用
  uint8_t ModulationParam[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  uint8_t PacketParam[9] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

  context_cmd=*(tuya_mqtt_context_t*)arg;

  if(wiringPiSetup()<0)
  {
    printf("init wiringPi error\n");
  }
  
  initSPI();  //spi的初始化
  LoRa_GPIO_Init();
	
  SX126xReset();
  i=SX126xReadRegister(REG_LR_CRCSEEDBASEADDR);
  if(i==0x1D)
  {   
  	printf("SPI SUCCESS!\n\r");
  }

  else
  {
	printf("SPI Fail! REG_LR_CRCSEEDBASEADDR=%x\n\r",i);
  }
  RadioInit();
  SX126xWriteRegister(0x889, SX126xReadRegister(0x889) & 0xfB);//SdCfg0 (0x889) sd_res (bit 2) = 0 
  printf("RadioInit Done!\n\r");


#if (TEST_MODE==0)   //infinite preamble TX mode
	//连续发送
	SX126xSetStandby( STDBY_RC );
	SX126xSetPacketType(PACKET_TYPE_LORA);//todo: 增加发射FSK模式下的改指令
	
	printf("set lora params\n");
	ModulationParam[0]=LORA_SPREADING_FACTOR;
	ModulationParam[1]=Bandwidths_copy[LORA_BANDWIDTH];
	ModulationParam[2]=LORA_CODINGRATE;
	ModulationParam[3]=0;//1:SF11 and SF12 0:其他 低速率优化  
	SX126xWriteCommand( RADIO_SET_MODULATIONPARAMS, ModulationParam, 4 );//lora发射参数配置


	//设置lora包参数
	PacketParam[0]=(LORA_PREAMBLE_LENGTH>>8)& 0xFF;
	PacketParam[1]=LORA_PREAMBLE_LENGTH;
	PacketParam[2]=LORA_FIX_LENGTH_PAYLOAD_ON;//head type
	PacketParam[3]=0xFF;//0Xff is MaxPayloadLength
	PacketParam[4]=true;//CRC on
	PacketParam[5]=LORA_IQ_INVERSION_ON;
	SX126xWriteCommand( RADIO_SET_PACKETPARAMS, PacketParam, 6 );

	//SX126xWriteBuffer( 0x00, SendData, 10 );

	//连续发送lora
	SX126xSetRfFrequency( RF_FREQUENCY );
  SX126xSetRfTxPower( TX_OUTPUT_POWER );
	SX126xSetTxInfinitePreamble();

	printf("TxContinuousWave Now--infinite preamble!\n\r");
	while(1);
#elif (TEST_MODE==1) //TX CW

	RadioSetTxContinuousWave( RF_FREQUENCY, TX_OUTPUT_POWER, TX_TIMEOUT );
	printf("TxContinuousWave Now---CW!\n\r");
	while(1);

#endif


#if (FSK_MODE==1)

	SX126xSetRfFrequency(RF_FREQUENCY);
	RadioSetTxConfig( MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, FSK_BANDWIDTH,
						FSK_DATARATE, 0,
						FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
						true, 0, 0, 0, 3000 );
	
	RadioSetRxConfig( MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
						0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
						0, FSK_FIX_LENGTH_PAYLOAD_ON, FSK_FIX_LENGTH_PAYLOAD, FSK_CRC,0, 0,false, RX_CONTINOUS );
	
	printf("FSK:%d,Fdev=%ld,BitRate=%ld,BW=%ld,PWR=%d,PreLen=%d,PYLOAD=%d\n\r",RF_FREQUENCY,FSK_FDEV,FSK_DATARATE,FSK_BANDWIDTH,TX_OUTPUT_POWER,FSK_PREAMBLE_LENGTH,BUFFER_SIZE);
	printf("configure FSK parameters done\n!");

	

#elif (LORA_MODE==1)

			SX126xSetRfFrequency(RF_FREQUENCY);
			RadioSetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
									   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
									   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
									   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );


		  RadioSetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
										 LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
										 LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
										 0, true, 0, 0, LORA_IQ_INVERSION_ON, RX_CONTINOUS );//最后一个参数设置是否是连续接收


		
		  printf("LORA:%d,SF=%d,codeRate=%d,BW=%d,PWR=%d,PreLen=%d,PYLOAD=%d\n\r",RF_FREQUENCY,LORA_SPREADING_FACTOR,LORA_CODINGRATE,LORA_BANDWIDTH,TX_OUTPUT_POWER,LORA_PREAMBLE_LENGTH,BUFFER_SIZE);
		  if (RadioPublicNetwork.Previous==true && RadioPublicNetwork.Current==false)
			printf("public\n\r");
		  else if (RadioPublicNetwork.Previous==false && RadioPublicNetwork.Current==false)
			printf("private\n\r");
	    printf("configure LORA parameters done\n!");
		 
#endif

while (1)
{
 #if (TRANSMITTER==1)
    while(1)
    {
      Buffer[0] = TxTime++;
      Buffer[1] = 1;
      Buffer[2] = 2;
      Buffer[3] = 3;
      Buffer[4] = 0;
      Buffer[5] = 0;
      RadioSend(Buffer,20);
    while(TXDone==false && TimeOutFlag==false);//一直等待tx done
    TXDone=false;
    TimeOutFlag=false;
    printf("TxTime=%d\n",TxTime);
    HAL_Delay(500); ///1s

    //读取状态
    RadioStatus=SX126xGetStatus();
    printf("RadioStatus is(after TX_DONE) %d\n",(((RadioStatus.Value)>>4)&0x07));

    }
 #elif (RECEIVER==1) 
while(1)
{	
 #if (RX_CONTINOUS==1)
    //开始接收	
    RadioRx(0xFFFFFF);//50MS(0XC80)超时  0-单次接收 无超时
    printf("continous RX...\n");
    while(1);//连续接收
  #endif
    RadioRx(2000);//50MS(0XC80)超时  0-单次接收 无超时
    while(RXDoneFlag==false && TimeOutFlag==false && CRCFail==false);
    if(RXDoneFlag==true || TimeOutFlag==true || CRCFail==true)
    {
      if(CRCFail==false)	//CRC无错误
      {
        if(RXDoneFlag==true)
        {
          printf("\n%d:RxCorrect-PING\n",RxCorrectTime);
          RxCorrectTime++;
        }
      }
      CRCFail=false;
      RXDoneFlag=false;
      TimeOutFlag=false;
    }
  }
 #endif		  
}
}
//DIO1的中断函数
void LoRaHandler(void)
{
	uint16_t irqRegs = SX126xGetIrqStatus( );
   SX126xClearIrqStatus( IRQ_RADIO_ALL );//这里清掉中断标志
     //发送结束
	 if( ( irqRegs & IRQ_TX_DONE ) == IRQ_TX_DONE )
        {
		      TXDone=true;
            digitalWrite(LED_Pin, HIGH);
            system_sleep(1000);
            digitalWrite(LED_Pin, LOW);
            system_sleep(1000);
		        OnTxDone();			
        }
		//在SX126xSetTx()设置了一个超时时间 可以检测改功能 --ok
 	 	if( ( irqRegs & IRQ_RX_TX_TIMEOUT ) == IRQ_RX_TX_TIMEOUT )
        {
		   TimeOutFlag=true;
		   printf(" RX/TX timeout\n");
        }
        if( ( irqRegs & IRQ_RX_DONE ) == IRQ_RX_DONE )
        {
            SX126xGetPayload( RadioRxPayload, &RadioRxPacketSize , 255 );
            SX126xGetPacketStatus( &RadioPktStatus );
            digitalWrite(LED_Pin, HIGH);
            system_sleep(1000);
            digitalWrite(LED_Pin, LOW);
            system_sleep(1000);
            OnRxDone();
            RXDoneFlag=true;
      }

      if( ( irqRegs & IRQ_CRC_ERROR ) == IRQ_CRC_ERROR )
        {
                printf("CRC fail\n");
                CRCFail=true;
        }
      if( ( irqRegs & IRQ_CAD_DONE ) == IRQ_CAD_DONE )
      {
			if ( ( irqRegs & IRQ_CAD_ACTIVITY_DETECTED ) == IRQ_CAD_ACTIVITY_DETECTED ) 
			{
           //printf("IRQ_CAD_ACTIVITY_DETECTED\n");	
           //CadDetect=true;
			}            
      }
      if( ( irqRegs & IRQ_PREAMBLE_DETECTED ) == IRQ_PREAMBLE_DETECTED )
        {
           
        }
      if( ( irqRegs & IRQ_SYNCWORD_VALID ) == IRQ_SYNCWORD_VALID )
        {
             
        }
      if( ( irqRegs & IRQ_HEADER_VALID ) == IRQ_HEADER_VALID )
        {

        }   
}
void OnTxDone(void)
{
  SleepParams_t params = { 0 };
  params.Fields.WarmStart = 1;//热启动
  //printf("params.value=%d\n",params.Value);
  //SX126xSetSleep( params );//热启动可以保存进入睡眠前的状态的相关寄存器  sleep可以清掉所有中断标志位
  printf("OnTxDone\n");

}
void OnRxDone()
{
  int i;
  uint16_t 	 tempvaluelora;
  uint16_t 	 humvaluelora;
  uint16_t   lightvaluelora;
  char	templora[128]= {0};
  char 	humlora[128]= {0};
  char  lightlora[128]= {0};
	system_sleep(100);
	printf("onRXDone\n");
  SleepParams_t params = { 0 };
  params.Fields.WarmStart = 1;//热启动

	//TODO:处理包的数据
	if (RadioPktStatus.packetType==PACKET_TYPE_LORA)
    printf("LoRa:PacketSIZE=%d,RSSI=%d,SNR=%d\n",
			RadioRxPacketSize,
					RadioPktStatus.Params.LoRa.RssiPkt, 
							RadioPktStatus.Params.LoRa.SnrPkt);
	else if (RadioPktStatus.packetType==PACKET_TYPE_GFSK)
	printf("FSK:PacketSIZE=%d,RssiAvg=%d,RssiSync=%d\n",
			RadioRxPacketSize,
					RadioPktStatus.Params.Gfsk.RssiAvg, 
							RadioPktStatus.Params.Gfsk.RssiSync);     
	printf("Payload: ");
	for(i=0;i<RadioRxPacketSize;i++)
	{
		printf("0x%x ",RadioRxPayload[i]);
	}
  //收到的数据送到平台
  for(i=0;i<8;i++)
	 {
		hh.val[i]=RadioRxPayload[i];	 
	 }
    tempvaluelora=hh.f_type*100;
    printf("temp_lora=%d ",tempvaluelora);
    sprintf(templora,"{\"templora\":{\"value\":%d,\"time\":1631708204231}}",tempvaluelora);
    tuyalink_thing_property_report_with_ack(&context_cmd, NULL, templora); 
	 
    memset(hh.val,0,8);  
    for(i=0;i<8;i++)
    {
      hh.val[i]=RadioRxPayload[8+i];	 
    }
    humvaluelora=hh.f_type*100;
    printf("hum_lora=%d ",humvaluelora);
    sprintf(humlora,"{\"humlora\":{\"value\":%d,\"time\":1631708204231}}",humvaluelora);
    tuyalink_thing_property_report_with_ack(&context_cmd, NULL, humlora); 


  	lightvaluelora=RadioRxPayload[17]*256+RadioRxPayload[16];
    printf("light_lora=%d ",lightvaluelora);
    sprintf(lightlora,"{\"lightlora\":{\"value\":%d,\"time\":1631708204231}}",lightvaluelora);
    tuyalink_thing_property_report_with_ack(&context_cmd, NULL, lightlora); 

/*!
* \brief   Sx1268 DIO1 interrupt callback to deal with LoRa statemachine
* \para    none
* \retval  none
*/
void EXTI0_IRQHandler(void)
{
	 LoRaHandler();
}
}
//管脚的初始化和中断函数注册
/*!
* \brief   Initialize RasperryPi GPIO which connects to Sx1268 DIO1 with WiringPi driver
* \para    none
* \retval  none
*/
//主要是设置DIO1_PIN为输入模式，设置上拉，然后注册外部中断函数EXT0_IRQHandler.
void LoRa_GPIO_Init(void)				
{
    pinMode(SW_CTL1_Pin, OUTPUT);
    pinMode(SW_CTL2_Pin, OUTPUT);
    pinMode(ANT_SWITCH_POWER_Pin, OUTPUT);
    pinMode(NRESET_Pin, OUTPUT);
    pinMode(LED_Pin, OUTPUT);
    pinMode(SPI_CS_Pin, OUTPUT);
    pinMode(BUSY_Pin, INPUT);
	  pinMode(DIO1_PIN, INPUT);
	  pullUpDnControl (DIO1_PIN, PUD_UP);
	  wiringPiISR(DIO1_PIN,INT_EDGE_RISING,&EXTI0_IRQHandler);
    digitalWrite(SW_CTL1_Pin, LOW);
    digitalWrite(SW_CTL2_Pin, LOW);
    digitalWrite(SPI_CS_Pin, HIGH);
    digitalWrite(ANT_SWITCH_POWER_Pin, LOW);
    digitalWrite(NRESET_Pin, LOW);
    digitalWrite(LED_Pin, HIGH);

}
void on_connected(tuya_mqtt_context_t* context, void* user_data)
{ 
    int error=0;
    pthread_t t1,t2;
    tuyalink_subdevice_bind(context, "[{\"productId\":\"snigjkwkheaxueqa\",\"nodeId\":\"255\",\"clientId\":\"1\"}]");//继电器
    tuyalink_subdevice_bind(context, "[{\"productId\":\"dtoqgbr5azgwvga3\",\"nodeId\":\"254\",\"clientId\":\"2\"}]");//门磁
    tuyalink_subdevice_bind(context, "[{\"productId\":\"6jmmnuwavyxkcv1x\",\"nodeId\":\"1\",\"clientId\":\"3\"}]");//温湿度
    tuyalink_subdevice_bind(context, "[{\"productId\":\"oneta7v3ahcdgxe1\",\"nodeId\":\"2\",\"clientId\":\"4\"}]");//lora
    //tuyalink_subdevice_bind_login(context, "[\"6ce7f8ed6a76407b9brbk3\"]");//平台创建新子设备时要上线一下，不然会显示离线，之后可以删掉此函数。
    error=pthread_create(&t1,NULL,thread_worker1,context);
    error=pthread_create(&t2,NULL,thread_worker2,context);
    if(error)
      {
        printf("create pthread error!\n");
        return;     
      } 
}

void on_disconnect(tuya_mqtt_context_t* context, void* user_data)
{
    TY_LOGI("on disconnect");
}


void on_messages(tuya_mqtt_context_t* context, void* user_data, const tuyalink_message_t* msg)
{
    char json_relay_state[255];
    TY_LOGI("on message id:%s, type:%d, code:%d", msg->msgid, msg->type, msg->code);
    switch (msg->type) {
        case THING_TYPE_MODEL_RSP:
            TY_LOGI("Model data:%s", msg->data_string);
            break;

        case THING_TYPE_PROPERTY_SET:
            TY_LOGI("property set:%s", msg->data_string);
            break;

        case  THING_TYPE_PROPERTY_REPORT_RSP:
           break;

        case THING_TYPE_ACTION_EXECUTE:
              TY_LOGI("action execute:%s", msg->data_string);
              if(write_file(msg->data_string) == 0)
                 {
                    printf ("write ok\r\n");
                    printf ("data_string:%s\r\n",msg->data_string);
                 }
             else
                {
                    printf ("write error\r\n");
                }
                 if(read_file(1,json_relay_state) == 0)
                 {
                   sprintf(msg->data_string,"%s",json_relay_state);
                   printf("json_relay_state:%s\r\n",json_relay_state);
                   tuyalink_thing_property_report(context, NULL, msg->data_string);  
                   memset(json_relay_state, 0, sizeof(json_relay_state));      
                 }
                 else
                 {
                  printf ("read relay_state error\r\n");
                 }     
            break;   

        default:
            break;
    }
    printf("\r\n");
}

int main(int argc, char** argv)
{
    int ret = OPRT_OK;
    tuya_mqtt_context_t* client = &client_instance;
    ret = tuya_mqtt_init(client, &(const tuya_mqtt_config_t) {
        .host = "m2.tuyacn.com",
        .port = 8883,
        .cacert = tuya_cacert_pem,
        .cacert_len = sizeof(tuya_cacert_pem),
        .device_id = deviceId,
        .device_secret = deviceSecret,
        .keepalive = 60,
        .timeout_ms = 2000,
        .on_connected = on_connected,
        .on_disconnect = on_disconnect,
        .on_messages = on_messages
    });
    assert(ret == OPRT_OK);
    ret = tuya_mqtt_connect(client);
    assert(ret == OPRT_OK);
    for (;;) 
      {
      /* Loop to receive packets, and handles client keepalive */
      tuya_mqtt_loop(client);
      }
    return ret;
}
       
   