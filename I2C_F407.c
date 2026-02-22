#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"                  // Device header
#include "I2C_OOP.h"
#include <string.h>
#include "FreeRTOS.h"
#include "Task.h"
#include "semphr.h"
#define I2C_TIMEOUT_MAX 0x2000
#define SAFE_I2C_WAIT_EVENT(I2Cx, Event) \
    do { \
        uint32_t wait_time = I2C_TIMEOUT_MAX; \
        while(!I2C_CheckEvent(I2Cx, Event)) { \
            if((wait_time--) == 0) { \
                HardI2C_Force_Recovery(cfg); \
                return 1; /* 1 代表失败 */ \
            } \
        } \
    } while(0)
#define MAX_DMA_I2C_INSTANCES 3
static I2C_Device* s_pI2C_DMA_Registry[MAX_DMA_I2C_INSTANCES] = {NULL};

							//函数声明
//SoftI2C
static void Soft_I2C_Init(I2C_Device* self);
static uint8_t SoftI2C_WriteReg(I2C_Device* self,uint8_t SlaveAddr,uint8_t RegAddr,uint8_t Data);
static uint16_t SoftI2C_ReadReg(I2C_Device* self,uint8_t SlaveAddr,uint8_t RegAddr);
static uint8_t SoftI2C_ReadRegs(I2C_Device* self,uint8_t SlaveAddr,uint8_t RegAddr,
	uint8_t ReceiveData[],uint16_t Size);
//HardI2C
static void Hard_I2C_Init(I2C_Device* self);
static uint8_t HardI2C_WriteReg(I2C_Device* self,uint8_t SlaveAddr,uint8_t RegAddr,uint8_t Data);
static uint16_t HardI2C_ReadReg(I2C_Device* self,uint8_t SlaveAddr,uint8_t RegAddr);
static uint8_t HardI2C_ReadRegs(I2C_Device* self,uint8_t SlaveAddr,uint8_t RegAddr,
	uint8_t ReceiveData[],uint16_t Size);
//HardI2C_DMA
static void HardI2C_DMA_Init(I2C_Device* self);
static uint8_t HardI2C_DMA_ReadRegs(I2C_Device* self,uint8_t SlaveAddr,uint8_t RegAddr,
	uint8_t ReceiveData[],uint16_t Size);

typedef struct
{
	uint32_t RCC_AHB1Periph_DMAx;  //DMA时钟(如 RCC_AHB1Periph_DMA1)
	DMA_Stream_TypeDef* Rx_Stream; // 接收数据流 (如F407的DMA1_Stream0)
    uint32_t Rx_Channel;           // 接收通道(如DMA_Channel_1)
    DMA_Stream_TypeDef* Tx_Stream; // 发送数据流(DMA1_Stream6)
    uint32_t Tx_Channel;           // 发送通道
	uint8_t Rx_IRQn;			   //中断号
	SemaphoreHandle_t DMA_RxSemaphoreHandle_t; //接收完成通知
}DMA_config;
typedef struct
{
	I2C_TypeDef* I2Cxx;				//I2C1,I2C2,I2C3
	uint32_t RCC_APB1Periph_I2Cx;	//RCC_APB1Periph_I2C1,I2C2,I2C3
	uint32_t ClockSpeed;			//I2C速度
	uint16_t Mode;					//模式
	uint16_t DutyCycle;				//占空比
	uint16_t OwnAddress1;			//主机地址
	uint16_t Ack;					//是否使能应答
	uint16_t AcknowledgedAddress;//从机地址模式：7位？11位？
	uint8_t  GPIO_AF_I2C;			//I2C引脚复用
	uint16_t GPIO_PinSourcex_SCL;	//SCK复用引脚 如GPIO_PinSource6
	uint16_t GPIO_PinSourcex_SDA;	//SDA复用引脚 如GPIO_PinSource7
	DMA_config* HardI2C_DMA_config; //硬件I2C的DMA
}HardwareI2C_config;
typedef struct
{
	//GPIO
	uint32_t RCC_GPIO;          	//GPIO 时钟 如 RCC_APB2Periph_GPIOA
    GPIO_TypeDef* GPIO_Port;      	//GPIO 端口(如 GPIOA)
    uint16_t Pin_SCL;            	//SCL引脚
    uint16_t Pin_SDA;            	//SDA引脚
	uint32_t Soft_Delay_Count; 		//软件模拟 延时
	HardwareI2C_config* Hardware_I2C;
	void* Extersion;
}I2C_config;

//stm32F4_SoftI2C1
static I2C_config stm32F4_SoftI2C_cfg1={
	.RCC_GPIO=RCC_AHB1Periph_GPIOE,
	.GPIO_Port=GPIOE,
	.Pin_SCL=GPIO_Pin_2,
	.Pin_SDA=GPIO_Pin_3,
	.Soft_Delay_Count=500,
	.Hardware_I2C=NULL,
};
static const VirtualTable stm32F4_SoftI2C_vTable1={
	.Init=Soft_I2C_Init,
	.WriteReg=SoftI2C_WriteReg,
	.ReadReg=SoftI2C_ReadReg,
	.ReadRegs=SoftI2C_ReadRegs,
};
static I2C_Device stm32F4_SoftI2C1={
	.vTable=&stm32F4_SoftI2C_vTable1,
	.Name="stm32F4_SoftI2C1",
	.config=&stm32F4_SoftI2C_cfg1,
};
//stm32F4_SoftI2C2
static I2C_config stm32F4_SoftI2C_cfg2={
	.RCC_GPIO=RCC_AHB1Periph_GPIOE,
	.GPIO_Port=GPIOE,
	.Pin_SCL=GPIO_Pin_4,
	.Pin_SDA=GPIO_Pin_5,
	.Soft_Delay_Count=100,
	.Hardware_I2C=NULL,
};
static const VirtualTable stm32F4_SoftI2C_vTable2={
	.Init=Soft_I2C_Init,
	.WriteReg=SoftI2C_WriteReg,
	.ReadReg=SoftI2C_ReadReg,
	.ReadRegs=SoftI2C_ReadRegs,
};
static I2C_Device stm32F4_SoftI2C2={
	.vTable=&stm32F4_SoftI2C_vTable2,
	.Name="stm32F4_SoftI2C2",
	.config=&stm32F4_SoftI2C_cfg2,
};
//stm32F4_HardI2C1
static HardwareI2C_config HardwareI2C={
	.I2Cxx=I2C1,
	.RCC_APB1Periph_I2Cx=RCC_APB1Periph_I2C1,
	.ClockSpeed=100000,
	.Mode=I2C_Mode_I2C,
	.DutyCycle=I2C_DutyCycle_2,
	.OwnAddress1=0x00,
	.Ack=I2C_Ack_Enable,
	.AcknowledgedAddress=I2C_AcknowledgedAddress_7bit,
	.GPIO_AF_I2C=GPIO_AF_I2C1,
	.GPIO_PinSourcex_SCL=GPIO_PinSource6,
	.GPIO_PinSourcex_SDA=GPIO_PinSource7,
};
static I2C_config stm32F4_HardI2C_cfg={
	.RCC_GPIO=RCC_AHB1Periph_GPIOB,
	.GPIO_Port=GPIOB,
	.Pin_SCL=GPIO_Pin_6,
	.Pin_SDA=GPIO_Pin_7,
	.Hardware_I2C=&HardwareI2C,
};
static const VirtualTable stm32F4_HardI2C_vTable1={
	.Init=Hard_I2C_Init,
	.WriteReg=HardI2C_WriteReg,
	.ReadReg=HardI2C_ReadReg,
	.ReadRegs=HardI2C_ReadRegs,
};
static I2C_Device stm32F4_HardI2C1={
	.vTable=&stm32F4_HardI2C_vTable1,
	.Name="stm32F4_HardI2C1",
	.config=&stm32F4_HardI2C_cfg,
};
//stm32F4_HardI2C1_DMA
static DMA_config stm32F4_HardI2C1_DMA_config={
	.RCC_AHB1Periph_DMAx=RCC_AHB1Periph_DMA1,
	.Rx_Stream=DMA1_Stream0,
	.Rx_Channel=DMA_Channel_1,
	.Tx_Stream=DMA1_Stream6,
	.Tx_Channel=DMA_Channel_1,
	.Rx_IRQn=DMA1_Stream0_IRQn,
	.DMA_RxSemaphoreHandle_t=NULL,
};
static HardwareI2C_config HardwareI2C1_DMA={
	.I2Cxx=I2C1,
	.RCC_APB1Periph_I2Cx=RCC_APB1Periph_I2C1,
	.ClockSpeed=100000,
	.Mode=I2C_Mode_I2C,
	.DutyCycle=I2C_DutyCycle_2,
	.OwnAddress1=0x00,
	.Ack=I2C_Ack_Enable,
	.AcknowledgedAddress=I2C_AcknowledgedAddress_7bit,
	.GPIO_AF_I2C=GPIO_AF_I2C1,
	.GPIO_PinSourcex_SCL=GPIO_PinSource6,
	.GPIO_PinSourcex_SDA=GPIO_PinSource7,
	.HardI2C_DMA_config=&stm32F4_HardI2C1_DMA_config,
};
static I2C_config stm32F4_HardI2C1_DMA_cfg={
	.RCC_GPIO=RCC_AHB1Periph_GPIOB,
	.GPIO_Port=GPIOB,
	.Pin_SCL=GPIO_Pin_6,
	.Pin_SDA=GPIO_Pin_7,
	.Hardware_I2C=&HardwareI2C1_DMA,
};
static const VirtualTable stm32F4_HardI2C1_DMA_vTable1={
	.Init=HardI2C_DMA_Init,
	.WriteReg=HardI2C_WriteReg,
	.ReadReg=HardI2C_ReadReg,
	.ReadRegs=HardI2C_DMA_ReadRegs,
};
static I2C_Device stm32F4_HardI2C1_DMA={
	.vTable=&stm32F4_HardI2C1_DMA_vTable1,
	.Name="stm32F4_HardI2C1_DMA",
	.config=&stm32F4_HardI2C1_DMA_cfg,
};
						//====Software-emulated I2C====
static void I2C_Delay(I2C_Device* self)
{
    I2C_config* cfg = self->config;
    uint32_t i = cfg->Soft_Delay_Count; 
    while(i--) { __NOP(); }
}
static void SoftI2C_Write_SCL(I2C_Device* self,uint8_t Bit)
{
	I2C_config* cfg=self->config;
	GPIO_WriteBit(cfg->GPIO_Port,cfg->Pin_SCL, (BitAction)Bit);
	
}

static void SoftI2C_Write_SDA(I2C_Device* self,uint8_t Bit)
{
	I2C_config* cfg=self->config;
	GPIO_WriteBit(cfg->GPIO_Port,cfg->Pin_SDA, (BitAction)Bit);
}

static uint8_t SoftI2C_Read_SDA(I2C_Device* self)
{
	I2C_config* cfg=self->config;
	return	GPIO_ReadInputDataBit(cfg->GPIO_Port,cfg->Pin_SDA);
}
//
static void SoftI2C_Start(I2C_Device* self)
{
	SoftI2C_Write_SDA(self,1);
	I2C_Delay(self);
	SoftI2C_Write_SCL(self,1);
	I2C_Delay(self);
	SoftI2C_Write_SDA(self,0);
	I2C_Delay(self);
	SoftI2C_Write_SCL(self,0);
	I2C_Delay(self);
}

static void SoftI2C_Stop(I2C_Device* self)
{
	SoftI2C_Write_SDA(self,0);
	I2C_Delay(self);
	SoftI2C_Write_SCL(self,1);
	I2C_Delay(self);
	SoftI2C_Write_SDA(self,1);
	I2C_Delay(self);
}

static void SoftI2C_SendByte(I2C_Device* self,uint8_t Byte)
{
	uint8_t i;
	for(i=0;i<8;i++)
	{
		SoftI2C_Write_SDA(self,!!((0x80>>i) & Byte));
		I2C_Delay(self);
		SoftI2C_Write_SCL(self,1);
		I2C_Delay(self);
		SoftI2C_Write_SCL(self,0);
		I2C_Delay(self);
	}
}

static uint8_t SoftI2C_ReceiveByte(I2C_Device* self)
{
	uint8_t i,Byte=0x00;
	SoftI2C_Write_SDA(self,1);
	I2C_Delay(self);
	for(i=0;i<8;i++)
	{
		SoftI2C_Write_SCL(self,1);
		I2C_Delay(self);
		if( SoftI2C_Read_SDA(self) )
		{
			Byte|=(0x80>>i);
		}
	    SoftI2C_Write_SCL(self,0);
		I2C_Delay(self);
	}
	return Byte;
}

static void SoftI2C_SendAck(I2C_Device* self,uint8_t Ackbit)
{
	SoftI2C_Write_SDA(self,Ackbit);
	I2C_Delay(self);
	SoftI2C_Write_SCL(self,1);
	I2C_Delay(self);
	SoftI2C_Write_SCL(self,0);
	I2C_Delay(self);
}

static uint8_t SoftI2C_ReceiveAck(I2C_Device* self)
{
	uint8_t Ackbit=1;
	SoftI2C_Write_SDA(self,1);
	I2C_Delay(self);
	SoftI2C_Write_SCL(self,1);
	I2C_Delay(self);
	Ackbit=SoftI2C_Read_SDA(self);
	SoftI2C_Write_SCL(self,0);
	I2C_Delay(self);
	return Ackbit;
}
							//======Software_I2C=====
static void Soft_I2C_Init(I2C_Device* self)
{
	I2C_config* cfg=self->config;
	RCC_AHB1PeriphClockCmd(cfg->RCC_GPIO, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin=cfg->Pin_SCL|cfg->Pin_SDA;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(cfg->GPIO_Port, &GPIO_InitStructure);
}
static uint8_t SoftI2C_WriteReg(I2C_Device* self,uint8_t SlaveAddr,uint8_t RegAddr,uint8_t Data)
{
	SoftI2C_Start(self);
	SoftI2C_SendByte(self,SlaveAddr<<1|0);
	if(SoftI2C_ReceiveAck(self)){ SoftI2C_Stop(self); return 1;}
	SoftI2C_SendByte(self,RegAddr);
	if(SoftI2C_ReceiveAck(self)){ SoftI2C_Stop(self); return 1;}
	SoftI2C_SendByte(self,Data);
	if(SoftI2C_ReceiveAck(self)){ SoftI2C_Stop(self); return 1;}
	SoftI2C_Stop(self);
	return 0;
}
static uint16_t SoftI2C_ReadReg(I2C_Device* self,uint8_t SlaveAddr,uint8_t RegAddr)
{
	uint8_t ReceiveData;
	SoftI2C_Start(self);
	SoftI2C_SendByte(self,SlaveAddr<<1|0);
	if(SoftI2C_ReceiveAck(self)){ SoftI2C_Stop(self); return 0xFFFF;}
	SoftI2C_SendByte(self,RegAddr);
	if(SoftI2C_ReceiveAck(self)){ SoftI2C_Stop(self); return 0xFFFF;}
	
	SoftI2C_Start(self);
	SoftI2C_SendByte(self,SlaveAddr<<1|1);
	if(SoftI2C_ReceiveAck(self)){ SoftI2C_Stop(self); return 0xFFFF;}
	ReceiveData=SoftI2C_ReceiveByte(self);
	SoftI2C_SendAck(self,1);
	SoftI2C_Stop(self);
	return ReceiveData;
}
static uint8_t SoftI2C_ReadRegs(I2C_Device* self,uint8_t SlaveAddr,uint8_t RegAddr,
	uint8_t ReceiveData[],uint16_t Size)
{
	SoftI2C_Start(self);
	SoftI2C_SendByte(self,SlaveAddr<<1|0);
	if(SoftI2C_ReceiveAck(self)){ SoftI2C_Stop(self); return 1;}
	SoftI2C_SendByte(self,RegAddr);
	if(SoftI2C_ReceiveAck(self)){ SoftI2C_Stop(self); return 1;}
	
	SoftI2C_Start(self);
	SoftI2C_SendByte(self,SlaveAddr<<1|1);
	if(SoftI2C_ReceiveAck(self)){ SoftI2C_Stop(self); return 1;}
	for(uint8_t i=0;i<Size;i++)
	{
		ReceiveData[i]=SoftI2C_ReceiveByte(self);
		if(i<Size-1)
		{
			SoftI2C_SendAck(self,0);
		}
	}
	SoftI2C_SendAck(self,1);
	SoftI2C_Stop(self);
	return 0;
}
							//========Hardware_I2C=====
static void Hard_I2C_Init(I2C_Device* self)
{
	I2C_config* cfg=self->config;
	RCC_AHB1PeriphClockCmd(cfg->RCC_GPIO, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin=cfg->Pin_SCL|cfg->Pin_SDA;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(cfg->GPIO_Port, &GPIO_InitStructure);
	
	RCC_APB1PeriphClockCmd(cfg->Hardware_I2C->RCC_APB1Periph_I2Cx, ENABLE);
	GPIO_PinAFConfig(cfg->GPIO_Port,cfg->Hardware_I2C->GPIO_PinSourcex_SCL,
		cfg->Hardware_I2C->GPIO_AF_I2C);//SCL复用
	GPIO_PinAFConfig(cfg->GPIO_Port,cfg->Hardware_I2C->GPIO_PinSourcex_SDA,
		cfg->Hardware_I2C->GPIO_AF_I2C); //SDA复用
	RCC_APB1PeriphResetCmd(cfg->Hardware_I2C->RCC_APB1Periph_I2Cx, ENABLE);
	RCC_APB1PeriphResetCmd(cfg->Hardware_I2C->RCC_APB1Periph_I2Cx, DISABLE);
	I2C_InitTypeDef I2C_InitStruct;
	I2C_InitStruct.I2C_ClockSpeed=cfg->Hardware_I2C->ClockSpeed;
	I2C_InitStruct.I2C_Mode = cfg->Hardware_I2C->Mode;
	I2C_InitStruct.I2C_DutyCycle =cfg->Hardware_I2C->DutyCycle;
	I2C_InitStruct.I2C_OwnAddress1 = cfg->Hardware_I2C->OwnAddress1;
	I2C_InitStruct.I2C_Ack = cfg->Hardware_I2C->Ack;
	I2C_InitStruct.I2C_AcknowledgedAddress = cfg->Hardware_I2C->AcknowledgedAddress;
	I2C_Init(cfg->Hardware_I2C->I2Cxx, &I2C_InitStruct);
	I2C_Cmd(cfg->Hardware_I2C->I2Cxx, ENABLE);
}
static void HardI2C_Force_Recovery(I2C_config* cfg)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = cfg->Pin_SCL | cfg->Pin_SDA;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(cfg->GPIO_Port, &GPIO_InitStructure);
    for (int i = 0; i < 9; i++) 
	{
        GPIO_ResetBits(cfg->GPIO_Port, cfg->Pin_SCL);
        for(int j=0; j<100; j++) __NOP();
        GPIO_SetBits(cfg->GPIO_Port, cfg->Pin_SCL);
        for(int j=0; j<100; j++) __NOP();
    }
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(cfg->GPIO_Port, &GPIO_InitStructure);
}
static uint8_t HardI2C_WriteReg(I2C_Device* self,uint8_t SlaveAddr,uint8_t RegAddr,uint8_t Data)
{
	I2C_config* cfg = self->config;
    I2C_TypeDef* I2Cx = cfg->Hardware_I2C->I2Cxx;
	uint32_t timeout = I2C_TIMEOUT_MAX;
    while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY)){
        if((timeout--) == 0) { HardI2C_Force_Recovery(cfg); return 1; }
    }

    I2C_GenerateSTART(I2Cx, ENABLE);
    SAFE_I2C_WAIT_EVENT(I2Cx, I2C_EVENT_MASTER_MODE_SELECT);

    I2C_Send7bitAddress(I2Cx, SlaveAddr << 1, I2C_Direction_Transmitter);
    SAFE_I2C_WAIT_EVENT(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);

    I2C_SendData(I2Cx, RegAddr);
    SAFE_I2C_WAIT_EVENT(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED);

    I2C_SendData(I2Cx, Data);
    SAFE_I2C_WAIT_EVENT(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED);

    I2C_GenerateSTOP(I2Cx, ENABLE);
	return 0;
}
static uint16_t HardI2C_ReadReg(I2C_Device* self,uint8_t SlaveAddr,uint8_t RegAddr)
{
	uint8_t data;
	uint32_t timeout = I2C_TIMEOUT_MAX;
    I2C_config* cfg = self->config;
    I2C_TypeDef* I2Cx = cfg->Hardware_I2C->I2Cxx;
	
    while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY)){
        if((timeout--) == 0) { HardI2C_Force_Recovery(cfg); return 0xFFFF; }
    }
    I2C_GenerateSTART(I2Cx, ENABLE);
    SAFE_I2C_WAIT_EVENT(I2Cx, I2C_EVENT_MASTER_MODE_SELECT);

    I2C_Send7bitAddress(I2Cx, SlaveAddr << 1, I2C_Direction_Transmitter);
    SAFE_I2C_WAIT_EVENT(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);

    I2C_SendData(I2Cx, RegAddr);
    SAFE_I2C_WAIT_EVENT(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
	
    I2C_GenerateSTART(I2Cx, ENABLE);
    SAFE_I2C_WAIT_EVENT(I2Cx, I2C_EVENT_MASTER_MODE_SELECT);

    I2C_Send7bitAddress(I2Cx, SlaveAddr << 1, I2C_Direction_Receiver);
    SAFE_I2C_WAIT_EVENT(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);

    I2C_AcknowledgeConfig(I2Cx, DISABLE);
    I2C_GenerateSTOP(I2Cx, ENABLE);

    SAFE_I2C_WAIT_EVENT(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED);
    data = I2C_ReceiveData(I2Cx);

    I2C_AcknowledgeConfig(I2Cx, ENABLE);

    return data;
}
static uint8_t HardI2C_ReadRegs(I2C_Device* self,uint8_t SlaveAddr,uint8_t RegAddr,
	uint8_t ReceiveData[],uint16_t Size)
{
	I2C_config* cfg = self->config;
    I2C_TypeDef* I2Cx = cfg->Hardware_I2C->I2Cxx;

    if (Size == 0) return 1;
	uint32_t timeout = I2C_TIMEOUT_MAX;
    while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY)){
        if((timeout--) == 0) { HardI2C_Force_Recovery(cfg); return 1; }
    }
    I2C_GenerateSTART(I2Cx, ENABLE);
    SAFE_I2C_WAIT_EVENT(I2Cx, I2C_EVENT_MASTER_MODE_SELECT);

    I2C_Send7bitAddress(I2Cx, SlaveAddr << 1, I2C_Direction_Transmitter);
    SAFE_I2C_WAIT_EVENT(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);

    I2C_SendData(I2Cx, RegAddr);
    SAFE_I2C_WAIT_EVENT(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED);

    I2C_GenerateSTART(I2Cx, ENABLE);
    SAFE_I2C_WAIT_EVENT(I2Cx, I2C_EVENT_MASTER_MODE_SELECT);

    I2C_Send7bitAddress(I2Cx, SlaveAddr << 1, I2C_Direction_Receiver);
    SAFE_I2C_WAIT_EVENT(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);

    for (uint16_t i = 0; i < Size; i++)
    {
        if (i == Size - 1)
        {
            I2C_AcknowledgeConfig(I2Cx, DISABLE);
            I2C_GenerateSTOP(I2Cx, ENABLE);
        }
        SAFE_I2C_WAIT_EVENT(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED);
        ReceiveData[i] = I2C_ReceiveData(I2Cx);
    }
    I2C_AcknowledgeConfig(I2Cx, ENABLE);
	return 0;
}
							//======HardI2C1_DMA=========
static void HardI2C_DMA_Init(I2C_Device* self)
{
	Hard_I2C_Init(self);
	
	I2C_config* i2c_dma_cfg=self->config;
	DMA_config* dma_cfg=i2c_dma_cfg->Hardware_I2C->HardI2C_DMA_config;
	
	dma_cfg->DMA_RxSemaphoreHandle_t=xSemaphoreCreateBinary();
	
	if (dma_cfg->Rx_Stream == DMA1_Stream0) {
        s_pI2C_DMA_Registry[0] = self;  // I2C1_RX
    } 
    else if (dma_cfg->Rx_Stream == DMA1_Stream3) {
        s_pI2C_DMA_Registry[1] = self;  // 预留给 I2C2_RX (假设用 Stream3)
    }
    else if (dma_cfg->Rx_Stream == DMA1_Stream2) {
        s_pI2C_DMA_Registry[2] = self;  // 预留给 I2C3_RX (假设用 Stream2)
    }
	
	RCC_AHB1PeriphClockCmd(dma_cfg->RCC_AHB1Periph_DMAx, ENABLE); 
	NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = dma_cfg->Rx_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 12; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
static uint8_t HardI2C_DMA_ReadRegs(I2C_Device* self,uint8_t SlaveAddr,uint8_t RegAddr,
	uint8_t ReceiveData[],uint16_t Size)
{
	I2C_config* cfg=self->config;
	I2C_TypeDef* I2Cx = cfg->Hardware_I2C->I2Cxx;
	DMA_config* dma_cfg=cfg->Hardware_I2C->HardI2C_DMA_config;
	
	DMA_InitTypeDef DMA_InitStructure;
    DMA_InitStructure.DMA_Channel = dma_cfg->Rx_Channel;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&I2Cx->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)ReceiveData; 
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = Size;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(dma_cfg->Rx_Stream, &DMA_InitStructure);
    DMA_ITConfig(dma_cfg->Rx_Stream, DMA_IT_TC, ENABLE);
	
	uint32_t timeout = I2C_TIMEOUT_MAX;
    while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY)){
        if((timeout--) == 0) { HardI2C_Force_Recovery(cfg); return 1; }
    }
	I2C_GenerateSTART(I2Cx,ENABLE);
	SAFE_I2C_WAIT_EVENT(I2Cx, I2C_EVENT_MASTER_MODE_SELECT);
	I2C_Send7bitAddress(I2Cx, SlaveAddr << 1, I2C_Direction_Transmitter);
    SAFE_I2C_WAIT_EVENT(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
	
    I2C_SendData(I2Cx, RegAddr);
    SAFE_I2C_WAIT_EVENT(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
	I2C_GenerateSTART(I2Cx,ENABLE);
	SAFE_I2C_WAIT_EVENT(I2Cx, I2C_EVENT_MASTER_MODE_SELECT);
	I2C_Send7bitAddress(I2Cx, SlaveAddr << 1, I2C_Direction_Receiver);
    while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_ADDR) == RESET);
	
	I2C_DMALastTransferCmd(I2Cx, ENABLE);
    I2C_DMACmd(I2Cx, ENABLE);
    DMA_Cmd(dma_cfg->Rx_Stream, ENABLE);
	(void)I2Cx->SR1;
    (void)I2Cx->SR2;
	xSemaphoreTake(dma_cfg->DMA_RxSemaphoreHandle_t,portMAX_DELAY);
	
    I2C_GenerateSTOP(I2Cx, ENABLE);
    I2C_DMACmd(I2Cx, DISABLE);
    DMA_Cmd(dma_cfg->Rx_Stream, DISABLE);
	return 0;
}
							//=========Interrupt_deal========
void DMA1_Stream0_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_Stream0, DMA_IT_TCIF0) != RESET)
    {
		DMA_ClearITPendingBit(DMA1_Stream0, DMA_IT_TCIF0);
		I2C_Device* pDevice = s_pI2C_DMA_Registry[0];
		if(pDevice!=NULL)
		{
			I2C_config* i2c_cfg=(I2C_config*)pDevice->config;
			DMA_config* dma_cfg=i2c_cfg->Hardware_I2C->HardI2C_DMA_config;
			DMA_ITConfig(dma_cfg->Rx_Stream, DMA_IT_TC, DISABLE);
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			xSemaphoreGiveFromISR(dma_cfg->DMA_RxSemaphoreHandle_t,&xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
		
    }
}
void DMA1_Stream2_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_Stream2, DMA_IT_TCIF2) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);
        I2C_Device* pDevice = s_pI2C_DMA_Registry[2]; 
        if (pDevice != NULL)
        {
            I2C_config* i2c_cfg=(I2C_config*)pDevice->config;
			DMA_config* dma_cfg=i2c_cfg->Hardware_I2C->HardI2C_DMA_config;
        }
    }
}
							//======Device_management====
static I2C_Device* const devices[] = {
	&stm32F4_SoftI2C1,
	&stm32F4_SoftI2C2,
	&stm32F4_HardI2C1,
	&stm32F4_HardI2C1_DMA,
};
I2C_Device* get_i2c_device(const char* name)
{
    for (int i = 0; i < sizeof(devices)/sizeof(devices[0]); i++)
	{
        if (strcmp(devices[i]->Name, name) == 0)
		{
            return devices[i]; 
        }
    }
    return NULL;
}