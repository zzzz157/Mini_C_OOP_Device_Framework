#ifndef __I2C_OOP_H
#define __I2C_OOP_H
#include <stdint.h>

struct I2C_Device;
typedef struct struct_I2C_Device I2C_Device;

//抽象类
typedef struct struct_I2C_Device
{
	//虚函数表
	void (*Init)(I2C_Device* self);
	uint8_t (*WriteReg)(I2C_Device* self,uint8_t SlaveAddr,uint8_t RegAddr,uint8_t Data);
	uint16_t (*ReadReg)(I2C_Device* self,uint8_t SlaveAddr,uint8_t RegAddr);
	uint8_t (*ReadRegs)(I2C_Device* self,uint8_t SlaveAddr,uint8_t RegAddr
		,uint8_t ReceiveData[],uint16_t Size);
	//属性
	char *Name;
	//行为
	void* config;
}I2C_Device;

extern I2C_Device* OLED_I2C;
extern I2C_Device* MAX_I2C;

#endif
