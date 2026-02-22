#ifndef __I2C_OOP_H
#define __I2C_OOP_H
#include <stdint.h>

struct I2C_Device;
typedef struct struct_I2C_Device I2C_Device;
//虚函数表
typedef struct 
{
	void (*Init)(I2C_Device* self);
	uint8_t (*WriteReg)(I2C_Device* self,uint8_t SlaveAddr,uint8_t RegAddr,uint8_t Data);
	uint16_t (*ReadReg)(I2C_Device* self,uint8_t SlaveAddr,uint8_t RegAddr);
	uint8_t (*ReadRegs)(I2C_Device* self,uint8_t SlaveAddr,uint8_t RegAddr
		,uint8_t ReceiveData[],uint16_t Size);
}VirtualTable;
//抽象类
typedef struct struct_I2C_Device
{
	const VirtualTable* vTable;
	char *Name;
	void* config;
}I2C_Device;

I2C_Device* get_i2c_device(const char* name);

#endif