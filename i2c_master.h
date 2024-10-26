/*
 * i2c_master.h
 *
 * Created: 2024/09/22 0:04:28
 *  Author: demicchi
 */ 


#ifndef I2C_MASTER_H_
#define I2C_MASTER_H_

void i2cMasterInit();
void i2cMasterStartWrite(unsigned char);
bool i2cMasterCheckSlaveExistance(unsigned char);
void i2cMasterStartRead(unsigned char);
void i2cMasterWrite(unsigned char *, unsigned char);
unsigned char i2cMasterRead(bool);
void i2cMasterStop();

#endif /* I2C_MASTER_H_ */
