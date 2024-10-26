/*
 * i2c_master.cpp
 *
 * Created: 2024/09/22 0:04:41
 *  Author: demicchi
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include "i2c_master.h"

volatile uint8_t i2c_timeout_count;

void i2cMasterInit()
{
    /*
    ** BAUD Value Calculation **
      f_SCL = f_CLK / (10 + 2*BAUD + f_CLK * T_RISE), where;
        f_SCL <= 100kHz in Standard mode
        T_RISE <= 1000ns in Standard mode,
    which can be converted as:
      BAUD = ((f_CLK / f_SCL) - 10 - f_CLK * T_RISE) / 2
           > ((f_CLK / 100k) - 10) / 2
    Now we can decide f_CLK from F_CPU, so the appropriate BAUD can be calculated as:
      (F_CPU / 100000 - 10) / 2
    */
    TWI0.MBAUD = ((F_CPU / 100000UL) - 10UL) / 2UL;
    TWI0.CTRLA = TWI_SDASETUP_8CYC_gc | TWI_SDAHOLD_500NS_gc;
    TWI0.MCTRLA |= TWI_ENABLE_bm;
    TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;
}


void i2cMasterStartWrite(unsigned char slave_addr)
{
    //TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;
    slave_addr = (slave_addr << 1) | 0; // Write
    TWI0.MADDR = slave_addr;
    while(!(TWI0.MSTATUS & TWI_WIF_bm));
}

bool i2cMasterCheckSlaveExistance(unsigned char slave_addr)
{
    slave_addr = (slave_addr << 1) | 0; // Write
    TWI0.MADDR = slave_addr;
    _delay_ms(100);
    
    if (!(TWI0.MSTATUS & TWI_WIF_bm)) {
        i2cMasterStop();
        return false;
    }
    if (TWI0.MSTATUS & TWI_ARBLOST_bm) {
        i2cMasterStop();
        return false;
    }        
    if (TWI0.MSTATUS & TWI_BUSERR_bm) {
        i2cMasterStop();
        return false;
    }
    if (!(TWI0.MSTATUS & TWI_RXACK_bm)) {
        // ACK = 0, NACK = 1
        i2cMasterStop();
        return true;
    }
    i2cMasterStop();
    return false;
}


void i2cMasterStartRead(unsigned char slave_addr)
{
    //TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;
    slave_addr = (slave_addr << 1) | 1; // Read
    TWI0.MADDR = slave_addr;
    while(!(TWI0.MSTATUS & TWI_RIF_bm));
}

void i2cMasterWrite(unsigned char *msg, unsigned char msg_size)
{
    while (msg_size--) {
        TWI0.MDATA = *(msg++);
        while(!(TWI0.MSTATUS & TWI_WIF_bm));
    }
}

unsigned char i2cMasterRead(bool ack)
{
    while(!(TWI0.MSTATUS & TWI_RIF_bm));
    if (ack) {
        TWI0.MCTRLB = TWI_ACKACT_ACK_gc;
    } else {
        TWI0.MCTRLB = TWI_ACKACT_NACK_gc;
    }
    unsigned char buf = TWI0.MDATA;
    TWI0.MCTRLB |= TWI_MCMD_RECVTRANS_gc;
    return buf;
}

void i2cMasterStop()
{
    TWI0.MCTRLB |= TWI_MCMD_STOP_gc;
    _delay_ms(100);
    TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;
}






