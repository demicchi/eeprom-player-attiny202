/*
 * eeprom-player-attiny202.cpp
 *
 * Created: 2024/09/15 19:20:53
 * Author : demicchi
 */ 

// 20MHz
// F_CPU should be defined as a compiler option (-DF_CPU=20000000UL)
//#define F_CPU 20000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <string.h>

//#include "lib/AVR310mod_USI_I2C_Master/USI_I2C_Master.h"
#include "i2c_master.h"

/************************************************************************
 * Configuration Begin
 ************************************************************************/

#define SOUND_TABLE_MAX_ATTEMPT 8 // how many table data to read for each EEPROM
#define SOUND_ID_MAX 8 // how many sounds to define
#define SOUND_BANK_MAX 8 // how many EEPROMs to support
#define SOUND_QUEUE_LEN 8 // how long the sounds can wait for being played
#define LONG_RUN_TIME 7200UL // how many seconds to count as a long run
#define DELAY_ACC_START 5UL // MAX EIGHT(8) SECONDS DUE TO WATCHDOG. how many seconds to wait to play sounds after ACC power becomes ON
#define DELAY_ACC_STOP 3UL // how many seconds to wait to play sounds after ACC power becomes ON
volatile uint8_t sound_bank[SOUND_BANK_MAX] = {0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57}; // EEPROM addresses for each bank_ids starting from 0
static uint8_t sound_acc_start[] = {1}; // sound_ids to play when ACC power becomes ON
static uint8_t sound_acc_stop[] = {3}; // sound_ids to play when ACC power becomes OFF
static uint8_t sound_long_run[] = {2}; // sound_ids to play when ACC power is ON for a long time

/************************************************************************
 * Configuration End
 ************************************************************************/

//volatile uint8_t wait_count_sound = 0;
//volatile uint8_t wait_count_second = 0;
volatile uint16_t running_time = 0;
volatile uint8_t sound_id = 0;
volatile bool load_next_byte = false;
volatile unsigned char sound_byte = 0x00;
volatile uint16_t bytes_loaded = 0;
volatile uint8_t sound_queue[SOUND_QUEUE_LEN];
volatile uint8_t sound_queue_pos = 0;
volatile bool acc_stop_detected = false;
volatile bool long_run_detected = false;
volatile uint8_t wait_count_acc_stop = 0;

volatile struct SoundTable {
    uint8_t bank;
    uint16_t addr;
    uint16_t len;
} sound_table[SOUND_ID_MAX + 1];

//void wdt_init() __attribute__((naked)) __attribute__((section(".init3")));

void initializeSoundQueue();
bool addToSoundQueue(uint8_t);
void addListToSoundQueue(uint8_t *, uint8_t);
uint8_t pickFromSoundQueue();

void readMemory(uint8_t, uint16_t, unsigned char *, uint8_t);

void scanSoundBank();
void readSoundTable();

void startSoundInQueue();
bool playSound(uint8_t);
void loadSoundByte();
void stopSound();

bool checkAccStop();
void checkLongRun();

void doProcedureForAccStart();
void doProcedureForAccStop();
void doProcedureForLongRun();

void shutdown();


/*
ISR(TIM0_OVF_vect) {
    // Note: the clock is 16 MHz, the counter is 8bit (0..255)
    if (++wait_count_sound >= 8) { // drop down to 8000 Hz (16 MHz / 256 / 8 = 7812.5 Hz)
        wait_count_sound = 0;
        if (sound_id == 0) {
            OCR0B = 0x00;
        } else {
            if (!load_next_byte) {
                OCR0B = sound_byte;
                load_next_byte = true;
            }
        }
    }
}
*/

ISR(TCA0_OVF_vect) {
    // Executed at sound resolution (8000Hz)
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm; // INTFLAGS must be cleared by software
    if (sound_id == 0) {
        //OCR0B = 0x00;
        TCB0.CCMPH = 0x00;
    } else {
        if (!load_next_byte) {
            //OCR0B = sound_byte;
            TCB0.CCMPH = sound_byte;
            load_next_byte = true;
        }
    }
}

/*
ISR(TIM1_OVF_vect) {
    // Note: the clock is 16 MHz, prescaled by 16384, the counter is 8bit (0..255)
    if (++wait_count_second >= 4) { // one second has passed (16 MHz / 16384 / 256 / 4 = 0.9537 Hz ~ 1s)
        wait_count_second = 0;
        if (++running_time >= LONG_RUN_TIME) {
            running_time = 0;
            long_run_detected = true;
        }
        if (wait_count_acc_stop > 0) {
            wait_count_acc_stop--;
        }
    }
}
*/

ISR(RTC_CNT_vect) {
    // Executed every one second
    RTC.INTFLAGS = RTC_OVF_bm; // INTFLAGS must be cleared by software
    if (++running_time >= LONG_RUN_TIME) {
        running_time = 0;
        long_run_detected = true;
    }
    if (wait_count_acc_stop > 0) {
        wait_count_acc_stop--;
    }
}


void initializeSoundQueue()
{
    memset((uint8_t *)sound_queue, 0x00, sizeof(sound_queue));
    sound_queue_pos = 0;
}

bool addToSoundQueue(uint8_t data)
{
    uint8_t index = sound_queue_pos;
    
    for (uint8_t i = 0; i < SOUND_QUEUE_LEN; i++) {
        if (sound_queue[index] == 0x00) {
            sound_queue[index] = data;
            return true;
        }
        if (index >= SOUND_QUEUE_LEN - 1) {
            index = 0;
        } else {
            index++;
        }
    }
    // Queue is occupied
    return false;
}

void addListToSoundQueue(uint8_t *data, uint8_t size)
{
    while (size--) {
        addToSoundQueue(*(data++));
    }
}

uint8_t pickFromSoundQueue()
{
    uint8_t picked_data = sound_queue[sound_queue_pos];
    
    if (picked_data == 0x00) {
        return picked_data;
    }
    
    sound_queue[sound_queue_pos] = 0x00;
    if (sound_queue_pos >= SOUND_QUEUE_LEN - 1) {
        sound_queue_pos = 0;
    } else {
        sound_queue_pos++;
    }
    return picked_data;
}

void readMemory(uint8_t device_addr, uint16_t memory_addr, unsigned char *buf, uint8_t len) {
    unsigned char addr[2];
    
    addr[0] = memory_addr >> 8;
    addr[1] = memory_addr & 0xff;
    
    //USI_TWI_Start_Transmission_Write(device_addr);
    i2cMasterStartWrite(device_addr);
    //USI_TWI_Write(addr, 2);
    i2cMasterWrite(addr, 2);
    
    //USI_TWI_Start_Transmission_Read(device_addr, buf);
    i2cMasterStartRead(device_addr);
    
    /*
    while (--len > 0) {
        *(++buf) = USI_TWI_Read_Next_One_Byte();
    }
    */
    //USI_TWI_Read_Finish();
    while (len-- > 1) {
        *(buf++) = i2cMasterRead(true);
    }
    *(buf) = i2cMasterRead(false);
    
    //USI_TWI_Master_Stop();
    i2cMasterStop();
}


void scanSoundBank()
{
    uint8_t index = 0;
    
    do {
        if (sound_bank[index] == 0x00) {
            continue;
        }
        //if (!USI_TWI_Start_Transmission_Write(sound_bank[index])) {
        if (!i2cMasterCheckSlaveExistance(sound_bank[index])) {
            // EEPROM not found
            sound_bank[index] = 0x00;
        }
        //USI_TWI_Master_Stop();
    } while (++index < SOUND_BANK_MAX);
}

void readSoundTable() {
    unsigned char buf[3];
    uint8_t index;
    uint8_t bank_id = 0;
    uint8_t id;
    
    for (index = 0; index <= SOUND_ID_MAX; index++) {
        sound_table[index].bank = 0x00;
        sound_table[index].addr = 0x00;
        sound_table[index].len = 0x00;
    }
    
    do {
        if (sound_bank[bank_id] == 0x00) {
            continue;
        }
        index = 0;
        do {
            memset(buf, 0, sizeof(buf));
            readMemory(sound_bank[bank_id], 8 * index, buf, 2);
            if (buf[0] == 0x00 && buf[1] == 0x00) {
                break;
            }
            if (buf[0] == 0x00 && buf[1] >= 1 && buf[1] <= SOUND_ID_MAX) {
                id = buf[1];
                sound_table[id].bank = bank_id;
                memset(buf, 0, sizeof(buf));
                readMemory(sound_bank[bank_id], 8 * index + 2, buf, 3);
                sound_table[id].addr = (buf[1] << 8) | (buf[2]);
                memset(buf, 0, sizeof(buf));
                readMemory(sound_bank[bank_id], 8 * index + 5, buf, 3);
                sound_table[id].len = (buf[1] << 8) | (buf[2]);
                break;
            }
        } while (++index < SOUND_TABLE_MAX_ATTEMPT);
    } while (++bank_id < SOUND_BANK_MAX);
}

void startSoundInQueue()
{
    uint8_t id = pickFromSoundQueue();
    if (id == 0x00) {
        return;
    }
    playSound(id);
}

bool playSound(uint8_t id)
{
    unsigned char addr[2];
    
    stopSound();
    if (id == 0 || id > SOUND_ID_MAX || sound_table[id].len == 0) {
        return false;
    }

    addr[0] = sound_table[id].addr >> 8;
    addr[1] = sound_table[id].addr & 0xff;
    //USI_TWI_Start_Transmission_Write(sound_bank[sound_table[id].bank]);
    i2cMasterStartWrite(sound_bank[sound_table[id].bank]);
    //USI_TWI_Write(addr, 2);
    i2cMasterWrite(addr, 2);
    //USI_TWI_Start_Transmission_Read(sound_bank[sound_table[id].bank], (unsigned char*)&sound_byte);
    i2cMasterStartRead(sound_bank[sound_table[id].bank]);
    sound_byte = i2cMasterRead(true);
    
    load_next_byte = false;
    sound_id = id;
    
    return true;
}

void loadSoundByte()
{
    if (!load_next_byte || sound_id == 0) {
        return;
    }
    if (bytes_loaded >= sound_table[sound_id].len) {
        stopSound();
        startSoundInQueue();
    } else {
        //sound_byte = USI_TWI_Read_Next_One_Byte();
        sound_byte = i2cMasterRead(true);
        bytes_loaded++;
        load_next_byte = false;
    }
}

void stopSound()
{
    if (sound_id == 0) {
        return;
    }
    //USI_TWI_Read_Finish();
    i2cMasterRead(false);
    //USI_TWI_Master_Stop();
    bytes_loaded = 0;
    sound_id = 0;
    i2cMasterStop();
}

bool checkAccStop() {
    if (acc_stop_detected) {
        if (wait_count_acc_stop != 0) {
            return false;
        }
        //if (PINB & _BV(PORTB4)) {
        if (PORTA.IN & PIN3_bm) {
            acc_stop_detected = false;
            return false;
        }
        acc_stop_detected = false;
        doProcedureForAccStop();
        return true;
    }
    
    //if (PINB & _BV(PORTB4)) {
    if (PORTA.IN & PIN3_bm) {
        return false;
    }
    acc_stop_detected = true;
    wait_count_acc_stop = DELAY_ACC_STOP;
    return false;
}

void checkLongRun() {
    if (!long_run_detected) {
        return;
    }
    long_run_detected = false;
    doProcedureForLongRun();
}

void doProcedureForAccStart()
{
    initializeSoundQueue();
    addListToSoundQueue(sound_acc_start, sizeof(sound_acc_start));
    startSoundInQueue();
}

void doProcedureForAccStop()
{
    //TIMSK &= ~_BV(TOIE1); // Disable TIM1_OVF interrupt to stop the long run detection
    RTC.INTCTRL &= ~RTC_OVF_bm; // Disable RTC overflow interrupt to stop the long run detection
    initializeSoundQueue();
    addListToSoundQueue(sound_acc_stop, sizeof(sound_acc_stop));
    startSoundInQueue();
}

void doProcedureForLongRun()
{
    initializeSoundQueue();
    addListToSoundQueue(sound_long_run, sizeof(sound_long_run));
    startSoundInQueue();
}

void shutdown()
{
    //PORTB &= ~_BV(PORTB3);
    PORTA.OUTCLR = PIN7_bm; // Power IC shuts down 5V line
    _delay_ms(2000); // wait for a while until the 5V power is completely shut down
    
    // ATtiny202 can invoke software reset without utilizing Watchdog
    //wdt_enable(WDTO_30MS);
    RSTCTRL.SWRR = RSTCTRL_SWRE_bm; // This line won't be executed, but reset here in case the power be still alive.
    
    while(1);
}

/*
void wdt_init()
{
    MCUSR = 0;
    wdt_disable();
}
*/

int main(void)
{
    //DDRB |= _BV(DDB1); // Set PB1(Pin 6) to output (PWM)
    PORTA.DIRSET = PIN6_bm; // Set PA6(Pin 2) to output (PWM, TCB0-WO0)
    //DDRB |= _BV(DDB3); // Set PB3(Pin 2) to output (PowerIC)
    PORTA.DIRSET = PIN7_bm; // Set PA7(Pin 3) to output (PowerIC)
    //DDRB &= ~_BV(DDB4); // Set PB4(Pin 3) to input (ACC check)
    PORTA.DIRCLR = PIN3_bm; // Set PA3(Pin 7) to input (ACC check)
    
    // Disable PowerIC until the first sound starts
    PORTA.OUTCLR = PIN7_bm;
    
    _delay_ms(100); // Running at 3.3MHz at this time. The actual delay should be 600ms.
    
    // Set CPU frequency to 20MHz
    //CCP = CCP_IOREG_gc; // Prepare to write CCP values
    //CLKCTRL.MCLKCTRLB = 0; // Disable prescaler (CLK_PER=20MHz)
    _PROTECTED_WRITE(CLKCTRL_MCLKCTRLB, 0); // Use a predefined macro instead.
    
    cli(); // Disable the global interrupt flag when configuring timers.
    // Timer 0
    //TCCR0A = _BV(COM0B1) | _BV(WGM01) | _BV(WGM00); // Clear OC0B on Compare Match, set OC0B at BOTTOM, Fast PWM
    //TCCR0B = _BV(CS00); // No Prescaling (= 16MHz)
    
    // ATtiny202 has an independent RTC in addition to two timers(A and B), so we can use both timers for sound output.
    // Timer A for sound resolution (8000Hz)
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc; // Clock is CLK_PER(=20MHz), Disable TCA0 peripheral
    TCA0.SINGLE.PER = 2500UL; // TOP is 2500 (20MHz / 8000Hz = 2500)
    TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_NORMAL_gc; // Operation is normal
    TCA0.SINGLE.DBGCTRL = TCA_SINGLE_DBGRUN_bm;
    TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm; // Enable TCA0 peripheral
    // Timer B for sound level (8 bit)
    TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc; // Clock is CLK_PER(=20MHz), Disable TCB0 peripheral
    TCB0.CCMPH = 0; // Duty cycle is 0 at this time
    TCB0.CCMPL = 0xFF;  // Counter resets at 0xFF
    //TCB0.CNT = 0x0000; // The datasheet recommends to write 0x0000 to the count register, but PWM somehow won't start in this way.
    TCB0.CTRLB = TCB_CNTMODE_PWM8_gc | TCB_CCMPEN_bm; // 8bit PWM, Enable TCB0-WO0
    TCB0.DBGCTRL = TCB_DBGRUN_bm;
    TCB0.CTRLA |= TCB_ENABLE_bm; // Enable TCB0 peripheral
    
    // Timer 1
    //TCCR1 = _BV(CS13) | _BV(CS12) | _BV(CS11) | _BV(CS10); // Prescale CK/16384 (= 1kHz)
    // RTC
    while (RTC.STATUS & RTC_CTRLABUSY_bm); // Wait for CTRLABUSY cleared
    RTC.CTRLA &= ~RTC_RTCEN_bm; // Disable RTC peripheral
    RTC.CLKSEL = RTC_CLKSEL_INT1K_gc; // RTC clock source is 1.024kHz
    while (RTC.STATUS & RTC_CTRLABUSY_bm); // Wait for CTRLABUSY cleared
    RTC.CTRLA |= RTC_PRESCALER_DIV1024_gc; // Prescale RTC by 1024 (1.024kHz / 1024 = 1Hz)
    while (RTC.STATUS & RTC_PERBUSY_bm); // Wait for PERBUSY cleared
    RTC.PER = 0; // OVF interrupt occurs every one second
    while (RTC.STATUS & RTC_CTRLABUSY_bm); // Wait for CTRLABUSY cleared
    RTC.DBGCTRL = RTC_DBGRUN_bm;
    RTC.CTRLA |= RTC_RTCEN_bm; // Enable RTC peripheral
    sei(); // Enable the global interrupt flag since all timers are configured.
    
    //USI_TWI_Master_Initialise();
    i2cMasterInit();
    scanSoundBank();
    readSoundTable();
    sound_id = 0;
    
    //TIMSK |= _BV(TOIE0) | _BV(TOIE1); // Enable TIM0_OVF and TIM1_OVF interrupt
    TCA0.SINGLE.INTCTRL |= TCA_SINGLE_OVF_bm; // Enable Timer A overflow interrupt (at 8000Hz)
    RTC.INTCTRL |= RTC_OVF_bm; // Enable RTC overflow interrupt (at 1Hz)
    
    wdt_reset();
    _delay_ms(DELAY_ACC_START * 1000);
    wdt_reset();
        
    // Enable PowerIC
    //PORTB |= _BV(PORTB3);
    PORTA.OUTSET = PIN7_bm;
    
    doProcedureForAccStart();
    
    while (1) {
        wdt_reset();
        loadSoundByte();
        if (checkAccStop()) {
            break;
        }
        checkLongRun();
    }
    while (sound_id != 0) {
        wdt_reset();
        loadSoundByte();
    }
    shutdown();
}



