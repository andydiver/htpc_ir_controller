/*
 *  22/05/2015
 * 
 *  Capture IR pulse lengths & decode
 *
 *  Interrupt driven detection of input IR pulses to decode IR keypresses in 
 *  NEC/Samsung/RC6 protocols  - stores in RAM IR_BUFFER) & logs -> USART after detection (1-shot)
 *  Tested on ATmega328P
 *
 * AVR ATmega328P pinout :
 *                         ----------
 *   RPi GPIO#22  -> /RST |1       28| PC5
 *             USART Rx   |2       27| PC4
 *   RPi Rx <- USART Tx   |3       26| PC3
 *                   INT0 |4       25| PC2
 *                   PD3  |5       24| PC1
 *                   PD4  |6       23| PC0
 *                   Vcc  |7       22| Gnd
 *                   Gnd  |8       21| Aref
 *             Xtal1,PB6  |9       20| AVcc
 *             Xtal2,PB6  |10      19| PB5,SCK  -> RPi prog SPI
 *                   PD5  |11      18| PB4,MISI -> RPi prog SPI
 *                   PD6  |12      17| PB3,MOSI -> RPi prog SPI
 *                   PD7  |13      16| PB2  ->  IR output LED
 *   IR detector ->  ICP1 |14      15| PB1  ->  LED
 *                         ----------
 *
 *  Connect IR detector output to ICP1 for 16b TIMER1 use, or INT0 for 8b TIMER0
 *  Use -DBIT8_TIMER  option to use 8b.  16b works better. 
 */

#define MAX_LOG_LINE_SIZE   20

#include "ir_controller.h"
#include <string.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

//  For USART
#define TX_QLEN        64                      // NB!  power of 2
#define TX_QLEN_MASK   (TX_QLEN - 1)
static volatile uint8_t tx_r, tx_w;
static volatile uint8_t tx_queue[TX_QLEN];

char logstr[MAX_LOG_LINE_SIZE];

static volatile uint16_t timer1_overflows = 0;
static volatile uint32_t interval_us = 0;
static volatile uint32_t interval_lirc = 0;
static volatile uint16_t last_to    = 0;
static volatile uint8_t  last_TCNT0 = 0;
static volatile uint8_t  samsung_nec;
static volatile uint8_t  nec_rpt_count = 0;
static volatile uint16_t timer0_ticks_countdown, timer2_ticks_countdown;
static volatile uint8_t  ir_blast_bit_no;
static volatile uint8_t  num_blast;
static volatile uint8_t  ir_blast_num;
static volatile uint8_t  tv_mode = 0;          //  0 is usual case where IR codes are sent to PC via serial -> lirc
static volatile uint8_t  num_rpt = 0;          //  counter for key held down in single keypress event

static volatile enum IR_STATE       ir_rcv_state   = IR_NO_START;
static volatile enum IR_BLAST_STATE ir_blast_state = IR_BLAST_NO_START;
static volatile uint32_t ir_scancode;
static volatile uint32_t ir_samsung_bits, ir_samsung_bits_tmp;
static volatile uint8_t  ir_scanbyte;
static volatile uint8_t  addr_byte;
static volatile uint8_t  cmd_byte;
static volatile uint8_t  bit_no        = 0;
static volatile uint8_t  rc6_bit_value = 1;        //  We use 2nd stop bit as the start, which has value 1, that is the initial value (hi -> lo voltage transition)
static volatile uint8_t  mce_toggle_bit, mce_toggle_bit_prev;   //  State of the toggle bit from the MCE remote (NB it's not the official toggle bit defined in the RC6 spec!)
static volatile uint8_t  status_code   = 0;        //  0 success, see .h  for error codes
static volatile uint8_t  scan_ready    = 0;        //  scan is ready to send via UDP
static volatile uint8_t  is_pulse      = TRUE;
static volatile uint8_t  led_active    = 0;
static volatile uint8_t  num_led_switches;

static uint8_t us_per_timer0_tick, us_per_timer1_tick, us_per_timer2_tick;
static uint8_t prescale_timer0, prescale_timer2;
static uint16_t prescale_factors[5]    = {1, 8, 64, 256, 1024};
static uint16_t prescale_factors2[7]   = {1, 8, 32, 64, 128, 256, 1024};

static void ir_blast_samsung(uint32_t, uint8_t);
static uint32_t samsung_bitorder(uint32_t);

/*  blink LED rapidly n times    */
void blink(uint8_t n) {
    uint8_t i;
    for (i=0; i < n * 2; i++) {
        LED_TOGGLE();
        _delay_ms(250);
    }
    return;
}

/*  Set up the hardware timers and interrupts.
 */
void hw_init() {
    /*  Using 16b TIMER1 with Input Capture on ICP1 pin which triggers interrupts on edges defined by ICES1
     *  We must detect rising & falling edges, so have to redefine ICES1 bit in the interrupt handler to alternate.
     */
    wdt_disable();

    TCNT1=0;
    /* ICES1 should be 0 (falling edge) initially, to trigger 1st on remote button press. */ 
    /* IR detector voltage on ICP1 is normally hi (space) and goes lo on IR pulse. */
    TCCR1B  = _BV(ICNC1); 
    TIMSK1 |= _BV(TOIE1) | _BV(ICIE1);

    uint8_t prescale_timer1 = 4;
    /*  Decrease the prescale factor until we get a Timer resolution of 10us or lower 
     * eg  F_CPU = 1Mz   prescale factor=8   ->   8us prescale_timer1=1 CSbits=010 (2)
     *     F_CPU = 8MHz  prescale factor=64  ->   8us prescale_timer1=2 CSbits=011 (3)
     *     F_CPU = 16MHz prescale factor=64  ->   4us prescale_timer1=2 CSbits=011 (3)
     */
    while (prescale_timer1--) {
        us_per_timer1_tick = 1000000 * prescale_factors[prescale_timer1] / F_CPU;    //  NB integer div so will round!
        if (us_per_timer1_tick <= 10)
            break;
    }
    TCCR1B |= (prescale_timer1 + 1);                               //  set prescale bits CS12, CS11, CS10 - starts TIMER1

    /*  Setup INT0  for PC powerdown detection  */
    EICRA = 0b00000010;                               //  interrupt on falling edge of INT0 -> PC powerdown event
    EIMSK |= _BV(INT0);

    /* Set up 8bit TIMER2 for IR blast timing */ 
    TCCR2A = _BV(WGM21);                              //  Use CTC mode WGM=2
    TIMSK2 = _BV(OCIE2A); 
    prescale_timer2 = 6;
    /*  Decrease the prescale factor until we get a Timer resolution of 16us or lower 
     *  eg F_CPU = 16MHz prescale factor=128  ->   us_per_timer1_tick=8 prescale_timer2=4 CSbits=101 (5)
     *  See ir_blast_samsung()
     */
    while (prescale_timer2--) {
        us_per_timer2_tick = 1000000 * prescale_factors2[prescale_timer2] / F_CPU;
        if (us_per_timer2_tick <= 12)
            break;
    }

    /*  Setup TIMER0 to control the user-feedback LED  */
    TCCR0A = 0;
    TIMSK0 = _BV(OCIE0A);
    prescale_timer0    = 3;                        //  Will set CS=100b - prescale factor 256
    us_per_timer0_tick = 1000000 * prescale_factors[prescale_timer0] / F_CPU;    //  eg 16us for F_CPU = 16MHz
}

void usart_init(uint16_t baudrate) {
    uint16_t UBRR_val = (F_CPU / 16) / (baudrate - 1);
    UBRR0H  = UBRR_val >> 8;
    UBRR0L  = UBRR_val;
    UCSR0B |= _BV(TXEN0) | _BV(RXEN0) | _BV(RXCIE0);     // UART TX
    UCSR0C |= _BV(USBS0) | _BV(UCSZ00);                  //  Asynch 8N1 (8 Databits, No Parity, 1 Stopbit)
}

/*
 * USART - send a serial character
 */
void tx_char(uint8_t t) {
    uint8_t tmp;
    tmp = (tx_w + 1) & TX_QLEN_MASK;
    while (tmp == tx_r)
        /* spin for freespace */;
    tx_queue[tmp] = t;
    tx_w = tmp;
    UCSR0B |= _BV(UDRIE0);
}

/*
 * USART - send 16 bits, little-endian, for avrlirc2udp
 */
void tx_word(uint16_t t) {
    tx_char(t & 0xff);
    tx_char((t >> 8) & 0xff);
}

/*
 * USART - send 32 bits, little-endian, for avrlirc2udp
 */
void tx_int(uint32_t t) {
    tx_char(t & 0xff);
    tx_char((t >> 8)  & 0xff);
    tx_char((t >> 16) & 0xff);
    tx_char((t >> 24) & 0xff);
}

/*  See http://stackoverflow.com/questions/960389/how-can-i-visualise-the-memory-sram-usage-of-an-avr-program  */
/* int free_ram () { */
/*     extern int __heap_start, *__brkval; */ 
/*     int v; */ 
/*     return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); */ 
/* } */

/*  eg  10110001 ->  10001101  */
uint8_t reverse_byte(uint8_t bite) {
    uint8_t out_bite = 0x00;
    uint8_t nbit = 7;
    while(nbit--) {
        if (bite & 0x01)
            out_bite |= 0x01;
        bite = bite >> 1;
        out_bite = out_bite << 1;
    }
    if (bite & 0x01)
        out_bite |= 0x01;
    return out_bite;
}

/*  Convert a 32bit scancode into the required bit order convention for Samsung remote
 *  eg the scancode for Samsung KEY_PWR = 0x07070202 -> 0xe0e040bf;
 */
uint32_t samsung_bitorder(uint32_t code) {
    uint8_t byte0, byte1, byte2, byte3;
    byte0 = reverse_byte( (uint8_t)(code >> 24) );
    byte1 = byte0;         //  shortcut - the 'custom code' byte is repeated
    byte2 = reverse_byte( (uint8_t) ((code & 0x0000ff00) >> 8) );
    byte3 = ~byte2;        //  scancode sent again with bits inverted
    return (uint32_t)byte0 << 24 | (uint32_t)byte1 << 16 | (uint32_t)byte2 << 8 | (uint32_t)byte3;
}

/*  initialize start of using timer2 (8b) to time the output of pulses to IR LED
 *  eg to control the Samsung TV as monitor from a MCE remote
 */
void ir_blast_samsung(uint32_t code, uint8_t rpt) {
    num_blast = rpt;
    ir_blast_num = 0;
    ir_samsung_bits = code;
    ir_blast_bit_no = 0;
    ir_blast_state = IR_BLAST_START;
    TCNT2 = 0;
    //  start the header with long pulse and setup timer2 to time the duration
    IR_LED_ON();
    timer2_ticks_countdown = IR_NEC_START_LEN2 / us_per_timer2_tick;
    OCR2A = min(timer2_ticks_countdown, 0xff);
    TIMSK2 |= _BV(OCIE2A);
    /* TCCR2B = _BV(CS22) | _BV(CS21);                            //  start timer2: prescale_timer2=256  (us_per_timer2_tick=16us @16MHz) */
    TCCR2B |= (prescale_timer2 + 1);                              //  start timer2: set prescale bits CS22, CS21, CS20
}

/*  It seems that to get decoded scancodes to lirc, the easiest way is to re-encode
 *  them as a series of pulse lengths (mode2) and send via UDP (as per avrlirc approach) to and lircd
 *  daemon running with udp driver (dflt port is 8765)
 *  lircd --driver=udp --driver-option=clocktick:1
 *
 *  The scancode 4bytes is held in volatile uint32_t  ir_scancode
 *
 *  Pulse lengths are output to USART tx_queue in same format as avrlirc as uint16_t words
 *  with MSB=1 meaning pulse and 0 for space.
 *  I am using with --driver-option=clocktick:1  which sets the units of pulse lengths to 1us
 *  instead of the fairly crude 61us dflt value.
 *
 *  RC-6 uses Manchester encoding with a lo->hi being 0 and hi->lo being 1
 *  NB  The MCE remote ignores the official RC-6 TOGGLE bit and instead uses bit17 in the 4bytes following.
 *  eg  KEY_OK ->  0x800f0422  next press -> 0x800f8422
 */
void rc6_encode_and_send(void) {
    uint8_t count;
    uint8_t is_high;
    uint32_t scancode_tmp = ir_scancode;

    /* A long 1b length value: this is a bit-defining transition, indicating a transition in the opposite sense from previous 
     * hard-code the lengths for the START bits, 3 field bits & toggle bit
     */
    tx_word((uint16_t)IR_RC6_START);                         // 2.7ms long pulse to 1st START bit transition hi->lo (1)
    tx_word((uint16_t)IR_RC6_BITLEN | 0x8000);               // 895us space
    tx_word((uint16_t)IR_RC6_HALF);                          // short 452us pulse to 2nd START bit transition hi->lo  (1)
    tx_word((uint16_t)IR_RC6_HALF   | 0x8000);               // 452us space
    tx_word((uint16_t)IR_RC6_HALF);                          // short pulse to 1st FIELD bit (1)
    tx_word((uint16_t)IR_RC6_HALF   | 0x8000);
    tx_word((uint16_t)IR_RC6_HALF);                          // short pulse to 2nd FIELD bit (1)
    tx_word((uint16_t)IR_RC6_BITLEN | 0x8000);               // long space to 3rd FIELD bit transition lo->hi (0)
    tx_word((uint16_t)IR_RC6_HALF);                          // short pulse to 2nd FIELD bit (1)
    tx_word((uint16_t)IR_RC6_BITLEN | 0x8000);               // 'short-for-toggle' space to TOGGLE bit transion hi->lo (0)  NB TOGGLE bit has longer timings

    /*  state is lo since we just output toggle bit = 0  */
    is_high = FALSE;

    /*  Go through the bits of scancode_tmp, encoding the MSbit & shifting remaining bits left each time.
     *  Have to deal with longer TOGGLE bit timing for MSB scancode bit as special case
     */
    if (scancode_tmp & 0x80000000) {
        //  We need to output 1 so transition is lo->hi
        tx_word((uint16_t)IR_RC6_TOGGLE);                    //  Stay lo for 1357us pulse (extra TOGGLE length)
        is_high = TRUE;                                      //  then lo->hi (1)
    } else {
        //  We need to output 0 so transition is hi->lo
        tx_word((uint16_t)IR_RC6_BITLEN);                    //  'short-for-TOGGLE' pulse 895us
        tx_word((uint16_t)IR_RC6_HALF | 0x8000);             //  short space of 452.5us
        is_high = FALSE;                                     //  then hi->lo (0)
    }
    scancode_tmp = (scancode_tmp << 1);                      //  shift to access next bit of scancode

    count = 30;                                              //  encode next 30 bits
    while(count--) { 
        if (scancode_tmp & 0x80000000) {
            if (is_high) {
                //  short space + short pulse, so transition is (hi->)lo->hi (1)
                tx_word((uint16_t)IR_RC6_HALF | 0x8000);     //  hi->lo
                tx_word((uint16_t)IR_RC6_HALF);              //  bit defined by lo->hi (1)
            } else {
                //  long pulse stays lo & transition lo->hi  (1)
                tx_word((uint16_t)IR_RC6_BITLEN);
            }
            is_high = TRUE;
        } else {
            //  We need to output 0 
            if (is_high) {
                //  long space then transition hi->lo (0)
                tx_word((uint16_t)IR_RC6_BITLEN | 0x8000);
            } else {
                //  short pulse + short space so (lo->)hi->lo (0)
                tx_word((uint16_t)IR_RC6_HALF);            //  lo->hi
                tx_word((uint16_t)IR_RC6_HALF | 0x8000);   //  bit defined by hi->lo (0)
            }
            is_high = FALSE;
        }
        scancode_tmp = (scancode_tmp << 1);
    }

    //  last bit, set up appropriate waiting space
    if (scancode_tmp & 0x80000000) {
        if (is_high) {
            //  short space + short pulse, so transition is lo->hi (1) 
            tx_word((uint16_t)IR_RC6_HALF | 0x8000);      // hi->lo
            tx_word((uint16_t)IR_RC6_HALF);               // bit defined by lo->hi (1)
        } else {
            tx_word((uint16_t)IR_RC6_BITLEN);             // long pulse then lo->hi (1) 
        }
        /*  Gap will overflow 2-bytes if 1us resolution used, use 'extended time' send extended time format
         *  15b value of 0 with MSBit set to indicate SPACE - see lirc/plugins/udp.c
         */
        tx_word(0x8000);                                    
        tx_int((uint32_t)IR_RC6_GAP);                     //  long space for GAP
    } else {
        //  We need to output 0 
        if (is_high) {
            //  long space & transition hi->lo (0)
            tx_word((uint16_t)IR_RC6_BITLEN | 0x8000);
        } else {
            //  short pulse + short space
            tx_word((uint16_t)IR_RC6_HALF);               //  lo->hi
            tx_word((uint16_t)IR_RC6_HALF | 0x8000);      //  bit defined by hi->lo (0)
        }
        //  Left lo so need to return to hi 
        tx_word((uint16_t)IR_RC6_HALF);                   //  short pulse  lo->hi
        tx_word(0x8000);                                    
        tx_int((uint32_t)(IR_RC6_GAP - IR_RC6_HALF));     //  long space for GAP
    }

}


/* Main MAIN */
uint8_t main(void) {

    /* GPIO Output pins */
    DDR_LED  = _BV(LED_OUT) | _BV(IR_LED_OUT);
    DDR_PC  |= _BV(PC_PWRSW_OUT);

    blink(2);                //  just to show some reset activity

    usart_init(38400);       //  Baudrate has to match avrlirc2upd daemon on PC client. 

    hw_init();               //  Set up interrupts & TIMER registers etc

    /*  Watchdog timer: ensure that wdt_reset() is called before timeout else a reset will occur */
    wdt_enable(WDTO_8S);
    sei();

    while(1) {
        wdt_reset();
        cli();
        if (!interval_lirc) {
            /*  see notes in  /usr/lib/avr/include/avr/sleep.h  */
            sleep_enable();
            sei();
            sleep_cpu();
            sleep_disable();
        }
        sei();

        if (scan_ready) {
            if (IS_PC_ON() && !tv_mode) {
                /* Send ocde to serial -> lirc */
                rc6_encode_and_send();
            }

            mce_toggle_bit = ir_scancode & (uint32_t)MCE_TOGGLE_MASK ? 1 : 0;

            if (mce_toggle_bit == mce_toggle_bit_prev) {
                num_rpt++;        //  toggle bit hasn't changed: this is a rpt keypress
            } else {
                num_rpt = 0;      //  new keypress
                mce_toggle_bit_prev = mce_toggle_bit;
            }

            /* The 0xffff7fff masks the MCE toggle bit */

            //switch (ir_scancode & 0xffff7fff) {
            switch (ir_scancode & ~(uint32_t)MCE_TOGGLE_MASK) {
                case IR_MCE_KEY_POWER:
                    if (tv_mode) {
                        /* Turn TV on/off  */
                        ir_blast_samsung(samsung_bitorder((uint32_t)IR_SAMSUNG_SCANCODE_PWR), 4);
                    } else {
                        if (!IS_PC_ON()) {
                            if ( !IS_TV_ON() ) {
                                /* Turn on Samsung TV (monitor) by sending Samsung/NEC code via IR LED 
                                 * Do this 1st as it's slow to turn on (10s or so) ...
                                 */
                                ir_blast_samsung(samsung_bitorder((uint32_t)IR_SAMSUNG_SCANCODE_PWR), 4);
                            }

                            /* The PC is off so start it up and turn the monitor/TV on if it's off 
                             * Send a pulse to NPN transistor to trigger the PC startup by taking 
                             * the voltage low on the wire connected to the push-to-make power switch
                             */
                            PC_PWRSW_PORT |=  _BV(PC_PWRSW_OUT);
                            _delay_ms(500);
                            PC_PWRSW_PORT &= ~_BV(PC_PWRSW_OUT);
                            /* Individual delays mustn't exceed the WDT interval else get infinite restart cycle! */
                            _delay_ms(5000);
                            wdt_reset();
                            _delay_ms(7000);
                            wdt_reset();
                            /*  Send HDMI-mode code in case TV was left in wrong mode */
                            ir_blast_samsung(samsung_bitorder((uint32_t)IR_SAMSUNG_SCANCODE_HDMI), 4);
                        }
                    }
                    break;;

                /* case IR_MCE_KEY_BLUE: */
                /*     tv_mode = 0; */
                /*     break;; */

                case IR_MCE_KEY_LIVETV:        //  TV mode toggles - LED will be on when in TV mode
                    if (num_rpt == 1) {        //  avoid rapid repeat toggling!
                        tv_mode = (tv_mode ^ 0x01);
                        if (tv_mode)
                            LED_ON();          //  LED on as visual indicator of TV mode
                    }
                    break;;

                case IR_MCE_KEY_VOLUMEUP:
                    if (tv_mode)  
                        ir_blast_samsung(samsung_bitorder((uint32_t)IR_SAMSUNG_SCANCODE_VOLUMEUP), 1);
                    break;;

                case IR_MCE_KEY_VOLUMEDOWN:
                    if (tv_mode)  
                        ir_blast_samsung(samsung_bitorder((uint32_t)IR_SAMSUNG_SCANCODE_VOLUMEDOWN), 1);
                    break;;

                case IR_MCE_KEY_MUTE:
                    if (tv_mode)  
                        ir_blast_samsung(samsung_bitorder((uint32_t)IR_SAMSUNG_SCANCODE_MUTE), 2);
                    break;;

                case IR_MCE_KEY_LEFT:
                    if (tv_mode)  
                        ir_blast_samsung(samsung_bitorder((uint32_t)IR_SAMSUNG_SCANCODE_LEFT), 1);
                    break;;

                case IR_MCE_KEY_RIGHT:
                    if (tv_mode)  
                        ir_blast_samsung(samsung_bitorder((uint32_t)IR_SAMSUNG_SCANCODE_RIGHT), 1);
                    break;;

                case IR_MCE_KEY_UP:
                    if (tv_mode)  
                        ir_blast_samsung(samsung_bitorder((uint32_t)IR_SAMSUNG_SCANCODE_UP), 1);
                    break;;

                case IR_MCE_KEY_DOWN:
                    if (tv_mode)  
                        ir_blast_samsung(samsung_bitorder((uint32_t)IR_SAMSUNG_SCANCODE_DOWN), 1);
                    break;;

                case IR_MCE_KEY_MEDIA:
                    if (tv_mode)  
                        ir_blast_samsung(samsung_bitorder((uint32_t)IR_SAMSUNG_SCANCODE_MENU), 1);
                    break;;

                case IR_MCE_KEY_OK:
                    if (tv_mode)  
                        ir_blast_samsung(samsung_bitorder((uint32_t)IR_SAMSUNG_SCANCODE_ENTER), 1);
                    break;;

                case IR_MCE_KEY_CLEAR:
                    if (tv_mode)  
                        ir_blast_samsung(samsung_bitorder((uint32_t)IR_SAMSUNG_SCANCODE_EXIT), 1);
                    break;;

                case IR_MCE_KEY_BACK:
                    if (tv_mode)  
                        ir_blast_samsung(samsung_bitorder((uint32_t)IR_SAMSUNG_SCANCODE_RETURN), 1);
                    break;;

                case IR_MCE_KEY_INFO:
                    if (tv_mode)  
                        ir_blast_samsung(samsung_bitorder((uint32_t)IR_SAMSUNG_SCANCODE_INFO), 1);
                    break;;
            }

            scan_ready = 0;
    	}
    }

    return 0;
}

/*
 * USART transmit interrupt handler.
 */
ISR(USART_UDRE_vect) {
    // disable USART interrupt, so we can reenable interrupts in general
    UCSR0B &= ~_BV(UDRIE0);
    sei();
    // if data in queue, send it, and re-enable the interrupt
    if (tx_r != tx_w) {
        tx_r  = (tx_r + 1) & TX_QLEN_MASK;
        UDR0  = tx_queue[tx_r];
        UCSR0B |= _BV(UDRIE0);
    }
}

/* PC shutdown event is detected by INT0 which monitors PC's 3V3 line, only hi->lo triggers the INT0 */
ISR(INT0_vect) {
    _delay_ms(2);
    if (IS_PC_ON() )        //  check for temporary glitch
        return;
    _delay_ms(100);
    if (IS_PC_ON() )
        return;
    /*  PC shutdown event: turn the Samsung TV/monitor OFF if it's on...  */
    if (IS_TV_ON() ) 
        ir_blast_samsung(samsung_bitorder((uint32_t)IR_SAMSUNG_SCANCODE_PWR), 4);
}

/* 8b TIMER0  handles visual feedback LED */
ISR(TIMER0_COMPA_vect) {
    /*  Handle delays > 8bit ticks  */
    if (timer0_ticks_countdown  > 0xff) {
        timer0_ticks_countdown -= 0xff;
        OCR0A = min(timer0_ticks_countdown, 0xff);
        return;
    }
    LED_TOGGLE();

    num_led_switches++;
    if (num_led_switches == 2) {
        if (tv_mode) {           //  LED on as visual reminder that it's in TV mode
            LED_ON();
        } else {
            LED_OFF();
        }
        num_led_switches = 0;
        led_active = 0;
        TCCR0B     = 0;        //  stop TIMER0
    }
}

ISR(TIMER2_COMPA_vect) {
    /*  Handle delays > 8bit ticks  */
    if (timer2_ticks_countdown  > 0xff) {
        timer2_ticks_countdown -= 0xff;
        OCR2A = min(timer2_ticks_countdown, 0xff);
        return;
    }

    switch (ir_blast_state) {
        case IR_BLAST_RESTART:
            //  Initial pulse of header 
            timer2_ticks_countdown = IR_NEC_START_LEN2 / us_per_timer2_tick;
            IR_LED_ON();
            //  the next state is...
            ir_blast_state = IR_BLAST_START;
            break;

        case IR_BLAST_START:
            //  setup timer2 for the long space 2nd part of header
            timer2_ticks_countdown = IR_NEC_START_LEN2 / us_per_timer2_tick;
            IR_LED_OFF();
            //  the next state is...
            ir_blast_state = IR_BLAST_STARTED_PULSE;
            ir_samsung_bits_tmp = ir_samsung_bits;
            break;

        case IR_BLAST_STARTED_PULSE:
            timer2_ticks_countdown = IR_NEC_SHORT / us_per_timer2_tick;
            IR_LED_ON();
            ir_blast_state = IR_BLAST_STARTED_SPACE;
            break;

        case IR_BLAST_STARTED_SPACE:
            if (ir_samsung_bits_tmp & 0x80000000) 
                timer2_ticks_countdown = IR_NEC_LONG  / us_per_timer2_tick;
            else
                timer2_ticks_countdown = IR_NEC_SHORT / us_per_timer2_tick;

            ir_samsung_bits_tmp = (ir_samsung_bits_tmp << 1);    //  NEC outputs LSB 1st
            ir_blast_bit_no++;
            IR_LED_OFF();
            if (ir_blast_bit_no == 32)
                ir_blast_state = IR_BLAST_FINAL_PULSE;
            else
                ir_blast_state = IR_BLAST_STARTED_PULSE;
            break;

        case IR_BLAST_FINAL_PULSE:
            timer2_ticks_countdown = IR_NEC_SHORT / us_per_timer2_tick;
            IR_LED_ON();
            ir_blast_state = IR_BLAST_GAP;
            break;

        case IR_BLAST_GAP:
            timer2_ticks_countdown = IR_NEC_GAP / us_per_timer2_tick;
            IR_LED_OFF();
            if (ir_blast_num++ < num_blast)
                ir_blast_state = IR_BLAST_RESTART;
            else
                ir_blast_state = IR_BLAST_COMPLETED;
            break;

        case IR_BLAST_COMPLETED:
            IR_LED_OFF();
            TCCR2B = 0;        //  stop timer2
            return;
    }

    OCR2A = min(timer2_ticks_countdown, 255);
}

ISR(TIMER1_OVF_vect) {
    timer1_overflows++; 
}


/*  Interrupt handler for voltage transitions from the IR detector
 *  responsible for measuring duration of IR spaces and pulsesxa on ICP1 pin.
 *  Sets interval_lirc in units that need to agree with resolution setting of lircd daemon.
 *
 *  TIMER1_CAPT_vect for 16bit TIMER1 using ICP1 pin as IR input pin.
 *  Alternate ICES1  between falling/rising edges should be done ASAP after reading ICR1 register.
 *  Then clear ICF1 flag as changing ICES1 triggers a new interrupt immediately.
 */
ISR(TIMER1_CAPT_vect) {
    interval_us = us_per_timer1_tick * ((timer1_overflows - last_to) * 0x10000UL + (uint32_t)ICR1);   //  time in us (micro-seconds)
    /*  Need recent version of lirc which allows the  --driver-option=clocktick:N  unless dflt value of 61 is used  */
    interval_lirc = interval_us / LIRC_CLOCKTICK;

    /*  ICES1 bit 1 means interrupt was rising edge lo->hi, so the interval just finished was a pulse  */
    is_pulse = TCCR1B & _BV(ICES1); 

    TCCR1B ^= _BV(ICES1);                  //  reverse the edge sense interrupt and
    TIFR1  |= _BV(ICF1);                   //  clear the immediate interrupt caused by change of ICES1
    TCNT1H  = 0;                           //  reset 16b Timer1 counter
    TCNT1L  = 0;
    last_to = timer1_overflows;

    /* User feedback - start LED blinking to show IR activity detected */
    if (!led_active) {
        timer0_ticks_countdown = LED_FLASH_TIME / us_per_timer0_tick;
        OCR2A = min(timer2_ticks_countdown, 255);
        led_active = 1;
        TCCR0B |= (prescale_timer0 + 1);
    }

    switch (ir_rcv_state) {
        case IR_NO_START:
            if ( RC6_START(interval_us) ) {
                ir_rcv_state = IR_RC6_START1;     //  found 1st START bit - long pulse of 3b (2.7ms). Expect space of 1b then pulse of 0.5b
            } else if ( NEC_START(interval_us) ) {
                samsung_nec = FALSE;
                ir_rcv_state = IR_NEC_START1;     //  found 1st START pulse for NEC:  9ms
            } else if ( NEC_SAMSUNG_START(interval_us) ) {
                samsung_nec = TRUE;               //  handle the slight variation of NEC protocol for Samsung remotes
                ir_rcv_state = IR_NEC_START1;     //  found 1st START pulse for Samsung/NEC:  4.5ms - decode data as per NEC 
            }
            break;

        case IR_RC6_START1:
            if ( RC6_START1(interval_us) )
                ir_rcv_state = IR_RC6_START2;
            else
                ir_rcv_state = IR_NO_START;
            break;

        case IR_RC6_START2:
            if ( RC6_START2(interval_us) ) {
                //  hi->lo transition of 2nd START bit, this is where we start counting bits from
                ir_rcv_state = IR_RC6_STARTED;
                // (re)initialize values
                status_code = 0;
                rc6_bit_value = 1;
                bit_no = 0;
                ir_scancode = 0;
            } else {
                ir_rcv_state = IR_NO_START;
            }
            break;

        /*  Strategy is to identify those interrupts which define bit transitions in Manchester encoding, as opposed to 
         *  interrupts which are in preparation for a bit transition.
         *  2 successive half length timings indicate the bit is same as previous, whereas one long timing means a
         *  reversal of the bit value.
         */
        case IR_RC6_STARTED:
            if ( RC6_SHORT(interval_us) ) {
                //  the 1st of what should be 2 short half-length values: this is not a bit-defining transition
                ir_rcv_state = IR_RC6_EXPECT_SHORT;
            } else if ( RC6_LONG(interval_us) ) {
                //  a long 1b length value: this is a bit-defining transition, indicating a transition in the opposite sense from previous
                rc6_bit_value ^= 0x01;
                ir_scancode = (ir_scancode << 1) | rc6_bit_value;
                bit_no++;
                if (bit_no == 3) {
                   ir_rcv_state = IR_RC6_BEFORE_TOGGLE;
                } else if (bit_no == 36) {
                    scan_ready = 1;
                    ir_rcv_state = IR_NO_START;
                }
            } else {
                  /* out-of-bounds pulse length: start over */
                if (interval_us > IR_RC6_BITLEN_MAX) 
                    status_code = ERC6_TOOLONG;
                else if (interval_us < IR_RC6_HALF_MIN)
                    status_code = ERC6_TOOSHORT;
                else if (interval_us > IR_RC6_HALF_MAX  && interval_us < IR_RC6_BITLEN_MIN)
                    status_code = ERC6_BETWEEN;
                goto error_reset;
            }
            break;

        case IR_RC6_EXPECT_SHORT:
            if ( RC6_SHORT( interval_us ) ) {
                /* As expected, this is 2nd of 2 half-length values. This is bit transition with same bit value as last. */
                ir_scancode = (ir_scancode << 1) | rc6_bit_value;
                bit_no++; 
                ir_rcv_state = IR_RC6_STARTED;
                if (bit_no == 3) {
                   ir_rcv_state = IR_RC6_BEFORE_TOGGLE;
                } else if (bit_no == 36) {
                    /* sprintf(logstr, "RC6:%#010lx\n", ir_scancode); */
                    /* DBG_PRINT(logstr); */
                    scan_ready = 1;
                    ir_rcv_state = IR_NO_START;
                }
            } else {
                /* something wrong - we've skipped over an expected bit-defining transition point & lost data. 
                 * Abort - wait for another START */
                status_code = ERC6_BADSHORT;
                goto error_reset;
            }
            break;
        
        /*  RC6 Toggle bit (after 3 'field bits') is 2 x longer in total than a normal bit, so increase what we see as 
         *  long interval (1.0 -> 1.5) and short (0,5 -> 1.0) intervals which occur on either side of it.
         */
        case IR_RC6_BEFORE_TOGGLE:
            if ( RC6_SHORT(interval_us) ) {
                ir_rcv_state = IR_RC6_TOGGLE_EXPECT_SHORT;
            } else if (RC6_TOGGLE_LONG(interval_us) ) {
                //  a toggle long 1.5b length value: this is toggle bit-defining transition, rc6_bit_value reversed from previous
                rc6_bit_value ^= 0x01;
                ir_scancode = (ir_scancode << 1) | rc6_bit_value;
                bit_no++;
                ir_rcv_state = IR_RC6_AFTER_TOGGLE;
            } else if (interval_us > IR_RC6_TOGGLE_MAX) {
                status_code = ERC6_TOGLONG;
                goto error_reset;
            } else if (interval_us > IR_RC6_HALF_MAX && interval_us < IR_RC6_TOGGLE_MIN) {
                status_code = ERC6_TOGBETWEEN;
                goto error_reset;
            } else if (interval_us < IR_RC6_TOGGLE_MIN) {
                status_code = ERC6_TOGSHORT;
                goto error_reset;
            } else {
                 status_code = ERC6_BADTOG;
                 goto error_reset;
            }
            break;
            
        case IR_RC6_TOGGLE_EXPECT_SHORT:
            //  the 'short' length is 1.0 for the toggle bit
            if ( RC6_TOGGLE_SHORT(interval_us) ) {
                //  As expected, this is 2nd of 2 short intervals. Toggle bit = previous bit value.
                ir_scancode = (ir_scancode << 1) | rc6_bit_value;
                bit_no++;
                ir_rcv_state = IR_RC6_AFTER_TOGGLE;
            } else if (interval_us > IR_RC6_BITLEN_MAX) {
                status_code = ERC6_TOGLONG;
                goto error_reset;
            } else if (interval_us < IR_RC6_BITLEN_MIN) {
                //  something wrong - we've skipped over an expected bit-defining transition point & lost data. Abort - wait for another START
                status_code = ERC6_TOGSHORT;
                goto error_reset;
            }
            break;

        /*  Still have longer toggle interval to deal with  */
        case IR_RC6_AFTER_TOGGLE:
            if ( RC6_TOGGLE_LONG(interval_us) ) {
                //  long interval - this is start of address bits - after the toggle bit
                rc6_bit_value ^= 0x01;
                ir_scancode = (ir_scancode << 1) | rc6_bit_value;
                bit_no++;
                //  Back to reading bits of normal length for the address & command bits
                ir_rcv_state = IR_RC6_STARTED;
            } else if ( RC6_TOGGLE_SHORT(interval_us) ) {
                ir_rcv_state = IR_RC6_EXPECT_SHORT;  //  after 'toggle-short', expect a 'normal-short' of 0.5 for 1st address bit
            } else {
                status_code = ERC6_BADTOG;
                goto error_reset;
            }
            break;

        case IR_NEC_START1:
            if ( NEC_START1(interval_us) ) {
                bit_no = 0;
                ir_scancode = 0;
                ir_rcv_state = IR_NEC_STARTED;        //  Got space of 4.5ms - confirmed NEC protocol
            } else if ( NEC_RPT_SHORT(interval_us) ) {
                //  NEC repeat code has 9ms hi -> 2.25ms lo -> 562us hi -> 96.2ms lo
                ir_rcv_state = IR_NEC_REPEAT1;
            } else {
                ir_rcv_state = IR_NO_START;
            }
            break;

        //  expecting a short pulse of 562.5us
        case IR_NEC_REPEAT1:
            if ( NEC_SHORT(interval_us) ) {
                ir_rcv_state = IR_NEC_REPEAT_WAIT;
            } else {
                ir_rcv_state = IR_NO_START;
            }
            break;

        case IR_NEC_REPEAT_WAIT:
            if ( NEC_RPT_TOSTART(interval_us) ) {
                ir_rcv_state = IR_NO_START;
            } else {
                ir_rcv_state = IR_NO_START;
            }
            break;

        case IR_NEC_STARTED:
            nec_rpt_count = 0; 
            if (is_pulse) {
                //  the pulse just completed at this interrupt (hi->lo) was a mark - 1st part of NEC bit
                if (!NEC_SHORT(interval_us) ) {
                    status_code = ENEC_BADSHORT;
                    goto error_reset;
                }
            } else {
                //  2nd part of bit - work out if 0 or 1 & store the value. Bits come LSB 1st
                if ( NEC_SHORT(interval_us) ) {
                    //  short+short is logical0
                    ir_scanbyte = (ir_scanbyte >> 1); 
                    bit_no++;
                } else if ( NEC_LONG(interval_us) ) {
                    //  short+long is logical1
                    ir_scanbyte = (ir_scanbyte >> 1) | 0x80;     //  set MSB to 1
                    bit_no++;
                } else {
                    if (interval_us > IR_NEC_LONG_MAX)
                        status_code = ENEC_TOOLONG;
                    else if (interval_us > IR_NEC_SHORT_MAX && interval_us < IR_NEC_LONG_MIN) 
                        status_code = ENEC_BETWEEN;
                    else 
                        status_code = ENEC_TOOSHORT;

                    goto error_reset;
                }
                if (bit_no == 8) {
                    addr_byte = ir_scanbyte;
                } else if (bit_no == 16) {
                    if (!samsung_nec) 
                        ir_scanbyte = ~ir_scanbyte;             // normal NEC protocol sends 2nd byte with bits inverted, Samsung doesn't

                    if (CHECK_NEC_CODES && addr_byte != ir_scanbyte) {
                        status_code = ENEC_ADDRMISMATCH;        //  inconsistent address bytes
                        goto error_reset;
                    }
                } else if (bit_no == 24) {
                    cmd_byte = ir_scanbyte;
                } else if (bit_no == 32) {
                    ir_scanbyte = ~ir_scanbyte;                // NEC protocol sends 4th byte with bits inverted
                    if (CHECK_NEC_CODES && cmd_byte != ir_scanbyte) {
                        status_code = ENEC_CMDMISMATCH;        //  inconsistent address bytes
                        goto error_reset;
                    }
                    ir_scancode = ((uint32_t)addr_byte << 24) | ((uint32_t)addr_byte << 16) | ((uint32_t)cmd_byte << 8) | (uint32_t)ir_scanbyte;
                    ir_rcv_state = IR_NO_START;
                }
            }
            break;
    }

    return;

error_reset:
    ir_rcv_state = IR_NO_START;
    /* sprintf(logstr, "\nErr%d l=%lu\n", status_code, interval_us); */
    /* DBG_PRINT(logstr); */
    return;
}
