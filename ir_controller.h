#ifndef IR_CONTROLLER_H
#define IR_CONTROLLER_H

#include <avr/io.h>
#include <avr/interrupt.h>

#if defined(__AVR_ATmega328P__)
    /* Output Connection to LED giving visual feedback on IR detection */
    #define DDR_LED       DDRB
    #define PORT_LED      PORTB
    #define LED_OUT       PB1
    /* Output Connection to IR diode for IR blasting to Samsung TV/monitor */
    #define IR_LED_OUT    PB2
    /* Input Connection to sense if PC is on/off from ATX PWR supply's 3V3 line */
    #define DDR_PC        DDRD
    #define PIN_PC        PIND
    #define PC_PWR_IN     PD2
    /* Output Connection to NPN transistor which switches PC PWR ON on startup  */
    #define PC_PWRSW_PORT PORTD
    #define PC_PWRSW_OUT  PD4
    /* Input Connection for sensing TV/monitor on/off state */
    #define PIN_TV        PIND
    #define TV_PWR_IN     PD3
#endif      //  __AVR_ATmega328P__

#define LED_ON()              { PORT_LED |=  _BV(LED_OUT); }
#define LED_OFF()             { PORT_LED &= ~_BV(LED_OUT); }
#define LED_TOGGLE()          { PORT_LED ^=  _BV(LED_OUT); }
#define IR_LED_ON()           { PORT_LED |=  _BV(IR_LED_OUT); }
#define IR_LED_OFF()          { PORT_LED &= ~_BV(IR_LED_OUT); }
#define IR_LED_TOGGLE()       { PORT_LED ^=  _BV(IR_LED_OUT); }
#define IS_PC_ON()            ( PIN_PC   & _BV(PC_PWR_IN) )
#define IS_TV_ON()            ( PIN_TV   & _BV(TV_PWR_IN) )

/* Visual feedback LED stays on for 50ms so approx 1 on/off cycle per 108ms IR code */
#define LED_FLASH_TIME        50000

/*  Factor to divide us timings to lircd's drv.resolution  value  eg --driver-option=clockticks:2      (dflt is 61 for lircd)
 */
// #define LIRC_CLOCKTICK    61;           //  not recommended - lircd has trouble decoding as resolution very crude
#define LIRC_CLOCKTICK       1;            //  run lircd  ver >= 0.9.2 with  '--driver-option=clockticks:1' option

/*  Timings for IR remote protocols:  units are in micro-seconds (us)
 *
 *  NEC  protocol: START pulse of 9ms then space of 4.5ms
 *  See   http://techdocs.altium.com/display/FPGA/NEC+Infrared+Transmission+Protocol
 *  Bits sent LSB 1st. Codes sent every 108ms
 *  logical 0, pulse length 562us, space 562us   (1:1)
 *  logical 1, pulse length 562us, space 1686us  (1:3)
 */
#define IR_NEC_START_LEN1_MIN    8950
#define IR_NEC_START_LEN1_MAX    9350
#define IR_NEC_START_LEN2        4500
#define IR_NEC_START_LEN2_MIN    4300
#define IR_NEC_START_LEN2_MAX    4750
#define IR_NEC_RPT_SHORT_MIN     2100        //  length of space after 9ms START pulse - 2.25ms (spec)
#define IR_NEC_RPT_SHORT_MAX     2400
#define IR_NEC_RPT_TOSTART_MIN   96000       //  delay (space) from end of rpt code to start of next - 96.2ms (spec)
#define IR_NEC_RPT_TOSTART_MAX   100000
//  legths that define logical0 and 1 (3:1 ratio)
#define IR_NEC_SHORT             562
#define IR_NEC_SHORT_MIN         400
#define IR_NEC_SHORT_MAX         680
#define IR_NEC_LONG              1686
#define IR_NEC_LONG_MIN          1600
#define IR_NEC_LONG_MAX          1800
#define IR_NEC_GAP               47000

/*  Samsung protocol is v. similar to NEC, but uses different START header.
 *  A pulse of 4.5ms is followed by a space of same length:
 *  It sends address ('custom code') twice as NEC, but unlike NEC, the 2nd byte isn't inverted.
 *  The key code is sent twice, but 2nd time with bits inverted (same as NEC).
 */
#define CHECK_NEC_CODES     1        //  NEC sends bytes twice, 2nd time inverted - do we error check for consistency?

#define NEC_START(len)       ({ len >= IR_NEC_START_LEN1_MIN && len <= IR_NEC_START_LEN1_MAX ? 1 : 0; })
#define NEC_START1(len)      ({ len >= IR_NEC_START_LEN2_MIN && len <= IR_NEC_START_LEN2_MAX ? 1 : 0; })
#define NEC_SAMSUNG_START    NEC_START1
#define NEC_SHORT(len)       ({ len >= IR_NEC_SHORT_MIN && len <= IR_NEC_SHORT_MAX ? 1 : 0; })
#define NEC_LONG(len)        ({ len >= IR_NEC_LONG_MIN  && len <= IR_NEC_LONG_MAX ?  1 : 0; })
#define NEC_RPT_SHORT(len)   ({ len >= IR_NEC_RPT_SHORT_MIN  && len <= IR_NEC_RPT_SHORT_MAX ?  1 : 0; })
#define NEC_RPT_TOSTART(len) ({ len >= IR_NEC_RPT_TOSTART_MIN  && len <= IR_NEC_RPT_TOSTART_MAX ?  1 : 0; })

#define IR_SAMSUNG_SCANCODE_PWR         0x07070202UL
#define IR_SAMSUNG_SCANCODE_VOLUMEUP    0x07070707UL
#define IR_SAMSUNG_SCANCODE_VOLUMEDOWN  0x07070b0bUL
#define IR_SAMSUNG_SCANCODE_MUTE        0x07070f0fUL
#define IR_SAMSUNG_SCANCODE_UP          0x07076060UL
#define IR_SAMSUNG_SCANCODE_DOWN        0x07076161UL
#define IR_SAMSUNG_SCANCODE_LEFT        0x07076565UL
#define IR_SAMSUNG_SCANCODE_RIGHT       0x07076262UL
#define IR_SAMSUNG_SCANCODE_MENU        0x07071a1aUL
#define IR_SAMSUNG_SCANCODE_ENTER       0x07076868UL
#define IR_SAMSUNG_SCANCODE_EXIT        0x07072d2dUL    // Home
#define IR_SAMSUNG_SCANCODE_RETURN      0x07075858UL    // Back
#define IR_SAMSUNG_SCANCODE_INFO        0x07071f1fUL
#define IR_SAMSUNG_SCANCODE_HDMI        0x07078b8bUL

/*  RC6 protocol for MCE IR remote  avge bit is 885
 *  See  http://www.pcbheaven.com/userpages/The_Philips_RC6_Protocol/
 *  START sequence is 2685us pulse (hi->lo defining 1st start bit '1') 895us space (lo->hi) 
 *  450us pulse (hi->lo defining 2nd start bit '1')
 *
 *  NB My Media Center remote control uses RC6 but the Toggle bit in the RC6 spec never changes.
 *  The toggle bit used by MCE remote is bit16 - the MSB of the 3rd byte after the 'official' RC6 toggle bit ie
 *  a scancode will change from  0x800f0416 -> 0x800f8016  the next time the 'PLAY' key is pressed
 */
#define MCE_TOGGLE_MASK        0x00008000
#define IR_RC6_START           2685
#define IR_RC6_START_MIN       2500   
#define IR_RC6_START_MAX       2900

#define IR_RC6_HALF            452
#define IR_RC6_HALF_MIN        280
#define IR_RC6_HALF_MAX        690
//  normal 1b length - should be 895us - can vary quite a bit
#define IR_RC6_BITLEN          895
#define IR_RC6_BITLEN_MIN      710
#define IR_RC6_BITLEN_MAX      1100

#define IR_RC6_TOGGLE          1357
#define IR_RC6_TOGGLE_MIN      1200
#define IR_RC6_TOGGLE_MAX      1600

//  Gap from end of IR signal pulses to next START
#define IR_RC6_GAP             68300

#define RC6_START(len)       ({ len >= IR_RC6_START_MIN  && len <= IR_RC6_START_MAX ?  1 : 0; })
#define RC6_START1(len)      ({ len >= IR_RC6_BITLEN_MIN && len <= IR_RC6_BITLEN_MAX ? 1 : 0; })
#define RC6_SHORT(len)       ({ len >= IR_RC6_HALF_MIN   && len <= IR_RC6_HALF_MAX ?   1 : 0; })
#define RC6_START2           RC6_SHORT
#define RC6_LONG(len)        ({ len >= IR_RC6_BITLEN_MIN && len <= IR_RC6_BITLEN_MAX ? 1 : 0; })
//  allowed a bit of extra leeway with toggle bit as lengths seem more variable. On MCE remote, the toggle bit has no meaning AFAICT
#define RC6_TOGGLE_SHORT(len) ({ len >= IR_RC6_BITLEN_MIN - 100  && len <= IR_RC6_BITLEN_MAX + 100 ?  1 : 0; })
#define RC6_TOGGLE_LONG(len) ({ len >= IR_RC6_TOGGLE_MIN && len <= IR_RC6_TOGGLE_MAX ? 1 : 0; })

#define IR_MCE_KEY_POWER           0x800f040c
#define IR_MCE_KEY_RED             0x800f045b
#define IR_MCE_KEY_GREEN           0x800f045c
#define IR_MCE_KEY_YELLOW          0x800f045d
#define IR_MCE_KEY_BLUE            0x800f045e
#define IR_MCE_KEY_VOLUMEUP        0x800f0410
#define IR_MCE_KEY_VOLUMEDOWN      0x800f0411
#define IR_MCE_KEY_MUTE            0x800f040e
#define IR_MCE_KEY_UP              0x800f041e
#define IR_MCE_KEY_DOWN            0x800f041f
#define IR_MCE_KEY_LEFT            0x800f0420
#define IR_MCE_KEY_RIGHT           0x800f0421
#define IR_MCE_KEY_MEDIA           0x800f040d    // Menu
#define IR_MCE_KEY_OK              0x800f0422    // Enter
#define IR_MCE_KEY_CLEAR           0x800f040a    // Exit
#define IR_MCE_KEY_BACK            0x800f0423    // Return
#define IR_MCE_KEY_INFO            0x800f040f
#define IR_MCE_KEY_LIVETV          0x800f0425    // LiveTV - for TV remote mode

//  error codes
#define ERC6_TOOSHORT              1
#define ERC6_BETWEEN               2
#define ERC6_TOOLONG               3 
#define ERC6_BADSHORT              4
#define ERC6_TOGSHORT              5
#define ERC6_TOGLONG               6
#define ERC6_TOGBETWEEN            7
#define ERC6_BADTOG                8

#define ENEC_TOOSHORT              10
#define ENEC_TOOLONG               11
#define ENEC_BETWEEN               12
#define ENEC_BADSHORT              13
#define ENEC_ADDRMISMATCH          14
#define ENEC_CMDMISMATCH           15

//  State variable for the ISR external interrupt processing of IR pulse intervals from IR detector
enum IR_STATE {IR_NO_START, IR_RC6_START1, IR_RC6_START2, IR_RC6_STARTED, IR_RC6_EXPECT_SHORT, IR_RC6_BEFORE_TOGGLE, \
               IR_RC6_TOGGLE_EXPECT_SHORT, IR_RC6_AFTER_TOGGLE, IR_NEC_START1, IR_NEC_STARTED, \
               IR_NEC_REPEAT1, IR_NEC_REPEAT_WAIT};

enum IR_BLAST_STATE {IR_BLAST_NO_START, IR_BLAST_RESTART, IR_BLAST_START, IR_BLAST_STARTED_PULSE, IR_BLAST_STARTED_SPACE, \
                     IR_BLAST_FINAL_PULSE, IR_BLAST_GAP, IR_BLAST_COMPLETED};

#define TRUE        1
#define FALSE       0

#define max(a,b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a > _b ? _a : _b; })
#define min(a,b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a < _b ? _a : _b; })

#ifdef DBG
    #ifdef I2C_LOG
      #define DBG_PRINT(msg) \
              do { i2c_puts(msg); } while (0)
    #endif  //  I2C_LOG
#else       //  DBG
  #define DBG_PRINT(msg) {}
#endif      //  DBG

#endif      // IR_CONTROLLER_H
