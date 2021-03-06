#include <avr/io.h> 
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdlib.h>

/*TYPES*/
#define byte unsigned char 

/*VARIABLES*/
#define ON 0xFF
#define OFF 0x00
#define HIGH 0xFF
#define LOW 0x00
#define IN 0x00
#define OUT 0xFF
#define ALL 0xFF
#define TRUE 1
#define FALSE 0
#define FORWARD 0x00
#define BACK 0xFF
#define COMMAND 0x00
#define DATA 0xFF
#define BLACK 0xFF
#define WHITE 0x00
#define EMPTY 0x00

#define MAX_COLUMNS 102
#define MAX_PAGES  8

#define F_CPU 8000000UL

#define WREN 0x06
#define READ 0x03
#define WRITE 0x02
#define FIRST 0x00

#define LSHALF 0x0F

#define HALFBIT 4

#define SPI_FINISHED (SPSR & (1<<SPIF))


/*SET and GET MACROS*/
#define SET(PORT,MASK,VALUE) PORT = ((MASK & VALUE) | (PORT & ~MASK))
#define GET(PORT,MASK) PORT & MASK

//#define MULTI_LINES_DIR(DIR) SET(DDR?,(_BV(P?#)|_BV(P?#)|_BV(P?#)),DIR)
#define BAT_LOW_LED_DIR(DIR) SET(DDRB,_BV(PB3),DIR)
#define LCD_BACK_LIGHT_DIR(DIR) SET(DDRD,_BV(PD7),DIR)

#define UP_BUTTON_DIR(DIR) SET(DDRD,_BV(PD5),DIR)
#define DOWN_BUTTON_DIR(DIR) SET(DDRD,_BV(PD6),DIR)
#define LEFT_BUTTON_DIR(DIR) SET(DDRD,_BV(PD3),DIR)
#define RIGHT_BUTTON_DIR(DIR) SET(DDRD,_BV(PD4),DIR)
#define F_A_BUTTON_DIR(DIR) SET(DDRA,_BV(PA5),DIR)
#define F_B_BUTTON_DIR(DIR) SET(DDRA,_BV(PA6),DIR)
#define F_C_BUTTON_DIR(DIR) SET(DDRA,_BV(PA7),DIR)

#define TS_BOTTOM_DIR(DIR) SET(DDRA,_BV(PA1),DIR)
#define TS_LEFT_DIR(DIR) SET(DDRA,_BV(PA2),DIR)
#define TS_TOP_DIR(DIR) SET(DDRA,_BV(PA3),DIR)
#define TS_RIGHT_DIR(DIR) SET(DDRA,_BV(PA4),DIR)

#define TS_BOTTOM(STATE) SET(PORTA,_BV(PA1),STATE)
#define TS_LEFT(STATE) SET(PORTA,_BV(PA2),STATE)
#define TS_TOP(STATE) SET(PORTA,_BV(PA3),STATE)
#define TS_RIGHT(STATE) SET(PORTA,_BV(PA4),STATE)

#define GET_TS_BOTTOM ~GET(PINA,_BV(PA3))

#define UP_BUTTON_PULLUP(STATE) SET(PORTD,_BV(PD5),STATE)
#define DOWN_BUTTON_PULLUP(STATE) SET(PORTD,_BV(PD6),STATE)
#define LEFT_BUTTON_PULLUP(STATE) SET(PORTD,_BV(PD3),STATE)
#define RIGHT_BUTTON_PULLUP(STATE) SET(PORTD,_BV(PD4),STATE)
#define F_A_BUTTON_PULLUP(STATE) SET(PORTA,_BV(PA5),STATE)
#define F_B_BUTTON_PULLUP(STATE) SET(PORTA,_BV(PA6),STATE)
#define F_C_BUTTON_PULLUP(STATE) SET(PORTA,_BV(PA7),STATE)

//Device Outputs
//#define MULTI_LINES(STATE) SET(DDR?,(_BV(P?#)|_BV(P?#)|_BV(P?#)),DIR)

#define BAT_LOW_LED(STATE) SET(PORTB,_BV(PB3),~STATE)
#define BAT_LOW_LED_GET ~GET(PORTB,_BV(PB3))

#define LCD_BACK_LIGHT(STATE) SET(PORTD,_BV(PD7),STATE)
#define LCD_BACK_LIGHT_GET GET(PORTD,_BV(PD7))


#define LCD_RST_DIR(DIR) SET(DDRB,_BV(PB2),DIR)
#define LCD_RST(STATE) SET(PORTB,_BV(PB2),STATE)

//Device Inputs
#define UP_BUTTON ~GET(PIND,_BV(PD5))
#define DOWN_BUTTON ~GET(PIND,_BV(PD6))
#define LEFT_BUTTON ~GET(PIND,_BV(PD3))
#define RIGHT_BUTTON ~GET(PIND,_BV(PD4))
#define F_A_BUTTON ~GET(PINA,_BV(PA5))
#define F_B_BUTTON ~GET(PINA,_BV(PA6))
#define F_C_BUTTON ~GET(PINA,_BV(PA7))

#define UP_B_BUFF    0b00000001
#define DOWN_B_BUFF  0b00000010
#define LEFT_B_BUFF    0b00000100
#define RIGHT_B_BUFF   0b00001000
#define FA_B_BUFF    0b00010000
#define FB_B_BUFF    0b00100000
#define FC_B_BUFF    0b01000000



// #define LCD_CHIP_SELECT PORTB = ((_BV(PB0) & 0x00) | (PORTB & ~0x00))

#define MOSI_DIR(DIR) SET(DDRB,_BV(PB5),DIR)
#define MISO_DIR(DIR) SET(DDRB,_BV(PB6),DIR)
#define MOSI(STATE) SET(PORTB,_BV(PB5),STATE)

#define SCK_DIR(DIR) SET(DDRB,_BV(PB7),DIR)
#define SCK(STATE) SET(PORTB,_BV(PB7),STATE)

#define LCD_CS_DIR(DIR) SET(DDRB,_BV(PB0),DIR)
#define MU_SS_DIR(DIR) SET(DDRB,_BV(PB4),DIR)

#define MU_SS(STATE) SET(PORTB,_BV(PB4),STATE)

#define RAM_CS_DIR(DIR) SET(DDRC,_BV(PC1),DIR)
#define RAM_CS(STATE) SET(PORTC,_BV(PC1),STATE)

#define RAM_WP_DIR(DIR) SET(DDRC,_BV(PC6),DIR)
#define RAM_WP(STATE) SET(PORTC,_BV(PC6),STATE)

#define RAM_HOLD_DIR(DIR) SET(DDRC,_BV(PC7),DIR)
#define RAM_HOLD(STATE) SET(PORTC,_BV(PC7),STATE)

#define LCD_CD(MODE) SET(PORTB,_BV(PB1),MODE) 
#define LCD_CD_DIR(DIR) SET(DDRB,_BV(PB1),DIR) 

#define LCD_CS(STATE) SET(PORTB,_BV(PB0),STATE)


//#define LCD_CHIP_DESELECT
#define CMD_PAGE 0xB0
#define CMD_COL_LSB 0x00
#define CMD_COL_MSB 0x10


