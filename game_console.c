#include "game_console.h" 


float vcc;
byte page = 0;
signed char column = 0;
signed char row = 0;
byte pixel = 0;
byte key_buffer = EMPTY;
byte frame_buffer[MAX_COLUMNS][MAX_PAGES];
byte brightness = 0;
byte key_status = 0x00;
byte width=16;


byte ball_array[5]={0b00001110,
					0b00011111,
					0b00011111,
					0b00011111,
					0b00001110,
					};

byte vert_pad = 0;
byte hor_pad = 0;
byte row_ball = 0;
byte col_ball = 0;

signed char dir_x = 1;
signed char dir_y = 1;

void RAM_write(byte spi_data)
{
  SPDR = spi_data;
  while (!SPI_FINISHED){} 
} 


byte RAM_read()
{
    SPDR = EMPTY;
    while (!SPI_FINISHED){} 
    return SPDR;
} 

void RAM_init()
{
    RAM_CS(LOW);
	RAM_CS_DIR(OUT);

    RAM_WP(HIGH);
	RAM_WP_DIR(OUT);

    RAM_HOLD(HIGH);
	RAM_HOLD_DIR(OUT);
}

void RAM_test_write()
{	
	RAM_CS(LOW);
	RAM_write(WREN); // Write Enable
	RAM_CS(HIGH);
	RAM_CS(LOW);
	RAM_write(WRITE); // Write
	RAM_write(FIRST); // first address in the memory
	RAM_write(brightness);
    RAM_CS(HIGH);
}

void RAM_test_read()
{		
	RAM_CS(LOW);
	RAM_write(READ); // Read
	RAM_write(FIRST); // first address in the memory
	brightness = RAM_read();
    RAM_CS(HIGH);
}


void buttons_init()
{
	UP_BUTTON_DIR(IN); //Set all the BUTTONs I/Os as input.
	DOWN_BUTTON_DIR(IN);
	LEFT_BUTTON_DIR(IN);
	RIGHT_BUTTON_DIR(IN);
	F_A_BUTTON_DIR(IN);
	F_B_BUTTON_DIR(IN);
	F_C_BUTTON_DIR(IN);

	UP_BUTTON_PULLUP(ON); //Set all the BUTTON's internal pullup resistors
	DOWN_BUTTON_PULLUP(ON);
	LEFT_BUTTON_PULLUP(ON);
	RIGHT_BUTTON_PULLUP(ON);
	F_A_BUTTON_PULLUP(ON);
	F_B_BUTTON_PULLUP(ON);
	F_C_BUTTON_PULLUP(ON);
}

void SPI_MasterInit(void)
{
    // Set MOSI, SCK and SS port of uC as output and MISO as input
    // (it sets SS high to make sure uC operates as Master)

	MU_SS_DIR(OUT);
	MU_SS(HIGH);                // Setting uC as a Master

    SCK_DIR(OUT);
    MOSI_DIR(OUT);
    MISO_DIR(IN);

    // Enable SPI, Master, set clock rate fck/16
    SPCR |= (TRUE<<SPE)|(TRUE<<MSTR);
	SPSR |= (TRUE<<SPI2X);
}



void LCD_command_tx(byte cData)
{
	LCD_CS(LOW);
	LCD_CD(COMMAND);
    // Start transmission
    SPDR = cData;
    // Wait for transmission complete
    while (!SPI_FINISHED);
	LCD_CS(HIGH);
}

void LCD_data_tx(byte cData)
{
	LCD_CS(LOW); 
	LCD_CD(DATA);
    // Start transmission
    SPDR = cData;
    // Wait for transmission complete
    while (!SPI_FINISHED);
	LCD_CS(HIGH);
}

byte select_page (byte page)
{	
	if (page>MAX_PAGES-1)
	{
		page = MAX_PAGES-1;
	}
	byte page_cmd_address;
	page_cmd_address =(CMD_PAGE | page);
	LCD_command_tx(page_cmd_address );
	return(TRUE);
}

byte select_column (byte column)
{	
	if (column>MAX_COLUMNS-1)
	{
		column = MAX_COLUMNS-1;
	}

	byte page_cmd_address_MSB;
	byte page_cmd_address_LSB;
	page_cmd_address_LSB =(CMD_COL_LSB | (column & LSHALF)); // LSHALF - Least significant half
	page_cmd_address_MSB =(CMD_COL_MSB | (column >> HALFBIT));
	LCD_command_tx(page_cmd_address_LSB);
	LCD_command_tx(page_cmd_address_MSB);
	return(TRUE);
}

void color_screen (byte color)
{	
	byte page;
	
	for (page = 0; page < MAX_PAGES; page++)
	{	
		byte column;
		
		for (column = 0; column<MAX_COLUMNS; column++)
		{
			select_page(page);
			select_column(column);
			LCD_data_tx(color);
		}
	}
}

void draw_screen()
{	
	byte page;
	
	for (page = 0; page < MAX_PAGES; page++)
	{	
		byte column;
		
		for (column = 0; column<MAX_COLUMNS; column++)
		{
			select_page(page);
			select_column(column);
			LCD_data_tx(frame_buffer[column][page]);
		}
	}
}

void draw_pixel (byte _row, byte _column, byte _color)

{	
	page = _row/8;
	pixel = _row%8;
	select_page(page);
	select_column(_column);
	if ((_color!=WHITE)){
		pixel = (_BV(pixel) | (frame_buffer[_column][page]));
		frame_buffer[_column][page] = pixel;
	}
	else if (_color==WHITE){
		pixel = (~(_BV(pixel)) & (frame_buffer[_column][page]));
		frame_buffer[_column][page] = pixel;
	}
	LCD_data_tx(pixel);
}


void draw_init_pads(byte _hor_pad, byte _vert_pad)
{	

	
	byte i;
	byte j;
	byte width=16;
	

	

	if ((_hor_pad+width<100) && (_hor_pad-width>=2) && (_vert_pad-width>0) && (_vert_pad+width<=62))
	{
		if(_hor_pad!=hor_pad)
		{
			for (i=0;i<2;i++)
			{
				for (j=0; j<width*2+1; j++)
				{
					draw_pixel(i,_hor_pad-width+j,BLACK);
					draw_pixel(62+i,_hor_pad-width+j,BLACK);
				}
			}

		}

		if(_vert_pad!=vert_pad)
{
			for (i=0;i<2;i++)
			{
				for (j=0; j<width*2+1; j++)
				{
					draw_pixel(_vert_pad-width+j,i,BLACK);
					draw_pixel(_vert_pad-width+j,i+100,BLACK);
				}
			}

		}
		hor_pad = _hor_pad;
		vert_pad = _vert_pad;
	}
}





void draw_pads(byte _hor_pad, byte _vert_pad)

{	

	
	byte i;
	byte j;
	
	

	

	if ((_hor_pad+width<100) && (_hor_pad-width>=2) && (_vert_pad-width>0) && (_vert_pad+width<=62))
	{
		if(_hor_pad!=hor_pad)
		{
			if(_hor_pad>hor_pad)
			{
				draw_pixel(0,_hor_pad+width,BLACK);
				draw_pixel(1,_hor_pad+width,BLACK);
				draw_pixel(62,_hor_pad+width,BLACK);
				draw_pixel(63,_hor_pad+width,BLACK);

				draw_pixel(0,hor_pad-width,WHITE);
				draw_pixel(1,hor_pad-width,WHITE);
				draw_pixel(62,hor_pad-width,WHITE);
				draw_pixel(63,hor_pad-width,WHITE);
			}
			else 
			{
				draw_pixel(0,_hor_pad-width,BLACK);
				draw_pixel(1,_hor_pad-width,BLACK);
				draw_pixel(62,_hor_pad-width,BLACK);
				draw_pixel(63,_hor_pad-width,BLACK);

				draw_pixel(0,hor_pad+width,WHITE);
				draw_pixel(1,hor_pad+width,WHITE);
				draw_pixel(62,hor_pad+width,WHITE);
				draw_pixel(63,hor_pad+width,WHITE);
			}

		}

		if(_vert_pad!=vert_pad)
		{
			if(_vert_pad>vert_pad)
			{
				draw_pixel(vert_pad+width,0,BLACK);
				draw_pixel(vert_pad+width,1,BLACK);
				draw_pixel(vert_pad+width,100,BLACK);
				draw_pixel(vert_pad+width,101,BLACK);

				draw_pixel(vert_pad-width,0,WHITE);
				draw_pixel(vert_pad-width,1,WHITE);
				draw_pixel(vert_pad-width,100,WHITE);
				draw_pixel(vert_pad-width,101,WHITE);
			}
			else 
			{
				draw_pixel(vert_pad-width,0,BLACK);
				draw_pixel(vert_pad-width,1,BLACK);
				draw_pixel(vert_pad-width,100,BLACK);
				draw_pixel(vert_pad-width,101,BLACK);

				draw_pixel(vert_pad+width,0,WHITE);
				draw_pixel(vert_pad+width,1,WHITE);
				draw_pixel(vert_pad+width,100,WHITE);
				draw_pixel(vert_pad+width,101,WHITE);
			}


		}		

		hor_pad = _hor_pad;
		vert_pad = _vert_pad;
	}
}


void draw_ball(byte _row_ball, byte _col_ball)


{	
	byte i;
	byte j;

	for (i=0;i<5;i++)
	{
		for (j=0;j<5;j++)
		{
			draw_pixel(row_ball-2+i,col_ball-2+j,WHITE);
		}
	}
	
	if ((_col_ball+2<103) && (_col_ball-2>=0) && (_row_ball-2>=0) && (_row_ball+2<63))
	{		
		for (i=0;i<5;i++)
		{
			for (j=0;j<5;j++)
			{
				byte ball_pixel = (ball_array[j] & ( 1 << i )) >> i; // reading ball pixel from the ball array
				draw_pixel(_row_ball-2+i,_col_ball-2+j,ball_pixel);
			}
		}
		row_ball=_row_ball;
		col_ball=_col_ball;

	}
}


void restart()
{
	int i = 0;
	for (i=0; i<8; i++)
	{
		color_screen(BLACK);
		_delay_ms(100);
		color_screen(WHITE);
		_delay_ms(100);
	}
	draw_ball(31,52);
	draw_screen();
}

void move_ball()

{


	int compar = 0;
	if (row_ball <= 4 || row_ball >= 59)
	{	
		if(((hor_pad>col_ball)?hor_pad-col_ball:col_ball-hor_pad)<width)
			{dir_y=-dir_y;}
		else restart();
	}

	if (col_ball <= 4 || col_ball >= 97)
	{
		if(((vert_pad>row_ball)?vert_pad-row_ball:row_ball-vert_pad)<width)
			{dir_x=-dir_x;}
		else restart();
	}

	draw_ball(row_ball+dir_y, col_ball+dir_x);


}

void move_pads(byte _move_buff)

{
	byte hor_dir = 0x00;
	byte ver_dir = 0x00;
	if (_move_buff & (1 << 0))
	{
		if (_move_buff & (1 << 1)) ver_dir = -1;
		else ver_dir =1;
	}
	if (_move_buff & (1 << 2))
	{
		if (_move_buff & (1 << 3)) hor_dir = -1;
		else hor_dir =1;
	}
	
	draw_pads(hor_pad+hor_dir, vert_pad+ver_dir);
}




void key_check()
{		
		
			byte move_buff = 0x00;
		    if (UP_BUTTON)
			{
			//	row--;
				move_buff |= ((1 << 0) | (1 << 1));
			}
			if (DOWN_BUTTON)
			{
			//	row++;
				move_buff |= (1 << 0);
			}
			if (LEFT_BUTTON) 
			{
			//	column--;
				move_buff |= ((1 << 2) | (1 << 3));
			}
		
			if (RIGHT_BUTTON)
			{
			//	column++;
				move_buff |= (1 << 2);

			}
		
			if (F_A_BUTTON)
			{
				BAT_LOW_LED(~BAT_LOW_LED_GET);
			};
		
		
			if (F_B_BUTTON)
			{
			    brightness += 1;	 
			}
		
		
			if (F_C_BUTTON)
			{
		    	brightness -= 1;
			}
		
			if (move_buff) move_pads(move_buff);
		/*	if (row > 63) row = 63;
			else if (row < 0) row = 0;
			if (column > 101) column = 101;
			else if (column < 0) column = 0;
			draw_pixel (row, column, BLACK);*/
			_delay_ms(120);
			OCR2 = brightness;
			RAM_test_write();
		
}





void interrupts_init()
{
	MCUCR=(1<<ISC00); // Trigger INT0 
	GICR=(TRUE<<INT0);  // Enable INT0
	sei();
	// ISC00 : Low level of INTx generates an interrupt request // przerwanie wystêpuje podczas wciœniêcia lub puszczenia klawisza
	// ISC01 : Any logic change on INTx generates an interrupt request // puszczenie klawisza wywo³uje przerwanie
	// ISC10 : The falling edge of INTx generates an interrupt request // przerwanie ca³y czas, wciœniêcie je wy³¹cza
	// ISC11 : The rising edge of INTx generates an interrupt request // przerwanie ca³y czas, wciœniêcie je wy³¹cza
}

byte LCD_initialise(void)
{
    LCD_CS_DIR(OUT);
	LCD_CD_DIR(OUT);
  

    LCD_RST_DIR(OUT);
    LCD_RST(LOW);               // RESET routine for the LCD
    _delay_ms(1);
    LCD_RST(HIGH);
    _delay_ms(5);


    LCD_command_tx(0x40);       //Display start line 0
    LCD_command_tx(0xA1);       //SEG reverse
    LCD_command_tx(0xC0);       //Normal COM0~COM63
    LCD_command_tx(0xA4);       //Disable -> Set All Pixel to ON
    LCD_command_tx(0xA6);       //Display inverse off
    _delay_ms(120);
    LCD_command_tx(0xA2);       //Set LCD Bias Ratio A2/A3
    LCD_command_tx(0x2F);       //Set Power Control 28...2F
    LCD_command_tx(0x27);       //Set VLCD Resistor Ratio 20...27
    LCD_command_tx(0x81);       //Set Electronic Volume
    LCD_command_tx(0x09);       //Set Electronic Volume 00...3F
    LCD_command_tx(0xFA);       //Set Adv. Program Control
    LCD_command_tx(0x90);       //Set Adv. Program Control x00100yz yz column wrap x Temp Comp
    LCD_command_tx(0xAF);       //Display on
    return (TRUE);
}




void pwm_init()
{
	    // initialize TCCR2 as per requirement, say as follows
    TCCR2 |= (TRUE<<WGM20)|(TRUE<<COM21)|(TRUE<<WGM21)|(TRUE<<CS20);
 
}

void timer1_init()
{
    // set up timer with prescaler = 64 and CTC mode
    TCCR1B |= (1 << WGM12)|(1 << CS11)|(1 << CS10);
 
    // initialize compare value
    OCR1A = 16000;
 
    // enable compare interrupt
    TIMSK |= (1 << OCIE1A);
 
}
 
 
void adc_init()
{
   ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Set ADC prescalar to 128 - 63KHz sample rate @ 16MHz

   ADMUX |= (1 << REFS0)|(1 << REFS1);  // Set ADC reference to internal
   ADMUX |= (1 << ADLAR); // Left adjust ADC result to allow easy 8 bit reading

   // No MUX values needed to be changed to use ADC0

   ADCSRA |= (1 << ADATE);  // Set ADC to Free-Running Mode

   ADCSRA |= (1 << ADEN);  // Enable ADC
   ADCSRA |= (1 << ADSC);  // Start A2D Conversions 
};


int main(void)
{	// Setting interrupts



	BAT_LOW_LED(OFF);						 //Make sure it is off before changing direction
	BAT_LOW_LED_DIR(OUT);					 //Set BATTERY LED I/Os as outputs.

	LCD_BACK_LIGHT_DIR(OUT);				 //Make sure it is off before changing direction
	LCD_BACK_LIGHT(OFF);    					//Set BATTERY LED I/Os as outputs.

	buttons_init();
	
    SPI_MasterInit();
    LCD_initialise();

	color_screen(WHITE);
	RAM_init();
	RAM_test_read();

	
	interrupts_init();
	pwm_init();
	draw_init_pads(52,31);
	draw_ball(31,52);
	timer1_init();
	adc_init();

/*
// This naive battery voltage meter does might require callibration with voltage supply in place of batteries.
	
	byte battbuff =0;
	byte i;
	for (i=0; i<10; i++)
	{
		if(ADCH < 130)
		{	
			_delay_ms(120);
			battbuff +=1;
		}
	}

	if (battbuff>=5)	BAT_LOW_LED(ON);
*/

    while (TRUE)                //Master loop always true so always loop
	{		
		if(key_status) key_check();
    }

}



ISR(INT0_vect)
{
	key_status = ~key_status;
} 
/*
ISR(TIMER2_OVF_vect)
{
    pwm();
	LCD_BACK_LIGHT(OFF);
}
*/

ISR (TIMER1_COMPA_vect)
{
move_ball();
}

