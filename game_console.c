#include "game_console.h" 

byte frame_buffer[MAX_COLUMNS][MAX_PAGES] = {0};
byte obstacle_buffer[32][2] = {0};
float vcc;
byte page = 0;
signed char column = 0;
signed char row = 0;
byte pixel = 0;
byte key_buffer = EMPTY;
byte obx = 21;
byte oby = 13;
byte obcnt =0;
byte brightness = 0;
byte key_status = 0x00;
byte width=16;
byte oc=0;
byte oc2=0;
byte limit = 40;
byte points = 0;
int compare_value = 10000;

byte ball_array[5]=
   {0b00001110,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00001110
   };


byte zero_array[5]=
   {0b00111110,
	0b01000101,
	0b01001001,
	0b01010001,
	0b00111110
   };
byte one_array[5]= 
   {0b00000000,
	0b01000010,
	0b01111111,
	0b01000000,
	0b00000000
   };
byte two_array[5]= 
   {0b01000010,
	0b01100001,
	0b01010001,
	0b01001001,
	0b01000110
    };
byte three_array[5]=
   {0b00100001,
	0b01000001,
	0b01000101,
	0b01001011,
	0b00110001
    };
byte four_array[5]=
   {0b00011000,
	0b00010100,
	0b00010010,
	0b01111111,
	0b00010000
    };
byte five_array[5]=
   {0b00100111,
	0b01000101,
	0b01000101,
	0b01000101,
	0b00111001
    };
byte six_array[5]=
   {0b00111100,
	0b01001010,
	0b01001001,
	0b01001001,
	0b00110000
    };
byte seven_array[5]=
   {0b00000001,
	0b01110001,
	0b00001001,
	0b00000101,
	0b00000011
    };
byte eight_array[5]=
   {0b00110110,
	0b01001001,
	0b01001001,
	0b01001001,
	0b00110110
    };
byte nine_array[5]=
   {0b00000110,
	0b01001001,
	0b01001001,
	0b00101001,
	0b00011110
    };

byte *numbers[10]={zero_array,one_array,two_array,three_array,four_array,five_array,six_array,seven_array,eight_array,nine_array};

byte vert_pad = 0;
byte hor_pad = 0;
byte row_ball = 100;
byte col_ball = 100;

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
	cli();
	LCD_CS(LOW);
	LCD_CD(COMMAND);
    // Start transmission
    SPDR = cData;
    // Wait for transmission complete
    while (!SPI_FINISHED);
	LCD_CS(HIGH);
	sei();
}

void LCD_data_tx(byte cData)
{	
	cli();
	LCD_CS(LOW); 
	LCD_CD(DATA);
    // Start transmission
    SPDR = cData;
    // Wait for transmission complete
    while (!SPI_FINISHED);
	LCD_CS(HIGH);
	sei();
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
	cli();
	select_page(page);
	select_column(_column);
	sei();
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
	
		
	if ((_hor_pad+width<100) && (_hor_pad-width>=2) && (_vert_pad-width>=0) && (_vert_pad+width<=63))
	{
		if(_hor_pad!=hor_pad)
		{
			for (i=0;i<2;i++)
			{
				for (j=0; j<=width*2; j++)
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
				for (j=0; j<=width*2; j++)
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



	if ((_hor_pad+width<100) && (_hor_pad-width>=2) && (_vert_pad-width>=0) && (_vert_pad+width<=63))
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

				
				draw_pixel(_vert_pad+width,0,BLACK);
				draw_pixel(_vert_pad+width,1,BLACK);
				draw_pixel(_vert_pad+width,100,BLACK);
				draw_pixel(_vert_pad+width,101,BLACK);

				draw_pixel(vert_pad-width,0,WHITE);
				draw_pixel(vert_pad-width,1,WHITE);
				draw_pixel(vert_pad-width,100,WHITE);
				draw_pixel(vert_pad-width,101,WHITE);
			}
			else 
			{
				draw_pixel(_vert_pad-width,0,BLACK);
				draw_pixel(_vert_pad-width,1,BLACK);
				draw_pixel(_vert_pad-width,100,BLACK);
				draw_pixel(_vert_pad-width,101,BLACK);

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

void draw_num(byte _number, byte _row_num, byte _col_num)
{	
	byte i;
	byte j;
	
	for (i=0;i<7;i++)
	{
		for (j=0;j<5;j++)
		{
			draw_pixel(_row_num-3+i,_col_num-2+j,WHITE);
			draw_pixel(_row_num-3+i,_col_num-9+j,WHITE);
			draw_pixel(_row_num-3+i,_col_num-16+j,WHITE);
		}
	}

	while(TRUE)
	{
		for (i=0;i<7	;i++)
		{
			for (j=0;j<5;j++)
			{
				byte num_pixel = (numbers[_number%10][j] & ( 1 << i )) >> i;
				draw_pixel(_row_num-3+i,_col_num-2+j,num_pixel);
			}
		}
		_col_num-=7;
		_number/=10;
		if (_number==0) break;
	}
}






void restart()
{
	cli();
/*	int i = 0;
	for (i=0; i<5; i++)
	{
		color_screen(BLACK);
		_delay_ms(100);
		color_screen(WHITE);
		_delay_ms(100);
	}*/
	UP_BUTTON_DIR(OUT); //Set all the BUTTONs I/Os as input.
	DOWN_BUTTON_DIR(OUT);
	LEFT_BUTTON_DIR(OUT);
	RIGHT_BUTTON_DIR(OUT);
	F_A_BUTTON_DIR(OUT);
	F_B_BUTTON_DIR(OUT);
	F_C_BUTTON_DIR(OUT);

	UP_BUTTON_PULLUP(OFF); //Set all the BUTTON's internal pullup resistors
	DOWN_BUTTON_PULLUP(OFF);
	LEFT_BUTTON_PULLUP(OFF);
	RIGHT_BUTTON_PULLUP(OFF);
	F_A_BUTTON_PULLUP(OFF);
	F_B_BUTTON_PULLUP(OFF);
	F_C_BUTTON_PULLUP(OFF);
	
	color_screen(WHITE);
	draw_num(points,30,40); //Raw reads:
	byte i=0;
	for (i=0;i<100;i++)
	{
		_delay_ms(100);
	};
	
}

void move_ball()

{


	int compar = 0;
	if (row_ball <= 4 || row_ball >= 59)
	{	
		if(((hor_pad>col_ball)?hor_pad-col_ball:col_ball-hor_pad)<width)
			{dir_y=-dir_y;
			points++;}
		else restart(); // if crashed with screen's edge
	}

	if (col_ball <= 4 || col_ball >= 97)
	{
		if(((vert_pad>row_ball)?vert_pad-row_ball:row_ball-vert_pad)<width)
			{dir_x=-dir_x;
			points++;}
		else restart(); // if crashed with screen's edge
	}

	byte i =0;
	for (i=0; i<32; i++)
	{
		if ((((obstacle_buffer[i][0]>=col_ball)?obstacle_buffer[i][0]-col_ball:col_ball-obstacle_buffer[i][0])<9) && (((obstacle_buffer[i][1]>=row_ball)?obstacle_buffer[i][1]-row_ball:row_ball-obstacle_buffer[i][1])<9) && row_ball!=0 && col_ball!=0)
		restart(); // if collision dedected
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
			if (row > 63) row = 63;
			else if (row < 0) row = 0;
			if (column > 101) column = 101;
			else if (column < 0) column = 0;
			draw_pixel (row, column, BLACK);
			_delay_ms(120);
			OCR2 = brightness;
			RAM_test_write();
		
}





void interrupts_init()
{
	MCUCR=(1<<ISC00); // Trigger INT0 
	GICR=(TRUE<<INT0);  // Enable INT0
	sei();
	// ISC00 : Low level of INTx generates an interrupt request // przerwanie wyst�puje podczas wci�ni�cia lub puszczenia klawisza
	// ISC01 : Any logic change on INTx generates an interrupt request // puszczenie klawisza wywo�uje przerwanie
	// ISC10 : The falling edge of INTx generates an interrupt request // przerwanie ca�y czas, wci�ni�cie je wy��cza
	// ISC11 : The rising edge of INTx generates an interrupt request // przerwanie ca�y czas, wci�ni�cie je wy��cza
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
    OCR1A = compare_value;
 
    // enable compare interrupt
    TIMSK |= (1 << OCIE1A);
 
}
 
 /*
void adc_init() //adc for battery
{
   ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Set ADC prescalar to 128 - 63KHz sample rate @ 16MHz

   ADMUX |= (1 << REFS0)|(1 << REFS1);  // Set ADC reference to internal
   ADMUX |= (1 << ADLAR); // Left adjust ADC result to allow easy 8 bit reading

   // No MUX values needed to be changed to use ADC0

   ADCSRA |= (1 << ADATE);  // Set ADC to Free-Running Mode

   ADCSRA |= (1 << ADEN);  // Enable ADC
   ADCSRA |= (1 << ADSC);  // Start A2D Conversions 
};
*/
/*
void touchscreen_init()
{
   TS_LEFT(ON);
   TS_RIGHT(OFF);
   TS_LEFT_DIR(OUT);
   TS_RIGHT_DIR(OUT);
   ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Set ADC prescalar to 128 - 63KHz sample rate @ 16MHz

   ADMUX |= (1 << REFS0);//|(1 << REFS1);  // Set ADC reference to internal
   ADMUX |= (1 << ADLAR); // Left adjust ADC result to allow easy 8 bit reading

   ADMUX |= (1 <<MUX0); // ADC1
   ADMUX |= (1 <<MUX0); // ADC1

   ADCSRA |= (1 << ADATE);  // Set ADC to Free-Running Mode

   ADCSRA |= (1 << ADEN);  // Enable ADC
   ADCSRA |= (1 << ADSC);  // Start A2D Conversions 
};
*/
byte touch_standby()
{
   TS_LEFT(OFF);
   TS_RIGHT(OFF);
   TS_TOP(OFF);
   TS_BOTTOM(ON);
   TS_LEFT_DIR(OUT);
   TS_RIGHT_DIR(IN);
   TS_TOP_DIR(IN);
   TS_BOTTOM_DIR(IN);
   _delay_ms(10);

   return GET_TS_BOTTOM;

}




byte get_touch_x()
{


   TS_TOP(OFF);
   TS_BOTTOM(OFF);
   TS_TOP_DIR(IN);
   TS_BOTTOM_DIR(IN);

   TS_LEFT(ON);
   TS_RIGHT(OFF);
   TS_LEFT_DIR(OUT);
   TS_RIGHT_DIR(OUT);
   ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Set ADC prescalar to 128 - 63KHz sample rate @ 16MHz

   ADMUX |= (1 << REFS0);//|(1 << REFS1);  // Set ADC reference to internal
   ADMUX |= (1 << ADLAR); // Left adjust ADC result to allow easy 8 bit reading

   ADMUX &= ~(1 <<MUX1); // ADC2 OFF
   ADMUX |= (1 <<MUX0)|(1 <<MUX1); // ADC1 ON

   ADCSRA |= (1 << ADATE);  // Set ADC to Free-Running Mode

   ADCSRA |= (1 << ADEN);  // Enable ADC
   ADCSRA |= (1 << ADSC);  // Start A2D Conversions 
   return ADCH;


}

byte get_touch_y()
{

   TS_LEFT(OFF);
   TS_RIGHT(OFF);
   TS_LEFT_DIR(IN);
   TS_RIGHT_DIR(IN);

   TS_TOP(ON);
   TS_BOTTOM(OFF);
   TS_TOP_DIR(OUT);
   TS_BOTTOM_DIR(OUT);


   ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Set ADC prescalar to 128 - 63KHz sample rate @ 16MHz

   ADMUX |= (1 << REFS0);//|(1 << REFS1);  // Set ADC reference to internal
   ADMUX |= (1 << ADLAR); // Left adjust ADC result to allow easy 8 bit reading

   ADMUX &= ~((1 <<MUX0)|(1 <<MUX1)); // ADC2 OFF
   ADMUX |= (1 <<MUX1); // ADC1 ON

   ADCSRA |= (1 << ADATE);  // Set ADC to Free-Running Mode

   ADCSRA |= (1 << ADEN);  // Enable ADC
   ADCSRA |= (1 << ADSC);  // Start A2D Conversions 
   return ADCH;


}

void draw_obstacle()
{
		if (oc<limit) oc++;
		else
		{
			byte z =0;
			for (z=0; z<32; z++)
			{
				if (obstacle_buffer[z][0]==0)
				{
					byte i=0;
					byte j=0;
					while(TRUE)
					{
						if ((((obx>=col_ball)?obx-col_ball:col_ball-obx)<12) && (((oby>=row_ball)?oby-row_ball:row_ball-oby)<12) && row_ball!=0 && col_ball!=0)
						{
							obx = rand()%86 + 10;
							oby = rand()%48 + 10;
						}
						else break;
					}
					compare_value-=200;
					OCR1A = compare_value;

					for (i =0; i<11; i++)
					{
						for (j =0; j<11; j++)
						{
							draw_pixel(oby-5+i,obx-5+j,BLACK);
						}
					}
					draw_screen();
					obstacle_buffer[z][0]=obx;
					obstacle_buffer[z][1]=oby;
	
					oc=0;
					limit = rand()%100+50;
					obx = rand()%86 + 10;
					oby = rand()%48 + 10;

					return;
				}
				else continue;
			}
			restart();

		}

}

void remove_obstacle(byte _id)
{
	byte locx = obstacle_buffer[_id][0];
	byte locy = obstacle_buffer[_id][1];
	byte i,j = 0;

	for (i =0; i<11; i++)
	{
		for (j =0; j<11; j++)
		{
			draw_pixel(locy-5+i,locx-5+j,WHITE);
		}
	}

	obstacle_buffer[_id][0]=0;
	obstacle_buffer[_id][1]=0;
	

}



void check_touch(byte _x, byte _y)
{
	byte i =0;
	for (i=0; i<10; i++)
	{
		if ((((obstacle_buffer[i][0]>_y)?obstacle_buffer[i][0]-_y:_y-obstacle_buffer[i][0])<9) && (((obstacle_buffer[i][1]>_x)?obstacle_buffer[i][1]-_x:_x-obstacle_buffer[i][0])<9))
		{
			remove_obstacle(i);
		}
	}
}


int main(void)
{	// Setting interrupts



	BAT_LOW_LED(OFF);						 //Make sure it is off before changing direction
	BAT_LOW_LED_DIR(OUT);					 //Set BATTERY LED I/Os as outputs.

	LCD_BACK_LIGHT_DIR(OUT);				 //Make sure it is off before changing direction
	LCD_BACK_LIGHT(ON);    					//Set BATTERY LED I/Os as outputs.

	buttons_init();
	
    SPI_MasterInit();
    LCD_initialise();

	color_screen(WHITE);
	RAM_init();
	RAM_test_read();

	
	interrupts_init();
	pwm_init();
	draw_init_pads(52,32);

	draw_ball(32,52);
	
	timer1_init();

	_delay_ms(120);
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
				//cli();
                if(touch_standby())
                {                  
                    BAT_LOW_LED(ON);
					_delay_ms(1);

					byte i = 0;
					int x = 0;
					int y = 0;

					for (i=0; i<3; i++)
					{
						x+= (get_touch_x()-30)/1.7;
						y+= (get_touch_y()-18)/2.1;
					//	x+= get_touch_x()/3;
					//	y+= get_touch_y()/3;
					}
					x/=3;
					y/=3;
				//	draw_num(x,20,40); //Raw reads:
				//	draw_num(y,40,40); //Raw reads: 
					_delay_ms(10);
				//	draw_pixel(x,y,BLACK);
					check_touch(x,y);
                }
				else
				{                  
                    BAT_LOW_LED(OFF);
				//	draw_num(0,20,40);
				//	draw_num(0,40,40);
                }
				//sei(); 


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
}*/

ISR (TIMER1_COMPA_vect)
{
move_ball();
draw_obstacle();
}
