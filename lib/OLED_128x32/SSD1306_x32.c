                /*SSD1306 128x32 OLED Display library, via I2C communication
                            Written by r0tary (G.R), student at SDU 
        
        Version 1. released 25.05.2022.
             Feutures include:
                - Turn on any pixel(draw_Pixel), on specified X and Y coordinates
                - Draw/Print ASCII characters(draw_Char), on specified coordinates
                - Print out a string of characters (print_String), on specified coordinates
                - Print an integer(print_int) or float(print_float) type variable on the display
                - Draw horizontal and vertical lines wherever specified in a specified width
                - Draw bitmapped images(draw_Bitmap) on the display from a bitmap array. Image size and coordinate can be specified in function call  

---------------------------------------------------------------------------------------
                                Instructions of use:
                -Initiliazie I2C(i2c_init), Initiliaze SSD1306(SSD1306_setup)
                -Clear and update the display using SSD1306_clear and SSD1306_update
                -Grids* are turned OFF by default. Grids divide the usable display in 128/5 and 32/8. Used so vertically no characters will overlap.
If grids are on it is just like a 25x4 LCD screen. When writing address with grids on you specify the grid not tyhe coordinate...
...i.e. draw char at x coordinate 40 and 16 y coordinate would be draw_Char(0xhex,5,2).
                - After using drawing/printing functions use SSSD1306_update to push the pixels to the display.
                - Use SSD1306_clear to clear all pixels from the display (all pixels set to 0)
                - When using draw_Char specify the ASCII code of the character, use the font.h file, it has them listed

Disclaimer: Bugs during use could happen!
----------------------------------------------------------------------------------------
*/
#include <string.h>
//need to use it for memset(str, value, size)
#include <util/delay.h> 
#include <math.h>

//included header files
#include "SSD1306_x32.h"
#include "i2cmaster.h"
#include "font.h"
#include "bitmap.h"

//global variables
unsigned char ssd1306_write = 0b01111000; //I2C address, write always ends in 0
unsigned char ssd1306_read = 0b01111001;  //I2C adress, read always ends in 1
unsigned char ssd1306_buffer[512];        //64*32/8. Every bit is a pixel
uint8_t grid;


void SSD1306_setup(void){   

    SSD1306_command(SSD1306_DISPLAYOFF);//Display off

    SSD1306_command(SSD1306_SETDISPLAYCLOCKDIV);//Clock Divide Ratio
    SSD1306_command(0x80);//105hZ. Oscillator frequency

    SSD1306_command(SSD1306_SETMULTIPLEX);//Set Multiplex Ratio
    SSD1306_command(0x1F);//32 - 1

    SSD1306_command(SSD1306_SETDISPLAYOFFSET);//Display Offset
    SSD1306_command(0x00);//COM0, start line is 0

    SSD1306_command(SSD1306_SETSTARTLINE);//display start line

    SSD1306_command(SSD1306_CHARGEPUMP);//Set Charge Pump
    SSD1306_command(0x14);//Turn on charge pump

    SSD1306_command(SSD1306_ADDRESSING);//Addressing mode
    SSD1306_command(0x00);//0x01 - vertical. 0x00 - horizontal, 0x02 - page mode
    
    SSD1306_command(SSD1306_SEGREMAP);//segment remap a0/a1

    SSD1306_command(SSD1306_COMSCANDEC);//Set Com Output Scan Direction
                                        //c0: scan dir normal, c8: reverse

    SSD1306_command(SSD1306_SETCOMPINS);//Set COM Pins Hardware Configuration 
    SSD1306_command(0x02);//could depend on manufacturer/hardware layout

    SSD1306_command(SSD1306_SETCONTRAST); //Contrast Control
    SSD1306_command(0x8F);

    SSD1306_command(SSD1306_SETPRECHARGE);//Set Pre-Charge period
    SSD1306_command(0xF1);

    SSD1306_command(SSD1306_SETDECOM);//Set Vcomh deselct level
    SSD1306_command(0x40);

    SSD1306_command(0x2E);//Deactivate scroll

    SSD1306_command(SSD1306_DISPLAYALLON_RESUME);//Entire display on
    SSD1306_command(SSD1306_NORMALDISPLAY);//Set normal display
    SSD1306_command(SSD1306_DISPLAYON);//Display ON

    grid_status(OFF);
}

//I2C communication
void SSD1306_command(unsigned char c){
    i2c_start(ssd1306_write); //Starts I2C communication and sends slave adress
    i2c_write(0x00);//bit 7 is 0 for Co bit (data bytes only), bit 6 is 0 for DC (data is a command)
    i2c_write(c);//send register or the value to set it as
    
    i2c_stop();//Stop I2C communication
}

//Push the pixels from the buffer to the display
void SSD1306_update(){
    
    SSD1306_command(SSD1306_COLUMNADDR);//Register to specifiy collumn start address
    SSD1306_command(0);                 //First collum
    SSD1306_command(127);               //last collum
    SSD1306_command(SSD1306_PAGEADDR);  //Register to specify page start adress
    SSD1306_command(0);                 //First page
    SSD1306_command(3);                 //last page
    
    
    unsigned short pixel_count = 512;//128*32/8      
    unsigned char * ptr = ssd1306_buffer; //first address of the pixel buffer
    
    i2c_start(ssd1306_write);
    i2c_write(0x40); //specifies that the following stream of bytes till I2C stops is data
     
    while(pixel_count--){
        i2c_write(*ptr++);
    }
    i2c_stop();  
}


//Set pixel value to either 1 or 0(on or off)
void draw_Pixel(unsigned char x, unsigned char y, unsigned char state){
    if((x < 0) || (x >= 128) || (y < 0) || (y >= 32)){
        return;//used if pixel that you want to turn on is out of the resolution, does nothing.
    }
    int16_t n;//place in the buffer
    n = x + (y>>3) *128;//finds which pixel it is in the array buffer
    
    if(state == 1){//pixel on
        ssd1306_buffer[n] |= (1 << (y % 8));//finds correct page(byte) based on Y coordinate, shift 1 to set
    }
    else{//pixel off
        ssd1306_buffer[n] &= ~(1 << (y % 8));
    }
}

//Draws ASCII character on screen
void draw_Char(uint8_t ASCII_Nr, uint8_t x, uint8_t y){
    char state;//equals either 1 or 0 (ON or OFF)

    //Checks if the screen is divided in to grids
    if (grid == ON){
        x = x*5;
        y = y*8;
    }
    
    //collum = character width, row = character height 
    for(uint8_t collum=0;collum<5;collum++){
        for(uint8_t row=0;row<8;row++){
            
            //finds correct character based on char value, finds corrct collum, shifts bit to then bitwise AND with 1
            state = (ASCII[ASCII_Nr-32][collum]>>row) & 1;
            draw_Pixel(x+collum,y+row,state);

        }
    }
}

//Draws string of ASCII characters
void print_String(char str[], uint8_t x, uint8_t y){
    uint8_t i = 0;//array element number
    int8_t increment;//how much to increment the X coordinate by, used for grids

    if (grid == 0){//grid check
        increment = 5;
    }else increment = 1;

    //loop executes till null character is met
    while(str[i] != '\0'){
        draw_Char(str[i++],x,y);
        x  = x + increment;
    }
}

//Prints integer numbers on screen. Prints starting from the right most digit 
void print_int(short num, uint8_t x, uint8_t y){
    uint8_t num_length;//how many digits are in the number, used to correctly get the X coordinate

    if(num<0){//checks if the number is negative
        draw_Char(13,x,y);
        x++;
        num = num*(-1);
    }
    
    num_length = int_length(num); 

    if(num>9){
        do{//prints digits, starts from the right most digit
            num_length--;
            draw_Char(0x30+(num%10),x+num_length,y);
            num/=10;//gets rid  of the right most digit
        }while(num_length != 0);

    }else if (num<10) draw_Char(0x30+num,x,y);//if the number is only one digit
}

//Prints floating point numbers. The function has two loops, one for printing the digits after the decimal point...
//...the second loop prints digits that are after the decimal point . Prints starting from the right most digit
void print_float(float num, uint8_t significant_digits, uint8_t x, uint8_t y){
    int placeholder;//used as placeholder, to not change the value of 'num', incase it is needed after this function 
    uint8_t num_length;
    
    if(num<0){//checks if number is negative
        draw_Char(13,x,y);
        x++;
        num = num*(-1);
    }
    
    num_length = int_length((int)num);//takes length of the number before the decimal point, to figure out where to print the decimal point 
    draw_Char(0x2e,x+num_length,y);//prints the decimel point
    
    num = num*pow(10,significant_digits);//gets the full number that needs to be printed
    placeholder = (int)num;//gets the full number as an integer, gets rid of everything after the floats decimal point

    //loop prints the digits that are after the decimal point
    while(significant_digits>0){
        draw_Char(0x30+(placeholder%10), x+num_length+significant_digits,y);
        placeholder = placeholder / 10;
        significant_digits--;

    }
    
    //loop prints the numbers that after the decimal point, prints from the right most digit
    while(placeholder>0){
        draw_Char(0x30+(placeholder%10),x+num_length-1,y);
        num_length--;
        placeholder = placeholder / 10;
    }
}

//A function used by "print_int" and "print_float". Gets the number of digits an number has
int8_t int_length(int num){
    int placeholder = num;
    uint8_t counter = 0;//counts digits

    do{//counts digits
        placeholder/=10;
        counter++;
    }while(placeholder !=0);

    return counter;
}

//Draws bit map images using a defined array in the "font.h" header file
void draw_BitMap(char bitmap[], uint8_t map_width, uint8_t map_height, char mode[], uint8_t x_cord, uint8_t y_cord){
    int8_t state;//Can be either 1 or 0, used for "draw_Pixel"
    int x;//keeps track of the X coordinate/image width 

    //loop length depends on the height of the bit map
    for(uint8_t y=0;y<map_height;y++){
        x=0;
        
        //loop goes through every byte on a row
        for(uint8_t byte = 0;byte<(map_width/8);byte++){
            
            //goes through each bit in a byte
            for(uint8_t bits=0;bits<8;bits++){
                
                if (strcmp( mode, "mirror") == 0){//"strcmp" is sting compare, checks if the byte needs to be mirrored 
                    state = (reverseByte(bitmap[y*(map_width/8)+byte])>>(bits)) & 1;}
                else state = bitmap[y*(map_width/8)+byte] >> (bits) & 1;
                
                draw_Pixel(x_cord+x*8+(bits),y+y_cord,state);

            }
            x++;
        }
    }
}

//Reverses bit order in a byte used if Bitmap is mirrored, used by "Bitmap" function
uint8_t reverseByte(char byte){
    uint8_t num_of_bits; 
    uint8_t reverse_byte = 0;
    
    num_of_bits = sizeof(byte) * 8;

    for (int8_t i = 0; i < num_of_bits; i++) {
        if ((byte & (1 << i)))
            reverse_byte |= 1 << ((num_of_bits - 1) - i);
    }
    return reverse_byte;
}

//Draw a horizontal line from x1 to x2
void draw_hline(uint8_t x1, uint8_t x2, uint8_t y, uint8_t width){
    
    while(x1<x2){
        for(uint8_t i=0;i<width;i++){
            draw_Pixel(x1,y+i,1);
        }
        x1++;
    }
}

//Draw a vertical line from y1 to y2
void draw_vline(uint8_t x, uint8_t y1, uint8_t y2, uint8_t width){

    while(y1<y2){
        for(uint8_t i=0;i<width;i++){
            draw_Pixel(x+i,y1,1);
        }
        y1++;
    }
}

//Divides screen in to grids, 
void grid_status(uint8_t x){
    
    if((x>1) || (x<0)){
        return;
    }
    grid = x; 
}

//urns on every pixel on the screen (sets everything to 1)
void SSD1306_AllOn(){
    
    for(uint8_t y = 0;y < 4;y++){
        for(uint8_t x = 0; x < 128; x++){
            draw_Pixel(x,y,1);
        }
    }
}

//Sets every pixerl to 0(OFF)
void SSD1306_clear(){
    memset(ssd1306_buffer,0,512);//make every bit a 0
}

/*
void OLED_print(char string[], uint8_t x, uint8_t y){
    char text[25];
    
    strcpy(text, string);
    print_String(text,x,y);
}
*/

