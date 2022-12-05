/*SSD1306 128x32 OLED Display library, via I2C communication
    Written by Gustavs Riekstins
        Version 1. released 25.05.2022.
            Feutures include:
                - Turn on any pixel(draw_Pixel), on specified X and Y coordinates
                - Draw/Print ASCII characters(draw_Char), on specified coordinates
                - Print out a string of characters (print_String), on specified coordinates
                - Print an integer(print_int) or float(print_float) type variable on the display
                - Draw horizontal and vertical lines wherever specified in a specified width
                - Draw bitmapped images(draw_Bitmap) on the display from a bitmap array. Image size and coordinate can be specified in function call
                - Able to enable grids for easier string printing  
*/

#ifndef SSD1306_x32_H__ //checks if macro is not defined by #define. If yes, it executes the code otherwise #else code is executed, if present.
#define SSD1306_x32_H__

//Used for I2C communication
    #define SSD1306_SUCCESS 0
    #define SSD1306_ERROR 1

//OLED screen size definitions(pixels)
//*will later be used when functionality with different sized displays will be implemented
    //#define screen_width 128
    //#define screen_height 32

//For grids function
    #define ON 1
    #define OFF 0
    #define Check 2

//Used for initialization of the SSD1306 driver, register/command definitions
//...for some commands you also have to send a second byte.
//-----------------------------------------
    #define SSD1306_DISPLAYOFF           0xAE//Display off
    #define SSD1306_SETDISPLAYCLOCKDIV   0xD5//Display Clock Divide Ratio, sets the divide ratio to generate DCLK from CLK
    #define SSD1306_SETMULTIPLEX         0xA8//Set Multiplex Ratio, ranging from 16 to 63
    #define SSD1306_SETDISPLAYOFFSET     0xD3//Set Display Offset. specifies the mapping of the display start line
    #define SSD1306_SETSTARTLINE         0x40//Set Display Start Line. Sets the Display Start Line register to determine starting address of display RAM
    #define SSD1306_CHARGEPUMP           0x8D//Set Charge Pump, supply voltage to the OLED driver block.
    #define SSD1306_SEGREMAP             0xA0//Set Segment Re-map. Changes the mapping between the display data column address and the segment driver.
    #define SSD1306_COMSCANDEC           0xC0//Set COM Output Scan Direction, sets the scan direction.
    #define SSD1306_SETCOMPINS           0xDA//Set COM Pins Hardware Configuration, sets the COM signals pin configuration to match the OLED panel hardware layout.
    #define SSD1306_SETCONTRAST          0x81//Set contrast
    #define SSD1306_SETPRECHARGE         0xD9//Set Pre-Charge Period, sets the duration of the pre-charge period.
    #define SSD1306_SETDECOM             0xDB//Set VCOMh Deselect level, adjusts its regulator output
    #define SSD1306_DISPLAYON            0xAF//Turns on Display
//-----------------------------------------

//display registers
#define SSD1306_DISPLAYALLON_RESUME      0xA4//set entire display on
#define SSD1306_NORMALDISPLAY            0xA6//Set normal display, A7 would be inverse

#define SSD1306_ADDRESSING               0x20//Set Memory Addressing mode. Sets the way of memory addresing.
//0x01 is vertical. 0x00 is horizontal. 0x02 is page 

//specifies column start address and end address of the display data RAM...
//...also sets the column address pointer to column start address
#define SSD1306_COLUMNADDR           0x21

//Page start address and end address of the display data RAM... 
//...This command also sets the page address pointer to page start address.
#define SSD1306_PAGEADDR             0x22

//Function prototypes/definitions
    void SSD1306_setup(void);         //Initialize SSD1306 driver
    void SSD1306_update(void);        //Push every pixel status to the screen
    void SSD1306_clear(void);         //Sets every pixel value to 0(screen black)
    void SSD1306_AllOn(void);

//takes X and Y coordinates and sets their value in the buffer array to either 1 or 0(state)
    void draw_Pixel(uint8_t x, uint8_t y, uint8_t state);
    
/*Functions related to drawing on screen
**Disclaimer: Remember to take into account the font size of characters**
-----------------------------------------------------------------------
    Draws ASCII symbols
    Input: ASCII_Nr - hex code of ASCII charcter to be drawn, use the array in font.h to see values 
            x - the X coordinates of where to draw character 
            y - the Y coordinates of where to draw the character
    */void draw_Char(uint8_t ASCII_Nr, uint8_t x, uint8_t y);

    /*Draw a String of characters that consist of ASCII symbols
    Input: str[] - String of characters, use " "
           x - the X coordinates of where to start print of string
           y - the Y coordinates of where to print string of characters 
    */void print_String(char str[], uint8_t x, uint8_t y);

    /*Draw horizontal line
    Input: x1 - start X coordinate
           x2 - end X coordinate
           y - Y coordinate
           width - the width of the line
    */void draw_hline(uint8_t x1, uint8_t x2, uint8_t y, uint8_t width);

    /*Draws vertical line
    Input: x - the X cordinate of the line
           y1 - the start Y coordinate of the line
           y2 - the end Y coordinate of the line
    */void draw_vline(uint8_t x, uint8_t y1, uint8_t y2, uint8_t width);

    /*Draws BitMap image
    Input:
    */void draw_BitMap(char bitmap[], uint8_t map_width, uint8_t map_height, char mode[], uint8_t x_cord, uint8_t y_cord);
//-----------------------------------------
        
/*Functions related to printing numbers on the display
**Disclaimer: Remember to take into account the font size of characters**
-------------------------------------------------------------------------
    Prints integers on screen.
    Input: number - the integer to print
           x - the X coordinates of where to start print of number
           y - the y coordinates of where to print number
    */void print_int(short number, uint8_t x, uint8_t y);

    /*Prints float on screen
    Input: number - the float number to be printed
           x - the X coordinates of where to start print
           y - the Y coordinates pf where to print float 
    */
    void print_float(float num, uint8_t significant_digits, uint8_t x, uint8_t y);
//-----------------------------------------
        
/*Divides screen in to grids, so characters vertically wouldnt overlap each other
Max characters using 5x8: Horizontally ist 25, vertically 4. 
    Inputs: ON(1) - turns on grids, returns nothing. OFF(0) - turns off grids, returns nothing...
    ...Status(2) - returns what it is currently set to, ON or OFF (1 or 0).
    */void grid_status(uint8_t status);    

/*Reverses a byte. Used by the "draw_Bitmap" function. Used because, depending on where you get tyhe bitmap, there bytes could be reversed
    Inputs: byte - the byte that needs to be mirrored
    Output: returns the mirrored byte
*/uint8_t reverseByte(char byte);

/*Calculates how many digits are in a integer
    Input: num - the integer whoose length needs to be calculated
    Output: returns the lenghth of an integer
*/int8_t int_length(int num);

/*Command definition
    Input: c - a byte. Either to access register or to send value to register
    */void SSD1306_command(unsigned char c);

#endif