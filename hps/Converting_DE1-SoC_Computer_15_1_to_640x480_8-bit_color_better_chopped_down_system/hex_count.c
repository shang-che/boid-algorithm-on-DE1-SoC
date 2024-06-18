///////////////////////////////////////
/// 640x480 version!
/// This code will segfault the original
/// DE1 computer
///////////////////////////////////////
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include "address_map_arm_brl4.h"
// resolution register
#define resOffset 0x00003028
#define statusOffset 0x0000302c

#define SWAP(X,Y) do{int temp=X; X=Y; Y=temp;}while(0) 

/* function prototypes */
void VGA_text (int, int, char *);
void VGA_box (int, int, int, int, short);
void VGA_line(int, int, int, int, short) ;

// virtual to real address pointers

volatile unsigned int * red_LED_ptr = NULL ;
volatile unsigned int * res_reg_ptr = NULL ;
volatile unsigned int * stat_reg_ptr = NULL ;
void *h2p_lw_virtual_base;

volatile unsigned int * vga_pixel_ptr = NULL ;
void *vga_pixel_virtual_base;

volatile unsigned int * vga_char_ptr = NULL ;
void *vga_char_virtual_base;

int fd;

int x1, y1, x2, y2;

int main(void)
{
	// Declare volatile pointers to I/O registers (volatile 	// means that IO load and store instructions will be used 	// to access these pointer locations, 
	// instead of regular memory loads and stores) 

  	
	// === need to mmap: =======================
	// FPGA_CHAR_BASE
	// FPGA_ONCHIP_BASE      
	// HW_REGS_BASE        
  
	// === get FPGA addresses ==================
    // Open /dev/mem
	if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) 	{
		printf( "ERROR: could not open \"/dev/mem\"...\n" );
		return( 1 );
	}
    
    // get virtual addr that maps to physical
	h2p_lw_virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_REGS_BASE );	
	if( h2p_lw_virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap1() failed...\n" );
		close( fd );
		return(1);
	}
    
    // Get the address that maps to the FPGA LED control 
	red_LED_ptr =(unsigned int *)(h2p_lw_virtual_base +  	 			LEDR_BASE);

	// address to resolution register
	res_reg_ptr =(unsigned int *)(h2p_lw_virtual_base +  	 			resOffset);

	 //addr to vga status
	stat_reg_ptr = (unsigned int *)(h2p_lw_virtual_base +  	 			statusOffset);

	// === get VGA char addr =====================
	// get virtual addr that maps to physical
	vga_char_virtual_base = mmap( NULL, FPGA_CHAR_SPAN, ( 	PROT_READ | PROT_WRITE ), MAP_SHARED, fd, FPGA_CHAR_BASE );	
	if( vga_char_virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap2() failed...\n" );
		close( fd );
		return(1);
	}
    
    // Get the address that maps to the FPGA LED control 
	vga_char_ptr =(unsigned int *)(vga_char_virtual_base);

	// === get VGA pixel addr ====================
	// get virtual addr that maps to physical
	vga_pixel_virtual_base = mmap( NULL, FPGA_ONCHIP_SPAN, ( 	PROT_READ | PROT_WRITE ), MAP_SHARED, fd, 			FPGA_ONCHIP_BASE);	
	if( vga_pixel_virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap3() failed...\n" );
		close( fd );
		return(1);
	}
    
    // Get the address that maps to the FPGA LED control 
	vga_pixel_ptr =(unsigned int *)(vga_pixel_virtual_base);

	// ===========================================

	/* create a message to be displayed on the VGA 
          and LCD displays */
	char text_top_row[40] = "Altera DE1-SoC\0";
	char text_bottom_row[40] = "Cornell ece5760\0";

	VGA_text (34, 29, text_top_row);
	VGA_text (34, 30, text_bottom_row);
	// clear the screen
	VGA_box (0, 0, 639, 479, 0);
	// x1 y1 x2 y2	320x240					
	//VGA_box (33*4, 28*4, 49*4, 32*4, 0x187F);
	//VGA_box (100, 210, 300, 220, 0xffe0);
	//VGA_line(10, 20, 100, 50, 0xf000) ;
	
	// get the y and x res
	// y in top 16, x in bottom 16 bits
	printf("%0x\n", (int)*res_reg_ptr );
	// m bits y is top8 -- n bits x is next8
	printf("%0x\n", (int)*stat_reg_ptr); 
	
	short color, toggle ;
	while(1)
	{
		//toggle = ~toggle;
		if(*(red_LED_ptr) == 0x1)
			*(red_LED_ptr) = 0x0;
		else
			*(red_LED_ptr) = 0x1;

		x1 = (int)((rand() & 0x2ff));
		y1 = (int)((rand() & 0x1ff));
		x2 = (int)((rand() & 0x2ff));
		y2 = (int)((rand() & 0x1ff));
		if (x1>639) x1 = 639;
		if (y1>479) y1 = 479;
		if (x2>639) x2 = 639;
		if (y2>479) y2 = 479;
		if (x1>x2) SWAP(x1,x2);
		if (y1>y2) SWAP(y1,y2);
		VGA_box (x1,y1,x2,y2, rand() & 0xff);

		//VGA_box (0,0,99,99, rand() & 0xffff);

		VGA_line(0, 0, 320, 240, 0xe0) ; // red
		VGA_line(639, 0, 320, 240, 0x1c) ; // green
		VGA_line(639, 479, 320, 240, 0x03) ; // blue
					
	}
}

/****************************************************************************************
 * Subroutine to send a string of text to the VGA monitor 
****************************************************************************************/
void VGA_text(int x, int y, char * text_ptr)
{
	int offset;
  	volatile char * character_buffer = (char *) vga_char_ptr ;	// VGA character buffer

	/* assume that the text string fits on one line */
	offset = (y << 7) + x;
	while ( *(text_ptr) )
	{
		// write to the character buffer
		*(character_buffer + offset) = *(text_ptr);	
		++text_ptr;
		++offset;
	}
}

/****************************************************************************************
 * Draw a filled rectangle on the VGA monitor 
****************************************************************************************/
void VGA_box(int x1, int y1, int x2, int y2, short pixel_color)
{
	int *pixel_ptr, row, col;

	/* assume that the box coordinates are valid */
	for (row = y1; row <= y2; row++)
		for (col = x1; col <= x2; ++col)
		{
			//640x480
			pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
			// set pixel color
			*(char *)pixel_ptr = pixel_color;		
		}
}

// =============================================
// === Draw a line
// =============================================
//plot a line 
//at x1,y1 to x2,y2 with color 
//Code is from David Rodgers,
//"Procedural Elements of Computer Graphics",1985
void VGA_line(int x1, int y1, int x2, int y2, short c) {
	int e;
	signed int dx,dy,j, temp;
	signed int s1,s2, xchange;
     signed int x,y;
	int *pixel_ptr ;
        
	x = x1;
	y = y1;
	
	//take absolute value
	if (x2 < x1) {
		dx = x1 - x2;
		s1 = -1;
	}

	else if (x2 == x1) {
		dx = 0;
		s1 = 0;
	}

	else {
		dx = x2 - x1;
		s1 = 1;
	}

	if (y2 < y1) {
		dy = y1 - y2;
		s2 = -1;
	}

	else if (y2 == y1) {
		dy = 0;
		s2 = 0;
	}

	else {
		dy = y2 - y1;
		s2 = 1;
	}

	xchange = 0;   

	if (dy>dx) {
		temp = dx;
		dx = dy;
		dy = temp;
		xchange = 1;
	} 

	e = ((int)dy<<1) - dx;  
	 
	for (j=0; j<=dx; j++) {
		//video_pt(x,y,c); //640x480
		pixel_ptr = (char *)vga_pixel_ptr + (y<<10)+ x; 
		// set pixel color
		*(char *)pixel_ptr = c;	
		 
		if (e>=0) {
			if (xchange==1) x = x + s1;
			else y = y + s2;
			e = e - ((int)dx<<1);
		}

		if (xchange==1) y = y + s2;
		else x = x + s1;

		e = e + ((int)dy<<1);
	}
}