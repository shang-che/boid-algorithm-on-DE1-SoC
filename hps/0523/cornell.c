#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ipc.h>
#include <sys/mman.h>
#include <sys/shm.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include "address_map_arm_brl4.h"
// #include "address_map_arm_brl4.h"
// boid define
#define boid_num 50
#define visual_range 40.0
#define protected_range 8.0
#define max_speed 3.0
#define min_speed 2.0
#define avoidfactor 0.05
#define matchingfactor 0.1
#define centeringfactor 0.005
#define turnfactor 0.5

/* function prototypes */
void VGA_text(int, int, char *);
void VGA_text_clear();
void VGA_box(int, int, int, int, short);
void VGA_line(int, int, int, int, short);
void VGA_disc(int, int, int, short);

// define colors
#define WHITE 0xff
#define BLACK 0x00
#define BLUE 0b00011111
#define RED 0b11100000
#define GREEN 0b00011100
#define PURPLE 0b11100011
#define YELLOW 0b11111101

// PIO ports
volatile unsigned int *right_audio = NULL;
volatile unsigned int *left_audio = NULL;
volatile unsigned int *debug1 = NULL;
volatile unsigned int *debug2 = NULL;
volatile unsigned int *debug3 = NULL;
volatile unsigned int *freq1 = NULL;
volatile unsigned int *freq2 = NULL;
volatile unsigned int *freq3 = NULL;
volatile unsigned int *freq4 = NULL;
volatile unsigned int *freq5 = NULL;

// PIO ports base address offsets
#define RIGHT_AUDIO_OFFSET 0x10
#define DEBUG1_OFFSET 0x20
#define DEBUG2_OFFSET 0x30
#define DEBUG3_OFFSET 0x40
#define FREQ1_OFFSET 0x50
#define FREQ2_OFFSET 0x60
#define FREQ3_OFFSET 0x70
#define FREQ4_OFFSET 0x80
#define FREQ5_OFFSET 0x90

// the light weight buss base
void *h2p_lw_virtual_base;

// RAM FPGA command buffer
volatile unsigned int *sram_ptr = NULL;
void *sram_virtual_base;

// pixel buffer
volatile unsigned int *vga_pixel_ptr = NULL;
void *vga_pixel_virtual_base;

// character buffer
volatile unsigned int *vga_char_ptr = NULL;
void *vga_char_virtual_base;

// /dev/mem file id
int fd;

//=======================================================
// pixel macro
// !!!PACKED VGA MEMORY!!!
// The lines are contenated with no padding
#define VGA_PIXEL(x, y, color)                                 \
    do {                                                       \
        char *pixel_ptr;                                       \
        pixel_ptr = (char *)vga_pixel_ptr + ((y) * 640) + (x); \
        *(char *)pixel_ptr = (color);                          \
    } while (0)
//========================================================
// swap macro
#define SWAP(X, Y)    \
    do {              \
        int temp = X; \
        X = Y;        \
        Y = temp;     \
    } while (0)
//========================================================

// === the fixed point macros ========================================
typedef signed int fix15;
#define multfix15(a, b) \
    ((fix15)((((signed long long)(a)) * ((signed long long)(b))) >> 15))
#define float2fix15(a) ((fix15)((a) * 32768.0))
#define fix2float15(a) ((float)(a) / 32768.0)
#define absfix15(a) abs(a)
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a, b) (fix15)((((signed long long)(a)) << 15) / (b))

// global variable for pixel boundaries
int leftBound;
int rightBound;
int topBound;
int bottomBound;

// mode of boid boundaries
int mode = 0;  // 0 for box, 1 for column, 2 for wraparound

// define boundary hit
#define hitBottom(b) (b > int2fix15(bottomBound))
#define hitTop(b) (bint2fix15(rightBound))

// uS per frame
#define FRAME_RATE 33000

// define boid quantity conditions
#define numBoids 300
int half_numBoids = numBoids / 2;
int numBoids_cutoff = numBoids / 5;

// the color of the boid
char color = 0xff;

// boid parameters
fix15 turnfactor = float2fix15(0.2);
fix15 visualRange = int2fix15(40);
fix15 visualRange_sq = multfix15(int2fix15(40), int2fix15(40));
fix15 protectedRange = int2fix15(8);
fix15 protectedRange_sq = multfix15(int2fix15(8), int2fix15(8));
fix15 centeringfactor = float2fix15(0.0005);
fix15 avoidfactor = float2fix15(0.05);
fix15 matchingfactor = float2fix15(0.05);
fix15 maxspeed = int2fix15(6);
fix15 minspeed = int2fix15(3);
fix15 maxbias = float2fix15(0.01);
fix15 bias_increment = float2fix15(0.00004);
fix15 biasval = float2fix15(
    1.0);  // default value (user-changeable, or updated dynamically)

static char framerate_text[40];

// struct for boids
struct boids {
    fix15 x;
    fix15 y;
    fix15 vx;
    fix15 vy;
    int direction;
} boids_t;

// array of boids
struct boids boidArray[numBoids];

// measure time
struct timeval t1, t2, start_time, end_time;
double elapsedTime;
struct timespec delay_time;

//////////////////////////////////////////
/// Function to create a single boid   ///
//////////////////////////////////////////
void spawnBoid(fix15 *x, fix15 *y, fix15 *vx, fix15 *vy, int direction) {
    // Start in center of screen
    int random = rand() % 4 + 3;
    *x = int2fix15(rand() % 441 + 100);
    *y = int2fix15(rand() % 281 + 100);

    // Choose left or right
    if (direction)
        *vx = int2fix15(random);
    else
        *vx = int2fix15(-random);
    // Moving down
    *vy = int2fix15(random);
}

/////////////////////////////////////////////////////
/// Function to map frequencies to boid parameters///
/////////////////////////////////////////////////////
float map(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/////////////////////////////////////////////////////
/// Function to update the conditions of each boid///
/////////////////////////////////////////////////////
void update_one_boid(fix15 *x, fix15 *y, fix15 *vx, fix15 *vy, int idx) {
    // For every boid . . .

    // Zero all accumulator variables (can't do this in one line in C)
    fix15 xpos_avg = 0;
    fix15 ypos_avg = 0;
    fix15 xvel_avg = 0;
    fix15 yvel_avg = 0;
    fix15 neighboring_boids = 0;
    fix15 close_dx = 0;
    fix15 close_dy = 0;

    // For every other boid in the flock . . .
    int i;
    for (i = 0; i 0) {
        xpos_avg = divfix(xpos_avg, neighboring_boids);
        ypos_avg = divfix(ypos_avg, neighboring_boids);
        xvel_avg = divfix(xvel_avg, neighboring_boids);
        yvel_avg = divfix(yvel_avg, neighboring_boids);

        *vx = *vx + multfix15((xpos_avg - (*x)), centeringfactor) +
              multfix15((xvel_avg - (*vx)), matchingfactor);
        *vy = *vy + multfix15((ypos_avg - (*y)), centeringfactor) +
              multfix15((yvel_avg - (*vy)), matchingfactor);
    }

    *vx = *vx + multfix15(close_dx, avoidfactor);
    *vy = *vy + multfix15(close_dy, avoidfactor);

    // If the boid is near an edge, make it turn by turnfactor
    if (hitTop(*y) && mode == 0) {
        *vy = (*vy + turnfactor);
    }
    if (hitBottom(*y) && mode == 0) {
        *vy = (*vy - turnfactor);
    }
    if (hitRight(*x) && mode != 2) {
        *vx = (*vx - turnfactor);
    }
    if (hitLeft(*x) && mode != 2) {
        *vx = (*vx + turnfactor);
    }

    // Connecting the left and right walls, top and bottom walls
    if (*x <= int2fix15(0) && mode == 2) {
        *x = int2fix15(639);
    }
    if (*x >= int2fix15(640) && mode == 2) {
        *x = int2fix15(1);
    }
    if (*y <= int2fix15(0) && mode != 0) {
        *y = int2fix15(479);
    }
    if (*y >= int2fix15(480) && mode != 0) {
        *y = int2fix15(1);
    }

    // Calculate the boid's speed
    float float_speed = sqrt(fix2float15(*vx) * fix2float15(*vx) +
                             fix2float15(*vy) * fix2float15(*vy));
    fix15 speed = float2fix15(float_speed);
    fix15 vx_div_speed = divfix(*vx, speed);
    fix15 vy_div_speed = divfix(*vy, speed);

    // Enforce min and max speeds
    if (speed < minspeed) {
        *vx = multfix15(vx_div_speed, minspeed);
        *vy = multfix15(vy_div_speed, minspeed);
    } else if (speed > maxspeed) {
        *vx = multfix15(vx_div_speed, maxspeed);
        *vy = multfix15(vy_div_speed, maxspeed);
    }
    // Update boid's position

    *x = *x + *vx;
    *y = *y + *vy;
}

//////////////////////////////////////////
/// Function to draw boid boundary (box)//
//////////////////////////////////////////
void drawBox() {
    topBound = 100;
    bottomBound = 380;
    leftBound = 100;
    rightBound = 540;

    VGA_line(leftBound, topBound, leftBound, bottomBound, WHITE);
    VGA_line(rightBound, topBound, rightBound, bottomBound, WHITE);
    VGA_line(leftBound, topBound, rightBound, topBound, WHITE);
    VGA_line(leftBound, bottomBound, rightBound, bottomBound, WHITE);
}

/////////////////////////////////////////////
/// Function to draw boid boundary (column)//
/////////////////////////////////////////////
void drawColumn() {
    leftBound = 250;
    rightBound = 390;
    VGA_line(leftBound, 0, leftBound, 500, WHITE);
    VGA_line(rightBound, 0, rightBound, 500, WHITE);
}

// Max and min macros
#define max(a, b) ((a > b) ? a : b)
#define min(a, b) ((a bin2){
max12 = bin1;
max_freq12 = 350.;
}
else {
    max12 = bin2;
    max_freq12 = 750.;
}

// compare bin 3 and 4
if (bin3 > bin4) {
    max34 = bin3;
    max_freq34 = 1050.;
} else {
    max34 = bin4;
    max_freq34 = 1400.;
}

// compare result from 12 and 34
if (max12 > max34) {
    max_amp = max12;
    max_freq = max_freq12;
} else {
    max_amp = max34;
    max_freq = max_freq34;
}

// compare to bin5
if (bin5 > max_amp) {
    max_amp = bin5;
    max_freq = 1800.;
}

// map max frequency to boid parameters
float float_centering_factor = map(max_freq, 350, 1800, 0.0005, 0.2);
centeringfactor = float2fix15(float_centering_factor);

float float_avoid_factor = map(max_freq, 1800, 350, 0.0005, 0.2);
avoidfactor = float2fix15(float_avoid_factor);

// update each boid in boid array
for (c = 0; c < numBoids; c++) {
    // erase boid
    VGA_disc(fix2int15(boidArray[c].x), fix2int15(boidArray[c].y), 2, BLACK);

    // update boid's position and velocity
    update_one_boid(&(boidArray[c].x), &(boidArray[c].y), &(boidArray[c].vx),
                    &(boidArray[c].vy), c);

    // draw the boid at its new position, with different colors
    if (c >= 0 && c < numBoids_cutoff) {  // 0-79
        VGA_disc(fix2int15(boidArray[c].x), fix2int15(boidArray[c].y), 2, RED);
    } else if (c >= numBoids_cutoff && c < (numBoids_cutoff << 1)) {
        VGA_disc(fix2int15(boidArray[c].x), fix2int15(boidArray[c].y), 2,
                 YELLOW);
    } else if (c >= (numBoids_cutoff << 1) && c < (numBoids_cutoff * 3)) {
        VGA_disc(fix2int15(boidArray[c].x), fix2int15(boidArray[c].y), 2,
                 GREEN);
    } else if (c >= (numBoids_cutoff * 3) && c < (numBoids_cutoff << 2)) {
        VGA_disc(fix2int15(boidArray[c].x), fix2int15(boidArray[c].y), 2, BLUE);
    } else {
        VGA_disc(fix2int15(boidArray[c].x), fix2int15(boidArray[c].y), 2,
                 PURPLE);
    }

}  // end of for loop

// change boid boundaries based on mode value
if (mode != (*debug1)) {
    VGA_line(leftBound, topBound, leftBound, bottomBound, BLACK);
    VGA_line(rightBound, topBound, rightBound, bottomBound, BLACK);
    VGA_line(leftBound, topBound, rightBound, topBound, BLACK);
    VGA_line(leftBound, bottomBound, rightBound, bottomBound, BLACK);
    VGA_line(leftBound, 0, leftBound, 500, BLACK);
    VGA_line(rightBound, 0, rightBound, 500, BLACK);
    mode = *debug1;
}
if (mode == 0) {
    drawBox();
} else if (mode == 1) {
    drawColumn();
} else {
    topBound = 0;
    bottomBound = 0;
    leftBound = 0;
    rightBound = 0;
}

}  // end while

}  // end main

/****************************************************************************************
 * Subroutine to send a string of text to the VGA monitor
 ****************************************************************************************/
void VGA_text(int x, int y, char *text_ptr) {
    volatile char *character_buffer =
        (char *)vga_char_ptr;  // VGA character buffer
    int offset;
    /* assume that the text string fits on one line */
    offset = (y << 7) + x;
    while (*(text_ptr)) {
        // write to the character buffer
        *(character_buffer + offset) = *(text_ptr);
        ++text_ptr;
        ++offset;
    }
}

/****************************************************************************************
 * Subroutine to clear text to the VGA monitor
 ****************************************************************************************/
void VGA_text_clear() {
    volatile char *character_buffer =
        (char *)vga_char_ptr;  // VGA character buffer
    int offset, x, y;
    for (x = 0; x < 79; x++) {
        for (y = 0; y < 59; y++) {
            /* assume that the text string fits on one line */
            offset = (y << 7) + x;
            // write to the character buffer
            *(character_buffer + offset) = ' ';
        }
    }
}

/****************************************************************************************
 * Draw a filled rectangle on the VGA monitor
 ****************************************************************************************/
#define SWAP(X, Y)    \
    do {              \
        int temp = X; \
        X = Y;        \
        Y = temp;     \
    } while (0)

void VGA_box(int x1, int y1, int x2, int y2, short pixel_color) {
    char *pixel_ptr;
    int row, col;

    /* check and fix box coordinates to be valid */
    if (x1 > 639) x1 = 639;
    if (y1 > 479) y1 = 479;
    if (x2 > 639) x2 = 639;
    if (y2 > 479) y2 = 479;
    if (x1 < 0) x1 = 0;
    if (y1 < 0) y1 = 0;
    if (x2 < 0) x2 = 0;
    if (y2 < 0) y2 = 0;
    if (x1 > x2) SWAP(x1, x2);
    if (y1 > y2) SWAP(y1, y2);
    for (row = y1; row <= y2; row++)
        for (col = x1; col <= x2; ++col) {
            // 640x480
            VGA_PIXEL(col, row, pixel_color);
        }
}

/****************************************************************************************
 * Draw a filled circle on the VGA monitor using GPU FSM
 ****************************************************************************************/

void VGA_disc(int x, int y, int r, short pixel_color) {
    char *pixel_ptr;
    int row, col, rsqr, xc, yc;

    rsqr = r * r;

    for (yc = -r; yc <= r; yc++)
        for (xc = -r; xc <= r; xc++) {
            col = xc;
            row = yc;
            // add the r to make the edge smoother
            if (col * col + row * row <= rsqr + r) {
                col += x;  // add the center point
                row += y;  // add the center point
                // check for valid 640x480
                if (col > 639) col = 639;
                if (row > 479) row = 479;
                if (col < 0) col = 0;
                if (row < 0) row = 0;

                // set up scratch pad parameters
                *(sram_ptr + 1) = col - 0.5;
                *(sram_ptr + 2) = row - 0.5;
                *(sram_ptr + 3) = col;
                *(sram_ptr + 4) = row;
                *(sram_ptr + 5) = pixel_color;
                *(sram_ptr) = 1;  // the "data-ready" flag

                // wait for FPGA to zero the "data_ready" flag
                while (*(sram_ptr) > 0)
                    ;
            }
        }
}

// =============================================
// === Draw a line
// =============================================
// plot a line
// at x1,y1 to x2,y2 with color
// Code is from David Rodgers,
//"Procedural Elements of Computer Graphics",1985
void VGA_line(int x1, int y1, int x2, int y2, short c) {
    int e;
    signed int dx, dy, j, temp;
    signed int s1, s2, xchange;
    signed int x, y;
    char *pixel_ptr;

    /* check and fix line coordinates to be valid */
    if (x1 > 639) x1 = 639;
    if (y1 > 479) y1 = 479;
    if (x2 > 639) x2 = 639;
    if (y2 > 479) y2 = 479;
    if (x1 < 0) x1 = 0;
    if (y1 < 0) y1 = 0;
    if (x2 < 0) x2 = 0;
    if (y2 < 0) y2 = 0;

    x = x1;
    y = y1;

    // take absolute value
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

    if (dy > dx) {
        temp = dx;
        dx = dy;
        dy = temp;
        xchange = 1;
    }

    e = ((int)dy << 1) - dx;

    for (j = 0; j <= dx; j++) {
        // video_pt(x,y,c); //640x480
        VGA_PIXEL(x, y, c);
        // pixel_ptr = (char *)vga_pixel_ptr + (y<<10)+ x;
        //  set pixel color
        //*(char *)pixel_ptr = c;

        if (e >= 0) {
            if (xchange == 1)
                x = x + s1;
            else
                y = y + s2;
            e = e - ((int)dx << 1);
        }

        if (xchange == 1)
            y = y + s2;
        else
            x = x + s1;

        e = e + ((int)dy << 1);
    }
}
