///////////////////////////////////////
/// 640x480 version! 16-bit color
/// This code will segfault the original
/// DE1 computer
/// compile with
/// gcc dancing_boids.c -o gr -O2 -lm
///
///////////////////////////////////////
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
// #include "address_map_arm_brl4.h"
// boid define
#define boid_num 200
#define visual_range 40.0
#define protected_range 20.0
#define max_speed 7.0
#define min_speed 4.0
// #define avoidfactor 0.05
#define matchingfactor 0.05
// #define centeringfactor 0.0005
#define turnfactor 0.5
double avoidfactor;
// double matchingfactor;
double centeringfactor;

#define WIDTH 640
#define HEIGHT 480

// video display
#define SDRAM_BASE 0xC0000000
#define SDRAM_END 0xC3FFFFFF
#define SDRAM_SPAN 0x04000000
// characters
#define FPGA_CHAR_BASE 0xC9000000
#define FPGA_CHAR_END 0xC9001FFF
#define FPGA_CHAR_SPAN 0x00002000
/* Cyclone V FPGA devices */
#define HW_REGS_BASE 0xff200000
// #define HW_REGS_SPAN        0x00200000
#define HW_REGS_SPAN 0x00005000

#define FPGA_ONCHIP_BASE 0xC8000000
#define FPGA_ONCHIP_END 0xC803FFFF
#define FPGA_ONCHIP_SPAN 0x00040000

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

// graphics primitives
void VGA_text(int, int, char *);
void VGA_text_clear();
void VGA_box(int, int, int, int, short);
void VGA_rect(int, int, int, int, short);
void VGA_line(int, int, int, int, short);
void VGA_Vline(int, int, int, short);
void VGA_Hline(int, int, int, short);
void VGA_disc(int, int, int, short);
void VGA_circle(int, int, int, int);
// 16-bit primary colors
#define red (0 + (0 << 5) + (31 << 11))
#define dark_red (0 + (0 << 5) + (15 << 11))
#define green (0 + (63 << 5) + (0 << 11))
#define dark_green (0 + (31 << 5) + (0 << 11))
#define blue (31 + (0 << 5) + (0 << 11))
#define dark_blue (15 + (0 << 5) + (0 << 11))
#define yellow (0 + (63 << 5) + (31 << 11))
#define cyan (31 + (63 << 5) + (0 << 11))
#define magenta (31 + (0 << 5) + (31 << 11))
#define black (0x0000)
#define gray (15 + (31 << 5) + (51 << 11))
#define white (0xffff)
int colors[] = {red,    dark_red, green,   dark_green, blue,  dark_blue,
                yellow, cyan,     magenta, gray,       white, black};

// pixel macro
#define VGA_PIXEL(x, y, color)                                                 \
    do {                                                                       \
        int *pixel_ptr;                                                        \
        pixel_ptr = (int *)((char *)vga_pixel_ptr + (((y) * 640 + (x)) << 1)); \
        *(short *)pixel_ptr = (color);                                         \
    } while (0)

// the light weight buss base
void *h2p_lw_virtual_base;

// pixel buffer
volatile unsigned int *vga_pixel_ptr = NULL;
void *vga_pixel_virtual_base;

// character buffer
volatile unsigned int *vga_char_ptr = NULL;
void *vga_char_virtual_base;

// RAM FPGA command buffer
volatile unsigned int *sram_ptr = NULL;
void *sram_virtual_base;

// /dev/mem file id
int fd;

// measure time
struct timeval t1, t2;
double elapsedTime;

// boid function
typedef struct boid {
    double x;
    double y;
    double z;
    double vx;
    double vy;
    double vz;
    int monitor_x;
    int monitor_y;
    int color;

} Boid;

// 初始化boid的位置及速度
void initialize_boid(Boid boids[]) {
    int i;
    for (i = 0; i < boid_num; i++) {
        boids[i].color = colors[rand() % 10];
        boids[i].x = rand() % (WIDTH - 40);
        boids[i].y = rand() % (HEIGHT - 80);
        boids[i].z = rand() % 100;
        boids[i].vx =
            (rand() % 3 + 1) *
            (rand() % 2 == 0 ? 1 : -1);  //(double)(rand() % 100 - 50) / 50.0;
        boids[i].vy =
            (rand() % 3 + 1) *
            (rand() % 2 == 0 ? 1 : -1);  //(double)(rand() % 40 - 20) / 20.0;
        boids[i].vz =
            (rand() % 3 + 1) *
            (rand() % 2 == 0 ? 1 : -1);  //(double)(rand() % 100 - 50) / 50.0;
    }
}

// 判斷boid
void update_boid(Boid boids[]) {
    int i;
    for (i = 0; i < boid_num; i++) {
        double avg_vx = 0.0;  // average
        double avg_vy = 0.0;
        double avg_vz = 0.0;
        int neighbors = 0;

        double avg_xpos = 0.0;
        double avg_ypos = 0.0;
        double avg_zpos = 0.0;

        double close_dx = 0.0;
        double close_dy = 0.0;
        double close_dz = 0.0;

        double speed = 0.0;

        // alignment
        int j;
        for (j = 0; j < boid_num; j++) {
            if (i != j) {
                double dx = boids[j].x - boids[i].x;
                double dy = boids[j].y - boids[i].y;
                double dz = boids[j].z - boids[i].z;
                double distance = sqrt(dx * dx + dy * dy + dz * dz);
                if (distance < protected_range) {  // seperation
                    close_dx += boids[i].x - boids[j].x;
                    close_dy += boids[i].y - boids[j].y;
                    close_dz += boids[i].z - boids[j].z;

                } else if (distance < visual_range) {  // in visual area
                    // alignment
                    avg_vx += boids[j].vx;
                    avg_vy += boids[j].vy;
                    avg_vz += boids[j].vz;
                    // cohesion
                    avg_xpos += boids[j].x;
                    avg_ypos += boids[j].y;
                    avg_zpos += boids[j].z;

                    neighbors++;
                }
            }
        }
        if (neighbors > 0) {
            // alignment
            avg_vx = avg_vx / neighbors;
            avg_vy = avg_vy / neighbors;
            avg_vz = avg_vz / neighbors;
            boids[i].vx += (avg_vx - boids[i].vx) * matchingfactor;
            boids[i].vy += (avg_vy - boids[i].vy) * matchingfactor;
            boids[i].vz += (avg_vz - boids[i].vz) * matchingfactor;
            // cohesion
            avg_xpos = avg_xpos / neighbors;
            avg_ypos = avg_ypos / neighbors;
            avg_zpos = avg_zpos / neighbors;
            boids[i].vx += (avg_xpos - boids[i].x) * centeringfactor;
            boids[i].vy += (avg_ypos - boids[i].y) * centeringfactor;
            boids[i].vz += (avg_zpos - boids[i].z) * centeringfactor;
        }

        boids[i].vx += close_dx * avoidfactor;
        boids[i].vy += close_dy * avoidfactor;
        boids[i].vz += close_dz * avoidfactor;

        if (boids[i].x > WIDTH - 80) {
            boids[i].vx -= turnfactor;
        }
        if (boids[i].x < 80) {
            boids[i].vx += turnfactor;
        }
        if (boids[i].y > HEIGHT - 40) {
            boids[i].vy -= turnfactor;
        }
        if (boids[i].y < 40) {
            boids[i].vy += turnfactor;
        }
        speed = sqrt(boids[i].vx * boids[i].vx + boids[i].vy * boids[i].vy);
        if (speed < min_speed) {
            boids[i].vx = (boids[i].vx / speed) * min_speed;
            boids[i].vy = (boids[i].vy / speed) * min_speed;
        }
        if (speed > max_speed) {
            boids[i].vx = (boids[i].vx / speed) * max_speed;
            boids[i].vy = (boids[i].vy / speed) * max_speed;
        }
        boids[i].x = boids[i].x + boids[i].vx;
        boids[i].y = boids[i].y + boids[i].vy;
    }
}

int main(void) {
    // === need to mmap: =======================
    // FPGA_CHAR_BASE
    // FPGA_ONCHIP_BASE
    // HW_REGS_BASE

    // === get FPGA addresses ==================
    // Open /dev/mem
    if ((fd = open("/dev/mem", (O_RDWR | O_SYNC))) == -1) {
        printf("ERROR: could not open \"/dev/mem\"...\n");
        return (1);
    }

    // get virtual addr that maps to physical
    h2p_lw_virtual_base = mmap(NULL, HW_REGS_SPAN, (PROT_READ | PROT_WRITE),
                               MAP_SHARED, fd, HW_REGS_BASE);
    if (h2p_lw_virtual_base == MAP_FAILED) {
        printf("ERROR: mmap1() failed...\n");
        close(fd);
        return (1);
    }

    // === get VGA char addr =====================
    // get virtual addr that maps to physical
    vga_char_virtual_base = mmap(NULL, FPGA_CHAR_SPAN, (PROT_READ | PROT_WRITE),
                                 MAP_SHARED, fd, FPGA_CHAR_BASE);
    if (vga_char_virtual_base == MAP_FAILED) {
        printf("ERROR: mmap2() failed...\n");
        close(fd);
        return (1);
    }

    // Get the address that maps to the FPGA LED control
    vga_char_ptr = (unsigned int *)(vga_char_virtual_base);

    // === get VGA pixel addr ====================
    // get virtual addr that maps to physical
    vga_pixel_virtual_base = mmap(NULL, SDRAM_SPAN, (PROT_READ | PROT_WRITE),
                                  MAP_SHARED, fd, SDRAM_BASE);
    if (vga_pixel_virtual_base == MAP_FAILED) {
        printf("ERROR: mmap3() failed...\n");
        close(fd);
        return (1);
    }

    // Get the address that maps to the FPGA pixel buffer
    vga_pixel_ptr = (unsigned int *)(vga_pixel_virtual_base);

    // === get RAM FPGA parameter addr =========
    /*sram_virtual_base = mmap(NULL, FPGA_ONCHIP_SPAN, (PROT_READ | PROT_WRITE),
                             MAP_SHARED, fd, FPGA_ONCHIP_BASE);  // fp

    if (sram_virtual_base == MAP_FAILED) {
        printf("ERROR: mmap3() failed...\n");
        close(fd);
        return (1);
    }
    // Get the address that maps to the RAM buffer
    sram_ptr = (unsigned int *)(sram_virtual_base);*/

    // get freq address pointer
    freq1 = (unsigned int *)(h2p_lw_virtual_base + FREQ1_OFFSET);
    freq2 = (unsigned int *)(h2p_lw_virtual_base + FREQ2_OFFSET);
    freq3 = (unsigned int *)(h2p_lw_virtual_base + FREQ3_OFFSET);
    freq4 = (unsigned int *)(h2p_lw_virtual_base + FREQ4_OFFSET);
    freq5 = (unsigned int *)(h2p_lw_virtual_base + FREQ5_OFFSET);

    // ===========================================

    /* create a message to be displayed on the VGA
      and LCD displays */
    char text_top_row[40] = "DE1-SoC ARM/FPGA\0";
    char text_bottom_row[40] = "Cornell ece5760\0";
    char text_next[40] = "Graphics primitives\0";
    char num_string[20], time_string[20];
    char color_index = 0;
    int color_counter = 0;

    // position of disk primitive
    int disc_x = 0;
    // position of circle primitive
    int circle_x = 0;
    // position of box primitive
    int box_x = 5;
    // position of vertical line primitive
    int Vline_x = 350;
    // position of horizontal line primitive
    int Hline_y = 250;

    // VGA_text (34, 1, text_top_row);
    // VGA_text (34, 2, text_bottom_row);
    //  clear the screen
    VGA_box(0, 0, 639, 479, 0x0000);
    // clear the text
    VGA_text_clear();
    // write text
    // VGA_text (10, 1, text_top_row);
    // VGA_text (10, 2, text_bottom_row);
    // VGA_text (10, 3, text_next);

    // R bits 11-15 mask 0xf800
    // G bits 5-10  mask 0x07e0
    // B bits 0-4   mask 0x001f
    // so color = B+(G<<5)+(R<<11);
    Boid boids[boid_num];
    Boid boid_prev[boid_num];
    srand(time(NULL));
    initialize_boid(boids);
    // memcpy(boid_prev, boids, sizeof(Boid) * boid_num);
    while (1) {
        int freq1_weight = (*freq1 + 2000000) % 100;
        int freq2_weight = (*freq2 + 2000000) % 100;
        int freq3_weight = (*freq3 + 2000000) % 100;
        int freq4_weight = (*freq4 + 2000000) % 100;
        int freq5_weight = (*freq5 + 2000000) % 100;

        int highest_freq = 0;
        int channel = 0;

        if (freq1_weight > highest_freq) {
            highest_freq = freq1_weight;
            channel = 1;
        }
        if (freq2_weight > highest_freq) {
            highest_freq = freq2_weight;
            channel = 2;
        }
        if (freq3_weight > highest_freq) {
            highest_freq = freq3_weight;
            channel = 3;
        }
        if (freq4_weight > highest_freq) {
            highest_freq = freq4_weight;
            channel = 4;
        }
        if (freq5_weight > highest_freq) {
            highest_freq = freq5_weight;
            channel = 5;
        }

        // 0.05 0.0005
        switch (channel) {
            case 1:
                avoidfactor = 0.2;
                centeringfactor = 0.0001;
                break;
            case 2:
                avoidfactor = 0.2;
                centeringfactor = 0.0001;
                break;
            case 3:
                avoidfactor = 0.2;
                centeringfactor = 0.0001;
                break;
            case 4:
                avoidfactor = 0.01;
                centeringfactor = 0.0025;
                break;
            case 5:
                avoidfactor = 0.01;
                centeringfactor = 0.0025;
                break;
            default:
                avoidfactor = 0.05;
                centeringfactor = 0.05;
                break;
        }

        //printf("%f %f %f\n", avoidfactor, centeringfactor, matchingfactor);
        //printf("%d %d %d %d %d\n", freq1_weight, freq2_weight, freq3_weight,
        //       freq4_weight, freq5_weight);
        printf ("%d %d %d %d %d\n",*freq1,*freq2,*freq3,*freq4,*freq5);
        
        
        //printf("%d\n", channel);

        // start timer
        gettimeofday(&t1, NULL);
        //
        //		//VGA_box(int x1, int y1, int x2, int y2, short
        // pixel_color) 		VGA_box(64, 0, 240, 50, blue); // blue
        // box 		VGA_box(250, 0, 425, 50, red); // red box
        // VGA_box(435, 0, 600, 50, green); // green box
        //
        //		// cycle thru the colors
        //		if (color_index++ == 11) color_index = 0;
        //
        //		//void VGA_disc(int x, int y, int r, short pixel_color)
        //		VGA_disc(disc_x, 100, 20, colors[color_index]);
        //		disc_x += 35 ;
        //		if (disc_x > 640) disc_x = 0;
        //
        //		//void VGA_circle(int x, int y, int r, short
        // pixel_color) 		VGA_circle(320, 200, circle_x,
        // colors[color_index]); 		VGA_circle(320, 200, circle_x+1,
        // colors[color_index]); 		circle_x += 2 ;
        // if (circle_x > 99) circle_x = 0;
        //
        //		//void VGA_rect(int x1, int y1, int x2, int y2, short
        // pixel_color) 		VGA_rect(10, 478, box_x, 478-box_x,
        // rand()&0xffff); 		box_x
        //+= 3 ; 		if (box_x > 195) box_x = 10;
        //
        //		//void VGA_line(int x1, int y1, int x2, int y2, short c)
        //		VGA_line(210+(rand()&0x7f), 350+(rand()&0x7f),
        // 210+(rand()&0x7f), 				350+(rand()&0x7f),
        // colors[color_index]);
        //
        //		// void VGA_Vline(int x1, int y1, int y2, short
        // pixel_color) 		VGA_Vline(Vline_x, 475,
        // 475-(Vline_x>>2), rand()&0xffff); 		Vline_x += 2 ; if
        // (Vline_x > 620)
        // Vline_x = 350;
        //
        //		//void VGA_Hline(int x1, int y1, int x2, short
        // pixel_color) 		VGA_Hline(400, Hline_y, 550,
        // rand()&0xffff); 		Hline_y += 2 ; 		if (Hline_y >
        // 400) Hline_y = 240;
        //
        // stop timer
        gettimeofday(&t2, NULL);
        elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000000.0;  // sec to us
        elapsedTime += (t2.tv_usec - t1.tv_usec);           // us
        sprintf(time_string, "T = %6.0f uSec  ", elapsedTime);
        // VGA_text(10, 4, time_string);

        update_boid(boids);
        int i;
        Boid boid_prev[boid_num];
        for (i = 1; i < boid_num; i++) {
            boid_prev[i].monitor_x = boid_prev[i].x;
            boid_prev[i].monitor_y = boid_prev[i].y;
            VGA_disc(boid_prev[i].monitor_x, boid_prev[i].monitor_y, 2, black);
            boids[i].monitor_x = boids[i].x;
            boids[i].monitor_y = boids[i].y;
            VGA_disc(boids[i].monitor_x, boids[i].monitor_y, 2, boids[i].color);
            boid_prev[i].x = boids[i].x;
            boid_prev[i].y = boids[i].y;
        }
        // set frame rate
        usleep(17000);  // 17000
        // sleep(0.25);
        //  clear the screen
        // VGA_box(0, 0, 639, 479, 0x0000);
    }  // end while(1)
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
            // pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
            //  set pixel color
            //*(char *)pixel_ptr = pixel_color;
            VGA_PIXEL(col, row, pixel_color);
        }
}

/****************************************************************************************
 * Draw a outline rectangle on the VGA monitor
 ****************************************************************************************/
#define SWAP(X, Y)    \
    do {              \
        int temp = X; \
        X = Y;        \
        Y = temp;     \
    } while (0)

void VGA_rect(int x1, int y1, int x2, int y2, short pixel_color) {
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
    // left edge
    col = x1;
    for (row = y1; row <= y2; row++) {
        // 640x480
        // pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
        //  set pixel color
        //*(char *)pixel_ptr = pixel_color;
        VGA_PIXEL(col, row, pixel_color);
    }

    // right edge
    col = x2;
    for (row = y1; row <= y2; row++) {
        // 640x480
        // pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
        //  set pixel color
        //*(char *)pixel_ptr = pixel_color;
        VGA_PIXEL(col, row, pixel_color);
    }

    // top edge
    row = y1;
    for (col = x1; col <= x2; ++col) {
        // 640x480
        // pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
        //  set pixel color
        //*(char *)pixel_ptr = pixel_color;
        VGA_PIXEL(col, row, pixel_color);
    }

    // bottom edge
    row = y2;
    for (col = x1; col <= x2; ++col) {
        // 640x480
        // pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
        //  set pixel color
        //*(char *)pixel_ptr = pixel_color;
        VGA_PIXEL(col, row, pixel_color);
    }
}

/****************************************************************************************
 * Draw a horixontal line on the VGA monitor
 ****************************************************************************************/
#define SWAP(X, Y)    \
    do {              \
        int temp = X; \
        X = Y;        \
        Y = temp;     \
    } while (0)

void VGA_Hline(int x1, int y1, int x2, short pixel_color) {
    char *pixel_ptr;
    int row, col;

    /* check and fix box coordinates to be valid */
    if (x1 > 639) x1 = 639;
    if (y1 > 479) y1 = 479;
    if (x2 > 639) x2 = 639;
    if (x1 < 0) x1 = 0;
    if (y1 < 0) y1 = 0;
    if (x2 < 0) x2 = 0;
    if (x1 > x2) SWAP(x1, x2);
    // line
    row = y1;
    for (col = x1; col <= x2; ++col) {
        // 640x480
        // pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
        //  set pixel color
        //*(char *)pixel_ptr = pixel_color;
        VGA_PIXEL(col, row, pixel_color);
    }
}

/****************************************************************************************
 * Draw a vertical line on the VGA monitor
 ****************************************************************************************/
#define SWAP(X, Y)    \
    do {              \
        int temp = X; \
        X = Y;        \
        Y = temp;     \
    } while (0)

void VGA_Vline(int x1, int y1, int y2, short pixel_color) {
    char *pixel_ptr;
    int row, col;

    /* check and fix box coordinates to be valid */
    if (x1 > 639) x1 = 639;
    if (y1 > 479) y1 = 479;
    if (y2 > 479) y2 = 479;
    if (x1 < 0) x1 = 0;
    if (y1 < 0) y1 = 0;
    if (y2 < 0) y2 = 0;
    if (y1 > y2) SWAP(y1, y2);
    // line
    col = x1;
    for (row = y1; row <= y2; row++) {
        // 640x480
        // pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
        //  set pixel color
        //*(char *)pixel_ptr = pixel_color;
        VGA_PIXEL(col, row, pixel_color);
    }
}

/****************************************************************************************
 * Draw a filled circle on the VGA monitor
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
                // pixel_ptr = (char *)vga_pixel_ptr + (row<<10) + col ;
                //  set pixel color
                //*(char *)pixel_ptr = pixel_color;
                VGA_PIXEL(col, row, pixel_color);
            }
        }
}

/****************************************************************************************
 * Draw a  circle on the VGA monitor
 ****************************************************************************************/

void VGA_circle(int x, int y, int r, int pixel_color) {
    char *pixel_ptr;
    int row, col, rsqr, xc, yc;
    int col1, row1;
    rsqr = r * r;

    for (yc = -r; yc <= r; yc++) {
        // row = yc;
        col1 = (int)sqrt((float)(rsqr + r - yc * yc));
        // right edge
        col = col1 + x;  // add the center point
        row = yc + y;    // add the center point
        // check for valid 640x480
        if (col > 639) col = 639;
        if (row > 479) row = 479;
        if (col < 0) col = 0;
        if (row < 0) row = 0;
        // pixel_ptr = (char *)vga_pixel_ptr + (row<<10) + col ;
        //  set pixel color
        //*(char *)pixel_ptr = pixel_color;
        VGA_PIXEL(col, row, pixel_color);
        // left edge
        col = -col1 + x;  // add the center point
        // check for valid 640x480
        if (col > 639) col = 639;
        if (row > 479) row = 479;
        if (col < 0) col = 0;
        if (row < 0) row = 0;
        // pixel_ptr = (char *)vga_pixel_ptr + (row<<10) + col ;
        //  set pixel color
        //*(char *)pixel_ptr = pixel_color;
        VGA_PIXEL(col, row, pixel_color);
    }
    for (xc = -r; xc <= r; xc++) {
        // row = yc;
        row1 = (int)sqrt((float)(rsqr + r - xc * xc));
        // right edge
        col = xc + x;    // add the center point
        row = row1 + y;  // add the center point
        // check for valid 640x480
        if (col > 639) col = 639;
        if (row > 479) row = 479;
        if (col < 0) col = 0;
        if (row < 0) row = 0;
        // pixel_ptr = (char *)vga_pixel_ptr + (row<<10) + col ;
        //  set pixel color
        //*(char *)pixel_ptr = pixel_color;
        VGA_PIXEL(col, row, pixel_color);
        // left edge
        row = -row1 + y;  // add the center point
        // check for valid 640x480
        if (col > 639) col = 639;
        if (row > 479) row = 479;
        if (col < 0) col = 0;
        if (row < 0) row = 0;
        // pixel_ptr = (char *)vga_pixel_ptr + (row<<10) + col ;
        //  set pixel color
        //*(char *)pixel_ptr = pixel_color;
        VGA_PIXEL(col, row, pixel_color);
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
        // pixel_ptr = (char *)vga_pixel_ptr + (y<<10)+ x;
        //  set pixel color
        //*(char *)pixel_ptr = c;
        VGA_PIXEL(x, y, c);

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
