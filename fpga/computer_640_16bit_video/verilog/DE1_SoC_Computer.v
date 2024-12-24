

module DE1_SoC_Computer (
	////////////////////////////////////
	// FPGA Pins
	////////////////////////////////////

	// Clock pins
	CLOCK_50,
	CLOCK2_50,
	CLOCK3_50,
	CLOCK4_50,

	// ADC
	ADC_CS_N,
	ADC_DIN,
	ADC_DOUT,
	ADC_SCLK,

	// Audio
	AUD_ADCDAT,
	AUD_ADCLRCK,
	AUD_BCLK,
	AUD_DACDAT,
	AUD_DACLRCK,
	AUD_XCK,

	// SDRAM
	DRAM_ADDR,
	DRAM_BA,
	DRAM_CAS_N,
	DRAM_CKE,
	DRAM_CLK,
	DRAM_CS_N,
	DRAM_DQ,
	DRAM_LDQM,
	DRAM_RAS_N,
	DRAM_UDQM,
	DRAM_WE_N,

	// I2C Bus for Configuration of the Audio and Video-In Chips
	FPGA_I2C_SCLK,
	FPGA_I2C_SDAT,

	// 40-Pin Headers
	GPIO_0,
	GPIO_1,
	
	// Seven Segment Displays
	HEX0,
	HEX1,
	HEX2,
	HEX3,
	HEX4,
	HEX5,

	// IR
	IRDA_RXD,
	IRDA_TXD,

	// Pushbuttons
	KEY,

	// LEDs
	LEDR,

	// PS2 Ports
	PS2_CLK,
	PS2_DAT,
	
	PS2_CLK2,
	PS2_DAT2,

	// Slider Switches
	SW,

	// Video-In
	TD_CLK27,
	TD_DATA,
	TD_HS,
	TD_RESET_N,
	TD_VS,

	// VGA
	VGA_B,
	VGA_BLANK_N,
	VGA_CLK,
	VGA_G,
	VGA_HS,
	VGA_R,
	VGA_SYNC_N,
	VGA_VS,

	////////////////////////////////////
	// HPS Pins
	////////////////////////////////////
	
	// DDR3 SDRAM
	HPS_DDR3_ADDR,
	HPS_DDR3_BA,
	HPS_DDR3_CAS_N,
	HPS_DDR3_CKE,
	HPS_DDR3_CK_N,
	HPS_DDR3_CK_P,
	HPS_DDR3_CS_N,
	HPS_DDR3_DM,
	HPS_DDR3_DQ,
	HPS_DDR3_DQS_N,
	HPS_DDR3_DQS_P,
	HPS_DDR3_ODT,
	HPS_DDR3_RAS_N,
	HPS_DDR3_RESET_N,
	HPS_DDR3_RZQ,
	HPS_DDR3_WE_N,

	// Ethernet
	HPS_ENET_GTX_CLK,
	HPS_ENET_INT_N,
	HPS_ENET_MDC,
	HPS_ENET_MDIO,
	HPS_ENET_RX_CLK,
	HPS_ENET_RX_DATA,
	HPS_ENET_RX_DV,
	HPS_ENET_TX_DATA,
	HPS_ENET_TX_EN,

	// Flash
	HPS_FLASH_DATA,
	HPS_FLASH_DCLK,
	HPS_FLASH_NCSO,

	// Accelerometer
	HPS_GSENSOR_INT,
		
	// General Purpose I/O
	HPS_GPIO,
		
	// I2C
	HPS_I2C_CONTROL,
	HPS_I2C1_SCLK,
	HPS_I2C1_SDAT,
	HPS_I2C2_SCLK,
	HPS_I2C2_SDAT,

	// Pushbutton
	HPS_KEY,

	// LED
	HPS_LED,
		
	// SD Card
	HPS_SD_CLK,
	HPS_SD_CMD,
	HPS_SD_DATA,

	// SPI
	HPS_SPIM_CLK,
	HPS_SPIM_MISO,
	HPS_SPIM_MOSI,
	HPS_SPIM_SS,

	// UART
	HPS_UART_RX,
	HPS_UART_TX,

	// USB
	HPS_CONV_USB_N,
	HPS_USB_CLKOUT,
	HPS_USB_DATA,
	HPS_USB_DIR,
	HPS_USB_NXT,
	HPS_USB_STP
);


//=======================================================
//  PARAMETER declarations
//=======================================================


//=======================================================
//  PORT declarations
//=======================================================

////////////////////////////////////
// FPGA Pins
////////////////////////////////////

// Clock pins
input						CLOCK_50;
input						CLOCK2_50;
input						CLOCK3_50;
input						CLOCK4_50;

// ADC
inout						ADC_CS_N;
output					ADC_DIN;
input						ADC_DOUT;
output					ADC_SCLK;

// Audio
input						AUD_ADCDAT;
inout						AUD_ADCLRCK;
inout						AUD_BCLK;
output					AUD_DACDAT;
inout						AUD_DACLRCK;
output					AUD_XCK;

// SDRAM
output 		[12: 0]	DRAM_ADDR;
output		[ 1: 0]	DRAM_BA;
output					DRAM_CAS_N;
output					DRAM_CKE;
output					DRAM_CLK;
output					DRAM_CS_N;
inout			[15: 0]	DRAM_DQ;
output					DRAM_LDQM;
output					DRAM_RAS_N;
output					DRAM_UDQM;
output					DRAM_WE_N;

// I2C Bus for Configuration of the Audio and Video-In Chips
output					FPGA_I2C_SCLK;
inout						FPGA_I2C_SDAT;

// 40-pin headers
inout			[35: 0]	GPIO_0;
inout			[35: 0]	GPIO_1;

// Seven Segment Displays
output		[ 6: 0]	HEX0;
output		[ 6: 0]	HEX1;
output		[ 6: 0]	HEX2;
output		[ 6: 0]	HEX3;
output		[ 6: 0]	HEX4;
output		[ 6: 0]	HEX5;

// IR
input						IRDA_RXD;
output					IRDA_TXD;

// Pushbuttons
input			[ 3: 0]	KEY;

// LEDs
output		[ 9: 0]	LEDR;

// PS2 Ports
inout						PS2_CLK;
inout						PS2_DAT;

inout						PS2_CLK2;
inout						PS2_DAT2;

// Slider Switches
input			[ 9: 0]	SW;

// Video-In
input						TD_CLK27;
input			[ 7: 0]	TD_DATA;
input						TD_HS;
output					TD_RESET_N;
input						TD_VS;

// VGA
output		[ 7: 0]	VGA_B;
output					VGA_BLANK_N;
output					VGA_CLK;
output		[ 7: 0]	VGA_G;
output					VGA_HS;
output		[ 7: 0]	VGA_R;
output					VGA_SYNC_N;
output					VGA_VS;



////////////////////////////////////
// HPS Pins
////////////////////////////////////
	
// DDR3 SDRAM
output		[14: 0]	HPS_DDR3_ADDR;
output		[ 2: 0]  HPS_DDR3_BA;
output					HPS_DDR3_CAS_N;
output					HPS_DDR3_CKE;
output					HPS_DDR3_CK_N;
output					HPS_DDR3_CK_P;
output					HPS_DDR3_CS_N;
output		[ 3: 0]	HPS_DDR3_DM;
inout			[31: 0]	HPS_DDR3_DQ;
inout			[ 3: 0]	HPS_DDR3_DQS_N;
inout			[ 3: 0]	HPS_DDR3_DQS_P;
output					HPS_DDR3_ODT;
output					HPS_DDR3_RAS_N;
output					HPS_DDR3_RESET_N;
input						HPS_DDR3_RZQ;
output					HPS_DDR3_WE_N;

// Ethernet
output					HPS_ENET_GTX_CLK;
inout						HPS_ENET_INT_N;
output					HPS_ENET_MDC;
inout						HPS_ENET_MDIO;
input						HPS_ENET_RX_CLK;
input			[ 3: 0]	HPS_ENET_RX_DATA;
input						HPS_ENET_RX_DV;
output		[ 3: 0]	HPS_ENET_TX_DATA;
output					HPS_ENET_TX_EN;

// Flash
inout			[ 3: 0]	HPS_FLASH_DATA;
output					HPS_FLASH_DCLK;
output					HPS_FLASH_NCSO;

// Accelerometer
inout						HPS_GSENSOR_INT;

// General Purpose I/O
inout			[ 1: 0]	HPS_GPIO;

// I2C
inout						HPS_I2C_CONTROL;
inout						HPS_I2C1_SCLK;
inout						HPS_I2C1_SDAT;
inout						HPS_I2C2_SCLK;
inout						HPS_I2C2_SDAT;

// Pushbutton
inout						HPS_KEY;

// LED
inout						HPS_LED;

// SD Card
output					HPS_SD_CLK;
inout						HPS_SD_CMD;
inout			[ 3: 0]	HPS_SD_DATA;

// SPI
output					HPS_SPIM_CLK;
input						HPS_SPIM_MISO;
output					HPS_SPIM_MOSI;
inout						HPS_SPIM_SS;

// UART
input						HPS_UART_RX;
output					HPS_UART_TX;

// USB
inout						HPS_CONV_USB_N;
input						HPS_USB_CLKOUT;
inout			[ 7: 0]	HPS_USB_DATA;
input						HPS_USB_DIR;
input						HPS_USB_NXT;
output					HPS_USB_STP;

//=======================================================
//  REG/WIRE declarations
//=======================================================

// audio input/output from audio module FIFO
reg signed [31:0] right_audio_input, left_audio_input ;
reg audio_input_ready ;
//wire [15:0] right_audio_output, left_audio_output ;
wire [31:0] audio_data_left_address = 32'h00003048 ;  // Avalon address +8
wire [31:0] audio_data_right_address = 32'h0000304c ;  // Avalon address +12

wire			[15: 0]	hex3_hex0;
//wire			[15: 0]	hex5_hex4;

//assign HEX0 = ~hex3_hex0[ 6: 0]; // hex3_hex0[ 6: 0];
//assign HEX1 = ~hex3_hex0[14: 8];
//assign HEX2 = ~hex3_hex0[22:16];
//assign HEX3 = ~hex3_hex0[30:24];
assign HEX4 = 7'b1111111;
assign HEX5 = 7'b1111111;
assign HEX3 = 7'b1111111;
assign HEX2 = 7'b1111111;
assign HEX1 = 7'b1111111;
assign HEX0 = 7'b1111111;

//HexDigit Digit0(HEX0, hex3_hex0[3:0]);
//HexDigit Digit1(HEX1, hex3_hex0[7:4]);
//HexDigit Digit2(HEX2, hex3_hex0[11:8]);
//HexDigit Digit3(HEX3, hex3_hex0[15:12]);

//=======================================================
// Bus controller for AVALON bus-master
//=======================================================
// computes DDS for sine wave and fills audio FIFO

reg [31:0] bus_addr ; // Avalon address
// see
// ftp://ftp.altera.com/up/pub/Altera_Material/15.1/University_Program_IP_Cores/Audio_Video/Audio.pdf
// for addresses
wire [31:0] audio_base_address = 32'h00003040 ;  // Avalon address
wire [31:0] audio_fifo_address = 32'h00003044 ;  // Avalon address +4 offset
wire [31:0] audio_left_address = 32'h00003048 ;  // Avalon address +8
wire [31:0] audio_right_address = 32'h0000304c ;  // Avalon address +12
reg [3:0] bus_byte_enable ; // four bit byte read/write mask
reg bus_read  ;       // high when requesting data
reg bus_write ;      //  high when writing data
reg [31:0] bus_write_data ; //  data to send to Avalog bus
wire bus_ack  ;       //  Avalon bus raises this when done
wire [31:0] bus_read_data ; // data from Avalon bus
reg [30:0] audio_timer ;
reg [7:0] audio_state ;
wire state_clock ;

// current free words in audio interface
reg [7:0] fifo_space ;
// debug check of space
//assign LEDR = fifo_space ;

// get some signals exposed
// connect bus master signals to i/o for probes
assign GPIO_0[0] = bus_write ;
assign GPIO_0[1] = bus_read ;
assign GPIO_0[2] = bus_ack ;
//assign GPIO_0[3] = ??? ;

//shifting audio sample array (array length = 21)
reg signed [26:0] sample_array [20:0];
//FIR weights array
reg signed [26:0] weights1 [num_samples-1:0];

reg [1:0] start ; 
reg [31:0] counter ;
//assign LEDR = counter;

always @(posedge CLOCK_50) begin //CLOCK_50
	// reset state machine and read/write controls
	if (~KEY[0]) begin
		counter <= 0;
		start <= 1;
		audio_state <= 0 ;
		bus_read <= 0 ; // set to one if a read opeation from bus
		bus_write <= 0 ; // set to one if a write operation to bus
		audio_timer <= 0;

		sample_array[20] <= 27'b0;
		sample_array[19] <= 27'b0;
		sample_array[18] <= 27'b0;
		sample_array[17] <= 27'b0;
		sample_array[16] <= 27'b0;
		sample_array[15] <= 27'b0;
		sample_array[14] <= 27'b_0;
		sample_array[13] <= 27'b_0;
		sample_array[12] <= 27'b_0;
		sample_array[11] <= 27'b_0;
		sample_array[10] <= 27'b_0;
		sample_array[9]  <= 27'b_0;
		sample_array[8]  <= 27'b_0;
		sample_array[7]  <= 27'b_0;
		sample_array[6]  <= 27'b_0;
		sample_array[5]  <= 27'b_0;
		sample_array[4]  <= 27'b_0;
		sample_array[3]  <= 27'b_0;
		sample_array[2]  <= 27'b_0;
		sample_array[1]  <= 27'b_0;
		sample_array[0]  <= 27'b_0;

	end
	else begin
		// timer just for deubgging
		audio_timer <= audio_timer + 1;
		if (start==1) begin
			counter <= counter + 10;
		end
	end

	// === writing stereo to the audio FIFO ==========
	// set up read FIFO available space
	if (audio_state==4'd0) begin
		bus_addr <= audio_fifo_address ;
		bus_read <= 1'b1 ;
		bus_byte_enable <= 4'b1111;
		audio_state <= 4'd1 ; // wait for read ACK
	end

	// wait for read ACK and read the fifo available
	// bus ACK is high when data is available
	if (audio_state==4'd1 && bus_ack==1) begin
		audio_state <= 4'd2 ; //4'd2
		// FIFO write space is in high byte
		fifo_space <= (bus_read_data>>24) ;
		// end the read
		bus_read <= 1'b0 ;
	end

	// When there is room in the FIFO
	// -- start write to fifo for each channel
	// -- first the left channel
	if (audio_state==4'd2 && fifo_space>8'd2) begin //
		audio_state <= 4'd3;
		bus_write_data <= left_audio_input ;
		bus_addr <= audio_data_left_address ;
		bus_byte_enable <= 4'b1111;
		bus_write <= 1'b1 ;
	end
	// if no space, try again later
	else if (audio_state==4'd2 && fifo_space<=8'd2) begin
		audio_state <= 4'b0 ;
	end

	// detect bus-transaction-complete ACK
	// for left channel write
	// You MUST do this check
	if (audio_state==4'd3 && bus_ack==1) begin
		audio_state <= 4'd4 ; // include right channel
		//state <= 4'd0 ; // left channel only!
		bus_write <= 0;
	end

	// -- now the right channel
	if (audio_state==4'd4) begin //
		audio_state <= 4'd5;
		// loop back audio input data
		bus_write_data <= right_audio_input ;

		//shift audio sample array
		sample_array[20]<= sample_array[19];
		sample_array[19] <= sample_array[18];
		sample_array[18] <= sample_array[17];
		sample_array[17] <= sample_array[16];
		sample_array[16] <= sample_array[15];
		sample_array[15] <= sample_array[14];
		sample_array[14] <= sample_array[13];
		sample_array[13] <= sample_array[12];
		sample_array[12] <= sample_array[11];
		sample_array[11] <= sample_array[10];
		sample_array[10]<= sample_array[9];
		sample_array[9] <= sample_array[8];
		sample_array[8] <= sample_array[7];
		sample_array[7] <= sample_array[6];
		sample_array[6] <= sample_array[5];
		sample_array[5] <= sample_array[4];
		sample_array[4] <= sample_array[3];
		sample_array[3] <= sample_array[2];
		sample_array[2] <= sample_array[1];
		sample_array[1] <= sample_array[0];
		sample_array[0] <= (right_audio_input[26:0]) ;

		bus_addr <= audio_data_right_address ;
		bus_write <= 1'b1 ;
		
		
	end

	// detect bus-transaction-complete ACK
	// for right channel write
	// You MUST do this check
	if (audio_state==4'd5 && bus_ack==1) begin
		// state <= 4'd0 ; // for write only function
		audio_state <= 4'd6 ; // for read/write  function
		bus_write <= 0;
	end

	// === reading stereo from the audio FIFO ==========
	// set up read FIFO for available read values
	if (audio_state==4'd6 ) begin
		bus_addr <= audio_fifo_address ;
		bus_read <= 1'b1 ;
		bus_byte_enable <= 4'b1111;
		audio_state <= 4'd7 ; // wait for read ACK
	end

	// wait for read ACK and read the fifo available
	// bus ACK is high when data is available
	if (audio_state==4'd7 && bus_ack==1) begin
		audio_state <= 4'd8 ; //4'dxx
		// FIFO read space is in low byte
		// which is zero when empty
		fifo_space <= bus_read_data & 8'hff ;
		// end the read
		bus_read <= 1'b0 ;
	end

	// When there is data in the read FIFO
	// -- read it from both channels
	// -- first the left channel
	if (audio_state==4'd8 && fifo_space>8'd0) begin //
		audio_state <= 4'd9;
		bus_addr <= audio_data_left_address ;
		bus_byte_enable <= 4'b1111;
		bus_read <= 1'b1 ;
	end
	// if no data, try again later
	else if (audio_state==4'd8 && fifo_space<=8'd0) begin
		audio_state <= 4'b0 ;
	end

	// detect bus-transaction-complete ACK
	// for left channel read
	// You MUST do this check
	if (audio_state==4'd9 && bus_ack==1) begin
		audio_state <= 4'd10 ; // include right channel
		left_audio_input <= bus_read_data; //((bus_read_data < 0) && SW[2]) ?
									//(0 - bus_read_data) : bus_read_data;
		bus_read <= 0;
	end

	// When there is data in the read FIFO
	// -- read it from both channels
	// -- now right channel
	if (audio_state==4'd10) begin //
		audio_state <= 4'd11;
		bus_addr <= audio_data_right_address ;
		bus_byte_enable <= 4'b1111;
		bus_read <= 1'b1 ;
	end

	// detect bus-transaction-complete ACK
	// for right channel read
	// You MUST do this check
	if (audio_state==4'd11 && bus_ack==1) begin
		audio_state <= 4'd12 ; // back to beginning
		right_audio_input <= bus_read_data;//((bus_read_data < 0) && SW[2]) ?
									//(0 - bus_read_data) : bus_read_data;
		// set the data-ready strobe
		audio_input_ready <= 1'b1;
		bus_read <= 0;
	end

	// wait 1 cycle data-ready strobe
	if (audio_state==4'd12) begin
		audio_state <= 4'd13 ; // back to beginning
		//audio_input_ready <= 1'b0;
	end
	// end data-ready strobe
	if (audio_state==4'd13) begin
		start <= 0;
		audio_state <= 4'd0 ; // back to beginning
		audio_input_ready <= 1'b0;
	end

end // always @(posedge state_clock)

wire signed [31:0] right_audio_input_export, debug1, debug2, debug3;

assign right_audio_input_export = right_audio_input;
//assign debug1 = bus_ack;
assign debug2 = fifo_space;
assign debug3 = audio_state;


//=======================================================
// SRAM/VGA state machine
//=======================================================
// --Check for sram address=0 nonzero, which means that
//   HPS wrote some new data.
//
// --Read sram address 1 and 2 to get x1, y1
//   left-most x, upper-most y
// --Read sram address 3 and 4 to get x2, y2
//   right-most x, lower-most y
// --Read sram address 5 to get color
// --write a rectangle to VGA
//
// --clear sram address=0 to signal HPS
//=======================================================
// Controls for Qsys sram slave exported in system module
//=======================================================

wire [31:0] sram_readdata ;
reg [31:0] data_buffer, sram_writedata ;
reg [7:0] sram_address;
reg sram_write ;
wire sram_clken = 1'b1;
wire sram_chipselect = 1'b1;
reg [7:0] state ;

// rectangle corners
reg [9:0] x1, y1, x2, y2 ;
reg [31:0] timer ; // may need to throttle write-rate
//=======================================================
// Controls for VGA memory
//=======================================================
wire [31:0] vga_out_base_address = 32'h0000_0000 ;  // vga base addr
reg [7:0] vga_sram_writedata ;
reg [31:0] vga_sram_address;
reg vga_sram_write ;
wire vga_sram_clken = 1'b1;
wire vga_sram_chipselect = 1'b1;

//=======================================================
// pixel address is
reg [9:0] vga_x_cood, vga_y_cood ;
reg [7:0] pixel_color ;

//=======================================================
//pio ports
reg [31:0] mode;

always @(posedge CLOCK_50) begin // CLOCK_50
   // reset state machine and read/write controls
	if (~KEY[0]) begin
		state <= 0 ;
		vga_sram_write <= 1'b0 ; // set to on if a write operation to bus
		sram_write <= 1'b0 ;
		timer <= 0;
	end
	else begin
		// general purpose tick counter
		timer <= timer + 1;
	end

	if (~KEY[1]) begin
		mode <= 2'd0;
	end

	if (~KEY[2]) begin
		mode <= 2'd1;
	end

	if (~KEY[3]) begin
		mode <= 2'd2;
	end

	// --------------------------------------
	// did the HPS send a command
	// --- set up read for HPS data-ready ---
	if (state == 8'd0) begin
		sram_address <= 8'd0 ;
		sram_write <= 1'b0 ;
		state <= 8'd1 ;
	end
	// wait 1 for read
	if (state == 8'd1) begin
		state <= 8'd2 ;
	end
	// do data-read read
	if (state == 8'd2) begin
		data_buffer <= sram_readdata ;
		sram_write <= 1'b0 ;
		state <= 8'd3 ;
	end

	// --------------------------------------
	// --- is there new command? ---
	if (state == 8'd3) begin
		// if (addr 0)==0 try again
		if (data_buffer==0) state <= 8'd0 ;
		// if nonzero, do the add
		else state <= 8'd4 ;
	end

	// --------------------------------------
	// --- read first Qsys sram: x1 ---
	if (state == 8'd4) begin
		sram_address <= 8'd1 ;
		sram_write <= 1'b0 ;
		state <= 8'd5 ;
	end
	// wait 1
	if (state == 8'd5) begin
		state <= 8'd6 ;
	end
	// do data-read x1
	if (state == 8'd6) begin
		x1 <= sram_readdata ;
		sram_write <= 1'b0 ;
		state <= 8'd7 ;
	end

	// --------------------------------------
	// --- read second Qsys sram: y1 ---
	if (state == 8'd7) begin
		sram_address <= 8'd2 ;
		sram_write <= 1'b0 ;
		state <= 8'd8 ;
	end
	// wait 1
	if (state == 8'd8) begin
		state <= 8'd9 ;
	end
	// do data-read y1
	if (state == 8'd9) begin
		y1 <= sram_readdata ;
		sram_write <= 1'b0 ;
		state <= 8'd10 ;
	end

	// --------------------------------------
	// --- read third Qsys sram: x2 ---
	if (state == 8'd10) begin
		sram_address <= 8'd3 ;
		sram_write <= 1'b0 ;
		state <= 8'd11 ;
	end
	// wait 1
	if (state == 8'd11) begin
		state <= 8'd12 ;
	end
	// do data-read x2
	if (state == 8'd12) begin
		x2 <= sram_readdata ;
		sram_write <= 1'b0 ;
		state <= 8'd13 ;
	end

	// --------------------------------------
	// --- read fourth Qsys sram: y2 ---
	if (state == 8'd13) begin
		sram_address <= 8'd4 ;
		sram_write <= 1'b0 ;
		state <= 8'd14 ;
	end
	// wait 1
	if (state == 8'd14) begin
		state <= 8'd15 ;
	end
	// do data-read y2
	if (state == 8'd15) begin
		y2 <= sram_readdata ;
		sram_write <= 1'b0 ;
		state <= 8'd16 ;
	end

	// --------------------------------------
	// --- read fifth Qsys sram: color ---
	if (state == 8'd16) begin
		sram_address <= 8'd5 ;
		sram_write <= 1'b0 ;
		state <= 8'd17 ;
	end
	// wait 1
	if (state == 8'd17) begin
		state <= 8'd18 ;
	end
	// do data-read y2
	if (state == 8'd18) begin
		pixel_color <= sram_readdata ;
		sram_write <= 1'b0 ;
		state <= 8'd19 ;
		// initialize pixel state machine
		// for the next phase of the state machine
		vga_x_cood <= x1 ;
		vga_y_cood <= y1 ;
	end


	// --------------------------------------
	// Now have all info, so:
	// write to the VGA sram

	if (state==8'd19) begin // && ((timer & 15)==0)
		vga_sram_write <= 1'b1;
		// compute address
	   vga_sram_address <= vga_out_base_address + {22'b0, vga_x_cood} + ({22'b0,vga_y_cood}*640) ;
		// data
		vga_sram_writedata <= pixel_color  ;
		// iterate through all x for each y in the list
		if (vga_x_cood < x2) begin
			vga_x_cood <= vga_x_cood + 10'd1 ;
		end
		else begin
			vga_x_cood <= x1 ;
			vga_y_cood <= vga_y_cood + 10'd1 ;
		end
		if (vga_x_cood>=x2 && vga_y_cood>=y2) state <= 8'd22 ; // ending
		else state  <= 8'd19 ;
		// write a point
		state <= 8'd22 ;
	end

	// write a pixel to VGA memory
	if (state==20) begin
		vga_sram_write <= 1'b1;
		// vga_sram_address is combinatorial;
		vga_sram_writedata <= pixel_color  ;
		// done?
		if (vga_x_cood>=x2 && vga_y_cood>=y2) state <= 8'd22 ; // ending
		else state  <= 8'd19 ;
	end


	// -- finished: --
	// -- set up done flag to Qsys sram 0 ---
	if (state == 8'd22) begin
		// end vga write
		vga_sram_write <= 1'b0;
		// signal the HPS we are done
		sram_address <= 8'd0 ;
		sram_writedata <= 32'b0 ;
		sram_write <= 1'b1 ;
		state <= 8'd0 ;
	end

end // always @(posedge state_clock)

assign debug1 = mode; //assign value to boid boundaries

//=======================================================
// FIR filtering for audio samples
//=======================================================

localparam num_samples = 5'd21;

///making weights arrays
reg signed [26:0] weights2 [num_samples-1:0];
reg signed [26:0] weights3 [num_samples-1:0];
reg signed [26:0] weights4 [num_samples-1:0];
reg signed [26:0] weights5 [num_samples-1:0];

always @(*) begin
	weights1[0] = -27'sb_0000_0000_0000__0000_0010_1010_110;
	weights1[1] = -27'sb_0000_0000_0000__0000_0011_0100_110;
	weights1[2] = -27'sb_0000_0000_0000__0000_0100_1001_011;
	weights1[3] = -27'sb_0000_0000_0000__0000_0101_0010_110;
	weights1[4] = -27'sb_0000_0000_0000__0000_0011_0100_110;
	weights1[5] = 27'b_0000_0000_0000__0000_0010_0101_010;
	weights1[6] = 27'b_0000_0000_0000__0000_1011_1101_100;
	weights1[7] = 27'b_0000_0000_0000__0001_0111_1101_110;
	weights1[8] = 27'b_0000_0000_0000__0010_0011_1100_010;
	weights1[9] = 27'b_0000_0000_0000__00101_100_1000_100;
	weights1[10] = 27'b_0000_0000_0000__0010_1111_1100_001;
	weights1[11] = 27'b_0000_0000_0000__00101_100_1000_100;
	weights1[12] = 27'b_0000_0000_0000__0010_0011_1100_010;
	weights1[13] = 27'b_0000_0000_0000__0001_0111_1101_110;
	weights1[14] = 27'b_0000_0000_0000__0000_1011_1101_100;
	weights1[15] = 27'b_0000_0000_0000__0000_0010_0101_010;
	weights1[16] = -27'sb_0000_0000_0000__0000_0011_0100_110;
	weights1[17] = -27'sb_0000_0000_0000__0000_0101_0010_110;
	weights1[18] = -27'sb_0000_0000_0000__0000_0100_1001_011;
	weights1[19] = -27'sb_0000_0000_0000__0000_0011_0100_110;
	weights1[20] = -27'sb_0000_0000_0000__0000_0010_1010_110;

	weights2[0] = 27'b_0000_0000_0000__0000_0010_0101_101;
	weights2[1] = 27'b_0000_0000_0000__0000_0001_1000_111;
	weights2[2] = -27'sb_0000_0000_0000__0000_0001_0000_111;
	weights2[3] = -27'sb_0000_0000_0000__0000_0111_1001_010;
	weights2[4] = -27'sb_0000_0000_0000__0001_0000_1110_110;
	weights2[5] = -27'sb_0000_0000_0000__0001_0111_1010_110;
	weights2[6] = -27'sb_0000_0000_0000__0001_0100_1110_011;
	weights2[7] = -27'sb_0000_0000_0000__0000_0101_0101_101;
	weights2[8] = 27'b_0000_0000_0000__0001_0010_1011_011;
	weights2[9] = 27'b_0000_0000_0000__0010_1000_1011_110;
	weights2[10] = 27'b_0000_0000_0000__0011_0001_1010_001;
	weights2[11] = 27'b_0000_0000_0000__0010_1000_1011_110;
	weights2[12] = 27'b_0000_0000_0000__0001_0010_1011_011;
	weights2[13] = -27'sb_0000_0000_0000__0000_0101_0101_101;
	weights2[14] = -27'sb_0000_0000_0000__0001_0100_1110_011;
	weights2[15] = -27'sb_0000_0000_0000__0001_0111_1010_110;
	weights2[16] = -27'sb_0000_0000_0000__0001_0000_1110_110;
	weights2[17] = -27'sb_0000_0000_0000__0000_0111_1001_010;
	weights2[18] = -27'sb_0000_0000_0000__0000_0001_0000_111;
	weights2[19] = 27'b_0000_0000_0000__0000_0001_1000_111;
	weights2[20] = 27'b_0000_0000_0000__0000_0010_0101_101;

	weights3[0] = -27'sb_0000_0000_0000__0000_0001_1001_010;
	weights3[1] = 27'b_0000_0000_0000__0000_0000_1111_110;
	weights3[2] = 27'b_0000_0000_0000__0000_0110_0000_101;
	weights3[3] = 27'b_0000_0000_0000__0000_1010_1010_101;
	weights3[4] = 27'b_0000_0000_0000__0000_0110_0001_110;
	weights3[5] = -27'sb_0000_0000_0000__0000_1011_1011_010;
	weights3[6] = -27'sb_0000_0000_0000__0001_1111_0110_100;
	weights3[7] = -27'sb_0000_0000_0000__00100_000_0011_001;
	weights3[8] = -27'sb_0000_0000_0000__0000_0101_0100_011;
	weights3[9] = 27'b_0000_0000_0000__0010_0000_0100_000;
	weights3[10] = 27'b_0000_0000_0000__0011_0001_1101_010;
	weights3[11] = 27'b_0000_0000_0000__0010_0000_0100_000;
	weights3[12] = -27'sb_0000_0000_0000__0000_0101_0100_011;
	weights3[13] = -27'sb_0000_0000_0000__00100_000_0011_001;
	weights3[14] = -27'sb_0000_0000_0000__0001_1111_0110_100;
	weights3[15] = -27'sb_0000_0000_0000__0000_1011_1011_010;
	weights3[16] = 27'b_0000_0000_0000__0000_0110_0001_110;
	weights3[17] = 27'b_0000_0000_0000__0000_1010_1010_101;
	weights3[18] = 27'b_0000_0000_0000__0000_0110_0000_101;
	weights3[19] = 27'b_0000_0000_0000__0000_0000_1111_110;
	weights3[20] = -27'sb_0000_0000_0000__0000_0001_1001_010;

	weights4[0] = 27'b_0000_0000_0000__0000_0000_1000_111;
	weights4[1] = -27'sb_0000_0000_0000__0000_0011_0001_110;
	weights4[2] = -27'sb_0000_0000_0000__0000_0110_0000_101;
	weights4[3] = 27'b_0000_0000_0000__0000_0000_0011_100;
	weights4[4] = 27'b_0000_0000_0000__0001_0000_0000_100;
	weights4[5] = 27'b_0000_0000_0000__0001_0011_0011_010;
	weights4[6] = -27'sb_0000_0000_0000__0000_0111_1000_101;
	weights4[7] = -27'sb_0000_0000_0000__0010_0110_0101_100;
	weights4[8] = -27'sb_0000_0000_0000__0001_1011_1100_110;
	weights4[9] = 27'b_0000_0000_0000__0001_0101_0011_001;
	weights4[10] = 27'b_0000_0000_0000__0011_0001_1101_100;
	weights4[11] = 27'b_0000_0000_0000__0001_0101_0011_001;
	weights4[12] = -27'sb_0000_0000_0000__0001_1011_1100_110;
	weights4[13] = -27'sb_0000_0000_0000__0010_0110_0101_100;
	weights4[14] = -27'sb_0000_0000_0000__0000_0111_1000_101;
	weights4[15] = 27'b_0000_0000_0000__0001_0011_0011_010;
	weights4[16] = 27'b_0000_0000_0000__0001_0000_0000_100;
	weights4[17] = 27'b_0000_0000_0000__0000_0000_0011_100;
	weights4[18] = -27'sb_0000_0000_0000__0000_0110_0000_101;
	weights4[19] = -27'sb_0000_0000_0000__0000_0011_0001_110;
	weights4[20] = 27'b_0000_0000_0000__0000_0000_1000_111;

	//originally, weights5[0] = 5.545776377261806e-18; <- too small so changed to 0
	weights5[0] = 27'b_0;
	weights5[1] = 27'b_0000_0000_0000__0000_0011_1001_010;
	weights5[2] = 27'b_0000_0000_0000__0000_0001_1111_110;
	weights5[3] = -27'sb_0000_0000_0000__0000_1001_1101_100;
	weights5[4] = -27'sb_0000_0000_0000__0000_1010_0010_011;
	weights5[5] = 27'b_0000_0000_0000__0001_0001_0110_001;
	weights5[6] = 27'b_0000_0000_0000__0001_1010_0001_110;
	weights5[7] = -27'sb_0000_0000_0000__0001_0001_1110_110;
	weights5[8] = -27'sb_0000_0000_0000__0010_1011_0010_100;
	weights5[9] = 27'b_0000_0000_0000__0000_0111_1011_001;
	weights5[10] = 27'b_0000_0000_0000__0011_0010_1001_010;
	weights5[11] = 27'b_0000_0000_0000__0000_0111_1011_001;
	weights5[12] = -27'sb_0000_0000_0000__0010_1011_0010_100;
	weights5[13] = -27'sb_0000_0000_0000__0001_0001_1110_110;
	weights5[14] = 27'b_0000_0000_0000__0001_1010_0001_110;
	weights5[15] = 27'b_0000_0000_0000__0001_0001_0110_001;
	weights5[16] = -27'sb_0000_0000_0000__0000_1010_0010_011;
	weights5[17] = -27'sb_0000_0000_0000__0000_1001_1101_100;
	weights5[18] = 27'b_0000_0000_0000__0000_0001_1111_110;
	weights5[19] = 27'b_0000_0000_0000__0000_0011_1001_010;
	weights5[20] = 27'b_0;

end

///making output arrays
wire signed [26:0] fir_out1 [num_samples-1:0];
wire signed [26:0] fir_out2 [num_samples-1:0];
wire signed [26:0] fir_out3 [num_samples-1:0];
wire signed [26:0] fir_out4 [num_samples-1:0];
wire signed [26:0] fir_out5 [num_samples-1:0];

///making output freq
wire signed [31:0] freq1 ;
wire signed [31:0] freq2 ;
wire signed [31:0] freq3 ;
wire signed [31:0] freq4 ;
wire signed [31:0] freq5 ;

wire signed [31:0] freq11 ;
wire signed [31:0] freq12 ;
wire signed [31:0] freq13 ;
wire signed [31:0] freq14 ;
wire signed [31:0] freq15 ;


//calculating filtered output
// repeat 11 times via generate per filter
genvar i;
generate
	for (i = 0; i < 21; i = i + 1) begin: mult1
		signed_mult mult1(
			.out(fir_out1[i]),
			.a(sample_array[i]),
			.b(weights1[i])
		);

	end
endgenerate

assign freq1 = fir_out1[0] +  fir_out1[1] +  fir_out1[2] +  fir_out1[3] +  fir_out1[4] +  fir_out1[5] +  fir_out1[6] +  fir_out1[7] +  fir_out1[8] +  fir_out1[9] +  fir_out1[10] +  fir_out1[11] +  fir_out1[12] +  fir_out1[13] +  fir_out1[14] +  fir_out1[15] +  fir_out1[16] +  fir_out1[17] +  fir_out1[18] +  fir_out1[19] +  fir_out1[20];
//assign debug3 = sample_array[0];


genvar j;
generate
	for (j = 0; j < 21; j = j + 1) begin: mult2

		signed_mult mult2(
			.out(fir_out2[j]),
			.a(sample_array[j]),
			.b(weights2[j])
		);

	end
endgenerate

assign freq2 = fir_out2[0] +  fir_out2[1] +  fir_out2[2] +  fir_out2[3] +  fir_out2[4] +  fir_out2[5] +  fir_out2[6] +  fir_out2[7] +  fir_out2[8] +  fir_out2[9] +  fir_out2[10] +  fir_out2[11] +  fir_out2[12] +  fir_out2[13] +  fir_out2[14] +  fir_out2[15] +  fir_out2[16] +  fir_out2[17] +  fir_out2[18] +  fir_out2[19] +  fir_out2[20];



genvar k;
generate
	for (k = 0; k < 21; k = k + 1) begin: mult3

		signed_mult mult3(
			.out(fir_out3[k]),
			.a(sample_array[k]),
			.b(weights3[k])
		);

	end
endgenerate

assign freq3 = fir_out3[0] +  fir_out3[1] +  fir_out3[2] +  fir_out3[3] +  fir_out3[4] +  fir_out3[5] +  fir_out3[6] +  fir_out3[7] +  fir_out3[8] +  fir_out3[9] +  fir_out3[10] +  fir_out3[11] +  fir_out3[12] +  fir_out3[13] +  fir_out3[14] +  fir_out3[15] +  fir_out3[16] +  fir_out3[17] +  fir_out3[18] +  fir_out3[19] +  fir_out3[20];



genvar l;
generate
	for (l = 0; l < 21; l = l + 1) begin: mult4

		signed_mult mult4(
			.out(fir_out4[l]),
			.a(sample_array[l]),
			.b(weights4[l])
		);

	end
endgenerate

assign freq4 = fir_out4[0] +  fir_out4[1] +  fir_out4[2] +  fir_out4[3] +  fir_out4[4] +  fir_out4[5] +  fir_out4[6] +  fir_out4[7] +  fir_out4[8] +  fir_out4[9] +  fir_out4[10] + fir_out4[11] +  fir_out4[12] +  fir_out4[13] +  fir_out4[14] +  fir_out4[15] +  fir_out4[16] +  fir_out4[17] +  fir_out4[18] +  fir_out4[19] +  fir_out4[20];


genvar m;
generate
	for (m = 0; m < 21; m = m + 1) begin: mult5

		signed_mult mult5(
			.out(fir_out5[m]),
			.a(sample_array[m]),
			.b(weights5[m])
		);

	end
endgenerate

assign freq5 = fir_out5[0] +  fir_out5[1] +  fir_out5[2] +  fir_out5[3] +  fir_out5[4] +  fir_out5[5] +  fir_out5[6] +  fir_out5[7] +  fir_out5[8] +  fir_out5[9] +  fir_out5[10] +  fir_out5[11] +  fir_out5[12] +  fir_out5[13] +  fir_out5[14] +  fir_out5[15] +  fir_out5[16] +  fir_out5[17] +  fir_out5[18] +  fir_out5[19] +  fir_out5[20];


//=======================================================
//  Structural coding
//=======================================================


Computer_System The_System (
	////////////////////////////////////
	// FPGA Side
	////////////////////////////////////
	
	//PIO ports
	.right_audio_input_external_connection_export(right_audio_input_export),
	.debug1_external_connection_export(debug1),
	.debug2_external_connection_export(debug2),
	.debug3_external_connection_export(debug3),
	.freq1_external_connection_export(freq1),
	.freq2_external_connection_export(freq2),
	.freq3_external_connection_export(freq3),
	.freq4_external_connection_export(freq4),
	.freq5_external_connection_export(freq5),
	.counter_external_connection_export(counter),
	
	// Audio Subsystem
	.audio_pll_ref_clk_clk					(CLOCK3_50),
	.audio_pll_ref_reset_reset				(1'b0),
	.audio_clk_clk								(AUD_XCK),
	.audio_ADCDAT								(AUD_ADCDAT),
	.audio_ADCLRCK								(AUD_ADCLRCK),
	.audio_BCLK									(AUD_BCLK),
	.audio_DACDAT								(AUD_DACDAT),
	.audio_DACLRCK								(AUD_DACLRCK),

	// bus-master state machine interface
	.bus_master_audio_external_interface_address     (bus_addr),
	.bus_master_audio_external_interface_byte_enable (bus_byte_enable),
	.bus_master_audio_external_interface_read        (bus_read),
	.bus_master_audio_external_interface_write       (bus_write),
	.bus_master_audio_external_interface_write_data  (bus_write_data),
	.bus_master_audio_external_interface_acknowledge (bus_ack),
	.bus_master_audio_external_interface_read_data   (bus_read_data),
	

	// Global signals
	.system_pll_ref_clk_clk					(CLOCK_50),
	.system_pll_ref_reset_reset			(1'b0),
	
	/*// SRAM shared block with HPS
	.onchip_sram_s1_address               (sram_address),
	.onchip_sram_s1_clken                 (sram_clken),
	.onchip_sram_s1_chipselect            (sram_chipselect),
	.onchip_sram_s1_write                 (sram_write),
	.onchip_sram_s1_readdata              (sram_readdata),
	.onchip_sram_s1_writedata             (sram_writedata),
	.onchip_sram_s1_byteenable            (4'b1111),

	//  sram to video
	.onchip_vga_buffer_s1_address    (vga_sram_address),
	.onchip_vga_buffer_s1_clken      (vga_sram_clken),
	.onchip_vga_buffer_s1_chipselect (vga_sram_chipselect),
	.onchip_vga_buffer_s1_write      (vga_sram_write),
	.onchip_vga_buffer_s1_readdata   (),   // never read from vga here
	.onchip_vga_buffer_s1_writedata  (vga_sram_writedata),*/

	// AV Config
	.av_config_SCLK							(FPGA_I2C_SCLK),
	.av_config_SDAT							(FPGA_I2C_SDAT),
	
	/*// 50 MHz clock bridge
	.clock_bridge_0_in_clk_clk            (CLOCK_50), //(CLOCK_50),*/

	// VGA Subsystem
	.vga_pll_ref_clk_clk 					(CLOCK2_50),
	.vga_pll_ref_reset_reset				(1'b0),
	.vga_CLK										(VGA_CLK),
	.vga_BLANK									(VGA_BLANK_N),
	.vga_SYNC									(VGA_SYNC_N),
	.vga_HS										(VGA_HS),
	.vga_VS										(VGA_VS),
	.vga_R										(VGA_R),
	.vga_G										(VGA_G),
	.vga_B										(VGA_B),
	
	// SDRAM
	.sdram_clk_clk								(DRAM_CLK),
   .sdram_addr									(DRAM_ADDR),
	.sdram_ba									(DRAM_BA),
	.sdram_cas_n								(DRAM_CAS_N),
	.sdram_cke									(DRAM_CKE),
	.sdram_cs_n									(DRAM_CS_N),
	.sdram_dq									(DRAM_DQ),
	.sdram_dqm									({DRAM_UDQM,DRAM_LDQM}),
	.sdram_ras_n								(DRAM_RAS_N),
	.sdram_we_n									(DRAM_WE_N),
	
	////////////////////////////////////
	// HPS Side
	////////////////////////////////////
	// DDR3 SDRAM
	.memory_mem_a			(HPS_DDR3_ADDR),
	.memory_mem_ba			(HPS_DDR3_BA),
	.memory_mem_ck			(HPS_DDR3_CK_P),
	.memory_mem_ck_n		(HPS_DDR3_CK_N),
	.memory_mem_cke		(HPS_DDR3_CKE),
	.memory_mem_cs_n		(HPS_DDR3_CS_N),
	.memory_mem_ras_n		(HPS_DDR3_RAS_N),
	.memory_mem_cas_n		(HPS_DDR3_CAS_N),
	.memory_mem_we_n		(HPS_DDR3_WE_N),
	.memory_mem_reset_n	(HPS_DDR3_RESET_N),
	.memory_mem_dq			(HPS_DDR3_DQ),
	.memory_mem_dqs		(HPS_DDR3_DQS_P),
	.memory_mem_dqs_n		(HPS_DDR3_DQS_N),
	.memory_mem_odt		(HPS_DDR3_ODT),
	.memory_mem_dm			(HPS_DDR3_DM),
	.memory_oct_rzqin		(HPS_DDR3_RZQ),
		  
	// Ethernet
	.hps_io_hps_io_gpio_inst_GPIO35	(HPS_ENET_INT_N),
	.hps_io_hps_io_emac1_inst_TX_CLK	(HPS_ENET_GTX_CLK),
	.hps_io_hps_io_emac1_inst_TXD0	(HPS_ENET_TX_DATA[0]),
	.hps_io_hps_io_emac1_inst_TXD1	(HPS_ENET_TX_DATA[1]),
	.hps_io_hps_io_emac1_inst_TXD2	(HPS_ENET_TX_DATA[2]),
	.hps_io_hps_io_emac1_inst_TXD3	(HPS_ENET_TX_DATA[3]),
	.hps_io_hps_io_emac1_inst_RXD0	(HPS_ENET_RX_DATA[0]),
	.hps_io_hps_io_emac1_inst_MDIO	(HPS_ENET_MDIO),
	.hps_io_hps_io_emac1_inst_MDC		(HPS_ENET_MDC),
	.hps_io_hps_io_emac1_inst_RX_CTL	(HPS_ENET_RX_DV),
	.hps_io_hps_io_emac1_inst_TX_CTL	(HPS_ENET_TX_EN),
	.hps_io_hps_io_emac1_inst_RX_CLK	(HPS_ENET_RX_CLK),
	.hps_io_hps_io_emac1_inst_RXD1	(HPS_ENET_RX_DATA[1]),
	.hps_io_hps_io_emac1_inst_RXD2	(HPS_ENET_RX_DATA[2]),
	.hps_io_hps_io_emac1_inst_RXD3	(HPS_ENET_RX_DATA[3]),

	// Flash
	.hps_io_hps_io_qspi_inst_IO0	(HPS_FLASH_DATA[0]),
	.hps_io_hps_io_qspi_inst_IO1	(HPS_FLASH_DATA[1]),
	.hps_io_hps_io_qspi_inst_IO2	(HPS_FLASH_DATA[2]),
	.hps_io_hps_io_qspi_inst_IO3	(HPS_FLASH_DATA[3]),
	.hps_io_hps_io_qspi_inst_SS0	(HPS_FLASH_NCSO),
	.hps_io_hps_io_qspi_inst_CLK	(HPS_FLASH_DCLK),

	// Accelerometer
	.hps_io_hps_io_gpio_inst_GPIO61	(HPS_GSENSOR_INT),

	//.adc_sclk                        (ADC_SCLK),
	//.adc_cs_n                        (ADC_CS_N),
	//.adc_dout                        (ADC_DOUT),
	//.adc_din                         (ADC_DIN),

	// General Purpose I/O
	.hps_io_hps_io_gpio_inst_GPIO40	(HPS_GPIO[0]),
	.hps_io_hps_io_gpio_inst_GPIO41	(HPS_GPIO[1]),

	// I2C
	.hps_io_hps_io_gpio_inst_GPIO48	(HPS_I2C_CONTROL),
	.hps_io_hps_io_i2c0_inst_SDA		(HPS_I2C1_SDAT),
	.hps_io_hps_io_i2c0_inst_SCL		(HPS_I2C1_SCLK),
	.hps_io_hps_io_i2c1_inst_SDA		(HPS_I2C2_SDAT),
	.hps_io_hps_io_i2c1_inst_SCL		(HPS_I2C2_SCLK),

	// Pushbutton
	.hps_io_hps_io_gpio_inst_GPIO54	(HPS_KEY),

	// LED
	.hps_io_hps_io_gpio_inst_GPIO53	(HPS_LED),

	// SD Card
	.hps_io_hps_io_sdio_inst_CMD	(HPS_SD_CMD),
	.hps_io_hps_io_sdio_inst_D0	(HPS_SD_DATA[0]),
	.hps_io_hps_io_sdio_inst_D1	(HPS_SD_DATA[1]),
	.hps_io_hps_io_sdio_inst_CLK	(HPS_SD_CLK),
	.hps_io_hps_io_sdio_inst_D2	(HPS_SD_DATA[2]),
	.hps_io_hps_io_sdio_inst_D3	(HPS_SD_DATA[3]),

	// SPI
	.hps_io_hps_io_spim1_inst_CLK		(HPS_SPIM_CLK),
	.hps_io_hps_io_spim1_inst_MOSI	(HPS_SPIM_MOSI),
	.hps_io_hps_io_spim1_inst_MISO	(HPS_SPIM_MISO),
	.hps_io_hps_io_spim1_inst_SS0		(HPS_SPIM_SS),

	// UART
	.hps_io_hps_io_uart0_inst_RX	(HPS_UART_RX),
	.hps_io_hps_io_uart0_inst_TX	(HPS_UART_TX),

	// USB
	.hps_io_hps_io_gpio_inst_GPIO09	(HPS_CONV_USB_N),
	.hps_io_hps_io_usb1_inst_D0		(HPS_USB_DATA[0]),
	.hps_io_hps_io_usb1_inst_D1		(HPS_USB_DATA[1]),
	.hps_io_hps_io_usb1_inst_D2		(HPS_USB_DATA[2]),
	.hps_io_hps_io_usb1_inst_D3		(HPS_USB_DATA[3]),
	.hps_io_hps_io_usb1_inst_D4		(HPS_USB_DATA[4]),
	.hps_io_hps_io_usb1_inst_D5		(HPS_USB_DATA[5]),
	.hps_io_hps_io_usb1_inst_D6		(HPS_USB_DATA[6]),
	.hps_io_hps_io_usb1_inst_D7		(HPS_USB_DATA[7]),
	.hps_io_hps_io_usb1_inst_CLK		(HPS_USB_CLKOUT),
	.hps_io_hps_io_usb1_inst_STP		(HPS_USB_STP),
	.hps_io_hps_io_usb1_inst_DIR		(HPS_USB_DIR),
	.hps_io_hps_io_usb1_inst_NXT		(HPS_USB_NXT)
);


endmodule

//////////////////////////////////////////////////
//// signed mult of 12.15 format 2'comp////////////
//////////////////////////////////////////////////
module signed_mult (out, a, b);
	output 	signed  [26:0]	out;
	input 	signed	[26:0] 	a;
	input 	signed	[26:0] 	b;
	// intermediate full bit length
	wire 	signed	[53:0]	mult_out;
	assign mult_out = a * b;
	// select bits for 7.20 fixed point
	assign out = { mult_out[41:30], mult_out[29:15]};
endmodule