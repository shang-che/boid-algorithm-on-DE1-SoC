	component Computer_System is
		port (
			audio_ADCDAT                                    : in    std_logic                     := 'X';             -- ADCDAT
			audio_ADCLRCK                                   : in    std_logic                     := 'X';             -- ADCLRCK
			audio_BCLK                                      : in    std_logic                     := 'X';             -- BCLK
			audio_DACDAT                                    : out   std_logic;                                        -- DACDAT
			audio_DACLRCK                                   : in    std_logic                     := 'X';             -- DACLRCK
			audio_clk_clk                                   : out   std_logic;                                        -- clk
			audio_pll_ref_clk_clk                           : in    std_logic                     := 'X';             -- clk
			audio_pll_ref_reset_reset                       : in    std_logic                     := 'X';             -- reset
			av_config_SDAT                                  : inout std_logic                     := 'X';             -- SDAT
			av_config_SCLK                                  : out   std_logic;                                        -- SCLK
			bus_master_audio_external_interface_address     : in    std_logic_vector(15 downto 0) := (others => 'X'); -- address
			bus_master_audio_external_interface_byte_enable : in    std_logic_vector(3 downto 0)  := (others => 'X'); -- byte_enable
			bus_master_audio_external_interface_read        : in    std_logic                     := 'X';             -- read
			bus_master_audio_external_interface_write       : in    std_logic                     := 'X';             -- write
			bus_master_audio_external_interface_write_data  : in    std_logic_vector(31 downto 0) := (others => 'X'); -- write_data
			bus_master_audio_external_interface_acknowledge : out   std_logic;                                        -- acknowledge
			bus_master_audio_external_interface_read_data   : out   std_logic_vector(31 downto 0);                    -- read_data
			debug1_external_connection_export               : in    std_logic_vector(31 downto 0) := (others => 'X'); -- export
			debug2_external_connection_export               : in    std_logic_vector(31 downto 0) := (others => 'X'); -- export
			debug3_external_connection_export               : in    std_logic_vector(31 downto 0) := (others => 'X'); -- export
			freq1_external_connection_export                : in    std_logic_vector(31 downto 0) := (others => 'X'); -- export
			freq2_external_connection_export                : in    std_logic_vector(31 downto 0) := (others => 'X'); -- export
			freq3_external_connection_export                : in    std_logic_vector(31 downto 0) := (others => 'X'); -- export
			freq4_external_connection_export                : in    std_logic_vector(31 downto 0) := (others => 'X'); -- export
			freq5_external_connection_export                : in    std_logic_vector(31 downto 0) := (others => 'X'); -- export
			hps_io_hps_io_emac1_inst_TX_CLK                 : out   std_logic;                                        -- hps_io_emac1_inst_TX_CLK
			hps_io_hps_io_emac1_inst_TXD0                   : out   std_logic;                                        -- hps_io_emac1_inst_TXD0
			hps_io_hps_io_emac1_inst_TXD1                   : out   std_logic;                                        -- hps_io_emac1_inst_TXD1
			hps_io_hps_io_emac1_inst_TXD2                   : out   std_logic;                                        -- hps_io_emac1_inst_TXD2
			hps_io_hps_io_emac1_inst_TXD3                   : out   std_logic;                                        -- hps_io_emac1_inst_TXD3
			hps_io_hps_io_emac1_inst_RXD0                   : in    std_logic                     := 'X';             -- hps_io_emac1_inst_RXD0
			hps_io_hps_io_emac1_inst_MDIO                   : inout std_logic                     := 'X';             -- hps_io_emac1_inst_MDIO
			hps_io_hps_io_emac1_inst_MDC                    : out   std_logic;                                        -- hps_io_emac1_inst_MDC
			hps_io_hps_io_emac1_inst_RX_CTL                 : in    std_logic                     := 'X';             -- hps_io_emac1_inst_RX_CTL
			hps_io_hps_io_emac1_inst_TX_CTL                 : out   std_logic;                                        -- hps_io_emac1_inst_TX_CTL
			hps_io_hps_io_emac1_inst_RX_CLK                 : in    std_logic                     := 'X';             -- hps_io_emac1_inst_RX_CLK
			hps_io_hps_io_emac1_inst_RXD1                   : in    std_logic                     := 'X';             -- hps_io_emac1_inst_RXD1
			hps_io_hps_io_emac1_inst_RXD2                   : in    std_logic                     := 'X';             -- hps_io_emac1_inst_RXD2
			hps_io_hps_io_emac1_inst_RXD3                   : in    std_logic                     := 'X';             -- hps_io_emac1_inst_RXD3
			hps_io_hps_io_qspi_inst_IO0                     : inout std_logic                     := 'X';             -- hps_io_qspi_inst_IO0
			hps_io_hps_io_qspi_inst_IO1                     : inout std_logic                     := 'X';             -- hps_io_qspi_inst_IO1
			hps_io_hps_io_qspi_inst_IO2                     : inout std_logic                     := 'X';             -- hps_io_qspi_inst_IO2
			hps_io_hps_io_qspi_inst_IO3                     : inout std_logic                     := 'X';             -- hps_io_qspi_inst_IO3
			hps_io_hps_io_qspi_inst_SS0                     : out   std_logic;                                        -- hps_io_qspi_inst_SS0
			hps_io_hps_io_qspi_inst_CLK                     : out   std_logic;                                        -- hps_io_qspi_inst_CLK
			hps_io_hps_io_sdio_inst_CMD                     : inout std_logic                     := 'X';             -- hps_io_sdio_inst_CMD
			hps_io_hps_io_sdio_inst_D0                      : inout std_logic                     := 'X';             -- hps_io_sdio_inst_D0
			hps_io_hps_io_sdio_inst_D1                      : inout std_logic                     := 'X';             -- hps_io_sdio_inst_D1
			hps_io_hps_io_sdio_inst_CLK                     : out   std_logic;                                        -- hps_io_sdio_inst_CLK
			hps_io_hps_io_sdio_inst_D2                      : inout std_logic                     := 'X';             -- hps_io_sdio_inst_D2
			hps_io_hps_io_sdio_inst_D3                      : inout std_logic                     := 'X';             -- hps_io_sdio_inst_D3
			hps_io_hps_io_usb1_inst_D0                      : inout std_logic                     := 'X';             -- hps_io_usb1_inst_D0
			hps_io_hps_io_usb1_inst_D1                      : inout std_logic                     := 'X';             -- hps_io_usb1_inst_D1
			hps_io_hps_io_usb1_inst_D2                      : inout std_logic                     := 'X';             -- hps_io_usb1_inst_D2
			hps_io_hps_io_usb1_inst_D3                      : inout std_logic                     := 'X';             -- hps_io_usb1_inst_D3
			hps_io_hps_io_usb1_inst_D4                      : inout std_logic                     := 'X';             -- hps_io_usb1_inst_D4
			hps_io_hps_io_usb1_inst_D5                      : inout std_logic                     := 'X';             -- hps_io_usb1_inst_D5
			hps_io_hps_io_usb1_inst_D6                      : inout std_logic                     := 'X';             -- hps_io_usb1_inst_D6
			hps_io_hps_io_usb1_inst_D7                      : inout std_logic                     := 'X';             -- hps_io_usb1_inst_D7
			hps_io_hps_io_usb1_inst_CLK                     : in    std_logic                     := 'X';             -- hps_io_usb1_inst_CLK
			hps_io_hps_io_usb1_inst_STP                     : out   std_logic;                                        -- hps_io_usb1_inst_STP
			hps_io_hps_io_usb1_inst_DIR                     : in    std_logic                     := 'X';             -- hps_io_usb1_inst_DIR
			hps_io_hps_io_usb1_inst_NXT                     : in    std_logic                     := 'X';             -- hps_io_usb1_inst_NXT
			hps_io_hps_io_spim1_inst_CLK                    : out   std_logic;                                        -- hps_io_spim1_inst_CLK
			hps_io_hps_io_spim1_inst_MOSI                   : out   std_logic;                                        -- hps_io_spim1_inst_MOSI
			hps_io_hps_io_spim1_inst_MISO                   : in    std_logic                     := 'X';             -- hps_io_spim1_inst_MISO
			hps_io_hps_io_spim1_inst_SS0                    : out   std_logic;                                        -- hps_io_spim1_inst_SS0
			hps_io_hps_io_uart0_inst_RX                     : in    std_logic                     := 'X';             -- hps_io_uart0_inst_RX
			hps_io_hps_io_uart0_inst_TX                     : out   std_logic;                                        -- hps_io_uart0_inst_TX
			hps_io_hps_io_i2c0_inst_SDA                     : inout std_logic                     := 'X';             -- hps_io_i2c0_inst_SDA
			hps_io_hps_io_i2c0_inst_SCL                     : inout std_logic                     := 'X';             -- hps_io_i2c0_inst_SCL
			hps_io_hps_io_i2c1_inst_SDA                     : inout std_logic                     := 'X';             -- hps_io_i2c1_inst_SDA
			hps_io_hps_io_i2c1_inst_SCL                     : inout std_logic                     := 'X';             -- hps_io_i2c1_inst_SCL
			hps_io_hps_io_gpio_inst_GPIO09                  : inout std_logic                     := 'X';             -- hps_io_gpio_inst_GPIO09
			hps_io_hps_io_gpio_inst_GPIO35                  : inout std_logic                     := 'X';             -- hps_io_gpio_inst_GPIO35
			hps_io_hps_io_gpio_inst_GPIO40                  : inout std_logic                     := 'X';             -- hps_io_gpio_inst_GPIO40
			hps_io_hps_io_gpio_inst_GPIO41                  : inout std_logic                     := 'X';             -- hps_io_gpio_inst_GPIO41
			hps_io_hps_io_gpio_inst_GPIO48                  : inout std_logic                     := 'X';             -- hps_io_gpio_inst_GPIO48
			hps_io_hps_io_gpio_inst_GPIO53                  : inout std_logic                     := 'X';             -- hps_io_gpio_inst_GPIO53
			hps_io_hps_io_gpio_inst_GPIO54                  : inout std_logic                     := 'X';             -- hps_io_gpio_inst_GPIO54
			hps_io_hps_io_gpio_inst_GPIO61                  : inout std_logic                     := 'X';             -- hps_io_gpio_inst_GPIO61
			memory_mem_a                                    : out   std_logic_vector(14 downto 0);                    -- mem_a
			memory_mem_ba                                   : out   std_logic_vector(2 downto 0);                     -- mem_ba
			memory_mem_ck                                   : out   std_logic;                                        -- mem_ck
			memory_mem_ck_n                                 : out   std_logic;                                        -- mem_ck_n
			memory_mem_cke                                  : out   std_logic;                                        -- mem_cke
			memory_mem_cs_n                                 : out   std_logic;                                        -- mem_cs_n
			memory_mem_ras_n                                : out   std_logic;                                        -- mem_ras_n
			memory_mem_cas_n                                : out   std_logic;                                        -- mem_cas_n
			memory_mem_we_n                                 : out   std_logic;                                        -- mem_we_n
			memory_mem_reset_n                              : out   std_logic;                                        -- mem_reset_n
			memory_mem_dq                                   : inout std_logic_vector(31 downto 0) := (others => 'X'); -- mem_dq
			memory_mem_dqs                                  : inout std_logic_vector(3 downto 0)  := (others => 'X'); -- mem_dqs
			memory_mem_dqs_n                                : inout std_logic_vector(3 downto 0)  := (others => 'X'); -- mem_dqs_n
			memory_mem_odt                                  : out   std_logic;                                        -- mem_odt
			memory_mem_dm                                   : out   std_logic_vector(3 downto 0);                     -- mem_dm
			memory_oct_rzqin                                : in    std_logic                     := 'X';             -- oct_rzqin
			onchip_sram_s1_address                          : in    std_logic_vector(16 downto 0) := (others => 'X'); -- address
			onchip_sram_s1_clken                            : in    std_logic                     := 'X';             -- clken
			onchip_sram_s1_chipselect                       : in    std_logic                     := 'X';             -- chipselect
			onchip_sram_s1_write                            : in    std_logic                     := 'X';             -- write
			onchip_sram_s1_readdata                         : out   std_logic_vector(7 downto 0);                     -- readdata
			onchip_sram_s1_writedata                        : in    std_logic_vector(7 downto 0)  := (others => 'X'); -- writedata
			right_audio_input_external_connection_export    : in    std_logic_vector(31 downto 0) := (others => 'X'); -- export
			sdram_addr                                      : out   std_logic_vector(12 downto 0);                    -- addr
			sdram_ba                                        : out   std_logic_vector(1 downto 0);                     -- ba
			sdram_cas_n                                     : out   std_logic;                                        -- cas_n
			sdram_cke                                       : out   std_logic;                                        -- cke
			sdram_cs_n                                      : out   std_logic;                                        -- cs_n
			sdram_dq                                        : inout std_logic_vector(15 downto 0) := (others => 'X'); -- dq
			sdram_dqm                                       : out   std_logic_vector(1 downto 0);                     -- dqm
			sdram_ras_n                                     : out   std_logic;                                        -- ras_n
			sdram_we_n                                      : out   std_logic;                                        -- we_n
			sdram_clk_clk                                   : out   std_logic;                                        -- clk
			system_pll_ref_clk_clk                          : in    std_logic                     := 'X';             -- clk
			system_pll_ref_reset_reset                      : in    std_logic                     := 'X';             -- reset
			vga_CLK                                         : out   std_logic;                                        -- CLK
			vga_HS                                          : out   std_logic;                                        -- HS
			vga_VS                                          : out   std_logic;                                        -- VS
			vga_BLANK                                       : out   std_logic;                                        -- BLANK
			vga_SYNC                                        : out   std_logic;                                        -- SYNC
			vga_R                                           : out   std_logic_vector(7 downto 0);                     -- R
			vga_G                                           : out   std_logic_vector(7 downto 0);                     -- G
			vga_B                                           : out   std_logic_vector(7 downto 0);                     -- B
			vga_pll_ref_clk_clk                             : in    std_logic                     := 'X';             -- clk
			vga_pll_ref_reset_reset                         : in    std_logic                     := 'X';             -- reset
			counter_external_connection_export              : in    std_logic_vector(31 downto 0) := (others => 'X')  -- export
		);
	end component Computer_System;

	u0 : component Computer_System
		port map (
			audio_ADCDAT                                    => CONNECTED_TO_audio_ADCDAT,                                    --                                 audio.ADCDAT
			audio_ADCLRCK                                   => CONNECTED_TO_audio_ADCLRCK,                                   --                                      .ADCLRCK
			audio_BCLK                                      => CONNECTED_TO_audio_BCLK,                                      --                                      .BCLK
			audio_DACDAT                                    => CONNECTED_TO_audio_DACDAT,                                    --                                      .DACDAT
			audio_DACLRCK                                   => CONNECTED_TO_audio_DACLRCK,                                   --                                      .DACLRCK
			audio_clk_clk                                   => CONNECTED_TO_audio_clk_clk,                                   --                             audio_clk.clk
			audio_pll_ref_clk_clk                           => CONNECTED_TO_audio_pll_ref_clk_clk,                           --                     audio_pll_ref_clk.clk
			audio_pll_ref_reset_reset                       => CONNECTED_TO_audio_pll_ref_reset_reset,                       --                   audio_pll_ref_reset.reset
			av_config_SDAT                                  => CONNECTED_TO_av_config_SDAT,                                  --                             av_config.SDAT
			av_config_SCLK                                  => CONNECTED_TO_av_config_SCLK,                                  --                                      .SCLK
			bus_master_audio_external_interface_address     => CONNECTED_TO_bus_master_audio_external_interface_address,     --   bus_master_audio_external_interface.address
			bus_master_audio_external_interface_byte_enable => CONNECTED_TO_bus_master_audio_external_interface_byte_enable, --                                      .byte_enable
			bus_master_audio_external_interface_read        => CONNECTED_TO_bus_master_audio_external_interface_read,        --                                      .read
			bus_master_audio_external_interface_write       => CONNECTED_TO_bus_master_audio_external_interface_write,       --                                      .write
			bus_master_audio_external_interface_write_data  => CONNECTED_TO_bus_master_audio_external_interface_write_data,  --                                      .write_data
			bus_master_audio_external_interface_acknowledge => CONNECTED_TO_bus_master_audio_external_interface_acknowledge, --                                      .acknowledge
			bus_master_audio_external_interface_read_data   => CONNECTED_TO_bus_master_audio_external_interface_read_data,   --                                      .read_data
			debug1_external_connection_export               => CONNECTED_TO_debug1_external_connection_export,               --            debug1_external_connection.export
			debug2_external_connection_export               => CONNECTED_TO_debug2_external_connection_export,               --            debug2_external_connection.export
			debug3_external_connection_export               => CONNECTED_TO_debug3_external_connection_export,               --            debug3_external_connection.export
			freq1_external_connection_export                => CONNECTED_TO_freq1_external_connection_export,                --             freq1_external_connection.export
			freq2_external_connection_export                => CONNECTED_TO_freq2_external_connection_export,                --             freq2_external_connection.export
			freq3_external_connection_export                => CONNECTED_TO_freq3_external_connection_export,                --             freq3_external_connection.export
			freq4_external_connection_export                => CONNECTED_TO_freq4_external_connection_export,                --             freq4_external_connection.export
			freq5_external_connection_export                => CONNECTED_TO_freq5_external_connection_export,                --             freq5_external_connection.export
			hps_io_hps_io_emac1_inst_TX_CLK                 => CONNECTED_TO_hps_io_hps_io_emac1_inst_TX_CLK,                 --                                hps_io.hps_io_emac1_inst_TX_CLK
			hps_io_hps_io_emac1_inst_TXD0                   => CONNECTED_TO_hps_io_hps_io_emac1_inst_TXD0,                   --                                      .hps_io_emac1_inst_TXD0
			hps_io_hps_io_emac1_inst_TXD1                   => CONNECTED_TO_hps_io_hps_io_emac1_inst_TXD1,                   --                                      .hps_io_emac1_inst_TXD1
			hps_io_hps_io_emac1_inst_TXD2                   => CONNECTED_TO_hps_io_hps_io_emac1_inst_TXD2,                   --                                      .hps_io_emac1_inst_TXD2
			hps_io_hps_io_emac1_inst_TXD3                   => CONNECTED_TO_hps_io_hps_io_emac1_inst_TXD3,                   --                                      .hps_io_emac1_inst_TXD3
			hps_io_hps_io_emac1_inst_RXD0                   => CONNECTED_TO_hps_io_hps_io_emac1_inst_RXD0,                   --                                      .hps_io_emac1_inst_RXD0
			hps_io_hps_io_emac1_inst_MDIO                   => CONNECTED_TO_hps_io_hps_io_emac1_inst_MDIO,                   --                                      .hps_io_emac1_inst_MDIO
			hps_io_hps_io_emac1_inst_MDC                    => CONNECTED_TO_hps_io_hps_io_emac1_inst_MDC,                    --                                      .hps_io_emac1_inst_MDC
			hps_io_hps_io_emac1_inst_RX_CTL                 => CONNECTED_TO_hps_io_hps_io_emac1_inst_RX_CTL,                 --                                      .hps_io_emac1_inst_RX_CTL
			hps_io_hps_io_emac1_inst_TX_CTL                 => CONNECTED_TO_hps_io_hps_io_emac1_inst_TX_CTL,                 --                                      .hps_io_emac1_inst_TX_CTL
			hps_io_hps_io_emac1_inst_RX_CLK                 => CONNECTED_TO_hps_io_hps_io_emac1_inst_RX_CLK,                 --                                      .hps_io_emac1_inst_RX_CLK
			hps_io_hps_io_emac1_inst_RXD1                   => CONNECTED_TO_hps_io_hps_io_emac1_inst_RXD1,                   --                                      .hps_io_emac1_inst_RXD1
			hps_io_hps_io_emac1_inst_RXD2                   => CONNECTED_TO_hps_io_hps_io_emac1_inst_RXD2,                   --                                      .hps_io_emac1_inst_RXD2
			hps_io_hps_io_emac1_inst_RXD3                   => CONNECTED_TO_hps_io_hps_io_emac1_inst_RXD3,                   --                                      .hps_io_emac1_inst_RXD3
			hps_io_hps_io_qspi_inst_IO0                     => CONNECTED_TO_hps_io_hps_io_qspi_inst_IO0,                     --                                      .hps_io_qspi_inst_IO0
			hps_io_hps_io_qspi_inst_IO1                     => CONNECTED_TO_hps_io_hps_io_qspi_inst_IO1,                     --                                      .hps_io_qspi_inst_IO1
			hps_io_hps_io_qspi_inst_IO2                     => CONNECTED_TO_hps_io_hps_io_qspi_inst_IO2,                     --                                      .hps_io_qspi_inst_IO2
			hps_io_hps_io_qspi_inst_IO3                     => CONNECTED_TO_hps_io_hps_io_qspi_inst_IO3,                     --                                      .hps_io_qspi_inst_IO3
			hps_io_hps_io_qspi_inst_SS0                     => CONNECTED_TO_hps_io_hps_io_qspi_inst_SS0,                     --                                      .hps_io_qspi_inst_SS0
			hps_io_hps_io_qspi_inst_CLK                     => CONNECTED_TO_hps_io_hps_io_qspi_inst_CLK,                     --                                      .hps_io_qspi_inst_CLK
			hps_io_hps_io_sdio_inst_CMD                     => CONNECTED_TO_hps_io_hps_io_sdio_inst_CMD,                     --                                      .hps_io_sdio_inst_CMD
			hps_io_hps_io_sdio_inst_D0                      => CONNECTED_TO_hps_io_hps_io_sdio_inst_D0,                      --                                      .hps_io_sdio_inst_D0
			hps_io_hps_io_sdio_inst_D1                      => CONNECTED_TO_hps_io_hps_io_sdio_inst_D1,                      --                                      .hps_io_sdio_inst_D1
			hps_io_hps_io_sdio_inst_CLK                     => CONNECTED_TO_hps_io_hps_io_sdio_inst_CLK,                     --                                      .hps_io_sdio_inst_CLK
			hps_io_hps_io_sdio_inst_D2                      => CONNECTED_TO_hps_io_hps_io_sdio_inst_D2,                      --                                      .hps_io_sdio_inst_D2
			hps_io_hps_io_sdio_inst_D3                      => CONNECTED_TO_hps_io_hps_io_sdio_inst_D3,                      --                                      .hps_io_sdio_inst_D3
			hps_io_hps_io_usb1_inst_D0                      => CONNECTED_TO_hps_io_hps_io_usb1_inst_D0,                      --                                      .hps_io_usb1_inst_D0
			hps_io_hps_io_usb1_inst_D1                      => CONNECTED_TO_hps_io_hps_io_usb1_inst_D1,                      --                                      .hps_io_usb1_inst_D1
			hps_io_hps_io_usb1_inst_D2                      => CONNECTED_TO_hps_io_hps_io_usb1_inst_D2,                      --                                      .hps_io_usb1_inst_D2
			hps_io_hps_io_usb1_inst_D3                      => CONNECTED_TO_hps_io_hps_io_usb1_inst_D3,                      --                                      .hps_io_usb1_inst_D3
			hps_io_hps_io_usb1_inst_D4                      => CONNECTED_TO_hps_io_hps_io_usb1_inst_D4,                      --                                      .hps_io_usb1_inst_D4
			hps_io_hps_io_usb1_inst_D5                      => CONNECTED_TO_hps_io_hps_io_usb1_inst_D5,                      --                                      .hps_io_usb1_inst_D5
			hps_io_hps_io_usb1_inst_D6                      => CONNECTED_TO_hps_io_hps_io_usb1_inst_D6,                      --                                      .hps_io_usb1_inst_D6
			hps_io_hps_io_usb1_inst_D7                      => CONNECTED_TO_hps_io_hps_io_usb1_inst_D7,                      --                                      .hps_io_usb1_inst_D7
			hps_io_hps_io_usb1_inst_CLK                     => CONNECTED_TO_hps_io_hps_io_usb1_inst_CLK,                     --                                      .hps_io_usb1_inst_CLK
			hps_io_hps_io_usb1_inst_STP                     => CONNECTED_TO_hps_io_hps_io_usb1_inst_STP,                     --                                      .hps_io_usb1_inst_STP
			hps_io_hps_io_usb1_inst_DIR                     => CONNECTED_TO_hps_io_hps_io_usb1_inst_DIR,                     --                                      .hps_io_usb1_inst_DIR
			hps_io_hps_io_usb1_inst_NXT                     => CONNECTED_TO_hps_io_hps_io_usb1_inst_NXT,                     --                                      .hps_io_usb1_inst_NXT
			hps_io_hps_io_spim1_inst_CLK                    => CONNECTED_TO_hps_io_hps_io_spim1_inst_CLK,                    --                                      .hps_io_spim1_inst_CLK
			hps_io_hps_io_spim1_inst_MOSI                   => CONNECTED_TO_hps_io_hps_io_spim1_inst_MOSI,                   --                                      .hps_io_spim1_inst_MOSI
			hps_io_hps_io_spim1_inst_MISO                   => CONNECTED_TO_hps_io_hps_io_spim1_inst_MISO,                   --                                      .hps_io_spim1_inst_MISO
			hps_io_hps_io_spim1_inst_SS0                    => CONNECTED_TO_hps_io_hps_io_spim1_inst_SS0,                    --                                      .hps_io_spim1_inst_SS0
			hps_io_hps_io_uart0_inst_RX                     => CONNECTED_TO_hps_io_hps_io_uart0_inst_RX,                     --                                      .hps_io_uart0_inst_RX
			hps_io_hps_io_uart0_inst_TX                     => CONNECTED_TO_hps_io_hps_io_uart0_inst_TX,                     --                                      .hps_io_uart0_inst_TX
			hps_io_hps_io_i2c0_inst_SDA                     => CONNECTED_TO_hps_io_hps_io_i2c0_inst_SDA,                     --                                      .hps_io_i2c0_inst_SDA
			hps_io_hps_io_i2c0_inst_SCL                     => CONNECTED_TO_hps_io_hps_io_i2c0_inst_SCL,                     --                                      .hps_io_i2c0_inst_SCL
			hps_io_hps_io_i2c1_inst_SDA                     => CONNECTED_TO_hps_io_hps_io_i2c1_inst_SDA,                     --                                      .hps_io_i2c1_inst_SDA
			hps_io_hps_io_i2c1_inst_SCL                     => CONNECTED_TO_hps_io_hps_io_i2c1_inst_SCL,                     --                                      .hps_io_i2c1_inst_SCL
			hps_io_hps_io_gpio_inst_GPIO09                  => CONNECTED_TO_hps_io_hps_io_gpio_inst_GPIO09,                  --                                      .hps_io_gpio_inst_GPIO09
			hps_io_hps_io_gpio_inst_GPIO35                  => CONNECTED_TO_hps_io_hps_io_gpio_inst_GPIO35,                  --                                      .hps_io_gpio_inst_GPIO35
			hps_io_hps_io_gpio_inst_GPIO40                  => CONNECTED_TO_hps_io_hps_io_gpio_inst_GPIO40,                  --                                      .hps_io_gpio_inst_GPIO40
			hps_io_hps_io_gpio_inst_GPIO41                  => CONNECTED_TO_hps_io_hps_io_gpio_inst_GPIO41,                  --                                      .hps_io_gpio_inst_GPIO41
			hps_io_hps_io_gpio_inst_GPIO48                  => CONNECTED_TO_hps_io_hps_io_gpio_inst_GPIO48,                  --                                      .hps_io_gpio_inst_GPIO48
			hps_io_hps_io_gpio_inst_GPIO53                  => CONNECTED_TO_hps_io_hps_io_gpio_inst_GPIO53,                  --                                      .hps_io_gpio_inst_GPIO53
			hps_io_hps_io_gpio_inst_GPIO54                  => CONNECTED_TO_hps_io_hps_io_gpio_inst_GPIO54,                  --                                      .hps_io_gpio_inst_GPIO54
			hps_io_hps_io_gpio_inst_GPIO61                  => CONNECTED_TO_hps_io_hps_io_gpio_inst_GPIO61,                  --                                      .hps_io_gpio_inst_GPIO61
			memory_mem_a                                    => CONNECTED_TO_memory_mem_a,                                    --                                memory.mem_a
			memory_mem_ba                                   => CONNECTED_TO_memory_mem_ba,                                   --                                      .mem_ba
			memory_mem_ck                                   => CONNECTED_TO_memory_mem_ck,                                   --                                      .mem_ck
			memory_mem_ck_n                                 => CONNECTED_TO_memory_mem_ck_n,                                 --                                      .mem_ck_n
			memory_mem_cke                                  => CONNECTED_TO_memory_mem_cke,                                  --                                      .mem_cke
			memory_mem_cs_n                                 => CONNECTED_TO_memory_mem_cs_n,                                 --                                      .mem_cs_n
			memory_mem_ras_n                                => CONNECTED_TO_memory_mem_ras_n,                                --                                      .mem_ras_n
			memory_mem_cas_n                                => CONNECTED_TO_memory_mem_cas_n,                                --                                      .mem_cas_n
			memory_mem_we_n                                 => CONNECTED_TO_memory_mem_we_n,                                 --                                      .mem_we_n
			memory_mem_reset_n                              => CONNECTED_TO_memory_mem_reset_n,                              --                                      .mem_reset_n
			memory_mem_dq                                   => CONNECTED_TO_memory_mem_dq,                                   --                                      .mem_dq
			memory_mem_dqs                                  => CONNECTED_TO_memory_mem_dqs,                                  --                                      .mem_dqs
			memory_mem_dqs_n                                => CONNECTED_TO_memory_mem_dqs_n,                                --                                      .mem_dqs_n
			memory_mem_odt                                  => CONNECTED_TO_memory_mem_odt,                                  --                                      .mem_odt
			memory_mem_dm                                   => CONNECTED_TO_memory_mem_dm,                                   --                                      .mem_dm
			memory_oct_rzqin                                => CONNECTED_TO_memory_oct_rzqin,                                --                                      .oct_rzqin
			onchip_sram_s1_address                          => CONNECTED_TO_onchip_sram_s1_address,                          --                        onchip_sram_s1.address
			onchip_sram_s1_clken                            => CONNECTED_TO_onchip_sram_s1_clken,                            --                                      .clken
			onchip_sram_s1_chipselect                       => CONNECTED_TO_onchip_sram_s1_chipselect,                       --                                      .chipselect
			onchip_sram_s1_write                            => CONNECTED_TO_onchip_sram_s1_write,                            --                                      .write
			onchip_sram_s1_readdata                         => CONNECTED_TO_onchip_sram_s1_readdata,                         --                                      .readdata
			onchip_sram_s1_writedata                        => CONNECTED_TO_onchip_sram_s1_writedata,                        --                                      .writedata
			right_audio_input_external_connection_export    => CONNECTED_TO_right_audio_input_external_connection_export,    -- right_audio_input_external_connection.export
			sdram_addr                                      => CONNECTED_TO_sdram_addr,                                      --                                 sdram.addr
			sdram_ba                                        => CONNECTED_TO_sdram_ba,                                        --                                      .ba
			sdram_cas_n                                     => CONNECTED_TO_sdram_cas_n,                                     --                                      .cas_n
			sdram_cke                                       => CONNECTED_TO_sdram_cke,                                       --                                      .cke
			sdram_cs_n                                      => CONNECTED_TO_sdram_cs_n,                                      --                                      .cs_n
			sdram_dq                                        => CONNECTED_TO_sdram_dq,                                        --                                      .dq
			sdram_dqm                                       => CONNECTED_TO_sdram_dqm,                                       --                                      .dqm
			sdram_ras_n                                     => CONNECTED_TO_sdram_ras_n,                                     --                                      .ras_n
			sdram_we_n                                      => CONNECTED_TO_sdram_we_n,                                      --                                      .we_n
			sdram_clk_clk                                   => CONNECTED_TO_sdram_clk_clk,                                   --                             sdram_clk.clk
			system_pll_ref_clk_clk                          => CONNECTED_TO_system_pll_ref_clk_clk,                          --                    system_pll_ref_clk.clk
			system_pll_ref_reset_reset                      => CONNECTED_TO_system_pll_ref_reset_reset,                      --                  system_pll_ref_reset.reset
			vga_CLK                                         => CONNECTED_TO_vga_CLK,                                         --                                   vga.CLK
			vga_HS                                          => CONNECTED_TO_vga_HS,                                          --                                      .HS
			vga_VS                                          => CONNECTED_TO_vga_VS,                                          --                                      .VS
			vga_BLANK                                       => CONNECTED_TO_vga_BLANK,                                       --                                      .BLANK
			vga_SYNC                                        => CONNECTED_TO_vga_SYNC,                                        --                                      .SYNC
			vga_R                                           => CONNECTED_TO_vga_R,                                           --                                      .R
			vga_G                                           => CONNECTED_TO_vga_G,                                           --                                      .G
			vga_B                                           => CONNECTED_TO_vga_B,                                           --                                      .B
			vga_pll_ref_clk_clk                             => CONNECTED_TO_vga_pll_ref_clk_clk,                             --                       vga_pll_ref_clk.clk
			vga_pll_ref_reset_reset                         => CONNECTED_TO_vga_pll_ref_reset_reset,                         --                     vga_pll_ref_reset.reset
			counter_external_connection_export              => CONNECTED_TO_counter_external_connection_export               --           counter_external_connection.export
		);

