-------------------------------------------------------------------------------
-- Title      : Testbench for design "gps_uart"
-- Project    : 
-------------------------------------------------------------------------------
-- File       : gps_uart_tb.vhd
-- Author     : Elya  <elya@maxwell>
-- Company    : 
-- Created    : 2026-09-14
-- Last update: 2026-09-14
-- Platform   : 
-- Standard   : VHDL'93/02
-------------------------------------------------------------------------------
-- Description: 
-------------------------------------------------------------------------------
-- Copyright (c) 2026 
-------------------------------------------------------------------------------
-- Revisions  :
-- Date        Version  Author  Description
-- 2026-09-14  1.0      elya	Created
-------------------------------------------------------------------------------

-------------------------------------------------------------------------------
-- Title      : Testbench for design "gps_uart"
-- Project    : 
-------------------------------------------------------------------------------
-- File       : gps_uart_tb.vhd
-- Author     : Elya  <elya@maxwell>
-- Company    : 
-- Created    : 2026-09-14
-- Last update: 2026-09-14
-- Platform   : 
-- Standard   : VHDL'93/02
-------------------------------------------------------------------------------
-- Description: 
-------------------------------------------------------------------------------
-- Copyright (c) 2026 
-------------------------------------------------------------------------------
-- Revisions  :
-- Date        Version  Author  Description
-- 2026-09-14  1.0      elya	Created
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;

-------------------------------------------------------------------------------

entity gps_uart_tb is

end entity gps_uart_tb;

-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;

-------------------------------------------------------------------------------

entity gps_uart_tb is

end entity gps_uart_tb;

-------------------------------------------------------------------------------

architecture behav of gps_uart_tb is

  -- component ports
  signal sys_clock     : std_logic := '0';
  signal sys_reset     : std_logic;
  signal rxd           : std_logic;
  signal pps           : std_logic;
  signal timestamp     : std_logic_vector(3*16-1 downto 0);
  signal timestamp_vld : std_logic;
  signal txd           : std_logic;
  signal dummy_out     : std_logic;


begin  -- architecture behav

  -- component instantiation
  DUT: entity work.gps_uart
    port map (
      sys_clock     => sys_clock,
      sys_reset     => sys_reset,
      rxd           => rxd,
      pps           => pps,
      timestamp     => timestamp,
      timestamp_vld => timestamp_vld,
      txd           => txd,
      dummy_out     => dummy_out);

  -- clock generation
  sys_clock <= not sys_clock after 6.25 ns;

  p_tb: process is
  begin  -- process p_tb
    sys_reset <= '0';
    wait for 30 ns;
    sys_reset <= '1';

    wait;
    
  end process p_tb;
  

end architecture behav;

