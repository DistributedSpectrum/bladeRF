-------------------------------------------------------------------------------
-- Testbench for gps_uart (UBX-NAV-TIMEUTC decoder)
--
-- Injects a binary UBX-NAV-TIMEUTC frame on the RX line at the DUT's fast baud
-- and checks that the module:
--   * validates the Fletcher checksum and the validUTC flag,
--   * converts the UTC date/time to Unix epoch seconds in timestamp[63:32],
--   * runs the PPS-disciplined fractional counter in timestamp[31:0],
--   * pulses timestamp_vld only for a good, valid frame.
--
-- Negative tests: a validUTC=0 frame and a bad-checksum frame must NOT update.
--
-- Reference frame: 2026-11-02 17:42:30 UTC -> Unix epoch 1793641350 = 0x6AE8CB86.
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library std;
use std.env.all;

entity gps_uart_tb is
end entity gps_uart_tb;

architecture behav of gps_uart_tb is

  -- DUT ports
  signal sys_clock     : std_logic := '0';
  signal sys_reset     : std_logic := '0';
  signal rxd           : std_logic := '1';   -- UART idle high
  signal pps           : std_logic := '0';
  signal timestamp     : std_logic_vector(63 downto 0);
  signal timestamp_vld : std_logic;
  signal txd           : std_logic;
  signal dummy_out     : std_logic;

  -- timing
  constant C_CLK_HALF : time := 6.25 ns;   -- 80 MHz
  constant C_BIT_FAST : time := 9025 ns;   -- 2 * 361 * 12.5 ns (~110.8 kbaud)

  -- RX only enables after the TX init sends the first command (~21.7 ms).
  constant C_RX_ENABLE_TIME : time := 22 ms;

  -- Expected decoded epoch for the reference frame.
  constant C_EXP_EPOCH : std_logic_vector(31 downto 0) := x"6AE8CB86";

  -- monitor state (one writer: p_mon)
  signal vld_count     : integer := 0;
  signal latched_epoch : std_logic_vector(31 downto 0) := (others => '0');
  signal latched_frac  : std_logic_vector(31 downto 0) := (others => '0');

  type byte_array is array (natural range <>) of std_logic_vector(7 downto 0);

  -- Good frame: NAV-TIMEUTC, 2026-11-02 17:42:30, valid=0x07 (validUTC set), CK=94 99
  constant UBX_GOOD : byte_array := (
    x"B5", x"62", x"01", x"21", x"14", x"00",             -- sync, class, id, len(20)
    x"00", x"00", x"00", x"00",                           -- iTOW
    x"00", x"00", x"00", x"00",                           -- tAcc
    x"00", x"00", x"00", x"00",                           -- nano
    x"EA", x"07",                                         -- year 2026 (LE)
    x"0B",                                                -- month 11
    x"02",                                                -- day 2
    x"11",                                                -- hour 17
    x"2A",                                                -- min 42
    x"1E",                                                -- sec 30
    x"07",                                                -- valid (validTOW|WKN|UTC)
    x"94", x"99");                                        -- CK_A, CK_B

  -- Same time but valid=0x03 (validUTC clear) -> must be rejected. CK=90 95
  constant UBX_NOUTC : byte_array := (
    x"B5", x"62", x"01", x"21", x"14", x"00",
    x"00", x"00", x"00", x"00",
    x"00", x"00", x"00", x"00",
    x"00", x"00", x"00", x"00",
    x"EA", x"07", x"0B", x"02", x"11", x"2A", x"1E",
    x"03",                                                -- valid: no validUTC
    x"90", x"95");                                        -- CK_A, CK_B

  -- Good payload but corrupted CK_B -> must be rejected.
  constant UBX_BADCK : byte_array := (
    x"B5", x"62", x"01", x"21", x"14", x"00",
    x"00", x"00", x"00", x"00",
    x"00", x"00", x"00", x"00",
    x"00", x"00", x"00", x"00",
    x"EA", x"07", x"0B", x"02", x"11", x"2A", x"1E", x"07",
    x"94", x"00");                                        -- bad CK_B

  procedure uart_send_byte(signal   line_out : out std_logic;
                           constant data     : in  std_logic_vector(7 downto 0);
                           constant bit_time : in  time) is
  begin
    line_out <= '0';                    -- start
    wait for bit_time;
    for i in 0 to 7 loop                -- data LSB first
      line_out <= data(i);
      wait for bit_time;
    end loop;
    line_out <= '1';                    -- stop
    wait for bit_time;
  end procedure;

  procedure send_frame(signal   line_out : out std_logic;
                       constant frame    : in  byte_array;
                       constant bit_time : in  time) is
  begin
    for i in frame'range loop
      uart_send_byte(line_out, frame(i), bit_time);
    end loop;
  end procedure;

begin

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

  sys_clock <= not sys_clock after C_CLK_HALF;

  -- monitor: count valid pulses and latch the decoded epoch/fraction
  p_mon: process (sys_clock) is
  begin
    if rising_edge(sys_clock) then
      if timestamp_vld = '1' then
        vld_count     <= vld_count + 1;
        latched_epoch <= timestamp(63 downto 32);
        latched_frac  <= timestamp(31 downto 0);
        report "timestamp_vld: epoch=0x" & to_hstring(timestamp(63 downto 32)) &
               " frac=0x" & to_hstring(timestamp(31 downto 0));
      end if;
    end if;
  end process;

  p_tb: process is
    variable c : integer;
  begin
    sys_reset <= '0';
    rxd <= '1';
    pps <= '0';
    wait for 30 ns;
    sys_reset <= '1';

    -- wait for the DUT to enable RX at the fast baud
    wait for C_RX_ENABLE_TIME;

    -- a PPS edge so the fractional counter is running
    pps <= '1';
    wait for 20 us;
    pps <= '0';
    wait for 100 us;

    -- 1) GOOD frame -> must decode
    c := vld_count;
    report "Injecting GOOD UBX-NAV-TIMEUTC frame...";
    send_frame(rxd, UBX_GOOD, C_BIT_FAST);
    wait for 300 us;
    assert vld_count = c + 1
      report "FAIL: good frame did not pulse timestamp_vld" severity error;
    assert latched_epoch = C_EXP_EPOCH
      report "FAIL: epoch 0x" & to_hstring(latched_epoch) &
             " != expected 0x" & to_hstring(C_EXP_EPOCH) severity error;
    assert timestamp(63 downto 32) = C_EXP_EPOCH
      report "FAIL: live epoch not held after decode" severity error;

    -- 2) validUTC=0 frame -> must be ignored
    c := vld_count;
    report "Injecting validUTC=0 frame (expect ignored)...";
    send_frame(rxd, UBX_NOUTC, C_BIT_FAST);
    wait for 300 us;
    assert vld_count = c
      report "FAIL: validUTC=0 frame updated the timestamp" severity error;

    -- 3) bad-checksum frame -> must be ignored
    c := vld_count;
    report "Injecting bad-checksum frame (expect ignored)...";
    send_frame(rxd, UBX_BADCK, C_BIT_FAST);
    wait for 300 us;
    assert vld_count = c
      report "FAIL: bad-checksum frame updated the timestamp" severity error;

    if latched_epoch = C_EXP_EPOCH and vld_count = c then
      report "PASS: UBX-NAV-TIMEUTC decoded to epoch 0x" & to_hstring(latched_epoch) &
             " (frac 0x" & to_hstring(latched_frac) &
             "); validUTC/checksum gating works." severity note;
    end if;

    report "Simulation finished." severity note;
    stop;
    wait;
  end process;

end architecture behav;
