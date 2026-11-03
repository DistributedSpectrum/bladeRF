library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-------------------------------------------------------------------------------
-- rx_sample_ts
--
-- Sample-exact GPS timestamp, owned by the rx_clock domain:
--   timestamp[63:32] = UTC seconds since the Unix epoch
--   timestamp[31:0]  = number of RX samples since the last PPS edge
--
-- gps_uart (sys_clock) still decodes UBX-NAV-TIMEUTC and qualifies PPS; this
-- module takes two single-bit toggles from it (an accepted PPS edge, and an
-- epoch (re)load from UBX) plus the epoch/lock state crossed as one coherent
-- word (ts_cdc), and keeps the whole timestamp in the rx_clock domain:
--   * on the PPS: the seconds advance and the sample count restarts in the
--     SAME rx_clock cycle, so epoch and fraction can never tear;
--   * ~42 ms later, when UBX delivers the absolute second, the seconds are
--     reloaded from the crossed word (never coincides with a PPS).
-- Counting sample strobes (one per sample, same domain as the sample data)
-- instead of snapshotting a sys_clock counter removes the clock-domain
-- snapshot jitter (~±100 ns): the block timestamp latched by fifo_writer is
-- exact to the sample. Host: utc(sample n of block) = epoch + (frac + n) / Fs.
-- Seconds stay 0 until the first valid UBX => "not locked".
-------------------------------------------------------------------------------

entity rx_sample_ts is
  port (
    -- sys_clock domain (from gps_uart)
    sys_clock   : in  std_logic;
    epoch_sec   : in  std_logic_vector(31 downto 0);
    utc_locked  : in  std_logic;
    sync_tgl    : in  std_logic;   -- toggles when epoch_sec was (re)loaded from UBX
    pps_tgl     : in  std_logic;   -- toggles on each accepted PPS edge
    -- rx_clock domain
    rx_clock    : in  std_logic;
    sample_v    : in  std_logic;   -- one pulse per RX sample
    timestamp   : out std_logic_vector(63 downto 0));
end entity rx_sample_ts;

architecture rtl of rx_sample_ts is
  signal src_word, dst_word : std_logic_vector(33 downto 0);
  signal pps_s     : std_logic_vector(2 downto 0) := (others => '0');
  signal sync_seen : std_logic := '0';
  signal epoch_rx  : unsigned(31 downto 0) := (others => '0');
  signal samp_cnt  : unsigned(31 downto 0) := (others => '0');
  attribute preserve : boolean;
  attribute preserve of pps_s : signal is true;
begin
  -- {utc_locked, sync_tgl, epoch_sec} cross as ONE coherent word, so the sync
  -- toggle always arrives together with the epoch value it refers to.
  src_word <= utc_locked & sync_tgl & epoch_sec;
  u_cdc : entity work.ts_cdc
    generic map (WIDTH => 34)
    port map (src_clk => sys_clock, src_data => src_word,
              dst_clk => rx_clock,  dst_data => dst_word);

  p_rx : process (rx_clock) is
    variable pps_rx : boolean;
  begin
    if rising_edge(rx_clock) then
      pps_s  <= pps_s(1 downto 0) & pps_tgl;          -- 3-FF toggle synchronizer
      pps_rx := (pps_s(2) /= pps_s(1));

      if pps_rx then
        -- seconds advance and the sample count restarts in the same cycle
        if sample_v = '1' then samp_cnt <= to_unsigned(1, 32);
        else                   samp_cnt <= (others => '0'); end if;
        if dst_word(33) = '1' then                     -- utc_locked
          epoch_rx <= epoch_rx + 1;
        end if;
      elsif sample_v = '1' then
        samp_cnt <= samp_cnt + 1;
      end if;

      -- absolute (re)load from UBX, coherent with its epoch value
      if dst_word(32) /= sync_seen then
        sync_seen <= dst_word(32);
        epoch_rx  <= unsigned(dst_word(31 downto 0));
      end if;
    end if;
  end process p_rx;

  timestamp <= std_logic_vector(epoch_rx) & std_logic_vector(samp_cnt);
end architecture rtl;
