-- rx_sample_ts: sample-exact GPS time for the RX metadata header, owned by the
-- rx_clock domain.
--
-- The stock free-running sample counter stays in the header's 64-bit timestamp
-- field, so every existing consumer (libbladeRF's sync layer, host software that
-- timestamps against NTP) keeps working whether or not a GPS receiver is fitted
-- or locked. GPS time rides in the two extra header dwords (fifo_writer,
-- ENABLE_META_V2) as a snapshot taken at the same sample as the counter:
--
--   gps_word  bits 25:0  FRAC       RX samples since the last accepted PPS edge,
--                                   saturating at 2^26-1 (61.44 MSps fits)
--             bit  26    UTC_VALID  the epoch has been loaded from a valid UBX decode
--             bit  27    PPS_ALIVE  an accepted PPS edge within the last 1.5 s
--             bit  28    FRAC_VALID a PPS edge has been seen since RX was enabled:
--                                   samples are only counted while they flow, so
--                                   until the first PPS of a session the fraction
--                                   is short by however long RX was off
--             bit  29    UBX_FRESH  a valid UBX decode within the last 5 s
--             bit  30    GPS_LOCK   UTC_VALID and PPS_ALIVE and FRAC_VALID
--             bit  31    GPS_PRESENT constant 1: this word carries GPS time
--   gps_epoch            UTC seconds since the Unix epoch (0 until UTC_VALID)
--
-- utc(sample n of a message) = gps_epoch + (FRAC + n) / sample_rate.
--
-- The seconds advance in the same rx_clock cycle in which the count restarts,
-- so epoch and fraction can never disagree. gps_uart (sys_clock) decodes UBX
-- and qualifies the PPS; it hands over the epoch/lock state as one coherent
-- word through ts_cdc plus two single-bit toggles (accepted PPS edge, UBX epoch
-- load) that are synchronized here.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity rx_sample_ts is
  port (
    -- sys_clock domain (from gps_uart)
    sys_clock    : in  std_logic;
    epoch_sec    : in  std_logic_vector(31 downto 0);
    utc_locked   : in  std_logic;
    pps_alive    : in  std_logic;
    ubx_fresh    : in  std_logic;
    sync_tgl     : in  std_logic;   -- toggles when epoch_sec was (re)loaded from UBX
    pps_tgl      : in  std_logic;   -- toggles on each accepted PPS edge
    -- rx_clock domain
    rx_clock     : in  std_logic;
    rx_enable    : in  std_logic;   -- RX streaming enabled
    sample_v     : in  std_logic;   -- one pulse per RX sample
    timestamp    : out std_logic_vector(63 downto 0);  -- {epoch, full 32-bit count}
    gps_word     : out std_logic_vector(31 downto 0);
    gps_epoch    : out std_logic_vector(31 downto 0));
end entity rx_sample_ts;

architecture rtl of rx_sample_ts is
  signal src_word, dst_word : std_logic_vector(35 downto 0);
  signal pps_s     : std_logic_vector(2 downto 0) := (others => '0');
  signal sync_seen : std_logic := '0';
  signal pps_seen  : std_logic := '0';   -- a PPS since RX was enabled
  signal rx_en_r   : std_logic := '0';
  signal epoch_rx  : unsigned(31 downto 0) := (others => '0');
  signal samp_cnt  : unsigned(31 downto 0) := (others => '0');
  signal frac_sat  : std_logic_vector(25 downto 0);

  attribute preserve : boolean;
  attribute preserve of pps_s : signal is true;
begin
  src_word <= ubx_fresh & pps_alive & utc_locked & sync_tgl & epoch_sec;

  u_cdc : entity work.ts_cdc
    generic map (WIDTH => 36)
    port map (src_clk => sys_clock, src_data => src_word,
              dst_clk => rx_clock,  dst_data => dst_word);

  p_rx : process (rx_clock) is
    variable pps_rx : boolean;
  begin
    if rising_edge(rx_clock) then
      pps_s   <= pps_s(1 downto 0) & pps_tgl;         -- 3-FF toggle synchronizer
      pps_rx  := (pps_s(2) /= pps_s(1));
      rx_en_r <= rx_enable;

      if pps_rx then
        pps_seen <= '1';
        if sample_v = '1' then samp_cnt <= to_unsigned(1, 32);
        else                   samp_cnt <= (others => '0'); end if;
        if dst_word(33) = '1' then                     -- utc_locked
          epoch_rx <= epoch_rx + 1;
        end if;
      elsif sample_v = '1' then
        samp_cnt <= samp_cnt + 1;
      end if;

      -- RX (re)enabled: the count is short until the next PPS re-anchors it.
      if rx_enable = '1' and rx_en_r = '0' then
        pps_seen <= '0';
      end if;

      if dst_word(32) /= sync_seen then                -- UBX (re)load of the epoch
        sync_seen <= dst_word(32);
        epoch_rx  <= unsigned(dst_word(31 downto 0));
      end if;
    end if;
  end process p_rx;

  frac_sat  <= std_logic_vector(samp_cnt(25 downto 0)) when samp_cnt(31 downto 26) = "000000"
               else (others => '1');

  timestamp <= std_logic_vector(epoch_rx) & std_logic_vector(samp_cnt);
  gps_epoch <= std_logic_vector(epoch_rx);

  gps_word(25 downto 0) <= frac_sat;
  gps_word(26) <= dst_word(33);                                  -- UTC_VALID
  gps_word(27) <= dst_word(34);                                  -- PPS_ALIVE
  gps_word(28) <= pps_seen;                                      -- FRAC_VALID
  gps_word(29) <= dst_word(35);                                  -- UBX_FRESH
  gps_word(30) <= dst_word(33) and dst_word(34) and pps_seen;    -- GPS_LOCK
  gps_word(31) <= '1';                                           -- GPS_PRESENT
end architecture rtl;
