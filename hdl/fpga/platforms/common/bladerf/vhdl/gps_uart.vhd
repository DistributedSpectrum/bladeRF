library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-------------------------------------------------------------------------------
-- gps_uart
--
-- Receives time from a u-blox GPS over a UART and produces an NTP-style 64-bit
-- timestamp for the RX sample metadata:
--   timestamp[63:32] = UTC seconds since the Unix epoch (1970-01-01)
--   timestamp[31:0]  = sub-second fraction = sys_clock ticks since the last PPS
--
-- Time source is the binary UBX-NAV-TIMEUTC message (class 0x01, id 0x21),
-- validated by its Fletcher checksum and the message's validUTC flag. The PPS
-- input disciplines the seconds boundary (seconds advance on each PPS edge; the
-- absolute value is (re)synced whenever a valid UBX message is received).
--
-- Replaces the earlier ASCII NMEA ($GxZDA) decoder.
-------------------------------------------------------------------------------

entity gps_uart is

  generic (
    -- PPS qualification. The raw PPS pin picks up edge noise (on hardware,
    -- UART RX activity ~42 ms after each PPS induced a burst of false edges
    -- on the PPS input, bumping epoch_sec by up to +12 and zeroing the
    -- fraction until the next UBX resync). A real PPS is 1 Hz, so:
    --   G_PPS_FILTER_CYCLES : pps must be high this many consecutive
    --                         sys_clock cycles before an edge counts
    --   G_PPS_REFRACTORY    : after an accepted edge, ignore the input for
    --                         this many cycles (0.9 s at 80 MHz)
    G_PPS_FILTER_CYCLES : natural := 8;
    G_PPS_REFRACTORY    : natural := 72_000_000);

  port (
    sys_clock       : in  std_logic;
    sys_reset   : in  std_logic;
    rxd       : in  std_logic;
    pps : in std_logic;
    timestamp : out std_logic_vector(63 downto 0);
    timestamp_vld : out std_logic;
    txd       : out std_logic;
    dummy_out : out std_logic);

end entity gps_uart;

architecture rtl of gps_uart is
    constant C_GSP_DIV_CNT : integer := 4340;  -- For 9.6 kbps
    constant C_GPS_DIV_CNT_115k : integer := C_GSP_DIV_CNT / 12; -- For 115.2 kbps

    constant CMD_MAX : integer := 3;
    constant CMD_LEN : integer := 51;

    constant C_CR : character := character'val(13); -- ASCII CR, 0x0D
    constant C_LF : character := character'val(10); -- ASCII LF, 0x0A

    function char_to_slv(c : character) return std_logic_vector is
    begin
      return std_logic_vector(to_unsigned(character'pos(c), 8));
    end function;

    type cmd_table_rec is record
      command : string(1 to CMD_LEN);
      len     : integer range 0 to 63;
    end record cmd_table_rec;

    type cmds_array is array (0 to CMD_MAX-1) of cmd_table_rec;

    -- Legacy init commands. The u-blox F9P ignores these $PMTK (MediaTek)
    -- strings; they are retained only so the existing baud-switch / RX-enable
    -- handoff (gps_uart_15k_baud) still runs. The F9P's UART1 baud and
    -- NAV-TIMEUTC output are configured host-side (ubxtool).
    constant init_commands : cmds_array :=
      (
        0 => (command => "$PMTK251,115200*1F" & C_CR & C_LF & (31 downto 1 => NUL), len => 20),
        1 => (command => "$PMTK314,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1*29" & C_CR & C_LF, len => 51),
        2 => (command => "$PMTK255,1*2D" & C_CR & C_LF & (36 downto 1 => NUL), len => 15)
        );

    -- TX init FSM
    signal tx_char_index : integer range 0 to 127;
    signal tx_cmd_index : integer range 0 to 31;
    signal tx_wait_cnt : integer range 0 to 15;

    type gps_tx_fsm_ty is (Idle, Tx0, Char_Incr, Cmd_Incr, Done);
    signal gps_tx_state : gps_tx_fsm_ty;
    signal gps_data_in : std_logic_vector(7 downto 0);
    signal gps_data_in_vld : std_logic;
    signal gps_tx_done : std_logic;

    -- UART RX datapath (byte + strobe out of uart_rx)
    signal gps_data_out : std_logic_vector(7 downto 0);
    signal gps_data_out_vld : std_logic;
    signal gps_uart_15k_baud : std_logic;
    signal gps_rx_clk_div : std_logic_vector(15 downto 0);

    -- Debug: 48-byte history of received bytes (handy for SignalTap)
    type del_line_ty is array (47 downto 0) of std_logic_vector(7 downto 0);
    signal gps_shift_reg : del_line_ty;

    -----------------------------------------------------------------------------
    -- UBX-NAV-TIMEUTC parser
    -----------------------------------------------------------------------------
    type ubx_state_ty is (Sync1, Sync2, Cls, Id, Len0, Len1, Payload, CkA, CkB);
    signal ubx_state : ubx_state_ty;
    signal ubx_pidx  : integer range 0 to 31;      -- payload byte index

    signal ck_a, ck_b : unsigned(7 downto 0);      -- running Fletcher checksum
    signal ck_a_ok    : std_logic;                 -- CK_A matched

    -- captured NAV-TIMEUTC fields (binary)
    signal ubx_year  : std_logic_vector(15 downto 0);
    signal ubx_month : std_logic_vector(7 downto 0);
    signal ubx_day   : std_logic_vector(7 downto 0);
    signal ubx_hour  : std_logic_vector(7 downto 0);
    signal ubx_min   : std_logic_vector(7 downto 0);
    signal ubx_sec   : std_logic_vector(7 downto 0);
    signal ubx_valid : std_logic_vector(7 downto 0);

    signal utc_update : std_logic;                 -- 1-cycle: checksum ok + validUTC

    -----------------------------------------------------------------------------
    -- Calendar -> Unix-epoch seconds (multi-cycle; runs on utc_update, once/sec)
    -----------------------------------------------------------------------------
    type cum_days_ty is array (0 to 11) of integer range 0 to 334;
    constant cum_days : cum_days_ty :=
      (0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334);

    type calc_state_ty is (Idle, Days, Secs);
    signal calc_state : calc_state_ty;
    signal cc_year  : integer range 0 to 65535;
    signal cc_month : integer range 0 to 15;
    signal cc_day   : integer range 0 to 31;
    signal cc_hour  : integer range 0 to 31;
    signal cc_min   : integer range 0 to 63;
    signal cc_sec   : integer range 0 to 63;
    signal cc_days  : integer range 0 to 100000;   -- days since 1970-01-01

    signal epoch_calc     : unsigned(31 downto 0);
    signal epoch_calc_vld : std_logic;

    -----------------------------------------------------------------------------
    -- PPS-disciplined timestamp output
    -----------------------------------------------------------------------------
    signal pip_pps      : std_logic_vector(1 downto 0);
    signal re_pps       : std_logic;
    -- PPS qualifier (glitch filter + refractory window)
    signal pps_hi_cnt   : integer range 0 to G_PPS_FILTER_CYCLES;
    signal pps_clean    : std_logic;
    signal pps_clean_d  : std_logic;
    signal pps_refr     : integer range 0 to G_PPS_REFRACTORY;
    -- cycles from the sampled PPS edge until frac_counter is (re)loaded:
    -- 2 (sync) + G_PPS_FILTER_CYCLES + 1 (pps_clean) + 1 (re_pps) = G+4
    constant C_PPS_LATENCY : natural := G_PPS_FILTER_CYCLES + 4;
    signal frac_counter : unsigned(31 downto 0);   -- sys_clock ticks since PPS
    signal epoch_sec    : unsigned(31 downto 0);    -- disciplined UTC seconds
    signal utc_locked   : std_logic;

begin  -- architecture rtl

  -----------------------------------------------------------------------------
  -- TX init FSM (kept; drives the uart_tx and the baud/RX-enable handoff)
  -----------------------------------------------------------------------------
  p_gps_tx: process (sys_clock, sys_reset) is
  begin  -- process p_gps_tx
    if (sys_reset = '0') then             -- asynchronous reset (active low)
      gps_tx_state <= Idle;
      tx_char_index <= 0;
      tx_cmd_index <= 0;

      tx_wait_cnt <= 15;

      gps_data_in <= (others => '0');
      gps_data_in_vld <= '0';

      gps_uart_15k_baud <= '0';
    elsif (rising_edge(sys_clock)) then   -- rising clock edge
      gps_data_in_vld <= '0';

      case gps_tx_state is
        when Idle =>
          gps_uart_15k_baud <= '0';
          tx_wait_cnt <= 15;

          gps_tx_state <= Char_Incr;

        when Tx0 =>
          if (gps_tx_done = '1') then
            tx_char_index <= tx_char_index + 1;
            gps_tx_state <= Char_Incr;
          end if;

        when Char_Incr =>
          if (tx_char_index < init_commands(tx_cmd_index).len ) then
            gps_data_in <= char_to_slv(init_commands(tx_cmd_index).command((tx_char_index+1)));
            gps_data_in_vld <= '1';

            gps_tx_state <= Tx0;
          else
            gps_tx_state <= Cmd_Incr;
            tx_cmd_index <= tx_cmd_index + 1;
            tx_char_index <= 0;
            tx_wait_cnt <= 15;

            -- 1st command is to set the baud rate to 115.2kbps
            if (tx_cmd_index = 0) then
              gps_uart_15k_baud <= '1';
            end if;
          end if;

        when Cmd_Incr =>
          tx_wait_cnt <= (tx_wait_cnt - 1) mod 16;

          if (tx_wait_cnt = 0) then
            if (tx_cmd_index < CMD_MAX) then
              gps_tx_state <= Char_Incr;
            else
              gps_tx_state <= Done;
            end if;
          end if;

        when Done =>
          tx_cmd_index <= 0;
          tx_char_index <= 0;

        when others =>
          gps_data_in <= (others => '0');
          gps_data_in_vld <= '0';

          gps_tx_state <= Idle;
          tx_char_index <= 0;
          tx_cmd_index <= 0;
          gps_uart_15k_baud <= '0';
      end case;
    end if;
  end process p_gps_tx;


  gps_rx_clk_div <= std_logic_vector(to_unsigned(C_GSP_DIV_CNT,16)) when gps_uart_15k_baud = '0'
                    else std_logic_vector(to_unsigned(C_GPS_DIV_CNT_115k,16));

    gps_uart_tx_1: entity work.uart_tx
      generic map (
        C_DIV_CNT => C_GSP_DIV_CNT)
      port map (
        clk         => sys_clock,
        reset_n     => sys_reset,
        clk_div => gps_rx_clk_div,
        data_in     => gps_data_in,
        data_in_vld => gps_data_in_vld,
        tx_done     => gps_tx_done,
        txd         => txd);

      gps_uart_rx_1: entity work.uart_rx
        generic map (
          C_DIV_CNT => C_GSP_DIV_CNT)
        port map (
          clk          => sys_clock,
          reset_n      => sys_reset,
          rxd          => rxd,
          enable => gps_uart_15k_baud,
          clk_div => gps_rx_clk_div,
          data_out     => gps_data_out,
          data_out_vld => gps_data_out_vld);

  -----------------------------------------------------------------------------
  -- UBX-NAV-TIMEUTC byte parser + Fletcher checksum
  -----------------------------------------------------------------------------
  p_ubx_parse: process (sys_clock, sys_reset) is
    variable b : unsigned(7 downto 0);
  begin
    if (sys_reset = '0') then
      ubx_state <= Sync1;
      ubx_pidx  <= 0;
      ck_a <= (others => '0');
      ck_b <= (others => '0');
      ck_a_ok <= '0';
      ubx_year  <= (others => '0');
      ubx_month <= (others => '0');
      ubx_day   <= (others => '0');
      ubx_hour  <= (others => '0');
      ubx_min   <= (others => '0');
      ubx_sec   <= (others => '0');
      ubx_valid <= (others => '0');
      utc_update <= '0';
    elsif (rising_edge(sys_clock)) then
      utc_update <= '0';

      if (gps_data_out_vld = '1') then
        b := unsigned(gps_data_out);

        case ubx_state is
          when Sync1 =>
            if (gps_data_out = x"B5") then
              ubx_state <= Sync2;
            end if;

          when Sync2 =>
            if (gps_data_out = x"62") then
              ubx_state <= Cls;
              ck_a <= (others => '0');   -- checksum covers class..payload
              ck_b <= (others => '0');
            elsif (gps_data_out = x"B5") then
              ubx_state <= Sync2;        -- resync on repeated 0xB5
            else
              ubx_state <= Sync1;
            end if;

          when Cls =>
            ck_a <= ck_a + b;
            ck_b <= ck_b + ck_a + b;
            if (gps_data_out = x"01") then   -- NAV
              ubx_state <= Id;
            else
              ubx_state <= Sync1;
            end if;

          when Id =>
            ck_a <= ck_a + b;
            ck_b <= ck_b + ck_a + b;
            if (gps_data_out = x"21") then   -- TIMEUTC
              ubx_state <= Len0;
            else
              ubx_state <= Sync1;
            end if;

          when Len0 =>
            ck_a <= ck_a + b;
            ck_b <= ck_b + ck_a + b;
            if (gps_data_out = x"14") then   -- length LSB = 20
              ubx_state <= Len1;
            else
              ubx_state <= Sync1;
            end if;

          when Len1 =>
            ck_a <= ck_a + b;
            ck_b <= ck_b + ck_a + b;
            if (gps_data_out = x"00") then   -- length MSB = 0
              ubx_state <= Payload;
              ubx_pidx  <= 0;
            else
              ubx_state <= Sync1;
            end if;

          when Payload =>
            ck_a <= ck_a + b;
            ck_b <= ck_b + ck_a + b;
            case ubx_pidx is
              when 12 => ubx_year(7 downto 0)  <= gps_data_out;  -- LE low byte
              when 13 => ubx_year(15 downto 8) <= gps_data_out;  -- LE high byte
              when 14 => ubx_month <= gps_data_out;
              when 15 => ubx_day   <= gps_data_out;
              when 16 => ubx_hour  <= gps_data_out;
              when 17 => ubx_min   <= gps_data_out;
              when 18 => ubx_sec   <= gps_data_out;
              when 19 => ubx_valid <= gps_data_out;
              when others => null;
            end case;
            if (ubx_pidx = 19) then
              ubx_state <= CkA;
            else
              ubx_pidx <= ubx_pidx + 1;
            end if;

          when CkA =>
            if (b = ck_a) then
              ck_a_ok <= '1';
            else
              ck_a_ok <= '0';
            end if;
            ubx_state <= CkB;

          when CkB =>
            -- checksum valid AND message's validUTC bit (bit 2) set
            if (ck_a_ok = '1' and b = ck_b and ubx_valid(2) = '1') then
              utc_update <= '1';
            end if;
            ubx_state <= Sync1;

          when others =>
            ubx_state <= Sync1;
        end case;
      end if;
    end if;
  end process p_ubx_parse;

  -----------------------------------------------------------------------------
  -- Calendar -> Unix epoch seconds (2-cycle; triggered by utc_update)
  --   leap rule year mod 4 is exact for 1970..2099
  -----------------------------------------------------------------------------
  p_epoch_calc: process (sys_clock, sys_reset) is
    variable doy  : integer range 0 to 366;
    variable leap : boolean;
  begin
    if (sys_reset = '0') then
      calc_state <= Idle;
      cc_year  <= 1970;
      cc_month <= 1;
      cc_day   <= 1;
      cc_hour  <= 0;
      cc_min   <= 0;
      cc_sec   <= 0;
      cc_days  <= 0;
      epoch_calc <= (others => '0');
      epoch_calc_vld <= '0';
    elsif (rising_edge(sys_clock)) then
      epoch_calc_vld <= '0';

      case calc_state is
        when Idle =>
          if (utc_update = '1') then
            cc_year  <= to_integer(unsigned(ubx_year));
            cc_month <= to_integer(unsigned(ubx_month));
            cc_day   <= to_integer(unsigned(ubx_day));
            cc_hour  <= to_integer(unsigned(ubx_hour));
            cc_min   <= to_integer(unsigned(ubx_min));
            cc_sec   <= to_integer(unsigned(ubx_sec));
            calc_state <= Days;
          end if;

        when Days =>
          leap := (cc_year mod 4 = 0);
          if (cc_month >= 1 and cc_month <= 12) then
            doy := cum_days(cc_month-1) + (cc_day - 1);
          else
            doy := 0;
          end if;
          if (leap and cc_month > 2) then
            doy := doy + 1;
          end if;
          -- days since 1970-01-01
          cc_days <= (cc_year - 1970)*365 + ((cc_year - 1)/4 - 492) + doy;
          calc_state <= Secs;

        when Secs =>
          epoch_calc <= resize(to_unsigned(cc_days, 17) * to_unsigned(86400, 17), 32)
                        + to_unsigned(cc_hour*3600 + cc_min*60 + cc_sec, 32);
          epoch_calc_vld <= '1';
          calc_state <= Idle;

        when others =>
          calc_state <= Idle;
      end case;
    end if;
  end process p_epoch_calc;

  -----------------------------------------------------------------------------
  -- PPS-disciplined 64-bit timestamp output
  -----------------------------------------------------------------------------
  p_time_out: process (sys_clock, sys_reset) is
  begin
    if (sys_reset = '0') then
      pip_pps <= (others => '0');
      re_pps  <= '0';
      pps_hi_cnt  <= 0;
      pps_clean   <= '0';
      pps_clean_d <= '0';
      pps_refr    <= 0;
      frac_counter <= (others => '0');
      epoch_sec    <= (others => '0');
      utc_locked   <= '0';
      timestamp_vld <= '0';
    elsif (rising_edge(sys_clock)) then
      -- synchronize PPS (2 FF)
      pip_pps <= pip_pps(0) & pps;

      -- glitch filter: pps_clean asserts only after G_PPS_FILTER_CYCLES
      -- consecutive high samples
      if (pip_pps(1) = '1') then
        if (pps_hi_cnt < G_PPS_FILTER_CYCLES) then
          pps_hi_cnt <= pps_hi_cnt + 1;
        end if;
      else
        pps_hi_cnt <= 0;
      end if;
      if (pps_hi_cnt >= G_PPS_FILTER_CYCLES) then
        pps_clean <= '1';
      else
        pps_clean <= '0';
      end if;
      pps_clean_d <= pps_clean;

      -- refractory window: once an edge is accepted, ignore the input for
      -- G_PPS_REFRACTORY cycles (a real PPS cannot recur sooner than 1 s)
      if (pps_refr /= 0) then
        pps_refr <= pps_refr - 1;
      end if;
      re_pps <= '0';
      if (pps_clean = '1' and pps_clean_d = '0' and pps_refr = 0) then
        re_pps   <= '1';
        pps_refr <= G_PPS_REFRACTORY;
      end if;

      -- fractional seconds = sys_clock ticks since the PPS edge. Preload the
      -- qualifier latency so frac still counts from the physical edge.
      if (re_pps = '1') then
        frac_counter <= to_unsigned(C_PPS_LATENCY, 32);
      else
        frac_counter <= frac_counter + 1;
      end if;

      -- seconds discipline: absolute re-sync from UBX, +1 per PPS otherwise
      if (epoch_calc_vld = '1') then
        epoch_sec  <= epoch_calc;
        utc_locked <= '1';
      elsif (re_pps = '1' and utc_locked = '1') then
        epoch_sec <= epoch_sec + 1;
      end if;

      -- pulse valid when epoch_sec takes the freshly computed value
      timestamp_vld <= epoch_calc_vld;
    end if;
  end process p_time_out;

  -- Concurrent output so timestamp tracks epoch_sec/frac_counter in lockstep with
  -- timestamp_vld (epoch_sec stays 0 until the first valid UBX => host sees "unlocked").
  timestamp <= std_logic_vector(epoch_sec) & std_logic_vector(frac_counter);


  --
  -- Debug
  --
  p_shift_line: process (sys_clock) is
  begin  -- process p_shift_line
    if (rising_edge(sys_clock)) then  -- rising clock edge
      if (gps_data_out_vld = '1') then
        gps_shift_reg <= gps_shift_reg(gps_shift_reg'high-1 downto 0) & gps_data_out;
      end if;
    end if;
  end process p_shift_line;

  -- Keep-alive: XOR-reduce internal signals into dummy_out so they survive
  -- synthesis and remain observable (dummy_out is a real debug pin).
  p_dummy: process (sys_clock) is
    variable t : std_logic;
  begin  -- process p_dummy
    if (rising_edge(sys_clock)) then  -- rising clock edge
      t := gps_data_out_vld xor utc_update xor utc_locked xor epoch_calc_vld;
      for i in 0 to 47 loop
        for j in 0 to 7 loop
          t := t xor gps_shift_reg(i)(j);
        end loop;
      end loop;
      for k in 0 to 31 loop
        t := t xor std_logic(epoch_sec(k));
        t := t xor std_logic(frac_counter(k));
      end loop;
      t := t xor ubx_valid(2);
      dummy_out <= t;
    end if;
  end process p_dummy;

end architecture rtl;
