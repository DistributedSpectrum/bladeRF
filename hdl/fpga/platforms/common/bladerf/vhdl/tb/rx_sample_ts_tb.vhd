library ieee; use ieee.std_logic_1164.all; use ieee.numeric_std.all;
-- rx_sample_ts: seconds+sample-count coherence across async sys/rx clocks.
entity rx_sample_ts_tb is end entity;
architecture sim of rx_sample_ts_tb is
  signal sys_clk : std_logic := '0'; signal rx_clk : std_logic := '0'; signal running : boolean := true;
  signal epoch : std_logic_vector(31 downto 0) := (others=>'0'); signal locked, sync_tgl, pps_tgl : std_logic := '0';
  signal sample_v : std_logic := '1';      -- one sample every rx cycle (40 MSps)
  signal ts : std_logic_vector(63 downto 0); signal mfails, sfails : integer := 0; signal checks : integer := 0;
  function ep(t: std_logic_vector(63 downto 0)) return integer is begin return to_integer(unsigned(t(63 downto 32))); end;
  function sc(t: std_logic_vector(63 downto 0)) return integer is begin return to_integer(unsigned(t(31 downto 0))); end;
begin
  sys_clk <= not sys_clk after 6.25 ns when running else '0';   -- 80 MHz
  rx_clk  <= not rx_clk  after 12.5 ns when running else '0';   -- 40 MHz, async phase
  dut: entity work.rx_sample_ts port map (sys_clock=>sys_clk, epoch_sec=>epoch, utc_locked=>locked,
       sync_tgl=>sync_tgl, pps_tgl=>pps_tgl, rx_clock=>rx_clk, sample_v=>sample_v, timestamp=>ts);
  -- coherence monitor: every rx cycle the count grows by exactly 1, except in the
  -- cycle the seconds change, where the count must restart (0 or 1) in that same cycle
  p_mon: process(rx_clk) variable pe, ps : integer := 0; variable first : boolean := true; begin
    if rising_edge(rx_clk) then
      if not first then checks <= checks + 1;
        if ep(ts) /= pe then
          if ep(ts) = pe + 1 then  -- PPS: count must restart in this very cycle
            if sc(ts) > 1 then mfails <= mfails + 1; report "TEAR: seconds advanced but count=" & integer'image(sc(ts)) severity error; end if;
          end if;                  -- a UBX load may change seconds without touching the count
        elsif sc(ts) /= ps + 1 and sc(ts) > 1 then
          mfails <= mfails + 1; report "count step " & integer'image(ps) & "->" & integer'image(sc(ts)) severity error;
        end if;
      end if; first := false; pe := ep(ts); ps := sc(ts);
    end if; end process;
  p_stim: process
    procedure expect(cond: boolean; msg: string) is begin
      if cond then report "ok   " & msg; else report "FAIL " & msg severity error; sfails <= sfails + 1; end if; end;
  begin
    wait for 2 us;
    -- PPS while unlocked: count restarts, seconds stay 0
    wait until rising_edge(sys_clk); pps_tgl <= not pps_tgl; wait for 2 us;
    expect(ep(ts) = 0 and sc(ts) < 90, "PPS unlocked: epoch 0, count restarted (" & integer'image(sc(ts)) & ")");
    -- UBX load 42 ms-style (here 20 us later): epoch 1000, locked
    wait for 18 us; wait until rising_edge(sys_clk); epoch <= std_logic_vector(to_unsigned(1000,32)); locked <= '1'; sync_tgl <= not sync_tgl;
    wait for 2 us; expect(ep(ts) = 1000, "UBX load: epoch=" & integer'image(ep(ts)));
    expect(sc(ts) > 800, "UBX load left the count running (" & integer'image(sc(ts)) & ")");
    -- next PPS: +1 second and count restart in the same cycle (sys side mirrors gps_uart: epoch_sec+1)
    wait for 8 us; wait until rising_edge(sys_clk); pps_tgl <= not pps_tgl; epoch <= std_logic_vector(to_unsigned(1001,32));
    wait for 1 us; expect(ep(ts) = 1001 and sc(ts) < 45, "PPS locked: epoch=" & integer'image(ep(ts)) & " count=" & integer'image(sc(ts)));
    -- UBX resync to the same second: no change
    wait for 20 us; wait until rising_edge(sys_clk); sync_tgl <= not sync_tgl; wait for 2 us;
    expect(ep(ts) = 1001 and sc(ts) > 800, "UBX resync same second: unchanged");
    -- a UBX correction (+5): absolute load wins
    wait for 5 us; wait until rising_edge(sys_clk); epoch <= std_logic_vector(to_unsigned(1006,32)); sync_tgl <= not sync_tgl; wait for 2 us;
    expect(ep(ts) = 1006, "UBX correction applied: epoch=" & integer'image(ep(ts)));
    -- several more PPS, checked by the monitor
    for i in 1 to 5 loop wait for 30 us; wait until rising_edge(sys_clk); pps_tgl <= not pps_tgl; epoch <= std_logic_vector(unsigned(epoch)+1); end loop;
    wait for 5 us; expect(ep(ts) = 1011, "after 5 PPS: epoch=" & integer'image(ep(ts)));
    running <= false; wait for 50 ns;
    report "monitor checks=" & integer'image(checks) & " fails=" & integer'image(mfails + sfails);
    if mfails + sfails = 0 then report "RX_SAMPLE_TS PASS" severity note; else report "RX_SAMPLE_TS FAIL" severity failure; end if;
    wait;
  end process;
end architecture;
