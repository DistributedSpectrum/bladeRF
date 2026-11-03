library ieee; use ieee.std_logic_1164.all; use ieee.numeric_std.all;
entity pps_qual_tb is end entity;
architecture sim of pps_qual_tb is
  signal clk : std_logic := '0'; signal rst_n : std_logic := '0';
  signal pps : std_logic := '0'; signal ts : std_logic_vector(63 downto 0);
  signal vld, txd, dummy : std_logic; signal running : boolean := true;
  signal fails : integer := 0;
  function frac(t : std_logic_vector(63 downto 0)) return integer is
  begin return to_integer(unsigned(t(31 downto 0))); end;
begin
  clk <= not clk after 6.25 ns when running else '0';   -- 80 MHz
  dut : entity work.gps_uart
    generic map (G_PPS_FILTER_CYCLES => 8, G_PPS_REFRACTORY => 8000)  -- 100 us window
    port map (sys_clock => clk, sys_reset => rst_n, rxd => '1', pps => pps,
              timestamp => ts, timestamp_vld => vld, txd => txd, dummy_out => dummy);
  process
    variable a : integer;
    procedure accepted(msg : string) is begin            -- call 500 ns after a rising edge
      if frac(ts) > 120 then fails <= fails + 1;
        report "FAIL " & msg & ": frac=" & integer'image(frac(ts)) & " (edge NOT accepted)" severity error;
      else report "ok   " & msg & ": accepted, frac=" & integer'image(frac(ts)); end if; end;
    procedure rejected(before : integer; msg : string) is begin
      if frac(ts) <= before then fails <= fails + 1;
        report "FAIL " & msg & ": frac " & integer'image(before) & "->" & integer'image(frac(ts)) & " (edge was ACCEPTED)" severity error;
      else report "ok   " & msg & ": rejected, frac kept counting " & integer'image(before) & "->" & integer'image(frac(ts)); end if; end;
  begin
    wait for 100 ns; rst_n <= '1'; wait for 10 us;
    -- 1) real PPS -> accepted
    pps <= '1'; wait for 500 ns; accepted("real PPS #1"); wait for 19.5 us; pps <= '0';
    -- 2) burst of WIDE pulses inside the refractory window (models UART crosstalk) -> rejected
    wait for 10 us; a := frac(ts);
    for i in 1 to 5 loop pps <= '1'; wait for 5 us; pps <= '0'; wait for 5 us; end loop;
    rejected(a, "5 wide pulses inside refractory");
    -- 3) refractory expired (t>110us): narrow glitches -> rejected by the filter
    wait for 60 us; a := frac(ts);
    for i in 1 to 10 loop pps <= '1'; wait for 37.5 ns; pps <= '0'; wait for 1 us; end loop;
    rejected(a, "10 x 3-cycle glitches after refractory");
    -- 4) 6-cycle pulse (below the 8-cycle filter) -> rejected
    a := frac(ts); pps <= '1'; wait for 75 ns; pps <= '0'; wait for 1 us;
    rejected(a, "6-cycle pulse");
    -- 5) real PPS again -> accepted (window expired, filter passes a real pulse)
    wait for 10 us; pps <= '1'; wait for 500 ns; accepted("real PPS #2"); wait for 19.5 us; pps <= '0';
    -- 6) 12-cycle pulse after this window expires -> accepted (filter boundary sanity)
    wait for 110 us; pps <= '1'; wait for 150 ns; pps <= '0'; wait for 350 ns; accepted("12-cycle pulse after window");
    wait for 1 us; running <= false; wait for 50 ns;
    if fails = 0 then report "PPS_QUAL PASS: glitches/bursts rejected, real PPS accepted" severity note;
    else report "PPS_QUAL FAIL: " & integer'image(fails) & " check(s) failed" severity failure; end if;
    wait;
  end process;
end architecture;
