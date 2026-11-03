library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity ts_cdc_tb is end entity;

architecture sim of ts_cdc_tb is
  signal src_clk : std_logic := '0';
  signal dst_clk : std_logic := '0';
  signal src_data, dst_data : std_logic_vector(63 downto 0);
  signal running : boolean := true;
  signal cnt : unsigned(31 downto 0) := (others=>'0');
  signal checks, tears : integer := 0;
begin
  -- 80 MHz source
  src_clk <= not src_clk after 6.25 ns when running else '0';
  -- ~61.44 MHz destination (asynchronous, different period) 
  dst_clk <= not dst_clk after 8.138 ns when running else '0';

  dut: entity work.ts_cdc
    generic map (WIDTH => 64)
    port map (src_clk=>src_clk, src_data=>src_data, dst_clk=>dst_clk, dst_data=>dst_data);

  -- src_data invariant: high 32 = NOT low 32, and low 32 = free-running counter
  -- that changes EVERY src cycle (models the fraction counter).
  p_src: process(src_clk) begin
    if rising_edge(src_clk) then
      cnt <= cnt + 1;
    end if;
  end process;
  src_data <= std_logic_vector(not cnt) & std_logic_vector(cnt);

  -- destination: every dst cycle, dst_data must satisfy high = NOT low (atomic),
  -- never a torn mix of two different source values.
  p_chk: process(dst_clk) begin
    if rising_edge(dst_clk) then
      if dst_data /= x"00000000_00000000" then   -- skip initial reset value
        checks <= checks + 1;
        if dst_data(63 downto 32) /= not dst_data(31 downto 0) then
          tears <= tears + 1;
          report "TEAR: dst_data high /= not low at check " & integer'image(checks)
            severity error;
        end if;
      end if;
    end if;
  end process;

  p_stim: process begin
    wait for 500 us;             -- ~40000 src cycles, ~61000 dst cycles
    running <= false;
    wait for 100 ns;
    report "checks=" & integer'image(checks) & "  tears=" & integer'image(tears);
    if tears = 0 and checks > 1000 then
      report "TS_CDC PASS: no torn samples across the crossing" severity note;
    else
      report "TS_CDC FAIL" severity failure;
    end if;
    wait;
  end process;
end architecture;
