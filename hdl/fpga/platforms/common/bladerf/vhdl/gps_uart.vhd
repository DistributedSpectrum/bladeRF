library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity gps_uart is
  
  port (
    sys_clock       : in  std_logic;
    sys_reset   : in  std_logic;
    rxd       : in  std_logic;
    pps : in std_logic;
    timestamp : out std_logic_vector(3*16-1 downto 0);
    timestamp_vld : out std_logic;
    txd       : out std_logic;
    dummy_out : out std_logic);

end entity gps_uart;

architecture rtl of gps_uart is
    constant C_GSP_DIV_CNT : integer := 4340;
  
    type del_line_ty is array (47 downto 0) of std_logic_vector(7 downto 0);
    signal gps_shift_reg : del_line_ty;

    type gps_tx_fsm_ty is (Idle, Tx0);
    signal gps_tx_state : gps_tx_fsm_ty;
    signal start_cnt : unsigned(9 downto 0);
    signal gps_data_in : std_logic_vector(7 downto 0);
    signal gps_data_in_vld : std_logic;
    signal gps_tx_done : std_logic;
    signal gps_tx_value : std_logic_vector(7 downto 0);

    type xtract_time_state_ty is (Idle, Pre0, Pre1, Pre2, Pre3, Pre4, Pre5,
                                  H0, H1, M0, M1, S0, S1,
                                  NL, CR);
    signal xtract_time_state : xtract_time_state_ty;
    signal hour, minute, second : std_logic_vector(15 downto 0);
    signal gps_rx_time_valid : std_logic;
    
    signal gps_data_out : std_logic_vector(7 downto 0);
    signal gps_data_out_vld : std_logic;


begin  -- architecture rtl

    p_gps_tx_fsm: process (sys_clock, sys_reset) is
    begin  -- process p_gps_tx_fsm
      if (sys_reset = '0') then           -- asynchronous reset (active low)
        gps_tx_state <= Idle;
        start_cnt <= to_unsigned(1023,10);

        gps_data_in <= (others => '0');
        gps_data_in_vld <= '0';

        gps_tx_value <= X"A4";
      elsif (rising_edge(sys_clock)) then  -- rising clock edge
        gps_data_in_vld <= '0';
        
        case gps_tx_state is
          when Idle =>
            start_cnt <= (start_cnt - 1) mod 1024;
            gps_data_in <= (others => '0');

                         
            if (start_cnt = 0) then
              gps_tx_state <= Tx0;
              gps_data_in <= gps_tx_value;
              gps_data_in_vld <= '1';
            end if;

          when Tx0 =>
            gps_data_in <= (others => '0');
            
            if ((gps_tx_done = '1')) then
              gps_tx_state <= Idle;
              start_cnt <= to_unsigned(1023,10);
              gps_tx_value <= not gps_tx_value;
            end if;
            
          when others =>
            gps_tx_value <= X"A4";
            gps_tx_state <= Idle;
            start_cnt <= to_unsigned(1023,10);

            gps_data_in <= (others => '0');
            gps_data_in_vld <= '0';
        end case;
      end if;
    end process p_gps_tx_fsm;

  
    gps_uart_tx_1: entity work.uart_tx
      generic map (
        C_DIV_CNT => C_GSP_DIV_CNT)
      port map (
        clk         => sys_clock,
        reset_n     => sys_reset,
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
          data_out     => gps_data_out,
          data_out_vld => gps_data_out_vld);
  
    p_xtract_time: process (sys_clock, sys_reset) is
    begin  -- process p_xtract_time
      if (sys_reset = '0') then         -- asynchronous reset (active low)
        xtract_time_state <= Idle;
        second <= (others => '0');
        minute <= (others => '0');
        hour <= (others => '0');

        gps_rx_time_valid <= '0';
      elsif (rising_edge(sys_clock)) then  -- rising clock edge
        gps_rx_time_valid <= '0';
        
        case xtract_time_state is
          when Idle =>
            -- Wait for '$'
            if (gps_data_out_vld = '1' and gps_data_out = X"24") then
              xtract_time_state <= Pre0;
            end if;
          when Pre0 =>
            -- Wait for 'G'
            if (gps_data_out_vld = '1') then
              xtract_time_state <= Idle;
              if (gps_data_out = X"47") then -- 'G'
                xtract_time_state <= Pre1;
              end if;
            end if;
          when Pre1 =>
            if (gps_data_out_vld = '1') then            
              xtract_time_state <= Pre2;
            end if;
          when Pre2 =>
            -- Wait for 'G'
            if (gps_data_out_vld = '1') then
              xtract_time_state <= Idle;
              if (gps_data_out = X"47") then -- 'G'
                xtract_time_state <= Pre3;
              end if;
            end if;
          when Pre3 =>
            -- Wait for 'G'
            if (gps_data_out_vld = '1') then
              xtract_time_state <= Idle;
              if (gps_data_out = X"47") then -- 'G'
                xtract_time_state <= Pre4;
              end if;
            end if;
          when Pre4 =>
            -- Wait for 'A'
            if (gps_data_out_vld = '1') then
              xtract_time_state <= Idle;
              if (gps_data_out = X"41") then -- 'A'
                xtract_time_state <= Pre5;
              end if;
            end if;
          when Pre5 =>
            if (gps_data_out_vld = '1') then
              xtract_time_state <= Idle;
              if (gps_data_out = X"2C") then -- ','
                xtract_time_state <= H0;
              end if;
            end if;
            
          when H0 =>
            if (gps_data_out_vld = '1') then
              hour(15 downto 8) <= gps_data_out;
              xtract_time_state <= H1;
            end if;
          when H1 =>
            if (gps_data_out_vld = '1') then
              hour(7 downto 0) <= gps_data_out;
              xtract_time_state <= M0;
            end if;

          when M0 =>
            if (gps_data_out_vld = '1') then
              minute(15 downto 8) <= gps_data_out;
              xtract_time_state <= M1;
            end if;
          when M1 =>
            if (gps_data_out_vld = '1') then
              minute(7 downto 0) <= gps_data_out;
              xtract_time_state <= S0;
            end if;
            
          when S0 =>
            if (gps_data_out_vld = '1') then
              second(15 downto 8) <= gps_data_out;
              xtract_time_state <= S1;
            end if;
          when S1 =>
            if (gps_data_out_vld = '1') then
              second(7 downto 0) <= gps_data_out;
              xtract_time_state <= CR;
              gps_rx_time_valid <= '1';
            end if;

          when CR =>
            if (gps_data_out_vld = '1') then
              if (gps_data_out = X"0D") then -- CR
                xtract_time_state <= NL;
              end if;
            end if;
          when NL =>
            if (gps_data_out_vld = '1') then
              if (gps_data_out = X"0A") then -- NL
                xtract_time_state <= Idle;
              end if;
            end if;
            
            
          when others =>
            xtract_time_state <= Idle;
            second <= (others => '0');
            minute <= (others => '0');
            hour <= (others => '0');
            gps_rx_time_valid <= '0';
        end case;
      end if;
    end process p_xtract_time;

    p_shift_line: process (sys_clock) is
    begin  -- process p_shift_line
      if (rising_edge(sys_clock)) then  -- rising clock edge
        if (gps_data_out_vld = '1') then
          gps_shift_reg <= gps_shift_reg(gps_shift_reg'high-1 downto 0) & gps_data_out;
        end if;
      end if;
    end process p_shift_line;
    
    p_dummy: process (sys_clock) is
      variable temp0, temp1, temp2, temp3 : std_logic;
    begin  -- process p_dummy
      if (rising_edge(sys_clock)) then  -- rising clock edge
        for i in 0 to 47 loop
          for j in 0 to 7 loop
            temp0 := temp0 xor gps_shift_reg(i)(j);            
          end loop;  -- j
        end loop;  -- i

        for k in 0 to 15 loop
          temp1 := temp1 xor hour(k);
          temp2 := temp2 xor minute(k);
          temp3 := temp3 xor second(k);
        end loop;  -- k
        
        dummy_out <= temp0 xor temp1 xor temp2 xor temp3 xor
                     gps_data_out_vld xor gps_rx_time_valid;
        
      end if;
    end process p_dummy;

    
end architecture rtl;
