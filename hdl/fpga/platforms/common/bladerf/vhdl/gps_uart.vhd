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
    constant C_GPS_DIV_CNT_115k : integer := C_GSP_DIV_CNT / 12;

    constant CMD_MAX : integer := 3;
    constant CMD_LEN : integer := 51;

    constant C_CR : character := character'val(13); -- ASCII CR, 0x0D
    constant C_LF : character := character'val(10); -- ASCII LF, 0x0A

    function char_to_slv(c : character) return std_logic_vector is
    begin
      return std_logic_vector(to_unsigned(character'pos(c), 8));
    end function;

    
    type pmtk_cmd_t is array (0 to CMD_LEN - 1) of std_logic_vector(7 downto 0);
    --type pmtk_cmd_t is array (0 to CMD_MAX - 1) of string(1 to CMD_LEN);
    
    type cmd_table_rec is record
      command : string(1 to CMD_LEN);
      len     : integer range 0 to 63;
    end record cmd_table_rec;
    
    type cmds_array is array (0 to CMD_MAX-1) of cmd_table_rec;
    
    constant init_commands : cmds_array :=
      (
        0 => (command => "$PMTK251,115200*1F" & C_CR & C_LF & (31 downto 1 => NUL), len => 20),
        1 => (command => "$PMTK314,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1*29" & C_CR & C_LF, len => 51), 
        2 => (command => "$PMTK255,1*2D" & C_CR & C_LF & (36 downto 1 => NUL), len => 15)
        );

       
    
    -- constant PMTK251_115200 : pmtk_cmd_t := (
    --     0  => x"24",   -- '$'
    --     1  => x"50",   -- 'P'
    --     2  => x"4D",   -- 'M'
    --     3  => x"54",   -- 'T'
    --     4  => x"4B",   -- 'K'
    --     5  => x"32",   -- '2'
    --     6  => x"35",   -- '5'
    --     7  => x"31",   -- '1'
    --     8  => x"2C",   -- ','
    --     9  => x"31",   -- '1'
    --     10 => x"31",   -- '1'
    --     11 => x"35",   -- '5'
    --     12 => x"32",   -- '2'
    --     13 => x"30",   -- '0'
    --     14 => x"30",   -- '0'
    --     15 => x"2A",   -- '*'
    --     16 => x"31",   -- '1'
    --     17 => x"46",   -- 'F'
    --     18 => x"0D",   -- '\r' (CR)
    --     19 => x"0A"    -- '\n' (LF)
    -- );

    signal tx_char_index : integer range 0 to 127;
    signal tx_cmd_index : integer range 0 to 31;
    signal tx_wait_cnt : integer range 0 to 15;
    
    type del_line_ty is array (47 downto 0) of std_logic_vector(7 downto 0);
    signal gps_shift_reg : del_line_ty;

    type gps_tx_fsm_ty is (Idle, Tx0, Char_Incr, Cmd_Incr, Done);
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
    signal gps_uart_15k_baud : std_logic;
    signal gps_rx_clk_div : std_logic_vector(15 downto 0);
    
    signal en0, en1, en2 : std_logic;
    signal rx_time_to_pps, pps_to_rx_time, rx_time_to_rx_time : unsigned(31 downto 0);

    
begin  -- architecture rtl

    -- p_gps_tx_fsm: process (sys_clock, sys_reset) is
    -- begin  -- process p_gps_tx_fsm
    --   if (sys_reset = '0') then           -- asynchronous reset (active low)
    --     gps_tx_state <= Idle;
    --     start_cnt <= to_unsigned(1023,10);

    --     gps_data_in <= (others => '0');
    --     gps_data_in_vld <= '0';

    --     gps_tx_value <= X"A4";
    --   elsif (rising_edge(sys_clock)) then  -- rising clock edge
    --     gps_data_in_vld <= '0';
        
    --     case gps_tx_state is
    --       when Idle =>
    --         start_cnt <= (start_cnt - 1) mod 1024;
    --         gps_data_in <= (others => '0');

                         
    --         if (start_cnt = 0) then
    --           gps_tx_state <= Tx0;
    --           gps_data_in <= gps_tx_value;
    --           gps_data_in_vld <= '1';
    --         end if;

    --       when Tx0 =>
    --         gps_data_in <= (others => '0');
            
    --         if ((gps_tx_done = '1')) then
    --           gps_tx_state <= Idle;
    --           start_cnt <= to_unsigned(1023,10);
    --           gps_tx_value <= not gps_tx_value;
    --         end if;
            
    --       when others =>
    --         gps_tx_value <= X"A4";
    --         gps_tx_state <= Idle;
    --         start_cnt <= to_unsigned(1023,10);

    --         gps_data_in <= (others => '0');
    --         gps_data_in_vld <= '0';
    --     end case;
    --   end if;
    -- end process p_gps_tx_fsm;


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
          --null;
          
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
          --enable => '1',          
          clk_div => gps_rx_clk_div,
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
              if (gps_data_out = X"47" or gps_data_out = X"52") then -- 'G' | 'R'
                xtract_time_state <= Pre3;
              end if;
            end if;
          when Pre3 =>
            -- Wait for 'G'
            if (gps_data_out_vld = '1') then
              xtract_time_state <= Idle;
              if (gps_data_out = X"47" or gps_data_out = X"4D") then -- 'G' | 'M'
                xtract_time_state <= Pre4;
              end if;
            end if;
          when Pre4 =>
            -- Wait for 'A'
            if (gps_data_out_vld = '1') then
              xtract_time_state <= Idle;
              if (gps_data_out = X"41" or gps_data_out = X"43") then -- 'A' | 'C'
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

    p_calc_delta: process (sys_clock, sys_reset) is
    begin  -- process p_calc_delta
      if (sys_reset = '0') then         -- asynchronous reset (active low)
        rx_time_to_pps <= (others => '0');
        pps_to_rx_time <= (others => '0');
        rx_time_to_rx_time <= (others => '0');
        
        en0 <= '0';
        en1 <= '0';
        en2 <= '0';
      elsif (rising_edge(sys_clock)) then  -- rising clock edge
        if (gps_rx_time_valid = '1') then
          en0 <= '1';
        elsif (pps = '1') then
          en0 <= '0';
        end if;

        if (gps_rx_time_valid = '1') then
          rx_time_to_pps <= (others => '0');          
        elsif (en0 = '1') then
          rx_time_to_pps <= rx_time_to_pps + 1;
        end if;
        
        if (pps = '1') then
          en1 <= '1';
        elsif (gps_rx_time_valid = '1') then
          en1 <= '0';
        end if;

        if (pps = '1') then
          pps_to_rx_time <= (others => '0');
        elsif (en1 = '1') then
          pps_to_rx_time <= pps_to_rx_time + 1;
        end if;

        if (gps_rx_time_valid = '1') then
          rx_time_to_rx_time <= (others => '0');
        else
          rx_time_to_rx_time <= rx_time_to_rx_time + 1;
        end if;
        
      end if;
    end process p_calc_delta;

    
    p_dummy: process (sys_clock) is
      variable temp0, temp1, temp2, temp3, temp4, temp5, temp6 : std_logic;
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

        for l in 0 to 31 loop
          temp4 := temp4 xor std_logic(rx_time_to_pps(l));
          temp5 := temp5 xor std_logic(pps_to_rx_time(l));
          temp6 := temp6 xor std_logic(rx_time_to_rx_time(l));
        end loop;  -- l
        
        dummy_out <= temp0 xor temp1 xor temp2 xor temp3 xor
                     temp4 xor temp5 xor temp6 xor
                     gps_data_out_vld xor gps_rx_time_valid;
        
      end if;
    end process p_dummy;

    
end architecture rtl;
