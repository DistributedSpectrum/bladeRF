library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
--use ieee.std_logic_arith.all;

entity uart_tx is
  generic (
    C_DIV_CNT : integer := 430);
  port (
    clk     : in  std_logic;
    reset_n : in  std_logic;

    clk_div : in std_logic_vector(15 downto 0);
    data_in : in std_logic_vector(7 downto 0);
    data_in_vld : in std_logic;
    tx_done : out std_logic;
    txd : out std_logic);
end entity uart_tx;

architecture rtl of uart_tx is
  constant C_MOD_MAX : integer := 65536;
  
  signal div_cnt : integer range 0 to 65535;
  signal div_cnt_en : std_logic;
  
  signal tx_data : std_logic_vector(7 downto 0);

  signal shift_en,pip1_shift_en : std_logic;

  signal bit_cnt : integer range 0 to 7;
    
  type fsm_ty is (Idle,Tx_Start,Tx,Tx_Stop);
  signal state : fsm_ty;
  
begin  -- architecture rtl
  
  p_shift_en: process (clk, reset_n) is
  begin  -- process p_shift_en
    if (reset_n = '0') then             -- asynchronous reset (active low)
      shift_en <= '0';
      pip1_shift_en <= '0';
      div_cnt <= to_integer(unsigned(clk_div));
      div_cnt_en <= '0';
      bit_cnt <= 0;
    elsif (rising_edge(clk)) then       -- rising clock edge
      div_cnt_en <= not div_cnt_en;

      if (div_cnt_en = '1') then
        if (state = Idle or div_cnt = 0) then
          div_cnt <= to_integer(unsigned(clk_div));
        elsif (state /= Idle) then
          div_cnt <= (div_cnt - 1) mod C_MOD_MAX;
        end if;
      end if;

      shift_en <= '0';
      if (div_cnt = 0 and div_cnt_en = '1') then
        shift_en <= '1';
      end if;
      pip1_shift_en <= shift_en;
      
      if (state = Tx and shift_en = '1') then
        bit_cnt <= (bit_cnt + 1) mod 8;
      end if;
      
    end if;
  end process p_shift_en;

  p_uart: process (clk, reset_n) is
  begin  -- process p_uart
    if (reset_n = '0') then             -- asynchronous reset (active low)
      state <= Idle;
      tx_data <= (others => '0');
      txd <= '1';
      tx_done <= '0';
    elsif (rising_edge(clk)) then       -- rising clock edge
      tx_done <= '0';
      case state is
        when Idle =>
          if (data_in_vld = '1') then
            state <= Tx_Start;
            tx_data <= data_in;
            txd <= '0';
          end if;
        when Tx_Start =>
          if (shift_en = '1') then
            state <= Tx;
            txd <= tx_data(0);
          end if;
          
        when Tx =>
          if (pip1_shift_en = '1') then
            txd <= tx_data(0);
            tx_data <= '0' & tx_data(7 downto 1);
          end if;

          if (shift_en = '1' and bit_cnt = 7) then
            state <= Tx_Stop;
            txd <= '1';
          end if;

        when Tx_Stop =>
          if (shift_en = '1') then
            state <= Idle;
            tx_done <= '1';
          end if;
          
        when others =>
          state <= Idle;
      end case;
    end if;
  end process p_uart;

end architecture rtl;
