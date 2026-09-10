library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
--use ieee.std_logic_arith.all;

entity uart_rx is
  generic (
    C_DIV_CNT : integer := 430;
    C_FE_CNT : integer := 0);
  port (
    clk     : in  std_logic;
    reset_n : in  std_logic;
    rxd     : in  std_logic;
    data_out : out std_logic_vector(7 downto 0);
    data_out_vld : out std_logic);


end entity uart_rx;

architecture rtl of uart_rx is
  constant C_DIV_CNT_START : integer := C_DIV_CNT/2;
  constant C_MOD_MAX : integer := 16384;
  
  signal pip_rxd : std_logic_vector(1 downto 0);
  signal fe_rxd  : std_logic;

  signal div_cnt : integer range 0 to 16383;
  signal div_cnt_en : std_logic;
  
  signal rx_data : std_logic_vector(7 downto 0);

  signal rx_shift_en,pip_rx_shift_en : std_logic;

  signal bit_cnt : integer range 0 to 7;
    
  type fsm_ty is (Idle,Rx_Start,Rx,Rx_Stop);
  signal state : fsm_ty;
  
begin  -- architecture rtl

  p_reg: process (clk, reset_n) is
  begin  -- process p_reg
    if (reset_n = '0') then             -- asynchronous reset (active low)
      pip_rxd <= "11";
    elsif (rising_edge(clk)) then       -- rising clock edge
      pip_rxd <= pip_rxd(0) & rxd;
    end if;
  end process p_reg;

  p_fe_rxd: process (clk, reset_n) is
  begin  -- process p_fe_rxd
    if (reset_n = '0') then             -- asynchronous reset (active low)
      fe_rxd <= '0';
    elsif (rising_edge(clk)) then       -- rising clock edge
      fe_rxd <= '0';
      if (pip_rxd = "10") then
        fe_rxd <= '1';
      end if;
    end if;
  end process p_fe_rxd;

  p_rx_shift_en: process (clk, reset_n) is
  begin  -- process p_rx_shift_en
    if (reset_n = '0') then             -- asynchronous reset (active low)
      rx_shift_en <= '0';
      div_cnt <= C_DIV_CNT_START;
      div_cnt_en <= '0';
      pip_rx_shift_en <= '0';
      bit_cnt <= 0;
    elsif (rising_edge(clk)) then       -- rising clock edge
      div_cnt_en <= not div_cnt_en;

      if (div_cnt_en = '1') then
        if (div_cnt = 0) then
          if (state = Idle or state = Rx_Stop) then
            div_cnt <= C_DIV_CNT_START;
          else
            div_cnt <= C_DIV_CNT;
          end if;

        elsif (state /= Idle) then
          div_cnt <= (div_cnt - 1) mod C_MOD_MAX;
        end if;
      end if;

      rx_shift_en <= '0';
      if (div_cnt = 0 and div_cnt_en = '1') then
        rx_shift_en <= '1';
      end if;
      pip_rx_shift_en <= rx_shift_en;

      if (state = Rx and rx_shift_en = '1') then
        bit_cnt <= (bit_cnt + 1) mod 8;
      end if;
      
    end if;
  end process p_rx_shift_en;

  p_uart: process (clk, reset_n) is
  begin  -- process p_uart
    if (reset_n = '0') then             -- asynchronous reset (active low)
      state <= Idle;
      rx_data <= (others => '0');
      data_out <= (others => '0');
      data_out_vld <= '0';
    elsif (rising_edge(clk)) then       -- rising clock edge
      data_out_vld <= '0';
      
      case state is
        when Idle =>
          if (fe_rxd = '1') then
            state <= Rx_Start;
            rx_data <= (others => '0');
          end if;
        when Rx_Start =>
          if (rx_shift_en = '1') then
            state <= Rx;
          end if;
          
        when Rx =>
          if (rx_shift_en = '1') then
            rx_data <= pip_rxd(1) & rx_data(7 downto 1);
          end if;

          if (rx_shift_en = '1' and bit_cnt = 7) then
            state <= Rx_Stop;
          end if;

        when Rx_Stop =>
          -- Did not get a stop bit
          if (rx_shift_en = '1' and pip_rxd(1) /= '1') then
            rx_data <= (others => '0');
            state <= Idle;
          elsif (rx_shift_en = '1') then
            data_out_vld <= '1';
            data_out <= rx_data;
            state <= Idle;
          end if;
          
        when others =>
          state <= Idle;
          rx_data <= (others => '0');
          data_out <= (others => '0');
          data_out_vld <= '0';
      end case;
    end if;
  end process p_uart;

end architecture rtl;
