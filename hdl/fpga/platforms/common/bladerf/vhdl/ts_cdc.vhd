library ieee;
use ieee.std_logic_1164.all;

-------------------------------------------------------------------------------
-- ts_cdc
--
-- Coherent (tear-free) clock-domain crossing for the 64-bit GPS timestamp.
--
-- The gps_uart timestamp is generated in the sys_clock domain and changes every
-- cycle (the sub-second fraction counter free-runs at sys_clock). The RX block
-- latches it in the rx_clock (AD9361) domain at the start of each DMA block. A
-- raw multi-bit sample across that boundary tears: when the latch instant
-- coincides with the per-second update (PPS rollover: epoch += 1, fraction -> 0),
-- some bits are captured pre-update and some post-update, yielding a garbage
-- timestamp (observed on hardware as ~7% of blocks reading epoch 0 or a few
-- seconds high).
--
-- This module crosses the word with a toggle/ack handshake: the source latches
-- a snapshot into a holding register and flips a request toggle; it does not
-- latch the next snapshot until the destination has acknowledged, so the held
-- value is guaranteed stable while the destination captures it. The destination
-- captures the (stable) holding register on each synchronized request edge. The
-- result (dst_data) therefore only ever updates as one atomic 64-bit value in
-- the rx_clock domain. The capture-to-capture latency is one handshake round
-- trip (a few cycles each way, << 1 us), so dst_data is at most sub-microsecond
-- stale -- far finer than a DMA block and well within the PPS-disciplined
-- fraction resolution.
-------------------------------------------------------------------------------

entity ts_cdc is
  generic (
    WIDTH : natural := 64);
  port (
    src_clk  : in  std_logic;
    src_data : in  std_logic_vector(WIDTH-1 downto 0);
    dst_clk  : in  std_logic;
    dst_data : out std_logic_vector(WIDTH-1 downto 0));
end entity ts_cdc;

architecture rtl of ts_cdc is

  -- source (src_clk) domain
  signal src_hold : std_logic_vector(WIDTH-1 downto 0) := (others => '0');
  signal req_tgl  : std_logic := '0';
  signal ack_s1   : std_logic := '0';   -- ack toggle synchronized into src
  signal ack_s2   : std_logic := '0';

  -- destination (dst_clk) domain
  signal req_s1   : std_logic := '0';   -- req toggle synchronized into dst
  signal req_s2   : std_logic := '0';
  signal req_s3   : std_logic := '0';
  signal ack_tgl  : std_logic := '0';
  signal dst_reg  : std_logic_vector(WIDTH-1 downto 0) := (others => '0');

  -- keep the synchronizer flops as discrete registers (no merge/retime)
  attribute preserve      : boolean;
  attribute preserve of ack_s1  : signal is true;
  attribute preserve of ack_s2  : signal is true;
  attribute preserve of req_s1  : signal is true;
  attribute preserve of req_s2  : signal is true;
  attribute preserve of req_s3  : signal is true;

begin

  -----------------------------------------------------------------------------
  -- Source domain: publish a fresh snapshot whenever the previous handshake
  -- has completed (req caught up by ack). While a transfer is in flight the
  -- holding register is frozen, guaranteeing stable data for the destination.
  -----------------------------------------------------------------------------
  p_src : process (src_clk) is
  begin
    if rising_edge(src_clk) then
      ack_s1 <= ack_tgl;
      ack_s2 <= ack_s1;
      if (req_tgl = ack_s2) then        -- handshake idle -> capture + request
        src_hold <= src_data;
        req_tgl  <= not req_tgl;
      end if;
    end if;
  end process p_src;

  -----------------------------------------------------------------------------
  -- Destination domain: on each synchronized request edge the held value is
  -- known stable, so capture it atomically; echo the request back as the ack.
  -----------------------------------------------------------------------------
  p_dst : process (dst_clk) is
  begin
    if rising_edge(dst_clk) then
      req_s1 <= req_tgl;
      req_s2 <= req_s1;
      req_s3 <= req_s2;
      if (req_s3 /= req_s2) then         -- req toggled -> new stable snapshot
        dst_reg <= src_hold;
      end if;
      ack_tgl <= req_s2;                 -- acknowledge back to source
    end if;
  end process p_dst;

  dst_data <= dst_reg;

end architecture rtl;
