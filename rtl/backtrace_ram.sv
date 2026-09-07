// =============================================================================
// backtrace_ram.sv
// WFA Accelerator — Backtrace RAM
// =============================================================================
// Single-port synchronous RAM (Xilinx BRAM inference style).
// Width  = BT_RAM_WIDTH  = 40 bits (8 cells × 5-bit origin per row)
// Depth  = BT_RAM_DEPTH  = 250 words (indexed by score step s)
//
// Write: 1 word/cycle when wr_en=1 (called once per score step during compute)
// Read:  registered output (1-cycle latency) when rd_en=1
//
// Địa chỉ: được quản lý bởi controller bên ngoài.
//   Lúc compute: ghi tuần tự wr_addr = 0, 1, 2, ...
//   Lúc backtrace: đọc theo rd_addr = f(score)
// =============================================================================

`default_nettype none
`include "pkg_wfa_params.sv"

module backtrace_ram
  import wfa_pkg::*;
#(
  parameter int RAM_WIDTH_P = BT_RAM_WIDTH,  // 40 bits
  parameter int RAM_DEPTH_P = BT_RAM_DEPTH   // 250 words
) (
  input  logic                                clk,

  // Write port
  input  logic                                wr_en,    // Write enable
  input  logic [$clog2(RAM_DEPTH_P)-1:0]     wr_addr,  // Write address
  input  logic [RAM_WIDTH_P-1:0]             wr_data,  // Data to write

  // Read port (registered output, 1-cycle latency)
  input  logic                                rd_en,    // Read enable
  input  logic [$clog2(RAM_DEPTH_P)-1:0]     rd_addr,  // Read address
  output logic [RAM_WIDTH_P-1:0]             rd_data   // Read data (next cycle)
);

  // ---------------------------------------------------------------------------
  // Memory array
  // Declared as logic array → Vivado infers Block RAM automatically
  // ---------------------------------------------------------------------------
  logic [RAM_WIDTH_P-1:0] mem [0:RAM_DEPTH_P-1];

  // ---------------------------------------------------------------------------
  // Write logic (synchronous)
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk) begin
    if (wr_en)
      mem[wr_addr] <= wr_data;
  end

  // ---------------------------------------------------------------------------
  // Read logic (synchronous, registered output = BRAM read-first behavior)
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk) begin
    if (rd_en)
      rd_data <= mem[rd_addr];
  end

endmodule

`default_nettype wire
