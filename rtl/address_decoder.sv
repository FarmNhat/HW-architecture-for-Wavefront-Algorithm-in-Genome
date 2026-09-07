// =============================================================================
// address_decoder.sv
// WFA Accelerator — Backtrace RAM Address Decoder
// =============================================================================
// Ánh xạ (score s) → địa chỉ RAM.
//
// Trong thiết kế này, mỗi score step s tương ứng với 1 từ RAM:
//   addr = s  (ghi tuần tự từ s=0, 1, 2, ...)
//
// Module này là pure combinational — chỉ cần 1 thanh ghi lưu "base address"
// (offset bắt đầu của alignment hiện tại) nếu nhiều alignment dùng chung RAM.
// Trong thiết kế 1 Aligner / 1 RAM, base_addr = 0.
//
// Điều kiện: addr phải < BT_RAM_DEPTH, nếu không → addr_valid = 0.
// =============================================================================

`default_nettype none
`include "pkg_wfa_params.sv"

module address_decoder
  import wfa_pkg::*;
#(
  parameter int RAM_DEPTH_P = BT_RAM_DEPTH,
  parameter int S_WIDTH_P   = S_WIDTH
) (
  // Input: score step (từ backtrace FSM)
  input  logic [S_WIDTH_P-1:0]               score_in,  // current score s

  // Output: RAM address
  output logic [$clog2(RAM_DEPTH_P)-1:0]     ram_addr,  // address into BT RAM
  output logic                               addr_valid  // 1 = address is in range
);

  localparam int ADDR_W = $clog2(RAM_DEPTH_P);

  always_comb begin
    if (score_in < RAM_DEPTH_P) begin
      ram_addr   = ADDR_W'(score_in);
      addr_valid = 1'b1;
    end else begin
      ram_addr   = '0;
      addr_valid = 1'b0;
    end
  end

endmodule

`default_nettype wire
