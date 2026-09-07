// =============================================================================
// base_encoder.sv
// WFA Accelerator — Base Encoder (ASCII → 2-bit combinational LUT)
// =============================================================================
// Chuyển đổi ký tự ASCII của base DNA (A/C/G/T) thành mã 2-bit:
//   A (0x41) → 2'b00
//   C (0x43) → 2'b01
//   G (0x47) → 2'b10
//   T (0x54) → 2'b11
//   other    → 2'b00 (default, sẽ ghi cờ error nếu cần)
//
// Module này là pure combinational, không dùng clock.
// Instantiate nhiều lần trong Group Maker để encode song song 8 bases.
// =============================================================================

`default_nettype none

module base_encoder (
  // Input: ký tự ASCII 8-bit (byte)
  input  logic [7:0] ascii_in,   // ASCII byte: 'A'=0x41, 'C'=0x43, 'G'=0x47, 'T'=0x54

  // Output: mã 2-bit tương ứng
  output logic [1:0] base_out,   // encoded base: 00=A, 01=C, 10=G, 11=T

  // Output: flag báo ký tự không hợp lệ (không phải A/C/G/T)
  output logic       invalid_base // 1 = ký tự không nhận ra
);

  // Bảng chuyển đổi combinational
  always_comb begin
    case (ascii_in)
      8'h41, 8'h61: begin base_out = 2'b00; invalid_base = 1'b0; end // 'A' or 'a'
      8'h43, 8'h63: begin base_out = 2'b01; invalid_base = 1'b0; end // 'C' or 'c'
      8'h47, 8'h67: begin base_out = 2'b10; invalid_base = 1'b0; end // 'G' or 'g'
      8'h54, 8'h74: begin base_out = 2'b11; invalid_base = 1'b0; end // 'T' or 't'
      default:       begin base_out = 2'b00; invalid_base = 1'b1; end // unknown
    endcase
  end

endmodule

`default_nettype wire
