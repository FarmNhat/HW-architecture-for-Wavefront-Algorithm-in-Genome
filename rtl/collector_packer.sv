// =============================================================================
// collector_packer.sv
// WFA Accelerator — Collector Packer (per Aligner)
// =============================================================================
// Tích lũy 8 kết quả × 16 byte/kết quả = 128 byte = 1 từ, rồi đẩy vào FIFO.
// Khi aligner_i gửi result_valid, packer nhận 16-byte result_word và đếm.
// Sau 8 kết quả, đánh dấu packer_word_valid = 1 cho Scheduler.
//
// Trong thiết kế đơn giản này: dùng FIFO 1-entry sâu (single buffer).
// Nếu buffer đầy khi kết quả mới đến → backpressure (pause aligner).
// =============================================================================

//
`include "pkg_wfa_params.sv"

module collector_packer
  import wfa_pkg::*;
(
  input  logic         clk,
  input  logic         rst_n,

  // From Aligner
  input  logic [RESULT_WIDTH-1:0] result_in,   // 128-bit result word
  input  logic                    result_valid, // Pulse: result ready

  // To Scheduler: packed 128-byte word (8 results)
  // NOTE: spec says 128 bytes; for implementation we pack 8 × 128-bit results
  // = 1024-bit output word. Scheduler selects one result at a time from the pool.
  // Simplified: we output 8 stored results as a flat array, with a valid flag.
  output logic [RESULT_WIDTH-1:0] pack_results [0:7], // 8 results buffered
  output logic                    pack_valid,          // 8 results ready
  input  logic                    pack_ack             // Scheduler acknowledged
);

  // ---------------------------------------------------------------------------
  // Buffer: 8 result slots
  // ---------------------------------------------------------------------------
  logic [RESULT_WIDTH-1:0] buf_r [0:7];
  logic [2:0]              cnt_r;     // 0..8 results filled
  logic                    full_r;    // 1 = 8 results ready

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      cnt_r      <= '0;
      full_r     <= 1'b0;
      pack_valid <= 1'b0;
      for (int i = 0; i < 8; i++) begin
        buf_r[i]       <= '0;
        pack_results[i] <= '0;
      end
    end else begin
      // Acknowledge clears buffer
      if (pack_ack && pack_valid) begin
        cnt_r      <= '0;
        full_r     <= 1'b0;
        pack_valid <= 1'b0;
      end

      // Receive new result (if not full/being cleared)
      if (result_valid && !full_r) begin
        buf_r[cnt_r] <= result_in;
        if (cnt_r == 3'd7) begin
          // 8th result received → latch all to output, set valid
          pack_valid <= 1'b1;
          full_r     <= 1'b1;
          for (int i = 0; i < 8; i++)
            pack_results[i] <= (i < 7) ? buf_r[i] : result_in;
        end else begin
          cnt_r <= cnt_r + 3'd1;
        end
      end
    end
  end

endmodule

`default_nettype wire
