// =============================================================================
// collector_scheduler.sv
// WFA Accelerator — Collector Round-Robin Scheduler
// =============================================================================
// Trọng tài round-robin giữa N Packer, mỗi cycle chọn 1 kết quả hợp lệ
// từ packer có dữ liệu sẵn, xuất ra data_out.
//
// Mỗi cycle, nếu packer[rr_ptr] có valid → xuất result[rr_idx], advance rr_idx.
// Nếu packer[rr_ptr] hết kết quả hoặc không valid → xoay sang packer tiếp theo.
// =============================================================================

//
`include "pkg_wfa_params.sv"

module collector_scheduler
  import wfa_pkg::*;
#(
  parameter int NUM_ALIGNERS_P = NUM_ALIGNERS
) (
  input  logic         clk,
  input  logic         rst_n,

  // From Packers
  input  logic [RESULT_WIDTH-1:0] packer_results [0:NUM_ALIGNERS_P-1][0:7],
  input  logic [NUM_ALIGNERS_P-1:0] packer_valid,   // 1 = packer[i] has 8 results ready
  output logic [NUM_ALIGNERS_P-1:0] packer_ack,     // 1 = packer[i] batch consumed

  // Output
  output logic [RESULT_WIDTH-1:0] data_out,
  output logic                    data_out_valid
);

  // ---------------------------------------------------------------------------
  // Round-robin pointer
  // ---------------------------------------------------------------------------
  logic [$clog2(NUM_ALIGNERS_P)-1:0] rr_ptr;   // which packer
  logic [2:0]                         res_idx;  // which result within packer (0..7)

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rr_ptr        <= '0;
      res_idx       <= '0;
      data_out      <= '0;
      data_out_valid <= 1'b0;
      packer_ack    <= '0;
    end else begin
      data_out_valid <= 1'b0;
      packer_ack     <= '0;

      // Scan for a valid packer starting from rr_ptr
      begin
        logic [$clog2(NUM_ALIGNERS_P)-1:0] scan_ptr;
        logic found;
        scan_ptr = rr_ptr;
        found    = 1'b0;

        for (int i = 0; i < NUM_ALIGNERS_P; i++) begin
          if (!found) begin
            if (packer_valid[scan_ptr]) begin
              // Serve this packer
              data_out       <= packer_results[scan_ptr][res_idx];
              data_out_valid <= 1'b1;
              found          <= 1'b1;

              if (res_idx == 3'd7) begin
                // Done with this batch
                packer_ack[scan_ptr] <= 1'b1;
                res_idx              <= '0;
                // Advance to next packer
                rr_ptr <= (scan_ptr == NUM_ALIGNERS_P - 1) ? '0 : scan_ptr + 1'b1;
              end else begin
                res_idx <= res_idx + 3'd1;
              end
            end else begin
              // Try next packer
              scan_ptr = (scan_ptr == NUM_ALIGNERS_P - 1) ? '0 : scan_ptr + 1'b1;
            end
          end
        end
      end
    end
  end

endmodule

`default_nettype wire
