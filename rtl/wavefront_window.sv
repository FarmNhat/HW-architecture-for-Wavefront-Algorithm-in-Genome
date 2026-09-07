// =============================================================================
// wavefront_window.sv
// WFA Accelerator — Wavefront Window (Circular Buffer Shift-Register-File)
// =============================================================================
// Lưu trữ cửa sổ trượt các cột wavefront cho 3 matrix M, I, D.
//
// Cấu trúc:
//   - 3 circular buffer độc lập cho M, I, D
//   - Mỗi buffer: depth = WIN_DEPTH_x cột, mỗi cột = NUM_DIAGS entries
//   - Mỗi entry: wf_entry_t = {valid:1, offset:OFF_WIDTH}
//
// Ghi/Đọc:
//   - Ghi: write_col(matrix, data[NUM_DIAGS]) → ghi vào wr_ptr, advance ptr
//   - Đọc: read_entry(matrix, lag, k) → trả về entry cách lag cột về trước
//     lag=0 là cột vừa ghi, lag=1 là cột trước đó, ...
//
// Địa chỉ cột trong circular buffer:
//   read_ptr = (wr_ptr - lag - 1 + depth) mod depth
//     (wr_ptr trỏ vào slot tiếp theo để ghi)
//
// K range: k từ -K_MAX đến +K_MAX → index = k + K_MAX (0 đến 2*K_MAX)
// =============================================================================

`default_nettype none
`include "pkg_wfa_params.sv"

module wavefront_window
  import wfa_pkg::*;
#(
  parameter int K_MAX_P      = K_MAX,
  parameter int WIN_DEPTH_M_P = WIN_DEPTH_M,  // depth for M buffer
  parameter int WIN_DEPTH_I_P = WIN_DEPTH_I,  // depth for I buffer
  parameter int WIN_DEPTH_D_P = WIN_DEPTH_D,  // depth for D buffer
  parameter int OFF_WIDTH_P  = OFF_WIDTH,
  parameter int K_WIDTH_P    = K_WIDTH
) (
  input  logic                        clk,
  input  logic                        rst_n,

  // -------------------------------------------------------------------------
  // Write interface
  // Ghi 1 cột đầy đủ (NUM_DIAGS entries) vào buffer
  // -------------------------------------------------------------------------
  // Cột M
  input  logic                        wr_en_M,    // Write enable for M buffer
  input  wf_entry_t [2*K_MAX_P:0]    wr_col_M,   // Column data to write (all k)

  // Cột I
  input  logic                        wr_en_I,
  input  wf_entry_t [2*K_MAX_P:0]    wr_col_I,

  // Cột D
  input  logic                        wr_en_D,
  input  wf_entry_t [2*K_MAX_P:0]    wr_col_D,

  // -------------------------------------------------------------------------
  // Read interface (combinational)
  // Đọc entry tại (lag, k) — lag=0 là cột cuối cùng được ghi
  // -------------------------------------------------------------------------
  // Đọc từ M buffer
  input  logic [$clog2(WIN_DEPTH_M_P)-1:0] rd_lag_M,  // 0..WIN_DEPTH_M-1
  input  logic signed [K_WIDTH_P-1:0]      rd_k_M,    // diagonal index
  output wf_entry_t                         rd_entry_M, // result

  // Đọc từ I buffer
  input  logic [$clog2(WIN_DEPTH_I_P)-1:0] rd_lag_I,
  input  logic signed [K_WIDTH_P-1:0]      rd_k_I,
  output wf_entry_t                         rd_entry_I,

  // Đọc từ D buffer
  input  logic [$clog2(WIN_DEPTH_D_P)-1:0] rd_lag_D,
  input  logic signed [K_WIDTH_P-1:0]      rd_k_D,
  output wf_entry_t                         rd_entry_D,

  // -------------------------------------------------------------------------
  // Control: reset pointers when starting new alignment
  // -------------------------------------------------------------------------
  input  logic                        clear   // synchronous clear (all entries → invalid)
);

  // ---------------------------------------------------------------------------
  // Local parameters
  // ---------------------------------------------------------------------------
  localparam int NUM_DIAGS  = 2 * K_MAX_P + 1;
  localparam int ENTRY_W    = 1 + OFF_WIDTH_P; // valid + offset
  localparam int PTR_W_M    = $clog2(WIN_DEPTH_M_P);
  localparam int PTR_W_I    = $clog2(WIN_DEPTH_I_P);
  localparam int PTR_W_D    = $clog2(WIN_DEPTH_D_P);

  // ---------------------------------------------------------------------------
  // M Buffer: WIN_DEPTH_M_P columns × NUM_DIAGS entries
  // ---------------------------------------------------------------------------
  wf_entry_t buf_M [0:WIN_DEPTH_M_P-1][0:NUM_DIAGS-1];
  logic [PTR_W_M-1:0] wr_ptr_M; // next write slot

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n || clear) begin
      wr_ptr_M <= '0;
      for (int col = 0; col < WIN_DEPTH_M_P; col++) begin
        for (int d = 0; d < NUM_DIAGS; d++) begin
          buf_M[col][d].valid  <= 1'b0;
          buf_M[col][d].offset <= '0;
        end
      end
    end else if (wr_en_M) begin
      for (int d = 0; d < NUM_DIAGS; d++)
        buf_M[wr_ptr_M][d] <= wr_col_M[d];
      // Advance write pointer (wrap around)
      if (wr_ptr_M == WIN_DEPTH_M_P - 1)
        wr_ptr_M <= '0;
      else
        wr_ptr_M <= wr_ptr_M + 1'b1;
    end
  end

  // Combinational read for M: lag 0 = last written col
  logic [PTR_W_M-1:0] rd_col_M;
  logic [K_WIDTH_P-1:0] k_idx_M;

  always_comb begin
    // rd_col = (wr_ptr - lag - 1 + depth) mod depth
    rd_col_M = (wr_ptr_M == 0) ?
               (WIN_DEPTH_M_P - 1 - rd_lag_M) :
               (wr_ptr_M - 1 - rd_lag_M + WIN_DEPTH_M_P) % WIN_DEPTH_M_P;
    // k index: k + K_MAX → [0, NUM_DIAGS-1]
    k_idx_M = rd_k_M + K_MAX_P;
    // Bounds check
    if (k_idx_M >= NUM_DIAGS || rd_lag_M >= WIN_DEPTH_M_P) begin
      rd_entry_M.valid  = 1'b0;
      rd_entry_M.offset = '0;
    end else begin
      rd_entry_M = buf_M[rd_col_M][k_idx_M];
    end
  end

  // ---------------------------------------------------------------------------
  // I Buffer: WIN_DEPTH_I_P columns × NUM_DIAGS entries
  // ---------------------------------------------------------------------------
  wf_entry_t buf_I [0:WIN_DEPTH_I_P-1][0:NUM_DIAGS-1];
  logic [PTR_W_I-1:0] wr_ptr_I;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n || clear) begin
      wr_ptr_I <= '0;
      for (int col = 0; col < WIN_DEPTH_I_P; col++) begin
        for (int d = 0; d < NUM_DIAGS; d++) begin
          buf_I[col][d].valid  <= 1'b0;
          buf_I[col][d].offset <= '0;
        end
      end
    end else if (wr_en_I) begin
      for (int d = 0; d < NUM_DIAGS; d++)
        buf_I[wr_ptr_I][d] <= wr_col_I[d];
      if (wr_ptr_I == WIN_DEPTH_I_P - 1)
        wr_ptr_I <= '0;
      else
        wr_ptr_I <= wr_ptr_I + 1'b1;
    end
  end

  logic [PTR_W_I-1:0] rd_col_I;
  logic [K_WIDTH_P-1:0] k_idx_I;

  always_comb begin
    rd_col_I = (wr_ptr_I == 0) ?
               (WIN_DEPTH_I_P - 1 - rd_lag_I) :
               (wr_ptr_I - 1 - rd_lag_I + WIN_DEPTH_I_P) % WIN_DEPTH_I_P;
    k_idx_I = rd_k_I + K_MAX_P;
    if (k_idx_I >= NUM_DIAGS || rd_lag_I >= WIN_DEPTH_I_P) begin
      rd_entry_I.valid  = 1'b0;
      rd_entry_I.offset = '0;
    end else begin
      rd_entry_I = buf_I[rd_col_I][k_idx_I];
    end
  end

  // ---------------------------------------------------------------------------
  // D Buffer: WIN_DEPTH_D_P columns × NUM_DIAGS entries
  // ---------------------------------------------------------------------------
  wf_entry_t buf_D [0:WIN_DEPTH_D_P-1][0:NUM_DIAGS-1];
  logic [PTR_W_D-1:0] wr_ptr_D;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n || clear) begin
      wr_ptr_D <= '0;
      for (int col = 0; col < WIN_DEPTH_D_P; col++) begin
        for (int d = 0; d < NUM_DIAGS; d++) begin
          buf_D[col][d].valid  <= 1'b0;
          buf_D[col][d].offset <= '0;
        end
      end
    end else if (wr_en_D) begin
      for (int d = 0; d < NUM_DIAGS; d++)
        buf_D[wr_ptr_D][d] <= wr_col_D[d];
      if (wr_ptr_D == WIN_DEPTH_D_P - 1)
        wr_ptr_D <= '0;
      else
        wr_ptr_D <= wr_ptr_D + 1'b1;
    end
  end

  logic [PTR_W_D-1:0] rd_col_D;
  logic [K_WIDTH_P-1:0] k_idx_D;

  always_comb begin
    rd_col_D = (wr_ptr_D == 0) ?
               (WIN_DEPTH_D_P - 1 - rd_lag_D) :
               (wr_ptr_D - 1 - rd_lag_D + WIN_DEPTH_D_P) % WIN_DEPTH_D_P;
    k_idx_D = rd_k_D + K_MAX_P;
    if (k_idx_D >= NUM_DIAGS || rd_lag_D >= WIN_DEPTH_D_P) begin
      rd_entry_D.valid  = 1'b0;
      rd_entry_D.offset = '0;
    end else begin
      rd_entry_D = buf_D[rd_col_D][k_idx_D];
    end
  end

endmodule

`default_nettype wire
