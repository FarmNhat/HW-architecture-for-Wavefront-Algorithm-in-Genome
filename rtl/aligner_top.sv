// =============================================================================
// aligner_top.sv
// WFA Accelerator — Full Aligner (1 bản sao)
// =============================================================================
// Ghép Extend + Compute + Wavefront Window + Backtrace thành 1 Aligner hoàn chỉnh.
//
// FSM điều phối:
//   IDLE       → chờ start từ Extractor
//   LOAD_JOB   → latch job inputs
//   INIT_S0    → khởi tạo s=0: M[0][k=0] = extend(0,0)
//   COMPUTE    → tính I/D/M cho tất cả k trong [-K_MAX, +K_MAX]
//               (dùng P_SUBMODULES=8 instances, nhiều cycles nếu 2*K_MAX+1 > 8)
//   EXTEND_K   → gọi extend cho từng k vừa tính (tuần tự)
//   WR_WINDOW  → ghi cột mới vào Wavefront Window
//   WR_RAM     → ghi origin vào Backtrace RAM
//   CHECK_DONE → kiểm tra điều kiện dừng (M[s][k_target] >= n1 và v = n2)
//   ADVANCE_S  → tăng s, lặp lại COMPUTE
//   DONE_ALIGN → tìm thấy, ghi score
//   BACKTRACE  → gọi BT controller
//   OUTPUT     → xuất result_word, trở về IDLE
//   CPU_FALLBACK → score > K_MAX (không tìm thấy trong giới hạn), set flag
//
// P_SUBMODULES = 8 instances compute_submodule chạy song song.
// Mỗi cycle của COMPUTE xử lý 8 đường chéo liên tiếp.
// Tổng số cycles COMPUTE = ceil((2*K_MAX+1) / 8) per score step.
// =============================================================================

`default_nettype none
`include "pkg_wfa_params.sv"

module aligner_top
  import wfa_pkg::*;
#(
  parameter int K_MAX_P       = K_MAX,
  parameter int SEQ_LEN_MAX_P = SEQ_LEN_MAX,
  parameter int X_PEN_P       = X_PEN,
  parameter int O_PEN_P       = O_PEN,
  parameter int E_PEN_P       = E_PEN,
  parameter int P_SUB_P       = P_SUBMODULES,  // must be 8
  parameter int MAX_GROUPS_P  = MAX_GROUPS,
  parameter int ID_WIDTH_P    = ID_WIDTH,
  parameter int LEN_WIDTH_P   = LEN_WIDTH,
  parameter int OFF_WIDTH_P   = OFF_WIDTH,
  parameter int K_WIDTH_P     = K_WIDTH,
  parameter int S_WIDTH_P     = S_WIDTH
) (
  input  logic                          clk,
  input  logic                          rst_n,

  // -------------------------------------------------------------------------
  // Job input (from Extractor)
  // -------------------------------------------------------------------------
  input  logic                          start,          // Pulse: new job assigned
  input  logic [ID_WIDTH_P-1:0]         job_id,
  input  logic [LEN_WIDTH_P-1:0]        job_len1,
  input  logic [LEN_WIDTH_P-1:0]        job_len2,
  input  logic [15:0]  job_seq1_groups [0:MAX_GROUPS_P-1],
  input  logic [15:0]  job_seq2_groups [0:MAX_GROUPS_P-1],

  // -------------------------------------------------------------------------
  // Status (to Extractor)
  // -------------------------------------------------------------------------
  output logic                          status_idle,    // 1 = ready for new job

  // -------------------------------------------------------------------------
  // Result (to Collector)
  // -------------------------------------------------------------------------
  output logic [RESULT_WIDTH-1:0]       result_word,    // 128-bit: id+score+cigar+pad
  output logic                          result_valid,   // Pulse: result ready
  output logic                          needs_cpu_fallback // 1 = score exceeds K_MAX
);

  // ---------------------------------------------------------------------------
  // Local parameters
  // ---------------------------------------------------------------------------
  localparam int NUM_DIAGS   = 2 * K_MAX_P + 1;
  localparam int COMPUTE_CYC = (NUM_DIAGS + P_SUB_P - 1) / P_SUB_P; // cycles per score step
  localparam int ADDR_W      = $clog2(BT_RAM_DEPTH);
  localparam int WIN_M_LOG   = $clog2(WIN_DEPTH_M);
  localparam int WIN_I_LOG   = $clog2(WIN_DEPTH_I);
  localparam int WIN_D_LOG   = $clog2(WIN_DEPTH_D);

  // ---------------------------------------------------------------------------
  // FSM states
  // ---------------------------------------------------------------------------
  typedef enum logic [3:0] {
    A_IDLE        = 4'd0,
    A_LOAD        = 4'd1,
    A_INIT_S0     = 4'd2,  // s=0: extend at k=0, h=0
    A_INIT_WAIT   = 4'd3,  // wait for extend done
    A_COMPUTE     = 4'd4,  // compute I/D/M for a group of 8 diagonals
    A_EXTEND_K    = 4'd5,  // call extend for each k with valid m_pre
    A_EXTEND_WAIT = 4'd6,  // wait for extend done
    A_WR_WINDOW   = 4'd7,  // write new column to window
    A_WR_RAM      = 4'd8,  // write origins to BT RAM
    A_CHECK       = 4'd9,  // check termination
    A_ADVANCE     = 4'd10, // increment s
    A_DONE_ALIGN  = 4'd11, // alignment complete
    A_BACKTRACE   = 4'd12, // run backtrace
    A_OUTPUT      = 4'd13, // output result
    A_CPU_FALLBACK = 4'd14  // needs CPU
  } aligner_state_t;

  aligner_state_t state_r;

  // ---------------------------------------------------------------------------
  // Job registers
  // ---------------------------------------------------------------------------
  logic [ID_WIDTH_P-1:0]     r_job_id;
  logic [LEN_WIDTH_P-1:0]    r_len1, r_len2;
  logic [15:0]               r_seq1 [0:MAX_GROUPS_P-1];
  logic [15:0]               r_seq2 [0:MAX_GROUPS_P-1];
  logic signed [K_WIDTH_P-1:0] r_k_target;  // n1 - n2

  // ---------------------------------------------------------------------------
  // Score counter
  // ---------------------------------------------------------------------------
  logic [S_WIDTH_P-1:0] s_r;       // current score step
  logic [S_WIDTH_P-1:0] score_opt; // final score at termination

  // ---------------------------------------------------------------------------
  // Diagonal sweep counters (for COMPUTE phase)
  // ---------------------------------------------------------------------------
  // k_base: starting diagonal for current 8-group compute cycle
  // k values processed: k_base, k_base+1, ..., k_base+7 (clamped to ±K_MAX)
  logic signed [K_WIDTH_P-1:0] k_base_r; // current group's starting k
  logic [$clog2(COMPUTE_CYC+1)-1:0] compute_cyc_r; // which compute group cycle

  // ---------------------------------------------------------------------------
  // Wavefront columns (current frame, one entry per diagonal)
  // Built up during COMPUTE and EXTEND_K phases, written to window in WR_WINDOW
  // ---------------------------------------------------------------------------
  wf_entry_t new_col_M [0:NUM_DIAGS-1]; // M values for new column
  wf_entry_t new_col_I [0:NUM_DIAGS-1]; // I values
  wf_entry_t new_col_D [0:NUM_DIAGS-1]; // D values
  origin_t   new_origins [0:NUM_DIAGS-1]; // 5-bit origins

  // Keep m_pre for each diagonal (before extend), used in EXTEND_K phase
  wf_entry_t m_pre_col [0:NUM_DIAGS-1]; // M pre-extend

  // ---------------------------------------------------------------------------
  // Extend submodule (single instance, time-multiplexed across diagonals)
  // ---------------------------------------------------------------------------
  logic                         ext_start;
  logic                         ext_done;
  logic signed [K_WIDTH_P-1:0]  ext_k_in;
  logic [OFF_WIDTH_P-1:0]       ext_offset_in;
  logic [OFF_WIDTH_P-1:0]       ext_offset_out;

  extend_submodule #(
    .MAX_GROUPS_P (MAX_GROUPS_P),
    .OFF_WIDTH_P  (OFF_WIDTH_P),
    .K_WIDTH_P    (K_WIDTH_P),
    .LEN_WIDTH_P  (LEN_WIDTH_P)
  ) u_extend (
    .clk         (clk),
    .rst_n       (rst_n),
    .start       (ext_start),
    .done        (ext_done),
    .k_in        (ext_k_in),
    .offset_in   (ext_offset_in),
    .seq_len1    (r_len1),
    .seq_len2    (r_len2),
    .seq1_groups (r_seq1),
    .seq2_groups (r_seq2),
    .offset_out  (ext_offset_out)
  );

  // ---------------------------------------------------------------------------
  // P_SUBMODULES=8 compute_submodule instances (pure combinational)
  // ---------------------------------------------------------------------------
  // Inputs read from window for each of 8 diagonals
  wf_entry_t cmp_from_M_oe_km1 [0:P_SUB_P-1];
  wf_entry_t cmp_from_I_e_km1  [0:P_SUB_P-1];
  wf_entry_t cmp_from_M_oe_kp1 [0:P_SUB_P-1];
  wf_entry_t cmp_from_D_e_kp1  [0:P_SUB_P-1];
  wf_entry_t cmp_from_M_x_k    [0:P_SUB_P-1];

  wf_entry_t cmp_i_val   [0:P_SUB_P-1];
  wf_entry_t cmp_d_val   [0:P_SUB_P-1];
  wf_entry_t cmp_m_pre   [0:P_SUB_P-1];
  origin_t   cmp_origin  [0:P_SUB_P-1];
  logic      cmp_null    [0:P_SUB_P-1];

  genvar gi;
  generate
    for (gi = 0; gi < P_SUB_P; gi++) begin : gen_compute
      compute_submodule #(
        .OFF_WIDTH_P(OFF_WIDTH_P),
        .K_WIDTH_P  (K_WIDTH_P)
      ) u_compute (
        .from_M_oe_km1 (cmp_from_M_oe_km1[gi]),
        .from_I_e_km1  (cmp_from_I_e_km1[gi]),
        .from_M_oe_kp1 (cmp_from_M_oe_kp1[gi]),
        .from_D_e_kp1  (cmp_from_D_e_kp1[gi]),
        .from_M_x_k    (cmp_from_M_x_k[gi]),
        .i_val         (cmp_i_val[gi]),
        .d_val         (cmp_d_val[gi]),
        .m_pre         (cmp_m_pre[gi]),
        .origin_out    (cmp_origin[gi]),
        .null_out      (cmp_null[gi])
      );
    end
  endgenerate

  // ---------------------------------------------------------------------------
  // Wavefront Window
  // ---------------------------------------------------------------------------


  // Read ports: 5 reads needed per compute instance (M_oe_km1, I_e_km1, M_oe_kp1, D_e_kp1, M_x_k)
  // We drive them combinationally from k_base_r and gi offset

  // For simplicity, the window module has 3 read ports (one per matrix).
  // In COMPUTE phase, we read 8 diagonals simultaneously.
  // The compute submodules are combinational → we feed them directly.
  // We must read 5 different (lag, k) combinations per diagonal:
  //   from_M_oe_km1: M, lag=O_PEN+E_PEN, k = k_base+gi - 1
  //   from_I_e_km1:  I, lag=E_PEN,       k = k_base+gi - 1
  //   from_M_oe_kp1: M, lag=O_PEN+E_PEN, k = k_base+gi + 1
  //   from_D_e_kp1:  D, lag=E_PEN,       k = k_base+gi + 1
  //   from_M_x_k:    M, lag=X_PEN,       k = k_base+gi
  //
  // The window module has 1 read port per matrix. We cannot read 8 k-values
  // simultaneously from a single port. Two options:
  //   A) Duplicate window (5 read ports) → large area
  //   B) Time-multiplex: compute 1 diagonal/cycle (8 cycles per P_SUB group)
  //      but then P_SUBMODULES gives no speedup for compute.
  //   C) Use synchronous registers: replicate the window data as a "flat" register
  //      array and read combinationally → correct for simulation.
  //
  // For correctness-first implementation (as requested), we use option C:
  // Keep a FLAT REGISTER COPY of the window for combinational read.
  // The formal window (wavefront_window.sv) is also kept and written,
  // but combinational reads are from the flat copy.
  //
  // Flat copies of window (3 arrays × WIN_DEPTH × NUM_DIAGS):
  // ---------------------------------------------------------------------------
  wf_entry_t flat_M [0:WIN_DEPTH_M-1][0:NUM_DIAGS-1];
  wf_entry_t flat_I [0:WIN_DEPTH_I-1][0:NUM_DIAGS-1];
  wf_entry_t flat_D [0:WIN_DEPTH_D-1][0:NUM_DIAGS-1];

  // Write pointers for flat arrays (same logic as wavefront_window)
  logic [$clog2(WIN_DEPTH_M)-1:0] flat_wr_M;
  logic [$clog2(WIN_DEPTH_I)-1:0] flat_wr_I;
  logic [$clog2(WIN_DEPTH_D)-1:0] flat_wr_D;

  // Function-like task to compute read column for flat array
  // read_col = (wr_ptr - lag - 1 + depth) % depth
  function automatic logic [$clog2(WIN_DEPTH_M)-1:0] flat_rd_col_M(
    input logic [$clog2(WIN_DEPTH_M)-1:0] wr_ptr,
    input int lag
  );
    int tmp;
    tmp = int'(wr_ptr) - lag - 1;
    if (tmp < 0) tmp = tmp + WIN_DEPTH_M;
    return $clog2(WIN_DEPTH_M)'(tmp % WIN_DEPTH_M);
  endfunction

  function automatic logic [$clog2(WIN_DEPTH_I)-1:0] flat_rd_col_I(
    input logic [$clog2(WIN_DEPTH_I)-1:0] wr_ptr,
    input int lag
  );
    int tmp;
    tmp = int'(wr_ptr) - lag - 1;
    if (tmp < 0) tmp = tmp + WIN_DEPTH_I;
    return $clog2(WIN_DEPTH_I)'(tmp % WIN_DEPTH_I);
  endfunction

  function automatic logic [$clog2(WIN_DEPTH_D)-1:0] flat_rd_col_D(
    input logic [$clog2(WIN_DEPTH_D)-1:0] wr_ptr,
    input int lag
  );
    int tmp;
    tmp = int'(wr_ptr) - lag - 1;
    if (tmp < 0) tmp = tmp + WIN_DEPTH_D;
    return $clog2(WIN_DEPTH_D)'(tmp % WIN_DEPTH_D);
  endfunction

  // Helper: get wf_entry from flat_M with bounds check
  function automatic wf_entry_t get_M(
    input logic [$clog2(WIN_DEPTH_M)-1:0] wr_ptr,
    input int lag,
    input int k
  );
    int kidx;
    kidx = k + K_MAX_P;
    if (lag >= WIN_DEPTH_M || kidx < 0 || kidx >= NUM_DIAGS ||
        s_r < S_WIDTH_P'(lag + 1)) begin
      get_M.valid  = 1'b0;
      get_M.offset = '0;
    end else begin
      get_M = flat_M[flat_rd_col_M(wr_ptr, lag[$clog2(WIN_DEPTH_M)-1:0])][kidx];
    end
  endfunction

  function automatic wf_entry_t get_I(
    input logic [$clog2(WIN_DEPTH_I)-1:0] wr_ptr,
    input int lag,
    input int k
  );
    int kidx;
    kidx = k + K_MAX_P;
    if (lag >= WIN_DEPTH_I || kidx < 0 || kidx >= NUM_DIAGS ||
        s_r < S_WIDTH_P'(lag + 1)) begin
      get_I.valid  = 1'b0;
      get_I.offset = '0;
    end else begin
      get_I = flat_I[flat_rd_col_I(wr_ptr, lag[$clog2(WIN_DEPTH_I)-1:0])][kidx];
    end
  endfunction

  function automatic wf_entry_t get_D(
    input logic [$clog2(WIN_DEPTH_D)-1:0] wr_ptr,
    input int lag,
    input int k
  );
    int kidx;
    kidx = k + K_MAX_P;
    if (lag >= WIN_DEPTH_D || kidx < 0 || kidx >= NUM_DIAGS ||
        s_r < S_WIDTH_P'(lag + 1)) begin
      get_D.valid  = 1'b0;
      get_D.offset = '0;
    end else begin
      get_D = flat_D[flat_rd_col_D(wr_ptr, lag[$clog2(WIN_DEPTH_D)-1:0])][kidx];
    end
  endfunction

  // ---------------------------------------------------------------------------
  // Combinational: drive compute submodule inputs from flat window
  // ---------------------------------------------------------------------------
  always_comb begin
    for (int gi2 = 0; gi2 < P_SUB_P; gi2++) begin
      int k_cur;
      k_cur = int'(signed'(k_base_r)) + gi2;

      // Boundary: clamp k to valid range
      if (k_cur < -K_MAX_P || k_cur > K_MAX_P) begin
        cmp_from_M_oe_km1[gi2].valid  = 1'b0;
        cmp_from_M_oe_km1[gi2].offset = '0;
        cmp_from_I_e_km1[gi2].valid   = 1'b0;
        cmp_from_I_e_km1[gi2].offset  = '0;
        cmp_from_M_oe_kp1[gi2].valid  = 1'b0;
        cmp_from_M_oe_kp1[gi2].offset = '0;
        cmp_from_D_e_kp1[gi2].valid   = 1'b0;
        cmp_from_D_e_kp1[gi2].offset  = '0;
        cmp_from_M_x_k[gi2].valid     = 1'b0;
        cmp_from_M_x_k[gi2].offset    = '0;
      end else begin
        // For I: need M[s-(o+e), k-1] and I[s-e, k-1]
        cmp_from_M_oe_km1[gi2] = get_M(flat_wr_M, O_PEN_P + E_PEN_P, k_cur - 1);
        cmp_from_I_e_km1[gi2]  = get_I(flat_wr_I, E_PEN_P,           k_cur - 1);
        // For D: need M[s-(o+e), k+1] and D[s-e, k+1]
        cmp_from_M_oe_kp1[gi2] = get_M(flat_wr_M, O_PEN_P + E_PEN_P, k_cur + 1);
        cmp_from_D_e_kp1[gi2]  = get_D(flat_wr_D, E_PEN_P,           k_cur + 1);
        // For M: need M[s-x, k]
        cmp_from_M_x_k[gi2]    = get_M(flat_wr_M, X_PEN_P,           k_cur);
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Extend control (time-multiplexed during EXTEND_K phase)
  // ---------------------------------------------------------------------------
  logic signed [K_WIDTH_P-1:0] ext_k_cursor; // which k we're currently extending
  logic [$clog2(NUM_DIAGS+1)-1:0] ext_k_done_cnt; // how many extends completed

  // ---------------------------------------------------------------------------
  // BT RAM write interface
  // ---------------------------------------------------------------------------
  logic [ADDR_W-1:0]      bt_wr_addr_r;
  logic [BT_RAM_WIDTH-1:0] bt_wr_data;
  logic                   bt_wr_en;

  // Assemble BT RAM write data from new_origins (pack 8 5-bit cells)
  // For the current 8-diagonal group being written
  logic [ADDR_W-1:0]      bt_wr_group_addr; // address of the group being written
  logic [2:0]              bt_wr_group_idx;  // which group (0..ceil(NUM_DIAGS/8)-1)
  logic [$clog2((NUM_DIAGS+7)/8+1)-1:0] bt_wr_total_groups;

  // Collect all origins into BT word (we write 1 word per 8 diagonals per score)
  // Total words per score step = ceil(NUM_DIAGS / 8)
  // Address = s * ceil(NUM_DIAGS/8) + group_idx

  // BT RAM instance
  logic [ADDR_W-1:0]       bt_rd_addr;
  logic                    bt_rd_en;
  logic [BT_RAM_WIDTH-1:0] bt_rd_data;

  backtrace_ram #(
    .RAM_WIDTH_P(BT_RAM_WIDTH),
    .RAM_DEPTH_P(BT_RAM_DEPTH)
  ) u_bt_ram (
    .clk     (clk),
    .wr_en   (bt_wr_en),
    .wr_addr (bt_wr_addr_r),
    .wr_data (bt_wr_data),
    .rd_en   (bt_rd_en),
    .rd_addr (bt_rd_addr),
    .rd_data (bt_rd_data)
  );

  // ---------------------------------------------------------------------------
  // Backtrace Controller
  // ---------------------------------------------------------------------------
  logic        bt_start, bt_done;
  logic [63:0] bt_cigar;

  backtrace_ctrl #(
    .S_WIDTH_P (S_WIDTH_P),
    .K_WIDTH_P (K_WIDTH_P),
    .K_MAX_P   (K_MAX_P),
    .X_PEN_P   (X_PEN_P),
    .O_PEN_P   (O_PEN_P),
    .E_PEN_P   (E_PEN_P)
  ) u_bt_ctrl (
    .clk          (clk),
    .rst_n        (rst_n),
    .start        (bt_start),
    .done         (bt_done),
    .score_opt    (score_opt),
    .k_target     (r_k_target),
    .ram_rd_en    (bt_rd_en),
    .ram_rd_addr  (bt_rd_addr),
    .ram_rd_data  (bt_rd_data),
    .compact_cigar(bt_cigar)
  );

  // ---------------------------------------------------------------------------
  // BT RAM write data assembly
  // Pack 8 consecutive origin cells into 1 40-bit word.
  // Group g covers diagonals: k = -K_MAX + g*8 .. -K_MAX + g*8 + 7
  // k_index = k + K_MAX → diag_index in new_origins[]
  // ---------------------------------------------------------------------------
  always_comb begin
    bt_wr_data = '0;
    // Pack the 8 cells corresponding to the current write group
    for (int ci = 0; ci < 8; ci++) begin
      int diag_idx;
      diag_idx = int'(bt_wr_group_idx) * 8 + ci;
      if (diag_idx < NUM_DIAGS)
        bt_wr_data[ci*5 +: 5] = new_origins[diag_idx];
      else
        bt_wr_data[ci*5 +: 5] = '0;
    end
  end

  // ---------------------------------------------------------------------------
  // MAIN FSM
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state_r           <= A_IDLE;
      status_idle       <= 1'b1;
      result_valid      <= 1'b0;
      result_word       <= '0;
      needs_cpu_fallback <= 1'b0;
      s_r               <= '0;
      score_opt         <= '0;
      k_base_r          <= -K_WIDTH_P'(K_MAX_P);
      compute_cyc_r     <= '0;
      ext_start         <= 1'b0;
      ext_k_in          <= '0;
      ext_offset_in     <= '0;
      ext_k_cursor      <= -K_WIDTH_P'(K_MAX_P);
      ext_k_done_cnt    <= '0;
      bt_start          <= 1'b0;
      bt_wr_en          <= 1'b0;
      bt_wr_addr_r      <= '0;
      bt_wr_group_idx   <= '0;
      flat_wr_M         <= '0;
      flat_wr_I         <= '0;
      flat_wr_D         <= '0;

      // Clear flat window
      for (int col = 0; col < WIN_DEPTH_M; col++)
        for (int d = 0; d < NUM_DIAGS; d++) begin
          flat_M[col][d].valid  <= 1'b0;
          flat_M[col][d].offset <= '0;
        end
      for (int col = 0; col < WIN_DEPTH_I; col++)
        for (int d = 0; d < NUM_DIAGS; d++) begin
          flat_I[col][d].valid  <= 1'b0;
          flat_I[col][d].offset <= '0;
        end
      for (int col = 0; col < WIN_DEPTH_D; col++)
        for (int d = 0; d < NUM_DIAGS; d++) begin
          flat_D[col][d].valid  <= 1'b0;
          flat_D[col][d].offset <= '0;
        end

      // Clear new column buffers
      for (int d = 0; d < NUM_DIAGS; d++) begin
        new_col_M[d].valid   <= 1'b0;
        new_col_M[d].offset  <= '0;
        new_col_I[d].valid   <= 1'b0;
        new_col_I[d].offset  <= '0;
        new_col_D[d].valid   <= 1'b0;
        new_col_D[d].offset  <= '0;
        m_pre_col[d].valid   <= 1'b0;
        m_pre_col[d].offset  <= '0;
        new_origins[d]       <= '0;
      end
    end else begin
      // Default: deassert pulses
      ext_start    <= 1'b0;
      bt_start     <= 1'b0;
      bt_wr_en     <= 1'b0;
      result_valid <= 1'b0;

      case (state_r)
        // ===================================================================
        A_IDLE: begin
          status_idle       <= 1'b1;
          needs_cpu_fallback <= 1'b0;
          if (start) begin
            status_idle <= 1'b0;
            state_r     <= A_LOAD;
          end
        end

        // ===================================================================
        A_LOAD: begin
          // Latch job inputs
          r_job_id   <= job_id;
          r_len1     <= job_len1;
          r_len2     <= job_len2;
          for (int i = 0; i < MAX_GROUPS_P; i++) begin
            r_seq1[i] <= job_seq1_groups[i];
            r_seq2[i] <= job_seq2_groups[i];
          end
          r_k_target <= K_WIDTH_P'(signed'(int'(job_len1) - int'(job_len2)));

          // Reset score and window pointers
          s_r        <= '0;
          flat_wr_M  <= '0;
          flat_wr_I  <= '0;
          flat_wr_D  <= '0;
          bt_wr_addr_r <= '0;

          // Clear all new column buffers
          for (int d = 0; d < NUM_DIAGS; d++) begin
            new_col_M[d].valid   <= 1'b0;
            new_col_M[d].offset  <= '0;
            new_col_I[d].valid   <= 1'b0;
            new_col_I[d].offset  <= '0;
            new_col_D[d].valid   <= 1'b0;
            new_col_D[d].offset  <= '0;
            m_pre_col[d].valid   <= 1'b0;
            m_pre_col[d].offset  <= '0;
            new_origins[d]       <= '0;
          end

          // Clear flat window
          for (int col = 0; col < WIN_DEPTH_M; col++)
            for (int d = 0; d < NUM_DIAGS; d++) begin
              flat_M[col][d].valid  <= 1'b0;
              flat_M[col][d].offset <= '0;
            end
          for (int col = 0; col < WIN_DEPTH_I; col++)
            for (int d = 0; d < NUM_DIAGS; d++) begin
              flat_I[col][d].valid  <= 1'b0;
              flat_I[col][d].offset <= '0;
            end
          for (int col = 0; col < WIN_DEPTH_D; col++)
            for (int d = 0; d < NUM_DIAGS; d++) begin
              flat_D[col][d].valid  <= 1'b0;
              flat_D[col][d].offset <= '0;
            end

          state_r <= A_INIT_S0;
        end

        // ===================================================================
        // s=0: Only k=0 is active, M[0][0] = extend(0, 0)
        A_INIT_S0: begin
          ext_start     <= 1'b1;
          ext_k_in      <= '0;        // k=0
          ext_offset_in <= '0;        // h=0
          state_r       <= A_INIT_WAIT;
        end

        // ===================================================================
        A_INIT_WAIT: begin
          if (ext_done) begin
            // Store M[0][k=0] = ext_offset_out
            new_col_M[K_MAX_P].valid  <= 1'b1;
            new_col_M[K_MAX_P].offset <= ext_offset_out; // index K_MAX = k=0
            new_col_I[K_MAX_P].valid  <= 1'b0;
            new_col_I[K_MAX_P].offset <= '0;
            new_col_D[K_MAX_P].valid  <= 1'b0;
            new_col_D[K_MAX_P].offset <= '0;
            new_origins[K_MAX_P]      <= {M_ORIG_MM, 2'b00}; // k=0, s=0: came from nothing (INIT)

            // Write to flat window for M (so s=1 can read M[0][0])
            flat_M[flat_wr_M][K_MAX_P].valid  <= 1'b1;
            flat_M[flat_wr_M][K_MAX_P].offset <= ext_offset_out;

            // Check if s=0 already reaches end
            // M[0][k=0] should equal n1 if seq1 == seq2 (or prefix match)
            if (ext_offset_out >= r_len1 &&
                (ext_offset_out - OFF_WIDTH_P'(0)) >= r_len2) begin
              // Perfect match at s=0
              score_opt <= '0;
              state_r   <= A_DONE_ALIGN;
            end else begin
              // Move to WR_WINDOW to store s=0 column, then advance
              state_r <= A_WR_WINDOW;
            end
          end
        end

        // ===================================================================
        // Write current new_col_M/I/D into flat window (advance write pointer)
        A_WR_WINDOW: begin
          // Write entire column to flat_M at wr_ptr
          for (int d = 0; d < NUM_DIAGS; d++) begin
            flat_M[flat_wr_M][d] <= new_col_M[d];
            flat_I[flat_wr_I][d] <= new_col_I[d];
            flat_D[flat_wr_D][d] <= new_col_D[d];
          end
          // Advance write pointers (circular)
          flat_wr_M <= (flat_wr_M == WIN_DEPTH_M - 1) ? '0 : flat_wr_M + 1'b1;
          flat_wr_I <= (flat_wr_I == WIN_DEPTH_I - 1) ? '0 : flat_wr_I + 1'b1;
          flat_wr_D <= (flat_wr_D == WIN_DEPTH_D - 1) ? '0 : flat_wr_D + 1'b1;

          // Go to WR_RAM to store origins
          bt_wr_group_idx <= '0;
          state_r <= A_WR_RAM;
        end

        // ===================================================================
        // Write origin word(s) to BT RAM
        // One 40-bit word per 8 diagonals
        A_WR_RAM: begin
          // bt_wr_data is driven combinationally from bt_wr_group_idx
          bt_wr_en   <= 1'b1;
          // addr = s * ceil(NUM_DIAGS/8) + group_idx
          bt_wr_addr_r <= ADDR_W'(int'(s_r) * ((NUM_DIAGS + 7) / 8) + int'(bt_wr_group_idx));

          if (bt_wr_group_idx < ((NUM_DIAGS + 7) / 8 - 1)) begin
            bt_wr_group_idx <= bt_wr_group_idx + 1'b1;
            // Stay in A_WR_RAM to write next group
          end else begin
            bt_wr_group_idx <= '0;
            state_r <= A_CHECK;
          end
        end

        // ===================================================================
        // Check termination: does M[s][k_target] reach end of both sequences?
        A_CHECK: begin
          begin
            int kt_idx;
            kt_idx = int'(r_k_target) + K_MAX_P;
            if (kt_idx >= 0 && kt_idx < NUM_DIAGS &&
                new_col_M[kt_idx].valid &&
                new_col_M[kt_idx].offset >= r_len1 &&
                (int'(new_col_M[kt_idx].offset) - int'(r_k_target)) >= int'(r_len2)) begin
              // Found!
              score_opt <= s_r;
              state_r   <= A_DONE_ALIGN;
            end else if (s_r >= S_WIDTH_P'(K_MAX_P * (O_PEN_P + E_PEN_P))) begin
              // Exceeded K_MAX cost → CPU fallback
              state_r <= A_CPU_FALLBACK;
            end else begin
              state_r <= A_ADVANCE;
            end
          end
        end

        // ===================================================================
        A_ADVANCE: begin
          s_r       <= s_r + 1'b1;
          k_base_r  <= -K_WIDTH_P'(K_MAX_P);
          compute_cyc_r <= '0;
          // Clear new column buffers for next score step
          for (int d = 0; d < NUM_DIAGS; d++) begin
            new_col_M[d].valid   <= 1'b0;
            new_col_M[d].offset  <= '0;
            new_col_I[d].valid   <= 1'b0;
            new_col_I[d].offset  <= '0;
            new_col_D[d].valid   <= 1'b0;
            new_col_D[d].offset  <= '0;
            m_pre_col[d].valid   <= 1'b0;
            m_pre_col[d].offset  <= '0;
            new_origins[d]       <= '0;
          end
          state_r <= A_COMPUTE;
        end

        // ===================================================================
        // COMPUTE: use 8 compute_submodule instances to compute 8 diagonals
        // per cycle. Sweep k from -K_MAX to +K_MAX.
        A_COMPUTE: begin
          // compute_submodule outputs are combinational (driven from k_base_r)
          // Latch results for current 8 diagonals
          for (int gi2 = 0; gi2 < P_SUB_P; gi2++) begin
            int k_cur, d_idx;
            k_cur = int'(signed'(k_base_r)) + gi2;
            d_idx = k_cur + K_MAX_P;

            if (k_cur >= -K_MAX_P && k_cur <= K_MAX_P && d_idx < NUM_DIAGS) begin
              if (!cmp_null[gi2]) begin
                new_col_I[d_idx]   <= cmp_i_val[gi2];
                new_col_D[d_idx]   <= cmp_d_val[gi2];
                m_pre_col[d_idx]   <= cmp_m_pre[gi2];
                new_origins[d_idx] <= cmp_origin[gi2];
              end else begin
                // Null: mark as invalid
                new_col_I[d_idx].valid   <= 1'b0;
                new_col_I[d_idx].offset  <= '0;
                new_col_D[d_idx].valid   <= 1'b0;
                new_col_D[d_idx].offset  <= '0;
                m_pre_col[d_idx].valid   <= 1'b0;
                m_pre_col[d_idx].offset  <= '0;
                new_origins[d_idx]       <= '0;
              end
            end
          end

          // Advance k_base for next cycle
          if (int'(signed'(k_base_r)) + P_SUB_P - 1 >= K_MAX_P) begin
            // Done with all diagonals for this score step
            // Move to EXTEND_K phase
            ext_k_cursor   <= -K_WIDTH_P'(K_MAX_P);
            ext_k_done_cnt <= '0;
            state_r        <= A_EXTEND_K;
          end else begin
            k_base_r <= k_base_r + K_WIDTH_P'(P_SUB_P);
            compute_cyc_r <= compute_cyc_r + 1'b1;
          end
        end

        // ===================================================================
        // EXTEND_K: time-multiplex single extend_submodule across all diagonals
        // For each k with valid m_pre, call extend; skip nulls.
        A_EXTEND_K: begin
          if (ext_k_cursor > K_WIDTH_P'(K_MAX_P)) begin
            // All extends complete → write window
            state_r <= A_WR_WINDOW;
          end else begin
            begin
              int d_idx;
              d_idx = int'(signed'(ext_k_cursor)) + K_MAX_P;
              if (m_pre_col[d_idx].valid) begin
                // Issue extend for this k
                ext_start     <= 1'b1;
                ext_k_in      <= ext_k_cursor;
                ext_offset_in <= m_pre_col[d_idx].offset;
                state_r       <= A_EXTEND_WAIT;
              end else begin
                // Skip null diagonal
                ext_k_cursor <= ext_k_cursor + K_WIDTH_P'(1);
              end
            end
          end
        end

        // ===================================================================
        A_EXTEND_WAIT: begin
          if (ext_done) begin
            // Store extended result into new_col_M
            begin
              int d_idx;
              d_idx = int'(signed'(ext_k_cursor)) + K_MAX_P;
              if (d_idx >= 0 && d_idx < NUM_DIAGS) begin
                if (m_pre_col[d_idx].valid) begin
                  new_col_M[d_idx].valid  <= 1'b1;
                  new_col_M[d_idx].offset <= ext_offset_out;
                end
              end
            end
            // Advance to next diagonal
            ext_k_cursor <= ext_k_cursor + K_WIDTH_P'(1);
            state_r      <= A_EXTEND_K;
          end
        end

        // ===================================================================
        A_DONE_ALIGN: begin
          bt_start <= 1'b1;
          state_r  <= A_BACKTRACE;
        end

        // ===================================================================
        A_BACKTRACE: begin
          if (bt_done) begin
            state_r <= A_OUTPUT;
          end
        end

        // ===================================================================
        A_OUTPUT: begin
          // Assemble result_word: [127:112]=id, [111:96]=score, [95:32]=cigar, [31:0]=pad
          result_word  <= {r_job_id, 16'(score_opt), bt_cigar, 32'h0};
          result_valid <= 1'b1;
          state_r      <= A_IDLE;
        end

        // ===================================================================
        A_CPU_FALLBACK: begin
          needs_cpu_fallback <= 1'b1;
          result_word        <= {r_job_id, 16'hFFFF, 64'h0, 32'h0}; // sentinel
          result_valid       <= 1'b1;
          state_r            <= A_IDLE;
        end

        default: state_r <= A_IDLE;
      endcase
    end
  end

endmodule

`default_nettype wire
