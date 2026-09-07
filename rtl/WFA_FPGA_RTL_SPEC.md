# WFA-FPGA Accelerator — Dac Ta Thiet Ke RTL (Design Specification)

**Tham chieu:** A. Haghi, S. Marco-Sola, L. Alvarez, D. Diamantopoulos, C. Hagleitner,
M. Moreto, *"An FPGA Accelerator of the Wavefront Algorithm for Genomics Pairwise
Alignment"*, FPL 2021.

Tai lieu nay mo ta kien truc muc RTL de trien khai lai bo tang toc phan cung cho
thuat toan Wavefront Alignment (WFA), dung lam dac ta (spec) cho agent code sinh
RTL SystemVerilog. Cac cong thuc toan hoc va cac con so kien truc (do rong cua so,
so bit backtrace...) lay truc tiep tu bai bao.

---

## 1. Tong quan kien truc

```
                 +---------------------------+
Data In -------->|         EXTRACTOR         |
                 |  Extract -> Assign        |
                 +------------+--------------+
                              | Inputs (packed seq1/seq2, ID, Len1, Len2)
                              v
                 +---------------------------+
                 |   ALIGNER[0..N-1]         |  (N ban sao chay song song)
                 |  Extend -> Compute -> BT  |
                 +------------+--------------+
                     Status |  | Results (compact CIGAR, 16B/ket qua)
                     (ve Extractor)   v
                              +---------------------------+
                              |         COLLECTOR         |
                              | Packer x N -> Scheduler   |
                              +------------+--------------+
                                           v
                                       Data Out
```

Ba module chinh, dung nhu Hinh 3 cua bai bao:
- **Extractor**: doc trinh tu tho, nen 2-bit/base, dong goi nhom 8 base, phan
  cong cap trinh tu cho Aligner dang ranh (theo tin hieu `Status`).
- **Aligner** (x N ban sao, tham so hoa duoc): thuc hien thuat toan WFA
  (extend/compute/backtrace) cho 1 cap trinh tu.
- **Collector**: gom ket qua tu N Aligner, dong goi thanh tu 128-byte, gui ra
  ngoai theo thu tu do Scheduler quyet dinh.

---

## 2. Tham so thiet ke (parameters)

| Tham so | Y nghia | Gia tri vi du (Bang I bai bao) |
|---|---|---|
| `X_PEN` | mismatch penalty | 4 |
| `O_PEN` | gap-open penalty | 6 |
| `E_PEN` | gap-extend penalty | 2 |
| `K_MAX` | gioi han duong cheo (-K..+K) cua thiet ke | 16, 32, 64 (tuy design) |
| `SEQ_LEN_MAX` | do dai trinh tu toi da ho tro | 100, 150, 300 |
| `P_SUBMODULES` | so cap Extend/Compute sub-module song song | 8 (co dinh theo bai bao) |
| `BLOCK_SIZE` | so base moi khoi dong goi/so sanh | 8 |
| `NUM_ALIGNERS` | so ban sao Aligner tren 1 FPGA | 40..100 (tuy K, SEQ_LEN_MAX) |
| `BT_RAM_WIDTH` | do rong Backtrace RAM | 40 bit (8 o x 5 bit/o) |
| `BT_RAM_DEPTH` | do sau Backtrace RAM | 250 tu (ho tro K toi +-32) |

Cua so wavefront (Section III.B.1 bai bao):
```
window_I = E_PEN
window_D = E_PEN
window_M = max(X_PEN, O_PEN + E_PEN)
```
Vi du (4,6,2): window_I=2, window_D=2, window_M=8.

---

## 3. Cong thuc toan hoc (Equation 1) — dung nguyen ban khong thay doi

Voi offset = h (vi tri tren seq1), v = h - k (vi tri tren seq2), k = duong cheo:

```
I[s,k] = max( M[s-o-e, k-1], I[s-e, k-1] ) + 1
D[s,k] = max( M[s-o-e, k+1], D[s-e, k+1] )
M[s,k] = max( M[s-x, k] + 1, I[s,k], D[s,k] )        (truoc extend)
M[s,k] = extend(k, M[s,k])                            (sau extend)
```

`extend(k, h)`: tang h va v=h-k dong thoi min khi `seq1[h] == seq2[v]`.

Dieu kien dung: ton tai s sao cho `M[s][k_target] == n1` va
`M[s][k_target] - k_target == n2`, voi `k_target = n1 - n2`.

---

## 4. Module: EXTRACTOR

### 4.1 Sub-FSM "Extract"
- Doc tu bo nho: ID, Seq1 (raw ASCII), Len1, Seq2 (raw ASCII), Len2.
- Trang thai: `IDLE -> READ_HEADER -> READ_SEQ1 -> READ_SEQ2 -> DONE`.

### 4.2 Sub-FSM "Assign"
- **Base encoder**: bang tra cuu to hop (combinational LUT)
  `A->00, C->01, G->10, T->11` (2 bit/ky tu).
- **Group Maker**: dong goi 8 base lien tiep (2 bit x 8 = 16 bit) thanh 1
  "group". So group cho seq1 = `ceil(Len1 / BLOCK_SIZE)`.
- Output: mang cac group 16-bit cho seq1 va seq2, kem ID/Len1/Len2.
- Khi mot Aligner bao `Status = READY/IDLE`, Extractor gan (assign) 1 cap
  cong viec cho no qua bus `Inputs`.

### 4.3 Interface
```
input  logic         clk, rst_n;
input  logic [7:0]   data_in;        // Data In (byte stream)
input  logic         data_in_valid;
input  logic [NUM_ALIGNERS-1:0] aligner_status; // 1 = idle/ready
output logic [NUM_ALIGNERS-1:0] aligner_start;  // pulse khi gan job
output logic [ID_WIDTH-1:0]     job_id;
output logic [15:0]             job_seq1_groups [0:MAX_GROUPS-1];
output logic [15:0]             job_seq2_groups [0:MAX_GROUPS-1];
output logic [LEN_WIDTH-1:0]    job_len1, job_len2;
```

---

## 5. Module: ALIGNER (1 ban sao)

### 5.1 FSM tong quat
```
IDLE -> LOAD_JOB -> EXTEND(s=0) -> {COMPUTE(s) -> EXTEND(s)} lap tang s
      -> DONE_ALIGN -> BACKTRACE -> OUTPUT_CIGAR -> IDLE
```

### 5.2 Sub-module EXTEND

Chuc nang: voi (k, h) dau vao, tra ve h moi sau khi truot toi da theo duong cheo.

**Datapath (dung Hinh 5):**
1. `mux_seq1_blockA`, `mux_seq1_blockB` (MUX N:1): chon 2 group 8-base lien
   tiep cua seq1 chua vi tri h hien tai.
2. `concat`: ghep 2 group (16-bit + 16-bit = 32-bit, tuong duong 16 base).
3. `shift`: dich theo `(h mod BLOCK_SIZE)` de can dung 8 base can so sanh vao
   dau Comparator.
4. Tuong tu cho seq2 (dua vao "Seq 2 Comparator input").
5. `comparator_8`: so sanh song song 8 cap base, tra ve `matches_num` (0..8)
   = so ky tu khop lien tiep TU DAU (dung XOR + priority-encoder tim bit 0
   dau tien trong vector match).
6. `extend_ctrl` (FSM con): cong don `matches_num` vao offset; NEU
   `matches_num == 8` VA chua cham cuoi chuoi -> lap lai vong tiep theo
   (nap block moi); NGUOC LAI (matches_num < 8 hoac cham cuoi chuoi) -> xuat
   `Offset Out`, ket thuc.

**Interface:**
```
input  logic         start;
input  logic signed [K_WIDTH-1:0]  k_in;
input  logic [OFF_WIDTH-1:0]       offset_in;
input  logic [LEN_WIDTH-1:0]       seq_len1, seq_len2;
input  logic signed [K_WIDTH-1:0]  k_bound;      // K
output logic [OFF_WIDTH-1:0]       offset_out;
output logic                       done;
```

### 5.3 Sub-module COMPUTE

**Datapath (theo dung Equation 1, 8 ban sao song song cho 8 o k khac nhau
trong cung 1 frame column, dung MUX chon input theo So do Hinh 4B):**

```
for moi o k trong frame column (xu ly theo nhom P_SUBMODULES=8, nhieu chu
ky neu can, dung mux chon nhu Hinh 4B):

  from_M_I = window_M.read(s - o - e, k - 1)
  from_I   = window_I.read(s - e,     k - 1)
  i_val    = max(from_M_I, from_I) + 1
  i_origin = (from_M_I >= from_I) ? ORIGIN_FROM_M : ORIGIN_FROM_I   // 1 bit

  from_M_D = window_M.read(s - o - e, k + 1)
  from_D   = window_D.read(s - e,     k + 1)
  d_val    = max(from_M_D, from_D)
  d_origin = (from_M_D >= from_D) ? ORIGIN_FROM_M : ORIGIN_FROM_D   // 1 bit

  from_MM  = window_M.read(s - x, k) + 1
  m_pre    = max(from_MM, i_val, d_val)
  m_origin = argmax(from_MM, i_val, d_val)   // 3 gia tri -> can 2 bit,
                                              // bai bao dung 3 bit du phong

  null_flag = !(valid(from_M_I) || valid(from_I) || valid(from_M_D)
                || valid(from_D) || valid(from_MM))

  // dong goi origin: 1 (I) + 1 (D) + 3 (M) = 5 bit -> ghi Backtrace RAM
  origin_packed = {m_origin[2:0], d_origin, i_origin}

  goi m_pre sang Extend sub-module de tinh m_val = extend(k, m_pre)
```

**Null tag**: neu tat ca input deu invalid, danh dau `null_flag=1` cho o do;
Compute sub-module tiep theo tra ve gia tri am/invalid va **bo qua** goi
Extend cho o Null (tiet kiem 1 chu ky).

**Interface:**
```
input  logic [S_WIDTH-1:0]  s_in;
input  logic signed [K_WIDTH-1:0] k_in;
input  logic [OFF_WIDTH-1:0] win_M_rd, win_I_rd, win_D_rd; // tu Wavefront Window
output logic [OFF_WIDTH-1:0] m_pre_out;
output logic [4:0]           origin_out;     // 5-bit origin
output logic                 null_out;
```

### 5.4 Wavefront Window (bo nho cua so truot)

- 3 mang thanh ghi (shift-register-file), moi mang co do sau tuong ung
  `window_M`, `window_I`, `window_D` cot, moi cot co `2*K_MAX+1` phan tu
  (mot cho moi k tu -K_MAX den +K_MAX).
- Sau khi tinh xong 1 frame column moi: **ghi cot moi vao dau**, **xoa cot
  cu nhat** (shift toan bo cac cot con lai xuong 1 vi tri) — dung cai dat
  bang FIFO/shift-register cho tung k, hoac bank RAM dia chi vong (circular
  buffer) voi con tro `wr_ptr` chay vong theo modulo `window_depth`.

### 5.5 Sub-module BACKTRACE

**Backtrace RAM:**
- Do rong: `BT_RAM_WIDTH = 40 bit` (8 o x 5 bit/o, ghi cung luc 1 nhom 8 o
  duoc Compute xu ly trong 1 chu ky).
- Do sau: `BT_RAM_DEPTH = 250` tu (ho tro K toi +-32, theo Bang II bai bao).
- Ghi (write): dia chi tang dan theo thu tu Controller cua Aligner dieu
  khien (moi frame column moi -> 1 dia chi ghi moi).

**Address Decoder:**
- Dau vao: `score` hien tai (khi dang backtrace), `k` hien tai, `K_MAX`.
- Cong thuc dia chi (vi du don gian, can dieu chinh theo cach ma hoa cot
  thuc te): `addr = f(score, K_MAX)` — vi moi diem so `s` chi co dung 1
  "hang" trong RAM (ghi tuan tu luc compute), Address Decoder chi can anh
  xa `score -> addr` bang mot bo dem/cong don duoc luu lai tu luc ghi.

**Backtrace Controller FSM:**
```
INIT: s = score_opt, k = k_target, mat = M
LOOP:
  doc origin tai (s,k) tu Backtrace RAM qua Address Decoder
  neu mat == M:
    neu origin == MM: emit 'X'; s -= x
    neu origin == I:  mat = I  (khong emit, khong doi s,k)
    neu origin == D:  mat = D
  neu mat == I:
    emit 'I'
    neu origin == FROM_M: s -= (o+e); k -= 1; mat = M
    neu origin == FROM_I: s -= e;     k -= 1
  neu mat == D:
    emit 'D'
    neu origin == FROM_M: s -= (o+e); k += 1; mat = M
    neu origin == FROM_D: s -= e;     k += 1
  neu s==0 va k==0 va mat==M: DONE
```
- Ket qua: chuoi bit (2-bit/thao tac: 00=X, 01=I, 10=D) dong goi thanh
  **compact CIGAR 8-byte**, gui kem `job_id` ve Collector.

**Interface:**
```
input  logic                 start;
input  logic [S_WIDTH-1:0]   score_opt;
input  logic signed [K_WIDTH-1:0] k_target;
output logic [63:0]          compact_cigar;   // 8 byte
output logic                 done;
```

### 5.6 Aligner top interface (dung dung Hinh 3)
```
input  logic         clk, rst_n;
input  logic         start;             // tu Extractor
input  logic [...]   seq1_groups, seq2_groups, len1, len2, job_id;
output logic         status_idle;       // ve Extractor
output logic [127:0] result_word;       // ve Collector (16 byte: id+score+cigar)
output logic         result_valid;
```

---

## 6. Module: COLLECTOR

- **Packer[i]** (1 cai/Aligner): dem du 8 ket qua (16 byte/ket qua = 128
  byte) roi day 1 tu 128-byte vao FIFO cho Scheduler.
- **Scheduler**: trong tai round-robin (hoac priority) giua N Packer, chon
  1 tu 128-byte moi chu ky de day ra `Data Out`.

```
input  logic [127:0] packer_word [0:NUM_ALIGNERS-1];
input  logic [NUM_ALIGNERS-1:0] packer_valid;
output logic [127:0] data_out;
output logic          data_out_valid;
output logic [NUM_ALIGNERS-1:0] packer_ready; // bao packer nao da duoc phuc vu
```

---

## 7. Top-level

```systemverilog
module wfa_accelerator_top #(
    parameter int X_PEN = 4, O_PEN = 6, E_PEN = 2,
    parameter int K_MAX = 32,
    parameter int SEQ_LEN_MAX = 150,
    parameter int NUM_ALIGNERS = 64
) (
    input  logic clk, rst_n,
    input  logic [7:0] data_in, input logic data_in_valid,
    output logic [127:0] data_out, output logic data_out_valid
);
    // generate NUM_ALIGNERS ban sao aligner_top, noi Extractor <-> Collector
endmodule
```

---

## 8. Kiem chung (Verification Plan)

1. **Mo hinh tham chieu phan mem**: dung dung cong thuc muc 3 (co the tai su
   dung script Python WFA thuan thuat toan da co) de sinh test-vector:
   `(seq1, seq2, x, o, e, K) -> (score_ky_vong, compact_cigar_ky_vong)`.
2. **Testbench phan cap**:
   - Unit test rieng cho `extend_submodule` (so sanh 2 chuoi ngan, kiem tra
     `matches_num` va vong lap nhieu block).
   - Unit test cho `compute_submodule` (dua vao gia tri wavefront gia lap,
     kiem tra I/D/M va origin dung cong thuc).
   - Unit test cho `backtrace_ctrl` (nap san Backtrace RAM voi origin da
     biet, kiem tra chuoi CIGAR sinh ra).
   - Test tich hop `aligner_top` full-flow voi test-vector tu buoc 1.
   - Test `wfa_accelerator_top` voi nhieu cap trinh tu dong thoi, kiem tra
     Collector dong goi dung thu tu/khong mat du lieu.
3. **Test case bat buoc**: khop hoan toan; 1 mismatch; 1 insertion; 1
   deletion; ket hop nhieu loai; truong hop vuot K thiet ke (ky vong co
   flag "needs_cpu_fallback"/tin hieu loi ro rang, khong duoc cho ra ket
   qua sai ma khong bao).

---

## 9. Danh sach file RTL de xuat

```
rtl/
  pkg_wfa_params.sv
  base_encoder.sv
  extractor_assign.sv
  extractor_top.sv
  extend_submodule.sv
  compute_submodule.sv
  wavefront_window.sv
  backtrace_ram.sv
  address_decoder.sv
  backtrace_ctrl.sv
  aligner_top.sv
  collector_packer.sv
  collector_scheduler.sv
  collector_top.sv
  wfa_accelerator_top.sv
tb/
  tb_extend_submodule.sv
  tb_compute_submodule.sv
  tb_backtrace_ctrl.sv
  tb_aligner_top.sv
  tb_wfa_accelerator_top.sv
scripts/
  gen_reference_vectors.py   # sinh test-vector tu mo hinh phan mem
```
