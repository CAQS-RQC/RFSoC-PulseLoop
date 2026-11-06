`timescale 1ns / 1ps
// ============================================================================
// adc_capture.v  -  triggered capture of the raw ADC stream, locked to the AWG
//
//  * Input : TWO RFDC ADC slices of tile 224, captured together, sample-aligned:
//              ch0 = slice 00 (vin0_01, ADC D)  -> s_axis
//              ch1 = slice 02 (vin0_23)         -> s1_axis
//            each real output, no decimation: 4.9152 GSps, 12 x int16 per beat (192 bit),
//            sample 0 (earliest) in [15:0]. Both slices share the tile and the stream clock.
//            ADC stream clock 409.6 MHz, made by an MMCM (clk_wiz_adc) from the DAC0 clock (614.4 MHz).
//            The stream is never back-pressured.
//  * Memory: every beat goes straight into a true-dual-port BRAM (port A, ADC clock),
//            one 512-bit word per beat: ch0 in bits [191:0], ch1 in bits [447:256] (rest zero).
//            The PS reads it through an AXI BRAM controller on port B.
//            16384 beats = 196608 samples per channel = 40.0 us.
//  * Ring buffer with pre-trigger: after ARM, PRE beats are filled, then the engine waits for the
//    trigger, writes POST beats from the trigger beat on, and stops (DONE).
//    The valid window is beats [TADDR-PRE, TADDR+POST-1] (modulo DEPTH).
//  * Trigger sources (MODE register):
//      0 immediate        4 AWG ch2 sequence start (asynchronous, +-1 beat)
//      1 level rising     5 control_trigger rising edge
//      2 level falling    6 control_trigger falling edge
//      3 AWG ch0 sequence start, locked: TADDR/TSUB give the ADC sample that corresponds to the
//        AWG ch0 sequence start, the same one every time (+ a constant converter latency that
//        software calibrates with a loopback cable)
//      7 manual only (FORCE bit)
//    Level triggers are sample accurate: TSUB gives the sample inside the trigger beat.
//    MODE bit 3 selects the channel the level trigger looks at (0 = ch0, 1 = ch1).
//  * AWG lock: the AWG ch0 event crosses   AWG 614.4 MHz -> frame 204.8 MHz -> ADC 409.6 MHz
//    through plain registers (synchronous, timed by Vivado: all three clocks come from the DAC0
//    clock), together with its position in the frame.
//    One frame (4.883 ns) = 3 AWG beats = 2 ADC beats = 24 ADC samples = 48 DAC samples.
//  * PS <-> ADC crossings use XPM CDC macros.
//
// Register map (AXI4-Lite, byte offsets)
//   0x00 CTRL    W  bit0 ARM, bit1 FORCE, bit2 ABORT (write 1, self-clearing)
//   0x04 MODE    RW [2:0] trigger source, [3] level-trigger channel (0 = ch0, 1 = ch1)
//   0x08 LEVEL   RW [15:0] signed threshold (int16 ADC code)
//   0x0C PRE     RW beats before the trigger beat (0 .. DEPTH-1)
//   0x10 POST    RW beats from the trigger beat on (1 .. DEPTH-PRE)
//   0x14 STATUS  R  bit0 DONE, bit1 BUSY (arm not yet seen by ADC side), bit2 ADC CLOCK LOCKED, [6:4] state
//   0x18 TADDR   R  beat address of the trigger beat
//   0x1C TSUB    R  [3:0] sample index (0..11) of the trigger inside the beat (level and AWG ch0)
//   0x20 TTIME   R  ADC clock cycles from ARM to trigger
//   0x24 DEPTH   R  buffer depth in beats
//   0x28 ID      R  0xADC5_0005
//   0x2C ALIVE   R  free-running count of valid ADC beats (shows the ADC is streaming)
//   0x30 CFG     R  [7:0] samples per beat (12), [15:8] 32-bit words per memory word (16), [16] AWG ch0 locked (1),
//                   [27:24] number of channels (2), channel c starts at 32-bit word 8*c of each memory word
//   0x34 FS_KHZ  R  sample rate in kHz (4915200)
//   0x38 CLKCTL  RW bit0 MMCM reset (write 1 then 0 once the DAC clock runs)
// ============================================================================
module adc_capture #(
    parameter DEPTH_LOG2 = 14            // 16384 beats = 196608 samples = 40.0 us at 4.9152 GSps
)(
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 s_axi_aclk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axi, ASSOCIATED_RESET s_axi_aresetn" *)
    input  wire         s_axi_aclk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 s_axi_aresetn RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  wire         s_axi_aresetn,
    input  wire [6:0]   s_axi_awaddr,
    input  wire [2:0]   s_axi_awprot,
    input  wire         s_axi_awvalid,
    output reg          s_axi_awready,
    input  wire [31:0]  s_axi_wdata,
    input  wire [3:0]   s_axi_wstrb,
    input  wire         s_axi_wvalid,
    output reg          s_axi_wready,
    output wire [1:0]   s_axi_bresp,
    output reg          s_axi_bvalid,
    input  wire         s_axi_bready,
    input  wire [6:0]   s_axi_araddr,
    input  wire [2:0]   s_axi_arprot,
    input  wire         s_axi_arvalid,
    output reg          s_axi_arready,
    output reg  [31:0]  s_axi_rdata,
    output wire [1:0]   s_axi_rresp,
    output reg          s_axi_rvalid,
    input  wire         s_axi_rready,

    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 adc_clk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axis:s1_axis, ASSOCIATED_RESET adc_aresetn" *)
    input  wire         adc_clk,        // 409.6 MHz, MMCM clk_out1 (from the DAC0 clock)
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 adc_aresetn RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  wire         adc_aresetn,
    input  wire [191:0] s_axis_tdata,   // ch0: 12 x int16, sample 0 (earliest) in [15:0]
    input  wire         s_axis_tvalid,
    output wire         s_axis_tready,
    input  wire [191:0] s1_axis_tdata,  // ch1: same format, same tile, same clock
    input  wire         s1_axis_tvalid,
    output wire         s1_axis_tready,

    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTA CLK" *)
    (* X_INTERFACE_PARAMETER = "MASTER_TYPE BRAM_CTRL, MEM_ECC NONE, MEM_WIDTH 512, MEM_SIZE 1048576, READ_WRITE_MODE WRITE_ONLY" *)
    output wire         bram_clk,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTA RST" *)
    output wire         bram_rst,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTA EN" *)
    output wire         bram_en,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTA WE" *)
    output wire [63:0]  bram_we,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTA ADDR" *)
    output wire [31:0]  bram_addr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTA DIN" *)
    output wire [511:0] bram_din,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTA DOUT" *)
    input  wire [511:0] bram_dout,

    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 clk_awg CLK" *)
    input  wire         clk_awg,        // 614.4 MHz AWG ch0 clock (util_ds_buf_0/BUFG_O)
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 clk_frm CLK" *)
    input  wire         clk_frm,        // 204.8 MHz, MMCM clk_out2
    input  wire         mmcm_locked,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 mmcm_reset RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_HIGH" *)
    output wire         mmcm_reset,     // to clk_wiz_adc/reset (active high)

    input  wire         awg_seq0_tog,   // toggles at each AWG ch0 sequence start (clk_awg)
    input  wire         awg_seq2_tog,   // toggles at each AWG ch2 sequence start (DAC2 clock)
    input  wire         ctrl_trig
);
    localparam NS    = 12;                           // samples per beat
    localparam NCH   = 2;                            // channels
    localparam MEM_W = 512;                          // memory word: 2 x (12 samples + 64 unused bits)
    localparam [31:0] FS_KHZ = 32'd4915200;
    localparam BW    = 16 * NS;
    localparam MB    = $clog2(MEM_W / 8);            // byte-address bits per memory word
    localparam DEPTH = 1 << DEPTH_LOG2;
    localparam PW    = DEPTH_LOG2 + 1;               // width of PRE / POST / counters
    localparam [31:0] CORE_ID = 32'hADC5_0005;
    localparam integer WPW = MEM_W / 32;             // 32-bit words per memory word
    localparam [31:0] CFG_WORD = (NCH << 24) | 32'h0001_0000 | (WPW << 8) | NS;   // bit 16: AWG ch0 locked

    localparam [2:0] S_IDLE = 3'd0, S_FILL = 3'd1, S_ARMED = 3'd2, S_POST = 3'd3, S_DONE = 3'd4;

    // =====================================================================
    // PS clock domain: AXI4-Lite registers
    // =====================================================================
    reg  [3:0]  r_mode  = 4'd1;
    reg  [15:0] r_level = 16'd3277;               // ~0.1 of full scale
    reg  [PW-1:0] r_pre  = DEPTH / 4;
    reg  [PW-1:0] r_post = DEPTH / 2;
    reg         r_mmcm_rst  = 1'b0;
    reg         arm_tog_ps   = 1'b0;
    reg         force_tog_ps = 1'b0;
    reg         abort_tog_ps = 1'b0;

    wire        fin_tog_ps, ack_tog_ps, locked_ps;
    wire [2:0]  state_ps;
    wire [DEPTH_LOG2-1:0] taddr_ps;
    wire [3:0]  tsub_ps;
    wire [31:0] ttime_ps;
    wire [31:0] alive_ps;

    assign s_axi_bresp = 2'b00;
    assign s_axi_rresp = 2'b00;
    assign mmcm_reset  = r_mmcm_rst;

    wire wr_go = s_axi_awvalid & s_axi_wvalid & ~s_axi_awready & ~s_axi_bvalid;
    wire rd_go = s_axi_arvalid & ~s_axi_arready & ~s_axi_rvalid;
    wire busy_ps = arm_tog_ps ^ ack_tog_ps;
    wire done_ps = ~(arm_tog_ps ^ fin_tog_ps) & ~busy_ps;

    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            s_axi_awready <= 1'b0;
            s_axi_wready  <= 1'b0;
            s_axi_bvalid  <= 1'b0;
            s_axi_arready <= 1'b0;
            s_axi_rvalid  <= 1'b0;
            s_axi_rdata   <= 32'd0;
        end else begin
            s_axi_awready <= wr_go;
            s_axi_wready  <= wr_go;
            if (s_axi_awready) begin
                s_axi_bvalid <= 1'b1;
                case (s_axi_awaddr[6:2])
                    5'h00: begin
                        if (s_axi_wdata[0]) arm_tog_ps   <= ~arm_tog_ps;
                        if (s_axi_wdata[1]) force_tog_ps <= ~force_tog_ps;
                        if (s_axi_wdata[2]) abort_tog_ps <= ~abort_tog_ps;
                    end
                    5'h01: r_mode  <= s_axi_wdata[3:0];
                    5'h02: r_level <= s_axi_wdata[15:0];
                    5'h03: r_pre   <= s_axi_wdata[PW-1:0];
                    5'h04: r_post  <= s_axi_wdata[PW-1:0];
                    5'h0E: r_mmcm_rst <= s_axi_wdata[0];
                    default: ;
                endcase
            end else if (s_axi_bready) begin
                s_axi_bvalid <= 1'b0;
            end
            s_axi_arready <= rd_go;
            if (s_axi_arready) begin
                s_axi_rvalid <= 1'b1;
                case (s_axi_araddr[6:2])
                    5'h01: s_axi_rdata <= {28'd0, r_mode};
                    5'h02: s_axi_rdata <= {16'd0, r_level};
                    5'h03: s_axi_rdata <= {{(32-PW){1'b0}}, r_pre};
                    5'h04: s_axi_rdata <= {{(32-PW){1'b0}}, r_post};
                    5'h05: s_axi_rdata <= {25'd0, state_ps, 1'b0, locked_ps, busy_ps, done_ps};
                    5'h06: s_axi_rdata <= {{(32-DEPTH_LOG2){1'b0}}, taddr_ps};
                    5'h07: s_axi_rdata <= {28'd0, tsub_ps};
                    5'h08: s_axi_rdata <= ttime_ps;
                    5'h09: s_axi_rdata <= DEPTH;
                    5'h0A: s_axi_rdata <= CORE_ID;
                    5'h0B: s_axi_rdata <= alive_ps;
                    5'h0C: s_axi_rdata <= CFG_WORD;
                    5'h0D: s_axi_rdata <= FS_KHZ;
                    5'h0E: s_axi_rdata <= {31'd0, r_mmcm_rst};
                    default: s_axi_rdata <= 32'd0;
                endcase
            end else if (s_axi_rready) begin
                s_axi_rvalid <= 1'b0;
            end
        end
    end

    // =====================================================================
    // CDC  PS -> ADC
    // =====================================================================
    localparam CFGW = 4 + 16 + PW + PW;
    wire [CFGW-1:0] cfg_adc;
    xpm_cdc_array_single #(.DEST_SYNC_FF(2), .INIT_SYNC_FF(0), .SIM_ASSERT_CHK(0),
                           .SRC_INPUT_REG(1), .WIDTH(CFGW))
        u_cdc_cfg (.src_clk(s_axi_aclk), .src_in({r_mode, r_level, r_pre, r_post}),
                   .dest_clk(adc_clk), .dest_out(cfg_adc));
    wire [3:0]    mode_a  = cfg_adc[CFGW-1 -: 4];
    wire [15:0]   level_a = cfg_adc[2*PW +: 16];
    wire [PW-1:0] pre_a   = cfg_adc[PW +: PW];
    wire [PW-1:0] post_a  = cfg_adc[0 +: PW];

    wire arm_s, force_s, abort_s;
    xpm_cdc_single #(.DEST_SYNC_FF(4), .INIT_SYNC_FF(0), .SIM_ASSERT_CHK(0), .SRC_INPUT_REG(1))
        u_cdc_arm   (.src_clk(s_axi_aclk), .src_in(arm_tog_ps),   .dest_clk(adc_clk), .dest_out(arm_s));
    xpm_cdc_single #(.DEST_SYNC_FF(4), .INIT_SYNC_FF(0), .SIM_ASSERT_CHK(0), .SRC_INPUT_REG(1))
        u_cdc_force (.src_clk(s_axi_aclk), .src_in(force_tog_ps), .dest_clk(adc_clk), .dest_out(force_s));
    xpm_cdc_single #(.DEST_SYNC_FF(4), .INIT_SYNC_FF(0), .SIM_ASSERT_CHK(0), .SRC_INPUT_REG(1))
        u_cdc_abort (.src_clk(s_axi_aclk), .src_in(abort_tog_ps), .dest_clk(adc_clk), .dest_out(abort_s));

    wire seq2_s, ctrl_s;
    xpm_cdc_single #(.DEST_SYNC_FF(3), .INIT_SYNC_FF(0), .SIM_ASSERT_CHK(0), .SRC_INPUT_REG(0))
        u_cdc_seq2 (.src_clk(1'b0), .src_in(awg_seq2_tog), .dest_clk(adc_clk), .dest_out(seq2_s));
    xpm_cdc_single #(.DEST_SYNC_FF(3), .INIT_SYNC_FF(0), .SIM_ASSERT_CHK(0), .SRC_INPUT_REG(0))
        u_cdc_ctrl (.src_clk(1'b0), .src_in(ctrl_trig),    .dest_clk(adc_clk), .dest_out(ctrl_s));

    // =====================================================================
    // ADC clock domain
    // =====================================================================
    assign s_axis_tready  = 1'b1;    // never back-pressure the RFDC
    assign s1_axis_tready = 1'b1;

    reg arm_d = 1'b0, force_d = 1'b0, abort_d = 1'b0;
    reg seq2_d = 1'b0, ctrl_d = 1'b0;
    always @(posedge adc_clk) begin
        arm_d <= arm_s;   force_d <= force_s;   abort_d <= abort_s;
        seq2_d <= seq2_s;     ctrl_d <= ctrl_s;
    end
    wire arm_ev   = arm_s   ^ arm_d;
    wire force_ev = force_s ^ force_d;
    wire abort_ev = abort_s ^ abort_d;
    wire seq2_ev  = seq2_s  ^ seq2_d;
    wire ctrl_r   = ctrl_s & ~ctrl_d;
    wire ctrl_f   = ~ctrl_s & ctrl_d;

    // ---- data / level-trigger pipeline -----------------------------------
    // st0: register the RFDC output          (d0)
    // st1: compare each sample with LEVEL    (d1, ge1)
    // st2: find crossings                    (d2, rise2, fall2)
    // st3: trigger decision + BRAM write of d2
    reg [BW-1:0] d0, d1, d2;          // ch0
    reg [BW-1:0] e0, e1d, e2d;        // ch1 (same pipeline)
    reg [BW-1:0] t0;                  // channel the level trigger looks at
    reg          tch = 1'b0;          // level-trigger channel (from MODE[3])
    reg          v0 = 1'b0, v1 = 1'b0, v2 = 1'b0;
    reg [NS-1:0] ge1, rise2, fall2;
    reg          ge_last = 1'b0;
    reg [15:0]   level_r = 16'd0;

    integer i;
    always @(posedge adc_clk) begin
        level_r <= level_a;
        tch   <= mode_a[3];
        d0 <= s_axis_tdata;  e0 <= s1_axis_tdata;  v0 <= s_axis_tvalid;
        t0 <= tch ? s1_axis_tdata : s_axis_tdata;
        d1 <= d0;            e1d <= e0;            v1 <= v0;
        for (i = 0; i < NS; i = i + 1)
            ge1[i] <= ($signed(t0[16*i +: 16]) >= $signed(level_r));
        d2 <= d1;            e2d <= e1d;           v2 <= v1;
        if (v1) begin
            rise2[0] <=  ge1[0] & ~ge_last;
            fall2[0] <= ~ge1[0] &  ge_last;
            for (i = 1; i < NS; i = i + 1) begin
                rise2[i] <=  ge1[i] & ~ge1[i-1];
                fall2[i] <= ~ge1[i] &  ge1[i-1];
            end
            ge_last <= ge1[NS-1];
        end else begin
            rise2 <= {NS{1'b0}};
            fall2 <= {NS{1'b0}};
        end
    end

    function [3:0] first_one;      // index of the lowest set bit (earliest sample)
        input [NS-1:0] x;
        integer k;
        begin
            first_one = NS - 1;
            for (k = NS - 1; k >= 0; k = k - 1)
                if (x[k]) first_one = k[3:0];
        end
    endfunction

    // ---- capture state machine -------------------------------------------
    reg [2:0]            state = S_IDLE;
    reg [DEPTH_LOG2-1:0] wptr  = {DEPTH_LOG2{1'b0}};
    reg [PW-1:0]         cnt   = {PW{1'b0}};
    reg [PW-1:0]         wcnt  = {PW{1'b0}};      // beats written since ARM (saturating)
    reg [PW-1:0]         pre_l = {PW{1'b0}}, post_l = {PW{1'b0}};
    reg [PW-1:0]         pre_m1 = {PW{1'b0}}, post_m1 = {PW{1'b0}};
    reg [2:0]            mode_l = 3'd0;
    reg                  force_pend = 1'b0;
    reg                  ack_tog = 1'b0;
    reg                  fin_tog = 1'b0;
    reg [3:0]            ack_dly = 4'd0;
    reg [DEPTH_LOG2-1:0] taddr = {DEPTH_LOG2{1'b0}};
    reg [3:0]            tsub  = 4'd0;
    reg [31:0]           tcnt  = 32'd0, ttime = 32'd0;
    reg [31:0]           alive = 32'd0;

    wire writing = v2 & ((state == S_FILL) | (state == S_ARMED) | (state == S_POST));

    // ---- AWG ch0 locked trigger ---------------------------------------------
    // ev3      : an AWG ch0 sequence start, valid for this capture
    // ev3_addr : beat address of the ADC beat holding the corresponding sample
    // ev3_sub  : sample index inside that beat
    // ev3_n    : beats from ev3_addr up to and including the beat written in this cycle
    wire                  ev3;
    wire [DEPTH_LOG2-1:0] ev3_addr;
    wire [3:0]            ev3_sub;
    wire [6:0]            ev3_n;

    // frame clock: free-running frame number
    reg [7:0] fcnt = 8'd0;
    always @(posedge clk_frm) fcnt <= fcnt + 8'd1;

    // AWG clock: AWG beat time (fa, aph) = 3*fa + aph, one step per AWG beat
    reg [7:0] fa_s = 8'd0, fa_s_d = 8'd0, fa = 8'd0;
    reg [1:0] aph = 2'd0;
    reg       sq_d = 1'b0, ev_tog = 1'b0;
    reg [7:0] ev_fa = 8'd0;
    reg [1:0] ev_ph = 2'd0;
    always @(posedge clk_awg) begin
        fa_s   <= fcnt;                       // frame -> AWG (synchronous, timed)
        fa_s_d <= fa_s;
        if (fa_s != fa_s_d) begin fa <= fa_s; aph <= 2'd0; end
        else                       aph <= aph + 2'd1;
        sq_d <= awg_seq0_tog;
        if (awg_seq0_tog != sq_d) begin
            ev_tog <= ~ev_tog;
            ev_fa  <= fa;
            ev_ph  <= aph;
        end
    end

    // frame clock: carry the event bundle (stable for >= 1 AWG beat, sampled as a whole)
    reg       f_tog = 1'b0;
    reg [7:0] f_fa  = 8'd0;
    reg [1:0] f_ph  = 2'd0;
    always @(posedge clk_frm) begin
        f_tog <= ev_tog; f_fa <= ev_fa; f_ph <= ev_ph;
    end

    // ADC clock: ADC beat time (fd, dph) = 2*fd + dph, one step per ADC beat
    reg [7:0] fd_s = 8'd0, fd_s_d = 8'd0, fd = 8'd0;
    reg       dph = 1'b0;
    reg       a_tog = 1'b0, a_tog_d = 1'b0;
    reg [7:0] a_fa = 8'd0;
    reg [1:0] a_ph = 2'd0;
    // stage 1
    reg       e1 = 1'b0;
    reg [8:0] back1 = 9'd0;
    reg [3:0] sub1 = 4'd0;
    reg [DEPTH_LOG2-1:0] wp1 = {DEPTH_LOG2{1'b0}};
    reg [PW:0] wcp1 = {(PW+1){1'b0}};          // beats written since ARM minus PRE
    // stage 2
    reg       e2 = 1'b0;
    reg [DEPTH_LOG2-1:0] addr2 = {DEPTH_LOG2{1'b0}};
    reg [3:0] sub2 = 4'd0;
    reg [6:0] n2 = 7'd0;

    // event position in ADC beats back from now:
    //   AWG time in samples  = 24*a_fa + 8*a_ph,  ADC time in samples = 24*fd + 12*dph
    //   q = 2*(fd - a_fa) + dph ;  back = q - (a_ph == 2) ;  sub = {0, 8, 4}[a_ph]
    // BACK_OFS aligns TADDR/TSUB with the ADC beat in the BRAM (fixed pipeline latency).
    localparam [8:0] BACK_OFS = 9'd4;
    wire [7:0] df = fd - a_fa;
    wire [DEPTH_LOG2-1:0] back1_x = {{(DEPTH_LOG2-9){1'b0}}, back1};   // zero-extended to the address width
    wire [8:0] q  = {df, 1'b0} + {8'd0, dph};
    always @(posedge adc_clk) begin
        fd_s   <= fcnt;                       // frame -> ADC (synchronous, timed)
        fd_s_d <= fd_s;
        if (fd_s != fd_s_d) begin fd <= fd_s; dph <= 1'b0; end
        else                       dph <= ~dph;
        a_tog <= f_tog; a_fa <= f_fa; a_ph <= f_ph;      // frame -> ADC (synchronous, timed)
        a_tog_d <= a_tog;

        e1    <= (a_tog != a_tog_d) && (q < 9'd48) && (a_ph != 2'd3);
        back1 <= q - {8'd0, (a_ph == 2'd2)} + BACK_OFS;
        sub1  <= (a_ph == 2'd0) ? 4'd0 : (a_ph == 2'd1) ? 4'd8 : 4'd4;
        wp1   <= wptr;
        wcp1  <= {1'b0, wcnt} - {1'b0, pre_l};

        // stage 2: the event beat must have been written after ARM, PRE beats before it too
        e2    <= e1 && !wcp1[PW] && (wcp1[PW-1:0] >= {{(PW-9){1'b0}}, back1});
        addr2 <= wp1 - back1_x;
        sub2  <= sub1;
        n2    <= back1[6:0] + 7'd3;           // wptr has moved by 2 since stage 1
    end
    assign ev3 = e2;
    assign ev3_addr = addr2;
    assign ev3_sub  = sub2;
    assign ev3_n    = n2;

    reg       hit;
    reg [3:0] hit_sub;
    always @(*) begin
        hit = 1'b0; hit_sub = 4'd0;
        case (mode_l)
            3'd0: hit = 1'b1;
            3'd1: begin hit = |rise2; hit_sub = first_one(rise2); end
            3'd2: begin hit = |fall2; hit_sub = first_one(fall2); end
            3'd3: hit = 1'b0;                     // AWG ch0: awg_hit below
            3'd4: hit = seq2_ev;
            3'd5: hit = ctrl_r;
            3'd6: hit = ctrl_f;
            default: hit = 1'b0;
        endcase
    end
    wire awg_hit = (mode_l == 3'd3) && ev3;

    always @(posedge adc_clk) begin
        if (v0) alive <= alive + 32'd1;
        ack_dly <= {ack_dly[2:0], ack_tog};
        if (!adc_aresetn) begin
            state <= S_IDLE;
            force_pend <= 1'b0;
        end else if (abort_ev) begin
            state <= S_IDLE;
            force_pend <= 1'b0;
        end else if (arm_ev) begin
            pre_l   <= pre_a;
            pre_m1  <= pre_a - 1'b1;
            post_l  <= (post_a == {PW{1'b0}}) ? {{(PW-1){1'b0}}, 1'b1} : post_a;
            post_m1 <= (post_a == {PW{1'b0}}) ? {PW{1'b0}} : post_a - 1'b1;
            mode_l  <= mode_a[2:0];
            cnt     <= {PW{1'b0}};
            wcnt    <= {PW{1'b0}};
            tcnt    <= 32'd0;
            force_pend <= 1'b0;
            ack_tog <= arm_s;
            fin_tog <= ~arm_s;
            state   <= (pre_a == {PW{1'b0}}) ? S_ARMED : S_FILL;
        end else begin
            if (force_ev) force_pend <= 1'b1;
            if ((state == S_FILL) | (state == S_ARMED) | (state == S_POST))
                tcnt <= tcnt + 32'd1;
            if (writing) begin
                wptr <= wptr + 1'b1;
                if (~&wcnt) wcnt <= wcnt + 1'b1;
                case (state)
                    S_FILL: begin
                        cnt <= cnt + 1'b1;
                        if (cnt == pre_m1) state <= S_ARMED;
                    end
                    S_ARMED: begin
                        if (awg_hit) begin
                            taddr <= ev3_addr;
                            tsub  <= ev3_sub;
                            ttime <= tcnt;
                            force_pend <= 1'b0;
                            if ({{(PW-7){1'b0}}, ev3_n} >= post_l) begin
                                state <= S_DONE;
                                fin_tog <= ack_tog;
                            end else begin
                                state <= S_POST;
                                cnt   <= {{(PW-7){1'b0}}, ev3_n};
                            end
                        end else if (hit | force_pend | force_ev) begin
                            taddr      <= wptr;
                            tsub       <= hit ? hit_sub : 4'd0;
                            ttime      <= tcnt;
                            force_pend <= 1'b0;
                            if (post_m1 == {PW{1'b0}}) begin
                                state <= S_DONE;
                                fin_tog <= ack_tog;
                            end else begin
                                state <= S_POST;
                                cnt   <= {{(PW-1){1'b0}}, 1'b1};
                            end
                        end
                    end
                    S_POST: begin
                        cnt <= cnt + 1'b1;
                        if (cnt == post_m1) begin
                            state <= S_DONE;
                            fin_tog <= ack_tog;
                        end
                    end
                    default: ;
                endcase
            end
        end
    end

    assign bram_clk  = adc_clk;
    assign bram_rst  = 1'b0;
    assign bram_en   = writing;
    assign bram_we   = {(MEM_W/8){writing}};
    assign bram_addr = {{(32-DEPTH_LOG2-MB){1'b0}}, wptr, {MB{1'b0}}};
    assign bram_din  = {{(MEM_W/2-BW){1'b0}}, e2d, {(MEM_W/2-BW){1'b0}}, d2};   // ch1 | ch0

    // =====================================================================
    // CDC  ADC -> PS
    // =====================================================================
    xpm_cdc_single #(.DEST_SYNC_FF(3), .INIT_SYNC_FF(0), .SIM_ASSERT_CHK(0), .SRC_INPUT_REG(1))
        u_cdc_fin  (.src_clk(adc_clk), .src_in(fin_tog), .dest_clk(s_axi_aclk), .dest_out(fin_tog_ps));
    xpm_cdc_single #(.DEST_SYNC_FF(3), .INIT_SYNC_FF(0), .SIM_ASSERT_CHK(0), .SRC_INPUT_REG(1))
        u_cdc_ack  (.src_clk(adc_clk), .src_in(ack_dly[3]), .dest_clk(s_axi_aclk), .dest_out(ack_tog_ps));
    xpm_cdc_single #(.DEST_SYNC_FF(3), .INIT_SYNC_FF(0), .SIM_ASSERT_CHK(0), .SRC_INPUT_REG(0))
        u_cdc_lock (.src_clk(1'b0), .src_in(mmcm_locked), .dest_clk(s_axi_aclk), .dest_out(locked_ps));

    localparam STW = 3 + DEPTH_LOG2 + 4 + 32;
    wire [STW-1:0] st_ps;
    xpm_cdc_array_single #(.DEST_SYNC_FF(2), .INIT_SYNC_FF(0), .SIM_ASSERT_CHK(0),
                           .SRC_INPUT_REG(1), .WIDTH(STW))
        u_cdc_stat (.src_clk(adc_clk), .src_in({state, taddr, tsub, ttime}),
                    .dest_clk(s_axi_aclk), .dest_out(st_ps));
    assign state_ps = st_ps[STW-1 -: 3];
    assign taddr_ps = st_ps[36 +: DEPTH_LOG2];
    assign tsub_ps  = st_ps[32 +: 4];
    assign ttime_ps = st_ps[31:0];

    xpm_cdc_gray #(.DEST_SYNC_FF(3), .INIT_SYNC_FF(0), .REG_OUTPUT(1), .SIM_ASSERT_CHK(0),
                   .SIM_LOSSLESS_GRAY_CHK(0), .WIDTH(32))
        u_cdc_alive (.src_clk(adc_clk), .src_in_bin(alive), .dest_clk(s_axi_aclk), .dest_out_bin(alive_ps));

endmodule
