`timescale 1ns/1ps
// Time unit of this testbench = 1 ADC sample (203.45 ps at 4.9152 GSps).
//   AWG clock 614.4 MHz = 8 units, ADC clock 409.6 MHz = 12 units, frame 204.8 MHz = 24 units.
// Sample n of the ADC is "taken" at time n. The ADC beat presented after the ADC edge at time 12m
// carries samples 12m .. 12m+11 (any fixed latency is fine: it is one constant offset).
module tb;
    localparam DL = 14;                   // real depth: 16384 beats
    localparam DEPTH = 1 << DL;
    reg awg = 1, adc = 1, frm = 1, ps = 0;
    always #4  awg = ~awg;
    always #6  adc = ~adc;
    always #12 frm = ~frm;
    always #23 ps  = ~ps;                 // unrelated PS clock

    // ---------------- ADC stream ----------------
    integer off = 0;                      // sample(n) = n - off  (16-bit)
    // ch1 carries the same ramp shifted by D1 samples: ch1(n) = ch0(n) + D1
    localparam integer D1 = 7;
    reg [191:0] tdata = 0, tdata1 = 0;
    integer k;
    always @(posedge adc) begin : gen
        reg [191:0] t, u;
        for (k = 0; k < 12; k = k + 1) begin
            t[16*k +: 16] = ($time + k) - off;
            u[16*k +: 16] = ($time + k) - off + D1;
        end
        tdata <= t; tdata1 <= u;
    end

    // ---------------- AWG sequence toggle ----------------
    reg seq = 0;
    integer awg_n = 0, fire_at = -1, ev_time = -1;
    always @(posedge awg) begin
        awg_n <= awg_n + 1;
        if (awg_n == fire_at) begin seq <= ~seq; ev_time <= $time; end
    end

    // ---------------- AXI-Lite ----------------
    reg [6:0] awaddr = 0, araddr = 0; reg [31:0] wdata = 0;
    reg awvalid = 0, wvalid = 0, bready = 1, arvalid = 0, rready = 1;
    wire awready, wready, bvalid, arready, rvalid; wire [31:0] rdata; wire [1:0] bresp, rresp;
    task wr(input [6:0] a, input [31:0] d); begin
        @(posedge ps); awaddr <= a; wdata <= d; awvalid <= 1; wvalid <= 1;
        @(posedge ps); while (!awready) @(posedge ps);
        awvalid <= 0; wvalid <= 0;
        while (!bvalid) @(posedge ps);
    end endtask
    task rd(input [6:0] a, output [31:0] d); begin
        @(posedge ps); araddr <= a; arvalid <= 1;
        @(posedge ps); while (!arready) @(posedge ps);
        arvalid <= 0;
        while (!rvalid) @(posedge ps);
        d = rdata;
        @(posedge ps);
    end endtask

    // ---------------- DUT + memory ----------------
    wire bclk, brst, ben; wire [63:0] bwe; wire [31:0] baddr; wire [511:0] bdin; wire mrst;
    adc_capture #(.DEPTH_LOG2(DL)) dut (
        .s_axi_aclk(ps), .s_axi_aresetn(1'b1),
        .s_axi_awaddr(awaddr), .s_axi_awprot(3'd0), .s_axi_awvalid(awvalid), .s_axi_awready(awready),
        .s_axi_wdata(wdata), .s_axi_wstrb(4'hF), .s_axi_wvalid(wvalid), .s_axi_wready(wready),
        .s_axi_bresp(bresp), .s_axi_bvalid(bvalid), .s_axi_bready(bready),
        .s_axi_araddr(araddr), .s_axi_arprot(3'd0), .s_axi_arvalid(arvalid), .s_axi_arready(arready),
        .s_axi_rdata(rdata), .s_axi_rresp(rresp), .s_axi_rvalid(rvalid), .s_axi_rready(rready),
        .adc_clk(adc), .adc_aresetn(1'b1), .s_axis_tdata(tdata), .s_axis_tvalid(1'b1), .s_axis_tready(),
        .s1_axis_tdata(tdata1), .s1_axis_tvalid(1'b1), .s1_axis_tready(),
        .bram_clk(bclk), .bram_rst(brst), .bram_en(ben), .bram_we(bwe), .bram_addr(baddr),
        .bram_din(bdin), .bram_dout(512'd0),
        .clk_awg(awg), .clk_frm(frm), .mmcm_locked(1'b1), .mmcm_reset(mrst),
        .awg_seq0_tog(seq), .awg_seq2_tog(1'b0), .ctrl_trig(1'b0));
    reg [511:0] mem [0:DEPTH-1];
    always @(posedge bclk) if (ben && &bwe) mem[baddr >> 6] <= bdin;

    // ---------------- helpers ----------------
    integer errors = 0;
    reg [31:0] v;
    function signed [15:0] smp(input integer beat, input integer s);
        smp = mem[beat % DEPTH][16*s +: 16];
    endfunction
    function signed [15:0] smp1(input integer beat, input integer s);     // ch1 (bits 256 +)
        smp1 = mem[beat % DEPTH][256 + 16*s +: 16];
    endfunction

    task capture(input [3:0] mode, input integer pre, input integer post, output integer taddr, output integer tsub);
        integer t0; begin
        wr(7'h04, mode); wr(7'h0C, pre); wr(7'h10, post);
        wr(7'h00, 1);                                  // ARM
        rd(7'h14, v); while (v[1]) rd(7'h14, v);       // BUSY gone
    end endtask
    task wait_done(output integer taddr, output integer tsub);
        integer n; begin
        n = 0; rd(7'h14, v);
        while (!v[0] && n < 2000) begin rd(7'h14, v); n = n + 1; end
        if (!v[0]) begin $display("ERROR: no DONE"); errors = errors + 1; end
        rd(7'h18, v); taddr = v; rd(7'h1C, v); tsub = v;
    end endtask
    // window [taddr-pre, taddr+post-1] must be contiguous samples (no stale beats)
    task check_window(input integer taddr, input integer pre, input integer post);
        integer b, base, e; begin
        base = smp((taddr - pre + DEPTH) % DEPTH, 0);
        e = 0;
        for (b = 0; b < pre + post; b = b + 1)
            if (((smp((taddr - pre + b + DEPTH) % DEPTH, 0) - base - 12 * b) & 32'hFFFF) != 0) e = e + 1;
        if (e) begin $display("ERROR: window not contiguous (%0d bad beats)", e); errors = errors + 1; end
        // ch1 must be sample-aligned with ch0 in every memory word: ch1 = ch0 + D1, upper bits zero
        e = 0;
        for (b = 0; b < pre + post; b = b + 1)
            for (k = 0; k < 12; k = k + 1)
                if (((smp1((taddr - pre + b + DEPTH) % DEPTH, k) - smp((taddr - pre + b + DEPTH) % DEPTH, k) - D1) & 32'hFFFF) != 0) e = e + 1;
        for (b = 0; b < pre + post; b = b + 1)
            if (mem[(taddr - pre + b + DEPTH) % DEPTH][511:448] !== 0 || mem[(taddr - pre + b + DEPTH) % DEPTH][255:192] !== 0) e = e + 1;
        if (e) begin $display("ERROR: ch1 not aligned with ch0 (%0d bad samples)", e); errors = errors + 1; end
    end endtask

    integer ta, ts, i, ofs, ofs0, j, delay;
    integer trial;
    initial begin
        #2000;
        rd(7'h28, v); $display("ID %h", v);
        rd(7'h30, v); $display("CFG %h (NS=%0d words=%0d sync=%0d nch=%0d)", v, v[7:0], v[15:8], v[16], v[27:24]);
        rd(7'h34, v); $display("FS_KHZ %0d", v);
        rd(7'h24, v); $display("DEPTH %0d", v);

        // ---- AWG-locked trigger: many events with all AWG phases and delays after ARM
        ofs0 = 99999;
        for (trial = 0; trial < 36; trial = trial + 1) begin
            capture(3, 20, 30, ta, ts);
            delay = 10 + (trial * 7) % 50;              // AWG beats after arming
            @(posedge awg); fire_at = awg_n + delay;
            wait_done(ta, ts);
            // event time ev_time in samples; reported sample number:
            j = smp(ta, ts) + off;                      // the n of that sample (mod 65536)
            ofs = (j - ev_time) & 32'hFFFF;
            if (ofs0 == 99999) ofs0 = ofs;
            if (ofs != ofs0) begin
                $display("ERROR trial %0d: AWG edge t=%0d (phase %0d mod 24) -> reported sample %0d, offset %0d (expected %0d)",
                         trial, ev_time, ev_time % 24, j, ofs, ofs0);
                errors = errors + 1;
            end
            check_window(ta, 20, 30);
            if (trial < 6) $display("awg0 trial %0d: edge t=%0d phase=%0d  TADDR=%0d TSUB=%0d  offset=%0d samples",
                         trial, ev_time, (ev_time % 24) / 8, ta, ts, ofs);
        end
        $display("AWG-locked: offset constant = %0d samples over 36 events", ofs0);

        // POST shorter than the event latency: still DONE, window still valid
        capture(3, 30, 2, ta, ts);
        @(posedge awg); fire_at = awg_n + 20;
        wait_done(ta, ts);
        check_window(ta, 30, 2);
        j = smp(ta, ts) + off; if (((j - ev_time) & 32'hFFFF) != ofs0) begin $display("ERROR short POST offset"); errors = errors + 1; end

        // event right after ARM (before PRE could hold): must be ignored, next one taken
        capture(3, 200, 20, ta, ts);
        @(posedge awg); fire_at = awg_n + 3;
        repeat (400) @(posedge awg);
        rd(7'h14, v); if (v[0]) begin $display("ERROR: early event accepted"); errors = errors + 1; end
        @(posedge awg); fire_at = awg_n + 5;
        wait_done(ta, ts);
        check_window(ta, 200, 20);
        j = smp(ta, ts) + off; if (((j - ev_time) & 32'hFFFF) != ofs0) begin $display("ERROR late event offset"); errors = errors + 1; end

        // ---- level trigger, every sample position in the beat
        for (i = 0; i < 24; i = i + 1) begin
            off = 1000000;                              // all samples negative-ish: n - off
            capture(1, 10, 10, ta, ts);
            wr(7'h08, 0);
            repeat (5) @(posedge adc);
            // place the 0-crossing at sample n0
            @(posedge adc); off = $time + 200 + i;      // sample n == off -> value 0
            wait_done(ta, ts);
            if (smp(ta, ts) !== 0 || (ts > 0 && smp(ta, ts - 1) !== -1) || (ts == 0 && smp(ta - 1, 11) !== -1)) begin
                $display("ERROR level i=%0d: TADDR=%0d TSUB=%0d value=%0d", i, ta, ts, smp(ta, ts)); errors = errors + 1;
            end
            check_window(ta, 10, 10);
        end
        $display("level rising: all sample positions checked");

        // ---- level trigger on ch1 (MODE bit 3): ch1 crosses 0 where ch0 = -D1
        for (i = 0; i < 24; i = i + 1) begin
            off = 1000000;
            capture(4'h9, 10, 10, ta, ts);
            wr(7'h08, 0);
            repeat (5) @(posedge adc);
            @(posedge adc); off = $time + 200 + i;
            wait_done(ta, ts);
            if (smp1(ta, ts) !== 0 || (ts > 0 && smp1(ta, ts - 1) !== -1) || (ts == 0 && smp1(ta - 1, 11) !== -1)) begin
                $display("ERROR level ch1 i=%0d: TADDR=%0d TSUB=%0d ch1=%0d ch0=%0d", i, ta, ts, smp1(ta, ts), smp(ta, ts)); errors = errors + 1;
            end
            if (smp(ta, ts) !== -D1) begin $display("ERROR level ch1 i=%0d: ch0=%0d at trigger, expected %0d", i, smp(ta, ts), -D1); errors = errors + 1; end
            check_window(ta, 10, 10);
        end
        $display("level rising on ch1: all sample positions checked");
        rd(7'h04, v); if (v !== 32'h9) begin $display("ERROR MODE readback %h", v); errors = errors + 1; end

        // ---- immediate + force
        off = 0;
        capture(0, 5, 5, ta, ts); wait_done(ta, ts); check_window(ta, 5, 5);
        capture(7, 5, 5, ta, ts); repeat (50) @(posedge ps); wr(7'h00, 2); wait_done(ta, ts); check_window(ta, 5, 5);

        // ---- MMCM reset register
        wr(7'h38, 1); if (mrst !== 1) begin $display("ERROR mmcm_reset"); errors = errors + 1; end
        wr(7'h38, 0); if (mrst !== 0) begin $display("ERROR mmcm_reset"); errors = errors + 1; end
        rd(7'h14, v); $display("STATUS %h (locked bit %0d)", v, v[2]);

        if (errors == 0) $display("PASS"); else $display("FAIL: %0d errors", errors);
        $finish;
    end
endmodule
