`ifndef TIMER_MMIO_V
`define TIMER_MMIO_V

`timescale 1ns/1ps

`ifndef TIMER_DEFAULT_PRESCALER
`define TIMER_DEFAULT_PRESCALER 32'd1000
`endif

`ifndef TIMER_MAX_COMPARE
`define TIMER_MAX_COMPARE       32'hFFFF_FFFF
`endif

module timer_mmio #(
    parameter [31:0] BASE_ADDR         = 32'h8100_7000,
    parameter [31:0] CLK_FREQ          = 32'd100_000_000,            // 100MHz
    parameter [31:0] DEFAULT_PRESCALER = `TIMER_DEFAULT_PRESCALER,
    parameter [31:0] MAX_COMPARE       = `TIMER_MAX_COMPARE
)(
    input  wire                     clk,
    input  wire                     resetn,

    input  wire                     mem_valid,
    input  wire                     mem_instr,
    output reg                      mem_ready,
    input  wire [31:0]              mem_addr,
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [31:0]              mem_wdata,
    /* verilator lint_on  UNUSEDSIGNAL */
    input  wire [3:0]               mem_wstrb,
    output reg  [31:0]              mem_rdata,

    output reg                      timer_irq,
    input  wire                     eoi
);

    reg timer_irq_next;

    reg [31:0] ctrl_reg;
    reg [31:0] compare_reg;
    reg [31:0] prescaler_reg;
    reg [31:0] counter;
    reg [31:0] prescaler_cnt;

    reg        compare_match;
    reg        overflow;
    reg        prescaler_overflow;

    localparam [31:0]
        RW_TIMER_CTRL      = BASE_ADDR + 32'h00,
        RW_TIMER_COMPARE   = BASE_ADDR + 32'h04,
        RO_TIMER_CURRENT   = BASE_ADDR + 32'h08,
        RW_TIMER_PRESCALER = BASE_ADDR + 32'h0C,
        RS_TIMER_STATUS    = BASE_ADDR + 32'h10;

    wire ctrl_timer_en         = ctrl_reg[0];
    wire ctrl_auto_reload      = ctrl_reg[1];        // 1=auto reload, 0=continue inc
    wire ctrl_compare_irq_en   = ctrl_reg[2];
    wire ctrl_overflow_irq_en  = ctrl_reg[3];
    wire ctrl_prescaler_en     = ctrl_reg[4];        // 1=enable prescaler
    wire ctrl_oneshot          = ctrl_reg[5];        // 1=stop after match
    wire ctrl_prescaler_irq_en = ctrl_reg[6];        // 1=enabel irq when prescaler overflow

    wire status_compare_match  = compare_match;      // 1=match
    wire status_overflow       = overflow;           // 1=overflow
    wire status_prescaler_ovf  = prescaler_overflow; // 1=prescaler match
    wire status_timer_running  = ctrl_timer_en;      // 1=timer is running

    wire [31:0] status_wire = {
        28'd0,
        status_timer_running,
        status_prescaler_ovf,
        status_overflow,
        status_compare_match
    };

    wire [31:0] wmask = { {8{mem_wstrb[3]}}, {8{mem_wstrb[2]}}, {8{mem_wstrb[1]}}, {8{mem_wstrb[0]}} };
    wire [31:0] wdata = mem_wdata & wmask;

    wire prescaler_tick = (ctrl_timer_en && ctrl_prescaler_en && (prescaler_cnt == 0));
    wire timer_should_inc = ctrl_timer_en && (!ctrl_prescaler_en || prescaler_tick);

    always @(posedge clk) begin: PRESCALER_COUNTER
        if (!resetn) begin
            prescaler_cnt      <= 0;
            prescaler_overflow <= 0;
        end else if (ctrl_timer_en && ctrl_prescaler_en) begin
            if (prescaler_cnt == 0) begin
                prescaler_cnt      <= prescaler_reg;
                prescaler_overflow <= 1'b1;
            end else begin
                prescaler_cnt      <= prescaler_cnt - 1;
                prescaler_overflow <= 1'b0;
            end
        end else begin
            prescaler_cnt      <= 0;
            prescaler_overflow <= 1'b0;
        end
    end

    always @(posedge clk) begin: TIMER_COUNTER
        if (!resetn) begin
            counter       <= 0;
            compare_match <= 0;
            overflow      <= 0;
        end else if (timer_should_inc) begin
            if (counter == compare_reg) begin
                compare_match <= 1'b1;

                if (ctrl_auto_reload) begin
                    counter <= 0;
                end else begin
                    counter <= counter + 1;
                end

                if (ctrl_oneshot) begin
                    ctrl_reg[0] <= 1'b0;
                end
            end else begin
                counter       <= counter + 1;
                compare_match <= 1'b0;
            end

            if (counter == MAX_COMPARE) begin
                overflow <= 1'b1;
                counter  <= 0;
            end else begin
                overflow <= 1'b0;
            end
        end else begin
            compare_match <= 1'b0;
            overflow      <= 1'b0;
        end
    end

    always @(*) begin: IRQ_GEN
        timer_irq_next = 0;
        if (ctrl_timer_en) begin
            if ((ctrl_compare_irq_en && compare_match) ||
                (ctrl_overflow_irq_en && overflow)     ||
                (ctrl_prescaler_irq_en && prescaler_overflow))
            begin: IRQ_CONDITIONS
                timer_irq_next = 1'b1;
            end
        end
    end

    always @(posedge clk) begin
        if (!resetn)
            timer_irq <= 0;
        else
            timer_irq <= eoi ? 0 : timer_irq_next;
    end

    always @(posedge clk) begin
        if (!resetn) begin
            mem_ready <= 0;
        end
        mem_ready <= mem_valid && !mem_instr;
    end

    always @(posedge clk) begin: MMIO_READ
        if (!resetn) begin
            mem_rdata <= 0;
        end else if (mem_valid && (!mem_instr) && mem_wstrb == 0) begin
            case (mem_addr)
                RW_TIMER_CTRL:      mem_rdata <= ctrl_reg;
                RW_TIMER_COMPARE:   mem_rdata <= compare_reg;
                RO_TIMER_CURRENT:   mem_rdata <= counter;
                RW_TIMER_PRESCALER: mem_rdata <= prescaler_reg;
                RS_TIMER_STATUS:    mem_rdata <= status_wire;
                default:            mem_rdata <= 0;
            endcase
        end else begin
            mem_rdata <= 0;
        end
    end

    always @(posedge clk) begin: MMIO_WRITE
        if (!resetn) begin
            ctrl_reg      <= 0;
            compare_reg   <= MAX_COMPARE;
            prescaler_reg <= (CLK_FREQ / DEFAULT_PRESCALER) - 1;
        end else begin
            if (mem_valid && (!mem_instr) && mem_wstrb != 0) begin
                case(mem_addr)
                    RW_TIMER_CTRL: begin
                        ctrl_reg <= wdata;
                    end
                    RW_TIMER_COMPARE: begin
                        /* verilator lint_off CMPCONST */
                        compare_reg <= (wdata > MAX_COMPARE) ? MAX_COMPARE : wdata;
                        /* verilator lint_on  CMPCONST */
                    end
                    RW_TIMER_PRESCALER: begin
                        prescaler_reg <= wdata;
                    end
                    RS_TIMER_STATUS: begin
                        if (wdata[0]) compare_match <= 0;
                        if (wdata[1]) overflow <= 0;
                        if (wdata[2]) prescaler_overflow <= 0;
                        if (wdata[3]) ctrl_reg[0] <= 0;
                    end
                    default: ;
                endcase
            end
        end
    end

endmodule

`endif
