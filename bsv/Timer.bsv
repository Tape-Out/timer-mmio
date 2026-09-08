package Timer;

import ConfigReg::*;
import Vector::*;
import RegIf::*;
import TimerRegs::*;

// 本包不认识任何总线：对外只给中立的 RegIf，接哪种总线由 wrap 或装配决定。
typedef struct {
  Bool capture;
} TimerCfg;

interface TimerPins#(numeric type channels);
  (* always_ready, always_enabled, prefix = "" *)
  method Action capt((* port = "capt_in" *) Bit#(channels) v);
endinterface

interface TimerIfc#(numeric type aw, numeric type dw, numeric type channels);
  interface RegIf#(aw, dw) regs;
  interface TimerPins#(channels) pins;
  (* always_ready *) method Bool irq;
endinterface

module mkTimer#(TimerCfg cfg)(TimerIfc#(aw, dw, channels))
    provisos (Mul#(TDiv#(dw, 8), 8, dw), Add#(_a, 8, aw), Add#(_b, 32, dw),
              Add#(_c, 16, dw), Add#(_d, 8, dw), Add#(_e, 1, dw),
              Add#(_f, channels, 8), Log#(TAdd#(channels, 1), _g));

  TimerRegsIfc#(aw, dw, channels) r <- mkTimerRegs(
      TimerRegsCfg { capture: cfg.capture });

  // 捕获要读计数器、计数器规则要读配置寄存器、总线又要写它们，
  // 三者首尾相接成环。ConfigReg 让读恒取旧值，环就断了（D39）。
  Reg#(Bit#(32)) cnt  <- mkConfigReg(0);
  Reg#(Bit#(16)) div  <- mkReg(0);
  // 上一拍的计数值。比较按「到达」而不是按电平：比较值复位是 0、
  // 计数器复位也是 0，按电平的话一使能所有通道当场全部比中。
  Reg#(Bit#(32)) prevCnt <- mkConfigReg(0);

  rule tick;
    r.cnt_in(cnt);
    if (r.ctrl_rst == 1) begin
      cnt <= 0;
      div <= 0;
    end else if (r.ctrl_en == 1) begin
      if (div >= r.presc) begin
        div <= 0;
        cnt <= cnt + 1;
      end else
        div <= div + 1;
    end
  endrule

  // 比较命中置位，写 1 清除由寄存器组代管
  rule cmpHit (r.ctrl_en == 1);
    Bit#(8) hit = 0;
    for (Integer i = 0; i < valueOf(channels); i = i + 1)
      if (cnt == r.cmp[i] && cnt != prevCnt) hit[i] = 1;
    r.ista_set(hit);
    prevCnt <= cnt;
  endrule

  Wire#(Bit#(channels)) capt     <- mkBypassWire;
  Reg#(Bit#(channels))  captPrev <- mkReg(0);

  if (cfg.capture) begin
    // 一拍最多锁一路：Action 方法在同一条规则里只能调一次，
    // 所以先在展开时选出优先级最高的那一路，再调一次。
    rule doCapture;
      captPrev <= capt;
      Bit#(channels) rise = capt & ~captPrev;
      Bit#(TLog#(TAdd#(channels, 1))) idx = 0;
      Bool any = False;
      for (Integer i = 0; i < valueOf(channels); i = i + 1)
        if (!any && rise[i] == 1) begin
          idx = fromInteger(i);
          any = True;
        end
      if (any) r.capt_in(idx, cnt);
    endrule
  end

  interface regs = r.regs;
  interface TimerPins pins;
    method Action capt(Bit#(channels) v); capt._write(v); endmethod
  endinterface
  method Bool irq = (r.ista & r.ien) != 0;
endmodule

endpackage
