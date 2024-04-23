package xiangshan.backend.fu.NewCSR

import chisel3._
import chisel3.util._
import xiangshan.ExceptionNO
import xiangshan.backend.fu.NewCSR.CSRBundles.{CauseBundle, PrivState, XtvecBundle}
import xiangshan.backend.fu.NewCSR.CSRDefines.{XtvecMode, PrivMode}
import xiangshan.backend.fu.util.CSRConst


class InterruptFilter extends Module {
  val io = IO(new InterruptFilterIO)

  val mstatusMIE = io.in.mstatusMIE
  val sstatusSIE = io.in.sstatusSIE
  val vsstatusSIE = io.in.vsstatusSIE
  val mip = io.in.mip
  val mie = io.in.mie
  val mideleg = io.in.mideleg
  val privState = io.in.privState
  val hip = io.in.hip
  val hie = io.in.hie
  val hideleg = io.in.hideleg
  val iprios = io.in.iprios

  // def mtopiIsNotZero: Bool = privState.isModeM && (mip.asUInt.asBool & mie.asUInt.asBool)
  // def stopiIsNotZero: Bool = privState.isModeHS && (mip.asUInt.asBool & mie.asUInt.asBool || hip.asUInt.asBool & hie.asUInt.asBool) && (hideleg.asUInt === 0.U)

  // todo: refactor this shit
  def mtopiIsNotZero: Bool = privState.isModeM && ((mip.MSIP.asBool && mie.MSIE.asBool) || (mip.MTIP.asBool && mie.MTIE.asBool) || (mip.MEIP.asBool && mie.MEIE.asBool))
  def stopiIsNotZero: Bool = privState.isModeHS && (((mip.SSIP.asBool && mie.SSIE.asBool) || (mip.STIP.asBool && mie.STIE.asBool) || (mip.SEIP.asBool && mie.SEIE.asBool)) || ((hip.VSSIP.asBool && hie.VSSIE.asBool) || (hip.VSTIP.asBool && hie.VSTIE.asBool) || (hip.VSEIP.asBool && hie.VSEIE.asBool))) && (hideleg.asUInt === 0.U)


  // def trapToM: Bool = mtopiIsNotZero && (privState.isModeM && io.in.mstatusMIE || !privState.isModeM) && (mip.asUInt.asBool && mie.asUInt.asBool) && (mideleg.asUInt === 0.U)

  def iprioIsZero: Bool = !iprios.orR

  val prio0 = iprios(7, 0)
  val highPrio = Wire(UInt(8.W))
  highPrio := prio0

  val defaultPrio = Seq( // todo:

  )

  def iprioHighPrio = (1 to 63).map(i =>
    highPrio := Mux(iprios(8*i-1, 8*i) > highPrio, highPrio, iprios(8*i-1, 8*i))
  )

  val iprio = Mux1H(
    Seq(
      mtopiIsNotZero && (highPrio >= 1.U && highPrio <= 255.U),
      mtopiIsNotZero && (highPrio > 255.U || iprioIsZero /* && iid.defaultPrio < machine external interrupt || iid.defaultPrio < supervisor external interrupt*/) ,
      mtopiIsNotZero && (iprioIsZero /* && iid.defaultPrio > machine external interrupt || iid.defaultPrio > supervisor external interrupt*/ ),
    ),
    Seq(
      highPrio,
      255.U,
      0.U,
    )
  )

  // update mtopi
  io.out.mtopi.valid := mtopiIsNotZero
  io.out.mtopi.bits.IID := 0.U
  io.out.mtopi.bits.IPRIO := iprio
  // upadte stopi
  io.out.stopi.valid := stopiIsNotZero
  io.out.stopi.bits.IID := 0.U
  io.out.stopi.bits.IPRIO := iprio

  // update vstopi
  io.out.vstopi.valid := false.B
  io.out.vstopi.bits.IID := 0.U
  io.out.vstopi.bits.IPRIO := 0.U


  val ideleg = mideleg.asUInt & mip.asUInt
    def priviledgeEnableDetect(x: Bool): Bool = Mux(x, ((privState.PRVM === PrivMode.S) && sstatusSIE) || (privState.PRVM < PrivMode.S),
    ((privState.PRVM === PrivMode.M) && mstatusMIE) || (privState.PRVM < PrivMode.M))

  val intrVecEnable = Wire(Vec(12, Bool()))
  intrVecEnable.zip(ideleg.asBools).map{ case(x, y) => x := priviledgeEnableDetect(y)} // todo: !disableInterrupt

  val intrVec = mie.asUInt(11, 0) & mip.asUInt & intrVecEnable.asUInt // todo: Cat(debugIntr && !debugMode, mie.rdata.asUInt(11, 0) & mip.rdata.asUInt & intrVecEnable.asUInt)

  io.out.interruptVec.valid := intrVec.orR
  io.out.interruptVec.bits := intrVec
}

class InterruptFilterIO extends Bundle {
  val in = Input(new Bundle {
    val mstatusMIE  = Bool()
    val sstatusSIE  = Bool()
    val vsstatusSIE = Bool()
    val mip = new MipBundle
    val mie = new MieBundle
    val mideleg = new MidelegBundle
    val privState = new PrivState
    val hip = new HipBundle
    val hie = new HieBundle
    val hideleg = new HidelegBundle
    val iprios = UInt((64*8).W)
  })

  val out = Output(new Bundle {
    val interruptVec = ValidIO(UInt(64.W))
    val mtopi = ValidIO(new TopIBundle)
    val stopi = ValidIO(new TopIBundle)
    val vstopi = ValidIO(new TopIBundle)
  })
}
