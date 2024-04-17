package device

import chisel3._
import chisel3.util._
import chisel3.experimental._

class IMSIC(
  NumIRFiles: Int = 7,
  NumHart: Int = 2,
  XLEN: Int = 64,
  NumIRSrc: Int = 12,
) extends Module {
  // has default clock and reset
  val i = IO(Input(new Bundle {
    val setIpNumValidVec2 = Vec(NumHart, Vec(NumIRFiles, Bool()))
    val setIpNum          = ValidIO(UInt(log2Up(NumIRSrc).W))
    val hartId            = UInt(log2Up(NumHart).W)
    val csr = new Bundle {
      val addr = ValidIO(new Bundle {
        val addr = UInt(12.W)
        val prvm = UInt(2.W)
        val v    = UInt(1.W)
      })
      val vgein = UInt(6.W)
      val mClaim  = Bool()
      val sClaim  = Bool()
      val vsClaim = Bool()
      val wdata = ValidIO(new Bundle{
        val data  = UInt(XLEN.W)
      })
    }
  }))
  val o = IO(Output(new Bundle {
    val csr = new Bundle {
      val rdata = ValidIO(new Bundle {
        val rdata   = UInt(XLEN.W)
        val illegal = Bool()
      })
    }
    val mtopei  = ValidIO(UInt(32.W))
    val stopei  = ValidIO(UInt(32.W))
    val vstopei = ValidIO(UInt(32.W))
  }))

  val imsicTop = Module(new imsic_csr_top)

  imsicTop.csr_clk         := clock
  imsicTop.csr_rstn        := reset
  imsicTop.i.setipnum_vld  := Cat(i.setIpNumValidVec2.flatten.reverse)
  imsicTop.i.setipnum      := i.setIpNum.bits
  imsicTop.i.hart_id       := i.hartId
  imsicTop.i.csr.addr_vld  := i.csr.addr.valid
  imsicTop.i.csr.addr      := i.csr.addr.bits.addr
  imsicTop.i.csr.priv_lvl  := i.csr.addr.bits.prvm
  imsicTop.i.csr.v         := i.csr.addr.bits.v
  imsicTop.i.csr.vgein     := i.csr.vgein
  imsicTop.i.csr.claim     := Cat(i.csr.vsClaim, i.csr.sClaim, i.csr.mClaim)
  imsicTop.i.csr.wdata_vld := i.csr.wdata.valid
  imsicTop.i.csr.wdata     := i.csr.wdata.bits.data

  o.csr.rdata.valid        := imsicTop.o.csr.rdata_vld
  o.csr.rdata.bits.rdata   := imsicTop.o.csr.rdata
  o.csr.rdata.bits.illegal := imsicTop.o.csr.illegal
  o.mtopei.valid           := imsicTop.o.csr.irq(0)
  o.stopei.valid           := imsicTop.o.csr.irq(1)
  o.vstopei.valid          := imsicTop.o.csr.irq(2)
  o.mtopei.bits            := imsicTop.o.csr.mtopei
  o.stopei.bits            := imsicTop.o.csr.stopei
  o.vstopei.bits           := imsicTop.o.csr.vstopei
}

class imsic_csr_top(
  NumIRFiles: Int = 7,
  NumIRSrc: Int = 12,
  NumHart: Int = 2,
  XLEN: Int = 64,
) extends BlackBox(Map(
  "NR_INTP_FILES" -> NumIRFiles,
  "NR_HARTS"      -> NumHart,
  "XLEN"          -> XLEN,
  "NR_SRC"        -> NumIRSrc,
)) with HasBlackBoxResource {
  private val HART_ID_WIDTH = log2Up(NumHart)
  private val NR_SRC_WIDTH = log2Up(NumIRSrc)

  val csr_clk = IO(Input(Clock()))
  val csr_rstn = IO(Input(Reset()))

  val i = IO(Input(new Bundle {
    val setipnum_vld = UInt((NumHart * NumIRFiles).W)
    val setipnum = UInt(NR_SRC_WIDTH.W)
    val hart_id = UInt(HART_ID_WIDTH.W)
    val csr = new Bundle {
      val addr_vld = Bool()
      val addr = UInt(12.W)
      val priv_lvl = UInt(2.W)
      val v = UInt(1.W)
      val vgein = UInt(6.W)
      val claim = UInt(3.W)
      val wdata_vld = Bool()
      val wdata = UInt(64.W)
    }
  }))

  val o = IO(Output(new Bundle{
    val csr = new Bundle {
      val rdata_vld = Bool()
      val rdata = UInt(XLEN.W)
      val illegal = Bool()
      val irq = UInt(3.W)
      val mtopei = UInt(32.W)
      val stopei = UInt(32.W)
      val vstopei = UInt(32.W)
    }
  }))

  addResource("/vsrc/imsic/imsic_csr_top.v")
}

