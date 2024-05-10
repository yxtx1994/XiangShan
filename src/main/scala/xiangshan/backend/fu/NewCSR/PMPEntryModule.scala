package xiangshan.backend.fu.NewCSR

import chisel3._
import chisel3.util._
import xiangshan._
import org.chipsalliance.cde.config.Parameters
import xiangshan.HasPMParameters

trait PMPConst extends HasPMParameters {
  val PMPOffBits = 2 // minimal 4bytes
  val CoarserGrain: Boolean = PlatformGrain > PMPOffBits
}

abstract class PMPBundle(implicit val p: Parameters) extends Bundle with PMPConst
abstract class PMPModule(implicit val p: Parameters) extends Module with PMPConst

class PMPEntryHandleModule(implicit p: Parameters) extends PMPModule {
  val io = IO(new PMPEntryHandleIOBundle)

  val pmpCfg   = io.in.pmpCfg
  val pmpAddr  = io.in.pmpAddr

  val wen   = io.in.wen
  val addr  = io.in.addr
  val wdata = io.in.wdata

  val pmpCfgs  = WireInit(pmpCfg).asTypeOf(Vec(p(PMParameKey).NumPMP, new PMPCfgBundle))
  val pmpAddrs = WireInit(pmpAddr).asTypeOf(Vec(p(PMParameKey).NumPMP, new PMPAddrBundle))
  val pmpMask  = WireInit(VecInit(Seq.fill(p(PMParameKey).NumPMP)(0.U(PMPAddrBits.W))))

  val pmpEntry = Reg(Vec(p(PMParameKey).NumPMP, new PMPEntry))
  for (i <- pmpEntry.indices) {
    pmpEntry(i).gen(pmpCfgs(i), pmpAddrs(i), pmpMask(i))
  }

  // write pmpCfg
  val cfgVec = WireInit(VecInit(Seq.fill(8)(0.U.asTypeOf(new PMPCfgBundle))))
  for (i <- 0 until (p(PMParameKey).NumPMP/8+1) by 2) {
    when (addr === (0x3A0 + i).U) {
      for (j <- cfgVec.indices) {
        val cfgOldTmp = pmpEntry(8*i/2+j).cfg
        val cfgNewTmp = wdata(8*(j+1)-1, 8*j).asTypeOf(new PMPCfgBundle)
        cfgVec(j) := cfgOldTmp
        when (!cfgOldTmp.L.asBool) {
          cfgVec(j) := cfgNewTmp
          cfgVec(j).W := cfgNewTmp.W.asBool && cfgNewTmp.R.asBool
          if (CoarserGrain) {
            cfgVec(j).A := Cat(cfgNewTmp.A.asUInt(1), cfgNewTmp.A.asUInt.orR)
          }
          when (PMPCfgAField.isNa4OrNapot(cfgVec(j))) {
            pmpEntry(8*i/2+j).mask := pmpEntry(8*i/2+j).matchMask(cfgVec(j), pmpEntry(8*i/2+j).addr)
          }
        }
      }
    }
  }

  io.out.pmpCfgWData := cfgVec.asUInt

  // write pmpAddr
  val pmpAddrW = WireInit(VecInit(Seq.fill(p(PMParameKey).NumPMP)(0.U(64.W))))
  for (i <- 0 until p(PMParameKey).NumPMP) {
    if (i != (p(PMParameKey).NumPMP - 1)) {
      pmpAddrW(i) := pmpEntry(i).writeAddr(pmpEntry(i + 1).cfg, pmpEntry(i).addr, pmpEntry(i).mask)
    } else {
      pmpAddrW(i) := pmpEntry(i).writeAddr(pmpEntry(i).addr, pmpEntry(i).mask)
    }
  }

  // read pmpAddr
  val pmpAddrR = Wire(Vec(p(PMParameKey).NumPMP, UInt(64.W)))
  for (i <- 0 until p(PMParameKey).NumPMP) {
    pmpAddrR(i) := pmpEntry(i).readAddr(pmpEntry(i).cfg, pmpEntry(i).addr.ADDRESS.asUInt)
  }

  io.out.pmpAddrWData := pmpAddrW
  io.out.pmpAddrRData := pmpAddrR

}

class PMPEntryHandleIOBundle(implicit p: Parameters) extends PMPBundle {
  val in = Input(new Bundle {
    val wen   = Bool()
    val addr  = UInt(12.W)
    val wdata = UInt(64.W)
    val pmpCfg  = UInt((NumPMP/8*PMXLEN).W)
    val pmpAddr = UInt((NumPMP*64).W)
  })

  val out = Output(new Bundle {
    val pmpCfgWData  = UInt(PMXLEN.W)
    val pmpAddrRData = Vec(NumPMP, UInt(64.W))
    val pmpAddrWData = Vec(NumPMP, UInt(64.W))
  })
}

trait PMPReadWrite extends PMPConst {
  def matchMask(cfg: PMPCfgBundle, paddr: PMPAddrBundle): UInt = {
    val matchMaskCAddr = Cat(paddr.ADDRESS.asUInt, cfg.A.asUInt(0)) | (((1 << PlatformGrain) - 1) >> PMPOffBits).U((paddr.getWidth + 1).W)
    Cat(matchMaskCAddr & (~(matchMaskCAddr + 1.U)).asUInt, ((1 << PMPOffBits) - 1).U(PMPOffBits.W))
  }

  /**
   * In general, the PMP grain is 2**{G+2} bytes. when G >= 1, na4 is not selectable.
   * When G >= 2 and cfg.A(1) is set(then the mode is napot), the bits addr(G-2, 0) read as zeros.
   * When G >= 1 and cfg.A(1) is clear(the mode is off or tor), the addr(G-1, 0) read as zeros.
   * The low Offbits is dropped
   * @param cfg
   * @param addr
   * @return
   */
  def readAddr(cfg: PMPCfgBundle, addr: UInt): UInt = {
    val G = PlatformGrain - PMPOffBits
    require(G >= 0)
    if (G == 0) {
      addr
    } else if (G >= 2) {
      Mux(PMPCfgAField.isNa4OrNapot(cfg), setLowBits(addr, G-1), clearLowBits(addr, G))
    } else { // G is 1
      Mux(PMPCfgAField.isOffOrTor(cfg), clearLowBits(addr, G), addr)
    }
  }

  def setLowBits(data: UInt, num: Int): UInt = {
    require(num >= 0)
    data | ((1 << num)-1).U
  }

  /**
   * mask the data's low num bits (lsb)
   * @param data
   * @param num
   * @return
   */
  def clearLowBits(data: UInt, num: Int): UInt = {
    require(num >= 0)
    // use Cat instead of & with mask to avoid "Signal Width" problem
    if (num == 0) {
      data
    } else {
      Cat(data(data.getWidth - 1, num), 0.U(num.W))
    }
  }

}

/**
 * PMPEntry for outside pmp copies with one more elements mask to help napot match
 * TODO: make mask an element, not an method, for timing opt
 */
class PMPEntry(implicit p: Parameters) extends PMPBundle with PMPReadWrite {
  val cfg  = new PMPCfgBundle
  val addr = new PMPAddrBundle
  val mask = UInt(PMPAddrBits.W) // help to match in napot

  def gen(cfg: PMPCfgBundle, addr: PMPAddrBundle, mask: UInt) = {
    require(addr.ADDRESS.getWidth == this.addr.ADDRESS.getWidth)
    this.cfg := cfg
    this.addr.ADDRESS := addr.ADDRESS
    this.mask := mask
  }

  // generate match mask to help match in napot mode
  def matchMask(paddr: PMPAddrBundle): UInt = {
    matchMask(cfg, paddr)
  }

  def writeAddr(next: PMPCfgBundle, paddr: PMPAddrBundle, mask: UInt): UInt = {
    val addrLocked: Bool = PMPCfgLField.addrLocked(cfg, next)
    mask := Mux(!addrLocked, matchMask(paddr), mask)
    Mux(!addrLocked, paddr.ADDRESS.asUInt, addr.ADDRESS.asUInt)
  }

  def writeAddr(paddr: PMPAddrBundle, mask: UInt): UInt = {
    val addrLocked: Bool = PMPCfgLField.addrLocked(cfg)
    mask := Mux(!addrLocked, matchMask(paddr), mask)
    Mux(!addrLocked, paddr.ADDRESS.asUInt, addr.ADDRESS.asUInt)
  }
}