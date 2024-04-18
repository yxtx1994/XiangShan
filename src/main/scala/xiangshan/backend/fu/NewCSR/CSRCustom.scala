package xiangshan.backend.fu.NewCSR

import chisel3._
import chisel3.util._
import xiangshan.backend.fu.NewCSR.CSRDefines.{
  CSRRWField => RW,
}
import scala.collection.immutable.SeqMap

trait CSRCustom { self: NewCSR =>
  val spfctl = Module(new CSRModule("spfctl", new SpfctlBundle))
    // .setAddr()

  // ICache
  val sfetchctl = Module(new CSRModule("sfetchctl"))

  val customCSRMods = Seq(
    spfctl,
  )

  val customCSRMap = SeqMap.from(
    customCSRMods.map(csr => (csr.addr -> (csr.w -> csr.rdata.asInstanceOf[CSRBundle].asUInt))).iterator
  )

  val customCSROutMap: SeqMap[Int, UInt] = SeqMap.from(
    customCSRMods.map(csr => (csr.addr -> csr.regOut.asInstanceOf[CSRBundle].asUInt)).iterator
  )
}

class SpfctlBundle extends CSRBundle {
  // turn off L2 BOP, turn on L1 SMS by default
  val L2_PF_STORE_ONLY        = RW(    17).withReset(false.B) // L2 pf store only
  val L1D_PF_ENABLE_STRIDE    = RW(    16).withReset(true.B)  // L1D prefetch enable stride
  val L1D_PF_ACTIVE_STRIDE    = RW(15, 10).withReset(30.U)    // L1D prefetch active page stride
  val L1D_PF_ACTIVE_THRESHOLD = RW( 9,  6).withReset(12.U)    // L1D prefetch active page threshold
  val L1D_PF_ENABLE_PHT       = RW(     5).withReset(true.B)  // L1D prefetch enable pht
  val L1D_PF_ENABLE_AGT       = RW(     4).withReset(true.B)  // L1D prefetch enable agt
  val L1D_PF_TRAIN_ON_HIT     = RW(     3).withReset(false.B) // L1D train prefetch on hit
  val L1D_PF_ENABLE           = RW(     2).withReset(true.B)  // L1D Cache Prefetcher Enable
  val L2_PF_ENABLE            = RW(     1).withReset(true.B)  // L2  Cache Prefetcher Enable
  val L1I_PF_ENABLE           = RW(     0).withReset(true.B)  // L1I Cache Prefetcher Enable
}