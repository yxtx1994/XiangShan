package xiangshan.backend.fu.NewCSR

import chisel3._
import chisel3.util._
import CSRConfig._
import xiangshan.backend.fu.NewCSR.CSRDefines._
import xiangshan.backend.fu.NewCSR.CSRDefines.{
  CSRWARLField => WARL,
  CSRRWField => RW,
  CSRROField => RO,
}
import xiangshan.backend.fu.NewCSR.CSRFunc._

import scala.collection.immutable.SeqMap

trait CSRDebugTrigger { self: NewCSR =>
  val dcsr = Module(new CSRModule("dcsr", new DcsrBundle))
    .setAddr(0x7B0)

  val dpc = Module(new CSRModule("dpc", new Dpc))
    .setAddr(0x7B1)

  val dscratch0 = Module(new CSRModule("dscratch0"))
    .setAddr(0x7B2)

  val dscratch1 = Module(new CSRModule("dscratch1"))
    .setAddr(0x7B3)

  val debugCSRMods = Seq(
    dcsr,
    dpc,
    dscratch0,
    dscratch1,
  )

  val debugCSRMap = SeqMap.from(
    debugCSRMods.map(csr => (csr.addr -> (csr.w -> csr.rdata.asInstanceOf[CSRBundle].asUInt))).iterator
  )

  val debugCSROutMap: SeqMap[Int, UInt] = SeqMap.from(
    debugCSRMods.map(csr => (csr.addr -> csr.regOut.asInstanceOf[CSRBundle].asUInt)).iterator
  )
}

class DcsrBundle extends CSRBundle {

  val DEBUGVER  = DebugverMode(31, 28).withReset(DebugverMode.Spec) // Debug implementation as it described in 0.13 draft // todo
  val PAD1      =   RO(27, 18).withReset(0.U)
  val EBREAKVS  = WARL(    17, wNoFilter).withReset(0.U)
  val EBREAKVU  = WARL(    16, wNoFilter).withReset(0.U)
  val EBREAKM   =   RW(    15).withReset(0.U)
  val PAD0      =   RO(    14).withReset(0.U) // ebreakh has been removed
  val EBREAKS   = WARL(    13, wNoFilter).withReset(0.U)
  val EBREAKU   = WARL(    12, wNoFilter).withReset(0.U)
  val STEPIE    = WARL(    11, wNoFilter).withReset(0.U)
  val STOPCOUNT = WARL(    10, wNoFilter) // Stop count updating has not been supported
  val STOPTIME  = WARL(     9, wNoFilter) // Stop time updating has not been supported
  val CAUSE     =   RO(  8, 6).withReset(0.U)
  val V         = WARL(     5, wNoFilter).withReset(0.U)
  val MPRVEN    = WARL(     4, wNoFilter) // Whether use mstatus.perven as mprven
  val NMIP      =   RO(     3).withReset(0.U)
  val STEP      =   RW(     2).withReset(0.U)
  val PRV       = PrivMode(1, 0).withReset(PrivMode.M)
}

class Dpc extends CSRBundle {

  val ALL = RW(63, 1)
}
