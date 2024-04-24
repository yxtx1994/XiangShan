/***************************************************************************************
* Copyright (c) 2020-2021 Institute of Computing Technology, Chinese Academy of Sciences
* Copyright (c) 2020-2021 Peng Cheng Laboratory
*
* XiangShan is licensed under Mulan PSL v2.
* You can use this software according to the terms and conditions of the Mulan PSL v2.
* You may obtain a copy of Mulan PSL v2 at:
*          http://license.coscl.org.cn/MulanPSL2
*
* THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
* EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
* MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
*
* See the Mulan PSL v2 for more details.
***************************************************************************************/

package top

import chisel3._
import chisel3.util._
import xiangshan._
import utils._
import huancun.{HCCacheParameters, PrefetchRecv}
import utility._
import system._
import device._
import org.chipsalliance.cde.config._
import freechips.rocketchip.amba.axi4._
import freechips.rocketchip.devices.debug._
import freechips.rocketchip.devices.tilelink._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.interrupts._
import freechips.rocketchip.jtag._
import freechips.rocketchip.tilelink._
import device.DebugModule
import coupledL2.tl2chi.PortIO

class XSNocTop()(implicit p: Parameters) extends LazyModule
  with BindingScope with HasSoCParameter
{
  val idBits = 14
  lazy val dts = DTS(bindingTree)
  lazy val json = JSON(bindingTree)

  ResourceBinding {
    val width = ResourceInt(2)
    val model = "freechips,rocketchip-unknown"
    Resource(ResourceAnchors.root, "model").bind(ResourceString(model))
    Resource(ResourceAnchors.root, "compat").bind(ResourceString(model + "-dev"))
    Resource(ResourceAnchors.soc, "compat").bind(ResourceString(model + "-soc"))
    Resource(ResourceAnchors.root, "width").bind(width)
    Resource(ResourceAnchors.soc, "width").bind(width)
    Resource(ResourceAnchors.cpus, "width").bind(ResourceInt(1))
    def bindManagers(xbar: TLNexusNode) = {
      ManagerUnification(xbar.edges.in.head.manager.managers).foreach{ manager =>
        manager.resources.foreach(r => r.bind(manager.toResource))
      }
    }
    // bindManagers(misc.l3_xbar.asInstanceOf[TLNexusNode])
    // bindManagers(misc.peripheralXbar.asInstanceOf[TLNexusNode])
  }

  println(s"FPGASoC cores: $NumCores")

  val core_with_l2 = tiles.map(coreParams =>
    LazyModule(new XSTile()(p.alterPartial({
      case XSCoreParamsKey => coreParams
    })))
  )

  val l3_xbar = TLXbar()
  val clint_xbar = TLXbar()
  val debugModule_xbar = TLXbar()

  val clintErrorDevice = LazyModule(new TLError(
    params = DevNullParams(
      address = AddressSet(0x0, 0xfffffffffL).subtract(AddressSet(0x38000000L, 0xffff)),
      maxAtomic = 8,
      maxTransfer = 64),
    beatBytes = 8
  ))
  val clint = LazyModule(new CLINT(CLINTParams(0x38000000L), 8))
  val clintNode = AXI4MasterNode(Seq(AXI4MasterPortParameters(
    Seq(AXI4MasterParameters(
      name = "clint",
      id = IdRange(0, 1 << idBits)
    ))
  )))
  clint.node := TLBuffer() := clint_xbar
  clintErrorDevice.node := TLBuffer() := clint_xbar
  clint_xbar :=
    TLFIFOFixer() :=
    TLWidthWidget(8) :=
    AXI4ToTL() :=
    AXI4UserYanker(Some(1)) :=
    AXI4Fragmenter() :=
    AXI4Buffer() :=
    AXI4Buffer() :=
    AXI4IdIndexer(1) :=
    clintNode
  val clintIO = InModuleBody {
    clintNode.makeIOs()
  }

  val debugModuleErrorDevice = LazyModule(new TLError(
    params = DevNullParams(
      address = AddressSet(0x0, 0xfffffffffL).subtract(AddressSet(p(DebugModuleKey).get.baseAddress, 0xfff)),
      maxAtomic = 8,
      maxTransfer = 64),
    beatBytes = 8
  ))
  val debugModule = LazyModule(new DebugModule(NumCores)(p))
  val debugModuleNode = AXI4MasterNode(Seq(AXI4MasterPortParameters(
    Seq(AXI4MasterParameters(
      name = "debugModule",
      id = IdRange(0, 1 << idBits)
    ))
  )))
  debugModule.debug.node := TLBuffer() := debugModule_xbar
  debugModuleErrorDevice.node := TLBuffer() := debugModule_xbar
  debugModule_xbar :=
    TLFIFOFixer() :=
    TLWidthWidget(8) :=
    AXI4ToTL() :=
    AXI4UserYanker(Some(1)) :=
    AXI4Fragmenter() :=
    AXI4Buffer() :=
    AXI4Buffer() :=
    AXI4IdIndexer(1) :=
    debugModuleNode
  debugModule.debug.dmInner.dmInner.sb2tlOpt.foreach { sb2tl  =>
    l3_xbar := TLBuffer() := sb2tl.node
  }
  val debugModuleIO = InModuleBody {
    debugModuleNode.makeIOs()
  }
  val errorDevice = LazyModule(new TLError(
    params = DevNullParams(
      address = Seq(AddressSet(0x0, 0x7fffffffL)),
      maxAtomic = 8,
      maxTransfer = 64),
    beatBytes = L3InnerBusWidth / 8
  ))
  errorDevice.node := l3_xbar

  // 36-bit physical address
  val memRange = AddressSet(0x00000000L, 0xfffffffffL).subtract(AddressSet(0x0L, 0x7fffffffL))
  val memAXI4SlaveNode = AXI4SlaveNode(Seq(
    AXI4SlavePortParameters(
      slaves = Seq(
        AXI4SlaveParameters(
          address = memRange,
          regionType = RegionType.UNCACHED,
          supportsRead = TransferSizes(1, L3BlockSize),
          supportsWrite = TransferSizes(1, L3BlockSize),
          interleavedId = Some(0)
        )
      ),
      beatBytes = L3OuterBusWidth / 8,
      requestKeys = if (debugOpts.FPGAPlatform) Seq() else Seq(ReqSourceKey),
    )
  ))
  memAXI4SlaveNode :=
    AXI4Buffer() :=
    AXI4Buffer() :=
    AXI4Buffer() :=
    AXI4IdIndexer(idBits) :=
    AXI4UserYanker() :=
    AXI4Deinterleaver(L3BlockSize) :=
    TLToAXI4() :=
    TLSourceShrinker(64) :=
    TLWidthWidget(L3OuterBusWidth / 8) :=
    TLBuffer.chainNode(2) :=
    l3_xbar
  val debugMasterIO = InModuleBody {
    memAXI4SlaveNode.makeIOs()
  }

  // recieve all prefetch req from cores
  val memblock_pf_recv_nodes: Seq[Option[BundleBridgeSink[PrefetchRecv]]] = core_with_l2.map(_.core_l3_pf_port).map{
    x => x.map(_ => BundleBridgeSink(Some(() => new PrefetchRecv)))
  }

  val l3_pf_sender_opt = soc.L3CacheParamsOpt.getOrElse(HCCacheParameters()).prefetch match {
    case Some(pf) => Some(BundleBridgeSource(() => new PrefetchRecv))
    case None => None
  }

  val fakePlicIntNode: IntNexusNode = IntNexusNode(
    sourceFn = { _ => IntSourcePortParameters(Seq(IntSourceParameters(1))) },
    sinkFn   = { _ => IntSinkPortParameters(Seq(IntSinkParameters())) },
    outputRequiresInput = false,
    inputRequiresOutput = false)
  for (i <- 0 until NumCores) {
    core_with_l2(i).clint_int_node := clint.intnode
    core_with_l2(i).plic_int_node :*= fakePlicIntNode
    core_with_l2(i).debug_int_node := debugModule.debug.dmOuter.dmOuter.intnode
    val intNode = IntSinkNode(IntSinkPortSimple(1, 1))
    intNode := IntBuffer() := core_with_l2(i).beu_int_source
    val beu_int_source = InModuleBody {
      intNode.makeIOs()
    }
    memblock_pf_recv_nodes(i).map(recv => {
      println(s"Connecting Core_${i}'s L1 pf source to L3!")
      recv := core_with_l2(i).core_l3_pf_port.get
    })
  }

  val core_rst_nodes = core_with_l2.map(_ => BundleBridgeSource(() => Reset()))

  core_rst_nodes.zip(core_with_l2.map(_.core_reset_sink)).foreach({
    case (source, sink) =>  sink := source
  })

  class XSNocTopImp(wrapper: LazyModule) extends LazyRawModuleImp(wrapper) {
    FileRegisters.add("dts", dts)
    FileRegisters.add("graphml", graphML)
    FileRegisters.add("json", json)
    FileRegisters.add("plusArgs", freechips.rocketchip.util.PlusArgArtefacts.serialize_cHeader())

    val io = IO(new Bundle {
      val clock = Input(Bool())
      val reset = Input(AsyncReset())
      val extIntrs = Input(UInt(NrExtIntr.W))
      val systemjtag = new Bundle {
        val jtag = Flipped(new JTAGIO(hasTRSTn = false))
        val reset = Input(AsyncReset()) // No reset allowed on top
        val mfr_id = Input(UInt(11.W))
        val part_number = Input(UInt(16.W))
        val version = Input(UInt(4.W))
      }
      val debug_reset = Output(Bool())
      val rtc_clock = Input(Bool())
      val riscv_halt = Output(Vec(NumCores, Bool()))
      val riscv_rst_vec = Input(Vec(NumCores, UInt(38.W)))
      val chi = Vec(NumCores, new PortIO)
    })

    val reset_sync = withClockAndReset(io.clock.asClock, io.reset) { ResetGen() }
    val jtag_reset_sync = withClockAndReset(io.systemjtag.jtag.TCK, io.systemjtag.reset) { ResetGen() }

    // override LazyRawModuleImp's clock and reset
    childClock := io.clock.asClock
    childReset := reset_sync

    // positive edge sampling of the lower-speed rtc_clock
    withClockAndReset(clint.module.clock, clint.module.reset) {
      val rtcTick = RegInit(0.U(3.W))
      rtcTick := Cat(rtcTick(1, 0), io.rtc_clock)
      clint.module.io.rtcTick := rtcTick(1) && !rtcTick(2)
    }

    // output
    io.debug_reset := debugModule.module.io.debugIO.ndreset

    // input
    dontTouch(io)
    // TODO: misc.module.ext_intrs := io.extIntrs
    // TODO: do we need this? misc.module.cacheable_check <> io.cacheable_check

    val msiInfo = WireInit(0.U.asTypeOf(ValidIO(new MsiInfoBundle)))

    for ((core, i) <- core_with_l2.zipWithIndex) {
      core.module.io.hartId := i.U
      core.module.io.msiInfo := msiInfo
      core.module.io.chi.foreach(_ <> io.chi(i))
      io.riscv_halt(i) := core.module.io.cpu_halt
      core.module.io.reset_vector := io.riscv_rst_vec(i)
    }
    // tie off core soft reset
    for(node <- core_rst_nodes){
      node.out.head._1 := false.B.asAsyncReset
    }

    fakePlicIntNode.out.unzip._1.foreach(_.foreach(_ := false.B))

    core_with_l2.foreach(_.module.io.debugTopDown.l3MissMatch := false.B)

    debugModule.module.io.resetCtrl.hartIsInReset := core_with_l2.map(_.module.reset.asBool)
    debugModule.module.io.clock := io.clock
    debugModule.module.io.reset := reset_sync

    debugModule.module.io.debugIO.reset := core_with_l2.head.module.reset
    debugModule.module.io.debugIO.clock := io.clock.asClock
    // TODO: delay 3 cycles?
    debugModule.module.io.debugIO.dmactiveAck := debugModule.module.io.debugIO.dmactive
    // jtag connector
    debugModule.module.io.debugIO.systemjtag.foreach { x =>
      x.jtag        <> io.systemjtag.jtag
      x.reset       := jtag_reset_sync
      x.mfr_id      := io.systemjtag.mfr_id
      x.part_number := io.systemjtag.part_number
      x.version     := io.systemjtag.version
    }

    withClockAndReset(io.clock.asClock, reset_sync) {
      // Modules are reset one by one
      // reset ----> SYNC --> {SoCMisc, L3 Cache, Cores}
      val resetChain = Seq(Seq(clint.module, errorDevice.module) ++ core_with_l2.map(_.module))
      ResetGen(resetChain, reset_sync, !debugOpts.FPGAPlatform)
    }

  }

  lazy val module = new XSNocTopImp(this)
}

object NocTopMain extends App {
  val (config, firrtlOpts, firtoolOpts) = ArgParser.parse(args)

  // tools: init to close dpi-c when in fpga
  val envInFPGA = config(DebugOptionsKey).FPGAPlatform
  val enableChiselDB = config(DebugOptionsKey).EnableChiselDB
  val enableConstantin = config(DebugOptionsKey).EnableConstantin
  Constantin.init(enableConstantin && !envInFPGA)
  ChiselDB.init(enableChiselDB && !envInFPGA)

  val soc = DisableMonitors(p => LazyModule(new XSNocTop()(p)))(config)
  Generator.execute(firrtlOpts :+ "--full-stacktrace", soc.module, firtoolOpts :+ "--disable-annotation-unknown" :+ "--lowering-options=explicitBitcast,disallowLocalVariables,disallowPortDeclSharing,locationInfoStyle=none")
  FileRegisters.write(fileDir = "./build", filePrefix = "XSNocTop.")
}
