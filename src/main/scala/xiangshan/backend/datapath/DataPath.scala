package xiangshan.backend.datapath

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import difftest.{DiffArchFpRegState, DiffArchIntRegState, DiffArchVecRegState, DifftestModule}
import freechips.rocketchip.diplomacy.{LazyModule, LazyModuleImp}
import utility._
import utils.SeqUtils._
import utils.{XSPerfAccumulate, XSPerfHistogram}
import xiangshan._
import xiangshan.backend.BackendParams
import xiangshan.backend.Bundles._
import xiangshan.backend.decode.ImmUnion
import xiangshan.backend.datapath.DataConfig._
import xiangshan.backend.datapath.RdConfig._
import xiangshan.backend.issue.{ImmExtractor, IntScheduler, MemScheduler, VfScheduler}
import xiangshan.backend.implicitCast._
import xiangshan.backend.regfile._

class DataPath(params: BackendParams)(implicit p: Parameters) extends LazyModule {
  override def shouldBeInlined: Boolean = false

  private implicit val dpParams: BackendParams = params
  lazy val module = new DataPathImp(this)

  println(s"[DataPath] Preg Params: ")
  println(s"[DataPath]   Int R(${params.getRfReadSize(IntData())}), W(${params.getRfWriteSize(IntData())}) ")
  println(s"[DataPath]   Vf R(${params.getRfReadSize(VecData())}), W(${params.getRfWriteSize(VecData())}) ")
}

class DataPathImp(override val wrapper: DataPath)(implicit p: Parameters, params: BackendParams)
  extends LazyModuleImp(wrapper) with HasXSParameter {

  private val VCONFIG_PORT = params.vconfigPort

  val io = IO(new DataPathIO())

  private val (fromIntIQ, toIntIQ, toIntExu) = (io.fromIntIQ, io.toIntIQ, io.toIntExu)
  private val (fromMemIQ, toMemIQ, toMemExu) = (io.fromMemIQ, io.toMemIQ, io.toMemExu)
  private val (fromVfIQ , toVfIQ , toVfExu ) = (io.fromVfIQ , io.toVfIQ , io.toFpExu)

  println(s"[DataPath] IntIQ(${fromIntIQ.size}), MemIQ(${fromMemIQ.size})")
  println(s"[DataPath] IntExu(${fromIntIQ.map(_.size).sum}), MemExu(${fromMemIQ.map(_.size).sum})")

  // just refences for convience
  private val fromIQ: Seq[MixedVec[DecoupledIO[IssueQueueIssueBundle]]] = fromIntIQ ++ fromVfIQ ++ fromMemIQ

  println(s"[DataPath] fromIQ: ${fromIQ}")

  private val toIQs = toIntIQ ++ toVfIQ ++ toMemIQ

  println(s"[DataPath] toIQs: ${toIQs}")

  private val toExu: Seq[MixedVec[DecoupledIO[ExuInput]]] = toIntExu ++ toVfExu ++ toMemExu

  private val fromFlattenIQ: Seq[DecoupledIO[IssueQueueIssueBundle]] = fromIQ.flatten

  private val toFlattenExu: Seq[DecoupledIO[ExuInput]] = toExu.flatten

  val intRFBankNum = backendParams.pregParams.filter(_.dataCfg == IntData()).head.numBank
  val vfRFBankNum = backendParams.pregParams.filter(x => ((x.dataCfg == VecData()) || x.dataCfg == FpData())).head.numBank

  require(2 == backendParams.pregParams.size)

  private val intWbBusyArbiter = Module(new IntRFWBCollideChecker(backendParams))
  private val vfWbBusyArbiter = Module(new VfRFWBCollideChecker(backendParams))
  private val intRFReadArbiterSeq = Seq.fill(intRFBankNum)(Module(new IntRFReadArbiter(backendParams)))
  private val vfRFReadArbiterSeq = Seq.fill(vfRFBankNum)(Module(new VfRFReadArbiter(backendParams)))

  private val og0FailedVec2: MixedVec[Vec[Bool]] = Wire(MixedVec(fromIQ.map(x => Vec(x.size, Bool())).toSeq))
  private val og1FailedVec2: MixedVec[Vec[Bool]] = Wire(MixedVec(fromIQ.map(x => Vec(x.size, Bool())).toSeq))

  // port -> win
  private val intRdArbWinnerSeq: Seq3[MixedVec[Bool]] = intRFReadArbiterSeq.map(arb => arb.io.in.map(_.map(x => MixedVecInit(x.map(_.ready).toSeq)).toSeq).toSeq)
  private val vfRdArbWinnerSeq: Seq3[MixedVec[Bool]] = vfRFReadArbiterSeq.map(arb => arb.io.in.map(_.map(x => MixedVecInit(x.map(_.ready).toSeq)).toSeq).toSeq)
  private val intWbNotBlock: Seq[MixedVec[Bool]] = intWbBusyArbiter.io.in.map(x => MixedVecInit(x.map(_.ready).toSeq)).toSeq
  private val vfWbNotBlock: Seq[MixedVec[Bool]] = vfWbBusyArbiter.io.in.map(x => MixedVecInit(x.map(_.ready).toSeq)).toSeq

  println(s"intRFRdArbWinnerSeq: ${intRdArbWinnerSeq}")
  println(s"vfRFRdArbWinnerSeq: ${vfRdArbWinnerSeq}")

  // private val intRdNotBlock: Seq2[Bool] = intRdArbWinner.map(_.map(_.asUInt.andR))
  // private val vfRdNotBlock: Seq2[Bool] = vfRdArbWinner.map(_.map(_.asUInt.andR))

 def getBankedAddr(addr: UInt, bankNum: Int): UInt = {
   require(bankNum > 0 && isPow2(bankNum))
   (addr >> log2Ceil(bankNum)).asUInt
 }
  def getBankedIdxMatch(addr: UInt, bankNum: Int, bankIdx: Int): Bool = {
    if (bankNum == 1) true.B
    else (addr(log2Ceil(bankNum) - 1, 0) === bankIdx.U)
    // TODO: turn '===' into a pre-generated OneHot for better timing
  }

  private val intRFReadReq: Seq3[ValidIO[RfReadPortWithConfig]] = fromIQ.map(iq =>
    iq.map(port =>
      port.bits.getIntRfReadValidBundle(port.valid)).toSeq).toSeq
  // iqVec->exuVec->srcVec->bankOH
  private val intRFReadReqBankOH: Seq4[Bool] = intRFReadReq.map(portVec =>
    portVec.map(srcVec =>
      srcVec.map(src =>
        (0 until intRFBankNum).map(bankIdx =>
          getBankedIdxMatch(src.bits.addr, intRFBankNum, bankIdx)))))
  private val intBankedOHReg: Seq4[Bool] = intRFReadReqBankOH.map(
    portVec => portVec.map(
      srcVec => srcVec.map(
        src => src.map(
          bankValid => RegNext(bankValid))))) // TODO: change RegNext to Enable
  private val intRFReadReqAddr: Seq3[UInt] = intRFReadReq.map(portVec =>
    portVec.map(srcVec =>
      srcVec.map(src =>
        src.bits.addr)))
  private val intRFReadReqReadReg: Seq3[Bool] = fromIQ.map(iq =>
    iq.map(port =>
      port.bits.common.dataSources.map(_.readReg)))

  println(s"[DataPath] intRFReadReq: ${intRFReadReq}")
  println(s"[DataPath] intRFReadReqReadReg: ${intRFReadReqReadReg}")
  println(s"[DataPath] intRFReadReqBankOH: ${intRFReadReqBankOH}")
  println(s"[DataPath] intRFReadReqAddr: ${intRFReadReqAddr}")
  intRFReadArbiterSeq.indices.foreach { bankIdx =>
    val arbIn = intRFReadArbiterSeq(bankIdx).io.in
    arbIn.indices.foreach { iqIdx =>
      arbIn(iqIdx).indices.foreach { portIdx =>
        val srcIndices: Seq[Int] = fromIQ(iqIdx)(portIdx).bits.exuParams.getRfReadSrcIdx(IntData())
        val inRFReadReqSeq = intRFReadReq(iqIdx)(portIdx)
        println(s"[DataPath] IntRFReadArbiter: srcIndices: ${srcIndices}, inRFReadReqSeq: ${inRFReadReqSeq}")

        for (srcIdx <- 0 until fromIQ(iqIdx)(portIdx).bits.exuParams.numRegSrc) {
          if (srcIndices.contains(srcIdx) && inRFReadReqSeq.isDefinedAt(srcIdx)) {
            println(s"[DataPath] IntRFReadArbiter: bankIdx ${bankIdx} iqIdx(${iqIdx}), portIdx(${portIdx}), srcIdx(${srcIdx})")
            val issueValid = fromIQ(iqIdx)(portIdx).valid
            val readReg = intRFReadReqReadReg(iqIdx)(portIdx)(srcIdx)
            val addr = intRFReadReqAddr(iqIdx)(portIdx)(srcIdx)
            val bankValid = intRFReadReqBankOH(iqIdx)(portIdx)(srcIdx)(bankIdx)

            arbIn(iqIdx)(portIdx)(srcIdx).valid := issueValid && readReg && bankValid
            arbIn(iqIdx)(portIdx)(srcIdx).bits.addr := addr
          } else {
            println(s"[DataPath] IntRFReadArbiter: bankIdx ${bankIdx} iqIdx(${iqIdx}), portIdx(${portIdx}), srcIdx(${srcIdx}) Assign DontCare")
            arbIn(iqIdx)(portIdx)(srcIdx).valid := false.B
            arbIn(iqIdx)(portIdx)(srcIdx).bits.addr := DontCare // 0.U
          }
        }
      }
    }
  }

  private val vfRFReadReq: Seq3[ValidIO[RfReadPortWithConfig]] = fromIQ.map(x => x.map(xx => xx.bits.getVfRfReadValidBundle(xx.valid)).toSeq).toSeq
  // iqVec->exuVec->srcVec->bankOH
  private val vfRFReadReqBankOH: Seq4[Bool] = vfRFReadReq.map(portVec =>
    portVec.map(srcVec =>
      srcVec.map(src =>
        (0 until intRFBankNum).map(bankIdx => getBankedIdxMatch(src.bits.addr, vfRFBankNum, bankIdx)))))
  private val vfBankedOHReg: Seq4[Bool] = vfRFReadReqBankOH.map(
    portVec => portVec.map(
      srcVec => srcVec.map(
        src => src.map(
          bankValid => RegNext(bankValid))))) // TODO: change RegNext to Enable
  private val vfRFReadReqAddr: Seq3[UInt] = vfRFReadReq.map(portVec =>
    portVec.map(srcVec =>
      srcVec.map(src =>
        src.bits.addr)))

  vfRFReadArbiterSeq.indices.foreach { bankIdx =>
    val arbIn = vfRFReadArbiterSeq(bankIdx).io.in
    arbIn.indices.foreach { iqIdx =>
      arbIn(iqIdx).indices.foreach { portIdx =>
        val srcIndices: Seq[Int] = VfRegSrcDataSet.flatMap(data =>
          fromIQ(iqIdx)(portIdx).bits.exuParams.getRfReadSrcIdx(data)).toSeq.sorted
        val inRFReadReqSeq = vfRFReadReq(iqIdx)(portIdx)
        println(s"[DataPath] VfRFReadArbiter: srcIndices: ${srcIndices}, inRFReadReqSeq: ${inRFReadReqSeq}")

        for (srcIdx <- 0 until fromIQ(iqIdx)(portIdx).bits.exuParams.numRegSrc) {
          if (srcIndices.contains(srcIdx) && inRFReadReqSeq.isDefinedAt(srcIdx)) {
            println(s"[DataPath] VfRFReadArbiter: bankIdx ${bankIdx} iqIdx(${iqIdx}), portIdx(${portIdx}), srcIdx(${srcIdx})")
            val issueValid = fromIQ(iqIdx)(portIdx).valid
            val addr = vfRFReadReqAddr(iqIdx)(portIdx)(srcIdx)
            val bankValid = vfRFReadReqBankOH(iqIdx)(portIdx)(srcIdx)(bankIdx)

            arbIn(iqIdx)(portIdx)(srcIdx).valid := issueValid && bankValid
            arbIn(iqIdx)(portIdx)(srcIdx).bits.addr := addr
          } else {
            println(s"[DataPath] VfRFReadArbiter: bankIdx ${bankIdx} iqIdx(${iqIdx}), portIdx(${portIdx}), srcIdx(${srcIdx}) Assign DontCare")
            arbIn(iqIdx)(portIdx)(srcIdx).valid := false.B
            arbIn(iqIdx)(portIdx)(srcIdx).bits.addr := DontCare // 0.U
          }
        }
      }
    }
  }

  def chooseWinnerByBank(winnerSeq: Seq3[MixedVec[Bool]], oh: Seq4[Bool]): Seq2[Vec[Bool]] = {
    println(s"[DataPath] chooseWinnerByBank: winnerSeq.size: ${winnerSeq.size}, oh.size: ${oh.size}")
    println(s"[DataPath] chooseWinnerByBank: winnerSeq.head.size: ${winnerSeq.head.size}")
    winnerSeq.head.indices.map { case iqIdx =>
      println(s"[DataPath] chooseWinnerByBank: iqIdx(${iqIdx})")
      println(s"[DataPath] chooseWinnerByBank: winnerSeq.head(iqIdx).size: ${winnerSeq.head(iqIdx).size}")
      winnerSeq.head(iqIdx).indices.map { case portIdx =>
        println(s"[DataPath] chooseWinnerByBank: portIdx(${portIdx})")
        println(s"[DataPath] chooseWinnerByBank: winnerSeq.head(iqIdx)(portIdx).size: ${winnerSeq.head(iqIdx)(portIdx).size}")
        // NOTE: there are some FakeUnit that need no read port / write port
        // So winnerSeq.head(iqIdx)(portIdx).indices is empty
        val isFakeUnit = winnerSeq.head(iqIdx)(portIdx).indices.isEmpty
        if (isFakeUnit) {
          VecInit(Seq.fill(1)(false.B))// TODO: deal with empty winnerSeq
        } else {
          VecInit(winnerSeq.head(iqIdx)(portIdx).indices.map {
            case srcIdx =>
            // Choose by Bank
            println(s"[DataPath] chooseWinnerByBank: srcIdx(${srcIdx})")
            println(s"[DataPath] chooseWinnerByBank: oh.size: ${oh.size} oh(iqIdx).size: ${oh(iqIdx).size} oh(iqIdx)(portIdx).size: ${oh(iqIdx)(portIdx).size}")
            // NOTE: bankOH is generated from ReadReq, which is filtered by readType
            // But winnerSeq is generated from Arbiter, which is not filtered by RdConfig
            //    which will be set false if not wantted.
            // For example, VecData Read will exist at IntArbiter'io.in and be set true
            //    but it doesn't exist at IntReadReq. So their shapes are not equal
            if (oh(iqIdx)(portIdx).size == 0) {
              // NOTE: No bankOH, the port/src doesn't participate in this arb actually
              assert(winnerSeq.map(_(iqIdx)(portIdx)(srcIdx)).reduce(_ && _), "winnerSeq should be all true")
              winnerSeq.map(_(iqIdx)(portIdx)(srcIdx)).reduce(_ && _)
            } else {
              require(oh(iqIdx)(portIdx).size == winnerSeq.head(iqIdx)(portIdx).size,
                s"reqBankOH(${oh(iqIdx)(portIdx).size} ) and winnerSeq(${winnerSeq.head(iqIdx)(portIdx).size}) should have same src size.")
              require(oh(iqIdx)(portIdx)(srcIdx).size == winnerSeq.map(_(iqIdx)(portIdx)(srcIdx)).size)

              oh(iqIdx)(portIdx)(srcIdx)
                .zip(winnerSeq.map(_(iqIdx)(portIdx)(srcIdx)))
                .map(x => x._1 && x._2) // choose by bank
                .reduce(_ || _) // the chosen bank is ready
            }
          })
        }
      }
    }
  }

  private val intRdArbWinner: Seq2[Vec[Bool]] = chooseWinnerByBank(intRdArbWinnerSeq, intRFReadReqBankOH)
  private val vfRdArbWinner: Seq2[Vec[Bool]] = chooseWinnerByBank(vfRdArbWinnerSeq, vfRFReadReqBankOH)

  private val intRFWriteReq: Seq2[Bool] = fromIQ.map(x => x.map(xx => xx.valid && xx.bits.common.rfWen.getOrElse(false.B)).toSeq).toSeq
  private val vfRFWriteReq: Seq2[Bool] = fromIQ.map(x => x.map(xx => xx.valid && xx.bits.common.getVfWen.getOrElse(false.B)).toSeq).toSeq

  // TODO: add write bank optimization
  intWbBusyArbiter.io.in.zip(intRFWriteReq).foreach { case (arbInSeq, inRFWriteReqSeq) =>
    arbInSeq.zip(inRFWriteReqSeq).foreach { case (arbIn, inRFWriteReq) =>
      arbIn.valid := inRFWriteReq
    }
  }

  vfWbBusyArbiter.io.in.zip(vfRFWriteReq).foreach { case (arbInSeq, inRFWriteReqSeq) =>
    arbInSeq.zip(inRFWriteReqSeq).foreach { case (arbIn, inRFWriteReq) =>
      arbIn.valid := inRFWriteReq
    }
  }

  private val intSchdParams = params.schdParams(IntScheduler())
  private val vfSchdParams = params.schdParams(VfScheduler())
  private val memSchdParams = params.schdParams(MemScheduler())

  private val numIntRfReadByExu = intSchdParams.numIntRfReadByExu + memSchdParams.numIntRfReadByExu
  private val numVfRfReadByExu = vfSchdParams.numVfRfReadByExu + memSchdParams.numVfRfReadByExu
  // Todo: limit read port
  private val numIntR = numIntRfReadByExu
  private val numVfR = numVfRfReadByExu
  println(s"[DataPath] RegFile read req needed by Exu: Int(${numIntRfReadByExu}), Vf(${numVfRfReadByExu})")
  println(s"[DataPath] RegFile read port: Int(${numIntR}), Vf(${numVfR})")

  private val schdParams = params.allSchdParams

  private val intRfWen = Wire(Vec(io.fromIntWb.length, Bool()))
  private val intRfWaddr = Wire(Vec(io.fromIntWb.length, UInt(intSchdParams.pregIdxWidth.W)))
  private val intRfWdata = Wire(Vec(io.fromIntWb.length, UInt(intSchdParams.rfDataWidth.W)))

  private val intBankedRfRaddr = Wire(Vec(intRFBankNum, Vec(params.numPregRd(IntData()), UInt((intSchdParams.pregIdxWidth - log2Ceil(intRFBankNum)).W))))
  private val intBankedRfRdata = Wire(Vec(intRFBankNum, Vec(params.numPregRd(IntData()), UInt(intSchdParams.rfDataWidth.W))))
  private val intBankedRfWen   = Wire(Vec(intRFBankNum, Vec(io.fromIntWb.length, Bool())))
  private val intBankedRfWaddr = Wire(Vec(intRFBankNum, Vec(io.fromIntWb.length, UInt((intSchdParams.pregIdxWidth - log2Ceil(intRFBankNum)).W))))
  private val intBankedRfWdata = Wire(Vec(intRFBankNum, Vec(io.fromIntWb.length, UInt(intSchdParams.rfDataWidth.W))))

  private val vfRfSplitNum = VLEN / XLEN
  private val vfRfWen = Wire(Vec(io.fromVfWb.length, Bool()))
  private val vfRfWaddr = Wire(Vec(io.fromVfWb.length, UInt(vfSchdParams.pregIdxWidth.W)))
  private val vfRfWdata = Wire(Vec(io.fromVfWb.length, UInt(vfSchdParams.rfDataWidth.W)))

  private val vfBankedRfRaddr = Wire(Vec(vfRFBankNum, Vec(params.numPregRd(VecData()), UInt((vfSchdParams.pregIdxWidth - log2Ceil(vfRFBankNum)).W))))
  private val vfBankedRfRdata = Wire(Vec(vfRFBankNum, Vec(params.numPregRd(VecData()), UInt(vfSchdParams.rfDataWidth.W))))
  private val vfBankedRfWen   = Wire(Vec(vfRFBankNum, Vec(io.fromVfWb.length, Bool())))
  private val vfBankedRfWaddr = Wire(Vec(vfRFBankNum, Vec(io.fromVfWb.length, UInt((vfSchdParams.pregIdxWidth - log2Ceil(vfRFBankNum)).W))))
  private val vfBankedRfWdata = Wire(Vec(vfRFBankNum, Vec(io.fromVfWb.length, UInt(vfSchdParams.rfDataWidth.W))))
  private val vfSplitBankedRfWen = Wire(Vec(vfRfSplitNum, Vec(vfRFBankNum, Vec(io.fromVfWb.length, Bool()))))

  private val intDebugRead: Option[(Vec[UInt], Vec[UInt])] =
    if (env.AlwaysBasicDiff || env.EnableDifftest) {
      Some(Wire(Vec(32, UInt(intSchdParams.pregIdxWidth.W))), Wire(Vec(32, UInt(XLEN.W))))
    } else { None }
  private val vfDebugRead: Option[(Vec[UInt], Vec[UInt])] =
    if (env.AlwaysBasicDiff || env.EnableDifftest) {
      // TODO: rm hard code
      Some(Wire(Vec(32 + 32 + 1, UInt(vfSchdParams.pregIdxWidth.W))), Wire(Vec(32 + 32 + 1, UInt(VLEN.W))))
    } else { None }

  private val fpDebugReadData: Option[Vec[UInt]] =
    if (env.AlwaysBasicDiff || env.EnableDifftest) {
      Some(Wire(Vec(32, UInt(XLEN.W))))
    } else { None }
  private val vecDebugReadData: Option[Vec[UInt]] =
    if (env.AlwaysBasicDiff || env.EnableDifftest) {
      Some(Wire(Vec(64, UInt(64.W)))) // v0 = Cat(Vec(1), Vec(0))
    } else { None }
  private val vconfigDebugReadData: Option[UInt] =
    if (env.AlwaysBasicDiff || env.EnableDifftest) {
      Some(Wire(UInt(64.W)))
    } else { None }


  fpDebugReadData.foreach(_ := vfDebugRead
    .get._2
    .slice(0, 32)
    .map(_(63, 0))
  ) // fp only used [63, 0]
  vecDebugReadData.foreach(_ := vfDebugRead
    .get._2
    .slice(32, 64)
    .map(x => Seq(x(63, 0), x(127, 64))).flatten
  )
  vconfigDebugReadData.foreach(_ := vfDebugRead
    .get._2(64)(63, 0)
  )

  io.debugVconfig.foreach(_ := vconfigDebugReadData.get)

  IntRegFile("IntRegFile",
    intSchdParams.numPregs,
    intRFBankNum,
    intBankedRfRaddr,
    intBankedRfRdata,
    intBankedRfWen,
    intBankedRfWaddr,
    intBankedRfWdata,
    debugReadAddr = intDebugRead.map(_._1),
    debugReadData = intDebugRead.map(_._2)
  )

  VfRegFile("VfRegFile",
    vfSchdParams.numPregs,
    vfRfSplitNum,
    vfRFBankNum,
    vfBankedRfRaddr,
    vfBankedRfRdata,
    vfSplitBankedRfWen,
    vfBankedRfWaddr,
    vfBankedRfWdata,
    debugReadAddr = vfDebugRead.map(_._1),
    debugReadData = vfDebugRead.map(_._2)
  )

  // o: original, b: banked
  def getBankedWriteBundle(numBank: Int, o_addr: Seq[UInt], o_data: Seq[UInt], o_en: Seq[Bool],
    b_addr: Seq[Seq[UInt]], b_data: Seq[Seq[UInt]], b_en: Seq[Seq[Bool]]) = {
    require(numBank > 0 && isPow2(numBank))
    require(numBank == b_addr.size)

    b_data.map(x => x.zip(o_data).map(xx => xx._1 := xx._2))
    b_addr.map(x => x.zip(o_addr).map(xx => xx._1 := (xx._2 >> log2Ceil(numBank)).asUInt))
    b_en.zipWithIndex.map{ case (x, bankIdx) => x.zip(o_addr).zip(o_en).map{
      case ((b, a), o) => b := o &&
        (if (numBank == 1) true.B
         else a(log2Ceil(numBank)-1, 0) === bankIdx.U)
    }}
  }

  intRfWaddr := io.fromIntWb.map(_.addr).toSeq
  intRfWdata := io.fromIntWb.map(_.data).toSeq
  intRfWen := io.fromIntWb.map(_.wen).toSeq

  getBankedWriteBundle(intRFBankNum, intRfWaddr, intRfWdata, intRfWen,
    intBankedRfWaddr, intBankedRfWdata, intBankedRfWen)

  for (bankIdx <- 0 until intRFBankNum) {
    for (portIdx <- intBankedRfRaddr(bankIdx).indices) {
      if (intRFReadArbiterSeq(bankIdx).io.out.isDefinedAt(portIdx)) {
        val fullAddr = intRFReadArbiterSeq(bankIdx).io.out(portIdx).bits.addr
        intBankedRfRaddr(bankIdx)(portIdx) := getBankedAddr(fullAddr, intRFBankNum)
      }
      else intBankedRfRaddr(bankIdx)(portIdx) := 0.U // TODO: use DontCare?
    }
  }

  vfRfWaddr := io.fromVfWb.map(_.addr).toSeq
  vfRfWdata := io.fromVfWb.map(_.data).toSeq
  vfRfWen.zip(io.fromVfWb.map(_.wen)).foreach { case (wenSink, wenSource) => wenSink := wenSource }
  // Todo: support fp multi-write

  getBankedWriteBundle(vfRFBankNum, vfRfWaddr, vfRfWdata, vfRfWen,
    vfBankedRfWaddr, vfBankedRfWdata, vfBankedRfWen)
  vfSplitBankedRfWen.map(_ := vfBankedRfWen)
  require(vfSplitBankedRfWen.size == vfRfSplitNum, s"vfSplitBankedRfWen.size(${vfSplitBankedRfWen.size}) != vfRfSplitNum(${vfRfSplitNum})")

  for (bankIdx <- 0 until vfRFBankNum) {
    for (portIdx <- vfBankedRfRaddr(bankIdx).indices) {
      if (vfRFReadArbiterSeq(bankIdx).io.out.isDefinedAt(portIdx)) {
        val fullAddr = vfRFReadArbiterSeq(bankIdx).io.out(portIdx).bits.addr
        vfBankedRfRaddr(bankIdx)(portIdx) := getBankedAddr(fullAddr, vfRFBankNum)
      }
      else vfBankedRfRaddr(bankIdx)(portIdx) := 0.U
    }
  }

  // vfRfRaddr(VCONFIG_PORT) := io.vconfigReadPort.addr
  vfBankedRfRaddr.map(x => x(VCONFIG_PORT) := getBankedAddr(io.vconfigReadPort.addr, vfRFBankNum))
  if (vfRFBankNum == 1)
    io.vconfigReadPort.data := vfBankedRfRdata.head(VCONFIG_PORT)
  else
    io.vconfigReadPort.data := VecInit(vfBankedRfRdata.map(_ (VCONFIG_PORT)))(RegNext(io.vconfigReadPort.addr(log2Ceil(vfRFBankNum)-1, 0)))

  intDebugRead.foreach { case (addr, _) =>
    addr := io.debugIntRat.get
  }

  vfDebugRead.foreach { case (addr, _) =>
    addr := io.debugFpRat.get ++ io.debugVecRat.get :+ io.debugVconfigRat.get
  }
  println(s"[DataPath] " +
    s"has intDebugRead: ${intDebugRead.nonEmpty}, " +
    s"has vfDebugRead: ${vfDebugRead.nonEmpty}")

  val s1_addrOHs = Reg(MixedVec(
    fromIQ.map(x => MixedVec(x.map(_.bits.addrOH.cloneType).toSeq)).toSeq
  ))
  val s1_toExuValid: MixedVec[MixedVec[Bool]] = Reg(MixedVec(
    toExu.map(x => MixedVec(x.map(_.valid.cloneType).toSeq)).toSeq
  ))
  val s1_toExuData: MixedVec[MixedVec[ExuInput]] = Reg(MixedVec(toExu.map(x => MixedVec(x.map(_.bits.cloneType).toSeq)).toSeq))
  val s1_toExuReady = Wire(MixedVec(toExu.map(x => MixedVec(x.map(_.ready.cloneType))))) // Todo
  val s1_srcType: MixedVec[MixedVec[Vec[UInt]]] = MixedVecInit(fromIQ.map(x => MixedVecInit(x.map(xx => RegEnable(xx.bits.srcType, xx.fire)))))

  val s1_intPregRData: MixedVec[MixedVec[Vec[UInt]]] = Wire(MixedVec(toExu.map(x => MixedVec(x.map(_.bits.src.cloneType)))))
  val s1_vfPregRData: MixedVec[MixedVec[Vec[UInt]]] = Wire(MixedVec(toExu.map(x => MixedVec(x.map(_.bits.src.cloneType)))))

  val rfrPortConfigs = schdParams.map(_.issueBlockParams).flatten.map(_.exuBlockParams.map(_.rfrPortConfigs))

  println(s"[DataPath] s1_intPregRData.flatten.flatten.size: " +
          s"${s1_intPregRData.flatten.flatten.size}, intBankedRfRdata.head.size: ${intBankedRfRdata.head.size}")
  s1_intPregRData.foreach(_.foreach(_.foreach(_ := 0.U)))
  s1_intPregRData.zip(rfrPortConfigs).zip(intBankedOHReg).foreach { case ((iqRdata, iqCfg), iqOH) =>
      iqRdata.zip(iqCfg).zip(iqOH).foreach { case ((iuRdata, iuCfg), iuOH) =>
        val realIuCfg = iuCfg.map(x => if(x.size > 1) x.filter(_.isInstanceOf[IntRD]) else x).flatten
        assert(iuRdata.size == realIuCfg.size, "iuRdata.size != realIuCfg.size")
        iuRdata.zip(realIuCfg).zip(iuOH)
          .filter { case ((_, rfrPortConfig), srcOH) => rfrPortConfig.isInstanceOf[IntRD] }
          .foreach { case ((sink, cfg), srcOH) =>
            sink := Mux1H(srcOH, intBankedRfRdata.map(_ (cfg.port)))
          }
      }
  }

  println(s"[DataPath] s1_vfPregRData.flatten.flatten.size: " +
          s"${s1_vfPregRData.flatten.flatten.size}, vfBankedRfRdata.head.size: ${vfBankedRfRdata.size}")
  s1_vfPregRData.foreach(_.foreach(_.foreach(_ := 0.U)))
  s1_vfPregRData.zip(rfrPortConfigs).zip(vfBankedOHReg).foreach{ case ((iqRdata, iqCfg), iqOH) =>
      iqRdata.zip(iqCfg).zip(iqOH).foreach { case ((iuRdata, iuCfg), iuOH) =>
        val realIuCfg = iuCfg.map(x => if(x.size > 1) x.filter(_.isInstanceOf[VfRD]) else x).flatten
        assert(iuRdata.size == realIuCfg.size, "iuRdata.size != realIuCfg.size")
        iuRdata.zip(realIuCfg).zip(iuOH)
          .filter { case ((_, rfrPortConfig), srcOH) => rfrPortConfig.isInstanceOf[VfRD] }
          .foreach { case ((sink, cfg), srcOH) =>
            sink := Mux1H(srcOH, vfBankedRfRdata.map(_ (cfg.port)))
          }
      }
  }

  for (i <- fromIQ.indices) {
    for (j <- fromIQ(i).indices) {
      // IQ(s0) --[Ctrl]--> s1Reg ---------- begin
      // refs
      val s1_valid = s1_toExuValid(i)(j)
      val s1_ready = s1_toExuReady(i)(j)
      val s1_data = s1_toExuData(i)(j)
      val s1_addrOH = s1_addrOHs(i)(j)
      val s0 = fromIQ(i)(j) // s0
      val srcNotBlock = s0.bits.common.dataSources
        .zip(intRdArbWinner(i)(j) zip vfRdArbWinner(i)(j))
        .map { case (source, win) =>
          // TODO: add srcType(Int/Vec) check when need Mix-Type-Read(STD?)
          !source.readReg || win._1 && win._2 }
        .fold(true.B)(_ && _)
      val notBlock = srcNotBlock && intWbNotBlock(i)(j) && vfWbNotBlock(i)(j)
      val s1_flush = s0.bits.common.robIdx.needFlush(Seq(io.flush, RegNextWithEnable(io.flush)))
      val s1_cancel = og1FailedVec2(i)(j)
      val s1_ldCancel = LoadShouldCancel(s0.bits.common.loadDependency, io.ldCancel)
      when (s0.fire && !s1_flush && notBlock && !s1_cancel && !s1_ldCancel) {
        s1_valid := s0.valid
        s1_data.fromIssueBundle(s0.bits) // no src data here
        s1_addrOH := s0.bits.addrOH
      }.otherwise {
        s1_valid := false.B
      }
      s0.ready := (s1_ready || !s1_valid) && notBlock
      // IQ(s0) --[Ctrl]--> s1Reg ---------- end

      // IQ(s0) --[Data]--> s1Reg ---------- begin
      // imm extract
      when (s0.fire && !s1_flush && notBlock) {
        if (s1_data.params.immType.nonEmpty && s1_data.src.size > 1) {
          // rs1 is always int reg, rs2 may be imm
          when(SrcType.isImm(s0.bits.srcType(1))) {
            s1_data.src(1) := ImmExtractor(
              s0.bits.common.imm,
              s0.bits.immType,
              s1_data.params.dataBitsMax,
              s1_data.params.immType.map(_.litValue)
            )
          }
        }
        if (s1_data.params.hasJmpFu) {
          when(SrcType.isPc(s0.bits.srcType(0))) {
            s1_data.src(0) := SignExt(s0.bits.common.pc.get, XLEN)
          }
        } else if (s1_data.params.hasVecFu) {
          // Fuck off riscv vector imm!!! Why not src1???
          when(SrcType.isImm(s0.bits.srcType(0))) {
            s1_data.src(0) := ImmExtractor(
              s0.bits.common.imm,
              s0.bits.immType,
              s1_data.params.dataBitsMax,
              s1_data.params.immType.map(_.litValue)
            )
          }
        } else if (s1_data.params.hasLoadFu || s1_data.params.hasHyldaFu) {
          // dirty code for fused_lui_load
          when(SrcType.isImm(s0.bits.srcType(0))) {
            s1_data.src(0) := SignExt(ImmUnion.U.toImm32(s0.bits.common.imm(s0.bits.common.imm.getWidth - 1, ImmUnion.I.len)), XLEN)
          }
        }
      }
      // IQ(s0) --[Data]--> s1Reg ---------- end
    }
  }

  private val fromIQFire = fromIQ.map(_.map(_.fire))
  private val toExuFire = toExu.map(_.map(_.fire))
  toIQs.zipWithIndex.foreach {
    case(toIQ, iqIdx) =>
      toIQ.zipWithIndex.foreach {
        case (toIU, iuIdx) =>
          // IU: issue unit
          val og0resp = toIU.og0resp
          og0FailedVec2(iqIdx)(iuIdx) := fromIQ(iqIdx)(iuIdx).valid && (!fromIQFire(iqIdx)(iuIdx))
          og0resp.valid := og0FailedVec2(iqIdx)(iuIdx)
          og0resp.bits.respType := RSFeedbackType.rfArbitFail
          og0resp.bits.dataInvalidSqIdx := DontCare
          og0resp.bits.robIdx := fromIQ(iqIdx)(iuIdx).bits.common.robIdx
          og0resp.bits.rfWen := fromIQ(iqIdx)(iuIdx).bits.common.rfWen.getOrElse(false.B)
          og0resp.bits.fuType := fromIQ(iqIdx)(iuIdx).bits.common.fuType

          val og1resp = toIU.og1resp
          og1FailedVec2(iqIdx)(iuIdx) := s1_toExuValid(iqIdx)(iuIdx) && !toExuFire(iqIdx)(iuIdx)
          og1resp.valid := s1_toExuValid(iqIdx)(iuIdx)
          og1resp.bits.respType := Mux(
            !og1FailedVec2(iqIdx)(iuIdx),
            if (toIU.issueQueueParams.isMemAddrIQ) RSFeedbackType.fuUncertain else RSFeedbackType.fuIdle,
            RSFeedbackType.fuBusy
          )
          og1resp.bits.dataInvalidSqIdx := DontCare
          og1resp.bits.robIdx := s1_toExuData(iqIdx)(iuIdx).robIdx
          og1resp.bits.rfWen := s1_toExuData(iqIdx)(iuIdx).rfWen.getOrElse(false.B)
          og1resp.bits.fuType := s1_toExuData(iqIdx)(iuIdx).fuType
      }
  }

  io.og0CancelOH := VecInit(fromFlattenIQ.map(x => x.valid && !x.fire)).asUInt
  io.og1CancelOH := VecInit(toFlattenExu.map(x => x.valid && !x.fire)).asUInt

  io.cancelToBusyTable.zipWithIndex.foreach { case (cancel, i) =>
    cancel.valid := fromFlattenIQ(i).valid && !fromFlattenIQ(i).fire && {
      if (fromFlattenIQ(i).bits.common.rfWen.isDefined)
        fromFlattenIQ(i).bits.common.rfWen.get && fromFlattenIQ(i).bits.common.pdest =/= 0.U
      else
        true.B
    }
    cancel.bits.rfWen := fromFlattenIQ(i).bits.common.rfWen.getOrElse(false.B)
    cancel.bits.fpWen := fromFlattenIQ(i).bits.common.fpWen.getOrElse(false.B)
    cancel.bits.vecWen := fromFlattenIQ(i).bits.common.vecWen.getOrElse(false.B)
    cancel.bits.pdest := fromFlattenIQ(i).bits.common.pdest
  }

  for (i <- toExu.indices) {
    for (j <- toExu(i).indices) {
      // s1Reg --[Ctrl]--> exu(s1) ---------- begin
      // refs
      val sinkData = toExu(i)(j).bits
      // assign
      toExu(i)(j).valid := s1_toExuValid(i)(j)
      s1_toExuReady(i)(j) := toExu(i)(j).ready
      sinkData := s1_toExuData(i)(j)
      // s1Reg --[Ctrl]--> exu(s1) ---------- end

      // s1Reg --[Data]--> exu(s1) ---------- begin
      // data source1: preg read data
      for (k <- sinkData.src.indices) {
        val srcDataTypeSet: Set[DataConfig] = sinkData.params.getSrcDataType(k)

        val readRfMap: Seq[(Bool, UInt)] = (Seq(None) :+
          (if (s1_intPregRData(i)(j).isDefinedAt(k) && srcDataTypeSet.intersect(IntRegSrcDataSet).nonEmpty)
            Some(SrcType.isXp(s1_srcType(i)(j)(k)) -> s1_intPregRData(i)(j)(k))
          else None) :+
          (if (s1_vfPregRData(i)(j).isDefinedAt(k) && srcDataTypeSet.intersect(VfRegSrcDataSet).nonEmpty)
            Some(SrcType.isVfp(s1_srcType(i)(j)(k))-> s1_vfPregRData(i)(j)(k))
          else None)
        ).filter(_.nonEmpty).map(_.get)
        if (readRfMap.nonEmpty)
          sinkData.src(k) := Mux1H(readRfMap)
      }

      // data source2: extracted imm and pc saved in s1Reg
      if (sinkData.params.immType.nonEmpty && sinkData.src.size > 1) {
        when(SrcType.isImm(s1_srcType(i)(j)(1))) {
          sinkData.src(1) := s1_toExuData(i)(j).src(1)
        }
      }
      if (sinkData.params.hasJmpFu) {
        when(SrcType.isPc(s1_srcType(i)(j)(0))) {
          sinkData.src(0) := s1_toExuData(i)(j).src(0)
        }
      } else if (sinkData.params.hasVecFu) {
        when(SrcType.isImm(s1_srcType(i)(j)(0))) {
          sinkData.src(0) := s1_toExuData(i)(j).src(0)
        }
      } else if (sinkData.params.hasLoadFu || sinkData.params.hasHyldaFu) {
        when(SrcType.isImm(s1_srcType(i)(j)(0))) {
          sinkData.src(0) := s1_toExuData(i)(j).src(0)
        }
      }
      // s1Reg --[Data]--> exu(s1) ---------- end
    }
  }

  if (env.AlwaysBasicDiff || env.EnableDifftest) {
    val delayedCnt = 2
    val difftestArchIntRegState = DifftestModule(new DiffArchIntRegState, delay = delayedCnt)
    difftestArchIntRegState.coreid := io.hartId
    difftestArchIntRegState.value := intDebugRead.get._2

    val difftestArchFpRegState = DifftestModule(new DiffArchFpRegState, delay = delayedCnt)
    difftestArchFpRegState.coreid := io.hartId
    difftestArchFpRegState.value := fpDebugReadData.get

    val difftestArchVecRegState = DifftestModule(new DiffArchVecRegState, delay = delayedCnt)
    difftestArchVecRegState.coreid := io.hartId
    difftestArchVecRegState.value := vecDebugReadData.get
  }

  val int_regcache_size = 48
  val int_regcache_tag = RegInit(VecInit(Seq.fill(int_regcache_size)(0.U(intSchdParams.pregIdxWidth.W))))
  val int_regcache_enqPtr = RegInit(0.U(log2Up(int_regcache_size).W))
  int_regcache_enqPtr := int_regcache_enqPtr + PopCount(intRfWen)
  for (i <- intRfWen.indices) {
    when (intRfWen(i)) {
      int_regcache_tag(int_regcache_enqPtr + PopCount(intRfWen.take(i))) := intRfWaddr(i)
    }
  }

  val vf_regcache_size = 48
  val vf_regcache_tag = RegInit(VecInit(Seq.fill(vf_regcache_size)(0.U(vfSchdParams.pregIdxWidth.W))))
  val vf_regcache_enqPtr = RegInit(0.U(log2Up(vf_regcache_size).W))
  vf_regcache_enqPtr := vf_regcache_enqPtr + PopCount(vfRfWen)
  for (i <- vfRfWen.indices) {
    when (vfRfWen(i)) {
      vf_regcache_tag(vf_regcache_enqPtr + PopCount(vfRfWen.take(i))) := vfRfWaddr(i)
    }
  }

  XSPerfHistogram(s"IntRegFileRead_hist", PopCount(intRFReadArbiterSeq.map(_.io.in.flatten.flatten.map(_.valid)).flatten), true.B, 0, 20, 1)
  XSPerfHistogram(s"VfRegFileRead_hist", PopCount(vfRFReadArbiterSeq.map(_.io.in.flatten.flatten.map(_.valid)).flatten), true.B, 0, 20, 1)
  XSPerfHistogram(s"IntRegFileWrite_hist", PopCount(intRFWriteReq.flatten), true.B, 0, 20, 1)
  XSPerfHistogram(s"VfRegFileWrite_hist", PopCount(vfRFWriteReq.flatten), true.B, 0, 20, 1)

  val int_regcache_part32 = (1 until 33).map(i => int_regcache_tag(int_regcache_enqPtr - i.U))
  val int_regcache_part24 = (1 until 24).map(i => int_regcache_tag(int_regcache_enqPtr - i.U))
  val int_regcache_part16 = (1 until 17).map(i => int_regcache_tag(int_regcache_enqPtr - i.U))
  val int_regcache_part8 = (1 until 9).map(i => int_regcache_tag(int_regcache_enqPtr - i.U))

  val int_regcache_48_hit_vec = intRFReadArbiterSeq.map(_.io.in.flatten.flatten.map(x => x.valid && int_regcache_tag.map(_ === x.bits.addr).reduce(_ || _))).flatten
  val int_regcache_8_hit_vec = intRFReadArbiterSeq.map(_.io.in.flatten.flatten.map(x => x.valid && int_regcache_part8.map(_ === x.bits.addr).reduce(_ || _))).flatten
  val int_regcache_16_hit_vec = intRFReadArbiterSeq.map(_.io.in.flatten.flatten.map(x => x.valid && int_regcache_part16.map(_ === x.bits.addr).reduce(_ || _))).flatten
  val int_regcache_24_hit_vec = intRFReadArbiterSeq.map(_.io.in.flatten.flatten.map(x => x.valid && int_regcache_part24.map(_ === x.bits.addr).reduce(_ || _))).flatten
  val int_regcache_32_hit_vec = intRFReadArbiterSeq.map(_.io.in.flatten.flatten.map(x => x.valid && int_regcache_part32.map(_ === x.bits.addr).reduce(_ || _))).flatten
  XSPerfAccumulate("IntRegCache48Hit", PopCount(int_regcache_48_hit_vec))
  XSPerfAccumulate("IntRegCache8Hit", PopCount(int_regcache_8_hit_vec))
  XSPerfAccumulate("IntRegCache16Hit", PopCount(int_regcache_16_hit_vec))
  XSPerfAccumulate("IntRegCache24Hit", PopCount(int_regcache_24_hit_vec))
  XSPerfAccumulate("IntRegCache32Hit", PopCount(int_regcache_32_hit_vec))
  XSPerfHistogram("IntRegCache48Hit_hist", PopCount(int_regcache_48_hit_vec), true.B, 0, 16, 2)

  XSPerfAccumulate(s"IntRFReadBeforeArb", PopCount(intRFReadArbiterSeq.map(_.io.in.flatten.flatten.map(_.valid)).flatten))
  XSPerfAccumulate(s"IntRFReadAfterArb", PopCount(intRFReadArbiterSeq.map(_.io.out.map(_.valid)).flatten))
  XSPerfAccumulate(s"VfRFReadBeforeArb", PopCount(vfRFReadArbiterSeq.map(_.io.in.flatten.flatten.map(_.valid)).flatten))
  XSPerfAccumulate(s"VfRFReadAfterArb", PopCount(vfRFReadArbiterSeq.map(_.io.out.map(_.valid)).flatten))
  XSPerfAccumulate(s"IntUopBeforeArb", PopCount(fromIntIQ.flatten.map(_.valid)))
  XSPerfAccumulate(s"IntUopAfterArb", PopCount(fromIntIQ.flatten.map(_.fire)))
  XSPerfAccumulate(s"MemUopBeforeArb", PopCount(fromMemIQ.flatten.map(_.valid)))
  XSPerfAccumulate(s"MemUopAfterArb", PopCount(fromMemIQ.flatten.map(_.fire)))
  XSPerfAccumulate(s"VfUopBeforeArb", PopCount(fromVfIQ.flatten.map(_.valid)))
  XSPerfAccumulate(s"VfUopAfterArb", PopCount(fromVfIQ.flatten.map(_.fire)))

  XSPerfHistogram(s"IntRFReadBeforeArb_hist", PopCount(intRFReadArbiterSeq.map(_.io.in.flatten.flatten.map(_.valid)).flatten), true.B, 0, 16, 2)
  XSPerfHistogram(s"IntRFReadAfterArb_hist", PopCount(intRFReadArbiterSeq.map(_.io.out.map(_.valid)).flatten), true.B, 0, 16, 2)
  XSPerfHistogram(s"VfRFReadBeforeArb_hist", PopCount(vfRFReadArbiterSeq.map(_.io.in.flatten.flatten.map(_.valid)).flatten), true.B, 0, 16, 2)
  XSPerfHistogram(s"VfRFReadAfterArb_hist", PopCount(vfRFReadArbiterSeq.map(_.io.out.map(_.valid)).flatten), true.B, 0, 16, 2)
  XSPerfHistogram(s"IntUopBeforeArb_hist", PopCount(fromIntIQ.flatten.map(_.valid)), true.B, 0, 8, 2)
  XSPerfHistogram(s"IntUopAfterArb_hist", PopCount(fromIntIQ.flatten.map(_.fire)), true.B, 0, 8, 2)
  XSPerfHistogram(s"MemUopBeforeArb_hist", PopCount(fromMemIQ.flatten.map(_.valid)), true.B, 0, 8, 2)
  XSPerfHistogram(s"MemUopAfterArb_hist", PopCount(fromMemIQ.flatten.map(_.fire)), true.B, 0, 8, 2)
  XSPerfHistogram(s"VfUopBeforeArb_hist", PopCount(fromVfIQ.flatten.map(_.valid)), true.B, 0, 8, 2)
  XSPerfHistogram(s"VfUopAfterArb_hist", PopCount(fromVfIQ.flatten.map(_.fire)), true.B, 0, 8, 2)
}

class DataPathIO()(implicit p: Parameters, params: BackendParams) extends XSBundle {
  // params
  private val intSchdParams = params.schdParams(IntScheduler())
  private val vfSchdParams = params.schdParams(VfScheduler())
  private val memSchdParams = params.schdParams(MemScheduler())
  // bundles
  val hartId = Input(UInt(8.W))

  val flush: ValidIO[Redirect] = Flipped(ValidIO(new Redirect))

  // Todo: check if this can be removed
  val vconfigReadPort = new RfReadPort(XLEN, PhyRegIdxWidth)

  val wbConfictRead = Input(MixedVec(params.allSchdParams.map(x => MixedVec(x.issueBlockParams.map(x => x.genWbConflictBundle())))))

  val fromIntIQ: MixedVec[MixedVec[DecoupledIO[IssueQueueIssueBundle]]] =
    Flipped(MixedVec(intSchdParams.issueBlockParams.map(_.genIssueDecoupledBundle)))

  val fromMemIQ: MixedVec[MixedVec[DecoupledIO[IssueQueueIssueBundle]]] =
    Flipped(MixedVec(memSchdParams.issueBlockParams.map(_.genIssueDecoupledBundle)))

  val fromVfIQ = Flipped(MixedVec(vfSchdParams.issueBlockParams.map(_.genIssueDecoupledBundle)))

  println(s"[DataPath] fromIntIQ.size: ${fromIntIQ.size}, fromMemIQ.size: ${fromMemIQ.size}, fromVfIQ.size: ${fromVfIQ.size}")
  println(s"[DataPath] fromIntIQ: ${fromIntIQ} fromMemIQ: ${fromMemIQ} fromVfIQ: ${fromVfIQ}")

  val toIntIQ = MixedVec(intSchdParams.issueBlockParams.map(_.genOGRespBundle))

  val toMemIQ = MixedVec(memSchdParams.issueBlockParams.map(_.genOGRespBundle))

  val toVfIQ = MixedVec(vfSchdParams.issueBlockParams.map(_.genOGRespBundle))

  val og0CancelOH = Output(ExuOH(backendParams.numExu))

  val og1CancelOH = Output(ExuOH(backendParams.numExu))

  val ldCancel = Vec(backendParams.LduCnt + backendParams.HyuCnt, Flipped(new LoadCancelIO))

  val cancelToBusyTable = Vec(backendParams.numExu, ValidIO(new CancelSignal))

  val toIntExu: MixedVec[MixedVec[DecoupledIO[ExuInput]]] = intSchdParams.genExuInputBundle

  val toFpExu: MixedVec[MixedVec[DecoupledIO[ExuInput]]] = MixedVec(vfSchdParams.genExuInputBundle)

  val toMemExu: MixedVec[MixedVec[DecoupledIO[ExuInput]]] = memSchdParams.genExuInputBundle

  val fromIntWb: MixedVec[RfWritePortWithConfig] = MixedVec(params.genIntWriteBackBundle)

  val fromVfWb: MixedVec[RfWritePortWithConfig] = MixedVec(params.genVfWriteBackBundle)

  val debugIntRat     = if (params.debugEn) Some(Input(Vec(32, UInt(intSchdParams.pregIdxWidth.W)))) else None
  val debugFpRat      = if (params.debugEn) Some(Input(Vec(32, UInt(vfSchdParams.pregIdxWidth.W)))) else None
  val debugVecRat     = if (params.debugEn) Some(Input(Vec(32, UInt(vfSchdParams.pregIdxWidth.W)))) else None
  val debugVconfigRat = if (params.debugEn) Some(Input(UInt(vfSchdParams.pregIdxWidth.W))) else None
  val debugVconfig    = if (params.debugEn) Some(Output(UInt(XLEN.W))) else None
}
