package xiangshan.frontend

import chipsalliance.rocketchip.config.Parameters
import chisel3._
import chisel3.util._
import utils._
import xiangshan._
import chisel3.experimental.chiselName

trait LoopPredictorParams extends HasXSParameter with HasBPUParameter {
  val nRows = 32
  val idxLen = log2Up(nRows) // 5
  val tagLen = VAddrBits - (idxLen + instOffsetBits)
  val cntBits = 10
  val confBits = 1

  def maxConf = 1.U
  def minConf = 0.U

  def dupForLp = dupForIttage

  def getLtbTag(pc: UInt) = pc >> (idxLen + instOffsetBits)
  def getLtbIdx(pc: UInt) = pc(instOffsetBits+idxLen-1,instOffsetBits)
  def initLTBentry(newLTBtag: UInt, target: UInt) : LoopEntry = {
    val newLTBentry = WireDefault( 0.U.asTypeOf(new LoopEntry) )
    newLTBentry.tag        := newLTBtag(tagLen-1, 0)
    newLTBentry.specCnt    := 1.U
    newLTBentry.target     := target
    newLTBentry
  }
  def increConf(conf: UInt) : UInt = {
    val newConf = Mux( (conf === maxConf), maxConf, (conf + 1.U) )
    newConf
  }
  def decreConf(conf: UInt) : UInt = {
    val newConf = Mux( (conf === minConf), minConf, (conf - 1.U) )
    newConf
  }
}

class LoopEntry(implicit p: Parameters) extends XSBundle with LoopPredictorParams {
  val tag        = UInt(tagLen.W)
  val specCnt    = UInt(cntBits.W)
  val tripCnt    = UInt(cntBits.W)
  val conf       = UInt(confBits.W)  
  val target     = UInt(VAddrBits.W)

  def isConf    = (conf === maxConf)
  def isNotConf = (conf === minConf) 
}

class LPmeta(implicit p: Parameters) extends XSBundle with LoopPredictorParams {
  val isLPpred          = Bool()
  val isNotExitLoop     = Bool() // taken
  val isExitLoop        = Bool() // not taken
  val isConfNotExitLoop = Bool() // taken
  val isConfExitLoop    = Bool() // not taken
  val specCnt           = UInt(cntBits.W)
  val tripCnt           = UInt(cntBits.W)
}

class LTBrwIO(implicit p: Parameters) extends XSBundle with LoopPredictorParams {
  val readEna    = Input(Bool())
  val readIdx    = Input(UInt(idxLen.W))
  val readEntry  = Output(new LoopEntry)

  val writeEna   = Input(Bool())
  val writeIdx   = Input(UInt(idxLen.W))
  val writeEntry = Input(new LoopEntry)
}

class LTBio(implicit p: Parameters) extends XSBundle with LoopPredictorParams {
  val pred    = new LTBrwIO
  val update  = new LTBrwIO
  // val recover = new LTBrwIO
}

class LTB(implicit p: Parameters) extends XSModule with LoopPredictorParams {
  val io = IO(new LTBio)

  val ltb = RegInit(VecInit(Seq.fill(nRows)(0.U.asTypeOf(new LoopEntry))))

  val wCflctUpdatePred = (io.pred.writeEna && io.update.writeEna && io.pred.writeIdx === io.update.writeIdx)

  // read
  when(io.pred.readEna) {
    when(io.update.writeEna && io.update.writeIdx === io.pred.readIdx) {
      io.pred.readEntry := io.update.writeEntry
    }.otherwise {
      io.pred.readEntry := ltb(io.pred.readIdx)
    }
  }.otherwise {
    io.pred.readEntry := 0.U.asTypeOf(new LoopEntry)
  }

  when(io.update.readEna) {
    io.update.readEntry := ltb(io.update.readIdx)
  }.otherwise {
    io.update.readEntry := 0.U.asTypeOf(new LoopEntry)
  }

  // write
  when( wCflctUpdatePred || io.update.writeEna ) {
    ltb(io.update.writeIdx) := io.update.writeEntry
  }.elsewhen(io.pred.writeEna) {
    ltb(io.pred.writeIdx) := io.pred.writeEntry
  }

}

class lpPredIO(implicit p: Parameters) extends XSBundle with LoopPredictorParams {
  val valid             = Input(Bool())
  val pc                = Input(UInt(VAddrBits.W))
  val isExitLoop        = Output(Bool())
  val isNotExitLoop     = Output(Bool())
  val isConfExitLoop    = Output(Bool())
  val isConfNotExitLoop = Output(Bool())
  val specCnt           = Output(UInt(cntBits.W))
  val tripCnt           = Output(UInt(cntBits.W))
  val target            = Output(UInt(VAddrBits.W))
}

class lpUpdateIO(implicit p: Parameters) extends XSBundle with LoopPredictorParams {
  val valid          = Input(Bool())
  val pc             = Input(UInt(VAddrBits.W))
  val isLoopBranch   = Input(Bool())
  val isUpdateTaken  = Input(Bool())
  val isPredTaken    = Input(Bool())
  val isPredNotTaken = Input(Bool())
  val isMispred      = Input(Bool())
  val predSpecCnt    = Input(UInt(cntBits.W))
  val target         = Input(UInt(VAddrBits.W))
}

class LoopPredictor(implicit p: Parameters) extends XSModule with LoopPredictorParams {
  val io = IO(new Bundle{
    val pred   = new lpPredIO
    val update = new lpUpdateIO
  })

  val ltb = Module(new LTB)

  // prediction
  val predLTBidx = getLtbIdx(io.pred.pc)
  ltb.io.pred.readEna := io.pred.valid
  ltb.io.pred.readIdx := predLTBidx
  val predLTBreadEntry = ltb.io.pred.readEntry

  val predTag = getLtbTag(io.pred.pc)
  val predTagMatch = (predTag === predLTBreadEntry.tag)
  val predLTBwriteEntry = WireDefault(predLTBreadEntry) 
  val predLTBwena = io.pred.valid && predTagMatch
  when( predLTBwena && (predLTBreadEntry.specCnt =/= predLTBreadEntry.tripCnt || predLTBreadEntry.specCnt === 0.U) ) {
    predLTBwriteEntry.specCnt := predLTBreadEntry.specCnt + 1.U
  }.elsewhen(predLTBwena && predLTBreadEntry.specCnt === predLTBreadEntry.tripCnt) {
    predLTBwriteEntry.specCnt := 1.U
  }
 
  ltb.io.pred.writeEna   := predLTBwena
  ltb.io.pred.writeIdx   := predLTBidx
  ltb.io.pred.writeEntry := predLTBwriteEntry  

  val predCntEq  = (predLTBwriteEntry.specCnt === predLTBwriteEntry.tripCnt)
  val predCntNeq = (predLTBwriteEntry.specCnt =/= predLTBwriteEntry.tripCnt)
  io.pred.isExitLoop        := (io.pred.valid && predTagMatch && predCntEq)
  io.pred.isNotExitLoop     := (io.pred.valid && !predTagMatch) || (io.pred.valid && predTagMatch && predCntNeq)
  io.pred.isConfExitLoop    := (io.pred.valid && predTagMatch && predCntEq  && predLTBreadEntry.isConf)  
  io.pred.isConfNotExitLoop := (io.pred.valid && predTagMatch && predCntNeq && predLTBreadEntry.isConf)  
  io.pred.specCnt := predLTBwriteEntry.specCnt
  io.pred.tripCnt := predLTBwriteEntry.tripCnt
  io.pred.target  := predLTBwriteEntry.target


  when(io.pred.valid) {
    printf("prediction--  tagMatch: %d;  idx: %d;  tag: %x;  spec-cnt: %d;  trip_cnt: %d;  conf: %d; isExitLoop: %d;  isNotExitLoop: %d\n",
           predTagMatch, predLTBidx, predTag, predLTBreadEntry.specCnt, predLTBreadEntry.tripCnt, predLTBreadEntry.conf, io.pred.isExitLoop, io.pred.isNotExitLoop)
  }

  // update
  val updtLTBidx = getLtbIdx(io.update.pc)
  ltb.io.update.readEna := io.update.valid
  ltb.io.update.readIdx := updtLTBidx
  val updtLTBreadEntry = ltb.io.update.readEntry

  val updtTag = getLtbTag(io.update.pc)
  val updtTagMatch = (updtTag === updtLTBreadEntry.tag)
  val updtLTBwriteEntry = WireDefault(updtLTBreadEntry)
  val updtIsUpdateEntry = (io.update.valid && updtTagMatch)
  when(updtIsUpdateEntry && io.update.isMispred) {
    when(io.update.isPredTaken) {
      updtLTBwriteEntry.specCnt := updtLTBreadEntry.specCnt - io.update.predSpecCnt
      updtLTBwriteEntry.tripCnt := io.update.predSpecCnt
      updtLTBwriteEntry.conf    :=  increConf(updtLTBreadEntry.conf) // 1.U // 
    }.elsewhen(io.update.isPredNotTaken) {
      updtLTBwriteEntry.tripCnt := 0.U
      updtLTBwriteEntry.conf    := decreConf(updtLTBreadEntry.conf) // 0.U // 
      updtLTBwriteEntry.specCnt := 1.U
    }
    printf("update---  tag: %x;  crt-spec-cnt: %d;  pred-spec-cnt: %d;  update-spec-cnt: %d;  update-trip-cnt: %d;  update-conf: %d\n", 
            updtTag, updtLTBreadEntry.specCnt, io.update.predSpecCnt, updtLTBwriteEntry.specCnt, updtLTBwriteEntry.tripCnt, updtLTBwriteEntry.conf)
  }
  // .elsewhen(updtIsUpdateEntry && !io.update.isMispred) {
  //   updtLTBwriteEntry.conf    := increConf(updtLTBreadEntry.conf)
  // }
  // .elsewhen(updtIsUpdateEntry && !io.update.isMispred) {
  //   when(io.update.isPredTaken) {
  //     updtLTBwriteEntry.nonSpecCnt := updtLTBreadEntry.nonSpecCnt + 1.U
  //   }.otherwise {
  //     updtLTBwriteEntry.conf       := Mux(updtLTBreadEntry.nonSpecCnt === updtLTBreadEntry.specCnt, 1.U, 0.U)
  //     updtLTBwriteEntry.tripCnt    := updtLTBreadEntry.nonSpecCnt + 1.U
  //     updtLTBwriteEntry.specCnt    := updtLTBreadEntry.specCnt - (updtLTBreadEntry.nonSpecCnt + 1.U)
  //     updtLTBwriteEntry.nonSpecCnt := 0.U
  //   }
  // }

  // val updtIsAllocEntry = (io.update.valid && io.update.isLoopBranch && !updtTagMatch && updtLTBreadEntry.isNotConf)
  val updtIsAllocEntry = (io.update.valid && io.update.isLoopBranch && !updtTagMatch && updtLTBreadEntry.isNotConf && io.update.isUpdateTaken)
  val updtAllocEntry = initLTBentry(updtTag, io.update.target)
  val updtLTBwena = (updtIsAllocEntry || updtIsUpdateEntry)
  ltb.io.update.writeEna := updtLTBwena
  ltb.io.update.writeIdx := updtLTBidx
  ltb.io.update.writeEntry := Mux(updtIsAllocEntry, updtAllocEntry, updtLTBwriteEntry)


  when(io.update.valid) {
    printf("alloc-info--- isLoopBranch: %d;  notUpdtTagMatch: %d;  updtTag: %x;  preTag: %x;  isNotConf: %d;  isUpdateTaken: %d;  updtIsAllocEntry: %d\n",
    io.update.isLoopBranch, !updtTagMatch, updtTag, updtLTBreadEntry.tag, updtLTBreadEntry.isNotConf, io.update.isUpdateTaken, updtIsAllocEntry)
  }

  when(updtIsAllocEntry) {
    printf("allocate-entry--- pc: %x; preTag: %x;  idx: %d\n", io.update.pc, updtLTBreadEntry.tag, updtLTBidx)
    printf("alloc-entry-info--- tag: %x;  spec-cnt: %d;  trip-cnt: %d;  conf: %d\n", 
          updtAllocEntry.tag, updtAllocEntry.specCnt, updtAllocEntry.tripCnt, updtAllocEntry.conf)
  }


  
}


class xsLPpredIO(implicit p: Parameters) extends XSBundle with LoopPredictorParams {
  val valid          = Input(Bool())
  val pc             = Input(UInt(VAddrBits.W))
//   val isConf     = Output(Bool())
//   val isExitLoop = Output(Bool())
  val isConfExitLoop = Output(Bool())
  val target         = Output(UInt(VAddrBits.W))
  val meta           = Output(new LPmeta)
}

class xsLPupdateIO(implicit p: Parameters) extends XSBundle with LoopPredictorParams {
  val valid        = Input(Bool())
  val pc           = Input(UInt(VAddrBits.W))
  val meta         = Input(new LPmeta)
  val taken        = Input(Bool())
  val isLoopBranch = Input(Bool())
  val target       = Input(UInt(VAddrBits.W))
}

class XSLoopPredictor(implicit p: Parameters) extends XSModule with LoopPredictorParams {
  val io = IO(new Bundle{
    val lpEna         = Input(Bool())
    val pred          = new xsLPpredIO
    val update        = new xsLPupdateIO
    val isInterNumGT2 = Output(Bool())
  })

  val lp = Module(new LoopPredictor)

  // prediction
  val lpPredValid = (io.lpEna && io.pred.valid)
  lp.io.pred.valid := lpPredValid
  lp.io.pred.pc    := io.pred.pc

  val lpMeta = WireDefault(0.U.asTypeOf(new LPmeta))
  lpMeta.isLPpred          := lpPredValid
  lpMeta.isNotExitLoop     := (lpPredValid && lp.io.pred.isNotExitLoop)
  lpMeta.isExitLoop        := (lpPredValid && lp.io.pred.isExitLoop)
  lpMeta.isConfNotExitLoop := (lpPredValid && lp.io.pred.isConfNotExitLoop)
  lpMeta.isConfExitLoop    := (lpPredValid && lp.io.pred.isConfExitLoop)
  lpMeta.specCnt           := lp.io.pred.specCnt
  lpMeta.tripCnt           := lp.io.pred.tripCnt

  io.pred.isConfExitLoop := lp.io.pred.isConfExitLoop
  io.pred.target         := lp.io.pred.target
  io.pred.meta           := lpMeta
  io.isInterNumGT2 := (lp.io.pred.tripCnt > lp.io.pred.specCnt && 
                       lp.io.pred.tripCnt - lp.io.pred.specCnt > 2.U)

  
  // update
  val updateMispred = (io.update.meta.isExitLoop    && io.update.taken) || 
                      (io.update.meta.isNotExitLoop && !io.update.taken)
  
  lp.io.update.valid          := (io.lpEna && io.update.valid && io.update.meta.isLPpred)
  lp.io.update.pc             := io.update.pc
  lp.io.update.isLoopBranch   := io.update.isLoopBranch
  lp.io.update.isUpdateTaken  := io.update.taken
  lp.io.update.isPredTaken    := io.update.meta.isNotExitLoop
  lp.io.update.isPredNotTaken := io.update.meta.isExitLoop
  lp.io.update.isMispred      := updateMispred
  lp.io.update.predSpecCnt    := io.update.meta.specCnt
  lp.io.update.target         := io.update.target


  val updateMeta = io.update.meta
  val updateLPhit = (io.update.meta.isExitLoop    && !io.update.taken) || 
                    (io.update.meta.isNotExitLoop && io.update.taken)
  XSPerfAccumulate("lp_provide_direction", io.update.valid && updateMeta.isConfExitLoop)
  XSPerfAccumulate("lp_provide_pred_hit",  io.update.valid && updateMeta.isConfExitLoop && updateLPhit)
  XSPerfAccumulate("lp_provide_pred_miss", io.update.valid && updateMeta.isConfExitLoop && updateMispred)

  val lpConfPredHit  = (io.update.valid && updateMeta.isConfExitLoop    && !io.update.taken) || 
                       (io.update.valid && updateMeta.isConfNotExitLoop && io.update.taken)
  val lpConfPredMiss = (io.update.valid && updateMeta.isConfExitLoop    &&  io.update.taken) || 
                       (io.update.valid && updateMeta.isConfNotExitLoop && !io.update.taken)
  XSPerfAccumulate("lp_conf_prediction", io.update.valid && (updateMeta.isConfExitLoop || updateMeta.isConfNotExitLoop))
  XSPerfAccumulate("lp_conf_pred_hit",   lpConfPredHit)
  XSPerfAccumulate("lp_conf_pred_miss",  lpConfPredMiss)

  generatePerfEvent()

}



