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
  val debugCntBits = 50

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
    newLTBentry.debugCnt   := 1.U
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
  def doPred(oldSpecCnt: UInt, tripCnt: UInt, conf: UInt): UInt =  {
    val newSpecCnt = WireDefault(oldSpecCnt)
    when(conf === 1.U) {
        newSpecCnt := (oldSpecCnt % tripCnt) + 1.U
    }.otherwise {
      newSpecCnt := oldSpecCnt + 1.U
    }
    newSpecCnt
  }

}

class LoopEntry(implicit p: Parameters) extends XSBundle with LoopPredictorParams {
  val tag        = UInt(tagLen.W)
  val specCnt    = UInt(cntBits.W)
  val tripCnt    = UInt(cntBits.W)
  val conf       = UInt(confBits.W)  
  val target     = UInt(VAddrBits.W)
  val debugCnt   = UInt(debugCntBits.W)

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
  val debugCnt          = UInt(debugCntBits.W)
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
  val recover = new LTBrwIO
}

class LTB(implicit p: Parameters) extends XSModule with LoopPredictorParams {
  val io = IO(new LTBio)

  val ltb = RegInit(VecInit(Seq.fill(nRows)(0.U.asTypeOf(new LoopEntry))))
  
  val predReadUpdateWrite    = io.pred.readEna && io.update.writeEna && 
                               (io.pred.readIdx === io.update.writeIdx)
  val predReadRecoverWrite   = io.pred.readEna && io.recover.writeEna && 
                               (io.pred.readIdx === io.recover.writeIdx)
  val recoverReadUpdateWrite = io.recover.readEna && io.update.writeEna && 
                               (io.recover.readIdx === io.update.writeIdx)

  // pred read
  val predReadEntry = WireDefault( ltb(io.pred.readIdx) )
  // tag & tripCnt & conf
  when(predReadUpdateWrite) {
    predReadEntry.tag     := io.update.writeEntry.tag
    predReadEntry.tripCnt := io.update.writeEntry.tripCnt
    predReadEntry.conf    := io.update.writeEntry.conf
  }
  // specCnt
  when(predReadUpdateWrite) {
    predReadEntry.specCnt := io.update.writeEntry.specCnt // allocate, specCnt = 0.U
  }.elsewhen(predReadRecoverWrite) {
    predReadEntry.specCnt := io.recover.writeEntry.specCnt
  }
  when(io.pred.readEna) {
    io.pred.readEntry := predReadEntry
  }.otherwise {
    io.pred.readEntry := 0.U.asTypeOf(new LoopEntry)
  }

  // recover read
  val recoverReadEntry = WireDefault( ltb(io.recover.readIdx) )
  // tag & conf & specCnt
  when(recoverReadUpdateWrite) {
    recoverReadEntry.tag     := io.update.writeEntry.tag
    recoverReadEntry.specCnt := io.update.writeEntry.specCnt
    recoverReadEntry.conf    := io.update.writeEntry.conf
  }
  when(io.recover.readEna) {
    io.recover.readEntry := recoverReadEntry
  }.otherwise{
    io.recover.readEntry := 0.U.asTypeOf(new LoopEntry)
  }

  // update read
  when(io.update.readEna) {
    io.update.readEntry := ltb(io.update.readIdx)
  }.otherwise {
    io.update.readEntry := 0.U.asTypeOf(new LoopEntry)
  }


  val predUpdateWrite    = io.pred.writeEna && io.update.writeEna &&
                           (io.pred.writeIdx === io.update.writeIdx)
  val predRecoverWrite   = io.pred.writeEna && io.recover.writeEna &&
                           (io.pred.writeIdx === io.recover.writeIdx)
  val recoverUpdateWrite = io.recover.writeEna && io.update.writeEna &&
                           (io.recover.writeIdx === io.update.writeIdx)
  // pred write 
  val predWriteEntry = WireDefault(io.pred.writeEntry)
  when(predUpdateWrite) {
    when(io.pred.writeEntry.tag === io.update.writeEntry.tag) {
      predWriteEntry.specCnt := doPred(io.update.writeEntry.specCnt, 
                                       io.update.writeEntry.tripCnt, io.update.writeEntry.conf)
    }.otherwise {
      predWriteEntry.specCnt := io.update.writeEntry.specCnt
    }
    predWriteEntry.tag     := io.update.writeEntry.tag
    predWriteEntry.tripCnt := io.update.writeEntry.tripCnt
    predWriteEntry.conf    := io.update.writeEntry.conf
  }.elsewhen(predRecoverWrite) {
    when(io.pred.writeEntry.tag === io.recover.writeEntry.tag) {
      predWriteEntry.specCnt := doPred(io.recover.writeEntry.specCnt, 
                                       io.recover.writeEntry.tripCnt, io.recover.writeEntry.conf)
    }.otherwise {
      predWriteEntry.specCnt := io.recover.writeEntry.specCnt
    }
    predWriteEntry.tag     := io.recover.writeEntry.tag
    predWriteEntry.tripCnt := io.recover.writeEntry.tripCnt
    predWriteEntry.conf    := io.recover.writeEntry.conf
  }
  when(io.pred.writeEna) {
    ltb(io.pred.writeIdx) := predWriteEntry
  }

  // recover write
  when(io.recover.writeEna) {
    when(recoverUpdateWrite) {
      ltb(io.recover.writeIdx) := io.update.writeEntry
    }.otherwise {
      ltb(io.recover.writeIdx) := io.recover.writeEntry
    }
  }

  // update write
  when(io.update.writeEna) {
    ltb(io.update.writeIdx) := io.update.writeEntry
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
  val debugCnt          = Output(UInt(debugCntBits.W))
}

class lpRedirectInfo(implicit p: Parameters) extends XSBundle with LoopPredictorParams {
  val startPC        = UInt(VAddrBits.W)
  val isLPpred       = Bool()
  val predSpecCnt    = UInt(cntBits.W)
  val isPredTaken    = Bool()
  val isPredNotTaken = Bool()
  val debugCnt       = UInt(debugCntBits.W)
}

class lpRedirectIO(implicit p: Parameters) extends XSBundle with LoopPredictorParams {
  val valid  = Input(Bool())  
  // val pc     = Input(UInt(VAddrBits.W))
  val lpInfo = Input(new lpRedirectInfo)
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
  val debugCnt       = Input(UInt(debugCntBits.W))
}

class commitSpecCntEntry(implicit p: Parameters) extends XSBundle with LoopPredictorParams {
  val tag      = UInt(tagLen.W)
  val spcCnt   = UInt(cntBits.W)
  val notTaken = Bool()
}

class LoopPredictor(implicit p: Parameters) extends XSModule with LoopPredictorParams {
  val io = IO(new Bundle{
    val pred     = new lpPredIO
    val redirect = new lpRedirectIO
    val update   = new lpUpdateIO
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
   when(predLTBwena) {
    predLTBwriteEntry.specCnt := doPred(predLTBreadEntry.specCnt, predLTBreadEntry.tripCnt, predLTBreadEntry.conf)
    predLTBwriteEntry.debugCnt := predLTBreadEntry.debugCnt + 1.U
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
  io.pred.debugCnt := predLTBwriteEntry.debugCnt

  when(io.pred.valid) {
    printf("prediction  tagMatch: %d;  idx: %d;  pc: %x; tag: %x;  " +
      "spec-cnt: %d;  trip_cnt: %d;  conf: %d; isExitLoop: %d; " +
      "isNotExitLoop: %d; debugCnt: %d\n",
      predTagMatch, predLTBidx, io.pred.pc, predTag, predLTBwriteEntry.specCnt, 
      predLTBreadEntry.tripCnt, predLTBreadEntry.conf, io.pred.isExitLoop, 
      io.pred.isNotExitLoop, predLTBwriteEntry.debugCnt)
  }

  // // redirect
  val redrctStartPC = io.redirect.lpInfo.startPC
  val redrctLTBidx = getLtbIdx(redrctStartPC)
  ltb.io.recover.readEna := io.redirect.valid
  ltb.io.recover.readIdx := redrctLTBidx
  val redrctLTBreadEntry = ltb.io.recover.readEntry

  val redrctTag = getLtbTag(redrctStartPC)
  val redrctTagMatch = (redrctTag === redrctLTBreadEntry.tag)
  val redrctLTBwriteEntry = WireDefault(redrctLTBreadEntry)
  val redrctIsWriteLTB = io.redirect.valid && 
                          io.redirect.lpInfo.isLPpred && redrctTagMatch
  when(redrctIsWriteLTB) {
    when(redrctLTBreadEntry.conf === 1.U) {
      when(io.redirect.lpInfo.isPredTaken) { 
        redrctLTBwriteEntry.specCnt := 0.U
      }.otherwise {
        redrctLTBwriteEntry.specCnt := Mux(io.redirect.lpInfo.predSpecCnt === redrctLTBreadEntry.tripCnt, 
                                           io.redirect.lpInfo.predSpecCnt, 
                                           (io.redirect.lpInfo.predSpecCnt % redrctLTBreadEntry.tripCnt) )
      }
    }.otherwise {
      redrctLTBwriteEntry.specCnt := io.redirect.lpInfo.predSpecCnt
    }
  
    printf("rdrct-update-specCnt pc: %x; preSpecCnt: %d; predTaken: %d; " +
        "recv-tripCnt: %d; recv-conf: %d; new-spcCnt: %d; crt-specCnt: %d; " +
        "conf: %d; pred-debugCnt: %d; crt-debugCnt: %d\n", 
    redrctStartPC, io.redirect.lpInfo.predSpecCnt, io.redirect.lpInfo.isPredTaken, 
    redrctLTBwriteEntry.tripCnt, redrctLTBwriteEntry.conf,
    redrctLTBwriteEntry.specCnt, redrctLTBreadEntry.specCnt, redrctLTBreadEntry.conf, 
    io.redirect.lpInfo.debugCnt, redrctLTBreadEntry.debugCnt)
  }
  ltb.io.recover.writeEna   := redrctIsWriteLTB
  ltb.io.recover.writeIdx   := redrctLTBidx
  ltb.io.recover.writeEntry := redrctLTBwriteEntry


  // update
  val commitSpecCntArray = RegInit(VecInit(Seq.fill(nRows)(0.U.asTypeOf(new commitSpecCntEntry))))

  val updtLTBidx = getLtbIdx(io.update.pc)
  ltb.io.update.readEna := io.update.valid
  ltb.io.update.readIdx := updtLTBidx
  val updtLTBreadEntry = ltb.io.update.readEntry
  val updtCSCAreadEntry = commitSpecCntArray(updtLTBidx)

  val updtTag = getLtbTag(io.update.pc)
  val updtTagMatch   = (updtTag === updtLTBreadEntry.tag)
  // val updtCSCATagMatch  = (updtTag === updtCSCAreadEntry.tag)
  val updtLTBwriteEntry = WireDefault(updtLTBreadEntry)
  val setTripCnt = (io.update.valid && updtTagMatch && updtCSCAreadEntry.notTaken && 
                    !io.update.isUpdateTaken && updtLTBreadEntry.conf === 0.U)
  val resetConf = (io.update.valid && updtTagMatch && 
                   io.update.isMispred && updtLTBreadEntry.conf === 1.U)

  val updtIsAllocEntry = (io.update.valid && io.update.isLoopBranch && !updtTagMatch && 
                          updtLTBreadEntry.isNotConf && io.update.isUpdateTaken)
  val updtAllocEntry = initLTBentry(updtTag, io.update.target)
  // val updtLTBwena = (updtIsAllocEntry || setTripCnt || resetConf)
  val updtLTBwena = (updtIsAllocEntry || setTripCnt)

  // write commitSpecCntArray
  val updateSpecArray = io.update.valid && updtTagMatch && 
                        !io.update.isUpdateTaken && updtLTBreadEntry.conf === 0.U
  val resetNotTaken = updtTagMatch && updtLTBreadEntry.conf === 1.U
  when(updateSpecArray) {
    commitSpecCntArray(updtLTBidx).spcCnt   := io.update.predSpecCnt
    commitSpecCntArray(updtLTBidx).notTaken := true.B
  }.elsewhen(resetNotTaken) {
    commitSpecCntArray(updtLTBidx).notTaken := false.B
  }

  // update tripCnt and conf
  when(setTripCnt) {
    updtLTBwriteEntry.tripCnt := io.update.predSpecCnt - updtCSCAreadEntry.spcCnt
    updtLTBwriteEntry.conf    := 1.U //increConf(updtLTBreadEntry.conf)

    printf("set-tripCnt pc: %x; update-tripCnt: %d; old-tripCnt: %d; " +
      "cpdt-specCnt: %d; old-specCnt: %d; pred-debugCnt: %d; " +
      "crt--debugCnt: %d;\n", 
    io.update.pc, updtLTBwriteEntry.tripCnt, updtLTBreadEntry.tripCnt,
    io.update.predSpecCnt, updtCSCAreadEntry.spcCnt, 
    io.update.debugCnt, updtLTBreadEntry.debugCnt)
  }
  // when(resetConf) {
  //   updtLTBwriteEntry.conf := 0.U
  // }

  ltb.io.update.writeEna := updtLTBwena
  ltb.io.update.writeIdx := updtLTBidx
  ltb.io.update.writeEntry := Mux(updtIsAllocEntry, updtAllocEntry, updtLTBwriteEntry)



  when(io.update.valid) {
    printf("alloc-info--- isLoopBranch: %d;  notUpdtTagMatch: %d;  updtTag: %x;  " +
      "preTag: %x;  isNotConf: %d;  isUpdateTaken: %d;  updtIsAllocEntry: %d; " +
      "pred-debugCnt: %d; crt--debugCnt: %d;\n",
    io.update.isLoopBranch, !updtTagMatch, updtTag, updtLTBreadEntry.tag, 
    updtLTBreadEntry.isNotConf, io.update.isUpdateTaken, updtIsAllocEntry, 
    io.update.debugCnt, updtLTBreadEntry.debugCnt)
  }

  when(updtIsAllocEntry) {
    printf("allocate-entry--- pc: %x; preTag: %x;  idx: %d; " +
      "pred-debugCnt: %d; crt--debugCnt: %d;\n", 
      io.update.pc, updtLTBreadEntry.tag, updtLTBidx, 
      io.update.debugCnt, updtLTBreadEntry.debugCnt)

    printf("alloc-entry-info--- tag: %x;  spec-cnt: %d;  trip-cnt: %d;" +
      " conf: %d; pred-debugCnt: %d; crt--debugCnt: %d;\n", 
        updtAllocEntry.tag, updtAllocEntry.specCnt, updtAllocEntry.tripCnt, 
        updtAllocEntry.conf, io.update.debugCnt, updtLTBreadEntry.debugCnt)
  }


  
}


class xsLPpredIO(implicit p: Parameters) extends XSBundle with LoopPredictorParams {
  val valid          = Input(Bool())
  val pc             = Input(UInt(VAddrBits.W))
  val isConf         = Output(Bool())
  val isConfExitLoop = Output(Bool())
  val target         = Output(UInt(VAddrBits.W))
  val meta           = Output(new LPmeta)
  val remainIterNum  = Output(UInt(cntBits.W))
  val lpInfo         = Output(new lpRedirectInfo)
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
    val redirect      = new lpRedirectIO
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
  lpMeta.debugCnt          := lp.io.pred.debugCnt

  val isConf = (lp.io.pred.isConfExitLoop || lp.io.pred.isConfNotExitLoop)
  io.pred.isConf         := isConf
  io.pred.isConfExitLoop := lp.io.pred.isConfExitLoop
  io.pred.target         := lp.io.pred.target
  io.pred.meta           := lpMeta
  io.isInterNumGT2 := (lp.io.pred.tripCnt > lp.io.pred.specCnt && 
                       lp.io.pred.tripCnt - lp.io.pred.specCnt > 2.U)
  io.pred.remainIterNum := Mux(isConf, lp.io.pred.tripCnt - lp.io.pred.specCnt, 0.U)

  // redirect info 
  io.pred.lpInfo.startPC        := io.pred.pc
  io.pred.lpInfo.isLPpred       := lpPredValid
  io.pred.lpInfo.predSpecCnt    := lp.io.pred.specCnt
  io.pred.lpInfo.isPredTaken    := lp.io.pred.isNotExitLoop
  io.pred.lpInfo.isPredNotTaken := lp.io.pred.isExitLoop
  io.pred.lpInfo.debugCnt       := lp.io.pred.debugCnt


  // redirect
  lp.io.redirect := io.redirect

  
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
  lp.io.update.debugCnt       := io.update.meta.debugCnt


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

  when(io.pred.valid) {
    printf("lp-pred-meta-- pc: %x; " +
      "isLPpred: %d; isNotExitLoop: %d; " +
      "isExitLoop: %d; isConfNotExitLoop: %d; " +
      "isConfExitLoop: %d; specCnt: %d; tripCnt: %d\n",
    io.pred.pc,
    io.pred.meta.isLPpred,       io.pred.meta.isNotExitLoop,
    io.pred.meta.isExitLoop,     io.pred.meta.isConfNotExitLoop,
    io.pred.meta.isConfExitLoop, io.pred.meta.specCnt,    
    io.pred.meta.tripCnt)
  }

  when(io.update.valid) {
    printf("updateMeta-- pc: %x;  isLPpred: %d;  isNotExitLoop: %d;  isExitLoop: %d;" +
           "isConfNotExitLoop: %d;  isConfExitLoop: %d;  specCnt: %d;  tripCnt: %d\n",
           io.update.pc, updateMeta.isLPpred, updateMeta.isNotExitLoop, 
           updateMeta.isExitLoop, updateMeta.isConfNotExitLoop,
           updateMeta.isConfExitLoop, updateMeta.specCnt, updateMeta.tripCnt)

    printf("mispred-info-- predExit: %d; predNotExit: %d; " +
      "updateTaken: %d; misPred: %d; lpUpdate-valid: %d\n",
      io.update.meta.isExitLoop, io.update.meta.isNotExitLoop, io.update.taken, 
      updateMispred, lp.io.update.valid)
  }


}



