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
package xiangshan.mem

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import utils._
import utility._
import xiangshan._

sealed trait MemUnitType

case class AtomicUnit() extends MemUnitType
case class LoadUnit() extends MemUnitType
case class StoreUnit() extends MemUnitType
case class HybridUnit() extends MemUnitType

case class MemUnitParams(
  name: String = "MemUnit",
  unitType: MemUnitType,
  dataBits: Int = 64,
  hasIntPort: Boolean = false,
  hasVecPort: Boolean = false,
  hasStoreForward: Boolean = false,
  hasBusForward: Boolean = false,
  hasMSHRForward: Boolean = false,
  hasFastReplay: Boolean = false,
  hasLoadToLoad: Boolean = false,
  hasReplay: Boolean = false,
  hasPrefetch: Boolean = false,
  hasMisalign: Boolean = false
) {
  def isAtomicUnit: Boolean = unitType == AtomicUnit()

  def isLoadUnit: Boolean = unitType == LoadUnit()

  def isStoreUnit: Boolean = unitType == StoreUnit()

  def isHybridUnit: Boolean = unitType == HybridUnit()

  def hasDCacheReq: Boolean = isLoadUnit || isHybridUnit || isStoreUnit && hasPrefetch

  def hasDCacheResp: Boolean = isLoadUnit || isHybridUnit

  def hasDebugInfo: Boolean = isLoadUnit || isStoreUnit || isHybridUnit

  def hasTopDownInfo: Boolean = isLoadUnit || isHybridUnit

  def unitTypeString: String = unitType match {
    case StoreUnit() => "Store"
    case LoadUnit() => "Load"
    case HybridUnit() => "Hybrid"
    case AtomicsUnit() => "Atomics"
    case _ => "unknown"
  }


}