// See LICENSE for license details.
package sifive.fpgashells.devices.xilinx.digilentnexysa7mig

import Chisel._
import freechips.rocketchip.config._
import freechips.rocketchip.subsystem.BaseSubsystem
import freechips.rocketchip.diplomacy.{LazyModule, LazyModuleImp, AddressRange}

case object MemoryDigilentDDRKey extends Field[DigilentNexysA7MIGParams]

trait HasMemoryDigilentNexysA7MIG { this: BaseSubsystem =>
  val module: HasMemoryDigilentNexysA7MIGModuleImp

  val digilentnexysa7mig = LazyModule(new DigilentNexysA7MIG(p(MemoryDigilentDDRKey)))

  digilentnexysa7mig.node := mbus.toDRAMController(Some("digilentnexysa7mig"))()
}

trait HasMemoryDigilentNexysA7MIGBundle {
  val digilentnexysa7mig: DigilentNexysA7MIGIO
  def connectDigilentNexysA7MIGToPads(pads: DigilentNexysA7MIGPads) {
    pads <> digilentnexysa7mig
  }
}

trait HasMemoryDigilentNexysA7MIGModuleImp extends LazyModuleImp
    with HasMemoryDigilentNexysA7MIGBundle {
  val outer: HasMemoryDigilentNexysA7MIG
  val ranges = AddressRange.fromSets(p(MemoryDigilentDDRKey).address)
  require (ranges.size == 1, "DDR range must be contiguous")
  val depth = ranges.head.size
  val digilentnexysa7mig = IO(new DigilentNexysA7MIGIO(depth))

  digilentnexysa7mig <> outer.digilentnexysa7mig.module.io.port
}
