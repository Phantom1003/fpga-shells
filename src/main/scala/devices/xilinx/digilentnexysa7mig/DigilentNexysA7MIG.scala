// See LICENSE for license details.
package sifive.fpgashells.devices.xilinx.digilentnexysa7mig

import Chisel._
import chisel3.experimental.{Analog,attach}
import freechips.rocketchip.amba.axi4._
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.subsystem._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.interrupts._
import sifive.fpgashells.ip.xilinx.nexysa7mig.{NexysA7MIGIOClocksReset, NexysA7MIGIODDR, nexysa7mig}

case class DigilentNexysA7MIGParams(
  address : Seq[AddressSet]
)

class DigilentNexysA7MIGPads(depth : BigInt) extends NexysA7MIGIODDR(depth) {
  def this(c : DigilentNexysA7MIGParams) {
    this(AddressRange.fromSets(c.address).head.size)
  }
  override def cloneType = (new DigilentNexysA7MIGPads(depth)).asInstanceOf[this.type]
}

class DigilentNexysA7MIGIO(depth : BigInt) extends NexysA7MIGIODDR(depth) with NexysA7MIGIOClocksReset {
  override def cloneType = (new DigilentNexysA7MIGIO(depth)).asInstanceOf[this.type]
}

class DigilentNexysA7MIGIsland(c : DigilentNexysA7MIGParams, val crossing: ClockCrossingType = AsynchronousCrossing(8))(implicit p: Parameters) extends LazyModule with CrossesToOnlyOneClockDomain {
  val ranges = AddressRange.fromSets(c.address)
  require (ranges.size == 1, "DDR range must be contiguous")
  val offset = ranges.head.base
  val depth = ranges.head.size
  require((depth<=0x10000000L),"artymig supports upto 256 MB depth configuraton")
  
  val device = new MemoryDevice
  val node = AXI4SlaveNode(Seq(AXI4SlavePortParameters(
      slaves = Seq(AXI4SlaveParameters(
      address       = c.address,
      resources     = device.reg,
      regionType    = RegionType.UNCACHED,
      executable    = true,
      supportsWrite = TransferSizes(1, 64),
      supportsRead  = TransferSizes(1, 64))),
    beatBytes = 8)))

  lazy val module = new LazyRawModuleImp(this) {
    val io = IO(new Bundle {
      val port = new DigilentNexysA7MIGIO(depth)
    })

    childClock := io.port.ui_clk
    childReset := io.port.ui_clk_sync_rst

    val blackbox = Module(new nexysa7mig(depth))
    val (axi_async, _) = node.in(0)

    //inouts
    attach(io.port.ddr2_dq, blackbox.io.ddr2_dq)
    attach(io.port.ddr2_dqs_n, blackbox.io.ddr2_dqs_n)
    attach(io.port.ddr2_dqs_p, blackbox.io.ddr2_dqs_p)

    //outputs
    io.port.ddr2_addr         := blackbox.io.ddr2_addr
    io.port.ddr2_ba           := blackbox.io.ddr2_ba
    io.port.ddr2_ras_n        := blackbox.io.ddr2_ras_n
    io.port.ddr2_cas_n        := blackbox.io.ddr2_cas_n
    io.port.ddr2_we_n         := blackbox.io.ddr2_we_n
    io.port.ddr2_ck_p         := blackbox.io.ddr2_ck_p
    io.port.ddr2_ck_n         := blackbox.io.ddr2_ck_n
    io.port.ddr2_cke          := blackbox.io.ddr2_cke
    io.port.ddr2_cs_n         := blackbox.io.ddr2_cs_n
    io.port.ddr2_dm           := blackbox.io.ddr2_dm
    io.port.ddr2_odt          := blackbox.io.ddr2_odt

    //inputs
    //NO_BUFFER clock
    blackbox.io.sys_clk_i     := io.port.sys_clk_i

    io.port.ui_clk            := blackbox.io.ui_clk
    io.port.ui_clk_sync_rst   := blackbox.io.ui_clk_sync_rst
    blackbox.io.aresetn       := io.port.aresetn
    blackbox.io.app_sr_req    := Bool(false)
    blackbox.io.app_ref_req   := Bool(false)
    blackbox.io.app_zq_req    := Bool(false)
    blackbox.io.device_temp_i := UInt(0)
    //app_sr_active           := unconnected
    //app_ref_ack             := unconnected
    //app_zq_ack              := unconnected

    val awaddr = axi_async.aw.bits.addr - UInt(offset)
    val araddr = axi_async.ar.bits.addr - UInt(offset)

    //slave AXI interface write address ports
    blackbox.io.s_axi_awid    := axi_async.aw.bits.id
    blackbox.io.s_axi_awaddr  := awaddr //truncated
    blackbox.io.s_axi_awlen   := axi_async.aw.bits.len
    blackbox.io.s_axi_awsize  := axi_async.aw.bits.size
    blackbox.io.s_axi_awburst := axi_async.aw.bits.burst
    blackbox.io.s_axi_awlock  := axi_async.aw.bits.lock
    blackbox.io.s_axi_awcache := UInt("b0011")
    blackbox.io.s_axi_awprot  := axi_async.aw.bits.prot
    blackbox.io.s_axi_awqos   := axi_async.aw.bits.qos
    blackbox.io.s_axi_awvalid := axi_async.aw.valid
    axi_async.aw.ready        := blackbox.io.s_axi_awready

    //slave interface write data ports
    blackbox.io.s_axi_wdata   := axi_async.w.bits.data
    blackbox.io.s_axi_wstrb   := axi_async.w.bits.strb
    blackbox.io.s_axi_wlast   := axi_async.w.bits.last
    blackbox.io.s_axi_wvalid  := axi_async.w.valid
    axi_async.w.ready         := blackbox.io.s_axi_wready

    //slave interface write response
    blackbox.io.s_axi_bready  := axi_async.b.ready
    axi_async.b.bits.id       := blackbox.io.s_axi_bid
    axi_async.b.bits.resp     := blackbox.io.s_axi_bresp
    axi_async.b.valid         := blackbox.io.s_axi_bvalid

    //slave AXI interface read address ports
    blackbox.io.s_axi_arid    := axi_async.ar.bits.id
    blackbox.io.s_axi_araddr  := araddr // truncated
    blackbox.io.s_axi_arlen   := axi_async.ar.bits.len
    blackbox.io.s_axi_arsize  := axi_async.ar.bits.size
    blackbox.io.s_axi_arburst := axi_async.ar.bits.burst
    blackbox.io.s_axi_arlock  := axi_async.ar.bits.lock
    blackbox.io.s_axi_arcache := UInt("b0011")
    blackbox.io.s_axi_arprot  := axi_async.ar.bits.prot
    blackbox.io.s_axi_arqos   := axi_async.ar.bits.qos
    blackbox.io.s_axi_arvalid := axi_async.ar.valid
    axi_async.ar.ready        := blackbox.io.s_axi_arready

    //slace AXI interface read data ports
    blackbox.io.s_axi_rready  := axi_async.r.ready
    axi_async.r.bits.id       := blackbox.io.s_axi_rid
    axi_async.r.bits.data     := blackbox.io.s_axi_rdata
    axi_async.r.bits.resp     := blackbox.io.s_axi_rresp
    axi_async.r.bits.last     := blackbox.io.s_axi_rlast
    axi_async.r.valid         := blackbox.io.s_axi_rvalid

    //misc
    io.port.init_calib_complete := blackbox.io.init_calib_complete
    blackbox.io.sys_rst       :=io.port.sys_rst
    //mig.device_temp         :- unconnceted
  }
}

class DigilentNexysA7MIG(c : DigilentNexysA7MIGParams, crossing: ClockCrossingType = AsynchronousCrossing(8))(implicit p: Parameters) extends LazyModule {
  val ranges = AddressRange.fromSets(c.address)
  val depth = ranges.head.size

  val buffer  = LazyModule(new TLBuffer)
  val toaxi4  = LazyModule(new TLToAXI4(adapterName = Some("mem")))
  val indexer = LazyModule(new AXI4IdIndexer(idBits = 4))
  val deint   = LazyModule(new AXI4Deinterleaver(p(CacheBlockBytes)))
  val yank    = LazyModule(new AXI4UserYanker)
  val island  = LazyModule(new DigilentNexysA7MIGIsland(c, crossing))

  val node: TLInwardNode =
    island.crossAXI4In(island.node) := yank.node := deint.node := indexer.node := toaxi4.node := buffer.node

  lazy val module = new LazyModuleImp(this) {
    val io = IO(new Bundle {
      val port = new DigilentNexysA7MIGIO(depth)
    })

    io.port <> island.module.io.port
  }
}