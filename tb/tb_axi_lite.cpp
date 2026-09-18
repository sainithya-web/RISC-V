*
------------------------------------------------------------------------------
 Author       : Deepak
 Designation  : Sr. VLSI Engineer
 Organization : NIELIT CoE
------------------------------------------------------------------------------

File: tb_axi_lite.cpp

Purpose
-------
Verilator C++ standalone testbench for the AXI-Lite protocol layer.

DUT: uart_axi (via tb_axi_lite_wrap.v)
  uart_rx = 1'b1 (line idle)
  Focus: AXI-Lite write/read handshake correctness, backpressure, RDATA values

Tests
------
  T1  single_write         Write 0x41 to offset 0x00; verify BRESP=OKAY
  T2  single_read_status   Read offset 0x08 (STATUS); rx_buf_valid=0 expected
  T3  single_read_data     Read offset 0x00 (DATA); bit[8]=rx_buf_valid=0
  T4  back_to_back_writes  3 successive writes; all must complete without error
  T5  write_then_poll_aw   After write, awready must be LOW while buf_valid=1
  T6  multiple_reads       Repeated STATUS reads; all must return RRESP=OKAY
  T7  wstrb_byte_lanes     Write with different wstrb values; no crash/deadlock

Compile / Run
--------------
  make axilite_sim
*/
// =============================================================================
// tb_axi_lite.cpp – AXI-Lite Verilator C++ Testbench (protocol-level)
// =============================================================================

#include "Vtb_axi_lite_wrap.h"
#include "verilated.h"
#include "verilated_vcd_c.h"

#include <cstdint>
#include <cstdio>
#include <cstring>

// ─────────────────────────────────────────────────────────────────────────────
// uart_axi Register Offsets
// ─────────────────────────────────────────────────────────────────────────────
#define REG_DATA    0x00u   // TX write / RX read
#define REG_STATUS  0x08u   // STATUS: bit[1]=rx_buf_valid (read-only, no clear)

// ─────────────────────────────────────────────────────────────────────────────
// Simulation globals
// ─────────────────────────────────────────────────────────────────────────────
#define RESET_CYCLES  20
// uart_axi stalls awready/wready while buf_valid=1 (TX serialising).
// One UART frame at 50 MHz/115200 baud ≈ 4340 cycles.  Use a generous
// 10 000 cycle timeout so back-to-back writes can drain the TX buffer.
#define AXI_TIMEOUT   10000  // max cycles per handshake step

static vluint64_t      sim_time = 0;
static Vtb_axi_lite_wrap *dut  = nullptr;
static VerilatedVcdC   *tfp    = nullptr;

double sc_time_stamp() { return (double)sim_time; }

// ─────────────────────────────────────────────────────────────────────────────
// Clock tick
// ─────────────────────────────────────────────────────────────────────────────
static void tick()
{
    dut->clk = 0;
    dut->eval();
    if (tfp) tfp->dump(sim_time);
    sim_time++;

    dut->clk = 1;
    dut->eval();
    if (tfp) tfp->dump(sim_time);
    sim_time++;
}


// ─────────────────────────────────────────────────────────────────────────────
// AXI-Lite Write
//   Returns true if transaction completed with BRESP=OKAY.
//   Drives AW+W simultaneously.
// ─────────────────────────────────────────────────────────────────────────────
static bool axi_write(uint32_t addr, uint32_t data,
                      uint8_t strb = 0xF, uint8_t *bresp_out = nullptr)
{
    dut->s_axi_awaddr  = addr;
    dut->s_axi_awvalid = 1;
    dut->s_axi_wdata   = data;
    dut->s_axi_wstrb   = strb;
    dut->s_axi_wvalid  = 1;
    dut->s_axi_bready  = 0;

    bool aw_done = false, w_done = false;
    int  cnt = 0;

    while ((!aw_done || !w_done) && cnt < AXI_TIMEOUT) {
        tick();
        if (dut->s_axi_awready && dut->s_axi_awvalid) {
            aw_done            = true;
            dut->s_axi_awvalid = 0;
        }
        if (dut->s_axi_wready && dut->s_axi_wvalid) {
            w_done            = true;
            dut->s_axi_wvalid = 0;
        }
        cnt++;
    }

    if (!aw_done || !w_done) {
        printf("[ERROR] axi_write: AW/W timeout at addr=0x%08X\n", addr);
        dut->s_axi_awvalid = 0;
        dut->s_axi_wvalid  = 0;
        return false;
    }

    // B-channel
    dut->s_axi_bready = 1;
    cnt = 0;
    while (!dut->s_axi_bvalid && cnt < AXI_TIMEOUT) { tick(); cnt++; }
    uint8_t bresp = dut->s_axi_bresp;
    if (bresp_out) *bresp_out = bresp;
    tick();
    dut->s_axi_bready = 0;

    if (cnt >= AXI_TIMEOUT) {
        printf("[ERROR] axi_write: B-channel timeout at addr=0x%08X\n", addr);
        return false;
    }
    return (bresp == 0);
}

// ─────────────────────────────────────────────────────────────────────────────
// AXI-Lite Read
// ─────────────────────────────────────────────────────────────────────────────
static uint32_t axi_read(uint32_t addr, uint8_t *rresp_out = nullptr)
{
    dut->s_axi_araddr  = addr;
    dut->s_axi_arvalid = 1;
    dut->s_axi_rready  = 1;

    int cnt = 0;
    while (!dut->s_axi_arready && cnt < AXI_TIMEOUT) { tick(); cnt++; }
    tick();
    dut->s_axi_arvalid = 0;

    cnt = 0;
    while (!dut->s_axi_rvalid && cnt < AXI_TIMEOUT) { tick(); cnt++; }
    uint32_t rdata = dut->s_axi_rdata;
    uint8_t  rresp = dut->s_axi_rresp;
    if (rresp_out) *rresp_out = rresp;
    tick();
    dut->s_axi_rready = 0;

    if (cnt >= AXI_TIMEOUT)
        printf("[ERROR] axi_read: timeout at addr=0x%08X\n", addr);

    return rdata;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test helpers
// ─────────────────────────────────────────────────────────────────────────────
static int g_tests_run    = 0;
static int g_tests_passed = 0;

static void check(bool cond, const char *name)
{
    g_tests_run++;
    if (cond) {
        g_tests_passed++;
        printf("  [PASS] %s\n", name);
    } else {
        printf("  [FAIL] %s\n", name);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
                                                               // ─────────────────────────────────────────────────────────────────────────────
// main()
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char **argv)
{
    Verilated::commandArgs(argc, argv);

    dut = new Vtb_axi_lite_wrap;
                                                                                                 
