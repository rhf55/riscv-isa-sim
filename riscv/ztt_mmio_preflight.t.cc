// Copyright (c) 2026 BOSC & ICT, CAS
// All rights reserved.
// See LICENSE for license details.

#include "abstract_interrupt_controller.h"
#include "cfg.h"
#include "devices.h"
#include "imsic.h"
#include "platform.h"
#include "processor.h"
#include "simif.h"
#include "trap.h"
#include "ztt_execute.h"
#include "ztt_state.h"
#include <cassert>
#include <cstdint>
#include <cstring>
#include <map>
#include <memory>
#include <sstream>
#include <vector>

namespace {

constexpr uint32_t kU32 = 32;

class counting_intc_t : public abstract_interrupt_controller_t {
 public:
  void set_interrupt_level(uint32_t, int) override { ++calls; }
  int calls = 0;
};

class test_sim_t final : public simif_t {
 public:
  test_sim_t() : memory(4096, 0) { debug_mmu = nullptr; }

  char* addr_to_mem(reg_t address) override
  {
    if (address < DRAM_BASE || address - DRAM_BASE >= memory.size())
      return nullptr;
    return reinterpret_cast<char*>(memory.data() + (address - DRAM_BASE));
  }
  bool mmio_load(reg_t, size_t, uint8_t*) override { return false; }
  bool mmio_store(reg_t address, size_t len, const uint8_t* bytes) override
  {
    if (address + len < address)
      return false;
    ++mmio_store_calls;
    return bus.store(address, len, bytes);
  }
  bool mmio_store_preflight(reg_t address, size_t len) override
  {
    if (address + len < address)
      return false;
    return bus.store_preflight(address, len);
  }
  void proc_reset(unsigned) override {}
  const cfg_t& get_cfg() const override { return cfg; }
  const std::map<size_t, processor_t*>& get_harts() const override
    { return harts; }
  const char* get_symbol(uint64_t) override { return nullptr; }

  cfg_t cfg;
  bus_t bus;
  std::vector<uint8_t> memory;
  std::map<size_t, processor_t*> harts;
  std::size_t mmio_store_calls = 0;
};

insn_t operands(unsigned rd, unsigned rs1 = 0, unsigned rs2 = 0)
{
  return insn_t((insn_bits_t(rs2) << 20) |
                (insn_bits_t(rs1) << 15) |
                (insn_bits_t(rd) << 7));
}

class machine_t {
 public:
  explicit machine_t(const char* isa = "rv64i_ztt")
    : proc(isa, "M", &sim.cfg, &sim, 0, false, nullptr, output),
      clint(&sim, 1000000, false),
      uart(&intc, NS16550_INTERRUPT_ID, NS16550_REG_SHIFT, NS16550_REG_IO_WIDTH),
      wide_uart(&intc, NS16550_INTERRUPT_ID, 0, 4),
      imsic_file(std::make_shared<imsic_file_t>(&proc, MIP_MEIP, IMSIC_NUM_EI_REGS)),
      imsic(imsic_file),
      aplic(&sim, nullptr),
      plic(&sim, 1)
  {
    sim.harts.emplace(0, &proc);
    proc.get_state()->mstatus->write(reg_t(1) << 25);
    execute_ztt(proc, ztt_opcode_t::ame_acquire, operands(1));
    assert(proc.ZTU.owned());
    sim.bus.add_device(CLINT_BASE, &clint);
    sim.bus.add_device(NS16550_BASE, &uart);
    sim.bus.add_device(NS16550_BASE + 0x1000, &wide_uart);
    sim.bus.add_device(IMSIC_M_BASE, &imsic);
    sim.bus.add_device(APLIC_S_BASE, &aplic);
  }

  test_sim_t sim;
  std::ostringstream output;
  processor_t proc;
  counting_intc_t intc;
  clint_t clint;
  ns16550_t uart;
  ns16550_t wide_uart;
  std::shared_ptr<imsic_file_t> imsic_file;
  imsic_mmio_t imsic;
  aplic_t aplic;
  plic_t plic;
};

std::vector<ztt_unit_t::element_t> sequence(uint32_t dtype,
                                            ztt_unit_t::element_t first)
{
  std::vector<ztt_unit_t::element_t> values(ztt_unit_t::element_count(dtype));
  for (std::size_t index = 0; index < values.size(); ++index)
    values[index] = first + index;
  return values;
}

void set_m(processor_t& proc, unsigned reg, uint32_t dtype,
           const std::vector<ztt_unit_t::element_t>& values)
{
  assert(proc.ZTU.set_m_datatype(reg, dtype));
  assert(proc.ZTU.write_m(reg, values));
}

void test_preflight_has_no_side_effects(machine_t& machine)
{
  const uint64_t mtime = machine.clint.get_mtime();
  const uint64_t mtimecmp = machine.clint.get_mtimecmp(0);
  const reg_t mip = machine.proc.get_state()->mip->read();
  uint8_t scratch = 0xff;
  assert(machine.uart.load(7, 1, &scratch));
  const int interrupts = machine.intc.calls;
  uint8_t domain[4];
  assert(machine.aplic.load(0, 4, domain));
  const reg_t eip = machine.imsic_file->get_reg(IMSIC_EIP(0))->read();

  assert(machine.clint.store_preflight(0, 1));
  assert(machine.clint.store_preflight(0, 4));
  assert(machine.clint.store_preflight(0, 8));
  assert(!machine.clint.store_preflight(0, 5));
  assert(!machine.clint.store_preflight(0, 9));
  assert(!machine.clint.store_preflight(CLINT_SIZE - 1, 2));
  assert(!machine.clint.store_preflight(~reg_t(0), 1));
  assert(machine.uart.store_preflight(7, 1));
  assert(!machine.uart.store_preflight(7, 4));
  assert(!machine.uart.store_preflight(NS16550_SIZE - 1, 2));
  assert(machine.wide_uart.store_preflight(7, 4));
  assert(!machine.wide_uart.store_preflight(7, 1));
  assert(!machine.imsic.store_preflight(SETEIPNUM_LE, 1));
  assert(!machine.imsic.store_preflight(SETEIPNUM_LE, 4));
  assert(!machine.aplic.store_preflight(0, 4));
  assert(!machine.plic.store_preflight(0, 4));
  rom_device_t rom(std::vector<char>(16, 'a'));
  assert(!rom.store_preflight(0, 1));

  assert(machine.sim.bus.store_preflight(CLINT_BASE, 1));
  assert(machine.sim.bus.store_preflight(NS16550_BASE + 7, 1));
  assert(!machine.sim.bus.store_preflight(NS16550_BASE + 7, 4));
  assert(!machine.sim.bus.store_preflight(NS16550_BASE + NS16550_SIZE - 1, 2));
  assert(!machine.sim.bus.store_preflight(IMSIC_M_BASE, 4));
  assert(!machine.sim.bus.store_preflight(APLIC_S_BASE, 4));
  assert(!machine.sim.bus.store_preflight(0x50000000, 1));
  assert(!machine.sim.mmio_store_preflight(~reg_t(0), 2));
  assert(machine.sim.mmio_store_calls == 0);

  uint8_t scratch_after = 0;
  uint8_t domain_after[4];
  assert(machine.uart.load(7, 1, &scratch_after));
  assert(machine.aplic.load(0, 4, domain_after));
  assert(machine.clint.get_mtime() == mtime);
  assert(machine.clint.get_mtimecmp(0) == mtimecmp);
  assert(machine.proc.get_state()->mip->read() == mip);
  assert(scratch_after == scratch);
  assert(machine.intc.calls == interrupts);
  assert(std::memcmp(domain, domain_after, sizeof(domain)) == 0);
  assert(machine.imsic_file->get_reg(IMSIC_EIP(0))->read() == eip);
}

void test_supported_store_matches_preflight(machine_t& machine)
{
  uint8_t scratch_before = 0xff;
  assert(machine.uart.load(7, 1, &scratch_before));
  assert(scratch_before == 0);
  const uint8_t scratch_value = 0x5a;
  assert(machine.uart.store_preflight(7, 1));
  assert(machine.uart.store(7, 1, &scratch_value));
  uint8_t scratch = 0;
  assert(machine.uart.load(7, 1, &scratch));
  assert(scratch == scratch_value);
  uint8_t rejected[4] = {1, 2, 3, 4};
  assert(!machine.uart.store_preflight(7, 4));
  assert(!machine.uart.store(7, 4, rejected));
  assert(machine.uart.load(7, 1, &scratch));
  assert(scratch == scratch_value);

  uint8_t wide[4] = {0xab, 0, 0, 0};
  assert(machine.wide_uart.store_preflight(7, 4));
  assert(machine.wide_uart.store(7, 4, wide));
  uint8_t wide_bytes[4] = {};
  assert(machine.wide_uart.load(7, 4, wide_bytes));
  assert(wide_bytes[0] == 0xab);
  assert(machine.intc.calls == 0);

  const uint8_t msip[4] = {1, 0, 0, 0};
  assert(machine.clint.store_preflight(0, 4));
  assert(machine.clint.store(0, 4, msip));
  const uint8_t mtime[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
  assert(machine.clint.store_preflight(0xbff8, 8));
  assert(machine.clint.store(0xbff8, 8, mtime));
  assert(machine.clint.get_mtime() == 0x8877665544332211ull);
}

void expect_store_fault(machine_t& machine, reg_t address)
{
  const std::size_t calls = machine.sim.mmio_store_calls;
  machine.proc.get_state()->XPR.write(4, address);
  bool trapped = false;
  try {
    execute_ztt(machine.proc, ztt_opcode_t::mss_rm, operands(0, 4));
  } catch (const trap_store_access_fault&) {
    trapped = true;
  }
  assert(trapped);
  assert(machine.sim.mmio_store_calls == calls);
}

void test_ztt_mmio_atomicity()
{
  machine_t machine;
  test_preflight_has_no_side_effects(machine);

  const auto spill = sequence(kU32, 0x5a);
  set_m(machine.proc, 0, kU32, spill);
  const uint64_t mtime = machine.clint.get_mtime();
  const uint64_t mtimecmp = machine.clint.get_mtimecmp(0);
  expect_store_fault(machine, CLINT_BASE + CLINT_SIZE - 8);
  assert(machine.clint.get_mtime() == mtime);
  assert(machine.clint.get_mtimecmp(0) == mtimecmp);

  const reg_t eip = machine.imsic_file->get_reg(IMSIC_EIP(0))->read();
  expect_store_fault(machine, IMSIC_M_BASE);
  assert(machine.imsic_file->get_reg(IMSIC_EIP(0))->read() == eip);

  uint8_t scratch[4] = {};
  assert(machine.wide_uart.load(7, 4, scratch));
  expect_store_fault(machine, NS16550_BASE + 0x1000);
  uint8_t scratch_after[4] = {};
  assert(machine.wide_uart.load(7, 4, scratch_after));
  assert(scratch_after[0] == scratch[0]);
  assert(machine.intc.calls == 0);

  std::vector<ztt_unit_t::element_t> values(ztt_unit_t::element_count(kU32), 0);
  values[0] = 0x11223344;
  values[1] = 0x55667788;
  set_m(machine.proc, 0, kU32, values);
  machine.proc.get_state()->XPR.write(4, CLINT_BASE + 0x4000);
  execute_ztt(machine.proc, ztt_opcode_t::mss_rm, operands(0, 4));
  assert(machine.sim.mmio_store_calls == 64);
  assert(machine.clint.get_mtimecmp(0) == 0x5566778811223344ull);

  test_supported_store_matches_preflight(machine);

  const reg_t eip_before = machine.imsic_file->get_reg(IMSIC_EIP(0))->read();
  uint8_t pending[4] = {1, 0, 0, 0};
  assert(!machine.imsic.store_preflight(SETEIPNUM_LE, 4));
  assert(machine.imsic_file->get_reg(IMSIC_EIP(0))->read() == eip_before);
  assert(machine.imsic.store(SETEIPNUM_LE, 4, pending));
  assert(machine.imsic_file->get_reg(IMSIC_EIP(0))->read() != eip_before);

  uint8_t domain_before[4];
  uint8_t domain_after[4];
  assert(machine.aplic.load(0, 4, domain_before));
  assert(!machine.aplic.store_preflight(0, 4));
  assert(machine.aplic.load(0, 4, domain_after));
  assert(std::memcmp(domain_before, domain_after, 4) == 0);
  uint8_t enable[4] = {0x00, 0x01, 0x00, 0x00};
  assert(machine.aplic.store(0, 4, enable));
  assert(machine.aplic.load(0, 4, domain_after));
  assert(std::memcmp(domain_before, domain_after, 4) != 0);
}

void test_rv32_and_rv64_dispatch()
{
  const struct isa_case_t {
    const char* isa;
    unsigned xlen;
  } cases[] = {
    {"rv32i_ztt", 32},
    {"rv64i_ztt", 64},
  };
  for (const auto& test : cases) {
    test_sim_t sim;
    std::ostringstream output;
    processor_t proc(test.isa, "M", &sim.cfg, &sim, 0, false, nullptr, output);
    sim.harts.emplace(0, &proc);
    assert(proc.get_xlen() == test.xlen);
    proc.get_state()->mstatus->write(reg_t(1) << 25);
    execute_ztt(proc, ztt_opcode_t::ame_acquire, operands(1));
    assert(proc.ZTU.owned());
    proc.get_state()->XPR.write(4, DRAM_BASE);
    std::vector<ztt_unit_t::element_t> values(ztt_unit_t::element_count(kU32), 0);
    values[0] = 0x10;
    assert(proc.ZTU.set_m_datatype(0, kU32));
    assert(proc.ZTU.write_m(0, values));
    execute_ztt(proc, ztt_opcode_t::mss_rm, operands(0, 4));
    assert(sim.memory[0] == 0x10);
  }
}

} // namespace

int main()
{
  test_ztt_mmio_atomicity();
  test_rv32_and_rv64_dispatch();
  return 0;
}
