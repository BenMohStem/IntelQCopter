/* Boot-only RP2350 check using the rp2350js library API (no daemon/socket). */
const path = require('node:path');
const emulatorRoot = process.env.RP2350JS_PATH ?? 'C:/WinLibs/rp2350js';
const { RP2350 } = require(path.join(emulatorRoot, 'dist/cjs'));

const firmware = process.argv[2];
const steps = Number(process.argv[3] ?? 1000000);
if (!firmware || !Number.isSafeInteger(steps) || steps < 1) {
  console.error('Usage: node sim/rp2350js/boot_check.cjs <firmware.hex|firmware.uf2> [steps]');
  process.exit(2);
}

const mcu = new RP2350({ coreArch: 'arm', loadFirmware: path.resolve(firmware) });
const resetPc = mcu.core0.PC >>> 0;
const startedAt = Date.now();
const progressEvery = Number(process.env.RP2350JS_PROGRESS_EVERY ?? 100000);
for (let i = 0; i < steps; i += 1) {
  mcu.step();
  if (progressEvery > 0 && (i + 1) % progressEvery === 0) {
    const pc = (mcu.core0.PC >>> 0).toString(16).padStart(8, '0');
    console.log(`[RP2350JS] step=${i + 1}/${steps} simulated_cycles=${mcu.cycles} core0_pc=0x${pc} wall_ms=${Date.now() - startedAt}`);
  }
}

console.log(`RP2350 ARM boot check passed: ${steps} steps, ${mcu.cycles} simulated cycles.`);
console.log(`Core 0 PC: 0x${(mcu.core0.PC >>> 0).toString(16)} (reset PC was 0x${resetPc.toString(16)}).`);
console.log('This checks instruction execution only; it does not validate timing, sensors, RF, or ESC outputs.');
