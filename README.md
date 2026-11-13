# AWG-MultiChannel-RFSoC

[![Platform](https://img.shields.io/badge/Platform-RealDigital%20RFSoC%204x2-red)]([https://redpitaya.com/](https://www.realdigital.org/hardware/rfsoc-4x2))
[![FPGA](https://img.shields.io/badge/FPGA-ZYNQ%20Ultrascale+%20RFSoC%20ZU48DR-blue)]([https://www.xilinx.com/](https://www.amd.com/en/products/adaptive-socs-and-fpgas/soc/zynq-ultrascale-plus-rfsoc.html))


## Overview
High-sampling-rate, high-memory-density, multi-channel arbitrary waveform generator. Waveforms are transferred from PS to PL via AXI DMA, stored in RAM, and streamed to the on-chip DACs.

## Features

- Data sampling at 9.8304 GSps
- Two output channels: DAC B (DAC tile 0) and DAC A (DAC tile 2), see `CHANNELS` in `python/rfsoc_awg.py`
- One looping waveform per DAC, up to $2^{18}$ samples (26.67 µs)
  - The RAM has two halves per DAC, selected by (`axi_control` OR the `control_trigger` pin). The pin floats high on this board, so the Python driver always plays half "x" (`axi_control` = 1) and leaves "z" at zero.
- Two-channel ADC capture (ADC D and ADC C) with triggers on each DAC's sequence start (`dac_b` locked, `dac_a` ±1 beat).
- Sample loading within 5-7 ms.
- One ADC input (ADC D) captured raw at the full 4.9152 GSps, with the ADC clock locked to the AWG, pre-trigger, and triggers from the ADC level, the AWG sequence start or the external trigger. See [ADC capture](#adc-capture).


## System Architecture
The implementation is summarized with the help of the following diagrams. It is separated into the waveform-write and the waveform-read blocks.

### Waveform write block
The sample comprising the waveforms A,B,C, and D are fed into the PL via an AXI-DMA with samples in the sequence shown in the diagram. The stream then passes through a Data Sorter module, which reorders the incoming samples and writes them into the RAM banks using the memory layout depicted in the diagram. During this process, the `write_enable` signal is held high. 
For the memory block RAM[0] and RAM[1], the first $2^{14}$ memory address contain waveform-A samples, and the rest $2^{14}$ address (from $2^{14}$ to $2^{15}$) contain waveform-B samples. A similar arrangement is used for waveform-C and -D on RAM blocks RAM[2], and RAM[3].

### Waveform read block
On the read side, the address counter increments with a clock signal clk0 for RAM[0],and RAM[1]) and with clk2 for RAM[2], and RAM[3]). Both clocks operate at 614.4 MHz -  16x slower than the DAC sampling rate. 
At each rising edge of clk0, the 16 samples - 8 samples on RAM[0], and 8 samples on RAM[1] at the same address counter - are concatenated and sent to the DAC IP for output on DAC0 (DAC B). When the waveform select signal : (`control_trigger` || `axi_control`) is high, an offset of $2^{14}$ is added to the base address, so that waveform-B is output on the same dac channel. A similar behavior is implemented to output waveform-C or waveform-D at DAC2.



<table cellspacing="0" cellpadding="10">
  <tr>
    <th align="left">Input</th>
    <th align="left">Output</th>
  </tr>
  <tr>
    <td><img src="./assets/arb_RFSoC_input_bkg.png" width="500" alt="Write"></td>
    <td><img src="./assets/arb_RFSoC_output_bkg.png" width="500" alt="Read"></td>
  </tr>
</table>
Internally, the design is pipelined to meet timing requirements for performance of the FPGA logic at an incredible clock rate of 614.4 MHz. There is no particular reason, for such arrangement of the memory blocks, other than historical. This design is an extension of earlier design of an AWG that outputs only one waveform over one DAC channel.

## Input/Output 
The relevant I/O pins and ports on the board are
| Name | Pin/Port |  Type | Description |
| -------- | -------- | -------- | -------- |
| control_trigger | PMOD0_1 | Input | External waveform-select trigger. Toggles DAC B (DAC0) between waveform-A/waveform-B and DAC A (DAC2) between waveform-C/waveform-D. Effective select: (axi_control || control_trigger)|
| pmod_out| PMOD0_0  | Output | Outputs control_trigger signal sampled on a 614.4 MHz clock.  |
| DAC 0| DAC B  | Output | AWG output |
| DAC 2| DAC A  | Output | AWG output |
| ADC | ADC D  | Input | Fast ADC input |

The design contains AXI GPIOs that Python (`python/rfsoc_awg.py`) drives as software-visible registers.
| Name | Memory Address |  Type | Description |
| -------- | -------- | -------- | -------- |
| rst_n | 0x00_A007_0000 | Output |1 bit. Software reset. Active low signal. |
| write_enable | 0x00_A005_0000  | Output | 1 bit. Enables writing waveform to RAM bank. Set to 0 to enable output; set enable_ch0 and enambe_ch2 accordingly. |
| MAX_POINTS| 0x00_A006_0000  | Output | 32 bit. (MAX_POINTS<<4) is the maximum number of points to output. |
| enable_ch0 | 0x00_A009_0000 | Output | 1 bit. Enables output on DAC B when write_enable is low. |
| enable_ch2 | 0x00_A00A_0000 | Output | 1 bit. Enables output on DAC A when write_enable is low. |
| axi_control| 0x00_A008_0000 | Output | 1 bit. Software trigger signal.  Switches between waveform-A and waveform-B for DAC0 output and between waveform-C and waveform-D for DAC2 output. The effective select signal is (axi_control || control_trigger) |
| axi_dma_0| 0x00_A004_0000 | Input | AMD's DMA IP for direct memory access and fast transfer of data from PS to PL.|
| adc (registers) | 0x00_A00B_0000 | In/Out | ADC capture engine control and status, 4 KB. See [ADC capture](#adc-capture). |
| axi_bram_ctrl_adc | assigned by Vivado | Input | ADC capture memory, 512 KB, read-only for software. Vivado picks a 512 KB-aligned address; the driver takes it from the `.hwh`. |


## ADC capture
The ADC path records what comes in on the ADC D input, aligned with the AWG output, so the two can be compared. It is a triggered scope: the PL writes every ADC sample into on-chip RAM, and the PS reads the window around the trigger afterwards. It is built by `fpga/tcl/add_adc.tcl` (called by the build script) from `fpga/adc/hdl/adc_capture.v`.

| Property | Value |
| -------- | -------- |
| Converter | RFDC ADC slice 00 (`vin0_01`), 4.9152 GSps from the 491.52 MHz reference on `adc0_clk_1`, real output, no decimation: raw samples, 203 ps apart, bandwidth up to 2.4576 GHz. 16-bit signed samples. |
| Stream | 12 samples per beat (192 bit, the widest the RFDC allows), sample 0 (the earliest) in bits [15:0]. |
| ADC stream clock | 409.6 MHz from `clk_wiz_adc`, an MMCM fed by the DAC0 614.4 MHz clock. |
| Memory | Ring buffer of 16384 beats x 256 bit (12 samples + 64 unused bits) = 196608 samples = 40.0 µs, in a dual-port BRAM. The PL never waits for the PS, so no sample is lost. |
| Trigger window | PRE beats before the trigger beat and POST beats from it on (PRE + POST at most 16384 beats). One beat is 2.44 ns. |
| Level trigger | Sample accurate. `TSUB` gives which of the 12 samples in the trigger beat crossed the level. |
| AWG ch0 trigger | Locked: the same ADC sample every time, see below. |

**How the ADC stays aligned with the DAC.** The DAC runs at 9.8304 GSps and the ADC at 4.9152 GSps from the same reference, so they never drift apart. The capture logic does not use an RFDC ADC clock: `clk_wiz_adc` makes 409.6 MHz (ADC stream) and 204.8 MHz (frame) from the 614.4 MHz AWG ch0 clock. One frame of 4.883 ns is exactly 3 AWG beats, 2 ADC beats and 24 ADC samples. The AWG sequence-start event goes AWG clock → frame clock → ADC clock through plain registers that Vivado times as synchronous paths, together with its position inside the frame, so the core knows to the sample which ADC sample belongs to the AWG beat where the sequence started. That gives `TADDR`/`TSUB` with no jitter. What is left is a constant delay (DAC and ADC converter latency, cables) that changes only when the RFDC tiles restart. Measure it once per power-up with a loopback cable from DAC B (tile 0, trigger `dac_b`) to ADC D: `AdcScope.calibrate_awg("B", adc="D")` does that and then puts t = 0 on the sequence start as seen at the ADC. After that, `configure(pre_ns=..., length_ns=...)` is in this calibrated time: the driver captures the longer raw window behind the scenes and crops it, so `pre_ns=5, length_ns=20` shows −5 to +15 ns around the DAC edge. `calibrate_awg("A")` does the same for DAC A (tile 2, trigger `dac_a`), which has its own tile clock and stays asynchronous (±1 beat). The DAC letter ↔ tile mapping is `CHANNELS` in `python/rfsoc_awg.py` (B = tile 0, as in the PYNQ 4x2 base overlay); `adc_scope.py` reads it from there.

The MMCM must lock after the DAC clock runs. `AdcScope(ol)` resets it through the `CLKCTL` register, waits for `STATUS` bit 2 and restarts the ADC tile. Create it after `xrfclk.set_ref_clks()`.

Trigger source (`MODE` register): 0 immediate, 1 level rising, 2 level falling, 3 AWG channel 0 sequence start (locked), 4 AWG channel 2 sequence start, 5 `control_trigger` rising edge, 6 `control_trigger` falling edge, 7 manual only. A `FORCE` write triggers at once, which software uses as an auto-trigger timeout.

Register map, base address 0x00_A00B_0000 (byte offsets):
| Offset | Name | Access | Description |
| -------- | -------- | -------- | -------- |
| 0x00 | CTRL | W | bit 0 ARM, bit 1 FORCE, bit 2 ABORT. Write 1, the bits clear themselves. |
| 0x04 | MODE | R/W | Trigger source, bits [2:0], see above. |
| 0x08 | LEVEL | R/W | Signed threshold in ADC codes (int16), bits [15:0]. |
| 0x0C | PRE | R/W | Beats stored before the trigger beat. |
| 0x10 | POST | R/W | Beats stored from the trigger beat on (at least 1). |
| 0x14 | STATUS | R | bit 0 DONE, bit 1 BUSY (ARM not yet taken by the ADC side), bit 2 ADC clock (MMCM) locked, bits [6:4] state. |
| 0x18 | TADDR | R | Beat address of the trigger beat in the capture memory. |
| 0x1C | TSUB | R | Sample index (0 to 11) of the trigger inside its beat, for level triggers and AWG ch0. 0 otherwise. |
| 0x20 | TTIME | R | ADC clock cycles from ARM to the trigger. |
| 0x24 | DEPTH | R | Buffer depth in beats (16384). |
| 0x28 | ID | R | `0xADC50005`. Reads something else if the bitstream has no ADC capture. |
| 0x2C | ALIVE | R | Free-running count of ADC beats. If it does not move, the ADC clock is not running. |
| 0x30 | CFG | R | bits [7:0] samples per beat (12), [15:8] 32-bit words per memory word (8), bit 16 AWG ch0 locked (1). |
| 0x34 | FS_KHZ | R | Sample rate in kHz (4915200). |
| 0x38 | CLKCTL | R/W | bit 0 resets the ADC clock MMCM (write 1, then 0). |

To read a capture: write PRE, POST, MODE and LEVEL, write CTRL = 1 (ARM), wait for STATUS bit 0 (DONE), then read the capture memory. A beat is one 32-byte memory word, samples in the lower 24 bytes. The valid window starts at beat `TADDR - PRE` and wraps around at DEPTH. `python/adc_scope.py` does all of this; it only accepts the v5 core (2 channels, locked to the AWG). `ALIVE` and `ID` are the first things to check on a new bitstream.

PS registers cross into the ADC clock with XPM CDC macros. The AWG ch2 toggle and `control_trigger` cross the same way (asynchronous); the AWG ch0 toggle uses the frame-locked path described above.

## Repository layout
- `fpga/` — RTL (`src/` AWG, `adc/` ADC capture), constraints, block-design Tcl, `awg_multichannel_rfsoc.tcl` (the single build script), `bitstreams/`, and `legacy/` (unused older attempts).
- `fpga/adc/sim/` — simulation of the ADC core with Icarus Verilog (`run_sim.sh`): triggers, windows and the AWG-locked timing.
- `fpga/bitstreams/` — ready-to-use `.bit` and `.hwh` pairs, see [Bitstreams](#bitstreams).
- `python/` — current dashboard (`rfsoc_dashboard.py`, `AWG_dashboard.ipynb`) and the modules it imports (`rfsoc_awg.py`, `adc_scope.py`, `bode_sweep.py`, `bode.ipynb`), plus the LMK and LMX register files. `xrfclk.set_ref_clks` looks for those register files in the working directory.
- `assets/` — architecture diagrams.

`fpga/awg_multichannel_rfsoc.tcl` is the single build script (see Modifications). It sources `fpga/bd/block_design.tcl` (Vivado 2024.1 export of the AWG block design), optionally adds the ADC capture path with `fpga/tcl/add_adc.tcl` (RTL in `fpga/adc/hdl/adc_capture.v`), and uses `fpga/constrs/awg_top.xdc`. `fpga/legacy/` holds the earlier DMA-based ADC capture attempt (`add_adc_capture.tcl`, `adc_capture.xdc`, `packet_generator_1.0`), older block-design exports and the original constraints. Nothing in it is used by the build.

## Bitstreams
A `.bit` always goes with the `.hwh` of the same name. Copy both to the board next to the notebook that loads them.

| Files | Design | Use with |
| -------- | -------- | -------- |
| `python/awg_adc_v5.bit` + `.hwh` | AWG + 2-channel ADC capture (4.9152 GSps raw, locked to AWG ch0), produced by the build script | everything in `python/` (default bitstream of `rfsoc_awg.AWG`) |
| `fpga/bitstreams/awg_clock_syncd.bit` (inside `awg_clock_syncd.7z`) + `.hwh` | AWG only, 2 DAC channels, no ADC | `python/rfsoc_awg.py` with `bitfile=` |

The ADC cells of `awg_adc_v5` are named `adc` and `axi_bram_ctrl_adc`. `python/adc_scope.py` only accepts this bitstream.

## Usage
1. On the board, put in one folder: the notebook you want from `python/`; `python/LMK04828_245.76.txt` and `python/LMX2594_491.52.txt`; and `awg_adc_v5.bit` + `.hwh`. For the AWG-only design, extract `fpga/bitstreams/awg_clock_syncd.7z` to obtain the `.bit`.
2. For the dashboard, copy `rfsoc_dashboard.py` and `rfsoc_awg.py` into that same folder. Packages beyond the PYNQ image are listed in `requirements.txt`.
3. Run the notebook from the Linux OS on the FPGA.


## Building the FPGA design
You only need this to change the FPGA design. To just run it, use a bitstream from [Bitstreams](#bitstreams).

**Needs:** Vivado **2024.1** with a license that covers the `xczu48dr` device, a clone of this repository, and about 50 minutes for a full build. Other Vivado versions are refused by the script, because the block design is a 2024.1 export.

**First time** (Vivado Tcl console; use forward slashes in paths):
1. Check that everything assembles without building (a few minutes):
   ```
   set AWG_BUILD 0
   source C:/path/to/AWG-MultiChannel-RFSoC/fpga/awg_multichannel_rfsoc.tcl
   ```
   It is fine when the console shows `Block design validated and saved.` and `--no-build: stopping here`.
2. Run the full build (synthesis, implementation, bitstream):
   ```
   unset AWG_BUILD
   source C:/path/to/AWG-MultiChannel-RFSoC/fpga/awg_multichannel_rfsoc.tcl
   ```
3. The build is good when the console ends with `DONE. Copied ... to fpga/bitstreams` and the printed `WNS` and `WHS` are zero or positive. A warning that timing is not met means do not use that bitstream.

**Batch mode** does the same: `vivado -mode batch -source fpga/awg_multichannel_rfsoc.tcl -tclargs [options]`.

| Option (command line) | Variable in the console | Effect |
| -------- | -------- | -------- |
| `--no-adc` | `AWG_ADC 0` | AWG only, no ADC capture. Output: `awg_only_new.bit/.hwh`. |
| `--no-build` | `AWG_BUILD 0` | Stop after the block design and wrapper are made. |
| `--jobs N` | `AWG_JOBS N` | Parallel jobs, default 4. |
| `--proj-dir DIR` | `AWG_PROJ_DIR` | Put the Vivado project in another folder. |

The script always recreates the Vivado project in `fpga/awg_multichannel_rfsoc/` from scratch, so that folder is disposable build output and git ignores it. Results go to `fpga/bitstreams/`.

**If something fails:**
- *Vivado version error*: use 2024.1.
- *`Missing file: ...`*: the checkout is incomplete or files were renamed. Pull again, or restore the file.
- *Build errors after pulling changes that moved or renamed files*: close the project, delete `fpga/awg_multichannel_rfsoc/` and run the script again. An old project keeps pointing at files that no longer exist.
- *The block design is not created, or `cell ... missing`*: look for the first `ERROR` line above it in the Tcl console. It is usually an IP version problem.
- *`path too long` errors on Windows*: use a short project folder, for example `set AWG_PROJ_DIR C:/awg_build`. The script warns when the project path is over 100 characters.
- *The RF data converter rejects the ADC settings*: the message names the setting and the values it accepts.

The script sets the FPGA part (`xczu48dr-ffvg1517-2-e`) directly and does not select a board. The RealDigital board files are only needed if you want to create a project from the board preset in the GUI: https://github.com/RealDigitalOrg/RFSoC4x2-BSP
