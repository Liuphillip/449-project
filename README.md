# 449-project

# Pipelined CPU for ECE 449

This project implements a **5-stage pipelined CPU** that can execute a small custom instruction set. It includes a rudimentary BIOS in ROM, a dual-ported RAM, memory-mapped I/O, and a controller for handling pipeline hazards. The CPU was designed to meet the requirements of the ECE 449 course.

## Table of Contents
1. [Project Overview](#project-overview)  
2. [Features and Architecture](#features-and-architecture)  
   - [Instruction Formats](#instruction-formats)  
   - [Pipeline Stages](#pipeline-stages)  
   - [Memory Map](#memory-map)  
3. [Usage](#usage)  

---

## Project Overview
The primary goal of this project is to implement a synthesizable CPU design that demonstrates instruction fetch, decode, execute, memory access, and write-back stages in a pipeline. The CPU is capable of handling basic arithmetic, branching, and load/store operations using an ALU, register file, and memory modules. A simple BIOS is provided in a ROM for loading user programs into RAM and then executing them.

---

## Features and Architecture

### Instruction Formats

1. **A-format (Arithmetic)**  
   - Example: `ADD r3, r2, r1`  
   - Performs arithmetic operations such as `ADD`, `SUB`, etc.

2. **B-format (Branch)**  
   - Example: `BR r3+0x8`  
   - Includes regular branches and a subroutine call instruction, `BR.SUB`, which uses register `r7` as the link register.

3. **L-format (Load/Store)**  
   - Example: `LOAD r1, r2` (equivalent to `LOAD r1, @r2`)  
   - Handles memory-access instructions like loading from or storing to memory locations.

### Pipeline Stages

1. **Instruction Fetch (IF)**  
   - Fetches the instruction from the instruction memory/ROM (or RAM, depending on the design).
   - Updates the Program Counter (PC).

2. **Decode (ID)**  
   - Decodes the fetched instruction to determine the operation type.
   - Reads source registers from the register file.

3. **Execute (EX)**  
   - Performs the arithmetic or logical operation in the ALU.
   - Calculates branch targets if needed.

4. **Memory Access (MEM)**  
   - Reads/writes data memory if the instruction is a load/store.
   - Accesses dual-ported memory to separate instruction and data traffic.

5. **Write Back (WB)**  
   - Writes the result back to the destination register in the register file.

### Memory Map

- **ROM (0x0000 to 0x03FF)**: Contains a rudimentary BIOS that loads user code into RAM and then transfers control to it.  
- **RAM (0x0400 to 0x07FF)**: Stores both instructions and data for user programs, in a Harvard-like approach (separate instruction/data interfaces).  
- **I/O Ports**:  
  - **Input Port** at address `0xFFF0`  
  - **Output Port** at address `0xFFF2`

---

## Usage

1. **Reset and Load**  
   - On `reset_and_load`, the BIOS loads user code from an external interface (or from the ROM) into RAM.

2. **Reset and Execute**  
   - On `reset_and_execute`, the CPU begins execution at address `0x0000`.
   - The BIOS branches to the loaded user code start address (`0x0400` by default).

3. **Memory-Mapped I/O**  
   - Read from `0xFFF0` to get input data.
   - Write to `0xFFF2` to send output data.


