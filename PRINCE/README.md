# Masked PRINCE Lightweight Block Cipher – 7 nm ASAP7 Implementation  
This repository contains a complete digital implementation flow for a **masked PRINCE block cipher core** targeting **7nm FinFET technology** (ASAP7 PDK). The design supports **dual-share first-order masking**, making it resilient against **side-channel attacks**. The flow includes **RTL-to-GDSII** implementation using **Cadence Genus** for synthesis and **Cadence Innovus** for physical implementation.

---

## Directory Structure

PRINCE_masked_asap7/
├── PRINCE.mmmc                # Multi-Mode Multi-Corner setup
├── PRINCE.sdc                 # Top-level timing constraints (1 GHz target)
├── RTL/
│   └── prince_masked.v        # Gate-level netlist (masked PRINCE, 2 shares)
├── lef/                       # All ASAP7 cell LEFs
│   ├── asap7sc7p5t_28_L_4x_220121a.lef
│   ├── asap7sc7p5t_28_SL_4x_220121a.lef
│   └── ...
├── lib/                       # Liberty timing libraries (TT corner used)
│   └── asap7sc7p5t_*.lib
├── qrc/                       # QRC-Tech file (for extraction)
│   └── asap7_7p5t_*.tch
├── techlef/
│   └── asap7_tech_4x_201209.lef
├── tcl/
│   └── innovus_flow.tcl       # Complete reproducible P&R script (floorplan to GDS)
└── res/
    └── innovus_rpt/           # All reports (timing, power, glitch, QoR, DRC, etc.)
    └── genus_rpt/ 

## Library Source
The ASAP7 libraries (LEFs, LIBs, QRC files) used in this implementation were obtained from a modified version of the original ASU PDK, available in the open-source repository:  
[Centre-for-Hardware-Security/asap7_reference_design](https://github.com/Centre-for-Hardware-Security/asap7_reference_design)  
This repo provides a reference block design and modified 7 nm technology files compatible with Cadence Innovus, including fixes for pin access and DRC issues commonly encountered in the original ASAP7 PDK.



## Layout Overview (Full Placed & Routed Core)

<img width="805" height="796" alt="image" src="https://github.com/user-attachments/assets/caad3a39-b16e-4097-96cf-0f3b21594fbb" />

