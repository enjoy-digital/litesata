# Glossary {#glossary}

The terms of SATA, ATA and LiteSATA used in this manual, in alphabetical order. Each entry points
to the chapter that explains it.

| Term | Meaning |
|------------------|----------------------------------------------------------------------------------|
| 8b/10b | Line code of SATA: each byte is sent as a 10-bit character, with control characters for the primitives. See [SATA in a nutshell](#sata-primer) |
| ACS | ATA Command Set: the specification of the ATA commands, such as READ DMA EXT. See [SATA in a nutshell](#sata-primer) |
| AHCI | Register interface of the SATA host controllers of PCs; LiteSATA does not implement it. See [Introduction](#introduction) |
| ALIGN | Primitive sent in pairs every 256 dwords for clock compensation, and first exchanged when the link starts. See [Link layer](#link-layer) |
| ATA | The command protocol of the drives, carried by SATA in FISes. See [SATA in a nutshell](#sata-primer) |
| BIST | Built-in self test: frontend that writes and checks a pattern on the drive and measures the rate. See [Frontends](#frontends) |
| Burst | OOB period during which a transmitter sends a signal, between two gaps. See [SATA in a nutshell](#sata-primer) |
| CDR | Clock and data recovery: the part of the transceiver that recovers the clock from the received bits. See [PHY architecture and OOB controller](#phy-architecture) |
| COMINIT | OOB sequence sent by the drive to start or restart the link; same timing as COMRESET. See [SATA in a nutshell](#sata-primer) |
| COMRESET | OOB sequence sent by the host to reset the drive and start the link. See [SATA in a nutshell](#sata-primer) |
| COMWAKE | OOB sequence exchanged after COMRESET and COMINIT, with shorter gaps, before the speed negotiation. See [SATA in a nutshell](#sata-primer) |
| Command layer | Layer of the core that runs the ATA commands from the user ports. See [Transport and command layers](#transport-command) |
| CONT | Primitive meaning "the previous primitive repeats", followed by scrambled filler. See [Link layer](#link-layer) |
| Controller | In this manual, the OOB and ALIGN state machine of the PHY (`LiteSATAPHYCtrl`). See [PHY architecture and OOB controller](#phy-architecture) |
| CRC | 32-bit check value at the end of each frame. See [Link layer](#link-layer) |
| Crossbar | Module that shares the core between user ports, with round-robin arbitration. See [Frontends](#frontends) |
| D2H | Device to host: direction of a FIS sent by the drive. See [SATA in a nutshell](#sata-primer) |
| D10.2 | Data character with alternating bits, sent by the host while it waits for ALIGN. See [PHY architecture and OOB controller](#phy-architecture) |
| DCU | Dual channel unit: the transceiver block of the Lattice ECP5 (`DCUA` primitive). See [Lattice ECP5 PHY with OOB in fabric](#ecp5-phy) |
| DF | Device fault: status bit of the drive; LiteSATA fails the command when it is set. See [Transport and command layers](#transport-command) |
| DMA Activate FIS | FIS by which the drive tells the host to send the next Data FIS of a write. See [Transport and command layers](#transport-command) |
| DRQ | Data request: status bit of the drive, set when it is ready to transfer data. See [SATA in a nutshell](#sata-primer) |
| Dword | 32 bits: one primitive or one data word on the link. See [SATA in a nutshell](#sata-primer) |
| EOF | Primitive that ends a frame. See [Link layer](#link-layer) |
| ERR | Error: status bit of the drive; the error register then gives the cause. See [Transport and command layers](#transport-command) |
| FIS | Frame Information Structure: the content of a frame, typed by its first byte. See [SATA in a nutshell](#sata-primer) |
| Frontend | Module on a user port of the crossbar: Identify, BIST, DMA, stream, striping, mirroring. See [Frontends](#frontends) |
| Gap | OOB period without signal, whose length tells COMINIT from COMWAKE. See [SATA in a nutshell](#sata-primer) |
| Gen1, Gen2, Gen3 | SATA generations at 1.5 Gbps, 3 Gbps and 6 Gbps. See [SATA in a nutshell](#sata-primer) |
| H2D | Host to device: direction of a FIS sent by the host. See [SATA in a nutshell](#sata-primer) |
| HOLD, HOLDA | Primitives of the flow control: HOLD pauses the frame, HOLDA acknowledges it. See [Link layer](#link-layer) |
| IDENTIFY DEVICE | ATA command that returns 256 words of drive information: model, serial, capacity, features. See [Transport and command layers](#transport-command) |
| JTAGBone, UARTBone | LiteX bridges that give the host access to the registers over JTAG or a UART. See [Example designs and targets](#examples) |
| K28.3, K28.5 | 8b/10b control characters; K28.5 is the comma of ALIGN, K28.3 starts the other primitives. See [SATA in a nutshell](#sata-primer) |
| LBA | Logical block address: the 48-bit number of a sector. See [SATA in a nutshell](#sata-primer) |
| Link layer | Layer of the core that frames, scrambles, checks and acknowledges the FISes. See [Link layer](#link-layer) |
| LiteScope | LiteX logic analyzer that records signals in the FPGA. See [Debugging](#debugging) |
| NCQ | Native Command Queuing: several commands pending in the drive; not supported. See [Introduction](#introduction) |
| OOB | Out-of-band signalling: bursts and gaps that wake up and reset the link before any data. See [SATA in a nutshell](#sata-primer) |
| PIO | Programmed input/output: transfer mode used by IDENTIFY DEVICE. See [SATA in a nutshell](#sata-primer) |
| PIO Setup FIS | FIS by which the drive announces the Data FIS of a PIO transfer. See [Transport and command layers](#transport-command) |
| Primitive | Dword that starts with a control character and carries link control, such as ALIGN or SYNC. See [SATA in a nutshell](#sata-primer) |
| R_ERR, R_OK | Primitives by which the receiver rejects or accepts a frame after its CRC. See [Link layer](#link-layer) |
| R_IP, R_RDY | Primitives of the receiver: reception in progress, ready to receive. See [Link layer](#link-layer) |
| Register FIS | FIS carrying the command registers (H2D) or the status of the drive (D2H). See [SATA in a nutshell](#sata-primer) |
| RLOS | Receive loss of signal: output of the ECP5 DCU used to measure the OOB gaps. See [Lattice ECP5 PHY with OOB in fabric](#ecp5-phy) |
| Scrambler | Linear feedback shift register whose output is XORed with the frame data to spread its spectrum. See [Link layer](#link-layer) |
| Sector | 512 bytes, the unit of the commands. See [SATA in a nutshell](#sata-primer) |
| SOF | Primitive that starts a frame. See [Link layer](#link-layer) |
| Squelch | Detector of the receiver that tells a signal from silence during OOB. See [SATA in a nutshell](#sata-primer) |
| SYNC | Idle primitive, sent when there is nothing else to send. See [Link layer](#link-layer) |
| Transport layer | Layer of the core that builds and decodes the FISes. See [Transport and command layers](#transport-command) |
| UI | Unit interval: one bit time on the link; OOB timing is given in Gen1 UI. See [SATA in a nutshell](#sata-primer) |
| User port | Port of the crossbar where a frontend or user logic sends commands. See [Core architecture](#core-architecture) |
| WTRM | Primitive sent by the transmitter after EOF while it waits for R_OK or R_ERR. See [Link layer](#link-layer) |
| X_RDY | Primitive by which a side announces a frame to send. See [Link layer](#link-layer) |
