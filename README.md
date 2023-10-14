# Dynamic Memory Manager in Hardware

This repository contains the source code of a dynamic memory manager hardware that can be integrated into a processor core. This work is published at the 30th IFIP/IEEE International Conference on Very Large Scale Integration and System-on-Chip (VLSI-SOC), 2023. A draft of the manuscript is available [here](docs/aquila-dmm.pdf).

# About the RISC-V core
Note that the source code of the RISC-V processor in this repository is based on an old version of the [Aquila project](https://github.com/eisl-nctu/aquila). The Aquila cores used in the HW TCP/IP SoC and the SW LwIP SoC are also slightly different.  We will upgrade, condense, and merge the various versions of the Aquila cores in the near future (promises, promises, ... ;).

# Creation of the Vivado workspaces of Aquila SoC with DMM HW
To create the complete Vivado workspace for the Aquila SoC with DMM hardware, you can use the build.tcl script under hw/. Simply download the hw source tree to a local directory, then run the following command under a DOS or Linux console:

```
<<Vivado Installation directory>>/bin/vivado -mode batch -source build.tcl
```

The created project workspace will be in the directory aquila_soc/.

# Contact Info
Embedded Intelligent Systems Lab (EISL)  
Department of Computer Science  
National Yang Ming Chiao Tung University  
Hsinchu, Taiwan  

eisl.nctu-at-gmail
