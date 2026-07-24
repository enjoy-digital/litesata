#!/bin/bash
# DCUA behavioral OOB simulation - run after renewing /opt/diamond/license.dat
# (free Diamond license from latticesemi.com, includes the ModelSim 'latticemsim' feature).
set -e
export PATH=/opt/diamond/3.12/modeltech/linuxloem:$PATH
export LM_LICENSE_FILE=/opt/diamond/license.dat
cd "$(dirname "$0")"
vlib work 2>/dev/null || true
vlog tb_bypass.v tb_g8b10b.v
for tb in tb_bypass tb_g8b10b; do
    vsim -c -L ovi_ecp5u $tb -do "run -all; quit -f" | tee $tb.log
done
python3 analyze_vcd.py tb_bypass.vcd
python3 analyze_vcd.py tb_g8b10b.vcd
