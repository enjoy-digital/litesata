PYTHON = python3

CMD = $(PYTHON)

CC=gcc
CFLAGS =-Wall -O0

crc: crc.c
	$(CC) $(CFLAGS) $(INC) -o $@ $<

scrambler: scrambler.c
	$(CC) $(CFLAGS) $(INC) -o $@ $<

phy_datapath_tb:
	$(CMD) phy_datapath_tb.py

link_crc_tb: crc
	$(CMD) link_crc_tb.py

link_scrambler_tb: crc scrambler
	$(CMD) link_scrambler_tb.py

link_cont_tb:
	$(CMD) link_cont_tb.py

link_tb: crc scrambler
	$(CMD) link_tb.py

command_tb: crc scrambler
	$(CMD) command_tb.py

bist_tb: crc scrambler
	$(CMD) bist_tb.py

striping_tb: crc scrambler
	$(CMD) striping_tb.py

mirroring_tb: crc scrambler
	$(CMD) mirroring_tb.py

robustness_tb: crc scrambler
	$(CMD) robustness_tb.py

example_designs:
	cd ../example_designs && $(PYTHON) make.py -t bist -s BISTSoCDevel -p kc705 -Ob run False build-bitstream
	cd ../example_designs && $(PYTHON) make.py -t bist -s BISTSoCDevel -p kc705 -Ob run False build-bitstream
	cd ../example_designs && $(PYTHON) make.py -t mirroring -s MirroringSoC -p kc705 -Ob run False build-bitstream
	cd ../example_designs && $(PYTHON) make.py -t striping -s StripingSoC -p kc705 -Ob run False build-bitstream
	cd ../example_designs && $(PYTHON) make.py -t striping -s StripingSoCDevel -p kc705 -Ob run False build-bitstream
	cd ../example_designs && $(PYTHON) make.py -t core -Ot design base build-core
	cd ../example_designs && $(PYTHON) make.py -t core -Ot design base -Ot ports_dw 64 build-core
	cd ../example_designs && $(PYTHON) make.py -t core -Ot design bist build-core
	cd ../example_designs && $(PYTHON) make.py -t core -Ot design striping build-core


all: phy_datapath_tb link_crc_tb link_scrambler_tb link_cont_tb link_tb command_tb bist_tb striping_tb mirroring_tb example_designs

clean:
	rm -f crc scrambler *.v *.vvp *.vcd
