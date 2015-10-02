COREDIR = ../
PYTHON = python3

CMD = PYTHONPATH=$(COREDIR) $(PYTHON)

CC=gcc
CFLAGS =-Wall -O0

phy_datapath_tb:
	$(CMD) phy_datapath_tb.py

link_crc_tb:
	$(CC) $(CFLAGS) $(INC) -o crc crc.c
	$(CMD) link_crc_tb.py

link_scrambler_tb: scrambler
	$(CMD) link_scrambler_tb.py

link_cont_tb:
	$(CMD) link_cont_tb.py

link_tb:
	$(CMD) link_tb.py

command_tb:
	$(CMD) command_tb.py

bist_tb: scrambler
	$(CMD) bist_tb.py

striping_tb:
	$(CMD) striping_tb.py

mirroring_tb:
	$(CMD) mirroring_tb.py

scrambler: scrambler.c
	$(CC) $(CFLAGS) $(INC) -o $@ $<

example_designs:
	cd ../example_designs && $(PYTHON) make.py -t bist -s BISTSoCDevel -p kc705 -Ob run False build-bitstream
	cd ../example_designs && $(PYTHON) make.py -t bist -s BISTSoCDevel -p kc705 -Ob run False build-bitstream
	cd ../example_designs && $(PYTHON) make.py -t mirroring -s MirroringSoC -p kc705 -Ob run False build-bitstream
	cd ../example_designs && $(PYTHON) make.py -t striping -s StripingSoC -p kc705 -Ob run False build-bitstream
	cd ../example_designs && $(PYTHON) make.py -t striping -s StripingSoCDevel -p kc705 -Ob run False build-bitstream
	cd ../example_designs && $(PYTHON) make.py -t core -Ot design bist build-core
	cd ../example_designs && $(PYTHON) make.py -t core -Ot design striping build-core

all: phy_datapath_tb link_crc_tb link_scrambler_tb link_cont_tb link_tb command_tb bist_tb striping_tb mirroring_tb

clean:
	rm -f crc scrambler *.vcd
