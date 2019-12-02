PYTHON = python3

CMD = $(PYTHON)

examples:
	cd ../examples && $(PYTHON) make.py -t core -Ot design base build-core
	cd ../examples && $(PYTHON) make.py -t core -Ot design base -Ot ports_dw 64 build-core
	cd ../examples && $(PYTHON) make.py -t core -Ot design bist build-core
	cd ../examples && $(PYTHON) make.py -t core -Ot design striping build-core


all: examples

