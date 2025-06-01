-include .env.example
-include .env

TIMEDATE_STR := $(shell date +"%H-%M_%d-%m-%Y")

VENV_PATH := ./.venv
PYTHON_BIN := $(VENV_PATH)/bin/python3

RAND_VAL := $(shell echo $$RANDOM)

default: init

init: cpenv download rmdefault link configure

run: run_ns3 analyze

build:
	$(NS3_BIN) build $(NS3_ADHOC_SIM_SRC)

download:
	wget 'https://www.nsnam.org/releases/ns-allinone-3.44.tar.bz2'
	tar xvf ns-allinone-3.44.tar.bz2
	rm ns-allinone-3.44.tar.bz2

rmdefault:
	rm -rf $(NS3_DIR)/scratch

clean:
	rm -rf $(NS3_AIO_DIR)
	rm -rf ./output

configure:
	$(NS3_BIN) configure
	$(NS3_BIN) build

cpenv:
	cp -f --update=none .env.example .env

link:
	ln -sfn $(shell pwd)/scratch $(NS3_DIR)/scratch
