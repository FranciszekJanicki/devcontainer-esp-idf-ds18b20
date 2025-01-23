include make/espidf.mk
include make/linters.mk
include make/proto.mk

.DEFAULT_GOAL = help

.PHONY: setup
setup: setup-linters setup-proto
# these USB rules apply only to boards which use CP210x USB-UART converter
	$(SUDO) cp 99-usb-serial.rules /etc/udev/rules.d/
	$(SUDO) service udev restart
	$(SUDO) chmod a+rw $(PORT)

.PHONY: build_all
build_all: build

.PHONY: all
all: lint build flash monitor

.PHONY: help
help:
	@echo "-------------------- USAGE --------------------"
	@echo ""
	@echo "setup"
	@echo "		setup project & ESP-IDF"
	@echo ""
	@echo "build"
	@echo "		build project"
	@echo ""
	@echo "flash"
	@echo "		flash project"
	@echo ""
	@echo "flash-monitor"
	@echo "		flash device & open serial monitor"
	@echo ""
	@echo "erase-flash-monitor"
	@echo "		erase, flash device & open serial monitor"
	@echo ""
	@echo "monitor"
	@echo "		open serial monitor"
	@echo ""
	@echo "all"
	@echo "		execute rules: lint, build, flash, monitor"
	@echo ""
	@echo "erase"
	@echo "		erase entire ESP flash memory"
	@echo ""
	@echo "clean"
	@echo "		clean project"
	@echo ""
	@echo "fullclean"
	@echo "		fullclean project"
	@echo ""
	@echo "menuconfig"
	@echo "		open ESP-IDF menuconfig"
	@echo ""
	@echo "lint"
	@echo "		run cppcheck & check clang-format on the code"
	@echo ""
	@echo "clang-format"
	@echo "		run clang-format on project files"
