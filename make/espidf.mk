include make/common.mk


.PHONY: build
build:
	$(IDF) build \
		-DMAKE_PROJECT_NAME="$(PROJECT_NAME)" \
		-DIDF_BUILD_TARGET="$(ESP_IDF_TARGET)" \
		-DIDF_TARGET=$(ESP_IDF_TARGET)

.PHONY: flash
flash:
	$(IDF) flash -p $(PORT)

.PHONY: flash-monitor
flash-monitor:
	$(IDF) -p $(PORT) flash monitor

.PHONY: erase-flash-monitor
erase-flash-monitor:
	$(IDF) -p $(PORT) erase_flash flash monitor

.PHONY: monitor
monitor:
	$(IDF) monitor -p $(PORT)

.PHONY: clean
clean:
	$(IDF) clean

.PHONY: fullclean
fullclean:
	$(IDF) fullclean
	rm -rf $(BUILD_DIR)

.PHONY: menuconfig
menuconfig:
	$(IDF) menuconfig
