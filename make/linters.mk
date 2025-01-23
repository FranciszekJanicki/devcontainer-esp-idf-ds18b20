include make/common.mk

.PHONY: setup-linters
setup-linters:
	$(SUDO) apt-get update
	$(SUDO) apt-get -y install cppcheck=2.7-1 clang-format
	$(PIP) install -r $(REQUIREMENTS_DIR)/linters.txt

.PHONY: yamllint
yamllint:
	yamllint .gitlab-ci.yml

.PHONY: cppcheck
cppcheck:
	cppcheck --enable=all --inconclusive --check-config \
		--suppress=missingInclude \
		$(MAIN_DIR)

	cppcheck --enable=all --inconclusive --check-config \
		--suppress=missingInclude \
		-i$(PROTO_COMPONENT_DIR) \
		$(COMPONENT_DIR)

.PHONY: pre-commit
pre-commit:
	pre-commit run --all-files

.PHONY: clang-format
clang-format:
	for ext in h c cpp hpp; do \
		find $(MAIN_DIR) -iname "*.$$ext" -print0 | xargs -0 -r clang-format -i; \
		find $(COMPONENT_DIR) -iname "*.$$ext" -not -path "*/putrl_proto/*" -print0 | xargs -0 -r clang-format -i; \
	done

.PHONY: clang-format-dry
clang-format-dry:
	for ext in h c cpp hpp; do \
		find $(MAIN_DIR) -iname "*.$$ext" -print0 | xargs -0 -r clang-format --dry-run -Werror -i; \
		find $(COMPONENT_DIR) -iname "*.$$ext" -not -path "*/putrl_proto/*" -print0 | xargs -0 -r clang-format --dry-run -Werror -i; \
	done

.PHONY: lint
lint: cppcheck clang-format-dry
