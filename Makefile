# Makefile for RoboOrchard
ROOTDIR = $(CURDIR)
version_type := local
EXTRA_ARGS =
PIP_ARGS =
BUILD_ARGS =
COMMIT_UNIXTIME := $(shell git log -n 1 --pretty='format:%ct')
COMMIT_DATETIME := $(shell date -d @${COMMIT_UNIXTIME} +'%Y%m%d%H%M%S')
COMMIT_ID := $(shell git rev-parse --short HEAD)

INSTALL_OPTS := version_type="$(version_type)" PIP_ARGS="$(PIP_ARGS)" BUILD_ARGS="$(BUILD_ARGS)"
export ROBO_ORCHARD_USE_LOCAL := 1

# Put it first so that "make" without argument is like "make help".
help:
	@echo "Usage: make <target>"
	@echo "Targets:"
	@echo "  install - Install all packages"
	@echo "  install-editable - Install all packages in editable mode"
	@echo "  dev-env - Install development dependencies"
	@echo "  auto-format - Auto format code"
	@echo "  check-lint - Check lint"
	@echo "  doc - Build documentation"
	@echo "  doc-clean - Clean documentation"
	@echo "  test - Run tests"
	@echo "  ros2-dev-env - Install ROS2 development dependencies"
	@echo "  ros2-build - Build ROS2 packages"
	@echo "  ros2-test - Run ROS2 tests"
	@echo "  ros2-clean - Clean ROS2 build"
	@echo "  show-args - Show arguments"

install:
	@$(MAKE) -C python/robo_orchard_core install $(INSTALL_OPTS)
	@$(MAKE) -C python/robo_orchard_schemas install $(INSTALL_OPTS)
	@$(MAKE) -C python/robo_orchard_lab install $(INSTALL_OPTS)
	@$(MAKE) -C python/robo_orchard_inference_app install $(INSTALL_OPTS)

install-editable:
	@$(MAKE) -C python/robo_orchard_core install-editable $(INSTALL_OPTS)
	@$(MAKE) -C python/robo_orchard_schemas install-editable $(INSTALL_OPTS)
	@$(MAKE) -C python/robo_orchard_lab install-editable $(INSTALL_OPTS)
	@$(MAKE) -C python/robo_orchard_inference_app install-editable $(INSTALL_OPTS)

dev-env:
	@pip3 install -r scm/requirements.txt $(PIP_ARGS)
	@pre-commit install

auto-format:
	python3 scm/lint/check_lint.py --auto_format

check-lint:
	@python3 scm/lint/check_lint.py
	@pre-commit run check-merge-conflict
	@pre-commit run check-license-header --all-files

doc:
	@$(MAKE) -C python/robo_orchard_inference_app doc
	@$(MAKE) -C projects/HoloBrain/docs doc

doc-clean:
	@$(MAKE) -C python/robo_orchard_inference_app doc-clean
	@$(MAKE) -C projects/HoloBrain/docs doc-clean

test:
	@$(MAKE) -C python/robo_orchard_inference_app test

ros2-dev-env:
	make -C ros2_package dev-env PIP_ARGS="$(PIP_ARGS)"

ros2-build:
	make -C ros2_package build

ros2-test:
	make -C ros2_package test

ros2-clean:
	make -C ros2_package clean

show-args:
	@echo "PIP_ARGS: $(PIP_ARGS)"
	@echo "BUILD_ARGS: $(BUILD_ARGS)"
	@echo "EXTRA_ARGS: $(EXTRA_ARGS)"
