#!/bin/bash

ROOT_DIR=${1}
# get the second argument if provided, else set to false
NEED_FORMAT=${2:-false}

set -e
# if NEED_FORMAT is true, then format the code
if [ "$NEED_FORMAT" = "true" ]; then
    ruff format --config=${ROOT_DIR}pyproject.toml ${ROOT_DIR}./
    ruff check --fix --config=${ROOT_DIR}pyproject.toml ${ROOT_DIR}./
# else check the code
else
    ruff check --config=${ROOT_DIR}pyproject.toml ${ROOT_DIR}./
fi
