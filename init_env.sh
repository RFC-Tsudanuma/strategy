#!/bin/bash
rm -rf .venv uv.lock
on_t1=$(uname -a | grep "tegra")
if [ -n "$on_t1" ]; then
    echo "Tegra detected, this is T1 env..."
    sudo apt-get install python3-pyqt5
    ln -sf ./t1_pyproject.toml ./pyproject.toml
else
    echo "Tegra not detected. This is dev env..."
    ln -fs ./dev_pyproject.toml ./pyproject.toml
    ln -fs ./dev_uv.lock ./uv.lock  # dev用のuv.lockを使うようにする
fi

uv venv --system-site-packages
if [ -n "$on_t1" ]; then
    uv sync
else
    uv sync
fi