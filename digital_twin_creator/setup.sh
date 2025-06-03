#!/bin/bash
if ! python3 -m venv --help > /dev/null 2>&1; then
    echo "Error: python3-venv is not installed. Please install it and try again."
    exit 1
fi

if ! command -v pip3 > /dev/null 2>&1; then
    echo "Error: pip3 is not installed. Please install it and try again."
    exit 1
fi

python3 -m venv venv
source venv/bin/activate
pip3 install -r requirements.txt
if [ $? -ne 0 ]; then
    echo "Error: Failed to install required packages. Please check requirements.txt."
    exit 1
fi
echo "Setup complete. To activate the virtual environment, run 'source venv/bin/activate'."