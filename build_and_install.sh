#!/bin/bash

# build
python3 -m build

# install
pip3 install  --no-index --find-links=./dist/ CL57T_Raspberry_Pi

# upload
# twine upload --skip-existing dist/*
