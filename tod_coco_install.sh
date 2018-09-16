#!/bin/bash
set -e

# From instructions at
#   https://github.com/alex-lechner/Traffic-Light-Classification#linux


# Install dependencies.
apt-get --yes install protobuf-compiler python-pil python-lxml python-tk wget unzip
pip install --user Cython contextlib2 jupyter matplotlib


# Install TOD API.
cd
git clone https://github.com/tensorflow/models.git
# Go to a particular commit. This is important because the code from the master
# branch won't work with TensorFlow version 1.4.
# Also, this commit has already fixed broken models from previous commits.
cd models && git checkout f7e99c0


# Get protoc compiler 3.3.0
# TODO: Remove if unnecessary.
cd
wget https://github.com/google/protobuf/releases/download/v3.3.0/protoc-3.3.0-linux-x86_64.zip
unzip protoc-3.3.0-linux-x86_64.zip  -d protoc330
export PROTOC="$(pwd)/protoc330/bin/protoc"


# Compile protobuf files
cd
cd models/research/
"$PROTOC" object_detection/protos/*.proto --python_out=.
export PYTHONPATH=$PYTHONPATH:`pwd`:`pwd`/slim
echo "export PYTHONPATH=\$PYTHONPATH:`pwd`:`pwd`/slim" >> ~/.bashrc
python object_detection/builders/model_builder_test.py


# Install COCO API.
# TODO: Remove if unnecessary.
cd
git clone https://github.com/cocodataset/cocoapi.git
cd cocoapi/PythonAPI
make
cp -r pycocotools ~/models/research/
cd
