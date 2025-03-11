#!/bin/bash
################################################################################
# Copyright (c) 2023 ModalAI, Inc. All rights reserved.
################################################################################

# Script to install build dependencies in voxl-cross docker image

# list all your dependencies here. Note for packages that have AMD64 equivalents
# in the ubuntu repositories you should specify the arm64 architecture to make
# sure the correct one is installed in voxl-cross.
DEPS_QRB5165="
libmodal-json
libmodal-pipe
libvoxl-cutils
voxl-ros2-foxy"

## this list is just for tab-completion
## it's not an exhaustive list of platforms available.
AVAILABLE_PLATFORMS="qrb5165"


print_usage(){
    echo ""
    echo " Install build dependencies from a specified repository."
    echo ""
    echo " Usage:"
    echo "  ./install_build_deps.sh {platform} {section}"
    echo ""
    echo " Examples:"
    echo ""
    echo "  ./install_build_deps.sh qrb5165 dev"
    echo "        Install from qrb5165 development repo."
    echo ""
    echo "  ./install_build_deps.sh qrb5165 sdk-1.0"
    echo "        Install from qrb5165 sdk-1.0 repo."
    echo ""
    echo ""
    echo " These examples are not an exhaustive list."
    echo " Any platform and section in this deb repo can be used:"
    echo "     http://voxl-packages.modalai.com/dists/"
    echo ""
}

# make sure two arguments were given
if [ "$#" -ne 2 ]; then
    print_usage
    exit 1
fi
MODE="DEB"

## convert arguments to lower case for robustness
PLATFORM=$(echo "$1" | tr '[:upper:]' '[:lower:]')
SECTION=$(echo "$2" | tr '[:upper:]' '[:lower:]')

# install deb packages with apt
if [ "$MODE" == "DEB" ]; then
    echo "using $PLATFORM $SECTION debian repo"
    # write in the new entry
    DPKG_FILE="/etc/apt/sources.list.d/modalai.list"
    LINE="deb [trusted=yes] http://voxl-packages.modalai.com/ ./dists/$PLATFORM/$SECTION/binary-arm64/"
    echo "${LINE}" > ${DPKG_FILE}

    ## make sure we have the latest package index
    ## only pull from voxl-packages to save time
    apt-get update -o Dir::Etc::sourcelist="sources.list.d/modalai.list" -o Dir::Etc::sourceparts="-" -o APT::Get::List-Cleanup="0"

    ## install the user's list of dependencies
    echo "installing: $DEPS_QRB5165"
    apt update
    apt install --fix-missing
    apt install -y $DEPS_QRB5165
    apt install -y libeigen3-dev
    apt install -y python3-pip
    apt install -y python3-numpy
    apt install -y libyaml-cpp-dev
    apt install -y nlohmann-json-dev
    python3 -m pip install pip -U
    python3 -m pip install colcon-common-extensions lark
    python3 -m pip uninstall -y empy
    python3 -m pip install empy==3.3.4

fi

echo ""
echo "Done installing dependencies"
echo ""
exit 0
