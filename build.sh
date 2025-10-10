#!/bin/bash

# 删除 build 文件夹（如果存在）

# 执行 cmake ..
cd build
cmake ..

# 执行 make -j20
make -j20