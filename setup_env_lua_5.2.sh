#!/bin/sh
sudo apt-get install lua5.2 liblua5.2-dev
wget http://luarocks.github.io/luarocks/releases/luarocks-2.4.4.tar.gz
tar zxvf luarocks-2.4.4.tar.gz
cd luarocks-2.4.4/
./configure --lua-version=5.2
make build
sudo make install
cd ..
rm -rf luarocks-2.4.4
rm luarocks-2.4.4.tar.gz
cd etc
luarocks install --local --only-deps ilkcompiler-0.4.0-1.rockspec
cd ..
eval `luarocks path`
