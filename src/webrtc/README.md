# Webrtc data channel class in progress

## TODO
- [x] OOP + testing

## Commands
```bash
# Build
make

# Server
./callee

# Client
./caller
```

## Installaion
```bash
rm -rf libdatachannel
git clone --recursive https://github.com/paullouisageneau/libdatachannel.git
cd libdatachannel
mkdir build
cd build
cmake ..
make -j$(nproc)
sudo make install

# Add the library to library path
exporet LD_LIBRARY_PATH=~/libdatachannel/build:$LD_LIBRARY_PATH 
```