# <div align="center"> Metamorphosis Task </div>

```Task project for Metamorphosis```

## Build instructions
### First you will need to set up all the dependencies:

Install PCL:
```bash
sudo apt install libpcl-dev
```

Get all the git submodules (this is for the nlohmann json library)
```bash
git submodule update --init --recursive 
```

### Once you're done it is time to build the project using CMake:
Run the following commands from the root folder:
```bash
cmake -S . -B build
cmake --build build
```

This should build the project, and the executable can be found in:
```bash
./build/metamorphosis-project
```

## Usage

```bash
./build/metamorphosis-project resources/back_closeup_body_realigned.ply resources/back_closeup_body_realigned.json
```

