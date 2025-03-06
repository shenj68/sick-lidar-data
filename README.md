# sick-lidar-data

Drivers: https://github.com/SICKAG/sick_safetyscanners_base

Working for:
Nvidia Jetson Orin AGX
Jetpack 6.0 (L4T 36.3.0)

### Installation

For installation this github repository has to be cloned and afterwards installed. If a custom install directory is wanted use the -DCMAKE_INSTALL_PREFIX option to specify a path.

Install boost:
```
sudo apt-get install libboost-system-dev
sudo apt-get install libboost-thread-dev
sudo apt-get install libboost-chrono-dev
```

```bash
git clone https://github.com/SICKAG/sick_safetyscanners_base.git
cd sick_safetyscanners_base
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=<path to install folder> ..
make -j8
make install
```

### Usage

To use the library in a driver the path of the installation has to be added to the cmake prefix path of your application. You can achieve this by using, bevor invoking cmake on your application.

```
export CMAKE_PREFIX_PATH=<path to install folder>
```