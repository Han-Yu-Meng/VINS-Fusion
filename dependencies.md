Install Ceres Solver

```
sudo apt-get update
sudo apt-get install libgoogle-glog-dev libgflags-dev
sudo apt-get install libatlas-base-dev
sudo apt-get install libsuitesparse-dev

cd ~/Kit
git clone https://github.com/ceres-solver/ceres-solver.git
cd ceres-solver
git checkout 1.14.0

cmake .. \
    -DBUILD_TESTING=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DCMAKE_CXX_STANDARD=14

make -j4
sudo make install

```