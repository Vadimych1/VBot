@echo off

cd %~dp0
cd ..

start miniros --trace server --superserver ./cfg/local-superserver.config.json
start miniros run ssmain ./cfg/ssmain.config.json ./cfg/local-superserver.config.json
start miniros run simulator
start miniros run vslam
start miniros run vpathfinder

miniros run vmain ./cfg/local-superserver.config.json