@echo off
start miniros server --superserver ./cfg/superserver.config.json
start miniros run vlidar
start miniros run vslam
start miniros run vpathfinder
start miniros run vmovement
miniros run vmain