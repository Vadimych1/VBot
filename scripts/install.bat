@echo off

cd %~dp0
cd ../src

pushd simulator
call miniros install
popd

pushd vlidar
call miniros install
popd

pushd vmovement
call miniros install
popd

pushd vpathfinder
call miniros install 
popd

pushd vslam 
call miniros install 
popd

pushd vcam
call miniros install
popd

pushd vmain
call miniros install
popd

pushd ssmain
call miniros install
popd