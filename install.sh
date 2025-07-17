cd "$(dirname "$0")/src"
(cd vlidar && miniros install)
(cd vmovement && miniros install)
(cd vpathfinder && miniros install)
(cd vslam && miniros install)
(cd vmain && miniros install)