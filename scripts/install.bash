cd "$(dirname "$0")/src"
(cd simulator && miniros install)
(cd vlidar && miniros install)
(cd vmovement && miniros install)
(cd vpathfinder && miniros install)
(cd vslam && miniros install)
(cd vmain && miniros install)