#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/baxter/Desktop/PierreROS/src/baxter_examples"

# snsure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/baxter/Desktop/PierreROS/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/baxter/Desktop/PierreROS/install/lib/python2.7/dist-packages:/home/baxter/Desktop/PierreROS/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/baxter/Desktop/PierreROS/build" \
    "/usr/bin/python" \
    "/home/baxter/Desktop/PierreROS/src/baxter_examples/setup.py" \
    build --build-base "/home/baxter/Desktop/PierreROS/build/baxter_examples" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/baxter/Desktop/PierreROS/install" --install-scripts="/home/baxter/Desktop/PierreROS/install/bin"
