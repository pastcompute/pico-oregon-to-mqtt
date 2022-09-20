if [ -z $PICO_SDK_PATH ] ; then
  echo -e "\e[31mPICO_SDK_PATH not found, did you set up the Pico SDK properly?\e[0m"
  exit 1
fi

mkdir -p lib

mkdir -p build
cd build
cmake ..
