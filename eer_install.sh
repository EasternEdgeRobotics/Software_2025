current_dir=$(dirname "$(readlink -f "$0")")

sudo ln -s ${current_dir}/CppGUI//EER/build/Desktop_Qt_6_8_0-Release/EER /usr/bin

