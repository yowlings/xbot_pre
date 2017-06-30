find . -name package.xml -exec dirname {} \; > file1
curdir=$(pwd)
mkdir deb
for p in $(cat file1)
do
    cd $p
    echo "bloom-generate $p"
    bloom-generate rosdebian --os-name ubuntu --os-version xenial --ros-distro kinetic
    fakeroot debian/rules binary
    cd $curdir
    mv *.deb ./deb/
    sudo dpkg -i ./deb/*.deb
done
rm file1
