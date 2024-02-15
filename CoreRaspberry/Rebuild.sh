cd /home/dev/RaspberryCore/
git pull

cd /home/dev/RaspberryCore/CoreRaspberry/
rm -r build install log
colcon build --symlink-install