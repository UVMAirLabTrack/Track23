cd /home/ian/RaspberryCore/
git pull

cd /home/ian/RaspberryCore/CoreRaspberry/
rm -r build install log
colcon build --symlink-install
