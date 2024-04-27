sudo systemctl stop brltty-udev.service
sudo systemctl mask brltty-udev.service
sudo systemctl stop brltty.service
sudo systemctl disable brltty.service
sudo systemctl set-default multi-user
sudo usermod -a -G tty dev
sudo usermod -a -G dialout dev
