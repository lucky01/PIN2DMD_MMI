systemctl stop serum
systemctl disable serum
cp serum.service /etc/systemd/system/serum.service
cp ../build/spi-serum /usr/local/bin/.
systemctl enable serum
systemctl start serum

