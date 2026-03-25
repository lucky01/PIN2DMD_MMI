#!/bin/bash

sudo systemctl stop serum
sudo systemctl disable serum
sudo cp serum.service /etc/systemd/system/serum.service
sudo mkdir '/opt/serum'
sudo cp ../build/spi-serum /opt/serum/.
sudo systemctl enable serum
sudo systemctl start serum

