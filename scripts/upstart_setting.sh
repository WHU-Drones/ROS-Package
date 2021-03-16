sudo cp link.service /etc/systemd/system/
chmod +x link.sh
sudo systemctl daemon-reload
sudo systemctl enable link
sudo systemctl start link
