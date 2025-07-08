# 1. 创建永久配置
sudo nmcli con add type ethernet ifname vnic0 \
    con-name vnic0-static \
    ip4 1.1.1.1/8 \
    gw4 1.1.1.254

# 2. 激活配置
sudo nmcli con up vnic0-static

# 3. 设置开机自启
#sudo nmcli con modify vnic0-static connection.autoconnect yes