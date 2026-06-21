2. Rock5C 检查 CANable

插上 MKS CANable 后：

lsusb
dmesg | tail -50
ip link

如果没有，先看是不是 slcan：

ls /dev/ttyACM*

3. 启动 can0，1 Mbps

如果已经有 can0：

sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details link show can0

装调试工具：

sudo apt update
sudo apt install -y can-utils python3-pip

监听：

candump can0

另一个 SSH 窗口可以试发：

cansend can0 001#0000000000000000
cansend can0 002#0000000000000000


先重置 CAN，并把队列加大：

sudo ip link set can0 down
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up
ip -details -statistics link show can0

sudo ip link set can0 down
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up
python3 ~/vla_gripper_force_backup.py