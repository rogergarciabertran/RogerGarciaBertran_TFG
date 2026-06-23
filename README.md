# RogerGarciaBertran_TFG
Repositori amb els codis complets que formen el sistema del crawler i la seva configuració.
# Requisits Principals
·Raspberry Pi amb Ubuntu
·ROS 2 Jazzy
·Python 3
·Paquets de ROS 2 necessaris per executar nodes, launch files i missatges estàndard
·Llibreries de GPIO per al control de pins de la Raspberry Pi
·Portàtil Windows per executar el script del comandament Xbox
# Com compilar
colcon build --symlink-install
# Launch Manual del sistema 
cd /home/crawler/crawler_ws 
source /opt/ros/jazzy/setup.bash 
source /home/crawler/crawler_ws/install/setup.bash export 
ROS_DOMAIN_ID=10 
ros2 launch crawler_control bringup.launch.py use_joy:=false use_udp_joy:=true
#  Instalar launch automàtic 
sudo cp systemd/crawler_bringup.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable crawler_bringup.service
sudo systemctl start crawler_bringup.service
# Comandament des de Windows
Abans d’executar-lo, cal assegurar-se que la IP de la Raspberry indicada al fitxer send_xbox_udp.py és correcta:
RASPBERRY_IP = "IP_DE_LA_RASPBERRY"
UDP_PORT = 5005
- Desde Powershell com admin
cd "$env:USERPROFILE\Desktop"
python send_xbox_udp.py
# Visualitzar telemetria del sistema 
journalctl -u crawler_bringup.service -f
# Tòpics principals 
-'/joy' 
/crawler/motor1/cmd 
/crawler/motor2/cmd 
/crawler/motor1/dac_v 
/crawler/motor2/dac_v 
/crawler/motor1/fg_hz 
/crawler/motor2/fg_hz 
/crawler/motor1/rpm 
/crawler/motor2/rpm 
/crawler/power_enable 
/crawler/imu/data
/crawler/imu/mag
