
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int8


# Subscriber Messages
from trz_can_msg.msg import TRZSFTINFO
from trz_can_msg.msg import TRZSPDINFO
from trz_can_msg.msg import TRZSTRINFO
from trz_can_msg.msg import TRZACCINFO
from autoware_auto_vehicle_msgs.msg import VelocityReport
from geometry_msgs.msg import PoseWithCovarianceStamped


import socket
import threading
import struct
import time
import math

class UDPInterface(Node):

    def __init__(self):
        super().__init__('udp_interface')

        # ROS2 Parameters
        self.declare_parameter('udp_local_port', 5001)
        self.declare_parameter('allowed_ips', ['165.229.50.126'])
        self.declare_parameter('send_rate_hz', 10.0)
        self.declare_parameter('gps_topic', '/ublox_gps_node/fix')
        self.declare_parameter('client_timeout', 6.0)

        self.udp_local_port = self.get_parameter('udp_local_port').value
        self.allowed_ips = list(self.get_parameter('allowed_ips').value)
        self.send_rate_hz = self.get_parameter('send_rate_hz').value
        self.gps_topic = self.get_parameter('gps_topic').value
        self.client_timeout = self.get_parameter('client_timeout').value

        # GPS Data
        self.latitude = 0.0
        self.longitude = 0.0
        self.gps_received = False

        # vehicle Data
        self.heading_angle = 0.0
        self.gear_state = 0 # 0 : P, 1 : D, 2 : N, 3 : R
        self.speed = 0.0
        self.automode = 0 # 0 : Off, 1 : On
        self.brake = 0 # 0 : Off, 1 : On
        self.acc_mode = 0 # 0 : Off, 1 : On
        self.str_mode = 0 # 0 : Off, 1 : On

        # ROS2 Subscriber
        self.gps_sub = self.create_subscription(NavSatFix, self.gps_topic, self.gps_callback, 10)
        self.gear_state_sub = self.create_subscription(TRZSFTINFO, '/can/receive/TRZ_SFT_INFO', self.gear_state_callback, 10)
        self.speed_sub = self.create_subscription(VelocityReport, '/vehicle/status/velocity_status', self.speed_callback, 10)
        self.str_sub = self.create_subscription(TRZSTRINFO, '/can/receive/TRZ_STR_INFO', self.str_callback, 10)
        self.acc_sub = self.create_subscription(TRZACCINFO, '/can/receive/TRZ_ACC_INFO', self.acc_callback, 10)
        self.initial_pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initial_pose_callback, 10)


        # UDP Socket
        self.sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM
        )

        # Ubuntu 모든 인터페이스에서 UDP 5001 수신
        self.sock.bind(('0.0.0.0', self.udp_local_port))
        self.windows_addrs = {}

        # receive thread / ROS timer 동시 접근 보호
        self.clients_lock = threading.Lock()

        # UDP Receive Thread
        self.recv_thread = threading.Thread(target=self.receive_udp, daemon=True)
        self.recv_thread.start()

        # UDP Send Timer
        self.timer = self.create_timer(
            1.0 / self.send_rate_hz,
            self.send_udp
        )

        # Startup Log
        self.get_logger().info('================================')
        self.get_logger().info('ACSLAB UDP Interface Started')
        self.get_logger().info(f'Local Port : {self.udp_local_port}')
        self.get_logger().info(f'Allowed IPs : {self.allowed_ips}')
        self.get_logger().info(f'GPS Topic  : {self.gps_topic}')
        self.get_logger().info(f'Send Rate  : {self.send_rate_hz} Hz')
        self.get_logger().info(f'Client Timeout : {self.client_timeout} sec')
        self.get_logger().info('Waiting for Windows CONNECT...')
        self.get_logger().info('================================')

    # Gear State Callback
    def gear_state_callback(self, msg):
        self.gear_state = msg.trz_sft_gear_stat

    # Speed Callback
    def speed_callback(self, msg):
        self.speed = msg.longitudinal_velocity

    def update_automode(self):
        self.automode = int(self.acc_mode == 1 or self.str_mode == 1)

    def str_callback(self, msg):
        self.str_mode = msg.trz_mdps_stat
        self.update_automode()

    def acc_callback(self, msg):
        self.acc_mode = msg.trz_scc_stat
        self.update_automode()

    # GPS Callback
    def gps_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.gps_received = True

    def initial_pose_callback(self, msg):
        q = msg.pose.pose.orientation

        # Quaternion -> Yaw (radian)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        # ROS Yaw -> Heading (degree)
        self.heading_angle = 90.0 - math.degrees(yaw)

        if self.heading_angle < 0.0:
            self.heading_angle += 360.0

    # UDP Receive
    # Windows -> Ubunt====
    def receive_udp(self):
        while rclpy.ok():
            try:
                data, addr = self.sock.recvfrom(4096)
                sender_ip = addr[0]

                # IP Filtering
                if sender_ip not in self.allowed_ips:
                    self.get_logger().warning(f'BLOCKED: {addr}')
                    continue

                if data == b'CONNECT':
                    current_time = time.monotonic()
                    with self.clients_lock:
                        is_new_client = addr not in self.windows_addrs

                        # 신규 클라이언트 등록 또는 기존 수신 시간 갱신
                        self.windows_addrs[addr] = current_time
                        client_count = len(self.windows_addrs)

                    if is_new_client:
                        self.get_logger().info(
                            f'Windows connected: {addr} | '
                            f'Total clients: {client_count}'
                        )

            except OSError:
                break

            except Exception as e:
                self.get_logger().error(f'UDP receive error: {e}')

    # UDP Send
    # Ubuntu -> Window
    def send_udp(self):
        current_time = time.monotonic()

        with self.clients_lock:

            expired_clients = [
                addr
                for addr, last_seen in self.windows_addrs.items()
                if current_time - last_seen > self.client_timeout
            ]

            for addr in expired_clients:
                del self.windows_addrs[addr]

            clients = list(self.windows_addrs.keys())
            client_count = len(self.windows_addrs)

        for addr in expired_clients:
            self.get_logger().warning(
                f'Windows disconnected (timeout): {addr} | '
                f'Total clients: {client_count}'
            )

        # GPS 수신 전에는 송신하지 않음
        if not self.gps_received:
            return

        # 연결된 PC가 없으면 송신하지 않음
        if not clients:
            return

        timestamp = time.time()

        try:
            packet = struct.pack(
                '!i3dfif2i',
                0,                   # Vehicle type
                timestamp,           # Timestamp
                self.latitude,       # Latitude
                self.longitude,      # Longitude
                self.heading_angle,  # Heading
                self.gear_state,     # Gear
                self.speed,          # Speed
                self.automode,       # Auto mode
                self.brake           # Brake
            )

        except (struct.error, ValueError, TypeError) as e:
            self.get_logger().error(f'Packet error: {e}')
            return

        # 모든 Windows PC에 동일한 패킷 송신
        for addr in clients:
            try:
                self.sock.sendto(packet, addr)

                self.get_logger().info(
                    f'SEND -> {addr} | '
                    f'Time={timestamp:.6f} | '
                    f'Lat={self.latitude:.8f} | '
                    f'Lon={self.longitude:.8f}'
                )

            except OSError as e:
                self.get_logger().error(
                    f'UDP send error -> {addr}: {e}'
                )
    # Shutdown
    def close(self):
        try:
            self.sock.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = UDPInterface()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()