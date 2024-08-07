import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
try:
    import serial
except:
    import pip
    pip.main(['install', pyserial])
    import serial
import math

class EbimuDriver(Node):
    def __init__(self):
        super().__init__('ebimu_driver')
        self.publisher = self.create_publisher(Imu, 'imu', 10)
        self.serial_port = serial.Serial('/dev/ttyIMU', 115200, timeout=1)  # 시리얼포트는 tty 고정해서 쓰면 좋아용
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100Hz, 필요에 따라 조정
        self.serial_port.write("<sof2>".encode()) # quaternion 모드로 설정
        #self.serial_port.write("<sem0>".encode()) # 자기장 센서 끄고 싶을 때

    def timer_callback(self):
        try:
            line = self.serial_port.readline().decode('utf-8').strip()

            if line.startswith('*'):
                data = line[1:].split(',')
                if len(data) == 10:  # 예상되는 데이터 개수
                    qz, qy, qx, qw, gx, gy, gz, ax, ay, az = map(float, data)
                    
                    imu_msg = Imu()
                    imu_msg.header.stamp = self.get_clock().now().to_msg()
                    imu_msg.header.frame_id = 'imu_link'
                    
                    imu_msg.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
                    imu_msg.angular_velocity = Vector3(x=gx, y=gy, z=gz)
                    imu_msg.linear_acceleration = Vector3(x=ax, y=ay, z=az)
                    
                    self.publisher.publish(imu_msg)
                    print("IMU data published: "+str(data))
        except Exception as e:
            self.get_logger().error(f'Error reading from serial port: {str(e)}')


    def __del__(self):
        self.serial_port.close()

def main(args=None):
    rclpy.init(args=args)
    ebimu_driver = EbimuDriver()
    rclpy.spin(ebimu_driver)
    ebimu_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
