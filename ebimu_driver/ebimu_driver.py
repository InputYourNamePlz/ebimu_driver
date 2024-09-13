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
import threading

class EbimuDriver(Node):
    def __init__(self):
        super().__init__('ebimu_driver')
        self.publisher = self.create_publisher(Imu, 'imu', 10)
        self.serial_port = serial.Serial('/dev/ttyIMU', 115200, timeout=1)  # 시리얼포트는 tty 고정해서 쓰면 좋아용
        #self.timer = self.create_timer(0.01, self.timer_callback)  # 100Hz, 필요에 따라 조정
        self.timer2 = self.create_timer(0.1, self.timer2_callback)
        #self.serial_port.write("<sof2>".encode()) # quaternion 모드로 설정
        #self.serial_port.write("<sem0>".encode()) # 자기장 센서 끄고 싶을 때


        self.thread = threading.Thread(target=self.serial_read_thread)
        self.thread.daemon = True
        self.thread.start()

        

        self.qz=0.0
        self.qy=0.0
        self.qx=0.0
        self.qw=0.0
        self.gx=0.0
        self.gy=0.0
        self.gz=0.0
        self.ax=0.0
        self.ay=0.0
        self.az=0.0
        
        self.is_upside_down=False
        


    
    def timer2_callback(self):  

        qw=self.qw
        qx=-self.qy
        qy=self.qx
        qz=self.qz

        gx=self.gy
        gy=self.gx
        gz=self.gz

        ax=self.ay  
        ay=self.ax
        az=self.az

            
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        
        imu_msg.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        imu_msg.angular_velocity = Vector3(x=gx, y=gy, z=gz)
        imu_msg.linear_acceleration = Vector3(x=ax, y=ay, z=az)
        
        self.publisher.publish(imu_msg)
        print(f"IMU data published: {qx, qy, qz, qw, gx, gy, gz, ax, ay, az}")

    
    def serial_read_thread(self):
        while rclpy.ok():
            try:
                if self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode('utf-8').strip()

                if line.startswith('*'):
                    data = line[1:].split(',')
                    if len(data) == 10:  # 예상되는 데이터 개수
                        self.qz, self.qy, self.qx, self.qw, self.gx, self.gy, self.gz, self.ax, self.ay, self.az = map(float, data)
                else:
                    print(f"data length is {len(data)}")
            except Exception as e:
                self.get_logger().error(f'Error reading from serial port: {str(e)}')


    def timer_callback(self):
        try:
            line = self.serial_port.readline().decode('utf-8').strip()

            if line.startswith('*'):
                data = line[1:].split(',')
                if len(data) == 10:  # 예상되는 데이터 개수
                    self.qz, self.qy, self.qx, self.qw, self.gx, self.gy, self.gz, self.ax, self.ay, self.az = map(float, data)
            else:
                print(f"data length is {len(data)}")
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
