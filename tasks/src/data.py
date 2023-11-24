def read_yaml_file(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)





# Class for accesing the Robots Data in the state machine
class SharedData:
    def __init__(self):
        self.zed_data = {}
        self.dvl_data = None
        self.imu_data = {'yaw': None, 'pitch': None, 'roll': None}
        # Add more attributes as needed

        # Match simulation Pose
        self.pose = None



# Global variable for accesing the shared_data 
shared_data = SharedData()

def zed_objects_callback(msg):
    shared_data.zed_data['ObjectsStamped'] = msg

def zed_rgbd_callback(msg):
    # shared_data.zed_data[]
    pass

def dvl_callback(msg):
    shared_data.dvl_data = msg

def imu_yaw_callback(msg):
    shared_data.imu_data['yaw'] = msg.data

def pose_callback(msg):
    shared_data.pose = msg



def initialize_subscribers(topics: str):
    topics_info = read_yaml_file(topics)
    rospy.Subscriber(topics_info['zed_camera']['ObjectsStamped'], Float32, zed_objects_callback)  # Update the message type
    #rospy.Subscriber(topics_info['zed_camera']['rgbd_sensor'], Float32, zed_objects_callback)  # Update the message type
    rospy.Subscriber(topics_info['dvl']['DVL_Message'],  dvl_callback)  # Update the message type
    rospy.Subscriber(topics_info['imu']['yaw'], imu_yaw_callback)
    rospy.Subscriber(topics_info['imu']['yaw'],  pose_callback)