

def model(size, leg_radius, leg_length, id):
    lr = leg_radius
    ll = leg_length
    lp = size/2.0 - lr
    base_model = f'''<?xml version="1.0" ?>
<sdf version="1.5">
  <model name="carrier">
    <static>false</static>
    <link name="base_link">
      <inertial> <mass>1.0</mass> <inertia><ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz></inertia> </inertial>
      <collision name="surface">
        <pose>0 0 {ll+0.005} 0 0 0</pose>
        <geometry> <box> <size>{size} {size} 0.01</size> </box> </geometry>
        <surface> <friction> <ode> <mu>Inf</mu> <mu2>Inf</mu2> </ode> </friction> </surface>
      </collision>
      <visual name="visual1">
        <pose>0 0 {ll+0.005} 0 0 0</pose>
        <geometry> <box> <size>{size} {size} 0.01</size> </box> </geometry>
        <material> <script> <uri>file://media/materials/scripts/gazebo.material</uri> <name>Gazebo/GreyTransparent</name> </script> </material>
      </visual>
      <visual name="tagn">
        <pose>0 {ll+0.005} 0.200 1.5708 1.5708 0</pose>
        <geometry> <box> <size>0.05 0.05 0.001</size> </box> </geometry>
        <material> <script> <uri>file://media/materials/scripts/tag.material</uri> <name>tag{id}</name> </script> </material>
      </visual>
      <visual name="tags">
        <pose>0 -{ll+0.005} 0.200 1.5708 -1.5708 0</pose>
        <geometry> <box> <size>0.05 0.05 0.001</size> </box> </geometry>
        <material> <script> <uri>file://media/materials/scripts/tag.material</uri> <name>tag{id}</name> </script> </material>
      </visual>
      <visual name="tage">
        <pose>{ll+0.005} 0 0.200 1.5708 -1.5708 1.5708</pose>
        <geometry> <box> <size>0.05 0.05 0.001</size> </box> </geometry>
        <material> <script> <uri>file://media/materials/scripts/tag.material</uri> <name>tag{id}</name> </script> </material>
      </visual>
      <visual name="tagw">
        <pose>-{ll+0.005} 0 0.200 1.5708 1.5708 1.5708</pose>
        <geometry> <box> <size>0.05 0.05 0.001</size> </box> </geometry>
        <material> <script> <uri>file://media/materials/scripts/tag.material</uri> <name>tag{id}</name> </script> </material>
      </visual>
      <visual name="tagt">
        <pose>0.0 0 {ll} 0 0 0</pose>
        <geometry> <box> <size>0.3 0.3 0.001</size> </box> </geometry>
        <material> <script> <uri>file://media/materials/scripts/tag.material</uri> <name>mm49_target</name> </script> </material>
      </visual>
      <collision name="front_left_leg">
        <pose>{lp} {lp} 0.09 0 0 0</pose>
        <geometry> <cylinder> <radius>{lr}</radius> <length>{ll}</length> </cylinder> </geometry>
        <surface> <friction> <ode> <mu>0.1</mu> <mu2>0.1</mu2> </ode> </friction> </surface>
      </collision>
      <visual name="front_left_leg">
        <pose>{lp} {lp} 0.09 0 0 0</pose>
        <geometry> <cylinder> <radius>{lr}</radius> <length>{ll}</length> </cylinder> </geometry>
        <material> <script> <uri>file://media/materials/scripts/gazebo.material</uri> <name>Gazebo/White</name> </script> </material>
      </visual>
      <collision name="front_right_leg">
        <pose>{lp} -{lp} 0.09 0 0 0</pose>
        <geometry> <cylinder> <radius>{lr}</radius> <length>{ll}</length> </cylinder> </geometry>
        <surface> <friction> <ode> <mu>0.1</mu> <mu2>0.1</mu2> </ode> </friction> </surface>
      </collision>
      <visual name="front_right_leg">
        <pose>{lp} -{lp} 0.09 0 0 0</pose>
        <geometry> <cylinder> <radius>{lr}</radius> <length>{ll}</length> </cylinder> </geometry>
        <material> <script> <uri>file://media/materials/scripts/gazebo.material</uri> <name>Gazebo/White</name> </script> </material>
      </visual>
      <collision name="back_right_leg">
        <pose>-{lp} -{lp} 0.09 0 0 0</pose>
        <geometry> <cylinder> <radius>{lr}</radius> <length>{ll}</length> </cylinder> </geometry>
        <surface> <friction> <ode> <mu>0.1</mu> <mu2>0.1</mu2> </ode> </friction> </surface>
      </collision>
      <visual name="back_right_leg">
        <pose>-{lp} -{lp} 0.09 0 0 0</pose>
        <geometry> <cylinder> <radius>{lr}</radius> <length>{ll}</length> </cylinder> </geometry>
        <material> <script> <uri>file://media/materials/scripts/gazebo.material</uri> <name>Gazebo/White</name> </script> </material>
      </visual>
      <collision name="back_left_leg">
        <pose>-{lp} {lp} 0.09 0 0 0</pose>
        <geometry> <cylinder> <radius>{lr}</radius> <length>{ll}</length> </cylinder> </geometry>
        <surface> <friction> <ode> <mu>0.1</mu> <mu2>0.1</mu2> </ode> </friction> </surface>
      </collision>
      <visual name="back_left_leg">
        <pose>-{lp} {lp} 0.09 0 0 0</pose>
        <geometry> <cylinder> <radius>{lr}</radius> <length>{ll}</length> </cylinder> </geometry>
        <material> <script> <uri>file://media/materials/scripts/gazebo.material</uri> <name>Gazebo/White</name> </script> </material>
      </visual>
    </link>
    <plugin name="gazebo_ros_p3d_named" filename="libgazebo_ros_p3d_named.so">
      <ros> <remapping>odom:=/ground_truth</remapping> <namespace>carrier{id}</namespace> </ros>
      <body_name>base_link</body_name>
      <frame_name>map</frame_name>
      <child_frame>carrier{id}</child_frame>
      <update_rate>50</update_rate>
      <xyz_offset>0.0 0.0 0.0</xyz_offset>
      <rpy_offset>0.0 0.0 0.0</rpy_offset>
      <gaussian_noise>0.01</gaussian_noise>
    </plugin>
  </model>
</sdf>'''
    return base_model

for tid in range(100, 120):
    mstring = model(size=0.36, leg_radius=0.018, leg_length=0.175, id=tid)
    print('Generating %d\n%s' % (tid, mstring))
    with open ('src/dots_gazebo/dots_sim/models/carrier%02d/model.sdf' % tid, 'w') as f:
        f.write(mstring)


