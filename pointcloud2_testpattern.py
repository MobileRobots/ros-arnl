from __future__ import division # gives floating point values from integer division
import sys
import rospy
import std_msgs
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler

rospy.init_node('pointcloud2_testpattern')
pointcloud_publisher = rospy.Publisher('/pc2_testpattern', PointCloud2, queue_size=3)

header = std_msgs.msg.Header(frame_id='sensor')
fields = [
  PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
  PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
  PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
  PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
] 

color = 0x00ff00

rate = rospy.Rate(10)

w = 1
d = 1
h = 1
np = 6
n = 4
s = 1
sg = -1

animate = False

tfBroadcaster = tf2_ros.TransformBroadcaster()
tf = geometry_msgs.msg.TransformStamped()
tf.header.frame_id = 'base_link' # e.g. map, odom, base_link, camera_frame
tf.child_frame_id = 'sensor'
tf.transform.translation.x = 0 # TODO vary position over time like the data below
tf.transform.translation.y = 0
tf.transform.translation.z = 0
q = quaternion_from_euler(0, 0, 0)
print q
tf.transform.rotation.x = q[0]
tf.transform.rotation.y = q[1]
tf.transform.rotation.z = q[2]
tf.transform.rotation.w = q[3]

points = []

def genpoints(s):
  x = 1
  y = -(n*w)/2 
  z = 0
  for i in range(0,n):
    for iy1 in range(0,np):
      points.append([x, y, z, color])
      y += w*s/np
    for ix1 in range(0,np):
      points.append([x, y, z, color])
      x += d*s/np
    for iz1 in range(0, np):
      points.append([x, y, z, color])
      z += h*s/np
    for iy2 in range(0,np):
      points.append([x, y, z, color])
      y += w*s/np
    for ix2 in range(0,np):
      points.append([x, y, z, color])
      x -= d*s/np
    for iz2 in range(0, np):
      points.append([x, y, z, color])
      z -= h*s/np


if not animate:
 genpoints(1)

while not rospy.is_shutdown():
  if animate:
    genpoints(s)
    s += 0.02 * sg
    if s <= 0.2:
      s = 0.2 
      sg *= -1
    elif s >= 1:
      s = 1
      sg *= -1

  t = rospy.Time.now()

  tf.header.stamp = t
  tfBroadcaster.sendTransform(tf)

  cloud = pc2.create_cloud(header, fields, points)
  cloud.header.stamp = t

  pointcloud_publisher.publish(cloud)
  

  print '.',
  sys.stdout.flush()
  rate.sleep()


