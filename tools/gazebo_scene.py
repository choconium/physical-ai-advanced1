"""Visual-only tool proxy and visible ink, driven by measured ROS TF."""
import xml.etree.ElementTree as ET
from gz.transport13 import Node
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean


class GazeboScene:
    def __init__(self):
        self.transport = Node()
        self.world = 'empty'  # pinned AIWorker2.3.1 default.sdf

    def request(self, suffix, message, cls, timeout=2000):
        ok, response = self.transport.request('/world/' + self.world + '/' + suffix,
            message, cls, Boolean, timeout)
        if not ok or not response.data:
            raise RuntimeError('Gazebo request failed: ' + suffix)

    def create_xml(self, model_xml):
        req = EntityFactory(sdf='<sdf version="1.10">' + model_xml + '</sdf>')
        self.request('create', req, EntityFactory)

    def spawn_board(self, sdf):
        root = ET.fromstring(sdf)
        for model in root.findall('./world/model'):
            self.create_xml(ET.tostring(model, encoding='unicode'))
        self.create_xml('''<model name="sim_eraser"><static>true</static><pose>0 0 -10 0 0 0</pose>
          <link name="visual_proxy"><visual name="pad"><geometry><box><size>.045 .07 .04</size></box></geometry>
          <material><ambient>0.1 0.3 0.8 1</ambient><diffuse>0.1 0.3 0.8 1</diffuse></material></visual></link></model>''')

    def erase(self, name):
        self.request('remove', Entity(name=name, type=Entity.MODEL), Entity)

    def set_tool(self, xyz, xyzw):
        req = Pose(name='sim_eraser')
        req.position.x, req.position.y, req.position.z = xyz
        req.orientation.x, req.orientation.y, req.orientation.z, req.orientation.w = xyzw
        self.request('set_pose', req, Pose, timeout=500)
