import importlib.util
import tempfile
import unittest
from pathlib import Path
import xml.etree.ElementTree as ET
try:
    import cv2
    import yaml
    CV=True
except ImportError:
    CV=False


@unittest.skipUnless(CV,'Requires OpenCV and YAML')
class AssetTests(unittest.TestCase):
    def test_generated_world_and_real_marker_ids(self):
        script=Path(__file__).parents[1]/'scripts/prepare_sitl.py'
        spec=importlib.util.spec_from_file_location('prepare_sitl',script)
        mod=importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); px4=root/'px4'; out=root/'assets'
            default=px4/'Tools/simulation/gz/worlds/default.sdf'
            default.parent.mkdir(parents=True)
            default.write_text('<sdf version="1.9"><world name="default"/></sdf>')
            mod.prepare(px4,out)
            tree=ET.parse(out/'worlds/delivery.sdf')
            self.assertEqual(tree.find('world').get('name'),'delivery')
            self.assertEqual(len(tree.findall('world/include')),2)
            pads={i.find('uri').text:i.find('pose').text.split()[:2]
                  for i in tree.findall('world/include')}
            self.assertEqual(pads['model://delivery_pad_0'],['3.0','0'])
            self.assertEqual(pads['model://delivery_pad_1'],['0.0','0'])
            config=yaml.safe_load((out/'sitl.yaml').read_text())
            self.assertEqual(config['payload']['ros__parameters']['backend'],'sim')
            dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
            for marker_id in (0,1):
                img=cv2.imread(str(out/'models'/('delivery_pad_'+str(marker_id))/f'marker_{marker_id}.png'))
                if hasattr(cv2.aruco,'ArucoDetector'):
                    _,ids,_=cv2.aruco.ArucoDetector(dictionary).detectMarkers(img)
                else:
                    _,ids,_=cv2.aruco.detectMarkers(img,dictionary)
                self.assertEqual(ids.flatten().tolist(),[marker_id])
            vehicle=ET.parse(out/'models/x500_delivery/model.sdf')
            camera=vehicle.find('model/link[@name="delivery_camera"]')
            self.assertEqual(camera.find('pose').get('relative_to'),'base_link')
            self.assertEqual(camera.find('sensor[@type="gpu_lidar"]/lidar/scan/horizontal/samples').text,'3')

            table_out=root/'table_assets'
            course=Path(__file__).parents[1]/'config/table_c.yaml'
            mod.prepare(px4,table_out,table_c=True,course_file=course)
            table=ET.parse(table_out/'worlds/delivery.sdf')
            includes={i.find('uri').text:i.find('pose').text.split()[:3]
                      for i in table.findall('world/include')}
            self.assertEqual(includes['model://delivery_pad_10'],['12.0','-1.5','0.005'])
            names={m.get('name') for m in table.findall('world/model')}
            self.assertIn('building_left_1',names)
            self.assertIn('gate_1_left',names)
            self.assertIn('tunnel_top',names)
            table_config=yaml.safe_load((table_out/'sitl.yaml').read_text())
            self.assertEqual(table_config['vision']['ros__parameters']['package_marker_ids'],[20,21,22])
            self.assertEqual(table_config['safety_monitor']['ros__parameters']['geofence_xy'],
                             [-2.,-3.,14.,-3.,14.,3.,-2.,3.])
