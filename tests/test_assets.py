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
            config=yaml.safe_load((out/'sitl.yaml').read_text())
            self.assertEqual(config['payload']['ros__parameters']['backend'],'sim')
            dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
            for marker_id in (0,1):
                img=cv2.imread(str(out/'models'/('delivery_pad_'+str(marker_id))/'marker.png'))
                if hasattr(cv2.aruco,'ArucoDetector'):
                    _,ids,_=cv2.aruco.ArucoDetector(dictionary).detectMarkers(img)
                else:
                    _,ids,_=cv2.aruco.detectMarkers(img,dictionary)
                self.assertEqual(ids.flatten().tolist(),[marker_id])
            ET.parse(out/'models/x500_delivery/model.sdf')
