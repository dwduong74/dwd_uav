import tempfile
import unittest
from pathlib import Path
from delivery_ros.course import Course,RoutePlanner,point_in_polygon
from delivery_ros.competition_core import CompetitionEngine,CompetitionPhase


COURSE=Path(__file__).parents[1]/'config'/'table_c.yaml'


class CourseTests(unittest.TestCase):
    def test_course_and_alternate_route(self):
        course=Course.load(COURSE,'table_c_demo')
        self.assertEqual(course.packages[0].position,(0.,-1.,0.))
        self.assertAlmostEqual(course.destinations['alpha'].position[1],-1.5)
        planner=RoutePlanner(course.nodes,course.edges)
        path=planner.path('home','delivery_bravo')
        self.assertEqual((path[0],path[-1]),('home','delivery_bravo'))
        self.assertTrue(point_in_polygon((0,0),course.geofence))
        self.assertFalse(point_in_polygon((20,0),course.geofence))
        with self.assertRaises(ValueError):
            planner.path('home','delivery_bravo',{tuple(sorted(('gate_2','tunnel_in')))})

    def test_invalid_duplicate_marker(self):
        text=COURSE.read_text().replace('marker_id: 20','marker_id: 1')
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'bad.yaml'; path.write_text(text)
            with self.assertRaises(ValueError): Course.load(path,'table_c_demo')


class CompetitionTests(unittest.TestCase):
    def test_three_packages_in_order(self):
        course=Course.load(COURSE,'table_c_demo'); engine=CompetitionEngine(course)
        self.assertTrue(engine.start(0.,3))
        for index,package in enumerate(course.packages):
            self.assertEqual(engine.phase,CompetitionPhase.SCAN_PACKAGE)
            self.assertFalse(engine.marker(99,index+1.))
            for n in range(5):
                accepted=engine.marker(package.marker_id,index+1.+n*.05)
            self.assertTrue(accepted)
            self.assertEqual(engine.phase,CompetitionPhase.GRAB_PACKAGE)
            self.assertTrue(engine.grab_result(True))
            self.assertEqual(engine.phase,CompetitionPhase.DELIVER)
            self.assertTrue(engine.leg_result(True,index+2.))
        self.assertEqual(engine.phase,CompetitionPhase.FINISHED)
        self.assertEqual(engine.completed,3)
        self.assertEqual(engine.progress,1.)

    def test_fail_closed(self):
        engine=CompetitionEngine(Course.load(COURSE,'table_c_demo'))
        engine.start(0.,3)
        for n in range(5): engine.marker(20,n*.05)
        engine.grab_result(False,'switch did not close')
        self.assertEqual(engine.phase,CompetitionPhase.FAILED)
        self.assertIn('switch',engine.detail)
