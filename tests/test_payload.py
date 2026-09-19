import unittest
from delivery_ros.payload_protocol import crc16,frame,unframe,parse_state,SimBackend


class PayloadTests(unittest.TestCase):
    def test_crc_and_corruption(self):
        self.assertEqual(crc16(b'123456789'),0x29b1)
        self.assertEqual(unframe(frame('PING')),'PING')
        with self.assertRaises(ValueError): unframe(b'PING*0000')
        with self.assertRaises(ValueError): unframe(b'x'*161)

    def test_parse_and_validate(self):
        r=parse_state(frame('STATE 12 1 0 0 0 '+'a'*32))
        self.assertTrue(r.closed)
        self.assertFalse(r.present)
        for s in ('STATE 12 2 0 0 0 -','STATE 12 1 0 0 0 bad','STATE x 1 0 0 0 -'):
            with self.assertRaises(ValueError): parse_state(frame(s))

    def test_dedup_and_sensor_completion(self):
        p=SimBackend()
        p.release('a'*32,0)
        p.release('a'*32,.1)
        self.assertEqual(p.count,1)
        self.assertTrue(p.update(.6).busy)
        r=p.update(1.1)
        self.assertTrue(r.closed)
        self.assertFalse(r.present or r.busy)
        p.release('a'*32,2.)
        self.assertEqual(p.count,1)

    def test_stop_latches_fault(self):
        p=SimBackend(); p.release('a'*32,0); p.stop()
        self.assertTrue(p.update(2).fault)
        with self.assertRaises(ValueError): p.release('b'*32,3)

    def test_grab_then_release(self):
        p=SimBackend(present=False)
        p.grab('a'*32,0.)
        self.assertTrue(p.update(.6).present)
        self.assertTrue(p.update(1.1).closed)
        p.release('b'*32,2.)
        self.assertFalse(p.update(2.6).present)
        self.assertTrue(p.update(3.1).closed)

    def test_grab_interlocks_and_deduplicates(self):
        p=SimBackend(present=False)
        p.grab('a'*32,0.); p.grab('a'*32,.1)
        self.assertEqual(p.count,1)
        with self.assertRaises(ValueError): p.release('b'*32,.2)
