r"""
    Unit tests for heizung.

    > set PYTHONPATH=D:\Users\micha\Documents\git_projects\heizungsregelung
    > pytest -v tests\test_heizung.py

"""

import unittest

import heizung

class DefalutValueNode(heizung.SignalProcessor):

    def __init__(self,sName,value):
        super(DefalutValueNode,self).__init__(sName)
        self.value = value

    def get_value(self):
        return self.value

class TestHeizung(unittest.TestCase):

    DUMMY = 'Dummy'
    INPUT_NODE = 'DummyInput'

    DUMMY_INPUT = 'dummy_input'

    def test_SignalProcessor_constructor(self):
        obj = heizung.SignalProcessor(self.DUMMY)
        self.assertIsNotNone(obj)
        self.assertEqual(self.DUMMY, obj.get_name())

    def test_SignalProcessor_activated(self):
        obj = heizung.SignalProcessor(self.DUMMY)
        self.assertTrue(obj.is_activated())
        obj.set_activated(False)
        self.assertFalse(obj.is_activated())
        obj.set_activated(True)
        self.assertTrue(obj.is_activated())

    def test_SignalProcessor_inputs(self):
        obj = heizung.SignalProcessor(self.DUMMY)
        objInput = heizung.SignalProcessor(self.INPUT_NODE)
        self.assertFalse(obj.has_input())
        obj.connect_input(objInput, self.DUMMY_INPUT)
        self.assertTrue(obj.has_input())
        self.assertEqual(objInput, obj.get_input(self.DUMMY_INPUT))

    def test_SignalProcessor_push_value(self):
        obj = heizung.SignalProcessor(self.DUMMY)
        obj._push_value(7)
        obj._push_value(14)
        obj._push_value(3)
        self.assertEqual(3, obj.get_value())
        self.assertEqual(14, obj.get_value(1))
        self.assertEqual(7, obj.get_value(2))
        self.assertEqual(3, obj.get_value(0))
        self.assertEqual(None, obj.get_value(4))
        self.assertEqual(None, obj.get_value(42))
        self.assertEqual(14, obj.get_value(-1))
        self.assertEqual(7, obj.get_value(-2))
        self.assertEqual(None, obj.get_value(-42))
        self.assertEqual(3, obj.clock_tick())

    def test_NotNode_clock_tick(self):
        VAL_ON = 1
        VAL_OFF = 0
        valueObj = DefalutValueNode("ValueNode", VAL_ON)
        obj = heizung.NotNode(self.DUMMY, valueObj)
        self.assertIsNotNone(obj)
        self.assertEqual(not VAL_ON, obj.clock_tick())
        valueOffObj = DefalutValueNode("ValueNode", VAL_OFF)
        obj = heizung.NotNode(self.DUMMY, valueOffObj)
        self.assertIsNotNone(obj)
        self.assertEqual(not VAL_OFF, obj.clock_tick())

    def test_OrNode_clock_tick(self):
        VAL_ON = 1
        VAL_OFF = 0
        valueOnObj = DefalutValueNode("ValueNode", VAL_ON)
        valueOffObj = DefalutValueNode("ValueNode", VAL_OFF)
        obj = heizung.OrNode(self.DUMMY, [valueOnObj, valueOffObj])
        self.assertIsNotNone(obj)
        self.assertEqual(VAL_ON, obj.clock_tick())
        obj2 = heizung.OrNode(self.DUMMY, [valueOffObj, valueOffObj])
        self.assertIsNotNone(obj2)
        self.assertEqual(VAL_OFF, obj2.clock_tick())
        obj3 = heizung.OrNode(self.DUMMY, [valueOnObj, valueOnObj])
        self.assertIsNotNone(obj3)
        self.assertEqual(VAL_ON, obj3.clock_tick())

    def test_AndNode_clock_tick(self):
        VAL_ON = 1
        VAL_OFF = 0
        valueOnObj = DefalutValueNode("ValueNode", VAL_ON)
        valueOffObj = DefalutValueNode("ValueNode", VAL_OFF)
        obj = heizung.AndNode(self.DUMMY, [valueOnObj, valueOffObj])
        self.assertIsNotNone(obj)
        self.assertEqual(VAL_OFF, obj.clock_tick())
        obj2 = heizung.AndNode(self.DUMMY, [valueOffObj, valueOffObj])
        self.assertIsNotNone(obj2)
        self.assertEqual(VAL_OFF, obj2.clock_tick())
        obj3 = heizung.AndNode(self.DUMMY, [valueOnObj, valueOnObj])
        self.assertIsNotNone(obj3)
        self.assertEqual(VAL_ON, obj3.clock_tick())

if __name__ == '__main__':
    unittest.main()
