import unittest
import synch_node.synch_node.synch_block as synch_block

from std_msgs.msg import Int32
from portal_robot_interfaces.msg import IdClass, IdPosVel, IdPosTime

class TestSynch(unittest.TestCase):

    def test_classCallback(self):
        msg = IdClass()
        id = Int32()
        id.data = 1
        classification = Int32()
        classification.data = -1

        msg.id = id
        msg.classification = classification

        synch_block.class_callback(msg)

        self.assertEqual(len(synch_block.id_classes), 1)
        self.assertEqual(synch_block.id_classes[0], [1, -1])



if __name__ == '__main__':
    unittest.main()