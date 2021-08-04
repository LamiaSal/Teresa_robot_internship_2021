import unittest
from src.utils.training_tools import pos_to_state, state_to_pos, FINAL_X, FINAL_Y, ERROR, in_range

class FormulaTests(unittest.TestCase):

    def test_state_to_pos_function(self,state):
       	print(state_to_pos(state))

    def test_pos_to_state_function(self, x,y):
        print(pos_to_state(x,y))

    def test_in_range_function_success(self):
        x, y = FINAL_X - 1, FINAL_Y + 1
        self.assertTrue(in_range(x, FINAL_X, ERROR))
        self.assertTrue(in_range(y, FINAL_Y, ERROR))

    def test_in_range_function_failed(self):
        x, y = FINAL_X - 20, FINAL_Y - 20
        self.assertFalse(in_range(x, FINAL_X, ERROR))
        self.assertFalse(in_range(y, FINAL_Y, ERROR))

if __name__ == '__main__':
    unittest.main()
