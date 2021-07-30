import engine.state_manager as state_manager
import unittest


class StateTest(unittest.TestCase):
    #test the setting and getting of different mission states
    def test_state(self): 
        state_obj = state_manager.State()
        state_lst = ["startup", "path_traversal", "astar", "docking", "docked", "manual", "safehold"]

        for state in state_lst: 
            state_obj.set_state(state)
            test_state = state_obj.get_state()
            self.assertEqual(state, test_state)


class StateManagerTest(unittest.TestCase): 
    def test_startup(self): 
        state_obj = state_manager.State()
        state_obj.set_state("startup")
        state_manager_obj = state_manager.StateManager()
        # state_obj.set_state("startup")
        # print(state_obj.get_state())
        # state_manager_obj.
        # test_val = state_manager_obj.execute()
        # print(test_val)
        # self.assertEqual("executed", test_val)
    # def test_path_traversal(self): 
    #     pass 
    # def test_astar(self): 
    #     pass 
    # def test_docking(self): 
    #     pass 
    # def test_docked(self): 
    #     pass 
    # def test_manual(self): 
    #     pass 
    # def test_safehold(self): 
    #     pass 


if __name__ == '__main__':
    unittest.main()




