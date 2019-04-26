import unittest

from fct.acc.common import acc_global_defs as acc_gd
from fct.acc.acc_performance.acc_approach_analyzer441 import ApproachAnalyzer441


class ApproachAnalyzerTest(unittest.TestCase):
    def test_empty_slices(self):
        """ empty signal slices -> None of the test_criteria should match"""
        tunnel_state_of_scene = 0
        preselect_slice = []
        obstacle_detect_slice = []
        observed_class_slice = []

        ret = ApproachAnalyzer441.check_test_criteria(tunnel_state_of_scene, preselect_slice,
                                                      obstacle_detect_slice, observed_class_slice)
        self.assertEqual(ret, None)

    def test_slices_difflen(self):
        """ signal slices have different length -> should throw exception """
        tunnel_state_of_scene = 0
        preselect_slice = [acc_gd.DTR_OBJECT_PRESELECT_NOT_SELECTED] * 5
        obstacle_detect_slice = [acc_gd.DTR_OBJ_OBSTCLDETECT_NO_CLASS]*3
        observed_class_slice = [acc_gd.DTR_OBSERVED_CLASS_NO_CLASS]*3

        self.assertRaises(ApproachAnalyzer441.check_test_criteria(tunnel_state_of_scene, preselect_slice,
                                                                  obstacle_detect_slice, observed_class_slice))

    def test_first_ego(self):
        """ first ego is set over complete length of slice -> condition 'a' expected """
        tunnel_state_of_scene = 0
        preselect_slice = [acc_gd.DTR_OBJECT_PRESELECT_FIRST_EGO_LANE]*20
        obstacle_detect_slice = [0, 1, 2, 3, 4]*4
        observed_class_slice = [acc_gd.DTR_OBSERVED_CLASS_UNKNOWN]*20

        ret = ApproachAnalyzer441.check_test_criteria(tunnel_state_of_scene, preselect_slice,
                                                      obstacle_detect_slice, observed_class_slice)

        print ret
        self.assertEqual(ret, 'A')

    def test_first_ego_not(self):
        """ first ego is set over complete length of slice -> condition 'a' expected """
        tunnel_state_of_scene = 0
        preselect_slice = [acc_gd.DTR_OBJECT_PRESELECT_FIRST_EGO_LANE]*20
        preselect_slice[15] = acc_gd.DTR_OBJECT_PRESELECT_NOT_SELECTED  # dropout for one cycle
        obstacle_detect_slice = [0, 1, 2, 3, 4]*4
        observed_class_slice = [acc_gd.DTR_OBSERVED_CLASS_UNKNOWN]*20

        ret = ApproachAnalyzer441.check_test_criteria(tunnel_state_of_scene, preselect_slice,
                                                      obstacle_detect_slice, observed_class_slice)
        self.assertEqual(ret, None)

    def test_cond_b(self):
        """  LRR_Obj_ObstclDtct = Obstacle
           AND (NOT Observed_Class == Guardrail) AND (NOT Observed_Class == Curve_Entry)
           AND (NOT Observed_Class == Unknown) """
        tunnel_state_of_scene = 0
        preselect_slice = [acc_gd.DTR_OBJECT_PRESELECT_NOT_SELECTED]*20
        obstacle_detect_slice = [acc_gd.DTR_OBJ_OBSTCLDETECT_OBSTACLE]*20
        observed_class_slice = [0]*20

        ret = ApproachAnalyzer441.check_test_criteria(tunnel_state_of_scene, preselect_slice,
                                                      obstacle_detect_slice, observed_class_slice)
        self.assertEqual(ret, 'B')

    def test_cond_b_guardrail(self):
        """  LRR_Obj_ObstclDtct = Obstacle
           AND (NOT Observed_Class == Guardrail) AND (NOT Observed_Class == Curve_Entry)
           AND (NOT Observed_Class == Unknown) """
        tunnel_state_of_scene = 0
        preselect_slice = [acc_gd.DTR_OBJECT_PRESELECT_NOT_SELECTED]*20
        obstacle_detect_slice = [acc_gd.DTR_OBJ_OBSTCLDETECT_OBSTACLE]*20
        observed_class_slice = [0]*20
        observed_class_slice[5] = acc_gd.DTR_OBSERVED_CLASS_GUARDRAIL

        ret = ApproachAnalyzer441.check_test_criteria(tunnel_state_of_scene, preselect_slice,
                                                      obstacle_detect_slice, observed_class_slice)
        self.assertEqual(ret, None)

    def test_cond_b_curveentry(self):
        """  LRR_Obj_ObstclDtct = Obstacle
           AND (NOT Observed_Class == Guardrail) AND (NOT Observed_Class == Curve_Entry)
           AND (NOT Observed_Class == Unknown) """
        tunnel_state_of_scene = 0
        preselect_slice = [acc_gd.DTR_OBJECT_PRESELECT_NOT_SELECTED]*20
        obstacle_detect_slice = [acc_gd.DTR_OBJ_OBSTCLDETECT_OBSTACLE]*20
        observed_class_slice = [0]*20
        observed_class_slice[5] = acc_gd.DTR_OBSERVED_CLASS_CURVEENTRY

        ret = ApproachAnalyzer441.check_test_criteria(tunnel_state_of_scene, preselect_slice,
                                                      obstacle_detect_slice, observed_class_slice)
        self.assertEqual(ret, None)

    def test_cond_b_unknown(self):
        """  LRR_Obj_ObstclDtct = Obstacle
           AND (NOT Observed_Class == Guardrail) AND (NOT Observed_Class == Curve_Entry)
           AND (NOT Observed_Class == Unknown) """
        tunnel_state_of_scene = 0
        preselect_slice = [acc_gd.DTR_OBJECT_PRESELECT_NOT_SELECTED]*20
        obstacle_detect_slice = [acc_gd.DTR_OBJ_OBSTCLDETECT_OBSTACLE]*20
        observed_class_slice = [0]*20
        observed_class_slice[5] = acc_gd.DTR_OBSERVED_CLASS_UNKNOWN

        ret = ApproachAnalyzer441.check_test_criteria(tunnel_state_of_scene, preselect_slice,
                                                      obstacle_detect_slice, observed_class_slice)
        self.assertEqual(ret, None)

    def test_cond_c(self):
        """  LRR_Obj_ObstclDtct = Obstacle
           AND (NOT Observed_Class == Guardrail) AND (NOT Observed_Class == Curve_Entry)
           AND (NOT Observed_Class == Unknown) """
        tunnel_state_of_scene = 0
        preselect_slice = [acc_gd.DTR_OBJECT_PRESELECT_NOT_SELECTED]*20
        obstacle_detect_slice = [acc_gd.DTR_OBJ_OBSTCLDETECT_PROBABLY_OVERRIDABLE]*10
        obstacle_detect_slice.extend([acc_gd.DTR_OBJ_OBSTCLDETECT_PROBABLY_UNDERDRIVABLE]*10)
        observed_class_slice = [0]*20

        ret = ApproachAnalyzer441.check_test_criteria(tunnel_state_of_scene, preselect_slice,
                                                      obstacle_detect_slice, observed_class_slice)
        self.assertEqual(ret, 'C')

    def test_cond_d(self):
        """  (TunnelDetect = 2)
        LRR_Obj_ObjPreSelect = FIRST EGO ODER LRR_Obj_ObjPreSelect == Fusion_Obstacle """
        tunnel_state_of_scene = 2
        preselect_slice = [acc_gd.DTR_OBJECT_PRESELECT_FUSION_OBSTACLE]*10
        preselect_slice.extend([acc_gd.DTR_OBJECT_PRESELECT_FIRST_EGO_LANE]*10)
        obstacle_detect_slice = [acc_gd.DTR_OBJ_OBSTCLDETECT_PROBABLY_OVERRIDABLE]*10
        obstacle_detect_slice.extend([acc_gd.DTR_OBJ_OBSTCLDETECT_PROBABLY_UNDERDRIVABLE]*10)
        observed_class_slice = [0]*20

        ret = ApproachAnalyzer441.check_test_criteria(tunnel_state_of_scene, preselect_slice,
                                                      obstacle_detect_slice, observed_class_slice)
        self.assertEqual(ret, 'D')

    def test_cond_e(self):
        """  (TunnelDetect = 2)
           LRR_Obj_ObstclDtct == (Probably_Underdrivable ODER Probably_Overdrivable ODER Obstacle) UND
           ((NICHT Obeserved_Class == Guardrail) UND (NICHT Obeserved_Class == Curve_Entry) UND
           (NICHT Obeserved_Class == Unknown))
           """
        tunnel_state_of_scene = 2
        preselect_slice = [acc_gd.DTR_OBJECT_PRESELECT_NOT_SELECTED]*20
        obstacle_detect_slice = [acc_gd.DTR_OBJ_OBSTCLDETECT_OBSTACLE]*20
        observed_class_slice = [0]*20

        ret = ApproachAnalyzer441.check_test_criteria(tunnel_state_of_scene, preselect_slice,
                                                      obstacle_detect_slice, observed_class_slice)
        self.assertEqual(ret, 'E')
