"""
acc_approach_analyzer441.py
-------------------

extends statistcs for base approach testcase bci


:author:        Sahajhaksh Hariharan

"""
# ====================================================================
# Imports
# ====================================================================
import os

# ====================================================================
# Imports - Local
# ====================================================================

import stk.valf.signal_defs as sd

from stk.valf import BaseComponentInterface as bci

import fct.acc.common.acc_global_defs as acc_gd
from fct.acc.common.observer_dispatcher import ObserverDispatcher
from fct.acc.common.acc_meas_statistics_extractor import find_nearest

# ====================================================================
# Global Constant Declarations
# ====================================================================
GENERATOR_VERSION_STRING = "$Revision: 1.4 $"
GENERATOR_MODULE_REPORT = os.path.basename("%s" % __file__)

SPEED_THRESHOLD = 70.0  # [km/h]
TIMEGAP_THRESHOLD = 4.0  # [s]
DIST_EXT = 20.0  # [m]
DIST_THRESHOLD = 90.0  # [m]

#############################################################################


# =============================================================================
# Class
# =============================================================================
class ApproachAnalyzer441(bci):
    def __init__(self, data_manager, component_name, bus_name, version=GENERATOR_VERSION_STRING):
        """ Class initialisation.
        @Param data_manager:   Storage class for data.
        @Param component_name:   The name of the component.
        @Param bus_name:   The bus name. Just one bus for approach performance analysis.
        """
        super(ApproachAnalyzer441, self).__init__(data_manager, component_name, bus_name, version)
        self._logger.debug()

        self.__algo_version = None
        self.__timestamp = []
        self.__acc_event_list = []

        self.__DataBaseObjectsConnections = None
        self.__RecCatDB = None
        self.__ObjDataDB = None
        self.__GenLblDB = None
        self.__GblDB = None
        self.__ValResDB = None
        
        # init configuration ports from config-file
        self.__Read_In_From_DB = False
        self.__project_name = None

        self.__OutputDir = None
        self.__lstDeveloperDetails = []
        self.observer_dispatcher = None

    def Initialize(self):
        """ Initialize. Called once. """
        self._logger.debug()
        project_name = self._data_manager.GetDataPort(acc_gd.PROJECT_PORT_NAME)
        self.observer_dispatcher = ObserverDispatcher(project_name, self._logger)
        return sd.RET_VAL_OK

    def PostInitialize(self):
        """ PostInitialize. Called once. """
        self._logger.debug()
        return sd.RET_VAL_OK

    def LoadData(self):
        """ LoadData. Called for each file. """
        self._logger.debug()
        self.__timestamp = self._data_manager.GetDataPort(sd.TIMESTAMP_PORT_NAME, self._bus_name)
        return sd.RET_VAL_OK

    def ProcessData(self):
        """ ProcessData. Called for each file. """
        self._logger.debug()
        current_rec_file = self._data_manager.GetDataPort(sd.CURRENT_FILE_PORT_NAME)
        if not self.observer_dispatcher.is_configured_to_run(self._component_name, current_rec_file):
            return sd.RET_VAL_OK

        self.__acc_event_list = self._data_manager.GetDataPort(sd.ACC_EVENTS_PORT_NAME, self._bus_name)
        self._logger.info("number of ACC events: %d" % len(self.__acc_event_list))
        for ev in self.__acc_event_list:
            if ev.GetTestcaseErrorType() != acc_gd.TESTCASE_ERROR_TYPES.NONE:
                ev.AddAttribute('event_applicable', False, '', 'boolean')
                ev.AddAttribute('tunnel_state_of_scene', None, '', 'int')
                ev.AddAttribute('stat_approach_condition', None, '', 'string')
                continue
            if ev.GetType() == acc_gd.EVENT_TYPE_APPROACH_TESTCASE:
                self.analyze_event(ev)
                typename = acc_gd.EVENT_TYPE_STAT_APPROACH_TESTCASE
                ev.SetType(typename)

        self.__acc_event_list = []
        return sd.RET_VAL_OK

    def PostProcessData(self):
        """ PostProcessData. Called for each file. """
        self._logger.debug()
        return sd.RET_VAL_OK

    def PreTerminate(self):
        """ PreTerminate. Called once. """
        self._logger.debug()
        return sd.RET_VAL_OK

    def Terminate(self):
        """ Terminate. Called once. """
        self._logger.debug()
        return sd.RET_VAL_OK

    def analyze_event(self, event):
        """
        get event object
        get timestamp of Timegap4s+20m
          if this is before object lifetime:  event attribute states -> failed   (was not stable over the time)
        from this timestamp till end of TC
         create slices of the respective signals:
          -PreSelect
          -ObstclDtct
          -ObservedClass
        check TunnelDetect at timestamp of Timegap4s+20m
        check for each slice according to criteria if state given for whole slice
        """
        self._logger.info("length of bsig timestamp: %s" % str(len(self.__timestamp)))

        tunnel_detect = self._data_manager.GetDataPort("TunnelDtct", self._bus_name)
        vego = event.GetEgoKinematics().GetSpeed()

        obj = event.GetEventObject().get_object()
        self._logger.info("object lifetime: %s" % str(len(obj["Timestamp"])))

        # get timestamp of when Timegap=4s+20m ('extended PUD')
        timestamp_of_ext_pud = self.get_timestamp_of_pud(event, timegap_threshold=TIMEGAP_THRESHOLD,
                                                         dist_delta=DIST_EXT)
        # get timestamp of when Timegap=4s ('PUD')
        timestamp_of_pud = self.get_timestamp_of_pud(event, timegap_threshold=TIMEGAP_THRESHOLD)
        self._logger.info("timestamp when distance gets below pud: %s" % str(timestamp_of_ext_pud))
        # get timestamp when object comes closer than 90m
        timestamp_of_dist = self.get_timestamp_of_dist(event, dist_threshold=DIST_THRESHOLD)
        self._logger.info("timestamp when object gets below 90m: %s" % str(timestamp_of_dist))

        if timestamp_of_ext_pud == obj["Timestamp"][0]:
            # TODO: this scene not applicable according to my interpretation of the requirement
            # TODO: has to be regarded as attribute, so that it either can be counted as faild or not counted at all
            self._logger.warning("object only comes to life under the threshold")
            event_applicable = False
        else:
            event_applicable = True

        abs_idx = self.__timestamp.index(timestamp_of_ext_pud)
        tunnel_state_of_scene = tunnel_detect[abs_idx]
        self._logger.info("tunnel state at begin of scene: %s" % str(tunnel_state_of_scene))

        # distinguish slower and high speed approaches (e.g. 0-70 and 70-120km/h)
        if vego[0] < SPEED_THRESHOLD/3.6:
            timestamp_of_scene_begin = timestamp_of_pud
        else:
            # take maximum of timestamps (equivalent to minimum of distances during an approach)
            timestamp_of_scene_begin = max(timestamp_of_dist, timestamp_of_pud)

        # from this timestamp till end of TC
        #  create slices of the respective signals:
        #   -PreSelect
        #   -ObstclDtct
        #   -ObservedClass
        obj_idx_pud = self.get_index_relative_to_object_for_ts(obj, timestamp_of_scene_begin, self.__timestamp)
        tc_end = event.GetStopTime()  # TODO clarify if good enough since after TC detector this might be not TC endtime
        obj_idx_tc_end = self.get_index_relative_to_object_for_ts(obj, tc_end, self.__timestamp)

        observed_class_slice = obj["Observed_Class"][obj_idx_pud:obj_idx_tc_end + 1]
        obstacle_detect_slice = obj["DTR_Obj_ObstclDtct"][obj_idx_pud:obj_idx_tc_end + 1]
        preselect_slice = obj["DTR_ObjPreSelect"][obj_idx_pud:obj_idx_tc_end + 1]

        test_result = self.check_test_criteria(tunnel_state_of_scene, preselect_slice, obstacle_detect_slice,
                                               observed_class_slice)
        self._logger.info("Test Result: condition %s met." % str(test_result))

        # adding obstcldtct/observed class to plot
        self.add_objsignal_to_plot_data(event, "ObstclDtct", obj["DTR_Obj_ObstclDtct"], gain=10)
        self.add_objsignal_to_plot_data(event, "ObsClass", obj["Observed_Class"], gain=10)
        # TODO: ensure that signals will be aligned to other event signals (even if object starts before the event)
        # obstcldtct = [10*v for v in obj["DTR_Obj_ObstclDtct"]]
        # obsrvdcls = [10*v for v in obj["Observed_Class"]]

        # plotdata = event.GetPlotData()
        # plotdata["SignalValueList"].append(obstcldtct)
        # plotdata["SignalsNameList"].append("ObstclDtct")
        # plotdata["SignalValueList"].append(obsrvdcls)
        # plotdata["SignalsNameList"].append("ObsClass")

        event.AddAttribute('event_applicable', event_applicable, '', 'boolean')
        event.AddAttribute('tunnel_state_of_scene', tunnel_state_of_scene, '', 'int')
        event.AddAttribute('stat_approach_condition', test_result, '', 'string')

    @staticmethod
    # TODO put it as method into PlotData class
    def add_objsignal_to_plot_data(event, name, signalvalue, gain=1):
        signalvalue = [gain * v for v in signalvalue]
        obj = event.GetEventObject().get_object()
        plotdata = event.GetPlotData()

        PLOT_START_CYCLE = 30  # TODO put it as member into PlotData class
        PLOT_STOP_CYCLE = 30
        obj_start_index = obj["Index"]
        obj_end_index = obj_start_index + len(obj[sd.OBJ_DISTX])
        ind_start = max(0, max((event.GetStartIndex() - PLOT_START_CYCLE), obj_start_index))
        ind_stop = min(obj_end_index, event.GetStopIndex() + PLOT_STOP_CYCLE)
        # Calculate the object relative indexes.
        obj_rel_start = ind_start - obj["Index"]
        obj_rel_end = ind_stop - obj["Index"]
        signalvalue = signalvalue[obj_rel_start:obj_rel_end]

        plotdata["SignalValueList"].append(signalvalue)
        plotdata["SignalsNameList"].append(name)

    @staticmethod
    def check_test_criteria(tunnel, preselect, obstacle_detect, observed_class):

        if len(preselect) != len(obstacle_detect) != len(observed_class):
            raise StandardError
        if not len(preselect):
            return None

        a = b = c = d = e = False
        # A)  TunnelDetect != 2  and LRR_Obj_ObjPreSelect = FIRST EGO for whole scene
        if tunnel != 2:
            a = True
            for el in preselect:
                if el != acc_gd.DTR_OBJECT_PRESELECT_FIRST_EGO_LANE:
                    a = False
                    break
            if a:
                return 'A'
        # B)
            b = True
            for i, _ in enumerate(obstacle_detect):
                if obstacle_detect[i] != acc_gd.DTR_OBJ_OBSTCLDETECT_OBSTACLE or\
                        (observed_class[i] == acc_gd.DTR_OBSERVED_CLASS_GUARDRAIL or
                         observed_class[i] == acc_gd.DTR_OBSERVED_CLASS_CURVEENTRY or
                         observed_class[i] == acc_gd.DTR_OBSERVED_CLASS_UNKNOWN):
                    b = False
                    break
            if b:
                return 'B'
        # C)
            c = True
            for i, _ in enumerate(obstacle_detect):
                if obstacle_detect[i] != acc_gd.DTR_OBJ_OBSTCLDETECT_PROBABLY_UNDERDRIVABLE and\
                    obstacle_detect[i] != acc_gd.DTR_OBJ_OBSTCLDETECT_PROBABLY_OVERRIDABLE or\
                        (observed_class[i] == acc_gd.DTR_OBSERVED_CLASS_GUARDRAIL or
                         observed_class[i] == acc_gd.DTR_OBSERVED_CLASS_CURVEENTRY or
                         observed_class[i] == acc_gd.DTR_OBSERVED_CLASS_UNKNOWN):
                    c = False
                    break
            if c:
                return 'C'
        else:
            # D) TunnelDetect == 2
            d = True
            for el in preselect:
                if el != acc_gd.DTR_OBJECT_PRESELECT_FIRST_EGO_LANE and\
                        el != acc_gd.DTR_OBJECT_PRESELECT_FUSION_OBSTACLE:
                    d = False
                    break
            if d:
                return 'D'

            # E) TunnelDetect == 2
            e = True
            for i, _ in enumerate(obstacle_detect):
                if obstacle_detect[i] != acc_gd.DTR_OBJ_OBSTCLDETECT_PROBABLY_UNDERDRIVABLE and\
                    obstacle_detect[i] != acc_gd.DTR_OBJ_OBSTCLDETECT_PROBABLY_OVERRIDABLE and\
                    obstacle_detect[i] != acc_gd.DTR_OBJ_OBSTCLDETECT_OBSTACLE or\
                        (observed_class[i] == acc_gd.DTR_OBSERVED_CLASS_GUARDRAIL or
                         observed_class[i] == acc_gd.DTR_OBSERVED_CLASS_CURVEENTRY or
                         observed_class[i] == acc_gd.DTR_OBSERVED_CLASS_UNKNOWN):
                    e = False
                    break
            if e:
                return 'E'
        return None

    def get_timestamp_of_pud(self, ev, timegap_threshold, dist_delta=0.0):
        """ return the timestamp, when object is closer than a given timegap_threshold
        :param ev: event
        :param timegap_threshold: given timegap in [s] that the event object has to be closer than
        :param dist_delta: distance in [m] that the timegap gets extended by """
        obj_rel_start, obj_rel_end, length = ev.GetRelativeObjectIndexes()
        vego = ev.GetEgoKinematics().GetSpeed()
        distx = ev.GetEventObject().get_object()[sd.OBJ_DISTX][obj_rel_start:obj_rel_end]

        timegap_over_event = []
        for i, v in enumerate(vego):
            timegap_over_event.append(self.calc_timegap(distx[i], v))

        dist_via_tg_over_event = []
        for v in vego:
            cur_dist_in_m = timegap_threshold * v + dist_delta
            dist_via_tg_over_event.append(cur_dist_in_m)
        i = 0
        for i in range(length):
            if distx[i] < dist_via_tg_over_event[i]:
                break
        index_when_timegap_reached = ev.GetStartIndex() + i
        return self.__timestamp[index_when_timegap_reached]

    def get_timestamp_of_dist(self, ev, dist_threshold):
        """ return the timestamp, when object is closer than given distance threshold
        :param ev: event
        :param dist_threshold: given timegap in [s] that the event object has to be closer than """
        obj_rel_start, obj_rel_end, length = ev.GetRelativeObjectIndexes()
        distx = ev.GetEventObject().get_object()[sd.OBJ_DISTX][obj_rel_start:obj_rel_end]
        i = 0
        for i, v in enumerate(distx):
            if v < dist_threshold:
                break
        index_when_dist_reached = ev.GetStartIndex() + i
        return self.__timestamp[index_when_dist_reached]

    @staticmethod
    def calc_timegap(distx, speed):
        """ calculate the timegap for given distance and ego speed
          returns 10000 if egospeed lower than 1 m/s
        :param distx: distance in [m]
        :param speed: ego speed in [m/s]
        :return: calculated timegap value
        """
        if speed > 1.0:
            timegap = distx / speed
        else:
            timegap = 10000
        return timegap

    @staticmethod
    def get_index_relative_to_object_for_ts(obj, timestamp, timestamp_list):
        """ for given absolute timestamp, return an index relative to the obj signals
        (start timestamp of object will return 0)
        :param obj: obj dictionary
        :param timestamp: single timestamp to look up
        :param timestamp_list: list of all timestamps of the recording/bsig
        :return index
        """

        nearest_timestamp = find_nearest(timestamp_list, timestamp)
        print "get_index_relative_to_object_for_ts: ", timestamp, nearest_timestamp, abs(timestamp - nearest_timestamp)
        idx = timestamp_list.index(nearest_timestamp)
        obj_startidx = obj["Index"]
        return idx - obj_startidx


