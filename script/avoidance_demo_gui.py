#!/usr/bin/env python
from __future__ import print_function
import rospy, rostopic, roslaunch, rospkg, tf
import std_msgs.msg as rosmsg
import nav_msgs.msg as navmsg
import geometry_msgs.msg as geomsg
import sensor_msgs.msg as senmsg
import visualization_msgs.msg as vismsg
try:
    import novatel_gps_msgs.msg as novamsg
except:
    None
import autoware_msgs.msg as autowmsgs
import numpy as np
import pyqtgraph as pg
import pyqtgraph.Qt as qtgqt
import pyqtgraph.dockarea as darea
import os.path
from jsk_rviz_plugins.msg import OverlayText

class PlotHandler(object):
    def __init__(self, leaf):
        super(PlotHandler, self).__init__()
        pg.setConfigOptions(antialias=True)
        self.leaf = leaf
        self.app = qtgqt.QtGui.QApplication([])
        self.rospack = rospkg.RosPack()

    def initializePlot(self):
        self.first_run = True
        self.win = qtgqt.QtGui.QMainWindow()
        area = darea.DockArea()
        white = (200, 200, 200)
        red = (200, 66, 66); redB = pg.mkBrush(200, 66, 66, 200)
        blue = (6, 106, 166); blueB = pg.mkBrush(6, 106, 166, 200)
        green = (16, 200, 166); greenB = pg.mkBrush(16, 200, 166, 200)
        yellow = (244, 244, 160); yellowB = pg.mkBrush(244, 244, 160, 200)
        darkyellow = (224, 166, 58); darkyellowB = pg.mkBrush(224, 166, 58, 200);
        self.win.setWindowTitle("Elkerules Demo")
        self.win.setWindowIcon(qtgqt.QtGui.QIcon(self.rospack.get_path("ros_guis") + "/img/icon01.png"))
        self.win.resize(700, 600)
        self.win.setCentralWidget(area)
        dock1def = darea.Dock("Default", size = (1,1))  # give this dock minimum possible size
        dock2oth = darea.Dock("Control", size = (1,1))  # give this dock minimum possible size
        dock3ctr = darea.Dock("Control", size = (1,1))  # give this dock minimum possible size
        dock4gps = darea.Dock("2 Gps visualization", size = (500,400)) # size is only a suggestion
        area.addDock(dock1def, "left")
        area.addDock(dock2oth, "bottom", dock1def)
        #area.addDock(dock3ctr, "above", dock2oth)
        #area.addDock(dock4gps, "bottom", dock3ctr)
        dhLabel = qtgqt.QtGui.QLabel("Duro:"); dhLabel.setStyleSheet("background-color: rgb(4, 4, 4);"); dhLabel.setAlignment(pg.QtCore.Qt.AlignRight); dhLabel.setFixedSize(50, 25)
        dsLabel = qtgqt.QtGui.QLabel("Duro:"); dsLabel.setStyleSheet("background-color: rgb(4, 4, 4);"); dsLabel.setAlignment(pg.QtCore.Qt.AlignRight); dsLabel.setFixedSize(50, 25)
        nhLabel = qtgqt.QtGui.QLabel("Nova:"); nhLabel.setStyleSheet("background-color: rgb(4, 4, 4);"); nhLabel.setAlignment(pg.QtCore.Qt.AlignRight)
        nsLabel = qtgqt.QtGui.QLabel("Nova:"); nsLabel.setStyleSheet("background-color: rgb(4, 4, 4);"); nsLabel.setAlignment(pg.QtCore.Qt.AlignRight)     
        self.duroRtkLabel = qtgqt.QtGui.QLabel("+++")
        self.novaRtkLabel = qtgqt.QtGui.QLabel("+++")
        self.pauseSensorReadClickedBtn = qtgqt.QtGui.QPushButton("Pause")
        self.duroSensorLaunchBtn = qtgqt.QtGui.QPushButton("Start Duro GPS")
        self.novaSensorLaunchBtn = qtgqt.QtGui.QPushButton("Start Nova GPS")

        self.SensorLaunchBtn = qtgqt.QtGui.QPushButton("Start Sensors")
        self.avoidanceLaunchBtn = qtgqt.QtGui.QPushButton("Start Avoidance")

        
        self.ctrl_cmd_speedLabel = qtgqt.QtGui.QLabel(" **.* Km/h")
        self.isAutonomLabel = qtgqt.QtGui.QLabel("-")
        self.ctrl_cmd_steering_angle = qtgqt.QtGui.QLabel(" **.* rad")
        

        self.SensorsLaunched = False
        self.avoidanceLaunched = False

        
        widg1def = pg.LayoutWidget()
        widg1def.setStyleSheet("background-color: rgb(40, 44, 52); color: rgb(171, 178, 191);")
        dock1def.setStyleSheet("background-color: rgb(18, 20, 23);")
        dock1def.addWidget(widg1def)
        self.novaRtkLabel.setStyleSheet("font-family: Monospace; font: 20pt; color: rgb(6, 106, 166)")
        self.duroRtkLabel.setStyleSheet("font-family: Monospace; font: 20pt; color: rgb(244, 166, 58)")
        self.ctrl_cmd_steering_angle.setStyleSheet("font-family: Monospace; font: 20pt; color: rgb(200, 200, 200)")
        self.ctrl_cmd_speedLabel.setStyleSheet("font-family: Monospace; font: 20pt; color: rgb(200, 200, 200)")
        # self.isAutonomLabel.setStyleSheet("font-family: Monospace; font: 20pt; color: rgb(200, 200, 200)")
        #self.csvLabel.setStyleSheet("font: 10pt; color: rgb(244, 166, 58)")
        ctlCmdLabel = qtgqt.QtGui.QLabel("Ctrl_cmd:"); ctlCmdLabel.setStyleSheet("background-color: rgb(4, 4, 4);"); ctlCmdLabel.setAlignment(pg.QtCore.Qt.AlignRight); ctlCmdLabel.setFixedSize(80, 25)
        currPoseLabel = qtgqt.QtGui.QLabel("Current_pose:"); currPoseLabel.setStyleSheet("background-color: rgb(4, 4, 4);"); currPoseLabel.setAlignment(pg.QtCore.Qt.AlignRight); currPoseLabel.setFixedSize(100, 25)
        ousterLabel = qtgqt.QtGui.QLabel("OusterRight:"); ousterLabel.setStyleSheet("background-color: rgb(4, 4, 4);"); ousterLabel.setAlignment(pg.QtCore.Qt.AlignRight); ousterLabel.setFixedSize(95, 25)
        eucLabel = qtgqt.QtGui.QLabel("Euclidean Clustering:"); eucLabel.setStyleSheet("background-color: rgb(4, 4, 4);"); eucLabel.setAlignment(pg.QtCore.Qt.AlignRight)
        wayLabel = qtgqt.QtGui.QLabel("Base Waypoints:"); wayLabel.setStyleSheet("background-color: rgb(4, 4, 4);"); wayLabel.setAlignment(pg.QtCore.Qt.AlignRight)
        self.ctrl_cmdOkLabel = qtgqt.QtGui.QLabel("**")
        self.ctrl_cmdOkLabel.setStyleSheet("font-family: Monospace; font: 20pt; color: rgb(224, 166, 58)")
        self.ousterRigLabel = qtgqt.QtGui.QLabel(" **")
        self.ousterRigLabel.setStyleSheet("font-family: Monospace; font: 20pt; color: rgb(16, 200, 166)")
        self.currentPoseLabel = qtgqt.QtGui.QLabel(" **")
        self.currentPoseLabel.setStyleSheet("font-family: Monospace; font: 20pt; color: rgb(16, 200, 166)")
        self.eukDet = qtgqt.QtGui.QLabel(" **")
        self.eukDet.setStyleSheet("font-family: Monospace; font: 20pt; color: rgb(200, 66, 66)")
        self.baseWay = qtgqt.QtGui.QLabel(" **")
        self.baseWay.setStyleSheet("font-family: Monospace; font: 20pt; color: rgb(200, 66, 66)")      
        # default
        widg1def.addWidget(dsLabel, row=2, col=3)
        widg1def.addWidget(nsLabel, row=2, col=1)
        widg1def.addWidget(self.duroRtkLabel, row=2, col=4)
        widg1def.addWidget(self.novaRtkLabel, row=2, col=2)

        widg1def.addWidget(self.ctrl_cmd_speedLabel, row=4, col=3)
        # widg1def.addWidget(self.speedLabel, row=3, col=2)
        widg1def.addWidget(self.ctrl_cmd_steering_angle, row=4, col=4)
        # widg1def.addWidget(self.isAutonomLabel, row=4, col=4)

        widg1def.addWidget(self.pauseSensorReadClickedBtn, row=4, col=7)
        widg1def.addWidget(currPoseLabel, row=2, col=5)
        widg1def.addWidget(ousterLabel, row=1, col=1)
        widg1def.addWidget(eucLabel, row=3, col=1)
        widg1def.addWidget(wayLabel, row=3, col=3)
        widg1def.addWidget(ctlCmdLabel, row=4, col=1)
        widg1def.addWidget(self.ctrl_cmdOkLabel, row=4, col=2)
        widg1def.addWidget(self.ousterRigLabel, row=1, col=2)
        widg1def.addWidget(self.currentPoseLabel, row=2, col=6)
        widg1def.addWidget(self.eukDet, row=3, col=2)
        widg1def.addWidget(self.baseWay, row=3, col=4)
        # self.zedOkLabel = qtgqt.QtGui.QLabel("**")
        # self.zedOkLabel.setStyleSheet("font-family: Monospace; font: 20pt; color: rgb(123, 64, 133)")
        #zhLabel = qtgqt.QtGui.QLabel("Zed:"); zhLabel.setStyleSheet("background-color: rgb(4, 4, 4);"); zhLabel.setAlignment(pg.QtCore.Qt.AlignRight); zhLabel.setFixedSize(50, 25)
        # widg1def.addWidget(zhLabel, row=3, col=6)
        # widg1def.addWidget(self.zedOkLabel, row=3, col=7)

        self.state = None
        self.widgGps = pg.PlotWidget(title="Gps difference")
        self.widgGps.setAspectLocked(True)
        self.pltGpsOdom = pg.ScatterPlotItem(size = 10, pen = pg.mkPen(None), brush = blueB)
        self.pltDuroOrientation = pg.PlotCurveItem(pen=pg.mkPen(qtgqt.QtGui.QColor(244, 166, 58), width=6))
        self.pltNovaOrientation = pg.PlotCurveItem(pen=pg.mkPen(qtgqt.QtGui.QColor(200, 66, 66), width=8))
        self.pltLeafOdom = pg.ScatterPlotItem(size = 10, pen = pg.mkPen(None), brush = redB)
        self.widgGps.showGrid(x=True, y=True)
        self.widgGps.addItem(self.pltGpsOdom)
        self.widgGps.addItem(self.pltNovaOrientation)
        self.widgGps.addItem(self.pltDuroOrientation)
        self.widgGps.addItem(self.pltLeafOdom)
        dock4gps.addWidget(self.widgGps)
        self.pauseSensorReadClickedBtn.clicked.connect(self.pauseSensorReadClicked)
        

        self.SensorLaunchBtn.clicked.connect(self.SensorClicked)
        self.avoidanceLaunchBtn.clicked.connect(self.avoidanceClicked)
        self.novaSensorLaunchBtn.clicked.connect(self.novaSensorClicked)
        self.duroSensorLaunchBtn.clicked.connect(self.duroSensorClicked)


        
        
        self.tGps = pg.TextItem(text = "Gps", color = blue)
        self.tLeaf = pg.TextItem(text = "Leaf odom", color = red)
        self.tstart = pg.TextItem(text = "Start", color = white)

        
        dock2oth.setStyleSheet("background-color: rgb(18, 20, 23);")
        self.drawCircle(self.widgGps)
        # other controls
        widg2oth = pg.LayoutWidget()
        dock2oth.setStyleSheet("background-color: rgb(18, 20, 23);")
        widg2oth.setStyleSheet("background-color: rgb(40, 44, 52); color: rgb(171, 178, 191);")
        
        
        widg2oth.addWidget(self.SensorLaunchBtn, row = 1, col = 1)
        
        widg2oth.addWidget(self.avoidanceLaunchBtn, row = 3, col = 1)
        widg2oth.addWidget(self.novaSensorLaunchBtn, row = 2, col = 1)
        widg2oth.addWidget(self.duroSensorLaunchBtn, row = 2, col = 2)

        

        dock2oth.addWidget(widg2oth)

     
        self.pauseSensorReadClicked() # start paused - the same effect as pushed the pause button
        self.win.show()

    def updatePose(self):
        if self.leaf.stop_slow == False:
            self.duroRtkLabel.setText(self.leaf.duro_rtk)
            self.novaRtkLabel.setText(self.leaf.nova_rtk)
            # self.speedLabel.setText("%5.1f Km/h" % (self.leaf.leaf_speed * 3.6))
            # self.angleLabel.setText("%5.3f rad" % (self.leaf.leaf_angl))
            # self.isAutonomLabel.setText(str(self.leaf.leaf_is_autonomous))
            #self.ousterLefLabel.setText(self.leaf.ouster_lef_ok)
            self.ctrl_cmd_speedLabel.setText("%5.1f Km/h" % (self.leaf.ctrl_cmd_linear_speed))
            self.ctrl_cmd_steering_angle.setText("%5.3f rad" % (self.leaf.ctrl_cmd_steering_angle))
            self.ousterRigLabel.setText(self.leaf.ouster_rig_ok)
            self.currentPoseLabel.setText(self.leaf.current_pose_ok)
            self.ctrl_cmdOkLabel.setText(self.leaf.ctrl_cmd_ok)
            self.textLabel.setText(self.leaf.text)
            #self.zedOkLabel.setText(self.leaf.zed_ok)
            
            self.eukDet.setText(self.leaf.detection_ok)
            self.baseWay.setText(self.leaf.base_way_ok)
            #self.veloRig.setText(self.leaf.base_way_ok)
            self.pltGpsOdom.setPoints(self.leaf.pose_diff.x, self.leaf.pose_diff.y)
            
    def SensorClicked(self): 
        if self.SensorsLaunched is False:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launchTF = os.path.join(self.rospack.get_path("nissan_bringup"), "launch/tf_setup/tf_nissanleaf_statictf.launch")
            # launchTFNova = os.path.join(self.rospack.get_path("nissan_bringup"), "launch/tf_setup/tf_novatel_global_frame_tf_publisher.statictf.launch")
            # launchNovaGps = os.path.join(self.rospack.get_path("nissan_bringup"), "launch/sensory/gps.nova.launch")
            launchRightOuster = os.path.join(self.rospack.get_path("nissan_bringup"), "launch/sensory/ouster_right.launch")
            launchCan = os.path.join(self.rospack.get_path("can_leaf_driver"), "launch/nissan_can_control.launch")
            launchTmp = os.path.join(self.rospack.get_path("ros_guis"), "launch/temporary.launch")
            self.launchDuS = roslaunch.parent.ROSLaunchParent(uuid, [launchTF,launchRightOuster,launchCan,launchTmp])
            self.launchDuS.start()
            #rospy.loginfo(launchTFNova , launchTF + " started")
            self.SensorLaunchBtn.setText("Stop Sensors")
            self.SensorLaunchBtn.setStyleSheet("background-color: white")
        else:
            self.launchDuS.shutdown()
            rospy.loginfo("Sensors stopped.....")
            self.SensorLaunchBtn.setText("Start Sensors")
            self.SensorLaunchBtn.setStyleSheet("background-color: rgb(40, 44, 52)")
        self.SensorsLaunched = not self.SensorsLaunched

    def duroSensorClicked(self): 
        if self.duroSensorLaunched is False:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launchStr = os.path.join(self.rospack.get_path("nissan_bringup"), "launch/sensory/gps.duro.launch")
            launchTFDuro = os.path.join(self.rospack.get_path("nissan_bringup"), "launch/tf_setup/tf_duro_global_frame_tf_publisher.statictf.launch")
            self.launchDuS = roslaunch.parent.ROSLaunchParent(uuid, [launchStr,launchTFDuro])
            self.launchDuS.start()
            rospy.loginfo(launchStr + " started")
            self.duroSensorLaunchBtn.setText("Stop duro")
            self.duroSensorLaunchBtn.setStyleSheet("background-color: white")
        else:
            self.launchDuS.shutdown()
            rospy.loginfo("Duro stopped.....")
            self.duroSensorLaunchBtn.setText("Start Duro sensor")
            self.duroSensorLaunchBtn.setStyleSheet("background-color: rgb(40, 44, 52)")
        self.duroSensorLaunched = not self.duroSensorLaunched

    def novaSensorClicked(self): 
        if self.novaSensorLaunched is False:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launchStr = os.path.join(self.rospack.get_path("nissan_bringup"), "launch/sensory/gps.nova.launch")
            launchTFNova = os.path.join(self.rospack.get_path("nissan_bringup"), "launch/tf_setup/tf_novatel_global_frame_tf_publisher.statictf.launch")
            self.launchNoS = roslaunch.parent.ROSLaunchParent(uuid, [launchStr,launchTFNova])
            self.launchNoS.start()
            rospy.loginfo(launchStr + " started")
            self.novaSensorLaunchBtn.setStyleSheet("background-color: white")
            self.novaSensorLaunchBtn.setText("Stop Nova")
        else:
            self.launchNoS.shutdown()
            rospy.loginfo("Nova stopped.....")
            self.novaSensorLaunchBtn.setText("Start Nova sensor")
            self.novaSensorLaunchBtn.setStyleSheet("background-color: rgb(40, 44, 52)")
        self.novaSensorLaunched = not self.novaSensorLaunched

    def avoidanceClicked(self): 
        if self.avoidanceLaunched is False:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launchStr = os.path.join(self.rospack.get_path("obstacle_avoidance"), "launch/obstacle_avoidance.launch")
            self.launchtemp = roslaunch.parent.ROSLaunchParent(uuid, [launchStr])
            self.launchtemp.start()
            rospy.loginfo(launchStr + " started")
            self.avoidanceLaunchBtn.setText("Stop Avoidance")
            self.avoidanceLaunchBtn.setStyleSheet("background-color: white")
        else:
            self.launchtemp.shutdown()
            rospy.loginfo("Avoidance stopped.....")
            self.avoidanceLaunchBtn.setText("Start Avoidance")
            self.avoidanceLaunchBtn.setStyleSheet("background-color: rgb(40, 44, 52)")
        self.avoidanceLaunched = not self.avoidanceLaunched

    
    def pauseSensorReadClicked(self):
        self.leaf.stop_slow = not self.leaf.stop_slow
        self.leaf.sub_pos = not self.leaf.sub_pos
        if self.leaf.stop_slow:
            self.pauseSensorReadClickedBtn.setText("UnPause")
            self.ousterRigLabel.setText("paused")
            self.currentPoseLabel.setText("paused")
            self.duroRtkLabel.setText("paused")
            self.novaRtkLabel.setText("paused")
            self.eukDet.setText("paused")
            self.baseWay.setText("paused")
            self.ctrl_cmdOkLabel.setText("paused")
        else:
            self.pauseSensorReadClickedBtn.setText("Pause")

    def drawCircle(self, to_plot):
        circle = pg.ScatterPlotItem(size = 8, pen = pg.mkPen(None), brush = pg.mkBrush(80, 80, 80, 200))
        to_plot.addItem(circle)
        to_plot.setAspectLocked(lock = True, ratio = 1)
        x = np.sin(np.arange(0, np.pi*2, 0.1)) * 1.7
        y = np.cos(np.arange(0, np.pi*2, 0.1)) * 1.7
        circle.addPoints(x, y)

    
        



class Point2D(object):
    x = 0; y = 0
    def __init__(self, x = 0, y = 0):
        self.x = np.array([x])
        self.y = np.array([y])
    def abs(self):
        return (self.x**2 + self.y**2)**0.5
    def __add__(self, p):
        return Point2D(self.x + p.x, self.y + p.y)
    def __sub__(self, p):
        return Point2D(self.x - p.x, self.y - p.y)        

class LeafSubscriber(object):
    sub_pos = True; sub_pos_pre = True
    leaf_speed = 0.0; leaf_angl = 0.0
    pose_diff = Point2D(); pose_duro = Point2D(); pose_nova = Point2D()


    def __init__(self):
        self.registerPose()
        self.ouster_rig_ok = "-"
        self.current_pose_ok = "-"
        
        self.detection_ok = "-"
        self.base_way_ok = "-"
        self.ctrl_cmd_ok = "-"
        self.duro_rtk = "-"
        self.nova_rtk = "-"
        self.text="-"
        
        self.iterator = 0
        self.stop_slow = False # unpaused
        self.p4 = self.p5 = self.p6 = self.p7 = None
        #self.leaf_is_autonomous = "|"
        self.ctrl_cmd_linear_speed = 0.0
        self.ctrl_cmd_steering_angle = 0.0


    def slowGetMsg(self):
        if self.iterator == 100: # exeecute only every 100th time
            self.iterator = 0
            if self.stop_slow is False:
                try:
                    rospy.wait_for_message("/right_os1/os1_cloud_node/points", senmsg.PointCloud2, timeout=0.2)
                    self.ouster_rig_ok = "OK"
                except rospy.ROSException, e:
                    self.ouster_rig_ok = "ERR"
                try:
                    rospy.wait_for_message("/current_pose", geomsg.PoseStamped, timeout=0.2)
                    self.current_pose_ok = "OK"
                except rospy.ROSException, e:
                    #self.ouster_rig_ok = "ERR"
                    self.current_pose_ok = "ERR"
                try:
                    rospy.wait_for_message("/detection/lidar_detector/objects_markers", vismsg.MarkerArray, timeout=0.2)
                    #self.velo_lef_ok = "OK"
                    self.detection_ok = "OK"
                except rospy.ROSException, e:
                    self.detection_ok = "ERR"  
                try:
                    rospy.wait_for_message("/base_waypoints", autowmsgs.Lane, timeout=0.2)
                    self.base_way_ok = "OK"
                except rospy.ROSException, e:
                    #self.velo_rig_ok = "ERR" 
                    self.base_way_ok = "ERR" 
                try:
                    rospy.wait_for_message("/ctrl_cmd", autowmsgs.ControlCommandStamped, timeout=0.2)
                    #self.sick_ok = "OK"
                    self.ctrl_cmd_ok = "OK"
                except rospy.ROSException, e:
                    self.ctrl_cmd_ok = "ERR"                   
                
        else:
            self.iterator += 1

    def registerPose(self):
        self.stop_slow = False
        # self.p2 = rospy.Subscriber("/gps/duro/current_pose", geomsg.PoseStamped, self.duroPoseCallBack)
        # self.p3 = rospy.Subscriber("/gps/nova/current_pose", geomsg.PoseStamped, self.novaPoseCallBack)
        self.p4 = rospy.Subscriber("/gps/duro/status_string", rosmsg.String, self.duroRtkStatusCallBack)
        try:
            self.p5 = rospy.Subscriber("/gps/nova/bestvel", novamsg.NovatelVelocity, self.novaRtkStatusCallback)
        except:
            rospy.logwarn("no novatel_gps_msgs.msg custom messages built")
            self.nova_rtk = "NoNovaCustomMsg"
        self.p6 = rospy.Subscriber("/vehicle_status", autowmsgs.VehicleStatus, self.vehicleStatusCallback)
        self.p7 = rospy.Subscriber("/ctrl_cmd", autowmsgs.ControlCommandStamped, self.ctrlcmdStatusCallback)
        self.p8 = rospy.Subscriber("/text_overlay", OverlayText, self.textCallback)
    
    def unregisterPose(self):
        try:
            self.stop_slow = True
            # self.p2.unregister()
            # self.p3.unregister()
            self.p4.unregister()
            self.p5.unregister()
            self.p6.unregister()
            self.p7.unregister()
        except:
            None # not registered (subscubed) yet
        
    

    def vehicleStatusCallback(self, msg):
        self.leaf_angl = msg.angle
        self.leaf_speed = msg.speed
        if msg.drivemode == 0: 
            self.leaf_is_autonomous = "DRIVER"
        elif msg.drivemode == 1:
            self.leaf_is_autonomous = "AUTONOMOUS"
        else:
            self.leaf_is_autonomous = "UNDEF"

    def novaRtkStatusCallback(self, msg):
        self.nova_rtk = msg.velocity_type

    def duroRtkStatusCallBack(self, msg):
        self.duro_rtk = msg.data

    def ctrlcmdStatusCallback(self, msg):
        self.ctrl_cmd_linear_speed = msg.cmd.linear_velocity
        self.ctrl_cmd_steering_angle = msg.cmd.steering_angle

    def textCallback(self, msg):
        self.text = msg.text

    def handleRegistering(self):      
        if self.sub_pos != self.sub_pos_pre: # if subscribe / unsubscribe
            rospy.loginfo("Subscribed to pose topics: " + str(self.sub_pos))
            self.sub_pos_pre = self.sub_pos
            if self.sub_pos == True:
                self.registerPose()
            elif self.sub_pos == False:
                self.unregisterPose()  
  

if __name__ == "__main__":
    import sys
    rospy.loginfo("Leaf control 1 started... ")
    rospy.init_node("leafcontrol", anonymous=True)
    leafSub = LeafSubscriber()
    ph = PlotHandler(leafSub)
    ph.initializePlot()
    timerSlowSub = qtgqt.QtCore.QTimer()
    timerSlowSub.timeout.connect(leafSub.slowGetMsg)
    timerSlowSub.start(30)
    timerSubUnsub = qtgqt.QtCore.QTimer()
    timerSubUnsub.timeout.connect(leafSub.handleRegistering)
    timerSubUnsub.start(20)
    timerPose = qtgqt.QtCore.QTimer()
    timerPose.timeout.connect(ph.updatePose)
    timerPose.start(20)
    if (sys.flags.interactive != 1) or not hasattr(qtgqt.QtCore, "PYQT_VERSION"):
        qtgqt.QtGui.QApplication.instance().exec_()
