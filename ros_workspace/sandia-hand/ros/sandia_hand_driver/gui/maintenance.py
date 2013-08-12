#!/usr/bin/env python
import sys, os
from PyQt4.QtCore import *
from PyQt4.QtGui import *
import roslib.packages
import roslib; roslib.load_manifest('sandia_hand_driver')
import rospy, math, signal
from sandia_hand_msgs.msg import RawFingerState, RawPalmState
from sandia_hand_msgs.msg import RawFingerCommands

# all of this is an abomination. but it must get done asap. clean up sometime.

class BoardTab(QWidget):
  def __init__(self, board_name):
    super(BoardTab, self).__init__()
    self.board_name = board_name
    self.shd = roslib.packages.get_pkg_dir('sandia_hand_driver')
    self.process = None
    self.terminal = QTextEdit()
    self.terminal.setStyleSheet("QTextEdit { background-color: black; color: white;}")
    self.test_grid = QGridLayout()
    self.test_grid.setSpacing(5)
    for i in [0, 3]:
      self.test_grid.setColumnMinimumWidth(i  , 15)
      self.test_grid.setColumnMinimumWidth(i+1, 50)
      self.test_grid.setColumnStretch(i+2, 1)
  def processOnFinished(self, exitCode, exitStatus):
    if exitCode != 0:
      QMessageBox.about(self, "bad", "error detected. see terminal output.")
    self.process = None
  def terminalReadOutput(self):
    #sys.stdout.write(self.process.readAllStandardOutput())
    self.terminal.append(QString(self.process.readAllStandardOutput()))
  def spawnProcess(self, cmd, onFinished):
    self.terminal.append(cmd + "\n")
    self.process = QProcess(self)
    self.process.finished.connect(onFinished)
    self.process.setProcessChannelMode(QProcess.MergedChannels)
    self.process.readyReadStandardOutput.connect(self.terminalReadOutput)
    self.process.start("/bin/bash", ["-c", cmd])
 
class MotorBoardTab(BoardTab):
  def __init__(self, exit_on_success):
    super(MotorBoardTab, self).__init__("fmcb")
    self.rfc_pub = rospy.Publisher('raw_commands', RawFingerCommands)
    self.exit_on_success = exit_on_success
    self.bootloader_btn = QPushButton("Install Bootloader")
    self.bootloader_btn.clicked.connect(self.bootloaderClicked)
    self.application_btn = QPushButton("Install Application")
    self.application_btn.clicked.connect(self.applicationClicked)
    self.test_btn = QPushButton("Test")
    self.test_btn.clicked.connect(self.testClicked)
    self.auto_btn = QPushButton("Awesome")
    self.auto_btn.clicked.connect(self.autoClicked)
    button_hbox = QHBoxLayout()
    button_hbox.addWidget(self.bootloader_btn)
    button_hbox.addWidget(self.application_btn)
    button_hbox.addWidget(self.test_btn)
    button_hbox.addWidget(self.auto_btn)
    button_hbox.addStretch(1)
    self.test_grid.addWidget(QLabel("Accelerometer magnitude"), 0, 2)
    self.accel_mag_light = QLabel() #"FAIL")
    self.accel_mag_light.setStyleSheet("QWidget {background-color:yellow}")
    self.accel_mag_label = QLabel("0")
    self.test_grid.addWidget(self.accel_mag_light, 0, 0)
    self.test_grid.addWidget(self.accel_mag_label, 0, 1)
    # test distal accelerometer (to verify phalange comms works)
    self.test_grid.addWidget(QLabel("Distal accelerometer magnitude"), 1, 2)
    self.dp_accel_mag_light = QLabel() #"FAIL")
    self.dp_accel_mag_light.setStyleSheet("QWidget {background-color:yellow}")
    self.dp_accel_mag_label = QLabel("0")
    self.test_grid.addWidget(self.dp_accel_mag_light, 1, 0)
    self.test_grid.addWidget(self.dp_accel_mag_label, 1, 1)
    # set up motor tests
    x = 500
    self.motor_tests = \
            [ { 'start_time': 1.0, 'duration': 0.5, 'ticks': [ x, 0, 0] }, \
              { 'start_time': 2.0, 'duration': 0.7, 'ticks': [-x, 0, 0] }, \
              { 'start_time': 3.0, 'duration': 0.5, 'ticks': [ 0, 0, 0] }, \
              { 'start_time': 4.0, 'duration': 0.5, 'ticks': [ 0, x, 0] }, \
              { 'start_time': 5.0, 'duration': 0.7, 'ticks': [ 0,-x, 0] }, \
              { 'start_time': 6.0, 'duration': 0.5, 'ticks': [ 0, 0, 0] }, \
              { 'start_time': 7.0, 'duration': 0.5, 'ticks': [ 0, 0, x] }, \
              { 'start_time': 8.0, 'duration': 0.7, 'ticks': [ 0, 0,-x] }, \
              { 'start_time': 9.0, 'duration': 0.5, 'ticks': [ 0, 0, 0] } ]
    self.motor_test_start_time = 0
    self.motor_test_started = False
    self.motor_test_lights = []
    self.motor_test_labels = []
    self.motor_test_passed = [False] * len(self.motor_tests)
    for i in xrange(0, len(self.motor_tests)):
      light = QLabel()
      light.setStyleSheet("QWidget {background-color:yellow}")
      self.motor_test_lights.append(light)
      self.test_grid.addWidget(light, 2 + i, 0)
      label = QLabel()
      self.motor_test_labels.append(label)
      self.test_grid.addWidget(label, 2 + i, 1)
      t = self.motor_tests[i]['ticks']
      dur = self.motor_tests[i]['duration']
      self.test_grid.addWidget(QLabel("Motor test: [%d %d %d] in %.1f seconds" % (t[0], t[1], t[2], dur)), 2 + i, 2)

    vbox = QVBoxLayout()
    vbox.addLayout(button_hbox)
    vbox.addLayout(self.test_grid)
    vbox.addStretch(1)
    vbox.addWidget(self.terminal)
    self.setLayout(vbox)
    self.motor_test_idx = 0
  def onUpdateUI(self, accel_raw, dp_accel_raw, \
                       hall_tgt, hall_pos, fmcb_effort):
    # flip around hall target vector, argh 
    s = hall_tgt[0]
    hall_tgt[0] = hall_tgt[2]
    hall_tgt[2] = s
    if self.motor_test_start_time == 0:
      self.motor_test_start_time = rospy.get_time()
    # test motor module accel
    accel_mag = 0
    for i in xrange(0,3):
      accel_mag += accel_raw[i]**2
    accel_mag = math.sqrt(accel_mag)
    self.accel_mag_label.setText("%d" % accel_mag)
    ready_to_exit = self.exit_on_success
    if (accel_mag > 800 and accel_mag < 1200):
      self.accel_mag_light.setStyleSheet("QWidget {background-color:green}")
    else:
      ready_to_exit = False
      self.accel_mag_light.setStyleSheet("QWidget {background-color:red}")
    # test distal accel (to verify phalange comms works)
    dp_accel_mag = 0
    for i in xrange(0,3):
      dp_accel_mag += dp_accel_raw[i]**2
    dp_accel_mag = math.sqrt(dp_accel_mag)
    self.dp_accel_mag_label.setText("%d" % dp_accel_mag)
    if (dp_accel_mag > 800 and dp_accel_mag < 1200):
      self.dp_accel_mag_light.setStyleSheet("QWidget {background-color:green}")
    else:
      ready_to_exit = False
      self.dp_accel_mag_light.setStyleSheet("QWidget {background-color:red}")
    # check motor test results
    for b in self.motor_test_passed:
      if not b:
        ready_to_exit = False
    if ready_to_exit:
      if self.process:
        print "stopping child process"
        os.system("rosnode kill sandia_hand_loose_finger_node")
        print "sleeping..."
        os.system("sleep 0.25")
        print "quitting."
        QApplication.quit()
    # test motors
    if self.motor_test_idx < len(self.motor_tests):
      test_start_time = self.motor_tests[self.motor_test_idx]['start_time']
      if rospy.get_time() - self.motor_test_start_time > test_start_time:
        dt = rospy.get_time()-self.motor_test_start_time-test_start_time
        # this is so bad. fix it sometime.
        if not self.motor_test_started:
          self.motor_test_started = True
          self.sendMotorTicks(self.motor_tests[self.motor_test_idx]['ticks'])
        elif hall_tgt == self.motor_tests[self.motor_test_idx]['ticks']:
          error = 0
          for i in xrange(0, 3):
            error += hall_pos[i] - hall_tgt[i]
          #print "%.3f  %d" % (dt, error)
          if abs(error) < 3:
            print "reached goal in duration %.3f" % dt 
            self.motor_test_labels[self.motor_test_idx].setText("%.3f" % dt)
            test_max_duration = self.motor_tests[i]['duration']
            light = self.motor_test_lights[self.motor_test_idx]
            if (dt < test_max_duration):
              light.setStyleSheet("QWidget {background-color:green}")
              self.motor_test_passed[self.motor_test_idx] = True
            else:
              light.setStyleSheet("QWidget {background-color:red}")
            self.motor_test_idx += 1
            self.motor_test_started = False
  def sendMotorTicks(self, ticks):
    if len(ticks) != 3:
      print "invalid length of ticks vector: %d" % len(ticks)
      return
    print "sending: %.1f %.1f %.1f" % (ticks[0], ticks[1], ticks[2])
    rfc = RawFingerCommands()
    rfc.motor_targets = ticks
    self.rfc_pub.publish(rfc)
  def bootloaderClicked(self):
    self.spawnProcess("cd %s/../../firmware/build && make fmcb-bl-gpnvm && make fmcb-bl-program && echo \"\ntasks complete\"" % (self.shd), self.processOnFinished)
  def applicationClicked(self):
    self.spawnProcess("cd %s && bin/loose_finger_cli /dev/ttyUSB0 burn ../../firmware/build/fmcb/std/fmcb-std.bin && echo \"\ntasks complete\"" % (self.shd), self.processOnFinished)
  def testClicked(self):
    if self.process is None:
      self.spawnProcess("cd %s && bin/sandia_hand_loose_finger_node _use_proximal_phalange:=false" % self.shd, self.processOnFinished)
  def autoOnFinished(self, exitCode, exitStatus):
    if exitCode != 0:
      QMessageBox.about(self, "bad!", "error detected. see terminal output.")
      return
    self.spawnProcess("cd %s && bin/sandia_hand_loose_finger_node _use_proximal_phalange:=false" % self.shd, self.processOnFinished)
  def autoClicked(self):
    self.spawnProcess("cd %s/../../firmware/build && make fmcb-bl-gpnvm && make fmcb-bl-program && cd %s && bin/loose_finger_cli /dev/ttyUSB0 burn ../../firmware/build/fmcb/std/fmcb-std.bin && echo \"\ntasks complete\"" % (self.shd, self.shd), self.autoOnFinished)
############################################################################
class TactileBoardTab(BoardTab):
  def __init__(self, board_name, num_taxels, exit_on_success):
    super(TactileBoardTab, self).__init__(board_name)
    self.num_taxels = num_taxels
    self.exit_on_success = exit_on_success
    self.bootloader_btn = QPushButton("Install Bootloader")
    self.bootloader_btn.clicked.connect(self.bootloaderClicked)
    self.application_btn = QPushButton("Install Application")
    self.application_btn.clicked.connect(self.applicationClicked)
    self.test_btn = QPushButton("Test")
    self.test_btn.clicked.connect(self.testClicked)
    self.auto_btn = QPushButton("Awesome")
    self.auto_btn.clicked.connect(self.autoClicked)
    self.test_grid.addWidget(QLabel("Accelerometer magnitude"), 0, 2)
    self.accel_mag_light = QLabel() #"FAIL")
    self.accel_mag_light.setStyleSheet("QWidget {background-color:yellow}")
    self.accel_mag_label = QLabel("0")
    self.test_grid.addWidget(self.accel_mag_light, 0, 0)
    self.test_grid.addWidget(self.accel_mag_label, 0, 1)
    self.tactile_min = [65535] * self.num_taxels
    self.tactile_max = [0] * self.num_taxels
    self.tactile_labels = []
    self.tactile_lights = []
    self.tactile_desc = []
    for i in xrange(0, self.num_taxels):
      self.tactile_lights.append(QLabel())
      self.tactile_lights[i].setStyleSheet("QWidget {background-color:yellow}")
      self.test_grid.addWidget(self.tactile_lights[i], \
                               self.tactileRow(i), self.tactileCol(i))
      self.tactile_labels.append(QLabel("0"))
      self.test_grid.addWidget(self.tactile_labels[i], \
                               self.tactileRow(i), self.tactileCol(i)+1)
      self.tactile_desc.append(QLabel("Tactile %d" % i))
      self.test_grid.addWidget(self.tactile_desc[i], \
                               self.tactileRow(i), self.tactileCol(i)+2)
    button_hbox = QHBoxLayout()
    button_hbox.addWidget(self.bootloader_btn)
    button_hbox.addWidget(self.application_btn)
    button_hbox.addWidget(self.test_btn)
    button_hbox.addWidget(self.auto_btn)
    button_hbox.addStretch(1)
    self.vbox = QVBoxLayout()
    self.vbox.addLayout(button_hbox)
    self.vbox.addLayout(self.test_grid)
    self.vbox.addStretch(1)
    self.vbox.addWidget(self.terminal)
    self.setLayout(self.vbox)
  def tactileRow(self, tactile_idx): # tactile sensor row
    return 1 + tactile_idx % 16
  def tactileCol(self, tactile_idx): # tactile sensor col
    if tactile_idx < 16:
      return 0
    else:
      return 3
  def onUpdateCommonUI(self, accel_raw, tactile_raw):
    accel_mag = 0
    for i in xrange(0,3):
      accel_mag += accel_raw[i]**2
    accel_mag = math.sqrt(accel_mag)
    tactile_range = [0] * len(tactile_raw)
    for i in xrange(0, len(tactile_raw)):
      t = tactile_raw[i]
      if t > self.tactile_max[i]:
        self.tactile_max[i] = t
      if t < self.tactile_min[i]:
        self.tactile_min[i] = t
      tactile_range[i] = self.tactile_max[i] - self.tactile_min[i]
    self.accel_mag_label.setText("%d" % accel_mag)
    ready_to_exit = self.exit_on_success
    if (accel_mag > 800 and accel_mag < 1200):
      self.accel_mag_light.setStyleSheet("QWidget {background-color:green}")
    else:
      ready_to_exit = False
      self.accel_mag_light.setStyleSheet("QWidget {background-color:red}")
    for i in xrange(0, len(tactile_raw)):
      self.tactile_desc[i].setText("Tactile %02d: %d" % (i, tactile_raw[i]))
      self.tactile_labels[i].setText("%d" % tactile_range[i])
      if tactile_range[i] > 3000:
        self.tactile_lights[i].setStyleSheet("QWidget {background-color:green}")
      else:
        ready_to_exit = False
        self.tactile_lights[i].setStyleSheet("QWidget {background-color:red}")
    if ready_to_exit:
      if self.process:
        print "stopping child process"
        os.system("rosnode kill sandia_hand_loose_finger_node")
        os.system("rosnode kill sandia_hand_loose_palm_node")
        print "sleeping..."
        os.system("sleep 0.25")
        print "quitting."
        QApplication.quit()
  def onUpdateUI(self, accel_raw, tactile_raw):
    self.onUpdateCommonUI(accel_raw, tactile_raw)
  def onUpdatePalmUI(self, accel_raw, gyro_raw, mag_raw, tactile_raw):
    # todo: test gyro and mag
    self.onUpdateCommonUI(accel_raw, tactile_raw)
  def bootloaderClicked(self):
    if "palm" in self.board_name:
      self.spawnProcess("cd %s/../../firmware/build && make %s-bl-gpnvm && make %s-bl-program && echo \"\ntasks complete\"" % (self.shd, self.board_name, self.board_name), self.processOnFinished)
    else:
      self.spawnProcess("cd %s && bin/loose_finger_cli /dev/ttyUSB0 pb on && sleep 1 && cd %s/../../firmware/build && make %s-bl-gpnvm && make %s-bl-program && echo \"\ntasks complete\"" % (self.shd, self.shd, self.board_name, self.board_name), self.processOnFinished)
  def applicationClicked(self):
    bn = self.board_name
    if "palm" in self.board_name:
      self.spawnProcess("cd %s && bin/loose_palm_cli /dev/ttyUSB0 %s ../../firmware/build/palm/std/palm-std.bin && echo \"\ntasks complete\"" % (self.shd, self.applicationCmd()), self.processOnFinished)
    else:
      self.spawnProcess("cd %s && bin/loose_finger_cli /dev/ttyUSB0 %s ../../firmware/build/%s/std/%s-std.bin && echo \"\ntasks complete\"" % (self.shd, self.applicationCmd(), bn, bn), self.processOnFinished)
  def autoOnFinished(self, exitCode, exitStatus):
    if exitCode != 0:
      QMessageBox.about(self, "bad!", "error detected. see terminal output.")
      return
    if self.board_name == "f3":
      cmd = "cd %s && bin/sandia_hand_loose_finger_node _use_proximal_phalange:=false" % self.shd
    elif self.board_name == "f2":
      cmd = "cd %s && bin/sandia_hand_loose_finger_node _use_distal_phalange:=false" % self.shd
    elif "palm" in self.board_name:
      cmd = "cd %s && bin/sandia_hand_loose_palm_node" % self.shd
    else:
      QMessageBox.about(self, "bad!", "auto button not ready for this board")
      return
    self.spawnProcess(cmd, self.processOnFinished)
  def testClicked(self):
    self.autoOnFinished(0, 0)
  def autoClicked(self):
    bn = self.board_name
    shd = self.shd
    if "palm" in self.board_name:
      self.spawnProcess("cd %s/../../firmware/build && make %s-bl-gpnvm && make %s-bl-program && cd %s && bin/loose_palm_cli /dev/ttyUSB0 burn ../../firmware/build/palm/std/palm-std.bin && echo \"\ntasks complete\"" % (shd, bn, bn, shd), self.autoOnFinished)
    else:
      self.spawnProcess("cd %s && bin/loose_finger_cli /dev/ttyUSB0 pb on && sleep 1 && cd %s/../../firmware/build && make %s-bl-gpnvm && make %s-bl-program && cd %s && bin/loose_finger_cli /dev/ttyUSB0 %s ../../firmware/build/%s/std/%s-std.bin && echo \"\ntasks complete\"" % (shd, shd, bn, bn, shd, self.applicationCmd(), bn, bn), self.autoOnFinished)
  def applicationCmd(self):
    bn = self.board_name
    if bn == "f3":
      return "dburn"
    elif bn == "f2":
      return "pburn"
    elif bn == "fmcb" or ("palm" in bn):
      return "burn"
    else:
      QMessageBox.about(self, "bad!", "application not defined for %s" % bn)
      return 

class MaintenanceWindow(QWidget):
  def __init__(self):
    super(MaintenanceWindow, self).__init__()
    self.tab_widget = QTabWidget()
    # todo: parse which tab is auto exit worthy
    auto_exit_board_name = rospy.get_param("~auto_exit_board_name", "")
    auto_awesome = rospy.get_param("~auto_awesome", False)
    self.f3_tab    = TactileBoardTab("f3",    12, auto_exit_board_name == "f3") 
    self.f2_tab    = TactileBoardTab("f2",     6, auto_exit_board_name == "f2")
    self.rpalm_tab = TactileBoardTab("rpalm", 32, \
                                     auto_exit_board_name == "rpalm")
    self.lpalm_tab = TactileBoardTab("lpalm", 32, \
                                     auto_exit_board_name == "lpalm")
    self.fmcb_tab  = MotorBoardTab(auto_exit_board_name == "fmcb")
    self.tab_widget.addTab(self.f3_tab, "f3")
    self.tab_widget.addTab(self.f2_tab, "f2")
    self.tab_widget.addTab(self.fmcb_tab, "fmcb")
    self.tab_widget.addTab(self.rpalm_tab, "rpalm")
    self.tab_widget.addTab(self.lpalm_tab, "lpalm")

    # todo: find a cleaner way to do this
    if auto_exit_board_name == "f2":
      self.tab_widget.setCurrentIndex(1) 
    elif auto_exit_board_name == "fmcb":
      self.tab_widget.setCurrentIndex(2)
    elif auto_exit_board_name == "rpalm":
      self.tab_widget.setCurrentIndex(3)
    elif auto_exit_board_name == "lpalm":
      self.tab_widget.setCurrentIndex(4)


    vbox = QVBoxLayout()
    vbox.addWidget(self.tab_widget)
    self.setGeometry(850, 100, 800, 800)
    self.setLayout(vbox)
    self.setWindowTitle('Sandia Hand Maintenance')
    #cp = QDesktopWidget().availableGeometry().center()
    #qr = self.frameGeometry()
    #qr.moveCenter(cp)
    #self.move(qr.topLeft())
    self.show()
    self.finger_sub = rospy.Subscriber('raw_state', RawFingerState, 
                                       self.finger_state_cb)
    self.palm_sub = rospy.Subscriber('raw_palm_state', RawPalmState,
                                     self.palm_state_cb)
    self.connect(self, SIGNAL('updateF3'), self.f3_tab.onUpdateUI)
    self.connect(self, SIGNAL('updateF2'), self.f2_tab.onUpdateUI)
    self.connect(self, SIGNAL('updateFMCB'), self.fmcb_tab.onUpdateUI)
    self.connect(self, SIGNAL('updatePalm'), self.rpalm_tab.onUpdatePalmUI)
    self.connect(self, SIGNAL('updatePalm'), self.lpalm_tab.onUpdatePalmUI)
    # this is gross. figure out a better way sometime
    if auto_awesome:
      if auto_exit_board_name == "f2":
        self.f2_tab.autoClicked()
      elif auto_exit_board_name == "fmcb":
        self.fmcb_tab.autoClicked()
      elif auto_exit_board_name == "rpalm":
        self.rpalm_tab.autoClicked()
      elif auto_exit_board_name == "lpalm":
        self.lpalm_tab.autoClicked()

  def finger_state_cb(self, msg):
    # copy everything out of the ROS thread and into UI threads
    f3_accel_raw = [0] * 3
    f2_accel_raw = [0] * 3
    fmcb_accel_raw = [0] * 3
    f3_tactile_raw = [0] * 12
    f2_tactile_raw = [0] *  6
    for i in xrange(0, 3):
      f3_accel_raw[i] = msg.dp_accel[i]
      f2_accel_raw[i] = msg.pp_accel[i]
      fmcb_accel_raw[i] = msg.mm_accel[i]
    for i in xrange(0, 12):
      f3_tactile_raw[i] = msg.dp_tactile[i]
    for i in xrange(0,  6):
      f2_tactile_raw[i] = msg.pp_tactile[i]
    hall_tgt = [0] * 3
    hall_pos = [0] * 3
    fmcb_effort = [0] * 3
    for i in xrange(0, 3):
      hall_tgt[i] = msg.hall_tgt[i]
      hall_pos[i] = msg.hall_pos[i]
      fmcb_effort[i] = msg.fmcb_effort[i]
    self.emit(SIGNAL('updateF3'), f3_accel_raw, f3_tactile_raw)
    self.emit(SIGNAL('updateF2'), f2_accel_raw, f2_tactile_raw)
    self.emit(SIGNAL('updateFMCB'), fmcb_accel_raw, f3_accel_raw, hall_tgt, hall_pos, fmcb_effort)

  def palm_state_cb(self, msg):
    # copy everything out of the ROS thread and into UI threads
    palm_accel_raw = [0] * 3
    palm_gyro_raw  = [0] * 3
    palm_mag_raw   = [0] * 3
    palm_tactile_raw = [0] * 32
    for i in xrange(0, 3):
      palm_accel_raw[i] = msg.palm_accel[i]
      palm_gyro_raw[i]  = msg.palm_gyro[i]
      palm_mag_raw[i]   = msg.palm_mag[i]
    for i in xrange(0, 32):
      palm_tactile_raw[i] = msg.palm_tactile[i]
    self.emit(SIGNAL('updatePalm'), palm_accel_raw, palm_gyro_raw, palm_mag_raw, palm_tactile_raw)

if __name__ == '__main__':
  rospy.init_node('sandia_hand_maintenance')
  signal.signal(signal.SIGINT, signal.SIG_DFL) 
  app = QApplication(sys.argv)
  mw = MaintenanceWindow()
  rv = app.exec_()
  os.system("rosnode kill sandia_hand_loose_finger_node") # just in case
  os.system("rosnode kill sandia_hand_loose_palm_node") # just in case
  sys.exit(rv)

