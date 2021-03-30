#!/usr/bin/env python

import rospy, cv2, re, sys
import numpy as np
from std_msgs.msg import String, UInt32, Bool
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from python_qt_binding.QtCore import *
from python_qt_binding.QtGui import *
from python_qt_binding.QtWidgets import *

TEXT_WIDTH = 420
TEXT_HEIGHT = 150

class IrisGuiWidget(QWidget):
	def __init__(self):
		super(IrisGuiWidget, self).__init__()
		self.setWindowTitle('Robotic Assistance Interface')
		# check if stereo vision should be enabled
		T = rospy.get_published_topics()
		text = ''.join(str(r) for t in T for r in t)
		if re.match('.*?(cam2/camera).*?', text) != None:
			self.stereo_vision = True
			print "User Interface: Two cameras found."
		else:
			self.stereo_vision = False
			print "User Interface: One camera found."
		# widget variables
		self.m_vid1 = QLabel()
		self.s_vid1 = QLabel()
		self.r_vid1 = QLabel()
		self.v_vid1 = QLabel()
		self.directions = QLabel()
		self.repeat_text = QLabel()
		self.p2p_button = QPushButton('point to point')
		self.p2l_button = QPushButton('point to line')
		self.l2l_button = QPushButton('line to line')
		self.par_button = QPushButton('parallel lines')
		self.con_button = QPushButton('conic')
		self.done_button = QPushButton('confirm')
		self.no_button = QPushButton('no')
		self.yes_button = QPushButton('yes')
		self.pause_button = QPushButton('pause publishers')
		# shared variables
		self.bridge = CvBridge()
		self.total_num_tasks = 0
		self.task_ids = []
		self.current_widget = 'menu'
		self.publish_new_tasks_now = False
		self.static = [False, False]
		self.dynamic = [False, False]
		self.draw = True
		self.draw_vs = False
		self.x_input = 1
		self.y_input = 1
		self.current_id = None
		self.index = 0
		self.static_total = 0
		self.dynamic_total = 0
		# camera 1 variables
		self.static_count_cam1 = 0
		self.dynamic_count_cam1 = 0
		self.tasks_cam1 = []
		self.temp_cam1 = []
		self.static_cam1 = []
		self.dynamic_cam1 = []

		self.corners = []
		self.merged_list = []
		# camera 2 variables
		if self.stereo_vision:
			self.m_vid2 = QLabel()
			self.s_vid2 = QLabel()
			self.r_vid2 = QLabel()
			self.v_vid2 = QLabel()
			self.tasks_cam2 = []
			self.static_cam2 = []
			self.dynamic_cam2 = []
			self.temp_cam2 = []
			self.static_count_cam2 = 0
			self.dynamic_count_cam2 = 0
			self.corners2 = []
			self.merged_list2 = []
			self.directions_list = [['Click once in each image to set corresponding static points, then press confirm', 'Click once in each image to set corresponding dynamic points, then press confirm'], \
						['Click two points in each image to set corresponding static lines, then press confirm', 'Click once in each image to set corresponding dynamic points, then press confirm'], \
						['Click two points in each image to set corresponding static lines, then press confirm', 'Click two points in each image to set corresponding dynamic lines, then press confirm'], \
						['Click two points in each image to set corresponding static lines, then press confirm', 'Click two points in each image to set corresponding dynamic lines, then press confirm'], \
						['Click five points in each image to set corresponding static conics, then press confirm', 'Click three points in each image to set corresponding dynamic points, then press confirm']]
		else: 
			self.directions_list = [['Click to set a static point, then press confirm', 'Click to set a dynamic point, then press confirm'], \
						['Click two points to set a static line, then press confirm', 'Click to set a dynamic point, then press confirm'], \
						['Click two points to set a static line, then press confirm', 'Click to set two points to set a dynamic line, then press confirm'], \
						['Click two points to set a static line, then press confirm', 'Click to set two points to set a dynamic line, then press confirm'], \
						['Click five points to create a conic, then press confirm', 'Click to set three dynamic points, then press confirm']]
	
		# setup pens
		self.pens = [QPen(Qt.green), QPen(Qt.blue), QPen(Qt.red), QPen(Qt.magenta), QPen(Qt.yellow), QPen(Qt.darkBlue), QPen(Qt.darkMagenta), QPen(Qt.cyan)]
		self.line_pens = [QPen(Qt.green), QPen(Qt.blue), QPen(Qt.red), QPen(Qt.magenta), QPen(Qt.yellow), QPen(Qt.darkBlue), QPen(Qt.darkMagenta), QPen(Qt.cyan)]
		for i in range(len(self.pens)):
			self.line_pens[i].setWidth(2)
			self.pens[i].setWidth(10)
			self.pens[i].setCapStyle(Qt.RoundCap)
			self.pens[i].setJoinStyle(Qt.RoundJoin)
		# initialize and set layouts
		self.main_layout = QVBoxLayout()
		self.initialize_widgets()
		self.setLayout(self.main_layout)
		# setup publishers and subscribers 
		self.sub_cam1 = rospy.Subscriber('/cam1/camera/image_raw/compressed',  CompressedImage, self.callback_cam1, queue_size = 3)
		self.sub_mtf = rospy.Subscriber('/cam1/trackers/patch_tracker', String, self.callback_mtf1, queue_size = 3)
		self.sub_reset = rospy.Subscriber('/error_reset', Bool, self.callback_reset, queue_size = 3)
		if self.stereo_vision:
			self.sub_cam2 = rospy.Subscriber('/cam2/camera/image_raw/compressed',  CompressedImage, self.callback_cam2, queue_size = 3)
			self.sub_mtf2 = rospy.Subscriber('/cam2/trackers/patch_tracker', String, self.callback_mtf2, queue_size = 3)
			self.pub_tasks_cam2 = rospy.Publisher('/cam2/task_coordinates', String, queue_size = 1)
			self.pub_centers_cam2 = rospy.Publisher('/cam2/centers', String, queue_size = 1)
		self.pub_calculate = rospy.Publisher('/calculate', UInt32, queue_size = 1)
		self.pub_stereo = rospy.Publisher('/stereo_vision', UInt32, queue_size = 1)
		self.pub_ids = rospy.Publisher('/task_ids', String, queue_size = 1)
		self.pub_tasks_cam1 = rospy.Publisher('/cam1/task_coordinates', String, queue_size = 1)
		self.pub_centers_cam1 = rospy.Publisher('/cam1/centers', String, queue_size = 1)
		self.pub_reset = rospy.Publisher('/reset', Bool, queue_size = 1)
#############################################################################################################
# LOST TRACKER RESET WIDGET 
#############################################################################################################
	def reset_clicked(self):
		self.reset_all(True, 'menu')
		
	def reset_layout(self):
		layout = QHBoxLayout()
		text_layout = QVBoxLayout()
		reset_button = QPushButton('One or more trackers were lost.\nPress to reset.')
		reset_button.clicked.connect(self.reset_clicked)
		reset_button.setStyleSheet("background-color: rgba(16, 123, 227, 90%); selection-background-color: rgba(16, 123, 227, 50%); font-size: 32px")
		reset_button.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
		text_layout.addWidget(reset_button)
		layout.addLayout(text_layout)        
		return layout
#############################################################################################################
# VISUAL SERVOING WIDGET 
#############################################################################################################
	def pause_vs(self):
		if self.pause_button.text() == 'pause':
			self.pause_button.setText('start')
		else:
			self.pause_button.setText('pause')

	def vs_layout(self):
		layout = QVBoxLayout()
		image_layout = QHBoxLayout()
		self.pause_button.clicked.connect(self.pause_vs)
		self.pause_button.setStyleSheet("background-color: rgba(16, 123, 227, 60%); selection-background-color: rgba(16, 123, 227, 70%); font-size: 32px")     
		self.pause_button.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
		self.v_vid1.setAlignment(Qt.AlignCenter)
		image_layout.addWidget(self.v_vid1)
		if self.stereo_vision:
			self.v_vid2.setAlignment(Qt.AlignCenter)
			image_layout.addWidget(self.v_vid2)
		layout.addLayout(image_layout)
		# layout.addWidget(self.pause_button)
		return layout
#############################################################################################################
# REPEAT WIDGET 
#############################################################################################################
	# def publish_tasks(self, pub):
	# 	self.tasks_cam1.append(self.temp_cam1)
	# 	self.task_ids.append(self.current_id)
	# 	self.total_num_tasks += 1
	# 	if self.stereo_vision:
	# 		self.tasks_cam2.append(self.temp_cam2)
	# 	self.reset_all(False, 'vs')

	def add_current_task(self, pub):
		self.tasks_cam1.append(self.temp_cam1)
		self.task_ids.append(self.current_id)
		self.total_num_tasks += 1
		if self.stereo_vision:
			self.tasks_cam2.append(self.temp_cam2)
		if pub:
			self.reset_all(False, 'vs')
		else:
			self.reset_all(False, 'menu')

	def reset_repeat_labels(self):
		# Option Root
		self.repeat_text.setText('Would you like to set another task?')
		# Option 1 root
		self.yes_button.setText('yes')
		# Option 2 root
		self.no_button.setText('no')

	def yes(self):
		# Option 1 branch
		if self.repeat_text.text() == 'Would you like to set another task?':
			self.repeat_text.setText('Please confirm you would like to set another task')
			self.yes_button.setText('confirm')
			self.no_button.setText('back')
		# Option 1 branch
		elif self.repeat_text.text() == 'Please confirm you would like to set another task':
			self.add_current_task(False)
		# Option 2 branch
		elif self.repeat_text.text() == 'Please select an option:':
			self.repeat_text.setText('Please confirm you are ready to publish tasks')
			self.yes_button.setText('confirm')
			self.no_button.setText('back')  
		# Option 2 branch
		elif self.repeat_text.text() == 'Please confirm you are ready to publish tasks':
			self.add_current_task(True)
		# Option 2 branch
		elif self.repeat_text.text() == 'Would you like to cancel current task or all set tasks?':
			self.repeat_text.setText('Please confirm cancellation of current task')
			self.yes_button.setText('confirm')
			self.no_button.setText('back')
		# Option 2 branch
		elif self.repeat_text.text() == 'Please confirm cancellation of current task':
			self.reset_all(False, 'menu')
		# Option 2 branch
		elif self.repeat_text.text() == 'Please confirm cancellation of all tasks':
			self.reset_all(True, 'menu')
		else:
			print 'User Interface: Must have missed an edge case\n'
			print 'text:', self.repeat_text.text()
			print 'yes:', self.yes_button.text()
			print 'no:', self.no_button.text()

	def no(self):
		# Option 2 branch
		if self.repeat_text.text() == 'Would you like to set another task?':
			self.repeat_text.setText('Please select an option:')
			self.yes_button.setText('publish set tasks')
			self.no_button.setText('cancel tasks')
		# Option 2 branch
		elif self.repeat_text.text() == 'Please select an option:':
			self.repeat_text.setText('Would you like to cancel current task or all set tasks?')
			self.yes_button.setText('current task')
			self.no_button.setText('all tasks')
		# Option 2 branch
		elif self.repeat_text.text() == 'Would you like to cancel current task or all set tasks?':
			self.repeat_text.setText('Please confirm cancellation of all tasks')
			self.yes_button.setText('confirm')
			self.no_button.setText('back')
		# Option 2 branch
		elif self.repeat_text.text() == 'Please confirm cancellation of current task':
			self.reset_repeat_labels()
		# Option 2 branch
		elif self.repeat_text.text() == 'Please confirm cancellation of all tasks':
			self.reset_repeat_labels()
		# Option 2 branch
		elif self.repeat_text.text() == 'Please confirm you are ready to publish tasks':
			self.reset_repeat_labels()
		# Option 1 branch
		elif self.repeat_text.text() == 'Please confirm you would like to set another task':
			self.reset_repeat_labels()
		else:
			print 'User Interface: Must have missed an edge case\n'
			print 'text:', self.repeat_text.text()
			print 'yes:', self.yes_button.text()
			print 'no:', self.no_button.text()

	def repeat_layout(self):
		if self.stereo_vision:
			layout = QVBoxLayout()
			text_layout = QHBoxLayout()
			image_layout = QHBoxLayout()
		else:
			layout = QHBoxLayout()
			text_layout = QVBoxLayout()
			image_layout = QHBoxLayout()
		self.repeat_text.setText('Would you like to set another task?')
		self.repeat_text.setWordWrap(True)
		self.repeat_text.setStyleSheet("font-size: 32px")
		self.no_button.clicked.connect(self.no)
		self.no_button.setStyleSheet("background-color: rgba(16, 123, 227, 90%); selection-background-color: rgba(16, 123, 227, 70%); font-size: 32px")     
		self.no_button.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
		self.yes_button.clicked.connect(self.yes)
		self.yes_button.setStyleSheet("background-color: rgba(16, 123, 227, 50%); selection-background-color: rgba(16, 123, 227, 50%); font-size: 32px")
		self.yes_button.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
		image_layout.addWidget(self.r_vid1)
		if self.stereo_vision:
			self.repeat_text.setFixedHeight(TEXT_HEIGHT)
			self.no_button.setFixedHeight(TEXT_HEIGHT)
			self.yes_button.setFixedHeight(TEXT_HEIGHT)
			image_layout.addWidget(self.r_vid2)
		else:
			self.repeat_text.setFixedWidth(TEXT_WIDTH)
			self.no_button.setFixedWidth(TEXT_WIDTH)
			self.yes_button.setFixedWidth(TEXT_WIDTH)
		text_layout.addWidget(self.repeat_text)
		text_layout.addWidget(self.yes_button)
		text_layout.addWidget(self.no_button)
		layout.addLayout(image_layout)
		layout.addLayout(text_layout)        
		return layout
#############################################################################################################
# SELECT WIDGET 
#############################################################################################################
	def reset_points(self):
		self.index = 0
		self.static_count_cam1 = 0
		self.dynamic_count_cam1 = 0
		self.temp_cam1 = []
		if self.stereo_vision:
			self.static_count_cam2 = 0
			self.dynamic_count_cam2 = 0 
			self.temp_cam2 = []
		self.static = [False, False]
		self.dynamic = [False, False]
		self.next_direction()

	def next_direction(self):
		if self.static_count_cam1 == self.static_total:
			self.static[0] = True
		if self.dynamic_count_cam1 == self.dynamic_total:
			self.dynamic[0] = True
		if self.stereo_vision:
			if self.static_count_cam2 == self.static_total:
				self.static[1] = True
			if self.dynamic_count_cam2 == self.dynamic_total:
				self.dynamic[1] = True
		if self.index == 0:
			self.directions.setText(self.directions_list[self.current_id][self.index])
			self.index += 1
		elif self.stereo_vision:
			if not self.static[0] and not self.static[1]:
				self.directions.setStyleSheet("font-size: 32px; font: bold")
			elif self.static[0] and not self.static[1]:
				self.directions.setText('Please set a corresponding static point in image 2')
			elif not self.static[0] and self.static[1]:
				self.directions.setText('Please set a corresponding static point in image 1')
			elif self.index == 1 and self.static[0] and self.static[1]:
				self.directions.setStyleSheet("font-size: 32px")
				self.directions.setText(self.directions_list[self.current_id][self.index])
				self.index += 1
			elif self.index > 1:
				if not self.dynamic[0] and not self.dynamic[1]:
					self.directions.setStyleSheet("font-size: 32px; font: bold")
				elif self.dynamic[0] and not self.dynamic[1]:
					self.directions.setText('Please set a corresponding dynamic point in image 2')
				elif not self.dynamic[0] and self.dynamic[1]:
					self.directions.setText('Please set a corresponding dynamic point in image 1')
				elif self.dynamic[0] and self.dynamic[1]:
					self.update_current_widget('repeat')
				else:
					print 'User Interface: next direction --edge case 1'
			else:
				print 'User Interface: next direction -- edge case 2'
		else:
			if not self.static[0]:
				self.directions.setStyleSheet("font-size: 32px; font: bold")
			elif self.index == 1 and self.static[0]:
				self.directions.setStyleSheet("font-size: 32px")
				self.directions.setText(self.directions_list[self.current_id][self.index])
				self.index += 1
			elif self.index > 1:
				if not self.dynamic[0]:
					self.directions.setStyleSheet("font-size: 32px; font: bold")
				else:
					self.update_current_widget('repeat')
			else:
				print 'User Interface: next direction -- edge case 3'            

	def select_layout(self):
		if self.stereo_vision:
			layout = QVBoxLayout()
			text_layout = QHBoxLayout()
			image_layout = QHBoxLayout()
		else:
			layout = QHBoxLayout()
			text_layout = QVBoxLayout()
			image_layout = QVBoxLayout()
		reset_button = QPushButton('reset points')
		reset_button.clicked.connect(self.reset_points)
		reset_button.setStyleSheet("background-color: rgba(16, 123, 227, 90%); selection-background-color: rgba(16, 123, 227, 50%); font-size: 32px")
		self.done_button.clicked.connect(self.next_direction)
		self.done_button.setStyleSheet("background-color: rgba(16, 123, 227, 50%); selection-background-color: rgba(16, 123, 227, 70%); font-size: 32px")       
		self.directions.setWordWrap(True)
		self.directions.setStyleSheet("font-size: 32px")
		self.directions.setAlignment(Qt.AlignCenter)
		self.s_vid1.mousePressEvent = self.mousePressEventCam1
		image_layout.addWidget(self.s_vid1)
		if self.stereo_vision:
			self.directions.setFixedHeight(TEXT_HEIGHT)
			self.done_button.setFixedHeight(TEXT_HEIGHT)
			reset_button.setFixedHeight(TEXT_HEIGHT)			
			self.s_vid2.mousePressEvent = self.mousePressEventCam2
			image_layout.addWidget(self.s_vid2)
		else:
			self.directions.setFixedWidth(TEXT_WIDTH)
			self.done_button.setFixedWidth(TEXT_WIDTH)
			reset_button.setFixedWidth(TEXT_WIDTH)
			reset_button.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
			self.done_button.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)

		text_layout.addWidget(self.directions)
		text_layout.addWidget(self.done_button)
		text_layout.addWidget(reset_button)
		layout.addLayout(image_layout)
		layout.addLayout(text_layout)        
		return layout
#############################################################################################################
# MENU WIDGET 
#############################################################################################################
	def point_to_point(self):
		if self.p2p_button.text() == 'point to point':
			self.reset_button_labels()
			self.p2p_button.setText('confirm')
		elif self.p2p_button.text() == 'confirm':
			self.current_id = 0
			self.static_total = 1
			self.dynamic_total = 1
			self.update_current_widget('select')

	def point_to_line(self):
		if self.p2l_button.text() == 'point to line':
			self.reset_button_labels()
			self.p2l_button.setText('confirm')
		elif self.p2l_button.text() == 'confirm':
			self.current_id = 1
			self.static_total = 2
			self.dynamic_total = 1
			self.update_current_widget('select')

	def line_to_line(self):
		if self.l2l_button.text() == 'line to line':
			self.reset_button_labels()
			self.l2l_button.setText('confirm')
		elif self.l2l_button.text() == 'confirm':
			self.current_id = 2
			self.static_total = 2
			self.dynamic_total = 2
			self.update_current_widget('select')

	def parallel_lines(self):
		if self.par_button.text() == 'parallel lines':
			self.reset_button_labels()
			self.par_button.setText('confirm')
		elif self.par_button.text() == 'confirm':
			self.current_id = 3
			self.static_total = 2
			self.dynamic_total = 2
			self.update_current_widget('select')

	def conic(self):
		if self.con_button.text() == 'conic':
			self.reset_button_labels()
			self.con_button.setText('confirm')
		elif self.con_button.text() == 'confirm':
			self.current_id = 4
			self.static_total = 5
			self.dynamic_total = 3
			self.update_current_widget('select')

	def reset_button_labels(self):
		self.p2p_button.setText('point to point')
		self.p2l_button.setText('point to line')
		self.l2l_button.setText('line to line')
		self.par_button.setText('parallel lines')
		self.con_button.setText('conic')

	def menu_layout(self):
		if self.stereo_vision:
			layout = QVBoxLayout()
			button_layout = QHBoxLayout()
			image_layout = QHBoxLayout()
		else:
			layout = QHBoxLayout()
			button_layout = QVBoxLayout()
			image_layout = QVBoxLayout()        
		greeting = QLabel()
		greeting.setText('Please select a task: ')
		greeting.setWordWrap(True)
		greeting.setStyleSheet("font-size: 32px")
		greeting.setAlignment(Qt.AlignCenter)
		# greeting.setFixedWidth(TEXT_WIDTH)
		self.p2p_button.clicked.connect(self.point_to_point)
		self.p2p_button.setStyleSheet("background-color: rgba(16, 123, 227, 90%); selection-background-color: rgba(16, 123, 227, 80%); font-size: 32px")
		self.p2p_button.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
		self.p2l_button.clicked.connect(self.point_to_line)
		self.p2l_button.setStyleSheet("background-color: rgba(16, 123, 227, 80%); selection-background-color: rgba(16, 123, 227, 70%); font-size: 32px")
		self.p2l_button.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
		self.l2l_button.clicked.connect(self.line_to_line)
		self.l2l_button.setStyleSheet("background-color: rgba(16, 123, 227, 70%); selection-background-color: rgba(16, 123, 227, 60%); font-size: 32px")
		self.l2l_button.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
		self.par_button.clicked.connect(self.parallel_lines)
		self.par_button.setStyleSheet("background-color: rgba(16, 123, 227, 60%); selection-background-color: rgba(16, 123, 227, 50%); font-size: 32px")
		self.par_button.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
		self.con_button.clicked.connect(self.conic)
		self.con_button.setStyleSheet("background-color: rgba(16, 123, 227, 50%); selection-background-color: rgba(16, 123, 227, 40%); font-size: 32px")
		self.con_button.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
		self.m_vid1.setAlignment(Qt.AlignCenter)
		self.m_vid1.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
		image_layout.addWidget(self.m_vid1)
		if self.stereo_vision:
			self.p2p_button.setFixedHeight(TEXT_HEIGHT)
			self.p2l_button.setFixedHeight(TEXT_HEIGHT)
			self.l2l_button.setFixedHeight(TEXT_HEIGHT)
			self.par_button.setFixedHeight(TEXT_HEIGHT)
			self.con_button.setFixedHeight(TEXT_HEIGHT)
			self.m_vid2.setAlignment(Qt.AlignCenter)
			self.m_vid2.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
			image_layout.addWidget(self.m_vid2)
		else:
			self.p2p_button.setFixedWidth(TEXT_WIDTH)
			self.p2l_button.setFixedWidth(TEXT_WIDTH)
			self.l2l_button.setFixedWidth(TEXT_WIDTH)
			self.par_button.setFixedWidth(TEXT_WIDTH)
			self.con_button.setFixedWidth(TEXT_WIDTH)
		button_layout.addWidget(greeting)
		button_layout.addWidget(self.p2p_button)
		button_layout.addWidget(self.p2l_button)
		button_layout.addWidget(self.l2l_button)
		button_layout.addWidget(self.par_button)
		# button_layout.addWidget(self.con_button)
		layout.addLayout(image_layout)
		layout.addLayout(button_layout)        
		return layout

	def initialize_widgets(self):
		self.stacked_widget = QStackedWidget()
		self.menu_widget = QWidget()
		self.select_widget = QWidget()
		self.repeat_widget = QWidget()
		self.vs_widget = QWidget()
		self.reset_widget = QWidget()

		self.menu_widget.setLayout(self.menu_layout())
		self.select_widget.setLayout(self.select_layout())
		self.repeat_widget.setLayout(self.repeat_layout())
		self.vs_widget.setLayout(self.vs_layout())
		self.reset_widget.setLayout(self.reset_layout())

		self.stacked_widget.addWidget(self.menu_widget)
		self.stacked_widget.addWidget(self.select_widget)
		self.stacked_widget.addWidget(self.repeat_widget)
		self.stacked_widget.addWidget(self.vs_widget)
		self.stacked_widget.addWidget(self.reset_widget)
		self.main_layout.addWidget(self.stacked_widget)
#############################################################################################################
# WIDGET CHANGE METHODS
#############################################################################################################
	def reset_all(self, all_tasks, current_widget):
		self.draw = True
		self.reset_button_labels()
		self.reset_repeat_labels()
		if all_tasks:
			self.draw_vs = False
			self.tasks_cam1 = []
			self.total_num_tasks = 0
			self.task_ids = []
			self.static_cam1 = []
			self.dynamic_cam1 = []
			self.pub_reset.publish(True)
			self.pub_calculate.publish(False)
		self.temp_cam1 = []
		self.static_total = 0
		self.dynamic_total = 0
		self.static_count_cam1 = 0       
		self.dynamic_count_cam1 = 0 
		self.static = [False, False]
		self.dynamic = [False, False]
		if self.stereo_vision:
			if all_tasks:
				self.tasks_cam2 = []
				self.static_cam2 = []
				self.dynamic_cam2 = []
			self.temp_cam2 = []
			self.static_count_cam2 = 0       
			self.dynamic_count_cam2 = 0      
		self.current_id = None
		self.index = 0
		self.publish_new_tasks_now = False
		self.update_current_widget(current_widget)

	def activate_widget(self, index):
		i = self.stacked_widget.currentIndex()
		if i != index:
			self.stacked_widget.setCurrentIndex(index)

	def change_widget(self):
		if self.current_widget == 'menu':
			self.activate_widget(0)
		elif self.current_widget == 'select':
			self.pub_reset.publish(False)
			self.activate_widget(1)
			self.next_direction()
		elif self.current_widget == 'repeat':
			self.activate_widget(2)
		elif self.current_widget == 'vs':
			self.activate_widget(3)
			self.publish_new_tasks_now = True
		elif self.current_widget == 'reset':
			self.activate_widget(4)
		else:
			print 'User Interface: Change widget error'
		self.draw = True

	def update_current_widget(self, current_widget):
		self.draw = False
		self.current_widget = current_widget
		self.index = 0
		self.reset_button_labels()
		self.change_widget()
#############################################################################################################
# PUBLISHING METHODS
#############################################################################################################
	def remap(self, X, mtf_output):
		# f(x) = (x - input_start) / (input_end - input_start) * (output_end - output_start) + output_start
		if not mtf_output:
			x = int(float(X[0]) / self.x_input * 640)
			y = int(float(X[1]) / self.y_input * 480)
		else:
			x = float(X[0]) / 640 * self.x_input
			y = float(X[1]) / 480 * self.y_input
		return [x, y]		

	def format_string(self, my_list):
		msg = ''
		for i in range(len(my_list)):
			inner_list = my_list[i]
			for j in range(len(inner_list)):
				c = inner_list[j]
				# c = self.remap(c, False)
				msg = msg + str(c[0]) + ' ' + str(c[1]) + ' '
			msg = msg + '; '
		return msg

	def split_static_dynamic(self, my_list):
		S = []
		D = []
		for i in range(len(self.task_ids)):
			coords = my_list[i]
			s = []
			d = []
			if self.task_ids[i] == 0:
				# p2p -- 1 static, 1 dynamic
				s.append(coords[0])
				d.append(coords[1])
			elif self.task_ids[i] == 1:
				# p2l -- 2 static, 1 dynamic
				s.append(coords[0])
				s.append(coords[1])
				d.append(coords[2])
			elif self.task_ids[i] == 2:
				# l2l -- 2 static, 2 dynamic  
				s.append(coords[0])
				s.append(coords[1])
				d.append(coords[2])   
				d.append(coords[3])   
			elif self.task_ids[i] == 3:
				# par -- 2 static, 2 dynamic
				s.append(coords[0])
				s.append(coords[1])
				d.append(coords[2])   
				d.append(coords[3]) 
			elif self.task_ids[i] == 4:
				# con -- 5 static, 3 dynamic
				s.append(coords[0])
				s.append(coords[1])
				s.append(coords[2])
				s.append(coords[3])
				s.append(coords[4])
				d.append(coords[5])   
				d.append(coords[6])
			S.append(s)
			D.append(d)
		return S, D

	def merge_static_dynamic(self, static_list, dynamic_list):
		M = []
		ind = 0
		for i in range(len(self.task_ids)):
			ID = self.task_ids[i]
			s = static_list[i]
			m = []
			if ID == 0:
				m.append(s[0])
				m.append(dynamic_list[ind])
				ind += 1
			elif ID == 1:
				m.append(s[0])
				m.append(s[1])
				m.append(dynamic_list[ind])
				ind += 1
			elif ID == 2 or ID == 3:
				m.append(s[0])
				m.append(s[1])
				m.append(dynamic_list[ind])
				m.append(dynamic_list[ind + 1])
				ind += 2
			elif ID == 4:
				m.append(s[0])
				m.append(s[1])
				m.append(s[2])
				m.append(s[3])
				m.append(s[4])
				m.append(dynamic_list[ind])   
				m.append(dynamic_list[ind + 1])
				ind += 2
			M.append(m)
		return M

	def pub_check(self):
		# self.pub_stereo.publish(self.stereo_vision)
		if self.publish_new_tasks_now:
			self.static_cam1, D = self.split_static_dynamic(self.tasks_cam1)
			t1 = self.format_string(D)
			if self.stereo_vision:
				self.static_cam2, self.dynamic_cam2 = self.split_static_dynamic(self.tasks_cam2)
				t2 = self.format_string(self.dynamic_cam2)
				self.pub_tasks_cam2.publish(t2)
			self.pub_tasks_cam1.publish(t1)
			ids = ''
			for i in self.task_ids:
				ids = ids + ' ' + str(i) + ' '
			self.pub_ids.publish(ids)
			self.pub_stereo.publish(self.stereo_vision)
			self.pub_calculate.publish(True)
			self.draw_vs = True
			self.publish_new_tasks_now = False
		if self.draw_vs:
			s1 = self.format_string(self.merged_list)
			if self.stereo_vision:
				s2 = self.format_string(self.merged_list2)
				self.pub_centers_cam2.publish(s2)
			self.pub_centers_cam1.publish(s1)
#############################################################################################################
# DEBUGGING METHODS
#############################################################################################################
	def print_debugging(self):
		print '********************************************************\nDEBUGGING INFORMATION: '
		# print 'video label size', self.s_vid1.size()
		# print '\nstereo vision: ',  self.stereo_vision
		print 'total_num_tasks: ', self.total_num_tasks
		print 'task_ids: ', self.task_ids
		# print 'current_widget: ', self.current_widget
		# print 'publish now: ', self.publish_new_tasks_now
		print '********************************************************\ntasks_cam1: ', self.tasks_cam1
		print 'temp_cam1: ', self.temp_cam1
		# print 'cam1 static num: ', self.static_count_cam1
		# print 'cam1 dynamic num: ', self.dynamic_count_cam1
		print 'static_cam1: ', self.static_cam1
		print 'dynamic_cam1: ', self.dynamic_cam1
		# print 'static: ', self.static
		# print 'dynamic: ', self.dynamic 
		if self.stereo_vision:   
			print '********************************************************\ntasks_cam2: ', self.tasks_cam2
			print 'temp_cam2: ', self.temp_cam2
			print 'static_cam2: ', self.static_cam2
			print 'dynamic_cam2: ', self.dynamic_cam2
		# print '\ncurrent id: ', self.current_id
		# print 'index: ', self.index
		# print '\nrepeat text:', self.repeat_text.text()
		# print 'yes button text:', self.yes_button.text()
		# print 'no button text:', self.no_button.text()
		# print 'input x: ', self.x_input
		# print 'input y: ', self.y_input
		print '********************************************************\n'

#############################################################################################################
# CALLBACK METHODS
#############################################################################################################
	def callback_reset(self, data):
		reset = data.data
		if reset:
			print 'User Interface: Tracker lost, resetting.'
			self.update_current_widget('reset')


	def callback_cam1(self, data):
		if self.current_widget != 'reset':
			self.pixmap = self.convert_compressed_img(data)
			if self.draw:
				self.painter1(self.pixmap)
			if self.current_widget == 'menu':
				self.m_vid1.setPixmap(self.pixmap)
			elif self.current_widget == 'select':
				self.s_vid1.setPixmap(self.pixmap)
			elif self.current_widget == 'repeat':
				self.r_vid1.setPixmap(self.pixmap)        
			elif self.current_widget == 'vs':
				self.v_vid1.setPixmap(self.pixmap)
			else:
				print 'User Interface: image callback error -- invalid current_widget'

	def callback_cam2(self, data):
		if self.current_widget != 'reset':
			pixmap2 = self.convert_compressed_img(data)
			if self.draw:
				self.painter2(pixmap2)
			if self.current_widget == 'menu':
				self.m_vid2.setPixmap(pixmap2)
			elif self.current_widget == 'select':
				self.s_vid2.setPixmap(pixmap2)
			elif self.current_widget == 'repeat':
				self.r_vid2.setPixmap(pixmap2)
			elif self.current_widget == 'vs':
				self.v_vid2.setPixmap(pixmap2)
			else:
				print 'User Interface: image callback error -- invalid current_widget'

	def callback_mtf1(self, data):
		if self.draw_vs:
			p = data.data.split()
			self.dynamic_cam1 = []
			for a in range(0, len(p), 2):
				self.dynamic_cam1.append(self.remap([p[a], p[a + 1]], True))
			self.merged_list = self.merge_static_dynamic(self.static_cam1, self.dynamic_cam1)

	def callback_mtf2(self, data):
		if self.draw_vs:
			p2 = data.data.split()
			self.dynamic_cam2 = []
			for a2 in range(0, len(p2), 2):
				self.dynamic_cam2.append(self.remap([p2[a2], p2[a2 + 1]], True))
			self.merged_list2 = self.merge_static_dynamic(self.static_cam2, self.dynamic_cam2)

#############################################################################################################
# IMAGE METHODS
#############################################################################################################
	def painter1(self, pixmap):
		painter = QPainter(pixmap)
		if self.current_id != None:
			self.check_ID(0, painter, self.current_id, self.temp_cam1)
		for i in range(1, len(self.task_ids) + 1):
			ID = self.task_ids[i - 1]
			if not self.draw_vs:
				self.check_ID(i, painter, ID, self.tasks_cam1[i - 1])
			else:
				if (i - 1) < len(self.merged_list):
					self.check_ID(i, painter, ID, self.merged_list[i - 1])
					# self.paint_poly(i, painter, self.corners[i - 1])

	def check_ID(self, index, painter, ID, point_list):
		if ID == 0:
			self.paint_p2p(index, painter, point_list)
		elif ID == 1:
			self.paint_p2l(index, painter, point_list)
		elif ID == 2:
			self.paint_l2l(index, painter, point_list)     
		elif ID == 3:
			self.paint_par(index, painter, point_list)
		elif ID == 4:
			self.paint_con(index, painter, point_list)

	def painter2(self, pixmap2):
		painter2 = QPainter(pixmap2)
		if self.current_id != None:
			self.check_ID(0, painter2, self.current_id, self.temp_cam2)
		for i in range(1, len(self.task_ids) + 1):
			ID = self.task_ids[i - 1]
			if not self.draw_vs:
				self.check_ID(i, painter2, ID, self.tasks_cam2[i - 1])
			else:
				if (i - 1) < len(self.merged_list2):
					self.check_ID(i, painter2, ID, self.merged_list2[i - 1])
					# self.paint_poly(i, painter2, self.corners2[i - 1])

	def paint_p2p(self, index, painter, pt_list):
		painter.setPen(self.pens[index])
		for p in pt_list:
			painter.drawPoint(p[0], p[1])

	def paint_p2l(self, index, painter, pt_list):
		for p in pt_list:
			painter.setPen(self.pens[index])
			painter.drawPoint(p[0], p[1])
		if len(pt_list) == 3:
			painter.setPen(self.line_pens[index])
			painter.drawLine(pt_list[0][0], pt_list[0][1], pt_list[1][0], pt_list[1][1])

	def paint_l2l(self, index, painter, pt_list):
		for p in pt_list:
			painter.setPen(self.pens[index])
			painter.drawPoint(p[0], p[1])
		if len(pt_list) == 4:
			painter.setPen(self.line_pens[index])
			painter.drawLine(pt_list[0][0], pt_list[0][1], pt_list[1][0], pt_list[1][1])
			painter.drawLine(pt_list[2][0], pt_list[2][1], pt_list[3][0], pt_list[3][1])

	def paint_par(self, index, painter, pt_list):
		for p in pt_list:
			painter.setPen(self.pens[index])
			painter.drawPoint(p[0], p[1])
		if len(pt_list) == 4:
			painter.setPen(self.line_pens[index])
			painter.drawLine(pt_list[0][0], pt_list[0][1], pt_list[1][0], pt_list[1][1])
			painter.drawLine(pt_list[2][0], pt_list[2][1], pt_list[3][0], pt_list[3][1])        

	def paint_con(self, index, painter, pt_list):
		for p in pt_list:
			painter.setPen(self.pens[index])
			painter.drawPoint(p[0], p[1])
		if len(pt_list) == 5:
			return

	def paint_poly(self, index, painter, c):
		painter.setPen(self.line_pens[index])
		for i in range(0, len(c), 4):
			if (i + 3) < len(c):
				painter.drawPolygon(QPointF(c[i][0], c[i][1]), QPointF(c[i+1][0], c[i+1][1]), QPointF(c[i+2][0], c[i+2][1]), QPointF(c[i+3][0], c[i+3][1]))

	def convert_img(self, data):
		try:
			frame = self.bridge.imgmsg_to_cv2(data, "rgb8")
		except CvBridgeError as e:
			print(e)
		image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
		pixmap = QPixmap.fromImage(image)
		# pixmap = pixmap.scaledToHeight(self.s_vid1.size().height())   
		return pixmap

	def convert_compressed_img(self, data):
		np_arr = np.fromstring(data.data, np.uint8)
		frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
		frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
		image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
		pixmap = QPixmap.fromImage(image)
		# pixmap = pixmap.scaledToHeight(self.s_vid1.size().height())
		return pixmap
#############################################################################################################
# EVENT METHODS
#############################################################################################################
	def mousePressEventCam1(self, event):
		if not self.static[0] and self.static_count_cam1 < self.static_total:
			pos = self.s_vid1.mapFromGlobal(event.globalPos())
			print "Target cam1: ", pos.x(), ", ", pos.y()
			self.temp_cam1.append([pos.x(), pos.y()])
			self.static_count_cam1 += 1
		elif self.static[0] and not self.dynamic[0] and self.dynamic_count_cam1 < self.dynamic_total:
			if self.stereo_vision and not self.static[1]:
				return
			pos = self.s_vid1.mapFromGlobal(event.globalPos())
			print "Robot cam1: ", pos.x(), ", ", pos.y()
			self.temp_cam1.append([pos.x(), pos.y()])
			self.dynamic_count_cam1 += 1

	def mousePressEventCam2(self, event):
		if not self.static[1] and self.static_count_cam2 < self.static_total:
			pos = self.s_vid2.mapFromGlobal(event.globalPos())
			print "Target cam2: ", pos.x(), ", ", pos.y()
			self.temp_cam2.append([pos.x(), pos.y()])
			self.static_count_cam2 += 1
		elif self.static[1] and not self.dynamic[1] and self.dynamic_count_cam2 < self.dynamic_total:
			if self.stereo_vision and not self.static[0]:
				return
			pos = self.s_vid2.mapFromGlobal(event.globalPos())
			print "Robot cam2: ", pos.x(), ", ", pos.y()
			self.temp_cam2.append([pos.x(), pos.y()])
			self.dynamic_count_cam2 += 1

	def resizeEvent(self, QResizeEvent):
		self.x_input = self.s_vid1.size().width()
		self.y_input = self.s_vid1.size().height()
		if self.stereo_vision:
			self.repeat_text.setFixedWidth(self.size().width() / 3)
			self.directions.setFixedWidth(self.size().width() / 3)

	def keyPressEvent(self, QKeyEvent):
		key = QKeyEvent.key()
		if int(key) == 82:
			self.reset_all(True, 'menu')
		elif int(key) == 68:
			self.print_debugging()
		elif int(key) == 81:
			sys.exit()

	def save_settings(self, plugin_settings, instance_settings):
		pass

	def restore_settings(self, plugin_settings, instance_settings):
		pass