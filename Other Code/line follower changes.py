	def process(self, rgb_img, action):
		
		binary = []
		rgb_img = cv.resize(rgb_img, (640, 480))
        
		if self.img_flip == True: rgb_img = cv.flip(rgb_img, 1)
		if action == 32: self.Track_state = 'tracking'
		elif action == ord('i') or action == 105: self.Track_state = "identify"
		elif action == ord('r') or action == 114: self.Reset()
		elif action == ord('q') or action == 113: self.cancel()
		if self.Track_state == 'init':
			cv.namedWindow(self.windows_name, cv.WINDOW_AUTOSIZE)
			self.hsv_range[0] = input("H Min: ")
			self.hsv_range[1] = input("H Max: ")
            self.hsv_range[2] = input("S Min: ")
        x	self.hsv_range[3] = input("S Max: ")
            self.hsv_range[4] = input("V Min: ")
            self.hsv_range[5] = input("V Max: ")
			#cv.setMouseCallback(self.windows_name, self.onMouse, 0)
			if self.select_flags == True:
				#cv.line(rgb_img, self.cols, self.rows, (255, 0, 0), 2)
				##cv.rectangle(rgb_img, self.cols, self.rows, (0, 255, 0), 2)
				#if self.Roi_init[0]!=self.Roi_init[2] and self.Roi_init[1]!=self.Roi_init[3]:
				  #  rgb_img, self.hsv_range = self.color.Roi_hsv(rgb_img, self.Roi_init)
				self.dyn_update = True