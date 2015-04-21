#!/usr/bin/env python

# /*********************************************************************
# * Software License Agreement (BSD License)
# *
# * Copyright 2014, RSAIT research group, University of Basque Country
# * All rights reserved.
# *
# * Redistribution and use in source and binary forms, with or without
# * modification, are permitted provided that the following conditions
# * are met:
# *
# *  * Redistributions of source code must retain the above copyright
# *    notice, this list of conditions and the following disclaimer.
# *  * Redistributions in binary form must reproduce the above
# *    copyright notice, this list of conditions and the following
# *    disclaimer in the documentation and/or other materials provided
# *    with the distribution.
# *  * Neither the name of the University of Freiburg nor the names of its
# *    contributors may be used to endorse or promote products derived
# *    from this software without specific prior written permission.
# *
# * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# *********************************************************************/


import roslib;
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32

import shlex,subprocess,os

def speech():
	global file_path
	global audio_rec_path
        global speech_num
        global log_file

	pubs = rospy.Publisher('/google_speech/eu/text', String,queue_size=1)
	pubc = rospy.Publisher('/google_speech/eu/confidence', Float32, queue_size=1)
        	
	os.system('sox -e signed-integer -b 16 -c 1 -r 16000 -t alsa default ' + file_path + 'audio/recording.wav silence 1 0.1 0.5% 1 1.0 1%')

        start = rospy.get_rostime()
        rospy.loginfo("Start time %i", start.secs)

	args2 = shlex.split('sh ' + file_path + 'script/curl.sh ' + audio_rec_path)	
	output,error = subprocess.Popen(args2,stdout = subprocess.PIPE, stderr= subprocess.PIPE).communicate()
	
	a = output.split('\n')
	google_result = a[1]
	try:
		a = google_result.split('":"')
		google_text = a[1].split('","')
		text = google_text[0]
		#print text
		b = google_text[1].split('confidence":')
		b = b[1].split('}],"final"')
		confidence = b[0]
		#print confidence
                
                # If google understand something
		if text != " ":
			data = text
			if confidence < 0.15:
				data = "répétition"
			print String(data), confidence

                        # Save in file
                        speech_num = speech_num + 1
		# If Google not understand
                else:
			data = "répétition"
			print String(data)
                
		pubs.publish(String(data))
                print "Niveau de confiance", confidence
		pubc.publish(float(confidence))
                end = rospy.get_rostime()
                rospy.loginfo("End time %i", end.secs)
                time = (end.secs - start.secs)
                rospy.loginfo("Elapsed time %i", time)
                log_file = open(file_path + "script/log_file.txt", "a")
                log_file.write(str(speech_num) + "\t" + str(start.secs) + "\t" + str(data) + "\t" + str(confidence) + "\t" + str(time) + "\t")
                log_file.close()
	except Exception: 
		data = "répétition"
		#print "Exception", String(data)
		#pubs.publish(String(data))
               # confidence = -1
		#pubc.publish(Float(confidence))
                	
			
if __name__ == '__main__':

        # Initializations
	global file_path
	global audio_rec_path
        global speech_num
        global log_file
        file_path = String("")
    	file_path = rospy.get_param('file_path')
    	audio_rec_path = rospy.get_param('audio_rec_path')
        speech_num = 0

        log_file = open(file_path + "script/log_file.txt", "w")
        log_file.write("Number\t" + "Start Time\t" + "Understood\t" + "Confidence\t" + "GoogleTime\t" + "Command\t" + "ExecuteTime" + "\n")
        #file.close()

	rospy.init_node('gspeech')

	while not rospy.is_shutdown():
		try:
			speech()
		except KeyboardInterrupt:
			sys.exit(1)
		except rospy.ROSInterruptException: 
			pass
	#log_file.close()
        exit(0)
  


	
	
   
