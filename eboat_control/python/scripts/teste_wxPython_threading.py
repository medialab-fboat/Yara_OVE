#!/home/eduardo/miniconda3/envs/ctrgui/bin/python

import numpy as np
import time
from threading import *
import wx
import rospy
from std_msgs.msg import Float32MultiArray


# Button definitions
ID_START = wx.NewId()
ID_STOP = wx.NewId()

# Define notification event for thread completion
EVT_RESULT_ID = wx.NewId()

def EVT_RESULT(win, func):
    """Define Result Event."""
    win.Connect(-1, -1, EVT_RESULT_ID, func)

class ResultEvent(wx.PyEvent):
    """Simple event to carry arbitrary result data."""
    def __init__(self, data):
        """Init Result Event."""
        wx.PyEvent.__init__(self)
        self.SetEventType(EVT_RESULT_ID)
        self.data = data

# Thread class that executes processing
class WorkerThread(Thread):
    """Worker Thread Class."""
    def __init__(self, notify_window):
        """Init Worker Thread Class."""
        Thread.__init__(self)
        self._notify_window = notify_window
        self._want_abort = 0

        # This starts the thread running on creation, but you could
        # also make the GUI thread responsible for calling this
        self.start()

        # rospy.init_node('listener', anonymous=True)

    def getObservations(self):
        obsData = None
        count = 0
        while obsData is None:
            try:
                obsData = rospy.wait_for_message('/eboat/mission_control/observations', Float32MultiArray,
                                                 timeout=20).data
            except:
                pass
            print(obsData)
            if count > 10:
                obsData = 0
            else:
                count += 1
            # --> obsData = [distance, trajectory angle, linear velocity, aparent wind speed, aparent wind angle, boom angle, rudder angle, eletric propultion speed, roll angle]
            #               [   0    ,        1        ,       2        ,         3         ,         4         ,     5     ,      6      ,            7            ,     8     ]

        return np.array(obsData, dtype=float)

    def callback(self, data):
        wx.PostEvent(self._notify_window, ResultEvent(data.data))

    def run(self):
        while not self._want_abort:
            print(self._want_abort)
            obsData = None
            count = 0
            while obsData is None:
                try:
                    obsData = rospy.wait_for_message('/eboat/mission_control/observations', Float32MultiArray,timeout=20).data
                except:
                    pass
                if count > 1000:
                    wx.PostEvent(self._notify_window, ResultEvent("Error acquiring data from ROS topic!"))
                else:
                    count += 1
            # --> obsData = [distance, trajectory angle, linear velocity, aparent wind speed, aparent wind angle, boom angle, rudder angle, eletric propultion speed, roll angle]
            #               [   0    ,        1        ,       2        ,         3         ,         4         ,     5     ,      6      ,            7            ,     8     ]
            wx.PostEvent(self._notify_window, ResultEvent(obsData[0]))
        # """Run Worker Thread."""
        # # This is the code executing in the new thread. Simulation of
        # # a long process (well, 10s here) as a simple loop - you will
        # # need to structure your processing so that you periodically
        # # peek at the abort variable
        # for i in range(10):
        #     time.sleep(1)
        #     wx.PostEvent(self._notify_window, ResultEvent(time.time()))
        #     if self._want_abort:
        #         # Use a result of None to acknowledge the abort (of
        #         # course you can use whatever you'd like or even
        #         # a separate event type)
        #         wx.PostEvent(self._notify_window, ResultEvent(None))
        #         return
        # # Here's where the result would be returned (this is an
        # # example fixed result of the number 10, but it could be
        # # any Python object)
        # wx.PostEvent(self._notify_window, ResultEvent(10))
        # self.getObservations()
        # wx.PostEvent(self._notify_window, ResultEvent("sucesso!"))
        # rospy.Subscriber("/eboat/mission_control/observations", Float32MultiArray, self.callback)
        # rospy.spin()


    def abort(self):
        """abort worker thread."""
        # Method for use by main thread to signal an abort
        self._want_abort = 1

# GUI Frame class that spins off the worker thread
class MainFrame(wx.Frame):
    """Class MainFrame."""
    def __init__(self, parent, id):
        """Create the MainFrame."""
        wx.Frame.__init__(self, parent, id, 'Thread Test')

        # Dumb sample frame with two buttons
        wx.Button(self, ID_START, 'Start', pos=(0,0))
        wx.Button(self, ID_STOP, 'Stop', pos=(0,50))
        self.status = wx.StaticText(self, -1, '', pos=(0,100))

        self.Bind(wx.EVT_BUTTON, self.OnStart, id=ID_START)
        self.Bind(wx.EVT_BUTTON, self.OnStop, id=ID_STOP)

        # Set up event handler for any worker thread results
        EVT_RESULT(self,self.OnResult)

        # And indicate we don't have a worker thread yet
        self.worker = None

        rospy.init_node('listener', anonymous=True)

    def OnStart(self, event):
        """Start Computation."""
        # Trigger the worker thread unless it's already busy
        if not self.worker:
            self.status.SetLabel('Starting computation')
            self.worker = WorkerThread(self)

    def OnStop(self, event):
        """Stop Computation."""
        # Flag the worker thread to stop if running
        if self.worker:
            self.status.SetLabel('Trying to abort computation')
            self.worker.abort()

    def OnResult(self, event):
        """Show Result status."""
        if event.data is None:
            # Thread aborted (using our convention of None return)
            self.status.SetLabel('Computation aborted')
        else:
            # Process results here
            self.status.SetLabel('Computation Result: {}'.format(event.data))
        # In either event, the worker is done
        self.worker = None

class MainApp(wx.App):
    """Class Main App."""
    def OnInit(self):
        """Init Main App."""
        self.frame = MainFrame(None, -1)
        self.frame.Show(True)
        self.SetTopWindow(self.frame)
        return True

if __name__ == '__main__':
    app = MainApp(0)
    app.MainLoop()