#!/home/lmdc/miniconda3/envs/esailor/bin/python

from time import sleep
import wx
import rospy
import numpy as np
from std_msgs.msg import Int16, Float32
from geometry_msgs.msg import Point

class Sailor(wx.Frame):
    def __init__(self):
        super().__init__(parent=None, title='Sailor', size=(450,450))

        pnl = wx.Panel(self)
        main_sizer = wx.BoxSizer(wx.VERTICAL)
        hsizer = wx.BoxSizer()
        vsizer = wx.BoxSizer(wx.VERTICAL)

        trueWindSTR = wx.StaticText(pnl, label="True Wind Vector (x,y,z): ")
        title1 = wx.StaticText(pnl, label="Rudder")
        title2 = wx.StaticText(pnl, label="Sail")
        # title3 = wx.StaticText(pnl, label="Propultion")
        self.trueWindVec = wx.TextCtrl(pnl,
                                       value = "(0,0,0)",
                                       style = wx.TE_PROCESS_ENTER)
        self.rudder = wx.Slider(pnl           ,
                                value    = 0  ,
                                minValue = -60,
                                maxValue = 60 ,
                                style    = wx.SL_HORIZONTAL | wx.SL_VALUE_LABEL)
        self.sail = wx.Slider(pnl           ,
                              value    = 0  ,
                              minValue = 0,
                              maxValue = 90 ,
                              style    = wx.SL_HORIZONTAL | wx.SL_VALUE_LABEL)
        self.prop = wx.RadioBox(pnl,
                                label="Propultion engine",
                                choices=["-5","-4","-3","-2","-1","0","1","2","3","4","5"])
        hsizer.Add(trueWindSTR, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALIGN_LEFT | wx.LEFT, 5)
        hsizer.Add(self.trueWindVec, 0, wx.ALL | wx.LEFT, 5)
        vsizer.Add(hsizer)
        vsizer.AddSpacer(30)
        # vsizer.Add(title1, 0, wx.ALIGN_BOTTOM | wx.ALIGN_LEFT | wx.LEFT, 5)
        vsizer.Add(title1, 0, wx.ALIGN_LEFT | wx.LEFT, 5)
        vsizer.Add(self.rudder, 0, wx.ALL | wx.EXPAND, 5)
        vsizer.AddSpacer(30)
        # vsizer.Add(title2, 0, wx.ALIGN_BOTTOM | wx.ALIGN_LEFT | wx.LEFT, 5)
        vsizer.Add(title2, 0, wx.ALIGN_LEFT | wx.LEFT, 5)
        vsizer.Add(self.sail, 0, wx.ALL | wx.EXPAND, 5)
        vsizer.AddSpacer(30)
        vsizer.Add(self.prop, 0, wx.ALL | wx.LEFT, 10)

        self.prop.SetSelection(5) #--> Set initial position

        main_sizer.Add(vsizer, 0, wx.EXPAND, 10)

        self.trueWindVec.Bind(wx.EVT_TEXT_ENTER, self.SetTrueWind)
        self.sail.Bind(wx.EVT_SLIDER, self.SailHandler)
        self.rudder.Bind(wx.EVT_SLIDER, self.RudderHandler)
        self.prop.Bind(wx.EVT_RADIOBOX, self.PropHandler)
        pnl.SetSizer(main_sizer)
        # wx.BoxSizer.Fit(main_sizer)

        self.Show()

        rospy.init_node('Control_Interface', anonymous=True)
        self.bang_pub = rospy.Publisher('/eboat/control_interface/sail', Float32, queue_size=91)
        self.rang_pub = rospy.Publisher('/eboat/control_interface/rudder', Float32, queue_size=121)
        self.pvel_pub = rospy.Publisher('/eboat/control_interface/propulsion', Int16, queue_size=11)
        self.wind_pub = rospy.Publisher('/eboat/atmosferic_control/wind', Point, queue_size=5)

        self.reward   = 0

    def SetTrueWind(self, event):
        text = self.trueWindVec.GetLineText(0).translate({ord(i): None for i in '()'})
        wvec = np.array(text.split(","), dtype=np.float)
        if not rospy.is_shutdown():
            self.wind_pub.publish(Point(wvec[0], wvec[1], wvec[2]))

    def SailHandler(self, event):
        if not rospy.is_shutdown():
            self.bang_pub.publish(self.sail.GetValue())

    def RudderHandler(self, event):
        if not rospy.is_shutdown():
            self.rang_pub.publish(-self.rudder.GetValue())

    def PropHandler(self, event):
        if not rospy.is_shutdown():
            self.pvel_pub.publish(int(self.prop.GetString(self.prop.GetSelection())))
        # print(self.prop.GetString(self.prop.GetSelection()))

    def getObservations(self):
        obsData = None
        while obsData is None:
            try:
                obsData = rospy.wait_for_message('/eboat/mission_control/observations', Float32MultiArray,
                                                 timeout=20).data
            except:
                pass
            # --> obsData = [distance, trajectory angle, linear velocity, aparent wind speed, aparent wind angle, boom angle, rudder angle, eletric propultion speed, roll angle]
            #               [   0    ,        1        ,       2        ,         3         ,         4         ,     5     ,      6      ,            7            ,     8     ]

            return np.array(obsData, dtype=float)

    def rewardFunction(self, obs):
        #--> A reward is earned by get near the goal
        delta_dist  = self.D0 - obs[0]
        delta_theta = 90.0 - abs(obs[1])
        reward = 0
        if delta_dist > 0:
            reward += 2.0 * delta_dist

        # --> A reward is earned by changing the sailing point toward the goal.
        #     A penalty is earned by changing the sailing point away from the goal.
        if delta_theta > 0:
            reward += delta_theta
        else:
            reward -= 2.0 * delta_theta

        #--> If the eletric propulsion is activated a penalty is earned
        C = 4
        if obs[7] < 0:
            C = 6
        reward -= C * abs(obs[7])

        #--> If the boat heel over (adernar) more than 30o a penalty is earned.
        if obs[8] > 30.0:
            reward -= 100.0 * obs[8]

        #-> If the boat stay still a penalty is earned
        if (abs(delta_dist) < 0.01) & (abs(delta_theta) < 1):
            reward -= 100.0

        return reward

if __name__ == '__main__':
    app = wx.App(False)
    frame = Sailor()
    app.MainLoop()