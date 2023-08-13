import numpy as np 
from multiprocessing import Process, Queue
import pypangolin as pango 
from OpenGL.GL import *

class Point(object): 
    # representation of 3D point in a world 
    #each point should be observed in multiple frames (Frame object)
    def __init__(self, location, map):
        self.frames = [] 
        self.pointCoordinates = location 
        self.descriptor_indexes = [] #indexes of frames tha point was observed in 
        self.id = len(map.points)
        
        map.points.append(self) #add point to map 
        #
        
    def addObservation(self, frame, idx): 
        self.frames.append(frame)
        self.descriptor_indexes.append(idx)


class Map(object): 
    def __init__(self):
        self.frames = [] 
        self.points = [] 
        self.tread_queue = Queue() 
        self.visualizer = Process(target=self.visualization_thread, args=(self.tread_queue))
        
    def display(self):
        pass
    
    def visualization_thread(self, thread_queue): 
        pango.CreateWindowAndBind("Main", 640, 480)
        glEnable(GL_DEPTH_TEST)
        
        pm = pango.ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.1, 1000)
        mv = pango.ModelViewLookAt(-0, 0.5, -3, 0, 0, 0, pango.AxisY)
        s_cam = pango.OpenGlRenderState(pm, mv)
        
        ui_width = 180

        handler = pango.Handler3D(s_cam)
        
        d_cam = (pango.CreateDisplay().SetBounds(
            pango.Attach(0),
            pango.Attach(1),
            pango.Attach.Pix(ui_width),
            pango.Attach(1),
            -640.0 / 480.0).SetHandler(handler))
        
        pango.CreatePanel("ui").SetBounds(pango.Attach(0), pango.Attach(1), pango.Attach(0), pango.Attach.Pix(ui_width))
        
        var_ui = pango.Var("ui")
        var_ui.a_Button = False
        var_ui.a_Toggle = (False, pango.VarMeta(toggle=True))
        var_ui.a_double = (0.0, pango.VarMeta(0, 5))
        var_ui.an_int = (2, pango.VarMeta(0, 5))
        var_ui.a_double_log = (3.0, pango.VarMeta(1, 1e4, logscale=True))
        var_ui.a_checkbox = (False, pango.VarMeta(toggle=True))
        var_ui.an_int_no_input = 2
        var_ui.a_str = "sss"

        ctrl = -96
        pango.RegisterKeyPressCallback(ctrl + ord("a"))
        
        darr = None 
        while not pango.ShouldQuit():
            darr = thread_queue.get(darr is None )
            
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

            if var_ui.a_checkbox:
                var_ui.an_int = var_ui.a_double

            if var_ui.GuiChanged('an_int'):
                var_ui.an_int_no_input = var_ui.an_int

            d_cam.Activate(s_cam)
            pango.glDrawColouredCube()
            pango.FinishFrame()
