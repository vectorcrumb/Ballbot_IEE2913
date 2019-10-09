from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from direct.actor.Actor import Actor
import numpy as np

class MyApp(ShowBase):

    def __init__(self):
        ShowBase.__init__(self)
        # Load environment model
        self.scene = self.loader.loadModel("models/environment")
        # Reparent model to render
        self.scene.reparentTo(self.render)
        # Scale and position model
        self.scene.setScale(0.25, 0.25, 0.25)
        self.scene.setPos(-8, 42, 0)

        # Add spinCameraTask to task manager to execute
        self.taskMgr.add(self.spinCameraTask, "SpinCameraTask")

        # Load and transform panda actor
        self.pandaActor = Actor("models/panda-model", {"walk": "models/panda-walk4"})
        self.pandaActor.setScale(0.005, 0.005, 0.005)
        self.pandaActor.reparentTo(self.render)

        # Loop animation
        self.pandaActor.loop("walk")

    def spinCameraTask(self, task):
        angleDegs = task.time * 6.0
        angleRads = angleDegs * (np.pi / 180.0)
        self.camera.setPos(20*np.sin(angleRads), -20.0 * np.cos(angleRads), 3)
        self.camera.setHpr(angleDegs, 0, 0)
        return Task.cont

app = MyApp()
app.run()