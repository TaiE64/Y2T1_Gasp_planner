import pybullet as p
import os
import pybullet_data
pathSS = './data/experiments/SS'
path = "./data/experiments/bleach/"
address = "./data/objects/021_bleach_cleanser/poisson/textured.obj"
meshScale = [1.5, 1.5, 1.5]
ObjIniOrientation = p.getQuaternionFromEuler([0, 0, 0]) 
basePos = [0, 0, 0]
inertial = [0, 0, 0]
shift = [0, 0, 0]


class Thing:

    def __init__(self, name, position = [0.0, -0.4, 0], onObject=None):
        """"
        Input: name, initial position (optional), object to put thing on (optional)
            Some standard settings for specific object: plane, table and Object , todo: more

        """
        self.name = name
        self.initPos = position
        self.initOrientation=p.getQuaternionFromEuler([0,0,0])
        print(f"Created a {name}")

        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        if name=="plane":
            self.ID=p.loadURDF(os.path.join(pybullet_data.getDataPath(), 
                     "plane_transparent.urdf"))
            print('Load URDF plane_transparent.urdf')
            print(f"Set ID to {self.ID}")

        elif name=="table":
            self.height = 0.62
            self.initOrientation=[0, 0, 0, 1]
            self.ID=p.loadURDF(os.path.join(pybullet_data.getDataPath(), 
                       "table/table.urdf"), self.initPos, self.initOrientation, 0)
            print('Load URDF table/table.urdf')
            print(f"Set ID to {self.ID}")  
        
        elif name=="Bottle":
            self.objShapeID = p.createVisualShape(shapeType=p.GEOM_MESH, fileName=address, 
                                 rgbaColor=[1, 0, 0, 1], specularColor=[0.4, .4, 0], 
                                 visualFramePosition=shift, meshScale=meshScale)
            self.objCollisionID = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName=address, 
                                        collisionFramePosition=shift, meshScale=meshScale)
            if onObject is not None:
                if onObject.name == "table"  :
                    self.initPos[2] += onObject.getHeight()

            self.ID = p.createMultiBody(baseMass=0.2,
                               baseInertialFramePosition=inertial,
                               baseCollisionShapeIndex=self.objCollisionID,
                               baseVisualShapeIndex=self.objShapeID,
                               basePosition=self.initPos,
                               baseOrientation=ObjIniOrientation,
                               useMaximalCoordinates=True)
            
            print(f"Set ID to {self.ID}")  
        else :
            try :
                if onObject is not None:
                    if onObject.name == "table"  :
                        self.initPos[2] += onObject.getHeight()
                p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
                self.ID=p.loadURDF(name, self.initPos, self.initOrientation)
                print(f'Load {name}')
            except :
                print(f"{name} not found") 
                pass

    def setInitPos(self,pos):
        self.initPos=pos

    def getInitPos(self):
        return self.initPos

    def setID(self,ID):
        self.ID=ID

    def getPos(self):
        bpao=p.getBasePositionAndOrientation(self.ID)
        #print(bpao)
        return bpao[0]
    def setHeight(self,height):
        self.height=height

    def getHeight(self):
        return self.height
    def setWidth(self,width):
        self.width=width

    def getWidth(self):
        return self.width
    def setInitOrientation(self, quat):
        self.initOrientation=quat
    def getInitOrientation(self):
        return self.initOrientation


