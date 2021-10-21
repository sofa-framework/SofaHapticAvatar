
def createCustomDevice(root, tool_portal, ibox_portal):
    root.addObject('HapticAvatar_PortalManager', name="Portal_Manager", configFilename="./config/PortalSetup.xml", printLog="1")
    root.addObject('HapticAvatar_GrasperDeviceController', name="HA_Emulator", portName=tool_portal, iboxController="@HAIBox", portalManager="@Portal_Manager")# portName="//./COM3"
    root.addObject('HapticAvatar_IBoxController', name="HAIBox", portName=ibox_portal, printLog="1") # portName="//./COM4"
    

_tool_portal = "//./COM3"
_ibox_portal = "//./COM4"
    
def createDevice(root):
    root.addObject('HapticAvatar_PortalManager', name="Portal_Manager", configFilename="./config/PortalSetup.xml", printLog="1")
    root.addObject('HapticAvatar_GrasperDeviceController', name="HA_Emulator", portName=_tool_portal, iboxController="@HAIBox", portalManager="@Portal_Manager")
    root.addObject('HapticAvatar_IBoxController', name="HAIBox", portName=_ibox_portal, printLog="1")


def createRigidDevice(root):
    root.addObject('HapticAvatar_PortalManager', name="Portal_Manager", configFilename="./config/PortalSetup.xml", printLog="1")
    root.addObject('HapticAvatar_RigidGrasperDeviceController', name="HA_Emulator", portName=_tool_portal, iboxController="@HAIBox", portalManager="@Portal_Manager")
    root.addObject('HapticAvatar_IBoxController', name="HAIBox", portName=_ibox_portal, printLog="1")
    

def createArticulatedGrasper(root, toolPosition, toolRotation):
    # articulated grasper
    #position="0 0 200  0 0 0 1" rotation="0 -90 -90"
    tPosition=[toolPosition[0], toolPosition[1], toolPosition[2], 0, 0, 0, 1]
    
    tool = root.addChild('Tool')
    tool.addObject('MechanicalObject', name="bati", template="Rigid3d", position=tPosition, rotation=toolRotation)
    
    articulations = tool.addChild('Articulation')
    articulations.addObject('EulerImplicitSolver', name="cg_odesolver", rayleighStiffness=0.1, rayleighMass=0.1)
    articulations.addObject('SparseLUSolver', name="solver", tolerance=1e-09)
    
    articulations.addObject('MechanicalObject', name="Articulations", template="Vec1d", position="0 0 0 0 0 0", rest_position="@../../HA_Emulator.toolPosition")
    articulations.addObject('RestShapeSpringsForceField', points="0 1 2 3 4 5", stiffness="100000000 100000000 100000000 100000 100000000 100000000") # a ajuster 
    articulations.addObject('UniformMass', totalMass="1") # verifier dimension
    articulations.addObject('LinearSolverConstraintCorrection')

    articulations.addObject('LCPForceFeedback', template="Vec1d", activate="true", forceCoef="0.0005")
  
    models = articulations.addChild('Models')
    models.addObject('MechanicalObject', name="DOFs", template="Rigid3d", position="0 0 0  0 0 0 1  0 0 0  0 0 0 1  0 0 0  0 0 0 1  0 0 0  0 0 0 1  0 0 0  0 0 0 1   0 0 0  0 0 0 1")
    models.addObject('ArticulatedSystemMapping', input1="@../Articulations", input2="@../../bati", output="@DOFs")
    
    
    dBase = models.addChild('Device_base')
    dBase.addObject('MeshObjLoader', name="loader", filename="./mesh/Haptic_device_base.obj")
    dBase.addObject('OglModel', name="Visual", src="@loader")
    dBase.addObject('RigidMapping', input="@..", output="@.", index="0")       

    dHead = models.addChild('Device_head')
    dHead.addObject('MeshObjLoader', name="loader", filename="./mesh/Haptic_device_head.obj")
    dHead.addObject('OglModel', name="Visual", src="@loader")
    dHead.addObject('RigidMapping', input="@..", output="@.", index="1")       

    dPortal = models.addChild('Device_portal')
    dPortal.addObject('MeshObjLoader', name="loader", filename="./mesh/Haptic_device_portal.obj")
    dPortal.addObject('OglModel', name="Visual", src="@loader")
    dPortal.addObject('RigidMapping', input="@..", output="@.", index="2")
    
    dShaftVisu = models.addChild('Grasper_shaft_visu')
    dShaftVisu.addObject('MeshObjLoader', name="loader", filename="./mesh/Haptic_grasper_shaft.obj")
    dShaftVisu.addObject('OglModel', name="Visual", src="@loader")
    dShaftVisu.addObject('RigidMapping', input="@..", output="@.", index="3")
    
    dShaftCol = models.addChild('Grasper_shaft_collision')
    dShaftCol.addObject('MeshObjLoader', name="loader", filename="./mesh/Haptic_grasper_shaft_collision.obj")
    dShaftCol.addObject('MechanicalObject', template="Vec3d", position="@loader.position")    
    dShaftCol.addObject('MeshTopology', src="@loader")
    dShaftCol.addObject('TriangleCollisionModel', group="0")
    dShaftCol.addObject('LineCollisionModel', group="0")
    dShaftCol.addObject('PointCollisionModel', group="0")
    dShaftCol.addObject('RigidMapping', input="@..", output="@.", index="3")
    
    
    dJawUp = models.addChild('Grasper_jaws_up_visu')
    dJawUp.addObject('MeshObjLoader', name="loader", filename="./mesh/Haptic_grasper_jaws_up.obj")
    dJawUp.addObject('OglModel', name="Visual", src="@loader")
    dJawUp.addObject('RigidMapping', input="@..", output="@.", index="5")
    
    dJawUpCol = models.addChild('Grasper_jaws_up_collision')
    dJawUpCol.addObject('MeshObjLoader', name="loader", filename="./mesh/Haptic_grasper_jaws_up_collision.obj")
    dJawUpCol.addObject('MechanicalObject', template="Vec3d", position="@loader.position")    
    dJawUpCol.addObject('MeshTopology', src="@loader")
    dJawUpCol.addObject('TriangleCollisionModel', group="0")
    dJawUpCol.addObject('LineCollisionModel', group="0")
    dJawUpCol.addObject('PointCollisionModel', group="0")
    dJawUpCol.addObject('RigidMapping', input="@..", output="@.", index="5")
    
    
    dJawDown = models.addChild('Grasper_jaws_down_visu')
    dJawDown.addObject('MeshObjLoader', name="loader", filename="./mesh/Haptic_grasper_jaws_down.obj")
    dJawDown.addObject('OglModel', name="Visual", src="@loader")
    dJawDown.addObject('RigidMapping', input="@..", output="@.", index="4")
    
    dJawDownCol = models.addChild('Grasper_jaws_down_collision')
    dJawDownCol.addObject('MeshObjLoader', name="loader", filename="./mesh/Haptic_grasper_jaws_down_collision.obj")
    dJawDownCol.addObject('MechanicalObject', template="Vec3d", position="@loader.position")    
    dJawDownCol.addObject('MeshTopology', src="@loader")
    dJawDownCol.addObject('TriangleCollisionModel', group="0")
    dJawDownCol.addObject('LineCollisionModel', group="0")
    dJawDownCol.addObject('PointCollisionModel', group="0")
    dJawDownCol.addObject('RigidMapping', input="@..", output="@.", index="4")
    
    
    articulations.addObject('ArticulatedHierarchyContainer')
    artCenters = articulations.addChild('articulationCenters')
    
    artCt1 = artCenters.addChild('articulationCenter1')
    artCt1.addObject('ArticulationCenter', parentIndex="0", childIndex="1", posOnParent="0 0 0", posOnChild="0 0 0", articulationProcess="0")
    arts11 = artCt1.addChild('articulations1')
    arts11.addObject('Articulation', translation="0", rotation="1", rotationAxis="0 1 0", articulationIndex="0")
    
    artCt2 = artCenters.addChild('articulationCenter2')
    artCt2.addObject('ArticulationCenter', parentIndex="1", childIndex="2", posOnParent="0 0 0", posOnChild="0 0 0", articulationProcess="0")
    arts21 = artCt2.addChild('articulations2')
    arts21.addObject('Articulation', translation="0", rotation="1", rotationAxis="1 0 0", articulationIndex="1")
    
    artCt3 = artCenters.addChild('articulationCenter3')
    artCt3.addObject('ArticulationCenter', parentIndex="2", childIndex="3", posOnParent="0 9 0", posOnChild="0 9 0", articulationProcess="0")
    arts31 = artCt3.addChild('articulations3')
    arts31.addObject('Articulation', name="art1", translation="0", rotation="1", rotationAxis="0 0 1", articulationIndex="2")
    arts31.addObject('Articulation', name="art2", translation="1", rotation="0", rotationAxis="0 0 1", articulationIndex="3")
    
    artCt4 = artCenters.addChild('articulationCenter4')
    artCt4.addObject('ArticulationCenter', parentIndex="3", childIndex="4", posOnParent="0 0 -20", posOnChild="0 0 -20", articulationProcess="0")
    arts41 = artCt4.addChild('articulations4')
    arts41.addObject('Articulation', translation="0", rotation="1", rotationAxis="0 1 0", articulationIndex="4")
    
    artCt5 = artCenters.addChild('articulationCenter5')
    artCt5.addObject('ArticulationCenter', parentIndex="3", childIndex="5", posOnParent="0 0 -20", posOnChild="0 0 -20", articulationProcess="0")
    arts51 = artCt5.addChild('articulations5')
    arts51.addObject('Articulation', translation="0", rotation="1", rotationAxis="0 1 0", articulationIndex="5")



def createPortal(root, nodeName, portalId):
    
    portalPosition = "@../Portal_Manager.portalPosition" + str(portalId)
    
    portal = root.addChild(nodeName)
    portal.addObject('MeshObjLoader', name="ProtalMesh", filename="./mesh/Portal.obj", handleSeams=True)
    portal.addObject('MechanicalObject', name="ms", template="Rigid3d", position=portalPosition)
    
    portalVisu = portal.addChild('VisuPortal')
    portalVisu.addObject('OglModel', name="Visual", src="@../ProtalMesh")
    portalVisu.addObject('RigidMapping', input="@..", output="@Visual")


def createRigidGrasper(root, toolPosition, toolRotation):
    root.addObject('EulerImplicitSolver', name="cg_odesolver", rayleighStiffness=0.1, rayleighMass=0.1)
    root.addObject('CGLinearSolver', name="solver", tolerance=1e-09, threshold=1e-09, iterations=25)
    
    instru = root.addChild('Instrument')
    instru.addObject('MechanicalObject', name="GrasperDOFs", template="Rigid3d", position="@../HA_Emulator.toolPosition")
    
    articulations = root.addChild('Articulation')
    articulations.addObject('MechanicalObject', name="Articulations", template="Vec1d", position="0 0 0 0 0 0")
    articulations.addObject('UncoupledConstraintCorrection')
    
    tool = articulations.addChild('Tool')
    tool.addObject('MechanicalObject', name="DOFs", template="Rigid3d", translation=toolPosition, position="0 0 0  0 0 0 1    0 0 0  0 0 0 1    0 0 0  0 0 0 1    0 0 0  0 0 0 1    0 0 0  0 0 0 1    0 0 0  0 0 0 1")
    tool.addObject('UniformMass', totalMass="100")
    tool.addObject('LCPForceFeedback', template="Rigid3d", activate="true", forceCoef="0.0005")
    tool.addObject('RestShapeSpringsForceField', points="0 1 2 3 4 5", external_points='0 1 2 3 4 5', stiffness="100000000 100000000 100000000 100000000 100000000 100000000",
    angularStiffness='1000000000000 1000000000000 1000000000000 1000000000000 1000000000000 1000000000000', external_rest_shape='@../../Instrument/GrasperDOFs')
    
    tool.addObject('ArticulatedSystemMapping', input1="@../Articulations", output="@DOFs")

    # Shaft articulation
    dBase = tool.addChild('ShaftModel')
    dBase.addObject('MeshObjLoader', name="loader", filename="./mesh/grasper_shaft_collision_light.obj")
    dBase.addObject('MeshTopology', name="InstrumentCollisionModel", src="@loader")
    dBase.addObject('MechanicalObject', name="instrumentCollisionState", src="@loader", tags="toolPosition")
    dBase.addObject('SphereCollisionModel', name="shaftSphere", radius=2, group=0, tags="toolCollision shaft")
    dBase.addObject('RigidMapping', input="@..", output="@instrumentCollisionState", index="3")

    dShaftVisu = tool.addChild('Shaft_visu')
    dShaftVisu.addObject('MeshObjLoader', name="meshLoader_1", filename="./mesh/grasper_shaft.obj", handleSeams="1")
    dShaftVisu.addObject('OglModel', name="InstrumentVisualModel", src="@meshLoader_1")
    dShaftVisu.addObject('RigidMapping', input="@..", output="@InstrumentVisualModel", index="3")

    # JawUp articulation
    jawUp = tool.addChild('JawUp_collision')
    jawUp.addObject('MeshObjLoader', name="loader", filename="./mesh/grasper_jaws_up_collision_light.obj")
    jawUp.addObject('MeshTopology', name="InstrumentCollisionModel", src="@loader")
    jawUp.addObject('MechanicalObject', name="instrumentCollisionState", src="@loader", tags="toolPosition")
    jawUp.addObject('SphereCollisionModel', name="jawUpSphere", radius=1, group=0, tags="toolCollision jaws")
    jawUp.addObject('RigidMapping', input="@..", output="@instrumentCollisionState", index="4")

    jawUpVisu = tool.addChild('JawUp_visu')
    jawUpVisu.addObject('MeshObjLoader', name="meshLoader_1", filename="./mesh/grasper_jaws_up.obj", handleSeams="1")
    jawUpVisu.addObject('OglModel', name="InstrumentVisualModel", src="@meshLoader_1")
    jawUpVisu.addObject('RigidMapping', input="@..", output="@InstrumentVisualModel", index="4")


    # JawDown articulation
    jawUp = tool.addChild('JawDown_collision')
    jawUp.addObject('MeshObjLoader', name="loader", filename="./mesh/grasper_jaws_down_collision_light.obj")
    jawUp.addObject('MeshTopology', name="InstrumentCollisionModel", src="@loader")
    jawUp.addObject('MechanicalObject', name="instrumentCollisionState", src="@loader", tags="toolPosition")
    jawUp.addObject('SphereCollisionModel', name="jawDownSphere", radius=1, group=0, tags="toolCollision jaws")
    jawUp.addObject('RigidMapping', input="@..", output="@instrumentCollisionState", index="5")

    jawUpVisu = tool.addChild('JawUp_visu')
    jawUpVisu.addObject('MeshObjLoader', name="meshLoader_1", filename="./mesh/grasper_jaws_down.obj", handleSeams="1")
    jawUpVisu.addObject('OglModel', name="InstrumentVisualModel", src="@meshLoader_1")
    jawUpVisu.addObject('RigidMapping', input="@..", output="@InstrumentVisualModel", index="5")
    
    
    tool.addObject('ArticulatedHierarchyContainer')
    artCenters = tool.addChild('articulationCenters')
        
    artCt1 = artCenters.addChild('articulationCenter1')
    artCt1.addObject('ArticulationCenter', parentIndex="0", childIndex="1", posOnParent="0 0 0", posOnChild="0 0 0", articulationProcess="0")
    arts11 = artCt1.addChild('articulations1')
    arts11.addObject('Articulation', translation="0", rotation="1", rotationAxis="1 0 0", articulationIndex="0")
    arts12 = artCt1.addChild('articulations2')
    arts12.addObject('Articulation', translation="0", rotation="1", rotationAxis="0 1 0", articulationIndex="1")
    arts13 = artCt1.addChild('articulations3')
    arts13.addObject('Articulation', translation="0", rotation="1", rotationAxis="0 0 1", articulationIndex="2")
    
    artCt2 = artCenters.addChild('articulationCenter2')
    artCt2.addObject('ArticulationCenter', parentIndex="1", childIndex="3", posOnParent="0 0 0", posOnChild="0 0 0", articulationProcess="0")
    arts21 = artCt2.addChild('articulations4')
    arts21.addObject('Articulation', translation="1", rotation="0", rotationAxis="0 1 0", articulationIndex="3")
    
    artCt3 = artCenters.addChild('articulationCenter3')
    artCt3.addObject('ArticulationCenter', parentIndex="3", childIndex="4", posOnParent="0 0 0", posOnChild="0 0 0", articulationProcess="0")
    arts31 = artCt3.addChild('articulations5')
    arts31.addObject('Articulation', translation="0", rotation="1", rotationAxis="0 0 1", articulationIndex="4")

    artCt4 = artCenters.addChild('articulationCenter4')
    artCt4.addObject('ArticulationCenter', parentIndex="3", childIndex="5", posOnParent="0 0 0", posOnChild="0 0 0", articulationProcess="0")
    arts41 = artCt4.addChild('articulations6')
    arts41.addObject('Articulation', translation="0", rotation="1", rotationAxis="0 0 1", articulationIndex="5")
    
