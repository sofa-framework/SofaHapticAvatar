
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
    
    
    dShaftVisu = models.addChild('Grasper_jaws_up_visu')
    dShaftVisu.addObject('MeshObjLoader', name="loader", filename="./mesh/Haptic_grasper_jaws_up.obj")
    dShaftVisu.addObject('OglModel', name="Visual", src="@loader")
    dShaftVisu.addObject('RigidMapping', input="@..", output="@.", index="5")
    
    dShaftCol = models.addChild('Grasper_jaws_up_collision')
    dShaftCol.addObject('MeshObjLoader', name="loader", filename="./mesh/Haptic_grasper_jaws_up_collision.obj")
    dShaftCol.addObject('MechanicalObject', template="Vec3d", position="@loader.position")    
    dShaftCol.addObject('MeshTopology', src="@loader")
    dShaftCol.addObject('TriangleCollisionModel', group="0")
    dShaftCol.addObject('LineCollisionModel', group="0")
    dShaftCol.addObject('PointCollisionModel', group="0")
    dShaftCol.addObject('RigidMapping', input="@..", output="@.", index="5")
    
    
    dShaftVisu = models.addChild('Grasper_jaws_down_visu')
    dShaftVisu.addObject('MeshObjLoader', name="loader", filename="./mesh/Haptic_grasper_jaws_down.obj")
    dShaftVisu.addObject('OglModel', name="Visual", src="@loader")
    dShaftVisu.addObject('RigidMapping', input="@..", output="@.", index="4")
    
    dShaftCol = models.addChild('Grasper_jaws_down_collision')
    dShaftCol.addObject('MeshObjLoader', name="loader", filename="./mesh/Haptic_grasper_jaws_down_collision.obj")
    dShaftCol.addObject('MechanicalObject', template="Vec3d", position="@loader.position")    
    dShaftCol.addObject('MeshTopology', src="@loader")
    dShaftCol.addObject('TriangleCollisionModel', group="0")
    dShaftCol.addObject('LineCollisionModel', group="0")
    dShaftCol.addObject('PointCollisionModel', group="0")
    dShaftCol.addObject('RigidMapping', input="@..", output="@.", index="4")
    
    
    articulations.addObject('ArticulatedHierarchyContainer')
    artCenters = articulations.addChild('articulationCenters')
    
    artCt1 = artCenters.addChild('articulationCenter1')
    artCt1.addObject('ArticulationCenter', parentIndex="0", childIndex="1", posOnParent="0 0 0", posOnChild="0 0 0", articulationProcess="0")
    arts1 = artCt1.addChild('articulations1')
    arts1.addObject('Articulation', translation="0", rotation="1", rotationAxis="0 1 0", articulationIndex="0")
    
    artCt2 = artCenters.addChild('articulationCenter2')
    artCt2.addObject('ArticulationCenter', parentIndex="1", childIndex="2", posOnParent="0 0 0", posOnChild="0 0 0", articulationProcess="0")
    arts2 = artCt2.addChild('articulations2')
    arts2.addObject('Articulation', translation="0", rotation="1", rotationAxis="1 0 0", articulationIndex="1")
    
    artCt3 = artCenters.addChild('articulationCenter3')
    artCt3.addObject('ArticulationCenter', parentIndex="2", childIndex="3", posOnParent="0 9 0", posOnChild="0 9 0", articulationProcess="0")
    arts3 = artCt3.addChild('articulations3')
    arts3.addObject('Articulation', name="art1", translation="0", rotation="1", rotationAxis="0 0 1", articulationIndex="2")
    arts3.addObject('Articulation', name="art2", translation="1", rotation="0", rotationAxis="0 0 1", articulationIndex="3")
    
    artCt4 = artCenters.addChild('articulationCenter4')
    artCt4.addObject('ArticulationCenter', parentIndex="3", childIndex="4", posOnParent="0 0 -20", posOnChild="0 0 -20", articulationProcess="0")
    arts4 = artCt4.addChild('articulations4')
    arts4.addObject('Articulation', translation="0", rotation="1", rotationAxis="0 1 0", articulationIndex="4")
    
    artCt5 = artCenters.addChild('articulationCenter5')
    artCt5.addObject('ArticulationCenter', parentIndex="3", childIndex="5", posOnParent="0 0 -20", posOnChild="0 0 -20", articulationProcess="0")
    arts5 = artCt5.addChild('articulations5')
    arts5.addObject('Articulation', translation="0", rotation="1", rotationAxis="0 1 0", articulationIndex="5")
