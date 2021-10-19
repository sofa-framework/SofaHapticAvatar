def createRigidCube(root, nodeName, size3d, min3d, max3d):
    grid = root.addChild(nodeName)
    
    grid.addObject('RegularGridTopology', name="grid_1", n=size3d, min=min3d, max=max3d)

    rigCube = grid.addChild("Grid")
    rigCube.addObject('MechanicalObject', name="Volume", src="@../grid_1")
    
    # create 3D tetrahedral model
    rigCube.addObject('TetrahedronSetTopologyContainer', name="Tetra_topo")
    rigCube.addObject('TetrahedronSetTopologyModifier', name="Tetra_Modifier")
    rigCube.addObject('TetrahedronSetGeometryAlgorithms', name="Tetra_GeomAlgo", template="Vec3d")
    rigCube.addObject('Hexa2TetraTopologicalMapping', name="Hexa2Tetra", input="@../grid_1", output="@Tetra_topo")
    
    # create surface mesh for collision
    cSurface = rigCube.addChild("Surface")
    cSurface.addObject('TriangleSetTopologyContainer', name="Tri_topo")
    cSurface.addObject('TriangleSetTopologyModifier', name="Tri_Modifier")
    cSurface.addObject('TriangleSetGeometryAlgorithms', name="Tri_GeomAlgo", template="Vec3d")
    cSurface.addObject('Tetra2TriangleTopologicalMapping', name="Tetra2Tri", input="@../Tetra_topo", output="@Tri_topo")
    
    cSurface.addObject('TriangleCollisionModel', simulated=False, moving=False, bothSide=False, group="1")
    cSurface.addObject('LineCollisionModel', simulated=False, moving=False, group="1")
    cSurface.addObject('PointCollisionModel', simulated=False, moving=False, group="1")
    
    # map visual model on surface
    visu = cSurface.addChild('VisuSurface')
    visu.addObject('OglModel', name="VisualModel")
    visu.addObject('IdentityMapping', name="VisualMapping", input="@../../Volume", output="@VisualModel")



def createDeformableCube(root, nodeName, size3d, min3d, max3d, fixBox):
    grid = root.addChild(nodeName)
    
    grid.addObject('RegularGridTopology', name="grid_1", n=size3d, min=min3d, max=max3d)
    
    dCube = grid.addChild("Grid")
    dCube.addObject('EulerImplicitSolver', name="cg_odesolver")
    dCube.addObject('SparseLDLSolver', name="linear_solver")
    dCube.addObject('MechanicalObject', name="Volume", src="@../grid_1")
    
    # fix positions
    dCube.addObject('BoxROI', name="boxRoi1", box=fixBox, drawBoxes=True)
    dCube.addObject('FixedConstraint', name="Fix", indices="@boxRoi1.indices")
    
    # create 3D tetrahedral model
    dCube.addObject('TetrahedronSetTopologyContainer', name="Tetra_topo")
    dCube.addObject('TetrahedronSetTopologyModifier', name="Tetra_Modifier")
    dCube.addObject('TetrahedronSetGeometryAlgorithms', name="Tetra_GeomAlgo", template="Vec3d")
    dCube.addObject('Hexa2TetraTopologicalMapping', name="Hexa2Tetra", input="@../grid_1", output="@Tetra_topo")
    
    # mechanical parameters
    dCube.addObject('UniformMass', name="Mass", totalMass="0.1")
    dCube.addObject('TetrahedronFEMForceField', template="Vec3d", name="FEM", method="large", poissonRatio=0.3, youngModulus=1000)# verifier dimension
    dCube.addObject('LinearSolverConstraintCorrection')

    
    # create surface mesh for collision
    cSurface = dCube.addChild("Surface")
    cSurface.addObject('TriangleSetTopologyContainer', name="Tri_topo")
    cSurface.addObject('TriangleSetTopologyModifier', name="Tri_Modifier")
    cSurface.addObject('TriangleSetGeometryAlgorithms', name="Tri_GeomAlgo", template="Vec3d")
    cSurface.addObject('Tetra2TriangleTopologicalMapping', name="Tetra2Tri", input="@../Tetra_topo", output="@Tri_topo")
    
    cSurface.addObject('TriangleCollisionModel', simulated=True, moving=True, bothSide=False, group="1")
    cSurface.addObject('LineCollisionModel', simulated=True, moving=True, group="1")
    cSurface.addObject('PointCollisionModel', simulated=True, moving=True, group="1")
    
    # map visual model on surface
    visu = cSurface.addChild('VisuSurface')
    visu.addObject('OglModel', name="VisualModel", color="green")
    visu.addObject('IdentityMapping', name="VisualMapping", input="@../../Volume", output="@VisualModel")

