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
    dCube.addObject('SparseLDLSolver', name="linear_solver", template="CompressedRowSparseMatrixMat3x3d")
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



def createRigidCactus(root, nodeName, translation, size3d):
    cactus = root.addChild(nodeName)
    
    cactus.addObject('MeshObjLoader', name="loader", filename="./mesh/cactus.obj", translation=translation, scale3d=size3d)
    cactus.addObject('MechanicalObject', name="CollisModel", position="@loader.position")
    cactus.addObject('MeshTopology', src='@./loader')
    
    cactus.addObject('TriangleCollisionModel', simulated=False, moving=False, bothSide=False, group="1")
    cactus.addObject('LineCollisionModel', simulated=False, moving=False, group="1")
    cactus.addObject('PointCollisionModel', simulated=False, moving=False, group="1")
    
    # map visual model on surface
    visu = cactus.addChild('VisuSurface')
    visu.addObject('OglModel', name="VisualModel")
    visu.addObject('IdentityMapping', name="VisualMapping", input="@..", output="@VisualModel")
    


def createDeformableCactus(root, nodeName, translation, size3d, grid3d, fixBox):
    cactus = root.addChild(nodeName)
    cactus.addObject('MeshObjLoader', name="loader", filename="./mesh/cactus.obj", translation=translation, scale3d=size3d)
    cactus.addObject('SparseGridRamificationTopology', name="grid", n=grid3d, src='@./loader', nbVirtualFinerLevels=3, finestConnectivity=0)
        
    # create 3D tetrahedral model
    tCactus = cactus.addChild("Mecha")
    tCactus.addObject('EulerImplicitSolver', name="cg_odesolver")
    tCactus.addObject('SparseLDLSolver', name="linear_solver", template="CompressedRowSparseMatrixMat3x3d")
    #tCactus.addObject('CGLinearSolver', name="linear_solver", iterations=30, tolerance="1e-9", threshold="1e-9", template="CompressedRowSparseMatrixMat3x3d")
    
    # create 3D tetrahedral model
    tCactus.addObject('TetrahedronSetTopologyContainer', name="Tetra_topo")
    tCactus.addObject('TetrahedronSetTopologyModifier', name="Tetra_Modifier")
    tCactus.addObject('TetrahedronSetGeometryAlgorithms', name="Tetra_GeomAlgo", template="Vec3d")
    tCactus.addObject('Hexa2TetraTopologicalMapping', name="Hexa2Tetra", input="@../grid", output="@Tetra_topo")

    tCactus.addObject('MechanicalObject', name="Volume", position="@../grid.position")
    
    # fix positions
    tCactus.addObject('BoxROI', name="boxRoi1", box=fixBox, drawBoxes=True)
    tCactus.addObject('FixedConstraint', name="Fix", indices="@boxRoi1.indices")
    
    # mechanical parameters
    tCactus.addObject('UniformMass', name="Mass", totalMass="1.0")
    tCactus.addObject('FastTetrahedralCorotationalForceField', template="Vec3d", name="FEM", method="large", poissonRatio=0.3, youngModulus=3000)
    tCactus.addObject('LinearSolverConstraintCorrection')
    #cactus.addObject('UncoupledConstraintCorrection')

    
    # Add surface mesh with barycentric mapping for collision
    surfCactus = tCactus.addChild("Surface")
    surfCactus.addObject('MeshTopology', src='@../../loader')
    surfCactus.addObject('MechanicalObject', name="CollisModel", src="@../../loader")
    
    surfCactus.addObject('TriangleCollisionModel', simulated=True, moving=True, bothSide=False, group="1")
    surfCactus.addObject('LineCollisionModel', simulated=True, moving=True, group="1")
    surfCactus.addObject('PointCollisionModel', simulated=True, moving=True, group="1")
    surfCactus.addObject('BarycentricMapping', name="baryMapping", input="@..", output="@.")
    
    # map visual model on surface
    visu = surfCactus.addChild('VisuSurface')
    visu.addObject('OglModel', name="VisualModel", color="green")
    visu.addObject('IdentityMapping', name="VisualMapping", input="@..", output="@VisualModel")



def createLiver(root):
    liver = root.addChild('Liver')
    liver.addObject('EulerImplicitSolver', name="cg_odesolver")
    liver.addObject('SparseLDLSolver', name="linear_solver")
    
    # create 3D tetrahedral model
    liver.addObject('MeshGmshLoader', name="meshLoader", filename="./mesh/liver2.msh", scale3d="100 100 100", rotation="-90 -90 -70", translation="200 50 50")
    liver.addObject('MechanicalObject', name="dofs", src="@meshLoader")
    
    liver.addObject('TetrahedronSetTopologyContainer', name="tetraCon", src="@meshLoader")
    liver.addObject('TetrahedronSetTopologyModifier', name="tetraMod")
    liver.addObject('TetrahedronSetGeometryAlgorithms', template="Vec3d", name="tetraGeo")
    
    liver.addObject('DiagonalMass', name="Mass", massDensity="0.001")# verifier dimension
    liver.addObject('FastTetrahedralCorotationalForceField', template="Vec3d", name="FEM", method="large", poissonRatio=0.3, youngModulus=1000)# verifier dimension
    
    liver.addObject('BoxROI', name="ROI1", box="180 40 0  230 80 40", drawBoxes="1")
    liver.addObject('FixedConstraint', name="FixedConstraint", indices="@ROI1.indices")
    liver.addObject('LinearSolverConstraintCorrection')

    # create surface mesh for collision
    surf = liver.addChild('LiverSurface')
    surf.addObject('TriangleSetTopologyContainer', name="triCon")
    surf.addObject('TriangleSetTopologyModifier', name="triMod")
    surf.addObject('TriangleSetGeometryAlgorithms', template="Vec3d", name="triGeo")
    
    surf.addObject('Tetra2TriangleTopologicalMapping', name="topoMapping", input="@../tetraCon", output="@triCon")
    surf.addObject('TriangleCollisionModel', name="TModel")
    surf.addObject('LineCollisionModel', name="LModel")
    surf.addObject('PointCollisionModel', name="PModel")
       
    # map visual model on surface
    visu = surf.addChild('LiverVisu')
    visu.addObject('OglModel', name="VisualModel", color="red")
    visu.addObject('IdentityMapping', name="VisualMapping", input="@../../dofs", output="@VisualModel")
