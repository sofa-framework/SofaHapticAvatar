# Required import for python
import Sofa
import Sofa.Simulation
import SofaRuntime
import HapticAvatar
import Obstacles


def main():
    import SofaRuntime
    import Sofa.Gui

    root = Sofa.Core.Node("root")
    createScene(root)
    Sofa.Simulation.init(root)

    Sofa.Gui.GUIManager.Init("myscene", "qglviewer")
    Sofa.Gui.GUIManager.createGUI(root, __file__)
    Sofa.Gui.GUIManager.SetDimension(1080, 1080)
    Sofa.Gui.GUIManager.MainLoop(root)
    Sofa.Gui.GUIManager.closeGUI()

_method = "articulated1d"
#_method = "articulatedRigid3d"


def createScene(root):
    root.gravity=[0, -9.81, 0]
    root.dt=0.01

    root.addObject('DefaultVisualManagerLoop')
    root.addObject('DefaultAnimationLoop')

    root.addObject('VisualStyle', displayFlags="hideCollisionModels showVisualModels hideForceFields showBehaviorModels")
    root.addObject('RequiredPlugin', name="mecha", pluginName="SofaBoundaryCondition SofaConstraint SofaDeformable SofaImplicitOdeSolver SofaMeshCollision SofaMiscFem SofaSparseSolver") 
    root.addObject('RequiredPlugin', name="utils", pluginName="SofaEngine SofaLoader SofaOpenglVisual SofaTopologyMapping SofaRigid SofaGeneralRigid") 
    root.addObject('RequiredPlugin', name="haptic", pluginName="SofaHaptics SofaHapticAvatar") 
    
         
    root.addObject('DefaultPipeline', name="CollisionPipeline")
    root.addObject('BruteForceBroadPhase')
    root.addObject('BVHNarrowPhase')
    root.addObject('DefaultContactManager', name="CollisionResponse", response="FrictionContact")
    root.addObject('LocalMinDistance', name="proximity", alarmDistance="2", contactDistance="0.1", angleCone="0.1")
    root.addObject('FreeMotionAnimationLoop')
    root.addObject('LCPConstraintSolver', tolerance="0.001", maxIt="10000") #checker mu
 
    if (_method == "articulated1d"):
        # create device portal and driver
        HapticAvatar.createDevice(root)
        
        # create articulated grasper
        toolPosition=[0, 0, 200] # origin at the portal rotation
        toolRotation=[0, -90, -90]
        HapticAvatar.createArticulatedGrasper(root, toolPosition, toolRotation)
    elif (_method == "articulatedRigid3d"):
        # create device portal and driver
        HapticAvatar.createRigidDevice(root)
        
        HapticAvatar.createPortal(root, "Portal_4", 4)
        
        toolPosition=[8.8, 194.23, 0] # origin at the portal rotation
        toolRotation=[0, 0, 0]
        HapticAvatar.createRigidGrasper(root, toolPosition, toolRotation)
    
    # create deformable liver
    Obstacles.createLiver(root)
    
    return root


# Function used only if this script is called from a python environment
if __name__ == '__main__':
    main()
