# Primeiro Projeto

## Meta 1:

**Vídeo da simulação:** https://www.youtube.com/watch?v=o6U6WxO4qWg
**[Relatório:]** (https://github.com/souzala/sra-sistemas-roboticos-autonomos/blob/main/Relat%C3%B3rio_Semin%C3%A1rio_Sistemas_Rob%C3%B3ticos_Aut%C3%B4nomos_2024.pdf)

```
sim=require'sim'

function sysCall_init() 
    local robot=sim.getObject('..')
    local obstacles=sim.createCollection(0)
    sim.addItemToCollection(obstacles,sim.handle_all,-1,0)
    sim.addItemToCollection(obstacles,sim.handle_tree,robot,1)
    usensors={}
    for i=1,16,1 do
        usensors[i]=sim.getObject("../ultrasonicSensor",{index=i-1})
        sim.setObjectInt32Param(usensors[i],sim.proxintparam_entity_to_detect,obstacles)
    end
    
    motorLeft=sim.getObject("../leftMotor")
    motorRight=sim.getObject("../rightMotor")
    angle=sim.getObjectOrientation(motorLeft)
    i = 0
    
    graph = sim.getObject("/Graph")
    chassis_graph = sim.addGraphStream(graph, 'Left Wheel', 'rad/s', 
                                  0,{1, 0, 0})
    chassis_graph2 = sim.addGraphStream(graph, 'Right Wheel', 'rad/s', 
                                  0,{0, 1, 0})
    chassis_graph3 = sim.addGraphStream(graph, 'Angle', 'rad', 
                                  0,{0, 0, 1})
                                  
    graphB = sim.getObject("/Graph[1]")
    chassis_graphB1 = sim.addGraphStream(graphB, 'X', '', 
                                  0,{1, 0, 0})
    chassis_graphB2 = sim.addGraphStream(graphB, 'Y', '', 
                                  0,{0, 1, 0})
    chassis_graphB3 = sim.addGraphStream(graphB, 'Angle', 'rad', 
                                  0,{0, 0, 1})
    
    sim = require('sim')
    graphXY = sim.getObject('/Graph[2]')
    objectPosX = sim.addGraphStream(graphXY, 'object pos x', 'm', 1)
    objectPosY = sim.addGraphStream(graphXY, 'object pos y', 'm', 1)
    sim.addGraphCurve(graphXY, 'object pos x/y', 2, {objectPosX, objectPosY}, {0, 0}, 'm by m')
    
    
end
-- This is a very simple EXAMPLE navigation program, which avoids obstacles using the Braitenberg algorithm


function sysCall_cleanup() 
 
end 

function sysCall_actuation() 
    if i < 450 then
        vLeft = 2
        vRight = 3
    else
        vLeft = 3
        vRight = 2
    end
    
    if i >= 900 then
        i = 0
    else
        i = i + 1
    end
    
    
    sim.setJointTargetVelocity(motorLeft,vLeft)
    sim.setJointTargetVelocity(motorRight,vRight)
    angle=sim.getObjectOrientation(motorLeft)
    angleZ = angle[2]
    position = sim.getObjectPosition(motorLeft)
    
end 

function sysCall_sensing()
    sim.setGraphStreamValue(graph, chassis_graph, vLeft)
    sim.setGraphStreamValue(graph, chassis_graph2, vRight)
    sim.setGraphStreamValue(graph, chassis_graph3, angleZ)
    
    sim.setGraphStreamValue(graphB, chassis_graphB1, position[1])
    sim.setGraphStreamValue(graphB, chassis_graphB2, position[2])
    sim.setGraphStreamValue(graphB, chassis_graphB3, angleZ)

    sim.setGraphStreamValue(graphXY, objectPosX, position[1])
    sim.setGraphStreamValue(graphXY, objectPosY, position[2])
end
```
