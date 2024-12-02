# Primeiro Projeto

## Meta 1:

**[Vídeo da simulação](https://www.youtube.com/watch?v=o6U6WxO4qWg)**

**[Relatório](https://github.com/souzala/sra-sistemas-roboticos-autonomos/blob/main/Relat%C3%B3rio_Semin%C3%A1rio_Sistemas_Rob%C3%B3ticos_Aut%C3%B4nomos_2024.pdf)**

### Código
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

## Meta 2

**[Vídeo da simulação](https://www.youtube.com/watch?v=HAVnDSoytfM)**


### Código

```
sim=require'sim'

-- Par?metros do controlador
kp = 1.0  -- Ganho para o controle de velocidade linear (Certifique-se de que est? definido globalmente)
ktheta = 2.0  -- Ganho para o controle de velocidade angular

-- Definir a posi??o alvo (lembre-se de ajustar de acordo com o seu cen?rio)
targetPos = {1.0, 1.0}  -- Posi??o alvo [x, y]

distanceThreshold = 0.05  -- Dist?ncia m?nima aceit?vel para considerar que o rob? chegou ao ponto alvo

function sysCall_init() 
    print("Comeco")
    
    robot=sim.getObject('..')
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
    noDetectionDist=0.5
    maxDetectionDist=0.2
    detect={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
    braitenbergL={-0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}
    braitenbergR={-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}
    v0=2
    
    -- Certificar que targetPos foi definido corretamente
    if targetPos == nil or #targetPos ~= 2 then
        sim.addStatusbarMessage('Erro: targetPos n?o foi inicializado corretamente.')
        targetPos = {0, 0}  -- Valor padr?o de seguran?a para evitar falhas
    end
    
    -- Usando sim.getObject para os gr?ficos (obten??o dos handles)
    graphCaminho = sim.getObject('/Graph_caminho')
    graphVelRodas = sim.getObject('/Graph_vel_rodas')
    graphConfXYT = sim.getObject('/Graph_conf_xyt')
    graphPosXYT = sim.getObject('/Graph_pos_xyt')
    graphEntSai = sim.getObject('/Graph_ent_sai')
    
    -- Adiciona grafico para a posi??o do rob? (X e Y)
    position_graphCaminho = sim.addGraphStream(graphCaminho, 'Posicao Robo', 'X/Y', 0, {1, 0, 0}, 1)
    
    -- Adiciona grafico para as velocidades das rodas
    left_wheel_graph = sim.addGraphStream(graphVelRodas, 'Vel Roda Esq', 'Temp/Vel.', 0, {0, 1, 0}, 1)
    right_wheel_graph = sim.addGraphStream(graphVelRodas, 'Vel Roda Dir', 'Temp./Vel.', 0, {0, 0, 1}, 1)

     -- Adiciona grafico para a trajetoria (X vs Y)
    trajectory_graph = sim.addGraphStream(graphConfXYT, 'Trajetoria', 'X/Y', 0, {1, 1, 0}, 1) -- Trajetoria em amarelo
    
    -- Adiciona grafico para a trajetoria (X vs Y)
    trajectory_graph_pos = sim.addGraphStream(graphPosXYT, 'TrajetoriaPos', 'X/Y', 0, {1, 0, 1}, 1) -- Trajetoria em amarelo
    
    -- Inicializa??o do objeto de desenho da trajet?ria
    --trajectoryHandle = sim.addDrawingObject(sim.drawing_linestrip, {1, 1, 0}, 0, -1, 9999)
    trajectoryHandle = sim.addDrawingObject(sim.drawing_linestrip, 2, 0, -1, 9999, {0, 0, 1})  -- Atualizado
    
    -- Inicializa stream de dados do plot de posi??o no plano
    objectPosX = sim.addGraphStream(graphEntSai, 'object pos x', 'm', 1)
    objectPosY = sim.addGraphStream(graphEntSai, 'object pos y', 'm', 1)
    sim.addGraphCurve(graphEntSai, 'object pos x/y', 2, {objectPosX, objectPosY}, {0, 0}, 'm by m')
end

function sysCall_cleanup() 
 
end 

function sysCall_actuation() 

    -- Verificar se targetPos est? corretamente inicializado
    if targetPos == nil or #targetPos ~= 2 then
        sim.addStatusbarMessage('Erro: targetPos n?o est? definido corretamente!')
        return
    end

    -- Obter a posi??o e orienta??o do rob?
    pos = sim.getObjectPosition(robot, -1)  -- Posi??o [x, y, z]
    theta = sim.getObjectOrientation(robot, -1)[3]  -- Orienta??o (?ngulo theta em rela??o ao eixo Z)

    -- Calcular erro de posi??o
    errorX = targetPos[1] - pos[1]
    errorY = targetPos[2] - pos[2]
    distance = math.sqrt(errorX^2 + errorY^2)  -- Dist?ncia at? a posi??o alvo

    -- Calcular o ?ngulo desejado para alinhar o rob?
    desiredTheta = math.atan2(errorY, errorX)
    angleError = desiredTheta - theta

    -- Ajuste do angulo para estar entre -pi e pi
    if angleError > math.pi then
        angleError = angleError - 2 * math.pi
    elseif angleError < -math.pi then
        angleError = angleError + 2 * math.pi
    end
    
    -- Verificar se a dist?ncia at? o ponto alvo ? menor que o limite
    if distance < distanceThreshold then
        -- Parar o rob?
        sim.setJointTargetVelocity(motorLeft, 0)
        sim.setJointTargetVelocity(motorRight, 0)
        sim.addStatusbarMessage('Rob? chegou ao ponto final e parou.')
         -- Comando para parar a simula??o
        --sim.stopSimulation()
        return
    end
    
    -- Calcular o ?ngulo desejado para alinhar o rob?
    desiredTheta = math.atan2(errorY, errorX)
    angleError = desiredTheta - theta
    
    -- Ajuste do ?ngulo para estar entre -pi e pi
    if angleError > math.pi then
        angleError = angleError - 2 * math.pi
    elseif angleError < -math.pi then
        angleError = angleError + 2 * math.pi
    end

    -- Controlador de velocidade linear e angular
    v = kp * distance  -- Controle proporcional da velocidade linear
    omega = ktheta * angleError  -- Controle proporcional da velocidade angular

    -- Converter as velocidades para as rodas
    wheelRadius = 0.0975  -- Raio das rodas
    interWheelDist = 0.381  -- Dist?ncia entre as rodas (do rob? Pioneer)
    vLeft = (v - (omega * interWheelDist / 2)) / wheelRadius
    vRight = (v + (omega * interWheelDist / 2)) / wheelRadius

    -- Definir as velocidades dos motores
    sim.setJointTargetVelocity(motorLeft, vLeft)
    sim.setJointTargetVelocity(motorRight, vRight)

    -- Adicionar o ponto atual ao objeto de desenho da trajet?ria
    sim.addDrawingObjectItem(trajectoryHandle, pos)
    
    -- Atualizar graphs
    print("iniciar graficos")
    sim.setGraphStreamValue(graphCaminho, position_graphCaminho, pos[1])  -- Caminho seguido pelo rob?
    sim.setGraphStreamValue(graphCaminho, position_graphCaminho, pos[2])

    sim.setGraphStreamValue(graphVelRodas, left_wheel_graph, vLeft)  -- Velocidades das rodas
    sim.setGraphStreamValue(graphVelRodas, right_wheel_graph, vRight)

    sim.setGraphStreamValue(graphConfXYT, trajectory_graph, pos[1])  -- Posi??o x
    sim.setGraphStreamValue(graphConfXYT, trajectory_graph, pos[2])  -- Posi??o y
    sim.setGraphStreamValue(graphConfXYT, trajectory_graph, theta)   -- Orienta??o ?

    sim.setGraphStreamValue(graphPosXYT, trajectory_graph_pos, pos[1])  -- Posi??o x no gr?fico xy
    sim.setGraphStreamValue(graphPosXYT, trajectory_graph_pos, pos[2])  -- Posi??o y no gr?fico xy
    
    
end 


-- Armazenar dados para gr?ficos
velocities = {}
positions = {}

function sysCall_sensing()
    -- Salvar dados de velocidade e posi??o
    table.insert(velocities, {sim.getSimulationTime(), v, omega})
    table.insert(positions, {sim.getSimulationTime(), pos[1], pos[2], theta})
    
    local pos = sim.getObjectPosition(motorLeft)
    sim.setGraphStreamValue(graphEntSai, objectPosX, pos[1])
    sim.setGraphStreamValue(graphEntSai, objectPosY, pos[2])
end
```
