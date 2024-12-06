```

sim=require'sim'


-- Parametros do controlador
kp = 0.15  -- Ganho para o controle de velocidade linear (Certifique-se de que esta definido globalmente)
ktheta = 2.0  -- Ganho para o controle de velocidade angular

distanceThreshold = 0.4  -- Distancia minima aceitavel para considerar que o robo chegou ao ponto alvo



function sysCall_init() 

    cone=sim.getObject('::/Cone')
    conepos = sim.getObjectPosition(cone, sim.handle_world)
    print('cone', conepos[1], conepos[2])
    

    motorLeft=sim.getObject("../leftMotor")
    motorRight=sim.getObject("../rightMotor")
    

    ----------------------------------------
    -- Declaracoes de variaveis - C-Space --
    ----------------------------------------
    robot=sim.getObject('..') -- Programaticamente obtem uma referencia ao robo utilizado. 
    -- Como o script esta dentro do proprio robo, pode ser referenciado como '..' (a handle deste objeto)
    
    --Obtem a posicao do robo no inicio da simulacao
    position = sim.getObjectPosition(robot, sim.handle_world)
    -- Armazena as posicoes de X e Y iniciais
    oX = position[1]
    oY = position[2]

    -- Determinacao da malha de pontos, com coordenadas minima e maxima e resolucao de pontos
    step = 0.1
    x_min = -2.5
    x_max = 2.5
    y_min = -2.5
    y_max = 2.5
    
    -- Criacao de matriz de obstaculos
    
    sizeX = x_max - x_min
    sizeX = sizeX / step
    sizeY = y_max - y_min
    sizeY = sizeY / step
    
    matrix = {}
    for i=1, sizeX do
        matrix[i]={}
        for j=1,sizeY do
            matrix[i][j] = 0
        end
    end
    
    
    
    -- Armazena obstaculos em lista. Lista sera utilizada para calculos de colisao.
    -- '::' retorna o modelo pai de onde o script se localiza. (https://manual.coppeliarobotics.com/en/accessingSceneObjects.htm)
    obstacles =  {sim.getObject('::/Obstaculo1'), 
                        sim.getObject('::/Obstaculo2'), 
                        sim.getObject('::/Obstaculo3'), 
                        sim.getObject('::/Obstaculo4'), 
                        sim.getObject('::/Obstaculo5'), 
                        sim.getObject('::/Obstaculo6'), 
                        sim.getObject('::/Obstaculo7'), 
                        sim.getObject('::/Obstaculo8'), 
                        sim.getObject('::/Obstaculo9'), 
                        sim.getObject('::/Obstaculo10')}
               
               
    -----------------------------------------------
    -- Desenhar Espaco de Configuracao (C-Space) --
    -----------------------------------------------
    -- Desenha espaco livre em azul e o espaco ocupado em vermelho
    local freeSpaceContainer = sim.addDrawingObject(sim.drawing_points, 4, 0, -1, 99999, {0, 0, 1})
    local occupiedSpaceContainer = sim.addDrawingObject(sim.drawing_points, 4, 0, -1, 99999, {1, 0, 0})
    local targetSpaceContainer = sim.addDrawingObject(sim.drawing_points, 8, 0, -1, 1, {0, 1, 0})

    -- Inicializa o X e Y utilizados pelo laco nos minimos estabelecidos    
    local x = x_min
    local y = y_min
    
    -- Inicializa enderecos a desenhar na matriz do espaco de configuracao
    mX = 0
    mY = 0
    
    -- Laco que encontra pontos livres e ocupados.
    -- Percorre todo o espaco designado pelas variaveis declaradas ali em cima e calcula colisao.
    -- Comecando no X_min e Y_min, robo vai fazer um chamado para isColliding e conferira se cada 
    -- obstaculo da lista 'obstacles' esta em colisao com o robo. Se nao estiver, marca aquele espaco como livre.
    -- Se estiver, marca como impassavel.
    -- Isso permite que o robo inteiro possa ser simplificado a um ponto, constituindo um espaco de configuracao.
    while x < x_max do
        while y < y_max do
            -- Move o robo para os pontos X e Y
            sim.setObjectPosition(robot, {x, y, 0.1}, sim.handle_world)
            -- Se nao ha colisao...
            if not isColliding() then
                -- Desenha ponto azul.
                sim.addDrawingObjectItem(freeSpaceContainer, {x, y, 0})               
            else
                -- Se ha colisao, desenha ponto vermelho.
                sim.addDrawingObjectItem(occupiedSpaceContainer, {x, y, 0})
                -- Marca ponto na matriz do C-space como 1 (ocupado)
                matrix[mX][mY] = 1 
                
                matrix[mX-1][mY-1] = 1
                matrix[mX-1][mY] = 1
                matrix[mX-1][mY+1] = 1
                
                matrix[mX][mY-1] = 1
                matrix[mX][mY+1] = 1
                
                matrix[mX+1][mY-1] = 1
                matrix[mX+1][mY] = 1
                matrix[mX+1][mY+1] = 1


                
            end
            y = y + step
            mY = mY + 1
        end
        y = y_min
        x = x + step
        mY = 0
        mX = mX + 1
    end
    
    -- Volta o robo para a origem.
    sim.setObjectPosition(robot, {oX, oY, 0.1}, sim.handle_world)
    
    for i=1, sizeX do
        --print(matrix[i])
    end
    
    
    
    
    ------------------------------
    -- Mover para ponto marcado --
    ------------------------------
    
    -- Certificar que targetPos foi definido corretamente
    if targetPos == nil or #targetPos ~= 2 then
        sim.addStatusbarMessage('Erro: targetPos nao foi inicializado corretamente.')
        targetPos = {0, 0}  -- Valor padrao de seguranca para evitar falhas
    end
    
    -- Inicializacao do objeto de desenho da trajetoria
    trajectoryHandle = sim.addDrawingObject(sim.drawing_linestrip, 2, 0, -1, 9999, {0, 0, 1})
    
    
    
    
    
    
    
    
    
    -- pos Inicio
    posMX = mapValue(position[1], x_min, x_max, 0, sizeX)
    posMY = mapValue(position[2], y_min, y_max, 0, sizeY)
    
    posMXc = tonumber(string.format("%.0f", posMX))
    posMYc = tonumber(string.format("%.0f", posMY))

    
    -- pos Fim
    
    tgt=sim.getObject('::/Alvo')
    tgtPos = sim.getObjectPosition(tgt, sim.handle_world)
    targetPos = {tgtPos[1], tgtPos[2]}  -- Posicao alvo final [x, y]
    
    posMXT = mapValue(tgtPos[1], x_min, x_max, 0, sizeX)
    posMYT = mapValue(tgtPos[2], y_min, y_max, 0, sizeY)
    
    posMXTc = tonumber(string.format("%.0f", posMXT))
    posMYTc = tonumber(string.format("%.0f", posMYT))




     -- Start and goal positions (1-indexed)
    local start = {x = posMXc, y = posMYc}
    local goal = {x = posMXTc, y = posMYTc}
    -- Run A* algorithm
    
    path = aStar(start, goal, matrix)

    tgtmove = 12

    -- Visualize the path
    if path then
        visualizePath(path)
        print('path found', start.x, start.y, goal.x, goal.y)
        --print(path)
        pathsize = table.getn(path)
    else
        print("No path found!")
    end

end



function sysCall_cleanup() 
 
end 

function sysCall_actuation() 

    -- Pegar pos atual do rob?
    position = sim.getObjectPosition(robot, sim.handle_world)
    -- Mapeia posi??o do robo pra posicao da matriz
    posMX = mapValue(position[1], x_min, x_max, 0, sizeX)
    posMY = mapValue(position[2], y_min, y_max, 0, sizeY)
    
    posMXc = tonumber(string.format("%.0f", posMX))
    posMYc = tonumber(string.format("%.0f", posMY))


    -- Definir a posicao alvo
    tgt=sim.getObject('::/Alvo')
    tgtPos = sim.getObjectPosition(tgt, sim.handle_world)

    if tgtmove < pathsize then
        tgtMoveX = mapValue(path[tgtmove].x, 0, sizeX, x_min, x_max)
        tgtMoveY = mapValue(path[tgtmove].y, 0, sizeY, y_min, y_max)
        sim.setObjectPosition(tgt, {tgtMoveX, tgtMoveY, 0.1}, sim.handle_world)
    end
    
    targetPos = {tgtPos[1], tgtPos[2]}  -- Posicao alvo final [x, y]
    


    -- Verificar se targetPos esta corretamente inicializado
    if targetPos == nil or #targetPos ~= 2 then
        sim.addStatusbarMessage('Erro: targetPos nao esta definido corretamente!')
        return
    end

    -- Obter a posicao e orientacao do robo
    pos = sim.getObjectPosition(robot, -1)  -- Posicao [x, y, z]
    theta = sim.getObjectOrientation(robot, -1)[3]  -- Orientacao (angulo theta em relacao ao eixo Z)

    -- Calcular erro de posicao
    errorX = targetPos[1] - pos[1]
    errorY = targetPos[2] - pos[2]
    distance = math.sqrt(errorX^2 + errorY^2)  -- Distancia ate a posicao alvo

    -- Calcular o angulo desejado para alinhar o robo
    desiredTheta = math.atan2(errorY, errorX)
    angleError = desiredTheta - theta

    -- Ajuste do angulo para estar entre -pi e pi
    if angleError > math.pi then
        angleError = angleError - 2 * math.pi
    elseif angleError < -math.pi then
        angleError = angleError + 2 * math.pi
    end
    
    -- Verificar se a distancia ate o ponto alvo e menor que o limite
    if distance < distanceThreshold then
        -- Parar o robo
        sim.setJointTargetVelocity(motorLeft, 0)
        sim.setJointTargetVelocity(motorRight, 0)
        --sim.addStatusbarMessage('Robo chegou ao ponto final e parou.')
        tgtmove = tgtmove + 1
         -- Comando para parar a simulacao
        --sim.stopSimulation()
        return
    end
    
    -- Calcular o angulo desejado para alinhar o robo
    desiredTheta = math.atan2(errorY, errorX)
    angleError = desiredTheta - theta
    
    -- Ajuste do angulo para estar entre -pi e pi
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
    interWheelDist = 0.381  -- Distancia entre as rodas (do robo Pioneer)
    vLeft = (v - (omega * interWheelDist / 2)) / wheelRadius
    vRight = (v + (omega * interWheelDist / 2)) / wheelRadius

    -- Definir as velocidades dos motores
    sim.setJointTargetVelocity(motorLeft, vLeft)
    sim.setJointTargetVelocity(motorRight, vRight)

    -- Adicionar o ponto atual ao objeto de desenho da trajet?ria
    sim.addDrawingObjectItem(trajectoryHandle, pos)
    
end 

function isColliding()
-- Funcao para conferir se o robo esta colidindo com obstaculo qualquer a qualquer momento da simulacao.
    -- Para cada um dos dez itens da lista de obstaculos...
    for i = 1, 10 do
        -- Se estiver colidindo com o obstaculo de numero 'i', iterado no laco for...
        if sim.checkCollision(robot, obstacles[i]) > 0 then
            -- Retorna verdadeiro.
            return true
        end
    end
    -- Se nao colidir com nenhum, retorna falso.
    return false
end

function mapValue(value, in_min, in_max, out_min, out_max)
    -- Map the value from [in_min, in_max] to [out_min, out_max]
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
end













---------------------------------------
-- Aqui o filho chora e a mae nao ve --
---------------------------------------



function heuristic(node, goal)
    return math.abs(node.x - goal.x) + math.abs(node.y - goal.y)
end

function getNeighbors(node, grid)
    local neighbors = {}
    local directions = {{0,1}, {1,0}, {0,-1}, {-1,0}}
    for _, dir in ipairs(directions) do
        local nx, ny = node.x + dir[1], node.y + dir[2]
        if nx > 0 and ny > 0 and nx <= #grid and ny <= #grid[1] and grid[nx][ny] == 0 then
            table.insert(neighbors, {x = nx, y = ny})
        end
    end
    return neighbors
end

function isSameNode(node1, node2)
    return node1.x == node2.x and node1.y == node2.y
end

function aStar(start, goal, grid)
    local openList = {}
    local closedList = {}
    local cameFrom = {}
    local gScore = {}
    local fScore = {}

    for i = 1, #grid do
        gScore[i] = {}
        fScore[i] = {}
        for j = 1, #grid[1] do
            gScore[i][j] = math.huge
            fScore[i][j] = math.huge
        end
    end

    table.insert(openList, start)
    gScore[start.x][start.y] = 0
    fScore[start.x][start.y] = heuristic(start, goal)

    while #openList > 0 do
        local current, currentIndex
        local lowestFScore = math.huge
        for i, node in ipairs(openList) do
            if fScore[node.x][node.y] < lowestFScore then
                lowestFScore = fScore[node.x][node.y]
                current = node
                currentIndex = i
            end
        end

        if isSameNode(current, goal) then
            local path = {}
            while current do
                table.insert(path, 1, current)
                current = cameFrom[current.x .. "," .. current.y]
            end
            return path
        end

        table.remove(openList, currentIndex)
        table.insert(closedList, current)

        for _, neighbor in ipairs(getNeighbors(current, grid)) do
            local inClosedList = false
            for _, closedNode in ipairs(closedList) do
                if isSameNode(neighbor, closedNode) then
                    inClosedList = true
                    break
                end
            end
            if inClosedList then
                goto continue
            end

            local tentativeGScore = gScore[current.x][current.y] + 1

            local inOpenList = false
            for _, openNode in ipairs(openList) do
                if isSameNode(neighbor, openNode) then
                    inOpenList = true
                    break
                end
            end
            if not inOpenList or tentativeGScore < gScore[neighbor.x][neighbor.y] then
                cameFrom[neighbor.x .. "," .. neighbor.y] = current
                gScore[neighbor.x][neighbor.y] = tentativeGScore
                fScore[neighbor.x][neighbor.y] = gScore[neighbor.x][neighbor.y] + heuristic(neighbor, goal)
                if not inOpenList then
                    table.insert(openList, neighbor)
                end
            end
            ::continue::
        end
    end

    return nil
end

function visualizePath(path)

function sleep (a) 
    local sec = tonumber(os.clock() + a); 
    while (os.clock() < sec) do 
    end 
end
    local handle = sim.addDrawingObject(sim.drawing_lines, 2, 0, -1, #path/3, {1,0,0}) -- Red path
    for i = 1, #path - 1 do
        sim.addDrawingObjectItem(handle, {path[i].x, path[i].y, 0, path[i+1].x, path[i+1].y, 0})
    end
end

```
