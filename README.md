# PPGEEC2324 - Sistemas Robóticos Autônomos

**Para aprender sobre o software que sera utiliado:**
https://www.youtube.com/live/1MYtz5fwXZg

**Playlist em português sobre o uso do Coppelia:**
https://www.youtube.com/playlist?list=PL1WrY7PmiW_iwesX41-rS6ddHlvbrpe0O

## [Primeiro projeto](https://github.com/souzala/sra-sistemas-roboticos-autonomos/blob/main/primeiro-projeto.md)

🎯 **Objetivo:** 

Simular um robô móvel com acionamento diferencial e desenvolver um sistema de controle cinemático que permita ao mesmo executar movimentos especificados em espaço livre de obstáculos. 

**Meta 1️⃣** 

Simular no software CoppeliaSim um robô móvel com acionamento diferencial, de maneira a que o mesmo receba os comandos das velocidades de referências para as rodas e retorne a posição e orientação do robô (x,y,) em um referencial global. Além do movimento do robô no espaço de trabalho, mostrar os seguintes gráficos: velocidades das rodas (entradas) em função do tempo; configuração do robô (x,y,), (saídas), em função do tempo; gráfico das posições (x(t),y(t)) seguidas pelo robô no plano xy. Entregar relatório e vídeo mostrando o a simulação e os gráficos solicitados. 

**Meta 2️⃣** 

Implementar controlador cinemático de posição para o robô móvel que leve o robô a uma posição final especificada. Obter resultados de simulação (caminho seguido pelo robô, gráficos das variáveis de entrada e saída em função do tempo, etc.). Entregar relatório e vídeo mostrando os resultados obtidos. 


**Meta 3️⃣** 

Implementar gerador de caminho baseado em polinômios interpoladores de 3º grau para robô móvel. Incluir gerador de caminho na simulação. O simulador deve permitir mostrar o caminho gerado sobre a tela do espaço de trabalho do CoppeliaSim. Implementar controlador de caminhos para seguir o caminho especificado Obter resultados de simulação (caminho gerado, caminho seguido, gráficos das variáveis de entrada e saída em função do tempo, etc.). Entregar relatório e vídeo mostrando o sistema funcionando.


## Segundo projeto

🎯 **Objetivo:**

Desenvolver Planejadores de Caminhos para Robô Móvel que permitam ao mesmo executar movimentos especificados em espaço povoado de obstáculos, sem colidir com os mesmos. 

**Meta 1️⃣** 

Incluir obstáculos retangulares no espaço de trabalho simulado no CoppeliaSim. Considerando que o robô seja de forma quadrada e sua orientação não mude, obter o mapa correspondente em espaço de configuração. Incluir funcionalidade que permita apresentar o caminho do robô. Entregar relatório, e vídeo mostrando as novas funcionalidades implementadas. 

**Meta 2️⃣** 

Implementar planejador de caminhos baseado em campos de potenciais e planejador baseado em grafos para o robô móvel. A sua implementação deve permitir mostrar o caminho gerado no espaço de trabalho do CoppeliaSim. Apresentar gráfico também com a função de potencial calculada. Implementar controlador que permita ao robô seguir o caminho planejado. Entregar relatório e vídeo mostrando o robô seguindo o caminho planejado.

**Meta 3️⃣**

Implementar planejador de caminhos baseado em amostragem para o robô móvel. A sua implementação deve permitir mostrar o caminho gerado no espaço de trabalho do CoppeliaSim. Implementar controlador que permita ao robô seguir o caminho planejado. Entregar relatório e vídeo mostrando o robô seguindo o caminho planejado.
