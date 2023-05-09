# Grafos e trajetórias no Gazebo <!-- omit in toc -->

## Conteúdo <!-- omit in toc -->
- [Controlador simples utilizando Gazebo](#implementando-um-subscriber-em-ros)
- [Implementação de grafos em Python](#implementando-um-controlador-simples-em-ros)
- [Controle de trajetória utilizando grafos e Gazebo](#implementando-uma-fila-de-ações-em-ros)
- [Instalação do ambiente de simulação do turtlebot](#instalação-do-ambiente-de-simulação-do-turtlebot)
  - [Problemas comuns com o gazebo](#problemas-comuns-com-o-gazebo)

## Criando pacotes em ROS

O primeiro passo para criar um pacote em ROS é criar uma pasta para servir de workspace. Para fazer isso, rode:
```console
mkdir -p ros2_ws/src
```
Após criada a pasta, entre nela com:
```console
cd ros2_ws
```
Agora, crie toda a estrutura necessária para o seu workspace:
```console
colcon build
```
Note que agora você tem uma estrutura completa com diversas pastas. Uma delas é a pasta `install`, que serve para adicionar o seu workspace e os pacotes nele contidos à instalação do seu ROS2. Para fazer isso, rode:
```console
source install/setup.bash
```
Agora já podemos criar o nosso pacote em Python. Mova-se para a pasta src com:
```console
cd src
```
E rode:
```console
ros2 pkg create --build-type ament_python --dependencies rclpy {nome-do-pacote}
```
Onde o `{nome-do-pacote}` deve ser substituído por, bem, o nome do seu pacote :D

O pacote criado pelo ros2 terá precisamente a mesma estrutura de pastas de um pacote típico Python. Para compilar o pacote, volte para a pasta raíz do workspace e rode:
```console
colcon build --packages-select {nome-do-pacote}
```

Para executar o seu pacote, rode:
```console
ros2 run {nome-do-pacote} {nome-do-entrypoint}
```

Onde `{nome-do-entrypoint}` é o entrypoint do seu pacote Python. Caso não saiba como configurar isso, veja [este](https://packaging.python.org/guides/distributing-packages-using-setuptools/#entry-points) tutorial.





## Implementação de grafos em Python

Vídeo sendo feito...

## Interagindo com o Gazebo

Vídeo sendo feito...

