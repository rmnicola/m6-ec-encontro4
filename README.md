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
(Aqui, caso você receba um aviso/erro de `SetuptoolsDeprecationWarning`, recomendamos recorrer a área de [Erros comuns](#erros-comuns)).

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
ros2 pkg create {nome-do-pacote} --build-type ament_python --dependencies rclpy
```
Onde o `{nome-do-pacote}` deve ser substituído por, bem, o nome do seu pacote :D

O pacote criado pelo ros2 terá precisamente a mesma estrutura de pastas de um pacote típico Python. Porém agora precisaremos criar um novo `entry_point`, para isso precisaremos primeiro entrar em seu `__init__.py`(localizado em `src > {nome-do-pacote} > {nome-do-pacote} > __init__.py`) pelo VSCode(via conexão WSL) e escrever uma função para conseguirmos executar com o `entry_point`, vamos criar um `hello_world`:
```python
def hello_world():
  print("Hello World! :)")
```

Agora, precisaremos ir até o `setup.py` do nosso pacote(localizado em `src > {nome-do-pacote} > setup.py`), para assim adicionarmos o `entry_point` que iremos executar:

```python
[...]
  entry_points={
  'console_scripts': [
    '{nome-do-entrypoint} = {nome-do-pacote}:hello_world',
  ],
},

[...]
```
Após isso, precisaremos voltar a nossa pasta raíz do workspace com o seguinte comando:
```console
cd ..
```

Agora compilaremos o pacote, rodando:
```console
colcon build --packages-select {nome-do-pacote}
```

E então, recarregaremos nossos pacotes do ROS utilizando o comando source novamente:
```console
source install/setup.bash
```


Após isso, poderemos executar o nosso pacote e seu executável rodando:
```console
ros2 run {nome-do-pacote} {nome-do-entrypoint}
```

Onde `{nome-do-entrypoint}` é o entrypoint do seu pacote Python. Caso tenha dificuldades com o entrypoint, recomendamos dar uma olhada [neste](https://packaging.python.org/guides/distributing-packages-using-setuptools/#entry-points) tutorial.

### Erros comuns
Caso na hora de criar seu workspace com o `colcon`, utilizando o comando "`colcon build`", você receba um erro de `SetuptoolsDeprecationWarning`, que muito provavelmente acontecerá se você estiver utilizando o WSL, para resolver é muito simples, digite comando por comando no terminal:
```console
sudo apt install python3-pip
pip3 install setuptools==58.2.0
```
Após isso, você poderá seguir o tutorial.


## Implementação de grafos em Python

Vídeo sendo feito...

## Interagindo com o Gazebo

Vídeo sendo feito...

