# Pegasus Team - Projeto Seiya

Repositório dedicado ao ambiente de desenvolvimento, simulação e automação de voo da **Pegasus Team VANTs Autônomos**.

## 🐎 Sobre a Equipe
Criada no **IFES - Campus São Mateus**, a Pegasus Team é uma equipe de pesquisa e robótica focada no desenvolvimento de Veículos Aéreos Não Tripulados (VANTs) com capacidade de navegação totalmente autônoma. 

Nosso principal campo de prova são as competições de alto nível, com destaque para a **SAE EletroQUAD**. Em nossa participação mais recente na competição, fomos coroados com o prêmio de **Melhor Equipe Estreante**, consolidando nossa base técnica.

📱 **Acompanhe nossos bastidores e conquistas no Instagram:** [@pegasus_ifes](https://www.instagram.com/pegasus_ifes/)

## 💻 Para que serve este repositório?
O repositório **pegasus-seiya** abriga todo o ecossistema de software embarcado e simulação da equipe visando a SAE EletroQUAD 2027. Estruturado como um *workspace* ROS 2, ele contém os pacotes necessários para fazer a ponte entre os algoritmos de decisão e o controle físico do drone.

Os principais focos deste ambiente de desenvolvimento incluem:
* **Simulação de Voo (SITL):** Integração com o Gazebo para testes de física, aerodinâmica e lógica de controle em um ambiente virtual seguro, antes da aplicação no hardware real (Raspberry Pi / Cube Orange).
* **Controle Autônomo:** Pacotes dedicados (como o `seiya_control`) para gerenciar missões guiadas, *takeoff*, navegação, modos de voo e rotinas de pouso de precisão interagindo com o ArduPilot via MAVROS.
* **Visão Computacional & Navegação:** Base arquitetural preparada para processamento de imagens (YOLOv8, OpenCV) e tomada de decisão em tempo real.

## 🚀 Tecnologias Utilizadas
* **ROS 2** (Robot Operating System)
* **ArduPilot / SITL**
* **Gazebo Simulator**
* **MAVROS**
* **Python 3**

## 🛠️ Como Utilizar
*Este projeto foi desenvolvido para rodar preferencialmente em ambientes Linux (Ubuntu) com ROS 2 instalado.*

**1. Clonar o repositório:**
```bash
git clone [https://github.com/joseseuparente/pegasus-seiya.git](https://github.com/joseseuparente/pegasus-seiya.git)
cd pegasus-seiya
```
**2. Compilar Workspace**
```bash
colcon build --symlink-install
source install/setup.bash
```
