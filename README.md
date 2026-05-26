# Triângulo Autoequilibrante com Roda de Reação

Este repositório reúne os códigos desenvolvidos para o projeto **Desenvolvimento de um Triângulo Autoequilibrante como Plataforma Didática de Controle**, elaborado como parte do trabalho de conclusão do curso de Engenharia de Controle e Automação.

O projeto consiste no desenvolvimento de uma planta didática baseada em um triângulo autoequilibrante com roda de reação. A proposta tem como objetivo integrar conceitos de modelagem matemática, controle moderno, controle não linear, sistemas embarcados, instrumentação e validação experimental em uma plataforma física de baixo custo.

A planta utiliza uma estrutura fabricada por impressão 3D, um motor de corrente contínua com encoder integrado, um sensor inercial MPU6050, uma roda de reação e um microcontrolador ESP32. O controle é dividido em duas etapas principais: uma estratégia de elevação automática do tipo *swing-up*, baseada no princípio de controle de energia, e um controlador LQR discreto responsável pela estabilização da planta em torno da posição vertical.

## Vídeos do projeto

O vídeo demonstrativo do funcionamento da planta pode ser acessado no link abaixo:

- **Vídeo:** (https://youtu.be/2tPs7unF_Ww)

## Estrutura do repositório

O repositório contém os principais arquivos utilizados nas etapas de modelagem, simulação, implementação embarcada, coleta de dados e geração dos gráficos experimentais.

```text
.
├── triangulo_autoequilibrante_lqr.mlx
├── swing_up.mlx
├── triangulo_autoequilibrante_lqr.ino
├── triangulo_autoequilibrante_lqr.py
├── graficos.mlx
└── README.md
