# 📊 **Comunicação Serial UART e I2C com Display na Pi Pico W**

Projeto da área de sistemas embarcados para a Residência TIC 37 - Embarcatech, que simula o monitoramento de níveis de água utilizando joystick analógico, exibição de dados em display OLED SSD1306, sinalização com LEDs e alerta via buzzer. Desenvolvido com a placa BitDogLab e sistema operacional FreeRTOS.

---

## 🔎 **Objetivos**

Monitorar o nível de um "rio" simulado com joystick, exibindo os dados em tempo real no display OLED SSD1306, acionando LEDs com intensidade proporcional, e sinalizando situação crítica com matriz de LEDs RGB e buzzer PWM.

---

## 🎥 **Demonstração**

[Ver Vídeo do Projeto](https://drive.google.com/file/d/1S76Psc-qq6pxxSoDH1nHHW-2aFLstBe6/view?usp=drive_link)

---

## 🛠️ **Tecnologias Utilizadas**

- **Linguagem de Programação:** C / CMake
- **Placas Microcontroladoras:**
  - BitDogLab
  - Pico W

---

## 📖 **Como Utilizar**

O joystick simula o nível da água, gerando valores lidos por ADC.  
O display SSD1306 exibe os valores em tempo real.  
A intensidade dos LEDs verde e azul muda conforme os níveis X e Y.

Quando o valor do eixo Y ultrapassa 4000:

- A matriz WS2812 acende em vermelho (alerta visual)
- O buzzer emite um som agudo intermitente (alerta sonoro)

O botão físico também ativa o modo de alerta simulando uma emergência.

---

## 📊 **Funcionalidades Demonstradas**

- Leitura analógica via ADC (joystick)
- Manipulação de display OLED SSD1306 via I2C
- Controle de LEDs com PWM
- Alerta sonoro com buzzer PWM
- Controle de matriz WS2812 com PIO
- Criação de tarefas simultâneas com FreeRTOS
- Comunicação entre tarefas com filas (`xQueue`)
