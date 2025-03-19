# Sensor de Velocidade Arduino
Este projeto utiliza um sensor ultrassônico para medir a velocidade de um objeto em movimento. As medições de distância são usadas para calcular a velocidade do objeto, que é então exibida em um display LCD e impressa no monitor serial.


## Funcionalidade Principal
Este projeto mede a velocidade de um objeto em movimento usando um sensor ultrassônico, exibindo os resultados em um display LCD e no monitor serial do Arduino. O sensor ultrassônico (HC-SR04) calcula a distância até o objeto, e o Arduino utiliza essas medições para calcular a velocidade com base no tempo e na mudança de distância entre as medições sucessivas.

## Iniciação:

Inicializa a comunicação serial para a transmissão de dados para o computador.
Configura o display LCD para exibir mensagens.

## Medições de Distância:
O sensor ultrassônico envia pulsos de som e mede o tempo que leva para os ecos retornarem.
A distância é calculada com base no tempo que os pulsos de som levam para retornar.
Cálculo da Velocidade:

Calcula o tempo decorrido (deltaTime) entre duas medições consecutivas.
Calcula a diferença na distância (deltaDistance) entre duas medições consecutivas.
Calcula a velocidade em metros por segundo (m/s) usando a fórmula:

![image](https://github.com/Nicolejelinski/Edge-sprint1/assets/143125546/9c50f2f5-5247-4193-8414-d4bc4ab96b1a)
Converte a velocidade de metros por segundo (m/s) para quilômetros por hora (km/h) multiplicando por 3.6.

## Exibição dos Resultados:
Exibe a velocidade em m/s no display LCD.
Envia a velocidade em m/s e km/h para o monitor serial do Arduino.

## Limitações
Alcance do Sensor: O sensor ultrassônico tem um alcance limitado a 400 cm. Objetos fora desse alcance não serão detectados.
Precisão: A precisão das medições pode ser afetada por fatores ambientais, como temperatura e interferências acústicas.
Intervalo de Medição: A precisão da velocidade depende da frequência das medições (definida pelo delay(1000)), que pode ser ajustada conforme necessário.

## Componentes necessários
| Componente    | Quantidade    |
| ------------- | ------------- |
|Arduino Uno R3  | 1 |
| Display LCD | 1 |
| Ultrasonic Distance Sensor | 1 |

## Autores
Projeto desenvolvido para a matéria de Edge Computing do Professor Fabio Cabrini, por: 
Nicolle Pellegrino Jelinski;

# Code
```#include <NewPing.h>        // Biblioteca para lidar com o sensor ultrassônico
#include <LiquidCrystal.h>  // Biblioteca para lidar com o display LCD

// Definições de pinos
#define TRIGGER_PIN  12     // Pino TRIGGER do sensor ultrassônico
#define ECHO_PIN     11     // Pino ECHO do sensor ultrassônico
#define MAX_DISTANCE 400    // Distância máxima em cm que o sensor pode medir

// Instanciação do sensor ultrassônico e do display LCD
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// Variáveis para armazenar as últimas medições
float lastDistance = 0;
unsigned long lastTime = 0;
float lastVelocity = 0;

void setup() {
  Serial.begin(9600);  // Inicializa a comunicação serial a 9600 bps
  lcd.begin(16, 2);    // Inicializa o display LCD com 16 colunas e 2 linhas

  // Mensagem de inicialização no display LCD
  lcd.print("Sensor de");
  lcd.setCursor(0, 1);
  lcd.print("Velocidade");
  delay(2000);  // Pausa para leitura da mensagem
  lcd.clear();  // Limpa o display
}

void loop() {
  unsigned long currentTime = millis();    // Armazena o tempo atual em milissegundos
  float currentDistance = sonar.ping_cm(); // Mede a distância em cm usando o sensor ultrassônico

  if (currentDistance > 0 && currentDistance < MAX_DISTANCE) {
    if (lastTime != 0) { // Se não for a primeira medição
      float deltaTime = (currentTime - lastTime) / 1000.0; // Calcula o tempo decorrido em segundos
      float deltaDistance = lastDistance - currentDistance; // Calcula a diferença de distância

      float velocity = (deltaDistance / deltaTime); // Calcula a velocidade em cm/s
      velocity /= 100;  // Converte a velocidade para m/s

      float velocity_kmh = velocity * 3.6; // Converte a velocidade para km/h

      lastVelocity = velocity; // Atualiza a última velocidade

      // Atualiza o display LCD com a velocidade medida
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Velocidade do");
      lcd.setCursor(0, 1);
      lcd.print("carro: ");
      lcd.print(velocity);
      lcd.print(" m/s");

      // Envia a velocidade medida para o monitor serial
      Serial.print("Velocidade do veiculo: ");
      Serial.print(velocity);
      Serial.print(" m/s | ");
      Serial.print(velocity_kmh);
      Serial.println(" km/h");
    }

    lastTime = currentTime;       // Atualiza o tempo da última medição
    lastDistance = currentDistance; // Atualiza a distância da última medição
  }

  delay(1000); // Pausa de 1 segundo antes da próxima medição
}
```
