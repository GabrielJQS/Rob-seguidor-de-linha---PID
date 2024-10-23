#include <PID_v1.h>

// Definição dos pinos para os motores (Ponte H)
const int motorA_horario = 18;    // Pino de controle para sentido horário do Motor A
const int motorA_antihorario = 5;  // Pino de controle para sentido anti-horário do Motor A
const int motorB_horario = 21;   // Pino de controle para sentido horário do Motor B
const int motorB_antihorario = 19;  // Pino de controle para sentido anti-horário do Motor B

// Definição dos pinos dos sensores IR
const int sensorEsquerdo = 27;  // Sensor IR esquerdo
const int sensorCentral = 26;   // Sensor IR central
const int sensorDireito = 25;   // Sensor IR direito

// Variáveis de leitura dos sensores IR
int leituraEsquerda, leituraCentro, leituraDireita;

// Definição das velocidades específicas para os motores
const int velocidadeMotorA = 255;  // Velocidade máxima para Motor A
const int velocidadeMotorB = 255;   // Velocidade reduzida para Motor B

// Variáveis do PID
double Setpoint, Input, Output;  // Setpoint (valor alvo), Input (entrada do erro), Output (saída PID)

// Ajuste dos parâmetros do PID (proporcional, integral, derivativo)Os 
double Kp = 1.0, Ki = 0, Kd = 0;  // Esses valores devem ser ajustados conforme necessário

// Criando o objeto PID
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  // Configura os pinos como saída para os motores
  pinMode(motorA_horario, OUTPUT);
  pinMode(motorA_antihorario, OUTPUT);
  pinMode(motorB_horario, OUTPUT);
  pinMode(motorB_antihorario, OUTPUT);

  // Configura os pinos dos sensores IR como entrada
  pinMode(sensorEsquerdo, INPUT);
  pinMode(sensorCentral, INPUT);
  pinMode(sensorDireito, INPUT);

  // Inicializa os motores desligados
  pararMotores();

  // Inicializa o PID
  Setpoint = 0;  // Queremos que o erro seja 0 (robô no centro da linha)
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);  // Limitar a saída do PID para controlar a velocidade
}

// Função principal (loop)
void loop() {
  // Leitura dos sensores IR (1 = preto, 0 = branco)
  leituraEsquerda = digitalRead(sensorEsquerdo);
  leituraCentro = digitalRead(sensorCentral);
  leituraDireita = digitalRead(sensorDireito);

  // Calcula o erro baseado nas leituras dos sensores
  // Se o sensor esquerdo estiver sobre a linha, damos um peso maior ao erro
  Input = (leituraEsquerda * (-1)) + (leituraDireita * 1);

  // Atualiza o PID
  myPID.Compute();

  // Controle dos motores baseado na saída do PID
  if (leituraCentro == 1) {
    // Se o sensor central detectar preto, o robô avança em linha reta
    moverParaFrente(velocidadeMotorA - Output, velocidadeMotorB + Output);
  } else if (leituraEsquerda == 1 || leituraDireita == 1) {
    // Se desviar da linha, o PID ajusta a velocidade de cada motor
    moverParaFrente(velocidadeMotorA - Output, velocidadeMotorB + Output);
  } else {
    // Se nenhum sensor detectar a linha, parar os motores
    pararMotores();
  }
}

// Funções de controle dos motores

void moverParaFrente(int velocidadeA, int velocidadeB) {
  // Controla o motor A
  if (velocidadeA > 0) {
    digitalWrite(motorA_horario, HIGH);
    digitalWrite(motorA_antihorario, LOW);
  } else {
    digitalWrite(motorA_horario, LOW);
    digitalWrite(motorA_antihorario, HIGH);
    velocidadeA = -velocidadeA;  // Inverte o sentido
  }

  // Controla o motor B
  if (velocidadeB > 0) {
    digitalWrite(motorB_horario, HIGH);
    digitalWrite(motorB_antihorario, LOW);
  } else {
    digitalWrite(motorB_horario, LOW);
    digitalWrite(motorB_antihorario, HIGH);
    velocidadeB = -velocidadeB;  // Inverte o sentido
  }

  // Aplica as velocidades aos motores (0-255)
  analogWrite(motorA_horario, velocidadeA);
  analogWrite(motorB_horario, velocidadeB);
}

void pararMotores() {
  // Parar ambos os motores
  digitalWrite(motorA_horario, LOW);
  digitalWrite(motorA_antihorario, LOW);
  digitalWrite(motorB_horario, LOW);
  digitalWrite(motorB_antihorario, LOW);
}
