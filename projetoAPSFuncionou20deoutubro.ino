//Código oficial 26/10/2024//

//Sem inclusão de Controle PID, sistema oscilando sem parar com pouca fluidez nos movimentos de curva//

// Definições dos pinos dos sensores
const int sensorEsq = 27;         // Sensor esquerdo
const int sensorEsqCentro = 26;   // Sensor esquerdo central
const int sensorCentro = 25;       // Sensor central
const int sensorDirCentro = 32;    // Sensor direito central
const int sensorDir = 35;         // Sensor direito

// Definições dos pinos dos motores
const int motorAH = 5;    // PWM para motor esquerdo
const int motorAA = 18;     // Direção para motor esquerdo
const int motorBH = 19;    // PWM para motor direito
const int motorBA = 21;    // Direção para motor direito

void setup() {
    Serial.begin(9600);

    // Configuração dos pinos dos motores como saída
    pinMode(motorAH, OUTPUT);
    pinMode(motorAA, OUTPUT);
    pinMode(motorBH, OUTPUT);
    pinMode(motorBA, OUTPUT);

    // Configuração dos pinos dos sensores como entrada
    pinMode(sensorEsq, INPUT);
    pinMode(sensorEsqCentro, INPUT);
    pinMode(sensorCentro, INPUT);
    pinMode(sensorDirCentro, INPUT);
    pinMode(sensorDir, INPUT);
}

void loop() {
    // Leitura dos sensores infravermelhos
    int valorEsq = analogRead(sensorEsq);
    int valorEsqCentro = analogRead(sensorEsqCentro);
    int valorCentro = analogRead(sensorCentro);
    int valorDirCentro = analogRead(sensorDirCentro);
    int valorDir = analogRead(sensorDir);

    // Cálculo do erro (diferença entre sensores)
    int erro = 0;
    if (valorEsq > 512) erro -= 2;         // Sensor esquerdo detecta linha
    if (valorEsqCentro > 512) erro -= 1;   // Sensor esquerdo central detecta linha
    if (valorCentro > 512) erro += 0;       // Sensor central não altera
    if (valorDirCentro > 512) erro += 1;    // Sensor direito central detecta linha
    if (valorDir > 512) erro += 2;         // Sensor direito detecta linha

    // Ajustes de velocidade com base no erro
    int velocidadeBase = 150; // Velocidade padrão
    int velocidadeEsq = velocidadeBase + erro * 50; // Ajuste da velocidade do motor esquerdo
    int velocidadeDir = velocidadeBase - erro * 50; // Ajuste da velocidade do motor direito

    // Limita as velocidades para o valor PWM válido
    velocidadeEsq = constrain(velocidadeEsq, 0, 255);
    velocidadeDir = constrain(velocidadeDir, 0, 255);

    // Controle dos motores
    controlaMotor(velocidadeEsq, velocidadeDir);
}

// Função para controlar ambos os motores com direção e PWM
void controlaMotor(int velocidadeEsq, int velocidadeDir) {
    // Controle do motor esquerdo
    if (velocidadeEsq > 0) {
        digitalWrite(motorAA, HIGH); // Sentido horário
        digitalWrite(motorAH, LOW);
    } else if (velocidadeEsq < 0) {
        digitalWrite(motorAA, LOW);  // Sentido anti-horário
        digitalWrite(motorAH, HIGH);
        velocidadeEsq = -velocidadeEsq;   // Inverte a velocidade para PWM positivo
    } else {
        digitalWrite(motorAA, LOW);  // Desliga o motor
        digitalWrite(motorAH, LOW);
    }
    analogWrite(motorAH, velocidadeEsq); // Define o PWM do motor esquerdo

    // Controle do motor direito
    if (velocidadeDir > 0) {
        digitalWrite(motorBA, HIGH); // Sentido horário
        digitalWrite(motorBH, LOW);
    } else if (velocidadeDir < 0) {
        digitalWrite(motorBA, LOW);  // Sentido anti-horário
        digitalWrite(motorBH, HIGH);
        velocidadeDir = -velocidadeDir;    // Inverte a velocidade para PWM positivo
    } else {
        digitalWrite(motorBA, LOW);  // Desliga o motor
        digitalWrite(motorBH, LOW);
    }
    analogWrite(motorBH, velocidadeDir); // Define o PWM do motor direito
}
