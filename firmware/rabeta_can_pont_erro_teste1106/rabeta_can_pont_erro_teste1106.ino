#include <SPI.h>
#include <mcp_can.h>


const int SPI_CS_PIN = 10;


MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

void setup()
{
    Serial.begin(115200);

    while (CAN_OK != CAN.begin(CAN_50KBPS))              // init can bus : baudrate = 50k
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println("Init CAN BUS Shield again");
        delay(100);
    }
    Serial.println("CAN BUS Shield init ok!");

  pinMode(3, OUTPUT); // configura pino como saÃ­da
  pinMode(5, OUTPUT); // configura pino como saÃ­da
unsigned char stmp[1] = {123};
}



void loop()
{
    unsigned char len = 0;  //zerando tamanho
    unsigned char buf[1];   //quantidade de palavras enviada
    unsigned char ang_dir;
    
//----------------- Procedimento p/ receber informaçoes da direção -----------------

    if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
    {
        CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

        unsigned char canId = CAN.getCanId();
        Serial.print("ID =");
        Serial.println(canId);

        for(int i = 0; i<len; i++)    // print the data
        {
            Serial.print("DIRECAO =");            
            Serial.println(buf[i]);
            ang_dir = buf[i];
        }
        Serial.println();
    }
    else
    {
         Serial.println("Erro comunicacao"); 
         ang_dir = 128;  
         delay(1);  //teste se tinha erro 
    }
//------------------ Calculo/Analise/Ação do angulo da rabeta ----------------------
//ajustes
  int inzero = 2; // ajuste do intervalo de zero, estabilidade na direção
  int potini = 85; // ajuste de potencia inicial do pwm, ajusta a potencia e velocidade do motor
  int potfin = 90; // ajuste de potencia final do pwm, ajusta a potencia e velocidade do motor

  int sensormotor = analogRead(A0);
  sensormotor = map(sensormotor,0,1024,0,255);
  Serial.print("motor = ");
  Serial.println(sensormotor);
  delay(5);
 int potmotor = sensormotor - ang_dir;
  if (potmotor > inzero){
      Serial.print("positivo = ");
      Serial.println(potmotor);
      if (potmotor > potfin){
          potmotor = potfin; 
          }
      potmotor = map(potmotor,0,potfin,potini,255);
      analogWrite(5, 0);
      analogWrite(3, potmotor);
      
  }
  else if (potmotor < - inzero){
      Serial.print("negativo = ");
      potmotor = potmotor * (-1);
      Serial.println(potmotor); 
      if (potmotor > potfin){
          potmotor = potfin; 
          }
      potmotor = map(potmotor,0,potfin,potini,255);
      analogWrite(5, potmotor);
      analogWrite(3, 0);
 }
  else{
      analogWrite(5, 0);
      analogWrite(3, 0);    
    
  }

}

//--------------------------- Chamada p/ problema ---------------------------------  

void enviar()
{
  unsigned char stmp[1] = {78};
    Serial.println("Enviando informacao");
    // send data:  id = 0x00, standard frame, data len = 8, stmp: data buf
    CAN.sendMsgBuf(0x70,0, 1, stmp);
    delay(1);                       // tempo de envio
  
  }

//END FILE
