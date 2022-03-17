#include <mcp_can.h>
#include <SPI.h>


const int SPI_CS_PIN = 10; //Pino de saida para CS

MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

void setup()
{
    Serial.begin(115200);

    while (CAN_OK != CAN.begin(CAN_50KBPS))              // init can bus : baudrate = 50k
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println(" Init CAN BUS Shield again");
        delay(100);
    }
    Serial.println("CAN BUS Shield init ok!");
}

unsigned char stmp[1] = {127};
unsigned long sensorValues = 0;
int sensorValue = 0;
int Nmedia = 10;



void loop()
{   
      unsigned char len = 0;
      unsigned char buf[1]; 
//---------------------- Procedimento p/ pegar angulo direção -------------------
  for (int i=0; i <= Nmedia - 1; i++){
      sensorValue = analogRead(A2);
      sensorValues = sensorValues + sensorValue;
     // delay(1);                   // delay in between reads for stability
   } 
  sensorValue = sensorValues/Nmedia;
  Serial.println(sensorValue);
  sensorValues = 0;
  sensorValue = map(sensorValue,0,1022,0,255);
  if (sensorValue > 255){
    sensorValue = 255;
  }
  else if (sensorValue < 0){
    sensorValue = 0;
  }
  stmp[0] = sensorValue;

//---------------------- Procedimento p/ enviar informações ------------------- 
 
  Serial.println("Enviando informacao");
    // send data:  id = 0x00, standard frame, data len = 8, stmp: data buf
    CAN.sendMsgBuf(0x60,0, 1, stmp);
    delay(1);                       // tempo de envio

    
//------------------------ Procedimento p/ receber informaçoes -----------------

if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
    {
        CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

        unsigned char canId = CAN.getCanId();

        if(canId == 0x70) //Verificando se é o ID da rabeta
        {
         Serial.println("Problema");

          for(int i = 0; i<len; i++)    // print the data
         {
             Serial.print(buf[i]);

          }
        }
        Serial.println();
    }

    
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
