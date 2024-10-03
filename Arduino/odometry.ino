#define MAX_MESS_SIZE 13 //Максимальный размер буфера входящего сообщения

//Serial
uint8_t fdb_buff[MAX_MESS_SIZE];    //Буфер отправки ответа
uint8_t recv_buff[MAX_MESS_SIZE];   //Буфер приёма данных
int serial_size = 0;
uint32_t delayMessTime =0;
uint8_t mess_buf[MAX_MESS_SIZE];
uint8_t mess_size = 0;
uint8_t mess_count = 0;             //Счётчик-индикатор прихода нового сообщения по Serial


uint8_t CURR_ERR = 0;                 //Код текущей ошибки

volatile unsigned char input_zero_byte = 0; //0-й байт полученной команды

//Перечисление состояний
typedef enum: uint8_t 
{
  wait_for_task,                  //0: ожидание получения команды
  EMG_Stop,                       //1: аварийная остановка
} RobotMCUState;

//В рабочей программе автомат состояний будет начинаться с ожидания команды от бортового компьютера
volatile RobotMCUState robot_mcu_state = wait_for_task;
volatile RobotMCUState last_robot_mcu_state = wait_for_task;
volatile RobotMCUState next_robot_mcu_state = wait_for_task;


//Ошибки микроконтроллера
const uint8_t MCUErrCodes[] =
{
  0x00, //0:без ошибок
  0x01, //1:CRC check error
  0x02, //2:Error comand received
};

//Подсчёт контрольной суммы
  uint16_t ModbusRTU_CRC(uint8_t *raw_msg_data_byte, uint8_t size_buf)
  {
    //Calc the raw_msg_data_byte CRC code
    uint16_t crc = 0xFFFF;
    String crc_string = "";
    for (uint8_t pos = 0; pos < size_buf; pos++) {
      crc ^= (uint16_t)raw_msg_data_byte[pos];          // XOR byte into least sig. byte of crc
      for (uint8_t i = 8; i != 0; i--) {    // Loop over each bit
        if ((crc & 0x0001) != 0) {      // If the LSB is set
          crc >>= 1;
          crc ^= 0xA001;
        }
        else                            // Else LSB is not set
          crc >>= 1;                    // Just shift right
      }
    }
    return crc;
  }
//Проверка контрольной суммы
  bool CRC_Check(uint8_t byteData[], uint8_t size_buf)
  {
    bool Flag = false;
    uint8_t CRC[2] = {0};
    uint16_t wCrc = 0xFFFF;
    for (uint8_t i = 0; i < size_buf - 2; i++)
      {
        wCrc ^= (uint16_t)byteData[i];
        for (uint8_t j = 0; j < 8; j++)
        {
          if ((wCrc & 0x0001) == 1)
          {
            wCrc >>= 1;
            wCrc ^= 0xA001;
          }
          else
          {
            wCrc >>= 1;
          }
        }
      }
      CRC[1] = (uint8_t)((wCrc & 0xFF00) >> 8);
      CRC[0] = (uint8_t)(wCrc & 0x00FF);
      if ((CRC[1] == byteData[size_buf - 1]) && (CRC[0] == byteData[size_buf - 2]))
      {
        Flag = true;
      }
      return Flag;
  }

//Функция смены состояния в рамках концепта автомата состояний
void ChangeRobotMCUState(RobotMCUState newstate)
{
  if(robot_mcu_state != EMG_Stop)
  {
    last_robot_mcu_state = robot_mcu_state;
    robot_mcu_state = newstate;
  }
}

//Функция анализа входящих сообщений в байтах и формирование ответа
void AnalyzeRecvData(uint8_t *buf, uint8_t *fdbuf, uint8_t buffer_size)
{
  uint16_t temp_crc = 0;
  input_zero_byte = buf[0];
  bool CRCCheckRes = CRC_Check(buf, buffer_size);
  if(CRCCheckRes == true)
  {
    switch(input_zero_byte)
    {
      case 0x01:
        if(buf[1] == 0x01)
        {
          fdbuf[0] = buf[0];
          fdbuf[1] = 0x41;
          fdbuf[2] = 0x72;
          fdbuf[3] = 0x64;
          fdbuf[4] = 0x75;
          fdbuf[5] = 0x69;
          fdbuf[6] = 0x6E;
          fdbuf[7] = 0x6F;
          for(uint8_t i = 8; i < 10; i++) { fdbuf[i] = 0x00;}
        }
        else
        {
          fdbuf[0] = buf[0];
          fdbuf[1] = 0x06;
          fdbuf[2] = 0x02;
          for(uint8_t i = 3; i < 10; i++) { fdbuf[i] = 0x00;}
        }
        temp_crc = ModbusRTU_CRC(fdbuf, 10);
        fdbuf[11] = temp_crc >> 8;
        fdbuf[10] = temp_crc & 0xFF;
        break;
    }
  }
  else
  {
    fdbuf[0] = buf[0];
    fdbuf[1] = 0x06;
    fdbuf[2] = 0x01;
    for(uint8_t i = 3; i < 10; i++) { fdbuf[i] = 0x00;}
    temp_crc = ModbusRTU_CRC(fdbuf, 10);
    fdbuf[11] = temp_crc >> 8;
    fdbuf[10] = temp_crc & 0xFF;
  }

  Serial.write(fdbuf, buffer_size);
  //delay(100);
  Serial.flush();
  memset(fdbuf, 0,  buffer_size);
}


void setup() 
{
  Serial.begin(115200);
  //funcSetTimer2(1000);  
}

void loop() 
{
  if(robot_mcu_state == wait_for_task)
  {
    
  }
/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
/*--------------------------------------Приём сообщений--------------------------------------*/
  serial_size = Serial.available();
	for(uint8_t i = 0; i < serial_size; i++)
	{
		delayMessTime = millis();
		if(mess_size < MAX_MESS_SIZE)
		{
			mess_buf[mess_size] = Serial.read();
			mess_size++;
		}
		//если размер входного буфера больше заданного - лишние байты, мусор
		else { Serial.read();}
	}
  if(mess_size &&((millis() - delayMessTime) > 50))
	{
    AnalyzeRecvData(mess_buf, fdb_buff, mess_size);
		mess_size = 0;
		memset(mess_buf, 0, MAX_MESS_SIZE);
	}
}
//-----------------------------------------------------------------------------------------//


void  funcSetTimer2(uint32_t f){
      if(f>200000){f=200000;}
//    Определяем значение предделителя:
      uint16_t i, Temp;  
      uint8_t  j; 
      f*=2;
      Temp=F_CPU/255;
      if(f>Temp){i=1; j=1;}
      else if(f>(Temp/8)){i=   8; j=2;}
      else if(f>(Temp/32)){i=  32; j=3;}
      else if(f>(Temp/64)){i=  64; j=4;}
      else if(f>(Temp/128)){i= 128; j=5;}
      else if(f>(Temp/256)){i= 256; j=6;}
      else {i=1024; j=7;}
//    Устанавливаем регистры 2 таймера:
      TCCR2A = 0<<COM2A1 | 0<<COM2A0 | 0<<COM2B1 | 0<<COM2B0 | 1<<WGM21  | 0<<WGM20;
      TCCR2B = 0<<FOC2A  | 0<<FOC2B  | 0<<WGM22  | j;
      OCR2A  = (uint8_t)(F_CPU/(i*f))-1;
      TIMSK2 = 0<<OCIE2B | 1<<OCIE2A | 0<<TOIE2;
      SREG   = 1<<7;
}
