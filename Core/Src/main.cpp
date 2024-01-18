/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
float channel1avg();
float channel1mean();
float channel2avg();
float channel2mean();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//rozmiar bufora ADC
#define ADC_DMABUFFERSIZE 512
//definicje wartości zmiennych systemu gazowego
#define DELTIME 121
#define MULTIPL 3
#define BEWTIME 120
#define GVOTIME 140
#define SPTTIME 150
#include <string>
#include <algorithm>
enum Written {
  None,
  Char
};
uint16_t endcharcounter=0;
template <std::size_t maxSize>
class CircularBuffer {
  uint8_t buffer[maxSize];
  std::size_t head = 0;
  std::size_t tail = 0;
  Written writeFlag = None;
public:
  bool has(uint8_t item) {
    for (std::size_t i = 0; i < size(); ++i) {
      std::size_t inx = (head + i) % maxSize;
      if (buffer[inx] == item) return true;
    }
    return false;
  }
  bool written() {
    return writeFlag;
  }
  std::string read() {
	std::string a;
    switch (writeFlag) {
      case None:
        break;
      case Char:
        a.push_back(buffer[(tail - 1) % maxSize]);
        break;
    }
    writeFlag = None;
    return a;
  }
  void push(uint8_t item) {
    if (tail == (head - 1) % maxSize) return;
    buffer[tail] = item;
    //if(item=='|')endcharcounter++;
    volatile std::size_t temp = (tail + 1) % maxSize;
    tail = temp;
    writeFlag = Char;
  }
  void push(std::string a) {
    for (uint8_t c : a) {
      push(c);
    }
  }
  uint8_t pop() {
	if (empty()) return 0;
    uint8_t item = buffer[head];
    buffer[head] = 0;
    volatile std::size_t temp = (head + 1) % maxSize;
    head = temp;
    return item;
  }
  std::string popUntilExcl(uint8_t guard) {
      std::string ret;
      while (!empty()) {
        uint8_t a = pop();
        ret.push_back(a);
        if (a == guard) {
      	  break;}
      }
      return ret;
    }
  std::string popUntilIncl(uint8_t guard) {
    std::string ret;
    while (!empty()) {
      uint8_t a = pop();
      ret.push_back(a);
      if (a == guard) {
    	  ret.push_back(a);
    	  break;}
    }
    return ret;
  }
  bool empty() {
    return head == tail;
  }
  std::size_t size() {
    if (tail >= head) return tail - head;
    return maxSize - head - tail;
  }
  uint32_t capacity(void) {
      return maxSize;
    }
};
uint8_t itemRx;
CircularBuffer<4096> rx;
uint8_t itemTx;
CircularBuffer<4096> tx;
void USART_send(std::string a) {
  //tx.push(a);
	if(tx.size()+a.length()>tx.capacity())return;
  for(uint8_t charr:a)tx.push(charr);//dodaj do bufora znaki ze stringa
  __disable_irq();//zatrzymaj przerwania w celu nieprzerwanej transmisji
  if ((__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE) != RESET)) {//sprawdzanie czy linia jest obecnie wykorzystywana
    itemTx= tx.pop();//zczytaj znak na końcu
    HAL_UART_Transmit_IT(&huart2, &itemTx, 1);//prześlij pierwszy znak z
  }
  __enable_irq();//wznów przerwania
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (!(huart == &huart2)) return;//jeżeli odpowiedni interfejs
	if (tx.empty()) return;//jeżeli nie koniec buforu
	itemTx = tx.pop();//zczytaj znak na końcu
	HAL_UART_Transmit_IT(&huart2, &itemTx, 1);//prześlij

}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (!(huart == &huart2)) return;
	rx.push(itemRx);
	//if(itemRx=='|')endcharcounter++;
	HAL_UART_Receive_IT(&huart2, &itemRx, 1);
}

//----------------------------//
//POWYŻEJ UART, PONIŻEJ RAMKA
//----------------------------//
void replaceAll(std::string& str, const std::string& from, const std::string& to) {
    size_t start_pos = 0;
    while ((start_pos = str.find(from, start_pos)) != std::string::npos) {
        str.replace(start_pos, from.length(), to);
        start_pos += to.length();
    }
}
void trimStartEndCharacters(std::string& str, char startChar, char endChar) {

    // przycinanie do pierwszej instancji znaku
    size_t startPos = str.find_first_not_of(startChar);
    if (startPos != std::string::npos) {
        str.erase(0, startPos-1);
    }

    // ucinanie do jakiegoś znaku
    size_t endPos = str.find_last_not_of(endChar);
    if (endPos != std::string::npos) {
        str.erase(endPos + 2);
    }
}
/*uint16_t calculatechecksum(char cmd,std::string& data){
	std::string checkingstr=cmd+data;
	uint16_t checksum = 0;
	//dodawanie
	    for (char c : checkingstr) {
	        checksum += static_cast<int>(c);
	    }
	    USART_send("Calculated checksum of "+checkingstr+" we found out to be "+std::to_string(checksum%1000) +"\r\n");
	   //zwracanie modula
	 return checksum % 1000;

}*/
uint16_t calculateCRC16(const std::string& data) {
    const uint16_t polynomial = 0x8005; //  wielomian
    uint16_t crc = 0xFFFF; // wstepna wartość dla crc

    for (char c : data) {//dla każdego znaku w stringu
        crc ^= static_cast<uint16_t>(c) << 8;//XOR obecnego CRC z przesuniętym w lewo o 8 bit obecnym znakiem

        for (int i = 0; i < 8; ++i) {//dla nowego bitu w nowym bajcie po kolei:
            if (crc & 0x8000) {//jeżeli najważniejszy bit po lewej stronie
                crc = (crc << 1) ^ polynomial;//leftshift o 1 i xor z wielomianem
            } else {//jeżeli najważniejszy bit to 0
                crc <<= 1;//bitshift w lewo o 1
            }
        }
    }
    //USART_send("DEBUG:Calculated checksum: "+std::to_string(crc)+"\r\n");

    return crc;
}
std::string reconstructdata(std::string data){
	std::string unescapedData;
	//przejście przez dane przekazane
	  for (size_t i = 0; i < data.length(); ++i) {
	        // sprawdzanie czy jest znak escape
	        if (data[i] == '\\' && i + 1 < data.length()) {
	            // zamiana w wyniku escape
	            switch (data[i + 1]) {
	                case ':':
	                    unescapedData.push_back('^');
	                    break;
	                case ';':
	                    unescapedData.push_back('|');
	                    break;
	                case '@':
	                    unescapedData.push_back('\\');
	                    break;
	                default:
	                    // Handle other cases as needed
	                    break;
	            }
	            // Skip the next character as it was part of the escape sequence
	            ++i;
	        } else {
	            // Regular character, add to the unescaped data
	            unescapedData.push_back(data[i]);
	        }
	    }
	  	  //zwrócenie buforu string tymczasowego
	    return unescapedData;
}
std::string escapeCharacters(const std::string& data) {
    std::string escapedData = data;//zamiana znaków
    replaceAll(escapedData, "\\", "\\@");
    replaceAll(escapedData, "|", "\\;");
    replaceAll(escapedData, "^", "\\:");
    return escapedData;
}
//generowanie ramki odpowiedzi
void respondframe(std::string response){
	std::string escapedresponse=escapeCharacters(response);//zamiana znaków
	char rspchksmbuf[5],rpslenbuf[3];//temp tablice znaków
	uint8_t resplen=static_cast<uint8_t>(escapedresponse.length()-1);//obliczanie długości
	std::snprintf(rpslenbuf,3,"%02d",resplen);//formatowanie długości
	uint16_t respchecksum=calculateCRC16("n"+escapedresponse);//checksuma
	std::snprintf(rspchksmbuf,5,"%04X",respchecksum);//formatowanie checksumy
	std::string outputresponse="^"+(std::string)rpslenbuf+"n"+escapedresponse+(std::string)rspchksmbuf+"|\r\n";//składanie całości
	USART_send(outputresponse);
}
//ZMIENNE CZASOWE SYSTEMU GAZOWO-STRZELAJĄCEGO ORAZ RESET
int shotdelay=DELTIME, timebetween=BEWTIME,valvetime=GVOTIME,sparktime=SPTTIME;
float timemul=MULTIPL,realshotdelay=DELTIME*timemul;
void resettimevals(){
	shotdelay=DELTIME;
	timebetween=BEWTIME;
	valvetime=GVOTIME;
	sparktime=SPTTIME;
	timemul=MULTIPL;
	realshotdelay=DELTIME*MULTIPL;
}
//zmienne średnich i median ADC DMA
float c1avg,c2avg,c1med,c2med;
uint8_t FBpressed=0,LOCKpressed=0;
uint16_t ButtonPresses=0;
void processcmd(char cmd,const std::string& data){
	std::string respstr="0.00000";
	char str1[20]={0},str2[20]={0};

	switch(cmd){
	case 'n':
		//noop do fucking nothing
	break;

	case 'e':
		//echo
		respondframe("ECH:"+data);
	break;

	case 'd':
		//delay
		shotdelay=stof(data);
		respondframe("DEL:"+std::to_string(shotdelay));
		break;

	case 'm':
		//timemul
		timemul=stof(data);
		respondframe("MUL:"+std::to_string(timebetween));
	break;

	case 'b':
		//timebetween
		timebetween=stoi(data);
		respondframe("BEW:"+std::to_string(timebetween));
	break;

	case 'v':
		//valvetime
		valvetime=stoi(data);
		respondframe("GVO:"+std::to_string(valvetime));
	break;

	case 's':
		//sparktime
		sparktime=stoi(data);
		respondframe("SPT:"+std::to_string(sparktime));
	break;

	case 'r':
		resettimevals();
		respondframe("RESET");
		//reset
	break;
	case 'c':

		//custom - output string is respstr
		respstr="DEBUG:CUSTOM STATEMENTS UNDEFINED YET\r\n";
		//respondframe("CST:"+respstr);
		USART_send(respstr);
		respondframe("CMDERR");
		//TODO:ADD COMMANDS
	break;
	case 'a':
		//DMAbufferavearage
		snprintf(str1,10,"%f",c1avg);
		snprintf(str2,10,"%f",c2avg);
		respondframe("AVG:"+(std::string)str1+":"+(std::string)str2);

	break;
	case 'i':
		//DMAbuffermean
		snprintf(str1,10,"%f",c1med);
		snprintf(str2,10,"%f",c2med);
		respondframe("MED:"+(std::string)str1+":"+(std::string)str2);

	break;
	case 'p':
		//Button Press counter
		respondframe("PSS:"+std::to_string(ButtonPresses));
		break;
	case 'f':
		FBpressed=1;
		respondframe("FIRE!");
		break;
	default:
		respondframe("CMDERR");

	}
}

//dekonstrukcja oraz analiza ramki
bool decodePAWNET(const std::string& message) {
	//USART_send("\r\nSTARTING DECODING: "+message+" \r\n");
    //zmienna,znajdywanie końca bo długość to długość danych po rozkodowaniu
    uint8_t endidx=message.find('|');
    if(endidx<=8){return false;}
    std::string fixedData;
    USART_send("\r\n");
    //dlugosc
    std::string lengthStr = message.substr(1, 2);
    for (char ch:lengthStr){//sprawdzanie czy aby na pewno pierwsze 2 znaki po start to cyfry dziesiętne
    			if (!(ch >= '0' && ch <= '9')) {
    	            respondframe("LENERR");
    	            return false;
    	        }
    }
    size_t length = std::stoi(lengthStr)+1;//długość

    //komenda i dane
    char command = message[3];
    std::string data=message.substr(4,endidx-8);

    //rekonstrukcja danych z escapeowania
    fixedData=reconstructdata(data);
    //USART_send("\r\nDEBUG:RECONSTRUCTED DATA: "+fixedData+" \r\n");

    //porównywanie długości danych zrekonstruowanych z długością w ramce
    if(length!=fixedData.length()){
    	respondframe("LENERR");
    	//USART_send("\r\nLENERR\r\n DEBUGGIVEN "+std::to_string(length)+" != GATHERED "+std::to_string(fixedData.length())+"\r\n");
    	return false;
    }
    //checksuma
    std::string checksumString="0x"+message.substr(endidx-4,4);//0x dodane aby stoi z automatu konwertował na hex
    uint16_t packetchecksum=static_cast<uint16_t>(std::stoi(checksumString, 0, 0));
    //USART_send("DEBUG:ChecksumGathered: "+std::to_string(packetchecksum)+"\r\n");

    //sprawdzanie poprawnosci checksumy
      if(calculateCRC16(command+fixedData)==packetchecksum){
    	//sprawdzanie komendy i danych po potwierdzeniu zgodności
    	processcmd(command, fixedData);
    	return true;
    }else{//błąd w obliczeniach checksumy
    	respondframe("CVCERR");
    	return false;
    }
}
//--------//
//Powyżej ramka, poniżej DMA//
//--------//
float POTBufferMin,POTBufferMax,POTBufNormAvg;
uint16_t chn1=0,chn2=0,ADC_DMA_Buffer[ADC_DMABUFFERSIZE*2]={0};
bool convcompl=false;
void ADC_DMA_updateAverages() {
	uint16_t valid_entries=0;
	float tmpc1avg,tmpc2avg;
    for (int i = 0; i < ADC_DMABUFFERSIZE-1 * 2; i += 2) {
    	if (ADC_DMA_Buffer[i] == 0 || ADC_DMA_Buffer[i + 1] == 0) {
            break;//zakończ jeżeli którykolwiek z kanałów ma 0
        }

        tmpc1avg += ADC_DMA_Buffer[i]; //dodawanie do średniej.
        tmpc2avg += ADC_DMA_Buffer[i + 1];
        valid_entries++;

    }

    // nie dziel przez zero lol
    if (valid_entries > 0) {
        tmpc1avg /= valid_entries;
        tmpc2avg /= valid_entries;
        c1avg=tmpc1avg;
        c2avg=tmpc2avg;
    }
}
int compareUint16(const void* a, const void* b) {
    return (*(uint16_t*)a - *(uint16_t*)b);
}
int extractAndSortChannel(uint16_t* channel_buffer, int channel_index) {
    int extracted_entries = 0;//zapamiętaj ile jest prawidłowych wartości

    for (int i = 0; i < ADC_DMABUFFERSIZE; ++i) {
        channel_buffer[i] = ADC_DMA_Buffer[i * 2 + channel_index];
        if (channel_buffer[i] != 0) {
            extracted_entries++;//zlicz ilość prawidłowych nie zerowych wpisów
        }
    }

    // sortowanie
    qsort(channel_buffer, extracted_entries, sizeof(uint16_t), compareUint16);

    return extracted_entries;
}


int calculateMean(uint16_t* sorted_buffer, int entries) {
    if (entries == 0) {
        return 0;  // Avoid division by zero
    }
    return (sorted_buffer[entries / 2 - 1] + sorted_buffer[entries / 2]) / 2;
}
void ADC_DMA_updateMeans() {
		uint16_t channel1_buffer[ADC_DMABUFFERSIZE];
	    uint16_t channel2_buffer[ADC_DMABUFFERSIZE];

	    int entries_channel1 = extractAndSortChannel(channel1_buffer, 0);
	    int entries_channel2 = extractAndSortChannel(channel2_buffer, 1);

	    // Calculate mean for each channel using the middle values
	    c1med = calculateMean(channel1_buffer, entries_channel1);
	    c2med = calculateMean(channel2_buffer, entries_channel2);
}
void normaliseADCOut(){
	POTBufNormAvg=(c1avg-POTBufferMin)/(POTBufferMax-POTBufferMin);
}
bool ReadyToUpdateAvg=false;
void ADC_DMA_UPDATE(){
	ADC_DMA_updateAverages();
	if(convcompl){
		if(c1avg<POTBufferMin)POTBufferMin=c1avg;
		if(c1avg>POTBufferMax)POTBufferMax=c1avg;
		if(POTBufferMax>4100)POTBufferMax=c1avg;
		normaliseADCOut();
		ADC_DMA_updateMeans();
	}
	ReadyToUpdateAvg=false;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	ReadyToUpdateAvg=true;

	convcompl=true;//doszło do konwersji na całej długości bufora
}
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc){
	ReadyToUpdateAvg=true;
	if(!convcompl)POTBufferMin=ADC_DMA_Buffer[0];//inicjalizacja min max
	if(!convcompl)POTBufferMax=ADC_DMA_Buffer[0];
}

float firingcounter=1;
//uint8_t Firingstate=0;//0-gasvalvesopen/1-waitforgasmix/2-sparkplugignite/3-notfiring
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	ButtonPresses++;
	switch(GPIO_Pin){//TODO: EXTI przycisk debounce
	case FB_Pin://przerwanie wykonane przez przycisk fire
		if(HAL_GPIO_ReadPin(FB_GPIO_Port, FB_Pin)==GPIO_PIN_SET){
			//wykryto fire
			FBpressed=1;
			firingcounter=1;
			HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
		}else{
			//wykryto niefire
			FBpressed=0;
		}
		break;
	case B1_Pin://przerwanie wykonane przez przycisk lock
		if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin)==GPIO_PIN_SET){
				LOCKpressed=1;
				//USART_send("\r\nDEBUG: LOCKPRESSED\r\n");
				//hardcloseallvalves();
				}else{

			}
		break;
	default:
		__NOP();
		break;
	}

}
void hardcloseallvalves(){
	HAL_GPIO_WritePin(GV_GPIO_Port, GV_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OV_GPIO_Port, OV_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SP_GPIO_Port, SP_Pin, GPIO_PIN_RESET);
}
enum firingstateenum{GasValvesOpen,WaitForGasMix,SparkPlugIgnite,FiringDelay};
firingstateenum firingstate=GasValvesOpen;
float interval=GVOTIME;
void switchfiringstate(){
	switch(firingstate){
			case GasValvesOpen://otwarte zawory gazu
				HAL_GPIO_WritePin(GV_GPIO_Port, GV_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(OV_GPIO_Port, OV_Pin, GPIO_PIN_SET);
				interval=valvetime;
				firingstate=WaitForGasMix;
				break;
			case WaitForGasMix://zamkniete zawory, czekanie na mieszanke
				HAL_GPIO_WritePin(GV_GPIO_Port, GV_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(OV_GPIO_Port, OV_Pin, GPIO_PIN_RESET);
				interval=timebetween;
				firingstate=SparkPlugIgnite;
				break;
			case SparkPlugIgnite://swieca w ruch, wyliczamy delay
				HAL_GPIO_WritePin(SP_GPIO_Port, SP_Pin, GPIO_PIN_SET);
				interval=sparktime;
				firingstate=FiringDelay;
				break;
			case FiringDelay://wszystko zamkniete, oczekujemy do nastepnego strzalu
				HAL_GPIO_WritePin(SP_GPIO_Port, SP_Pin, GPIO_PIN_RESET);
				normaliseADCOut();
				realshotdelay=shotdelay*timemul*POTBufNormAvg;
				interval=realshotdelay;//powyżej zaktualizowanie i obliczenie czasu między strzałami
				if(HAL_GPIO_ReadPin(FB_GPIO_Port, FB_Pin)!=GPIO_PIN_SET){
					FBpressed=0;
					HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
				}
				firingstate=GasValvesOpen;
				break;
			default:
				//jakim cudem tu jesteś powiedz ty to mi
				hardcloseallvalves();
				interval=1000;
				break;
			}
}
void firingswitch(){
	if(FBpressed){
		if(firingcounter<=0){
			switchfiringstate();
			firingcounter=interval;
		}else{
			firingcounter--;
		}
	}

}
//TODO: CHECK GAS GUN SYSTEM - FiringSwitch Works! Delay Mult Works!
//i belive it worksss
	enum FrameReceiveState{noInput,startReceived,processing,frameReady};
	std::string frameMainBuffer="";
	char readchar=0;
	FrameReceiveState FrameState=noInput;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  hardcloseallvalves();
  HAL_ADC_Start_DMA(&hadc1,(uint32_t *) ADC_DMA_Buffer, ADC_DMABUFFERSIZE*2);
  HAL_UART_Receive_IT(&huart2, &itemRx, 1);
  USART_send("\r\n----------------------\r\n  STM32 Gas Gun INIT\r\n----------------------\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //todo: main loop
  FBpressed=1;
  while (1)
  {
	  if (rx.written()&&__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE) != RESET) {
	  	  USART_send(rx.read());
	  	}

	  if(ReadyToUpdateAvg){
		  ADC_DMA_UPDATE();
	  }
	  if(frameMainBuffer.length()>250){
		  //Jeżeli w buforze znajdzie się za dużo danych to czyścimy
		  frameMainBuffer.clear();
		  FrameState=noInput;
	  }
	  if(!rx.empty()){//jeżeli coś jest w buforze nie przeanalizowane
	  readchar=rx.pop();
	  if(readchar=='^'){//jeżeli znak start
	  		  frameMainBuffer.clear();//czyść bufor
	  		  FrameState=startReceived;
	  		  frameMainBuffer.push_back(readchar);//dodaj znak
	  	  }
	  	  if(FrameState==startReceived){//jeżeli doszedł znak początku
	  		  if(readchar!='|'){//dopóki nie ma znaku końca dodawaj znaki normalnie
	  			  frameMainBuffer.push_back(readchar);
	  		  }else{
	  			  frameMainBuffer.push_back(readchar);
	  			  FrameState=frameReady;//ustaw flagę końca.
	  		  }
	  	  }
	  }

	  if(FrameState==frameReady)
	   {
		    //USART_send("\r\n END CHAR FOUND\r\n");
	        //std::string msg = rx.popUntilIncl('|');
	        //przycinanie ramki
		  	//trimStartEndCharacters(msg, '^', '|');
		      trimStartEndCharacters(frameMainBuffer, '^', '|');
	        //rozkoduj ramkę
	        //if(decodePAWNET(msg)){

		  	  if(decodePAWNET(frameMainBuffer)){
	        	//na wypadek poprawnego rozkodowania
	        	//USART_send("\r\nDEBUG:MESSAGE DECODE SUCCESS\r\n");
	        }else{
	        	//jeżeli gdziekolwiek podczas rozkodowania wystąpi błąd
	        	//USART_send("\r\nDEBUG:MESSAGE DECODE FAIL\r\n");
	        }
	        FrameState=noInput;
	  }

  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|SP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GV_GPIO_Port, GV_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OV_GPIO_Port, OV_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin SP_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|SP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : GV_Pin */
  GPIO_InitStruct.Pin = GV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GV_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FB_Pin */
  GPIO_InitStruct.Pin = FB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(FB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OV_Pin */
  GPIO_InitStruct.Pin = OV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OV_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
